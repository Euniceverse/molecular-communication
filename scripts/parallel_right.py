#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

# -------------------- Params --------------------
def p(name, default):
    return rospy.get_param("~" + name, default)

# -------------------- Serial helpers --------------------
def write_cmd(ser, s: str):
    if not s.endswith("\n"):
        s += "\n"
    ser.write(s.encode("ascii", errors="ignore"))

def read_resistance(ser):
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            return None
        if line.startswith("R "):
            parts = line.split()
            if len(parts) >= 2:
                return float(parts[1])
        return None
    except Exception:
        return None

def wait_for_ready(ser, timeout=5.0):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(timeout):
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            line = ""
        if line == "READY":
            return True
    return False

def flush_input(ser):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

# -------------------- START/STOP control --------------------
_run_event = threading.Event()  # set()=running, clear()=paused/stopped

def control_cb(msg):
    cmd = msg.data.strip().lower()
    if cmd in ("start", "run", "go"):
        _run_event.set()
        rospy.loginfo("CONTROL: START")
    elif cmd in ("stop", "pause"):
        _run_event.clear()
        rospy.logwarn("CONTROL: STOP")
    else:
        rospy.logwarn("CONTROL: unknown '%s' (use 'start' or 'stop')", msg.data)

def wait_until_running(pub_state=None):
    if pub_state:
        pub_state.publish("STATE=WAIT_START")
    rospy.loginfo("Waiting for START... (publish 'start' to control topic)")
    while not rospy.is_shutdown() and not _run_event.is_set():
        rospy.sleep(0.1)

def sleep_while_running(duration_s, pub_state=None):
    """
    Sleep up to duration_s but return False early if STOP happens.
    True  = slept full duration
    False = stopped/paused during sleep
    """
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            if pub_state:
                pub_state.publish("STATE=PAUSED_DURING_SLEEP")
            return False
        if (rospy.Time.now() - t0).to_sec() >= float(duration_s):
            return True
        rospy.sleep(0.02)

def publish_stop(cmd_pub, tw_stop, repeats=8):
    for _ in range(int(repeats)):
        cmd_pub.publish(tw_stop)
        rospy.sleep(0.02)

def emergency_stop(ser, cmd_pub, tw_stop, pub_state=None, stop_stream=False):
    # TX OFF + robot stop (+ optionally STREAM0)
    try:
        write_cmd(ser, "S0")
        if stop_stream:
            write_cmd(ser, "STREAM0")
    except Exception:
        pass
    publish_stop(cmd_pub, tw_stop, repeats=10)
    if pub_state:
        pub_state.publish("STATE=EMERGENCY_STOP")

def handle_pause(ser, cmd_pub, tw_stop, pub_state):
    # When STOP: turn off TX and (optionally) stop streaming, then wait for START
    pub_state.publish("STATE=PAUSED")
    emergency_stop(ser, cmd_pub, tw_stop, pub_state=pub_state, stop_stream=True)
    wait_until_running(pub_state=pub_state)
    # On resume: enable streaming again and flush old data
    pub_state.publish("STATE=RESUMED")
    try:
        write_cmd(ser, "STREAM1")
        rospy.sleep(0.1)
        flush_input(ser)
    except Exception:
        pass

# -------------------- Experiment steps --------------------
def spray_step(ser, spray_time, pub_state=None):
    if pub_state:
        pub_state.publish(f"STATE=SPRAY_ON t={spray_time}")
    write_cmd(ser, "S1")
    ok = sleep_while_running(spray_time, pub_state=pub_state)
    write_cmd(ser, "S0")
    if pub_state:
        pub_state.publish("STATE=SPRAY_OFF")
    return ok

def find_max_for(ser, duration_s, pub_raw=None, sense_rate_hz=20.0):
    t0 = rospy.Time.now()
    max_value = None
    rate = rospy.Rate(float(sense_rate_hz)) 

    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(duration_s):
        if not _run_event.is_set():
            return None, "PAUSED"

        v = read_resistance(ser)
        if v is not None:
            if pub_raw:
                pub_raw.publish(Float32(v))
            if max_value is None or v > max_value:
                max_value = v

        rate.sleep()  
    return max_value, "OK"


def detect_threshold_for(ser, duration_s, threshold, pub_raw=None, sense_rate_hz=20.0):
    rospy.loginfo("start detection for %.1f sec, threshold=%.1f", float(duration_s), float(threshold))    t0 = rospy.Time.now()
    t0 = rospy.Time.now()
    is_threshold_exceeded = False
    rate = rospy.Rate(float(sense_rate_hz)) 

    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(duration_s):
        if not _run_event.is_set():
            return None  

        v = read_resistance(ser)
        if v is not None:
            if pub_raw:
                pub_raw.publish(Float32(v))
            if v > float(threshold):
                is_threshold_exceeded = True
                rospy.loginfo("found: %.1f", float(v))
        rate.sleep()   
    return is_threshold_exceeded


def move_step(cmd_pub, tw_go, tw_stop, move_time, pub_rate_hz, pub_state=None):
    if pub_state:
        pub_state.publish(f"STATE=MOVE t={move_time}")
    rate = rospy.Rate(float(pub_rate_hz))
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(move_time):
        if not _run_event.is_set():
            break
        cmd_pub.publish(tw_go)
        rate.sleep()
    publish_stop(cmd_pub, tw_stop, repeats=10)
    if pub_state:
        pub_state.publish("STATE=MOVE_DONE")

# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_tx")

    # Serial params
    PORT      = p("port", "/dev/ttyUSB0")
    BAUD      = int(p("baud", 9600))
    USE_READY = bool(p("use_ready", True))

    # Control topic
    CONTROL_TOPIC = p("control_topic", "/molecular_demo/control")

    # Debug topics (optional)
    RAW_TOPIC   = p("raw_topic",   "/molecular/tx/raw")
    STATE_TOPIC = p("state_topic", "/molecular/tx/state")

    # Calibration params
    CAL_SPRAY_NUM   = int(p("cal_spray_num", 3))
    CAL_SPRAY_TIME  = float(p("cal_spray_time", 10.0))
    CAL_WAIT_TIME   = float(p("cal_wait_time", 290.0))
    FIND_MAX_NUM    = int(p("find_max_num", 3))
    DETECT_TIME_CAL = float(p("detect_time_cal", 300.0))
    THRESH_FACTOR   = float(p("threshold_factor", 0.5))   # avg_max * factor
    THRESH_FALLBACK = float(p("threshold_fallback", 300.0))


    # Main loop params
    MAIN_SPRAY_NUM    = int(p("main_spray_num", 1))
    MAIN_SPRAY_TIME   = float(p("spray_time", 10.0))
    MAIN_WAIT_TIME    = float(p("wait_time", 290.0))
    DETECT_TIME_MAIN  = float(p("detect_time_main", 300.0))
    COOLDOWN_TIME     = float(p("cooldown_time", 0.0))

    # Motion params
    CMD_TOPIC   = p("cmd_vel_topic", "/cmd_vel")
    SPEED_LIN   = float(p("speed_linear", -0.04))
    SPEED_ANG   = float(p("speed_angular", 0.0))
    MOVE_TIME   = float(p("move_time", 2.0))
    PUB_RATE_HZ = float(p("publish_rate", 20.0))

    # ROS I/O
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=10)
    pub_raw   = rospy.Publisher(RAW_TOPIC, Float32, queue_size=1000)
    pub_state = rospy.Publisher(STATE_TOPIC, String, queue_size=1000)
    cmd_pub   = rospy.Publisher(CMD_TOPIC, Twist, queue_size=10)

    rospy.loginfo("Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)

    # Twist messages
    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG
    tw_stop = Twist()

    # Start paused
    _run_event.clear()
    wait_until_running(pub_state=pub_state)

    # Serial open
    pub_state.publish(f"STATE=SERIAL_OPEN port={PORT} baud={BAUD}")
    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    rospy.on_shutdown(lambda: emergency_stop(ser, cmd_pub, tw_stop, pub_state=pub_state, stop_stream=True))

    # Handshake
    pub_state.publish("STATE=ARDUINO_HANDSHAKE")
    rospy.loginfo("Waiting for Arduino handshake...")
    if USE_READY:
        if not wait_for_ready(ser, 5.0):
            rospy.logwarn("No 'READY' seen, falling back to fixed delay")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    # Start streaming
    pub_state.publish("STATE=STREAM_ON")
    write_cmd(ser, "STREAM1")
    rospy.sleep(0.1)
    flush_input(ser)

    # -------------------- CALIBRATION --------------------
    pub_state.publish("STATE=CALIBRATE_BEGIN")
    
    # Find max values several times
    max_values = []
    for j in range(FIND_MAX_NUM):
        rospy.loginfo("Calibration find peak %d/%d", j+1, FIND_MAX_NUM)
        if rospy.is_shutdown():
            return
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)

        pub_state.publish(f"STATE=FIND_MAX j={j+1}/{FIND_MAX_NUM} t={DETECT_TIME_CAL}")
        flush_input(ser)  # make sure max is from fresh window
        mx, status = find_max_for(ser, DETECT_TIME_CAL, pub_raw=pub_raw)
        if status == "PAUSED":
            continue
        if mx is None:
            pub_state.publish("EVENT=FIND_MAX_NO_DATA")
        else:
            pub_state.publish(f"EVENT=FIND_MAX_OK value={mx:.1f}")
            max_values.append(mx)

    rospy.loginfo("Done")

    # Calculate threshold
    if len(max_values) == 0:
        threshold = THRESH_FALLBACK
        pub_state.publish(f"STATE=THRESH_FALLBACK thr={threshold:.1f}")
    else:
        avg_max = sum(max_values) / float(len(max_values))
        threshold = avg_max * THRESH_FACTOR
        pub_state.publish(
            f"STATE=THRESH_OK avg_max={avg_max:.1f} factor={THRESH_FACTOR:.2f} thr={threshold:.1f}"
        )

    rospy.loginfo("Calibration done. threshold=%.1f", threshold)

    for i in range(CAL_SPRAY_NUM):
            rospy.loginfo("Calibration spray %d/%d", i+1, CAL_SPRAY_NUM)
            if rospy.is_shutdown():
                return
            if not _run_event.is_set():
                handle_pause(ser, cmd_pub, tw_stop, pub_state)
            pub_state.publish(f"STATE=CAL_SPRAY i={i+1}/{CAL_SPRAY_NUM}")
            ok = spray_step(ser, CAL_SPRAY_TIME, pub_state=pub_state)
            if not ok:
                continue
            rospy.loginfo("Calibration wait for %.1f", CAL_WAIT_TIME)
            pub_state.publish(f"STATE=CAL_WAIT t={CAL_WAIT_TIME}")
            if not sleep_while_running(CAL_WAIT_TIME, pub_state=pub_state):
                continue

        rospy.loginfo("Calibration done.")

    # -------------------- MAIN LOOP --------------------
    pub_state.publish("STATE=MAIN_LOOP")
    rospy.loginfo("Start Main loop")

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue

        # 1) Detect above threshold
        pub_state.publish(f"STATE=DETECT thr={threshold:.1f} t={DETECT_TIME_MAIN}")
        flush_input(ser)  # crucial: avoid buffered old R lines
        res = detect_threshold_for(ser, DETECT_TIME_MAIN, threshold, pub_raw=pub_raw)
        if res is None:
            # paused
            continue

        pub_state.publish(f"EVENT=DETECT_RESULT detected={res}")

        # 2) Move if detected
        if res is True:
            rospy.loginfo("GAS RECEIVED -> MOVE")
            move_step(cmd_pub, tw_go, tw_stop, MOVE_TIME, PUB_RATE_HZ, pub_state=pub_state)
        else:
            rospy.loginfo("GAS NOT RECEIVED -> NO MOVE")
            pub_state.publish("STATE=NO_RECEPTION")

        # 3) Spray N times
        for k in range(MAIN_SPRAY_NUM):
            rospy.loginfo("Spary %d/%d", k+1, MAIN_SPRAY_NUM)
            if rospy.is_shutdown():
                break
            if not _run_event.is_set():
                break

            pub_state.publish(f"STATE=MAIN_SPRAY k={k+1}/{MAIN_SPRAY_NUM}")
            ok = spray_step(ser, MAIN_SPRAY_TIME, pub_state=pub_state)
            if not ok:
                break

            rospy.loginfo("Wait for %.1f", MAIN_WAIT_TIME)
            pub_state.publish(f"STATE=MAIN_WAIT t={MAIN_WAIT_TIME}")
            if not sleep_while_running(MAIN_WAIT_TIME, pub_state=pub_state):
                break

        if rospy.is_shutdown():
            break
        if not _run_event.is_set():
            continue

        rospy.loginfo("Done")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
