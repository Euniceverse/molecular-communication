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

def write_cmd(ser, s):
    ser.write((s if s.endswith("\n") else s + "\n").encode("ascii"))

def readline_float_if_sensor(ser):
    """
    Arduino line format expected: 'R <number>'
    Returns float or None.
    """
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line or not line.startswith("R "):
            return None
        return float(line.split()[1])
    except Exception:
        return None

def wait_for_ready(ser, timeout=5.0):
    t0 = rospy.Time.now()
    while (rospy.Time.now() - t0).to_sec() < timeout and not rospy.is_shutdown():
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            line = ""
        if line == "READY":
            return True
    return False

# -------------------- START/STOP control --------------------
_run_event = threading.Event()  # set()=running, clear()=paused

def control_cb(msg):
    cmd = msg.data.strip().lower()
    if cmd in ("start", "run", "go"):
        _run_event.set()
        rospy.loginfo("[FRONT] CONTROL: START")
    elif cmd in ("stop", "pause"):
        _run_event.clear()
        rospy.logwarn("[FRONT] CONTROL: STOP")
    else:
        rospy.logwarn("[FRONT] CONTROL: unknown '%s' (use 'start' or 'stop')", msg.data)

def wait_until_running():
    rospy.loginfo("[FRONT] Waiting for START... (publish 'start' to control topic)")
    while not rospy.is_shutdown() and not _run_event.is_set():
        rospy.sleep(0.1)

def sleep_while_running(duration_s):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            return False
        if (rospy.Time.now() - t0).to_sec() >= duration_s:
            return True
        rospy.sleep(0.02)

def publish_stop(cmd_pub, tw_stop, repeats=8):
    for _ in range(repeats):
        cmd_pub.publish(tw_stop)
        rospy.sleep(0.02)

def emergency_stop(ser, cmd_pub, tw_stop):
    # spray off + robot stop
    try:
        write_cmd(ser, "S0")
    except Exception:
        pass
    publish_stop(cmd_pub, tw_stop, repeats=10)

# -------------------- Detect: positive slope rule --------------------
def detect_positive_slope(ser, pub_raw, pub_state, t_d, r, rate_hz=20.0, t_out=None, slope_eps=0.0):
    """
    네가 정의한 규칙:
      - 센서가 계속 publish되므로, 매 t_d 구간(window)마다
        '전체적으로 증가했는가(last > first)' 를 판단
      - 그 "증가" 판단이 연속 r번 나오면 SUCCESS
      - t_out 지정 시 timeout
    반환:
      True (success), False (timeout/fail), None (중간에 STOP으로 pause됨)
    """
    consecutive = 0
    start_total = rospy.Time.now()
    rate = rospy.Rate(rate_hz)

    pub_state.publish(f"STATE=DETECT_START t_d={t_d} r={r} t_out={t_out}")

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            pub_state.publish("STATE=DETECT_PAUSED")
            return None

        if t_out is not None and (rospy.Time.now() - start_total).to_sec() >= float(t_out):
            pub_state.publish("STATE=DETECT_TIMEOUT")
            return False

        # one window of length t_d
        w0 = rospy.Time.now()
        first = None
        last = None

        while not rospy.is_shutdown() and (rospy.Time.now() - w0).to_sec() < float(t_d):
            if not _run_event.is_set():
                pub_state.publish("STATE=DETECT_PAUSED")
                return None

            v = readline_float_if_sensor(ser)
            if v is not None:
                pub_raw.publish(Float32(v))
                if first is None:
                    first = v
                last = v
            rate.sleep()

        ok = (first is not None and last is not None and (last > first + float(slope_eps)))

        if ok:
            consecutive += 1
            pub_state.publish(f"EVENT=DETECT_WINDOW_OK cons={consecutive} first={first:.1f} last={last:.1f}")
        else:
            consecutive = 0
            if first is None or last is None:
                pub_state.publish("EVENT=DETECT_WINDOW_NO_DATA cons=0")
            else:
                pub_state.publish(f"EVENT=DETECT_WINDOW_NO cons=0 first={first:.1f} last={last:.1f}")

        if consecutive >= int(r):
            pub_state.publish("STATE=DETECT_SUCCESS")
            return True

# -------------------- Main --------------------
def main():
    rospy.init_node("vertical_front")

    ROLE = "front"

    # Topics
    CONTROL_TOPIC = p("control_topic", "/molecular_demo/control")
    RAW_TOPIC     = p("raw_topic",     f"/molecular/{ROLE}/raw")
    STATE_TOPIC   = p("state_topic",   f"/molecular/{ROLE}/state")
    CMD_TOPIC     = p("cmd_vel_topic", "/cmd_vel")

    # Serial params
    PORT      = p("port", "/dev/ttyUSB0")
    BAUD      = int(p("baud", 57600))
    USE_READY = bool(p("use_ready", False))

    # R1 flow params: SPRAY -> WAIT -> DETECT -> MOVE
    SPRAY_TIME = float(p("spray_time", 5.0))
    WAIT_TIME  = float(p("wait_time",  5.0))

    # Detect params (positive slope)
    TD        = float(p("t_d",  2.0))     # window length
    RREQ      = int(p("r",      3))       # consecutive windows
    TOUT      = p("t_out", 40.0)          # timeout (seconds)
    RATE_HZ   = float(p("sense_rate", 20.0))
    SLOPE_EPS = float(p("slope_eps", 0.0))

    # Motion params
    SPEED_LIN   = float(p("speed_linear", 0.05))
    SPEED_ANG   = float(p("speed_angular", 0.0))
    MOVE_TIME   = float(p("move_time", 2.0))
    PUB_RATE_HZ = float(p("publish_rate", 20.0))

    # Subscribers / Publishers
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=10)
    pub_raw   = rospy.Publisher(RAW_TOPIC, Float32, queue_size=1000)
    pub_state = rospy.Publisher(STATE_TOPIC, String, queue_size=1000)
    cmd_pub   = rospy.Publisher(CMD_TOPIC, Twist, queue_size=10)

    rospy.loginfo("[FRONT] Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)
    pub_state.publish("STATE=BOOT")

    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG
    tw_stop = Twist()

    _run_event.clear()
    wait_until_running()

    pub_state.publish(f"STATE=SERIAL_OPEN port={PORT} baud={BAUD}")
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    rospy.on_shutdown(lambda: emergency_stop(ser, cmd_pub, tw_stop))

    pub_state.publish("STATE=ARDUINO_HANDSHAKE")
    if USE_READY:
        if not wait_for_ready(ser, 5.0):
            pub_state.publish("EVENT=NO_READY_FALLBACK_DELAY")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # Prime relay
    pub_state.publish("STATE=PRIME_RELAY")
    write_cmd(ser, "S0"); rospy.sleep(0.2)
    write_cmd(ser, "S1"); rospy.sleep(0.2)
    write_cmd(ser, "S0"); rospy.sleep(0.2)
    pub_state.publish("STATE=PRIMED")

    # Main loop
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            pub_state.publish("STATE=PAUSED")
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            pub_state.publish("STATE=RESUMED")
            continue

        # 1) SPRAY
        pub_state.publish(f"STATE=SPRAY_ON t_s={SPRAY_TIME}")
        write_cmd(ser, "S1")
        ok = sleep_while_running(SPRAY_TIME)
        write_cmd(ser, "S0")
        pub_state.publish("STATE=SPRAY_OFF")
        if not ok:
            continue

        # 2) WAIT
        pub_state.publish(f"STATE=WAIT t_w={WAIT_TIME}")
        if not sleep_while_running(WAIT_TIME):
            continue

        # 3) DETECT (positive slope)
        res = detect_positive_slope(
            ser=ser,
            pub_raw=pub_raw,
            pub_state=pub_state,
            t_d=TD,
            r=RREQ,
            rate_hz=RATE_HZ,
            t_out=TOUT,
            slope_eps=SLOPE_EPS
        )
        if res is None:
            # paused
            continue
        if res is False:
            # timeout -> retry from SPRAY
            pub_state.publish("STATE=ROUND_FAIL_RETRY")
            continue

        # 4) MOVE
        pub_state.publish(f"STATE=MOVE time={MOVE_TIME}")
        t0 = rospy.Time.now()
        rate = rospy.Rate(PUB_RATE_HZ)
        while (rospy.Time.now() - t0).to_sec() < MOVE_TIME and not rospy.is_shutdown():
            if not _run_event.is_set():
                break
            cmd_pub.publish(tw_go)
            rate.sleep()
        publish_stop(cmd_pub, tw_stop)
        pub_state.publish("STATE=MOVE_DONE")

        # 2) WAIT
        pub_state.publish(f"STATE=WAIT t_w={WAIT_TIME}")
        if not sleep_while_running(WAIT_TIME):
            continue

        # loop repeats

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
