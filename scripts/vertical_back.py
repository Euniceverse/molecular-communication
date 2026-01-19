#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32

def p(name, default):
    return rospy.get_param("~" + name, default)

def write_cmd(ser, s):
    ser.write((s if s.endswith("\n") else s + "\n").encode("ascii"))

def readline_float_if_sensor(ser):
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

_run_event = threading.Event()

def control_cb(msg):
    cmd = msg.data.strip().lower()
    if cmd in ("start", "run", "go"):
        _run_event.set()
        rospy.loginfo("[BACK] CONTROL: START")
    elif cmd in ("stop", "pause"):
        _run_event.clear()
        rospy.logwarn("[BACK] CONTROL: STOP")
    else:
        rospy.logwarn("[BACK] CONTROL: unknown '%s' (use 'start' or 'stop')", msg.data)

def wait_until_running():
    rospy.loginfo("[BACK] Waiting for START... (publish 'start' to control topic)")
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
    try:
        write_cmd(ser, "S0")
    except Exception:
        pass
    publish_stop(cmd_pub, tw_stop, repeats=10)

def detect_positive_slope(ser, pub_raw, pub_state, t_d, r, rate_hz=20.0, t_out=None, slope_eps=0.0):
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

def do_move(cmd_pub, tw, tw_stop, duration_s, pub_state, label, pub_rate_hz=20.0):
    pub_state.publish(f"STATE={label} time={duration_s}")
    t0 = rospy.Time.now()
    rate = rospy.Rate(pub_rate_hz)
    while (rospy.Time.now() - t0).to_sec() < duration_s and not rospy.is_shutdown():
        if not _run_event.is_set():
            break
        cmd_pub.publish(tw)
        rate.sleep()
    publish_stop(cmd_pub, tw_stop)
    pub_state.publish(f"STATE={label}_DONE")

def main():
    rospy.init_node("vertical_back")

    ROLE = "back"

    CONTROL_TOPIC = p("control_topic", "/molecular_demo/control")
    RAW_TOPIC     = p("raw_topic",     f"/molecular/{ROLE}/raw")
    STATE_TOPIC   = p("state_topic",   f"/molecular/{ROLE}/state")
    CMD_TOPIC     = p("cmd_vel_topic", "/cmd_vel")

    PORT      = p("port", "/dev/ttyUSB0")
    BAUD      = int(p("baud", 57600))
    USE_READY = bool(p("use_ready", False))

    # R2 flow: DETECT1 -> SPRAY -> MOVE -> DETECT2(timeout) -> (timeout이면 BACKOFF) -> repeat
    SPRAY_TIME = float(p("spray_time", 5.0))

    # Detect params
    TD1       = float(p("t_d1", 2.0))
    R1REQ     = int(p("r1", 3))
    TD2       = float(p("t_d2", 2.0))
    R2REQ     = int(p("r2", 3))
    TOUT2     = float(p("t_out2", 40.0))
    RATE_HZ   = float(p("sense_rate", 20.0))
    SLOPE_EPS = float(p("slope_eps", 0.0))

    # Motion
    SPEED_LIN       = float(p("speed_linear", 0.05))
    SPEED_ANG       = float(p("speed_angular", 0.0))
    MOVE_TIME       = float(p("move_time", 2.0))
    SPEED_LIN_BACK  = float(p("speed_linear_back", -0.05))
    BACK_TIME       = float(p("back_time", 2.0))
    PUB_RATE_HZ     = float(p("publish_rate", 20.0))

    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=10)
    pub_raw   = rospy.Publisher(RAW_TOPIC, Float32, queue_size=1000)
    pub_state = rospy.Publisher(STATE_TOPIC, String, queue_size=1000)
    cmd_pub   = rospy.Publisher(CMD_TOPIC, Twist, queue_size=10)

    rospy.loginfo("[BACK] Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)
    pub_state.publish("STATE=BOOT")

    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG

    tw_back = Twist()
    tw_back.linear.x = SPEED_LIN_BACK
    tw_back.angular.z = 0.0

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

    pub_state.publish("STATE=PRIME_RELAY")
    write_cmd(ser, "S0"); rospy.sleep(0.2)
    write_cmd(ser, "S1"); rospy.sleep(0.2)
    write_cmd(ser, "S0"); rospy.sleep(0.2)
    pub_state.publish("STATE=PRIMED")

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            pub_state.publish("STATE=PAUSED")
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            pub_state.publish("STATE=RESUMED")
            continue

        # DETECT1 (timeout 없이 기다리는 단계)
        pub_state.publish("STATE=DETECT1")
        res1 = detect_positive_slope(
            ser=ser, pub_raw=pub_raw, pub_state=pub_state,
            t_d=TD1, r=R1REQ, rate_hz=RATE_HZ, t_out=None, slope_eps=SLOPE_EPS
        )
        if res1 is None:
            continue
        # res1은 timeout 없음이라 보통 True

        # SPRAY
        pub_state.publish(f"STATE=SPRAY_ON t_s={SPRAY_TIME}")
        write_cmd(ser, "S1")
        ok = sleep_while_running(SPRAY_TIME)
        write_cmd(ser, "S0")
        pub_state.publish("STATE=SPRAY_OFF")
        if not ok:
            continue

        # MOVE
        do_move(cmd_pub, tw_go, tw_stop, MOVE_TIME, pub_state, "MOVE", pub_rate_hz=PUB_RATE_HZ)

        # DETECT2 (timeout 있음)
        pub_state.publish("STATE=DETECT2")
        res2 = detect_positive_slope(
            ser=ser, pub_raw=pub_raw, pub_state=pub_state,
            t_d=TD2, r=R2REQ, rate_hz=RATE_HZ, t_out=TOUT2, slope_eps=SLOPE_EPS
        )
        if res2 is None:
            continue

        if res2 is True:
            pub_state.publish("STATE=ROUND_SUCCESS_NEXT")
            continue

        # timeout -> BACKOFF(후진) 후 DETECT1로
        pub_state.publish("STATE=DETECT2_FAIL_BACKOFF")
        do_move(cmd_pub, tw_back, tw_stop, BACK_TIME, pub_state, "BACKOFF", pub_rate_hz=PUB_RATE_HZ)
        pub_state.publish("STATE=BACK_TO_DETECT1")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
