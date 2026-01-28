#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# -------------------- Params --------------------
def p(name, default):
    return rospy.get_param("~" + name, default)

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

def write_cmd(ser, s):
    ser.write((s if s.endswith("\n") else s + "\n").encode("ascii"))

# -------------------- START/STOP control --------------------
_run_event = threading.Event()  # set() = running, clear() = paused/stopped

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

def wait_until_running():
    rospy.loginfo("Waiting for START... (publish 'start' to control topic)")
    while not rospy.is_shutdown() and not _run_event.is_set():
        rospy.sleep(0.1)

def sleep_while_running(duration_s):
    """
    Sleep up to duration_s but return False early if STOP happens.
    True  = slept full duration
    False = stopped/paused during sleep
    """
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            return False
        if (rospy.Time.now() - t0).to_sec() >= duration_s:
            return True
        rospy.sleep(0.05)

def publish_stop(cmd_pub, tw_stop, repeats=5):
    for _ in range(repeats):
        cmd_pub.publish(tw_stop)
        rospy.sleep(0.02)

def emergency_stop(ser, cmd_pub, tw_stop):
    # spray OFF + robot stop
    try:
        write_cmd(ser, "S0")
    except Exception:
        pass
    publish_stop(cmd_pub, tw_stop, repeats=8)

# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_rx")

    # Arduino / sensing params
    PORT              = p("port", "/dev/ttyUSB0")
    BAUD              = int(p("baud", 9600))
    DIFF_THRESHOLD    = float(p("diff_threshold", 300.0))
    BASELINE_ALPHA    = float(p("baseline_alpha", 0.05))
    SPRAY_TIME        = float(p("spray_time", 5.0))
    COOLDOWN_TIME     = float(p("cooldown_time", 5.0))
    WAIT_TIMEOUT      = float(p("wait_timeout", 40.0))
    PRIME_BASELINE_S  = float(p("prime_baseline_sec", 3.0))
    USE_READY         = bool(p("use_ready", False))

    # Motion params
    CMD_TOPIC         = p("cmd_vel_topic", "/cmd_vel")
    SPEED_LIN         = float(p("speed_linear", -0.04))
    SPEED_ANG         = float(p("speed_angular", 0.0))
    MOVE_TIME         = float(p("move_time", 2.0))
    PUB_RATE_HZ       = float(p("publish_rate", 20.0))

    # Control topic
    CONTROL_TOPIC     = p("control_topic", "/molecular_demo/control")
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=1)
    rospy.loginfo("Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)

    # Start in STOP state (important)
    _run_event.clear()

    # Publisher
    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    # Prebuilt Twist messages
    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG
    tw_stop = Twist()

    # Wait for START before touching hardware (optional but usually what you want)
    wait_until_running()

    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=1.0)

    # Ensure we stop safely on shutdown
    rospy.on_shutdown(lambda: emergency_stop(ser, cmd_pub, tw_stop))

    rospy.loginfo("Waiting for Arduino reboot/handshake...")
    if USE_READY:
        if not wait_for_ready(ser, 5.0):
            rospy.logwarn("No 'READY' seen, falling back to fixed delay")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # Prime relay
    write_cmd(ser, "S0"); rospy.sleep(0.2)
    write_cmd(ser, "S1"); rospy.sleep(0.2)
    write_cmd(ser, "S0"); rospy.loginfo("Arduino primed.")

    # Prime baseline
    baseline = None
    if PRIME_BASELINE_S > 0.0:
        rospy.loginfo("Priming baseline for %.1f s ...", PRIME_BASELINE_S)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < PRIME_BASELINE_S and not rospy.is_shutdown():
            if not _run_event.is_set():
                emergency_stop(ser, cmd_pub, tw_stop)
                wait_until_running()
                t0 = rospy.Time.now()
                continue

            val = readline_float_if_sensor(ser)
            if val is not None:
                baseline = val if baseline is None else (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * val

        if baseline is not None:
            rospy.loginfo("Initial baseline: %.2f", baseline)

    sensor_rate = rospy.Rate(20)

    rospy.loginfo("Running cycles. (Send 'stop' anytime to pause)")

    while not rospy.is_shutdown():
        # If stopped, ensure safe state and wait
        if not _run_event.is_set():
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        # 1) WAIT FOR RECEPTION
        received = False
        start_wait = rospy.Time.now()

        while not rospy.is_shutdown() and not received:
            if not _run_event.is_set():
                emergency_stop(ser, cmd_pub, tw_stop)
                wait_until_running()
                start_wait = rospy.Time.now()  # reset timeout after resume
                continue

            if (rospy.Time.now() - start_wait).to_sec() > WAIT_TIMEOUT:
                rospy.logwarn("No reception within timeout; next cycle.")
                break

            value = readline_float_if_sensor(ser)
            if value is None:
                sensor_rate.sleep()
                continue

            if baseline is None:
                baseline = value
            else:
                baseline = (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * value

            diff = value - baseline
            rospy.loginfo("value=%.1f, baseline=%.1f, diff=%.1f", value, baseline, diff)

            if diff >= DIFF_THRESHOLD:
                received = True
                break

            sensor_rate.sleep()

        if not received:
            rospy.loginfo("No reception -> do NOT spray. Cooldown and retry.")
            if not sleep_while_running(COOLDOWN_TIME):
                emergency_stop(ser, cmd_pub, tw_stop)
                wait_until_running()
            continue

        # 2) SPRAY ON
        spray_for = SPRAY_TIME

        rospy.loginfo("=== NEW CYCLE ===")
        rospy.loginfo("Spray ON for %.1f s", spray_for)

        write_cmd(ser, "S1")
        finished = sleep_while_running(spray_for)   # STOP can interrupt here
        write_cmd(ser, "S0")

        if not finished:
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

        rospy.loginfo("Spray OFF, waiting (timeout=%.1f s)", WAIT_TIMEOUT)

        # 3) MOVE
        if received:
            rospy.loginfo(">>> GAS RECEIVED: moving for %.1f s on %s", MOVE_TIME, CMD_TOPIC)
            t0 = rospy.Time.now()
            rate = rospy.Rate(PUB_RATE_HZ)

            while (rospy.Time.now() - t0).to_sec() < MOVE_TIME and not rospy.is_shutdown():
                if not _run_event.is_set():
                    break
                cmd_pub.publish(tw_go)
                rate.sleep()

            publish_stop(cmd_pub, tw_stop, repeats=8)
            rospy.loginfo("Move done.")
        else:
            rospy.loginfo("Cycle ended without reception")

        # 4) COOLDOWN (STOP can pause here too)
        if not sleep_while_running(COOLDOWN_TIME):
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
