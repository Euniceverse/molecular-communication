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

def capture_baseline_window(ser, duration_s, run_event, rate_hz=20.0):
    """Capture baseline for duration_s seconds. Returns float baseline or None."""
    if duration_s <= 0:
        return None

    vals = []
    t0 = rospy.Time.now()
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < duration_s:
        if not run_event.is_set():
            return None

        v = readline_float_if_sensor(ser)
        if v is not None:
            vals.append(v)

        rate.sleep()

    if not vals:
        return None

    return sum(vals) / len(vals)

def sleep_while_running(duration_s):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            return False
        if (rospy.Time.now() - t0).to_sec() >= duration_s:
            return True
        rospy.sleep(0.05)

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

# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_vertical_front")

    # Arduino / sensing params
    PORT           = p("port", "/dev/ttyUSB0")
    BAUD           = int(p("baud", 57600))
    DIFF_THRESHOLD = float(p("diff_threshold", 300.0))
    SPRAY_TIME     = float(p("spray_time", 5.0))
    COOLDOWN_TIME  = float(p("cooldown_time", 5.0))
    WAIT_TIMEOUT   = float(p("wait_timeout", 40.0))
    PRIME_BASELINE_S = float(p("prime_baseline_sec", 3.0))
    USE_READY      = bool(p("use_ready", False))

    # Motion params
    CMD_TOPIC   = p("cmd_vel_topic", "/cmd_vel")
    SPEED_LIN   = float(p("speed_linear", -0.04))
    SPEED_ANG   = float(p("speed_angular", 0.0))
    MOVE_TIME   = float(p("move_time", 2.0))
    PUB_RATE_HZ = float(p("publish_rate", 20.0))

    # Rest time 
    REST_TIME         = float(p("rest_time", 5.0))


    # Control topic
    CONTROL_TOPIC = p("control_topic", "/molecular_demo/control")
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=1)
    rospy.loginfo("Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)
    _run_event.clear()

    # Publisher
    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    # Twist messages
    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG
    tw_stop = Twist()

    # Wait for START before touching hardware
    wait_until_running()

    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
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

    sensor_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

        rospy.loginfo("Capturing baseline for %.1f s (this round)...", PRIME_BASELINE_S)
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        baseline = capture_baseline_window(ser, PRIME_BASELINE_S, _run_event, rate_hz=20.0)

        if baseline is None:
            # either stopped mid-capture or no sensor data
            if not _run_event.is_set():
                emergency_stop(ser, cmd_pub, tw_stop)
                wait_until_running()
            else:
                rospy.logwarn("Baseline capture failed (no data). Skipping this round.")
                rospy.sleep(0.5)
            continue

        rospy.loginfo("Round baseline fixed: %.2f", baseline)

        # Cycle 2+ : wait for reception BEFORE spraying
        received = False
        start_wait = rospy.Time.now()

        while not rospy.is_shutdown() and not received:
            if not _run_event.is_set():
                emergency_stop(ser, cmd_pub, tw_stop)
                wait_until_running()
                start_wait = rospy.Time.now()
                continue

            if (rospy.Time.now() - start_wait).to_sec() > WAIT_TIMEOUT:
                rospy.logwarn("No reception within timeout; skip spray this cycle.")
                break

            value = readline_float_if_sensor(ser)
            if value is None:
                sensor_rate.sleep()
                continue

            diff = value - baseline
            rospy.loginfo("value=%.1f, baseline=%.1f, diff=%.1f", value, baseline, diff)

            if diff >= DIFF_THRESHOLD:
                received = True
                break

            sensor_rate.sleep()

        if not received:
            if not sleep_while_running(COOLDOWN_TIME):
                emergency_stop(ser, cmd_pub, tw_stop)
                wait_until_running()
            continue

        # SPRAY (Cycle 1 always sprays; Cycle 2+ sprays only if received)
        rospy.loginfo("Spray ON for %.1f s", SPRAY_TIME)
        write_cmd(ser, "S1")
        ok = sleep_while_running(SPRAY_TIME)
        write_cmd(ser, "S0")

        if not ok:
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

        # MOVE
        rospy.loginfo("Moving for %.1f s on %s", MOVE_TIME, CMD_TOPIC)
        t0 = rospy.Time.now()
        rate = rospy.Rate(PUB_RATE_HZ)
        while (rospy.Time.now() - t0).to_sec() < MOVE_TIME and not rospy.is_shutdown():
            if not _run_event.is_set():
                break
            cmd_pub.publish(tw_go)
            rate.sleep()
        publish_stop(cmd_pub, tw_stop)

        rospy.loginfo("Rest for %.1f s", REST_TIME)
        if not sleep_while_running(REST_TIME):
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

        # COOLDOWN
        if not sleep_while_running(COOLDOWN_TIME):
            emergency_stop(ser, cmd_pub, tw_stop)
            wait_until_running()
            continue

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
