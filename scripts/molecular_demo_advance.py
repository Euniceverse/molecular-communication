#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from geometry_msgs.msg import Twist

def p(name, default):
    return rospy.get_param("~" + name, default)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def write_cmd(ser, s):
    ser.write((s if s.endswith("\n") else s + "\n").encode("ascii", errors="ignore"))

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

def readline_sensor_value(ser):
    """Expect lines like: 'R <float>'. Return float or None."""
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            return None
        if line.startswith("R "):
            return float(line.split()[1])
        return None
    except Exception:
        return None

def publish_stop(pub, n=5, dt=0.02):
    z = Twist()
    for _ in range(n):
        pub.publish(z)
        rospy.sleep(dt)

def main():
    rospy.init_node("molecular_demo_homing")

    # ---- Serial ----
    PORT      = p("port", "/dev/ttyUSB1")
    BAUD      = int(p("baud", 57600))
    USE_READY = bool(p("use_ready", False))

    # ---- Optional spray + receive gate ----
    ENABLE_SPRAY_CYCLE = bool(p("enable_spray_cycle", True))
    SPRAY_TIME         = float(p("spray_time", 5.0))
    WAIT_TIMEOUT       = float(p("wait_timeout", 40.0))
    DIFF_THRESHOLD     = float(p("diff_threshold", 300.0))

    # ---- Baseline background (for diff mode) ----
    PRIME_BASELINE_S = float(p("prime_baseline_sec", 3.0))
    BASELINE_ALPHA   = float(p("baseline_alpha", 0.05))

    # ---- Control signal ----
    # raw:  signal = value
    # diff: signal = value - baseline_bg  (baseline_bg is primed before spraying)
    CONTROL_SIGNAL = p("control_signal", "raw").lower()  # "raw" or "diff"

    # ---- "Bookmark" capture ----
    CAPTURE_SEC = float(p("capture_sec", 1.5))  # average signal for this long after reception

    # ---- Motion control ----
    CMD_TOPIC    = p("cmd_vel_topic", "/cmd_vel")
    KP           = float(p("kp", 0.002))
    MAX_SPEED    = float(p("max_speed", 0.06))
    TOL          = float(p("tolerance", 10.0))
    RATE_HZ      = float(p("rate_hz", 20.0))
    INVERT_CMD   = bool(p("invert_cmd", False))

    # ---- Drift compensation (decay handling) ----
    # We ONLY adapt the target when we believe we are already "at the right place".
    # target <- (1-alpha)*target + alpha*signal
    TARGET_ALPHA     = float(p("target_alpha", 0.01))   # smaller = slower drift-follow
    DRIFT_WINDOW     = float(p("drift_window", 2.5 * TOL))  # allow adaptation inside this error band
    V_DEADBAND       = float(p("v_deadband", 0.01))     # consider "not moving" if |v| < this
    REQUIRE_STABLE_N = int(p("require_stable_n", 10))   # consecutive stable ticks before adapting aggressively

    # ---- Safety ----
    STOP_ON_NO_SENSOR = bool(p("stop_on_no_sensor", True))
    CONTROL_TIMEOUT   = float(p("control_timeout", 0.0))  # 0 = infinite hold

    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    ser = None
    def shutdown_hook():
        try:
            publish_stop(cmd_pub)
        except Exception:
            pass
        try:
            if ser is not None:
                if ENABLE_SPRAY_CYCLE:
                    write_cmd(ser, "S0")
                ser.close()
        except Exception:
            pass
    rospy.on_shutdown(shutdown_hook)

    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=1.0)

    rospy.loginfo("Waiting for Arduino reboot/handshake...")
    if USE_READY:
        if not wait_for_ready(ser, 5.0):
            rospy.logwarn("No 'READY' seen, using fixed delay")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # ---- Prime baseline background (used for diff mode) ----
    baseline_bg = None
    if CONTROL_SIGNAL == "diff":
        rospy.loginfo("Priming baseline background for %.1f s (diff mode)...", PRIME_BASELINE_S)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < PRIME_BASELINE_S and not rospy.is_shutdown():
            v = readline_sensor_value(ser)
            if v is None:
                continue
            baseline_bg = v if baseline_bg is None else (1.0 - BASELINE_ALPHA) * baseline_bg + BASELINE_ALPHA * v
        rospy.loginfo("baseline_bg=%.2f", baseline_bg if baseline_bg is not None else float("nan"))

    def get_signal(value):
        if CONTROL_SIGNAL == "raw":
            return value
        # diff
        if baseline_bg is None:
            return 0.0
        return value - baseline_bg

    # ---- (Optional) prime sprayer relay ----
    if ENABLE_SPRAY_CYCLE:
        write_cmd(ser, "S0"); rospy.sleep(0.2)
        write_cmd(ser, "S1"); rospy.sleep(0.2)
        write_cmd(ser, "S0"); rospy.sleep(0.2)
        rospy.loginfo("Sprayer primed.")

    # =========================
    # 1) ACQUIRE: get initial "bookmark" target
    # =========================
    rospy.loginfo("ACQUIRE: will bookmark target signal at first reception.")
    target = None

    if ENABLE_SPRAY_CYCLE:
        rospy.loginfo("Spray ON for %.1f s", SPRAY_TIME)
        write_cmd(ser, "S1")
        rospy.sleep(SPRAY_TIME)
        write_cmd(ser, "S0")
        rospy.loginfo("Spray OFF. Waiting up to %.1f s for reception (diff_threshold=%.1f).",
                      WAIT_TIMEOUT, DIFF_THRESHOLD)

        start_wait = rospy.Time.now()
        received = False

        # note: reception logic uses DIFF (value-baseline_bg) if diff mode, else uses RAW threshold gate?
        # For simplicity: reception gate always uses "signal" and compares to DIFF_THRESHOLD when in diff mode,
        # and uses raw when in raw mode (you can tune DIFF_THRESHOLD accordingly).
        while not rospy.is_shutdown() and not received:
            if (rospy.Time.now() - start_wait).to_sec() > WAIT_TIMEOUT:
                rospy.logwarn("No reception within timeout. Exiting.")
                return

            value = readline_sensor_value(ser)
            if value is None:
                continue

            signal = get_signal(value)
            rospy.loginfo("WAIT value=%.1f signal(%s)=%.1f", value, CONTROL_SIGNAL, signal)

            if signal >= DIFF_THRESHOLD:
                received = True
                rospy.loginfo("RECEIVED: gate triggered.")
                break
    else:
        rospy.loginfo("enable_spray_cycle=false, capturing target immediately from current readings.")

    # Capture/average target for CAPTURE_SEC
    rospy.loginfo("Capturing target for %.1f s (averaging signal)...", CAPTURE_SEC)
    cap_start = rospy.Time.now()
    acc, n = 0.0, 0
    while not rospy.is_shutdown() and (rospy.Time.now() - cap_start).to_sec() < CAPTURE_SEC:
        value = readline_sensor_value(ser)
        if value is None:
            continue
        signal = get_signal(value)
        acc += signal
        n += 1

    if n == 0:
        rospy.logerr("Could not capture target: no sensor values.")
        return

    target = acc / n
    rospy.loginfo("BOOKMARKED target=%.2f (signal=%s, n=%d)", target, CONTROL_SIGNAL, n)

    # =========================
    # 2) HOLD: always try to return to the bookmarked signal,
    #          but allow target to drift slowly downward/upward when stable.
    # =========================
    rospy.loginfo("HOLD: if you move the robot, it will drive to restore signal to target.")
    rospy.loginfo("Drift compensation: target_alpha=%.4f, drift_window=%.2f", TARGET_ALPHA, DRIFT_WINDOW)

    rate = rospy.Rate(RATE_HZ)
    t_hold0 = rospy.Time.now()
    stable_ticks = 0
    last_cmd_v = 0.0

    while not rospy.is_shutdown():
        if CONTROL_TIMEOUT > 0.0 and (rospy.Time.now() - t_hold0).to_sec() > CONTROL_TIMEOUT:
            rospy.logwarn("Control timeout reached; stopping.")
            break

        value = readline_sensor_value(ser)
        if value is None:
            if STOP_ON_NO_SENSOR:
                publish_stop(cmd_pub, n=1, dt=0.0)
            rate.sleep()
            continue

        signal = get_signal(value)
        error = target - signal

        # P-control
        v = KP * error
        if INVERT_CMD:
            v = -v
        v = clamp(v, -MAX_SPEED, MAX_SPEED)

        # Deadband stop
        if abs(error) <= TOL:
            publish_stop(cmd_pub, n=1, dt=0.0)
        else:
            tw = Twist()
            tw.linear.x = v
            cmd_pub.publish(tw)

        # Stability detection (for safe target adaptation)
        moving = abs(v) > V_DEADBAND
        near_enough_for_drift = abs(error) <= DRIFT_WINDOW

        if (not moving) and near_enough_for_drift:
            stable_ticks += 1
        else:
            stable_ticks = 0

        # Drift compensation: only adapt when stable for a bit
        if stable_ticks >= REQUIRE_STABLE_N and near_enough_for_drift:
            target = (1.0 - TARGET_ALPHA) * target + TARGET_ALPHA * signal

        # Logging
        rospy.loginfo(
            "value=%.1f signal=%.1f target=%.1f err=%.1f cmd_v=%.3f stable=%d",
            value, signal, target, error, (0.0 if abs(error) <= TOL else v), stable_ticks
        )

        last_cmd_v = v
        rate.sleep()

    publish_stop(cmd_pub)
    rospy.loginfo("Exited HOLD, stopped.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
