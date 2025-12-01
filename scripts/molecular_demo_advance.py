#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from geometry_msgs.msg import Twist

# -------------------- Helpers --------------------
def p(name, default):
    return rospy.get_param("~" + name, default)

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def write_cmd(ser, s):
    ser.write((s if s.endswith("\n") else s + "\n").encode("ascii", errors="ignore"))

def readline_float_if_sensor(ser):
    """
    Expect Arduino lines like:
      READY
      R <float>
    Returns float for R-lines, otherwise None.
    """
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            return None
        if line.startswith("R "):
            return float(line.split()[1])
        return None
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

def publish_stop(pub, n=6, dt=0.02):
    z = Twist()
    for _ in range(n):
        pub.publish(z)
        rospy.sleep(dt)

# -------------------- Control core --------------------
def compute_signal(value, baseline_used, control_signal):
    diff = value - baseline_used
    if control_signal.lower() == "raw":
        return value, diff
    return diff, diff  # signal=diff

def step_cmd(error, kp, max_speed, invert):
    v = kp * error
    if invert:
        v = -v
    v = clamp(v, -max_speed, max_speed)
    tw = Twist()
    tw.linear.x = v
    return tw, v

# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_tx")

    # ---- Serial / sensing params ----
    PORT              = p("port", "/dev/ttyUSB0")
    BAUD              = int(p("baud", 57600))
    USE_READY         = bool(p("use_ready", False))

    # Spray + reception gate (optional)
    ENABLE_SPRAY_CYCLE = bool(p("enable_spray_cycle", True))
    SPRAY_TIME        = float(p("spray_time", 5.0))
    COOLDOWN_TIME     = float(p("cooldown_time", 5.0))
    WAIT_TIMEOUT      = float(p("wait_timeout", 40.0))
    DIFF_THRESHOLD    = float(p("diff_threshold", 300.0))

    # Baseline
    BASELINE_ALPHA    = float(p("baseline_alpha", 0.05))
    PRIME_BASELINE_S  = float(p("prime_baseline_sec", 3.0))

    # ---- Motion / control params ----
    CMD_TOPIC         = p("cmd_vel_topic", "/cmd_vel")

    CONTROL_SIGNAL    = p("control_signal", "diff")   # "diff" or "raw"
    TARGET            = float(p("target", 120.0))      # target signal (raw or diff)
    TOL               = float(p("tolerance", 10.0))
    KP                = float(p("kp", 0.002))
    MAX_SPEED         = float(p("max_speed", 0.06))
    CONTROL_HZ        = float(p("control_rate", 20.0))
    CONTROL_TIMEOUT   = float(p("control_timeout", 20.0))
    SETTLE_COUNT      = int(p("settle_count", 8))
    INVERT_CMD        = bool(p("invert_cmd", False))

    # Baseline handling during control/hold
    FREEZE_BASELINE_DURING_CONTROL = bool(p("freeze_baseline_during_control", True))
    FREEZE_BASELINE_DURING_HOLD    = bool(p("freeze_baseline_during_hold", True))
    BASELINE_ALPHA_HOLD            = float(p("baseline_alpha_hold", 0.005))  # if not frozen

    # ---- HOLD mode (the "always-on control loop") ----
    HOLD_ENABLED      = bool(p("hold_enabled", True))
    HOLD_TIME         = float(p("hold_time", 0.0))    # 0.0 = infinite until Ctrl-C
    HOLD_HZ           = float(p("hold_rate", 10.0))
    HOLD_TOL          = float(p("hold_tolerance", TOL))
    HOLD_KP           = float(p("hold_kp", KP))
    HOLD_MAX_SPEED    = float(p("hold_max_speed", MAX_SPEED))
    HOLD_DEADBAND_STOP_PUB = bool(p("hold_publish_stop_in_deadband", True))

    # Publishers
    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    ser = None

    def shutdown_hook():
        try:
            publish_stop(cmd_pub)
        except Exception:
            pass
        try:
            if ser is not None:
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
            rospy.logwarn("No 'READY' seen, falling back to fixed delay")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # Prime relay (optional but helpful)
    if ENABLE_SPRAY_CYCLE:
        write_cmd(ser, "S0"); rospy.sleep(0.2)
        write_cmd(ser, "S1"); rospy.sleep(0.2)
        write_cmd(ser, "S0"); rospy.loginfo("Arduino primed.")

    # Prime baseline
    baseline = None
    if PRIME_BASELINE_S > 0.0:
        rospy.loginfo("Priming baseline for %.1f s ...", PRIME_BASELINE_S)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < PRIME_BASELINE_S and not rospy.is_shutdown():
            val = readline_float_if_sensor(ser)
            if val is None:
                continue
            baseline = val if baseline is None else (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * val
        if baseline is not None:
            rospy.loginfo("Initial baseline: %.2f", baseline)

    sensor_rate = rospy.Rate(20)

    rospy.loginfo(
        "Params: signal=%s target=%.2f tol=%.2f kp=%.4f max=%.3f invert=%s hold=%s",
        CONTROL_SIGNAL, TARGET, TOL, KP, MAX_SPEED, str(INVERT_CMD), str(HOLD_ENABLED)
    )

    # -------------------- Main loop --------------------
    while not rospy.is_shutdown():
        # 1) Optional spray cycle + wait for reception
        received = True  # if spray cycle disabled, go straight to control
        if ENABLE_SPRAY_CYCLE:
            received = False
            try:
                ser.reset_input_buffer()
            except Exception:
                pass

            rospy.loginfo("=== NEW CYCLE ===")
            rospy.loginfo("Spray ON for %.1f s", SPRAY_TIME)
            write_cmd(ser, "S1")
            rospy.sleep(SPRAY_TIME)
            write_cmd(ser, "S0")
            rospy.loginfo("Spray OFF, waiting (timeout=%.1f s)", WAIT_TIMEOUT)

            start_wait = rospy.Time.now()
            while not rospy.is_shutdown() and not received:
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
                rospy.loginfo("WAIT value=%.1f baseline=%.1f diff=%.1f (thr=%.1f)", value, baseline, diff, DIFF_THRESHOLD)

                if diff >= DIFF_THRESHOLD:
                    received = True
                    break

                sensor_rate.sleep()

        if not received:
            rospy.sleep(COOLDOWN_TIME)
            continue

        # 2) Approach: control until at target (reach & settle)
        rospy.loginfo(">>> CONTROL: move until %s ~= %.1f (tol=%.1f)", CONTROL_SIGNAL, TARGET, TOL)

        frozen_baseline = baseline
        ok_streak = 0
        rate = rospy.Rate(CONTROL_HZ)
        t0 = rospy.Time.now()

        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > CONTROL_TIMEOUT:
                rospy.logwarn("Control timeout; stopping.")
                break

            value = readline_float_if_sensor(ser)
            if value is None:
                rate.sleep()
                continue

            # choose baseline used
            if baseline is None:
                baseline = value
            if FREEZE_BASELINE_DURING_CONTROL:
                baseline_used = frozen_baseline if frozen_baseline is not None else baseline
            else:
                baseline = (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * value
                baseline_used = baseline

            signal, diff = compute_signal(value, baseline_used, CONTROL_SIGNAL)
            error = TARGET - signal

            if abs(error) <= TOL:
                ok_streak += 1
            else:
                ok_streak = 0

            if ok_streak >= SETTLE_COUNT:
                rospy.loginfo("Reached target: value=%.1f base=%.1f diff=%.1f signal=%.1f", value, baseline_used, diff, signal)
                break

            tw, v = step_cmd(error, KP, MAX_SPEED, INVERT_CMD)
            cmd_pub.publish(tw)

            rospy.loginfo("CTRL value=%.1f base=%.1f diff=%.1f signal=%.1f err=%.1f v=%.3f ok=%d/%d",
                          value, baseline_used, diff, signal, error, v, ok_streak, SETTLE_COUNT)
            rate.sleep()

        publish_stop(cmd_pub)
        rospy.loginfo("Control done.")

        # 3) HOLD: always-on loop to keep near target
        if HOLD_ENABLED:
            rospy.loginfo(">>> HOLD: keep %s near target=%.1f (tol=%.1f). hold_time=%.1f (0=infinite)",
                          CONTROL_SIGNAL, TARGET, HOLD_TOL, HOLD_TIME)

            hold_start = rospy.Time.now()
            hold_rate = rospy.Rate(HOLD_HZ)

            # freeze baseline at the moment we enter HOLD (recommended for diff)
            hold_frozen_baseline = baseline_used if 'baseline_used' in locals() else baseline

            while not rospy.is_shutdown():
                if HOLD_TIME > 0.0 and (rospy.Time.now() - hold_start).to_sec() > HOLD_TIME:
                    rospy.loginfo("Hold time ended; leaving HOLD.")
                    break

                value = readline_float_if_sensor(ser)
                if value is None:
                    # if we can't read, safest is stop
                    publish_stop(cmd_pub, n=1, dt=0.0)
                    hold_rate.sleep()
                    continue

                # baseline used during hold
                if baseline is None:
                    baseline = value

                if FREEZE_BASELINE_DURING_HOLD or CONTROL_SIGNAL.lower() == "raw":
                    baseline_hold = hold_frozen_baseline if hold_frozen_baseline is not None else baseline
                else:
                    baseline = (1.0 - BASELINE_ALPHA_HOLD) * baseline + BASELINE_ALPHA_HOLD * value
                    baseline_hold = baseline

                signal, diff = compute_signal(value, baseline_hold, CONTROL_SIGNAL)
                error = TARGET - signal

                if abs(error) <= HOLD_TOL:
                    # inside band: stop (or keep last stop)
                    if HOLD_DEADBAND_STOP_PUB:
                        publish_stop(cmd_pub, n=1, dt=0.0)
                    rospy.loginfo("HOLD OK value=%.1f base=%.1f diff=%.1f signal=%.1f err=%.1f (STOP)",
                                  value, baseline_hold, diff, signal, error)
                else:
                    tw, v = step_cmd(error, HOLD_KP, HOLD_MAX_SPEED, INVERT_CMD)
                    cmd_pub.publish(tw)
                    rospy.loginfo("HOLD ADJ value=%.1f base=%.1f diff=%.1f signal=%.1f err=%.1f v=%.3f",
                                  value, baseline_hold, diff, signal, error, v)

                hold_rate.sleep()

            publish_stop(cmd_pub)
            rospy.loginfo("HOLD done.")

        # 4) Cooldown between cycles
        rospy.sleep(COOLDOWN_TIME)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
