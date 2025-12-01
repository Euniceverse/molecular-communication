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

def publish_stop(pub, n=5, dt=0.02):
    z = Twist()
    for _ in range(n):
        pub.publish(z)
        rospy.sleep(dt)

# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_tx")

    # Arduino / sensing params
    PORT              = p("port", "/dev/ttyUSB0")
    BAUD              = int(p("baud", 57600))
    DIFF_THRESHOLD    = float(p("diff_threshold", 300.0))
    BASELINE_ALPHA    = float(p("baseline_alpha", 0.05))
    SPRAY_TIME        = float(p("spray_time", 5.0))
    COOLDOWN_TIME     = float(p("cooldown_time", 5.0))
    WAIT_TIMEOUT      = float(p("wait_timeout", 40.0))
    FIRST_CYCLE_EXTRA = float(p("first_cycle_extra", 0.0))
    PRIME_BASELINE_S  = float(p("prime_baseline_sec", 3.0))
    USE_READY         = bool(p("use_ready", False))

    # Motion + control params
    CMD_TOPIC         = p("cmd_vel_topic", "/cmd_vel")

    # Control uses either raw sensor or (value - baseline)
    CONTROL_SIGNAL    = p("control_signal", "diff")  # "diff" or "raw"

    # If True: during control, baseline is frozen (recommended if baseline drift breaks control)
    FREEZE_BASELINE_DURING_CONTROL = bool(p("freeze_baseline_during_control", True))

    TARGET            = float(p("target", 120.0))    # target RAW or target DIFF depending on CONTROL_SIGNAL
    TOL               = float(p("tolerance", 10.0))  # stop band ±
    KP                = float(p("kp", 0.002))        # proportional gain
    MAX_SPEED         = float(p("max_speed", 0.06))  # clamp speed
    CONTROL_HZ        = float(p("control_rate", 20.0))
    CONTROL_TIMEOUT   = float(p("control_timeout", 20.0))
    SETTLE_COUNT      = int(p("settle_count", 8))    # consecutive samples within tolerance

    # Direction convention:
    # We compute  error = TARGET - signal
    # cmd.linear.x = KP * error
    # If robot moves the wrong way, set invert_cmd:=True.
    INVERT_CMD        = bool(p("invert_cmd", False))

    # Publishers
    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    def on_shutdown():
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

    rospy.on_shutdown(on_shutdown)

    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
    except Exception as e:
        rospy.logerr("Failed to open serial %s: %s", PORT, str(e))
        return

    # Handle Arduino reset when serial opens
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

    # Prime relay for reliable first ON
    write_cmd(ser, "S0"); rospy.sleep(0.2)
    write_cmd(ser, "S1"); rospy.sleep(0.2)
    write_cmd(ser, "S0"); rospy.loginfo("Arduino primed.")

    # Prime baseline (EMA)
    baseline = None
    if PRIME_BASELINE_S > 0.0:
        rospy.loginfo("Priming baseline for %.1f s ...", PRIME_BASELINE_S)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < PRIME_BASELINE_S and not rospy.is_shutdown():
            val = readline_float_if_sensor(ser)
            if val is not None:
                baseline = val if baseline is None else (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * val
        if baseline is not None:
            rospy.loginfo("Initial baseline: %.2f", baseline)

    sensor_rate = rospy.Rate(20)
    first_cycle = True

    rospy.loginfo("Control mode: signal=%s target=%.2f tol=%.2f kp=%.4f max_speed=%.3f invert_cmd=%s",
                  CONTROL_SIGNAL, TARGET, TOL, KP, MAX_SPEED, str(INVERT_CMD))

    while not rospy.is_shutdown():
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        # 1) SPRAY ON
        extra = FIRST_CYCLE_EXTRA if first_cycle else 0.0
        spray_for = SPRAY_TIME + max(0.0, extra)
        rospy.loginfo("=== NEW CYCLE ===")
        rospy.loginfo("Spray ON for %.1f s", spray_for)
        write_cmd(ser, "S1")
        rospy.sleep(spray_for)
        write_cmd(ser, "S0")
        rospy.loginfo("Spray OFF, waiting (timeout=%.1f s)", WAIT_TIMEOUT)

        # 2) WAIT FOR RECEPTION (diff threshold)
        received = False
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

        first_cycle = False

        # 3) CLOSED-LOOP MOVE until signal ~= TARGET
        if received:
            rospy.loginfo(">>> RECEIVED: closed-loop moving until %s ~= %.1f (tol=%.1f)",
                          CONTROL_SIGNAL, TARGET, TOL)

            # Optionally freeze baseline once control starts
            frozen_baseline = baseline

            rate = rospy.Rate(CONTROL_HZ)
            t0 = rospy.Time.now()
            ok_streak = 0

            while not rospy.is_shutdown():
                if (rospy.Time.now() - t0).to_sec() > CONTROL_TIMEOUT:
                    rospy.logwarn("Control timeout reached; stopping.")
                    break

                value = readline_float_if_sensor(ser)
                if value is None:
                    rate.sleep()
                    continue

                # Baseline update (or freeze)
                if baseline is None:
                    baseline = value

                if FREEZE_BASELINE_DURING_CONTROL:
                    baseline_used = frozen_baseline if frozen_baseline is not None else baseline
                else:
                    baseline = (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * value
                    baseline_used = baseline

                diff = value - baseline_used

                # Choose signal
                if CONTROL_SIGNAL.lower() == "raw":
                    signal = value
                else:
                    signal = diff

                error = TARGET - signal  # positive -> signal too low -> move "closer" (subject to invert_cmd)
                if abs(error) <= TOL:
                    ok_streak += 1
                else:
                    ok_streak = 0

                if ok_streak >= SETTLE_COUNT:
                    rospy.loginfo("Target reached: value=%.1f base=%.1f diff=%.1f signal=%.1f",
                                  value, baseline_used, diff, signal)
                    break

                v = KP * error
                if INVERT_CMD:
                    v = -v
                v = clamp(v, -MAX_SPEED, MAX_SPEED)

                tw = Twist()
                tw.linear.x = v
                cmd_pub.publish(tw)

                rospy.loginfo("CTRL value=%.1f base=%.1f diff=%.1f signal=%.1f err=%.1f v=%.3f ok=%d/%d",
                              value, baseline_used, diff, signal, error, v, ok_streak, SETTLE_COUNT)

                rate.sleep()

            publish_stop(cmd_pub)
            rospy.loginfo("Control done.")
        else:
            rospy.loginfo("Cycle ended without reception")

        # 4) COOLDOWN
        rospy.sleep(COOLDOWN_TIME)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
