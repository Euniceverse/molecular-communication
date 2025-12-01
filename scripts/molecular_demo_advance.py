#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import serial
from geometry_msgs.msg import Twist

def p(name, default): return rospy.get_param("~" + name, default)

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
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            return None
        if line.startswith("R "):
            return float(line.split()[1])
        return None
    except Exception:
        return None

def publish(pub, tw, hz, duration):
    r = rospy.Rate(hz)
    t0 = rospy.Time.now()
    while (rospy.Time.now() - t0).to_sec() < duration and not rospy.is_shutdown():
        pub.publish(tw)
        r.sleep()

def stop(pub, n=6, dt=0.02):
    z = Twist()
    for _ in range(n):
        pub.publish(z)
        rospy.sleep(dt)

def mean(samples):
    return sum(samples) / float(len(samples)) if samples else None

def robust_mean(samples, trim=0.2):
    if not samples:
        return None
    s = sorted(samples)
    k = int(trim * len(s))
    core = s[k:len(s)-k] if len(s) - 2*k >= 3 else s
    return sum(core) / float(len(core))

def read_signal_window(ser, signal_from_value, sec, min_n=5):
    """Collect signal samples for `sec` seconds and return robust mean."""
    t0 = rospy.Time.now()
    xs = []
    while (rospy.Time.now() - t0).to_sec() < sec and not rospy.is_shutdown():
        v = readline_sensor_value(ser)
        if v is None:
            continue
        xs.append(signal_from_value(v))
    if len(xs) < min_n:
        return None
    return robust_mean(xs)

def main():
    rospy.init_node("molecular_demo_homing_decay_step")

    # ---------- Serial ----------
    PORT      = p("port", "/dev/ttyUSB1")
    BAUD      = int(p("baud", 57600))
    USE_READY = bool(p("use_ready", False))

    # ---------- Optional spray + receive gate ----------
    ENABLE_SPRAY_CYCLE = bool(p("enable_spray_cycle", True))
    SPRAY_TIME         = float(p("spray_time", 5.0))
    WAIT_TIMEOUT       = float(p("wait_timeout", 40.0))
    THRESHOLD_SIGNAL   = float(p("threshold_signal", 250.0))  # threshold on chosen signal

    # ---------- Signal type ----------
    CONTROL_SIGNAL   = p("control_signal", "diff").lower()  # "raw" or "diff"
    PRIME_BASELINE_S = float(p("prime_baseline_sec", 8.0))  # longer helps
    BASELINE_ALPHA   = float(p("baseline_alpha", 0.05))

    # ---------- Capture timing ----------
    CAPTURE_DELAY_SEC = float(p("capture_delay_sec", 4.0))  # longer settling helps a LOT
    CAPTURE_SEC       = float(p("capture_sec", 3.0))

    # ---------- Decay model ----------
    DECAY_HALFLIFE_S = float(p("decay_halflife_sec", 180.0))
    DECAY_FLOOR      = float(p("decay_floor", 0.0))  # for diff: 0 is correct target floor

    # ---------- ROS cmd_vel ----------
    CMD_TOPIC  = p("cmd_vel_topic", "/cmd_vel")
    PUB_HZ     = float(p("publish_hz", 20.0))

    # ---------- Step controller (key!) ----------
    TOL              = float(p("tolerance", 25.0))
    STEP_SPEED       = float(p("step_speed", 0.015))   # m/s
    STEP_TIME        = float(p("step_time", 0.25))     # seconds per step
    STEP_TIME_MIN    = float(p("step_time_min", 0.08))
    STEP_TIME_MAX    = float(p("step_time_max", 0.45))
    SETTLE_SEC       = float(p("settle_sec", 1.0))     # wait for sensor after each motion
    MEASURE_SEC      = float(p("measure_sec", 0.6))    # measure window while stopped
    HOLD_TIME        = float(p("hold_time", 0.0))      # 0=infinite

    # Adaptation when overshoot happens (error sign flips)
    OVERSHOOT_SHRINK = float(p("overshoot_shrink", 0.6))  # multiply step_time when sign flips
    FAR_GROW         = float(p("far_grow", 1.10))          # grow step_time slowly when far
    FAR_ERR          = float(p("far_err", 200.0))          # threshold of "far"

    # ---------- Direction auto-calibration ----------
    AUTO_DIR           = bool(p("auto_direction", True))
    CALIB_SPEED        = float(p("calib_speed", 0.012))
    CALIB_TIME         = float(p("calib_time", 0.25))
    CALIB_SETTLE       = float(p("calib_settle", 1.0))
    CALIB_MEASURE_SEC  = float(p("calib_measure_sec", 0.6))

    # ---------- Safety ----------
    STOP_ON_NO_SENSOR = bool(p("stop_on_no_sensor", True))

    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    ser = None
    def shutdown_hook():
        try:
            stop(cmd_pub)
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
            rospy.logwarn("No READY; using fixed delay")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    # ---------- Prime baseline for diff ----------
    baseline_bg = None
    if CONTROL_SIGNAL == "diff":
        rospy.loginfo("Priming baseline for diff-mode (%.1f s)...", PRIME_BASELINE_S)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < PRIME_BASELINE_S and not rospy.is_shutdown():
            v = readline_sensor_value(ser)
            if v is None:
                continue
            baseline_bg = v if baseline_bg is None else (1.0-BASELINE_ALPHA)*baseline_bg + BASELINE_ALPHA*v
        rospy.loginfo("baseline_bg=%.2f", baseline_bg if baseline_bg is not None else float("nan"))

    def signal_from_value(v):
        if CONTROL_SIGNAL == "raw":
            return v
        return 0.0 if baseline_bg is None else (v - baseline_bg)

    # ---------- Decay lambda ----------
    lam = 0.0 if DECAY_HALFLIFE_S <= 0 else (math.log(2.0) / DECAY_HALFLIFE_S)
    rospy.loginfo("Decay: half-life=%.1fs lam=%.5f floor=%.2f signal=%s", DECAY_HALFLIFE_S, lam, DECAY_FLOOR, CONTROL_SIGNAL)

    # =========================
    # 1) Spray + receive gate
    # =========================
    if ENABLE_SPRAY_CYCLE:
        rospy.loginfo("Spray ON for %.1f s", SPRAY_TIME)
        write_cmd(ser, "S1")
        rospy.sleep(SPRAY_TIME)
        write_cmd(ser, "S0")

        rospy.loginfo("Spray OFF. Waiting up to %.1f s (threshold_signal=%.1f).", WAIT_TIMEOUT, THRESHOLD_SIGNAL)
        start = rospy.Time.now()
        received = False
        while not rospy.is_shutdown() and not received:
            if (rospy.Time.now() - start).to_sec() > WAIT_TIMEOUT:
                rospy.logwarn("No reception within timeout.")
                return
            v = readline_sensor_value(ser)
            if v is None:
                continue
            sig = signal_from_value(v)
            rospy.loginfo("WAIT value=%.1f signal=%.1f", v, sig)
            if sig >= THRESHOLD_SIGNAL:
                received = True
                rospy.loginfo("RECEIVED.")
                break

    # =========================
    # 2) Settle then capture home target0
    # =========================
    if CAPTURE_DELAY_SEC > 0:
        rospy.loginfo("Settling %.1f s before target capture...", CAPTURE_DELAY_SEC)
        endt = rospy.Time.now() + rospy.Duration.from_sec(CAPTURE_DELAY_SEC)
        while rospy.Time.now() < endt and not rospy.is_shutdown():
            _ = readline_sensor_value(ser)  # drain
            rospy.sleep(0.01)

    rospy.loginfo("Capturing target0 for %.1f s...", CAPTURE_SEC)
    target0 = read_signal_window(ser, signal_from_value, CAPTURE_SEC, min_n=8)
    if target0 is None:
        rospy.logerr("Failed to capture target0 (not enough samples).")
        return

    t_home = rospy.Time.now()
    rospy.loginfo("BOOKMARKED target0=%.2f at t0=%.3f", target0, t_home.to_sec())

    # =========================
    # 3) Auto-calibrate direction: which way increases signal?
    # =========================
    # dir_inc = +1 means +linear.x increases signal; -1 means -linear.x increases signal
    dir_inc = 1

    if AUTO_DIR:
        stop(cmd_pub)
        s0 = read_signal_window(ser, signal_from_value, CALIB_MEASURE_SEC, min_n=5)
        if s0 is None:
            rospy.logwarn("Calibration skipped (no signal).")
        else:
            tw = Twist(); tw.linear.x = CALIB_SPEED
            publish(cmd_pub, tw, PUB_HZ, CALIB_TIME)
            stop(cmd_pub)
            rospy.sleep(CALIB_SETTLE)

            s1 = read_signal_window(ser, signal_from_value, CALIB_MEASURE_SEC, min_n=5)
            # return back to original spot (best effort)
            tw2 = Twist(); tw2.linear.x = -CALIB_SPEED
            publish(cmd_pub, tw2, PUB_HZ, CALIB_TIME)
            stop(cmd_pub)
            rospy.sleep(CALIB_SETTLE)

            if s1 is not None:
                delta = s1 - s0
                dir_inc = 1 if delta > 0 else -1
                rospy.loginfo("DIR CALIB: s0=%.1f s1=%.1f delta=%.1f => dir_inc=%+d", s0, s1, delta, dir_inc)
            else:
                rospy.logwarn("Calibration s1 missing; using default dir_inc=+1")

    # =========================
    # 4) HOLD with step control to avoid overshoot
    # =========================
    step_time = STEP_TIME
    prev_err = None
    hold_start = rospy.Time.now()

    while not rospy.is_shutdown():
        if HOLD_TIME > 0 and (rospy.Time.now() - hold_start).to_sec() > HOLD_TIME:
            rospy.loginfo("Hold time ended.")
            break

        # target(t) with decay
        dt = (rospy.Time.now() - t_home).to_sec()
        target_t = DECAY_FLOOR + (target0 - DECAY_FLOOR) * math.exp(-lam * max(0.0, dt))

        # measure while stopped
        stop(cmd_pub, n=1, dt=0.0)
        sig = read_signal_window(ser, signal_from_value, MEASURE_SEC, min_n=5)
        if sig is None:
            if STOP_ON_NO_SENSOR:
                stop(cmd_pub)
            rospy.sleep(0.05)
            continue

        err = target_t - sig
        rospy.loginfo("dt=%.1f target=%.1f sig=%.1f err=%.1f step_time=%.2f",
                      dt, target_t, sig, err, step_time)

        # within tolerance -> just hold still
        if abs(err) <= TOL:
            prev_err = err
            rospy.sleep(0.1)
            continue

        # if we overshot (sign flip), shrink step_time
        if prev_err is not None and (err == 0 or (prev_err > 0) != (err > 0)):
            step_time = max(STEP_TIME_MIN, step_time * OVERSHOOT_SHRINK)
            rospy.loginfo("Overshoot detected -> step_time shrunk to %.2f", step_time)
            # extra settle to let sensor catch up
            stop(cmd_pub)
            rospy.sleep(SETTLE_SEC)

        # if still far, gently grow step_time (fewer steps)
        if abs(err) > FAR_ERR:
            step_time = min(STEP_TIME_MAX, step_time * FAR_GROW)

        # decide motion direction:
        # if err > 0 we need higher signal => move in direction that increases signal (dir_inc)
        # if err < 0 we need lower signal  => move opposite
        direction = dir_inc if err > 0 else -dir_inc

        tw = Twist()
        tw.linear.x = direction * STEP_SPEED
        publish(cmd_pub, tw, PUB_HZ, step_time)
        stop(cmd_pub)
        rospy.sleep(SETTLE_SEC)

        prev_err = err

    stop(cmd_pub)
    rospy.loginfo("Stopped.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
