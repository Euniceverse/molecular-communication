#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
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
    """Expect Arduino lines like: 'R <float>'. Return float or None."""
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
    rospy.init_node("molecular_demo_homing_decay")

    # ---- Serial ----
    PORT      = p("port", "/dev/ttyUSB1")
    BAUD      = int(p("baud", 57600))
    USE_READY = bool(p("use_ready", False))

    # ---- Spray + receive gate ----
    ENABLE_SPRAY_CYCLE = bool(p("enable_spray_cycle", True))
    SPRAY_TIME         = float(p("spray_time", 5.0))
    WAIT_TIMEOUT       = float(p("wait_timeout", 40.0))
    THRESHOLD_SIGNAL   = float(p("threshold_signal", 250.0))  # threshold on chosen signal ("raw" or "diff")

    # ---- Baseline for diff-mode ----
    CONTROL_SIGNAL   = p("control_signal", "raw").lower()  # "raw" or "diff"
    PRIME_BASELINE_S = float(p("prime_baseline_sec", 3.0))
    BASELINE_ALPHA   = float(p("baseline_alpha", 0.05))

    # ---- Capture initial home value ----
    CAPTURE_SEC = float(p("capture_sec", 2.0))  # average this long to get target0

    # ---- Motion control ----
    CMD_TOPIC    = p("cmd_vel_topic", "/cmd_vel")
    KP           = float(p("kp", 0.002))
    MAX_SPEED    = float(p("max_speed", 0.06))
    TOL          = float(p("tolerance", 10.0))
    RATE_HZ      = float(p("rate_hz", 20.0))
    INVERT_CMD   = bool(p("invert_cmd", False))
    HOLD_TIME    = float(p("hold_time", 0.0))   # 0 = infinite

    # ---- Decay model (the key part) ----
    # target(t) = floor + (target0 - floor) * exp(-lambda * dt)
    DECAY_HALFLIFE_S = float(p("decay_halflife_sec", 60.0))  # intuitive tuning knob (seconds)
    DECAY_FLOOR      = float(p("decay_floor", 0.0))          # diff-mode usually 0. raw-mode maybe ambient reading.
    # Optional: lightly learn floor from observations when "near home"
    LEARN_FLOOR       = bool(p("learn_floor", False))
    FLOOR_ALPHA       = float(p("floor_alpha", 0.001))       # very slow
    FLOOR_UPDATE_BAND = float(p("floor_update_band", 3.0*TOL))

    # ---- Stability safety ----
    STOP_ON_NO_SENSOR = bool(p("stop_on_no_sensor", True))

    # Publisher
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

    # ---- Open serial ----
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

    # ---- Prime diff baseline background (ambient) ----
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

    def signal_from_value(value):
        if CONTROL_SIGNAL == "raw":
            return value
        # diff mode
        if baseline_bg is None:
            return 0.0
        return value - baseline_bg

    # ---- decay lambda ----
    if DECAY_HALFLIFE_S <= 0:
        lam = 0.0
    else:
        lam = math.log(2.0) / DECAY_HALFLIFE_S

    rospy.loginfo("Decay model: target(t)=floor+(target0-floor)*exp(-lam*dt), half-life=%.1fs lam=%.5f floor=%.2f",
                  DECAY_HALFLIFE_S, lam, DECAY_FLOOR)

    # =========================
    # 1) ACQUIRE: spray + wait + capture initial home target0
    # =========================
    if ENABLE_SPRAY_CYCLE:
        rospy.loginfo("Spray ON for %.1f s", SPRAY_TIME)
        write_cmd(ser, "S1")
        rospy.sleep(SPRAY_TIME)
        write_cmd(ser, "S0")
        rospy.loginfo("Spray OFF. Waiting up to %.1f s for reception (threshold_signal=%.1f, signal=%s).",
                      WAIT_TIMEOUT, THRESHOLD_SIGNAL, CONTROL_SIGNAL)

        start_wait = rospy.Time.now()
        received = False
        while not rospy.is_shutdown() and not received:
            if (rospy.Time.now() - start_wait).to_sec() > WAIT_TIMEOUT:
                rospy.logwarn("No reception within timeout. Exiting.")
                return

            value = readline_sensor_value(ser)
            if value is None:
                continue
            sig = signal_from_value(value)
            rospy.loginfo("WAIT value=%.1f signal=%.1f", value, sig)

            if sig >= THRESHOLD_SIGNAL:
                received = True
                rospy.loginfo("RECEIVED: gate triggered.")
                break

    rospy.loginfo("Capturing home target0 for %.1f s (averaging signal)...", CAPTURE_SEC)
    cap_start = rospy.Time.now()
    acc, n = 0.0, 0
    while not rospy.is_shutdown() and (rospy.Time.now() - cap_start).to_sec() < CAPTURE_SEC:
        value = readline_sensor_value(ser)
        if value is None:
            continue
        sig = signal_from_value(value)
        acc += sig
        n += 1

    if n == 0:
        rospy.logerr("Could not capture target0: no sensor readings.")
        return

    target0 = acc / n
    t0 = rospy.Time.now()  # "home time"
    rospy.loginfo("BOOKMARKED: target0=%.2f at t0=%s (signal=%s, n=%d)", target0, str(t0.to_sec()), CONTROL_SIGNAL, n)

    # =========================
    # 2) HOLD: always return to the decaying target(t)
    # =========================
    rate = rospy.Rate(RATE_HZ)
    hold_start = rospy.Time.now()

    while not rospy.is_shutdown():
        if HOLD_TIME > 0.0 and (rospy.Time.now() - hold_start).to_sec() > HOLD_TIME:
            rospy.loginfo("Hold time ended; stopping.")
            break

        now = rospy.Time.now()
        dt = (now - t0).to_sec()
        # predicted target at the original place, considering decay
        target_t = DECAY_FLOOR + (target0 - DECAY_FLOOR) * math.exp(-lam * max(0.0, dt))

        value = readline_sensor_value(ser)
        if value is None:
            if STOP_ON_NO_SENSOR:
                publish_stop(cmd_pub, n=1, dt=0.0)
            rate.sleep()
            continue

        sig = signal_from_value(value)
        error = target_t - sig

        # Optional: learn floor VERY slowly when near home band (helps raw-mode)
        if LEARN_FLOOR and abs(error) <= FLOOR_UPDATE_BAND:
            DECAY_FLOOR = (1.0 - FLOOR_ALPHA) * DECAY_FLOOR + FLOOR_ALPHA * sig

        # P control
        v = KP * error
        if INVERT_CMD:
            v = -v
        v = clamp(v, -MAX_SPEED, MAX_SPEED)

        if abs(error) <= TOL:
            publish_stop(cmd_pub, n=1, dt=0.0)
            v_out = 0.0
        else:
            tw = Twist()
            tw.linear.x = v
            cmd_pub.publish(tw)
            v_out = v

        rospy.loginfo("dt=%.1f target(t)=%.1f signal=%.1f err=%.1f cmd_v=%.3f floor=%.1f",
                      dt, target_t, sig, error, v_out, DECAY_FLOOR)

        rate.sleep()

    publish_stop(cmd_pub)
    rospy.loginfo("Stopped.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
