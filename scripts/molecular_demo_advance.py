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

def robust_mean(samples):
    """Trim 20% tails then mean (more robust than plain mean)."""
    if not samples:
        return None
    s = sorted(samples)
    k = int(0.2 * len(s))
    core = s[k:len(s)-k] if len(s) - 2*k >= 3 else s
    return sum(core) / float(len(core))

def main():
    rospy.init_node("molecular_demo_homing_decay_v2")

    # ---- Serial ----
    PORT      = p("port", "/dev/ttyUSB1")
    BAUD      = int(p("baud", 57600))
    USE_READY = bool(p("use_ready", False))

    # ---- Spray + receive gate ----
    ENABLE_SPRAY_CYCLE = bool(p("enable_spray_cycle", True))
    SPRAY_TIME         = float(p("spray_time", 5.0))
    WAIT_TIMEOUT       = float(p("wait_timeout", 40.0))
    THRESHOLD_SIGNAL   = float(p("threshold_signal", 250.0))  # threshold on chosen signal

    # ---- Signal type ----
    CONTROL_SIGNAL   = p("control_signal", "raw").lower()  # "raw" or "diff"
    PRIME_BASELINE_S = float(p("prime_baseline_sec", 3.0))
    BASELINE_ALPHA   = float(p("baseline_alpha", 0.05))

    # ---- IMPORTANT: capture timing ----
    CAPTURE_DELAY_SEC = float(p("capture_delay_sec", 2.0))  # wait after reception before capturing target0
    CAPTURE_SEC       = float(p("capture_sec", 2.0))        # capture window length

    # ---- Smoothing (reduces flip-flop) ----
    SIGNAL_EMA_ALPHA  = float(p("signal_ema_alpha", 0.25))  # 0.1..0.3 typical

    # ---- Motion control ----
    CMD_TOPIC    = p("cmd_vel_topic", "/cmd_vel")
    KP           = float(p("kp", 0.0003))       # smaller than before
    MAX_SPEED    = float(p("max_speed", 0.02))  # smaller than before for stability
    TOL          = float(p("tolerance", 12.0))
    RATE_HZ      = float(p("rate_hz", 20.0))
    INVERT_CMD   = bool(p("invert_cmd", False))

    # Optional: limit how fast cmd changes (slew rate)
    CMD_SLEW_PER_S = float(p("cmd_slew_per_s", 0.10))  # m/s per second, 0 to disable

    # ---- Decay model ----
    DECAY_HALFLIFE_S = float(p("decay_halflife_sec", 180.0))  # make slower by default
    DECAY_FLOOR      = float(p("decay_floor", 0.0))          # for diff-mode use 0. for raw-mode set ambient.
    HOLD_TIME        = float(p("hold_time", 0.0))            # 0=infinite

    STOP_ON_NO_SENSOR = bool(p("stop_on_no_sensor", True))

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

    # ---- Prime baseline for diff ----
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
        if baseline_bg is None:
            return 0.0
        return value - baseline_bg

    # ---- Decay lambda ----
    lam = 0.0 if DECAY_HALFLIFE_S <= 0 else math.log(2.0) / DECAY_HALFLIFE_S
    rospy.loginfo("Decay: half-life=%.1fs lam=%.5f floor=%.2f signal=%s", DECAY_HALFLIFE_S, lam, DECAY_FLOOR, CONTROL_SIGNAL)

    # =========================
    # 1) ACQUIRE reception
    # =========================
    if ENABLE_SPRAY_CYCLE:
        rospy.loginfo("Spray ON for %.1f s", SPRAY_TIME)
        write_cmd(ser, "S1")
        rospy.sleep(SPRAY_TIME)
        write_cmd(ser, "S0")

        rospy.loginfo("Spray OFF. Waiting up to %.1f s (threshold_signal=%.1f).", WAIT_TIMEOUT, THRESHOLD_SIGNAL)
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
                rospy.loginfo("RECEIVED.")
                break

    # =========================
    # 2) IMPORTANT: let sensor/plume settle, then capture target0 robustly
    # =========================
    if CAPTURE_DELAY_SEC > 0.0:
        rospy.loginfo("Settling for %.1f s before target capture (reduces rising-transient flips)...", CAPTURE_DELAY_SEC)
        end = rospy.Time.now() + rospy.Duration.from_sec(CAPTURE_DELAY_SEC)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            _ = readline_sensor_value(ser)  # drain buffer
            rospy.sleep(0.01)

    rospy.loginfo("Capturing home target0 for %.1f s...", CAPTURE_SEC)
    cap_start = rospy.Time.now()
    samples = []
    while not rospy.is_shutdown() and (rospy.Time.now() - cap_start).to_sec() < CAPTURE_SEC:
        value = readline_sensor_value(ser)
        if value is None:
            continue
        samples.append(signal_from_value(value))

    target0 = robust_mean(samples)
    if target0 is None:
        rospy.logerr("No samples to capture target0.")
        return

    t0 = rospy.Time.now()
    rospy.loginfo("BOOKMARKED target0=%.2f at t0=%.3f (n=%d)", target0, t0.to_sec(), len(samples))

    # =========================
    # 3) HOLD: chase decaying target(t) using smoothed signal
    # =========================
    rate = rospy.Rate(RATE_HZ)
    hold_start = rospy.Time.now()

    sig_ema = None
    last_v = 0.0

    while not rospy.is_shutdown():
        if HOLD_TIME > 0.0 and (rospy.Time.now() - hold_start).to_sec() > HOLD_TIME:
            rospy.loginfo("Hold time ended; stopping.")
            break

        now = rospy.Time.now()
        dt = (now - t0).to_sec()
        target_t = DECAY_FLOOR + (target0 - DECAY_FLOOR) * math.exp(-lam * max(0.0, dt))

        value = readline_sensor_value(ser)
        if value is None:
            if STOP_ON_NO_SENSOR:
                publish_stop(cmd_pub, n=1, dt=0.0)
            rate.sleep()
            continue

        sig = signal_from_value(value)
        sig_ema = sig if sig_ema is None else (1.0 - SIGNAL_EMA_ALPHA) * sig_ema + SIGNAL_EMA_ALPHA * sig

        error = target_t - sig_ema

        # P control
        v = KP * error
        if INVERT_CMD:
            v = -v
        v = clamp(v, -MAX_SPEED, MAX_SPEED)

        # Slew limit (optional)
        if CMD_SLEW_PER_S > 0.0:
            dv_max = CMD_SLEW_PER_S / float(RATE_HZ)
            v = clamp(v, last_v - dv_max, last_v + dv_max)

        if abs(error) <= TOL:
            publish_stop(cmd_pub, n=1, dt=0.0)
            v_out = 0.0
        else:
            tw = Twist()
            tw.linear.x = v
            cmd_pub.publish(tw)
            v_out = v

        rospy.loginfo("dt=%.1f target=%.1f sig(raw)=%.1f sig(ema)=%.1f err=%.1f cmd_v=%.3f",
                      dt, target_t, sig, sig_ema, error, v_out)

        last_v = v
        rate.sleep()

    publish_stop(cmd_pub)
    rospy.loginfo("Stopped.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
