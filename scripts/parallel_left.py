#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import rospy
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


# -------------------- Param helper --------------------
def p(name, default):
    return rospy.get_param("~" + name, default)


# -------------------- Serial helpers --------------------
def write_cmd(ser, s: str):
    if not s.endswith("\n"):
        s += "\n"
    ser.write(s.encode("ascii", errors="ignore"))


def flush_input(ser):
    try:
        ser.reset_input_buffer()
    except Exception:
        pass


def wait_for_ready(ser, timeout=5.0):
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(timeout):
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            line = ""
        if line == "READY":
            return True
    return False


# -------------------- RUN/PAUSE control --------------------
_run_event = threading.Event()  # set()=running, clear()=paused


def control_cb(msg: String):
    cmd = msg.data.strip().lower()
    if cmd in ("start", "run", "go"):
        _run_event.set()
        rospy.loginfo("CONTROL: START")
    elif cmd in ("stop", "pause"):
        _run_event.clear()
        rospy.logwarn("CONTROL: STOP")
    else:
        rospy.logwarn("CONTROL: unknown '%s' (use 'start' or 'stop')", msg.data)


def wait_until_running(pub_state=None):
    if pub_state:
        pub_state.publish("STATE=WAIT_START")
    rospy.loginfo("Waiting for START... (publish 'start' to control topic)")
    while not rospy.is_shutdown() and not _run_event.is_set():
        rospy.sleep(0.1)


def sleep_while_running(duration_s, pub_state=None):
    """
    Sleep up to duration_s but return False early if STOP happens.
    True  = slept full duration
    False = stopped/paused during sleep
    """
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            if pub_state:
                pub_state.publish("STATE=PAUSED_DURING_SLEEP")
            return False
        if (rospy.Time.now() - t0).to_sec() >= float(duration_s):
            return True
        rospy.sleep(0.02)


# -------------------- Robot stop helpers --------------------
def publish_stop(cmd_pub, tw_stop, repeats=10):
    for _ in range(int(repeats)):
        cmd_pub.publish(tw_stop)
        rospy.sleep(0.02)


def emergency_stop(ser, cmd_pub, tw_stop, pub_state=None, stop_stream=True):
    # TX OFF + robot stop (+ optionally STREAM0)
    try:
        write_cmd(ser, "S0")
        if stop_stream:
            write_cmd(ser, "STREAM0")
    except Exception:
        pass
    publish_stop(cmd_pub, tw_stop, repeats=12)
    if pub_state:
        pub_state.publish("STATE=EMERGENCY_STOP")


def handle_pause(ser, cmd_pub, tw_stop, pub_state):
    # When STOP: TX OFF + STREAM0 + robot stop, then wait for START
    if pub_state:
        pub_state.publish("STATE=PAUSED")
    emergency_stop(ser, cmd_pub, tw_stop, pub_state=pub_state, stop_stream=True)

    wait_until_running(pub_state=pub_state)

    # On resume: STREAM1 + flush
    if pub_state:
        pub_state.publish("STATE=RESUMED")
    try:
        write_cmd(ser, "STREAM1")
        rospy.sleep(0.1)
        flush_input(ser)
    except Exception:
        pass


# -------------------- Serial sampler thread (constant publish) --------------------
class SerialSampler:
    """
    Only this thread reads from ser. It continuously parses lines like:
      'R <number>'
    It publishes to pub_raw (constant), and stores the latest sample (seq, v, t).
    """
    def __init__(self, ser, pub_raw=None):
        self.ser = ser
        self.pub_raw = pub_raw

        self._lock = threading.Lock()
        self._seq = 0
        self._last_v = None
        self._last_t = 0.0

        self._running = True
        self._th = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._th.start()

    def stop(self):
        self._running = False
        try:
            self._th.join(timeout=1.0)
        except Exception:
            pass

    def snapshot(self):
        with self._lock:
            return self._seq, self._last_v, self._last_t

    def _run(self):
        while self._running and not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except Exception:
                continue

            if not line:
                continue

            if not line.startswith("R "):
                continue

            parts = line.split()
            if len(parts) < 2:
                continue

            try:
                v = float(parts[1])
            except Exception:
                continue

            t = rospy.Time.now().to_sec()

            with self._lock:
                self._seq += 1
                self._last_v = v
                self._last_t = t

            if self.pub_raw:
                try:
                    self.pub_raw.publish(Float32(v))
                except Exception:
                    pass


# -------------------- Experiment primitives --------------------
def spray_step(ser, spray_time, pub_state=None):
    if pub_state:
        pub_state.publish(f"STATE=SPRAY_ON t={spray_time:.1f}")
    write_cmd(ser, "S1")
    ok = sleep_while_running(spray_time, pub_state=pub_state)
    write_cmd(ser, "S0")
    if pub_state:
        pub_state.publish("STATE=SPRAY_OFF")
    return ok


def baseline_avg_for(sampler: SerialSampler, duration_s, pub_state=None, sense_rate_hz=10.0):
    """
    Average over NEW samples arriving during this duration.
    Returns (avg, status) where status in {"OK","PAUSED","NO_DATA"}
    """
    if pub_state:
        pub_state.publish(f"STATE=BASELINE_AVG t={float(duration_s):.1f}")

    start_seq, _, _ = sampler.snapshot()
    s = 0.0
    n = 0

    rate = rospy.Rate(float(sense_rate_hz))
    t0 = rospy.Time.now()

    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(duration_s):
        if not _run_event.is_set():
            return None, "PAUSED"

        seq, v, _ = sampler.snapshot()
        if v is not None and seq > start_seq:
            # consume each new sample once by advancing start_seq
            start_seq = seq
            s += float(v)
            n += 1

        rate.sleep()

    if n == 0:
        return None, "NO_DATA"
    return (s / n), "OK"


def find_min_with_tail_baseline_avg(
    sampler: SerialSampler,
    duration_s,
    tail_s=20.0,
    pub_state=None,
    sense_rate_hz=10.0
):
    """
    Over duration_s:
      - peak (for drop sensor) = minimum value
      - tail baseline avg = avg of values in last tail_s seconds of the window
    Returns (min_v, tail_avg, status) where status in {"OK","PAUSED","NO_DATA"}
    """
    if pub_state:
        pub_state.publish(f"STATE=FIND_MIN t={float(duration_s):.1f} tail={float(tail_s):.1f}")

    start_seq, _, _ = sampler.snapshot()
    min_v = None

    tail_sum = 0.0
    tail_n = 0

    rate = rospy.Rate(float(sense_rate_hz))
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            return None, None, "PAUSED"

        elapsed = (rospy.Time.now() - t0).to_sec()
        if elapsed >= float(duration_s):
            break

        seq, v, _ = sampler.snapshot()
        if v is not None and seq > start_seq:
            start_seq = seq

            if min_v is None or v < min_v:
                min_v = v

            if elapsed >= float(duration_s) - float(tail_s):
                tail_sum += float(v)
                tail_n += 1

        rate.sleep()

    if min_v is None:
        return None, None, "NO_DATA"

    tail_avg = (tail_sum / tail_n) if tail_n > 0 else None
    return min_v, tail_avg, "OK"


def detect_drop_for(
    sampler: SerialSampler,
    duration_s,
    baseline,
    drop_threshold,
    pub_state=None,
    sense_rate_hz=10.0
):
    """
    Detect if there exists any sample v such that:
      (baseline - v) > drop_threshold

    Returns:
      True  = detected within duration
      False = not detected
      None  = paused/stopped during detection
    """
    result = False
    if pub_state:
        pub_state.publish(
            f"STATE=DETECT_DROP t={float(duration_s):.1f} baseline={float(baseline):.1f} dthr={float(drop_threshold):.1f}"
        )

    start_seq, _, _ = sampler.snapshot()
    rate = rospy.Rate(float(sense_rate_hz))
    t0 = rospy.Time.now()

    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(duration_s):
        if not _run_event.is_set():
            return None

        seq, v, _ = sampler.snapshot()
        if v is not None and seq > start_seq:
            start_seq = seq

            drop = float(baseline) - float(v)
            if drop > float(drop_threshold) and result is False:
                if pub_state:
                    pub_state.publish(f"EVENT=DROP_FOUND v={float(v):.1f} drop={drop:.1f}")
                rospy.loginfo("DROP_FOUND: v=%.1f drop=%.1f", float(v), float(drop))
                result = True
        rate.sleep()

    return result


def move_step(cmd_pub, tw_go, tw_stop, move_time, pub_rate_hz, pub_state=None):
    if pub_state:
        pub_state.publish(f"STATE=MOVE t={float(move_time):.1f}")
    rate = rospy.Rate(float(pub_rate_hz))
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < float(move_time):
        if not _run_event.is_set():
            break
        cmd_pub.publish(tw_go)
        rate.sleep()
    publish_stop(cmd_pub, tw_stop, repeats=12)
    if pub_state:
        pub_state.publish("STATE=MOVE_DONE")


# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_tx")

    # Serial params
    PORT      = p("port", "/dev/ttyUSB0")
    BAUD      = int(p("baud", 9600))
    USE_READY = bool(p("use_ready", True))

    # Topics
    CONTROL_TOPIC = p("control_topic", "/molecular_demo/control")
    RAW_TOPIC     = p("raw_topic",   "/molecular/tx/raw")
    STATE_TOPIC   = p("state_topic", "/molecular/tx/state")
    CMD_TOPIC     = p("cmd_vel_topic", "/cmd_vel")

    # Timing (your requested behavior)
    WAIT_80                 = float(p("wait_80", 80.0))
    BASELINE_20_CAL         = float(p("baseline_20_cal", 20.0))
    PEAK_300                = float(p("peak_300", 300.0))
    TAIL_20                 = float(p("tail_20", 20.0))
    WAIT_20_AFTER_THRESH    = float(p("wait_20_after_thresh", 20.0))

    CAL_SET_REPEATS         = int(p("cal_set_repeats", 3))   # "3 times as a set"
    CAL_SPRAY_10            = float(p("cal_spray_10", 10.0))
    CAL_WAIT_290            = float(p("cal_wait_290", 290.0))

    BASELINE_20_MAIN        = float(p("baseline_20_main", 20.0))
    DETECT_300_MAIN         = float(p("detect_300_main", 300.0))
    WAIT_20_BEFORE_ACTION   = float(p("wait_20_before_action", 20.0))

    ACTION_SPRAY_10         = float(p("action_spray_10", 10.0))
    ACTION_WAIT_290         = float(p("action_wait_290", 290.0))

    # Threshold rule you specified: (baseline - peak_avg) * 0.5
    THRESH_FACTOR           = float(p("threshold_factor", 0.5))
    DROP_THR_FALLBACK       = float(p("drop_threshold_fallback", 300.0))

    # Sampling
    SAMPLE_HZ               = float(p("sample_hz", 10.0))

    # Motion
    SPEED_LIN               = float(p("speed_linear", -0.04))
    SPEED_ANG               = float(p("speed_angular", 0.0))
    MOVE_TIME               = float(p("move_time", 2.0))
    PUB_RATE_HZ             = float(p("publish_rate", 20.0))

    # ROS I/O
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=10)
    pub_raw   = rospy.Publisher(RAW_TOPIC, Float32, queue_size=1000)
    pub_state = rospy.Publisher(STATE_TOPIC, String, queue_size=1000)
    cmd_pub   = rospy.Publisher(CMD_TOPIC, Twist, queue_size=10)

    rospy.loginfo("Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)

    # Twist messages
    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG
    tw_stop = Twist()

    # Start paused
    _run_event.clear()
    wait_until_running(pub_state=pub_state)

    # Serial open
    pub_state.publish(f"STATE=SERIAL_OPEN port={PORT} baud={BAUD}")
    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    rospy.on_shutdown(lambda: emergency_stop(ser, cmd_pub, tw_stop, pub_state=pub_state, stop_stream=True))

    # Handshake (before starting sampler thread)
    pub_state.publish("STATE=ARDUINO_HANDSHAKE")
    rospy.loginfo("Waiting for Arduino handshake...")
    if USE_READY:
        if not wait_for_ready(ser, 5.0):
            rospy.logwarn("No 'READY' seen, falling back to fixed delay")
            rospy.sleep(2.5)
    else:
        rospy.sleep(2.5)

    # Start streaming
    pub_state.publish("STATE=STREAM_ON")
    write_cmd(ser, "STREAM1")
    rospy.sleep(0.1)
    flush_input(ser)

    # Start constant serial reader/publisher
    sampler = SerialSampler(ser, pub_raw=pub_raw)
    sampler.start()

    # -------------------- CALIBRATION (once) --------------------
    rospy.loginfo("[[Start Calibration]]")
    pub_state.publish("STATE=CAL_BEGIN")

    # Wait 80s
    rospy.loginfo("[Wait] 80s")
    pub_state.publish(f"STATE=WAIT_80 t={WAIT_80:.1f}")
    if not sleep_while_running(WAIT_80, pub_state=pub_state):
        handle_pause(ser, cmd_pub, tw_stop, pub_state)
    rospy.loginfo("[Wait] Done")

    # Baseline avg 20s
    rospy.loginfo("[Baseline] 20s")
    flush_input(ser)
    baseline0, st = baseline_avg_for(sampler, BASELINE_20_CAL, pub_state=pub_state, sense_rate_hz=SAMPLE_HZ)
    if st != "OK" or baseline0 is None:
        pub_state.publish(f"EVENT=BASELINE0_FAIL status={st}")
        baseline0 = None
    else:
        pub_state.publish(f"EVENT=BASELINE0_OK value={baseline0:.1f}")
    rospy.loginfo("[Baseline] Set as %.1f", float(baseline0))

    # Find peak (lowest) for 300s + tail baseline avg over last 20s
    rospy.loginfo("[Find peak] 300s / [Find baseline] last 20s of 300s")
    flush_input(ser)
    min_v, tail_avg, st = find_min_with_tail_baseline_avg(
        sampler, PEAK_300, tail_s=TAIL_20, pub_state=pub_state, sense_rate_hz=SAMPLE_HZ
    )

    rospy.loginfo("[Find peak] Done")

    if st != "OK" or min_v is None or baseline0 is None:
        drop_threshold = DROP_THR_FALLBACK
        pub_state.publish(f"STATE=DROP_THR_FALLBACK dthr={drop_threshold:.1f} reason=NO_DATA")
    else:
        # Your rule: (baseline - peak_avg) * 0.5
        peak_avg = float(min_v)  # single-window "peak_avg" (lowest)
        raw_drop = float(baseline0) - float(peak_avg)
        drop_threshold = max(0.0, raw_drop * float(THRESH_FACTOR))

        pub_state.publish(f"EVENT=PEAK_MIN_OK value={float(min_v):.1f}")
        if tail_avg is not None:
            pub_state.publish(f"EVENT=TAIL_BASELINE_AVG value={float(tail_avg):.1f}")

        pub_state.publish(
            f"STATE=DROP_THR_OK baseline0={float(baseline0):.1f} peak_avg={float(peak_avg):.1f} "
            f"factor={float(THRESH_FACTOR):.2f} dthr={float(drop_threshold):.1f}"
        )

    rospy.loginfo("[Find threshold] threshold=%.1f", float(drop_threshold))

    # Wait 20s
    rospy.loginfo("[Wait] 20s")
    pub_state.publish(f"STATE=WAIT_20_AFTER_THRESH t={WAIT_20_AFTER_THRESH:.1f}")
    if not sleep_while_running(WAIT_20_AFTER_THRESH, pub_state=pub_state):
        handle_pause(ser, cmd_pub, tw_stop, pub_state)
    rospy.loginfo("[Wait] Done")

    # Spray 10s once + wait 290s, repeat 3 times
    rospy.loginfo("[Spray] 10s, [Wait] 290s: repeat 3 times")
    pub_state.publish(f"STATE=CAL_SPRAY_SET repeats={CAL_SET_REPEATS}")
    for i in range(int(CAL_SET_REPEATS)):
        if rospy.is_shutdown():
            return
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)

        pub_state.publish(f"STATE=CAL_SET_SPRAY i={i+1}/{CAL_SET_REPEATS}")
        rospy.loginfo("[Spray] 10s")
        if not spray_step(ser, CAL_SPRAY_10, pub_state=pub_state):
            continue

        rospy.loginfo("[Wait] 290s")
        pub_state.publish(f"STATE=CAL_SET_WAIT t={CAL_WAIT_290:.1f}")
        if not sleep_while_running(CAL_WAIT_290, pub_state=pub_state):
            continue

    rospy.loginfo("[[Calibration Done]]")
    pub_state.publish("STATE=CAL_DONE")

    # -------------------- MAIN LOOP --------------------
    pub_state.publish("STATE=MAIN_LOOP")
    rospy.loginfo("Entering main loop...")

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue

        # 1) Baseline avg 20s
        rospy.loginfo("[Baseline] 20s")
        flush_input(ser)
        baseline, st = baseline_avg_for(sampler, BASELINE_20_MAIN, pub_state=pub_state, sense_rate_hz=SAMPLE_HZ)
        if st != "OK" or baseline is None:
            pub_state.publish(f"EVENT=BASELINE_FAIL status={st}")
            continue
        pub_state.publish(f"EVENT=BASELINE_OK value={baseline:.1f}")

        # 2) Detect for 300s: exists v where (baseline - v) > drop_threshold
        rospy.loginfo("[Detect] 300s")
        flush_input(ser)
        res = detect_drop_for(
            sampler, DETECT_300_MAIN,
            baseline=baseline,
            drop_threshold=drop_threshold,
            pub_state=pub_state,
            sense_rate_hz=SAMPLE_HZ
        )
        if res is None:
            # paused
            continue
        pub_state.publish(f"EVENT=DETECT_RESULT detected={res}")
        rospy.loginfo("[Detect] Done.")

        # 3) Wait 20s
        rospy.loginfo("[Wait] 20s")
        pub_state.publish(f"STATE=WAIT_20_BEFORE_ACTION t={WAIT_20_BEFORE_ACTION:.1f}")
        if not sleep_while_running(WAIT_20_BEFORE_ACTION, pub_state=pub_state):
            continue
        rospy.loginfo("[Wait] Done")

        # 4) If detected: spray 10s + wait 290s, then move; else go back to baseline
        if res:
            pub_state.publish("STATE=ACTION_DETECTED")
            
            rospy.loginfo("[Spray] 10s, Detected")
            # Spray
            if not spray_step(ser, ACTION_SPRAY_10, pub_state=pub_state):
                continue
            
            rospy.loginfo("[Wait] 290s")
            # Wait
            pub_state.publish(f"STATE=ACTION_WAIT t={ACTION_WAIT_290:.1f}")
            if not sleep_while_running(ACTION_WAIT_290, pub_state=pub_state):
                continue

            rospy.loginfo("[Move] %.1f seconds", MOVE_TIME)
            # Move
            move_step(cmd_pub, tw_go, tw_stop, MOVE_TIME, PUB_RATE_HZ, pub_state=pub_state)

        else:
            pub_state.publish("STATE=ACTION_NOT_DETECTED")

    # shutdown cleanup
    sampler.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass