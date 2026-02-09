#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from collections import deque

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
def publish_stop(cmd_pub, tw_stop, repeats=12):
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

    # On resume: STREAM1 again (sampler thread will keep reading)
    if pub_state:
        pub_state.publish("STATE=RESUMED")
    try:
        write_cmd(ser, "STREAM1")
        rospy.sleep(0.1)
        flush_input(ser)
    except Exception:
        pass


# -------------------- Constant serial reader (publishes + buffers) --------------------
class SerialSampler:
    """
    A single thread reads from serial continuously:
      expects lines: 'R <float>'
    It:
      - publishes each parsed value to pub_raw
      - stores recent samples in a thread-safe buffer for windowed computations
    """
    def __init__(self, ser, pub_raw=None, maxlen=20000):
        self.ser = ser
        self.pub_raw = pub_raw

        self._lock = threading.Lock()
        self._buf = deque(maxlen=maxlen)   # (t_sec, v)

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

    def clear_buffer(self):
        with self._lock:
            self._buf.clear()

    def pop_all(self):
        """Return all buffered samples and clear the buffer."""
        with self._lock:
            items = list(self._buf)
            self._buf.clear()
        return items

    def _run(self):
        while self._running and not rospy.is_shutdown():
            try:
                line = self.ser.readline().decode("ascii", errors="ignore").strip()
            except Exception:
                continue

            if not line or not line.startswith("R "):
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
                self._buf.append((t, v))

            if self.pub_raw:
                try:
                    self.pub_raw.publish(Float32(v))
                except Exception:
                    pass


# -------------------- Experiment primitives --------------------
def spray_step(ser, spray_time, pub_state=None):
    if pub_state:
        pub_state.publish(f"STATE=SPRAY_ON t={float(spray_time):.1f}")
    write_cmd(ser, "S1")
    ok = sleep_while_running(spray_time, pub_state=pub_state)
    write_cmd(ser, "S0")
    if pub_state:
        pub_state.publish("STATE=SPRAY_OFF")
    return ok


def baseline_avg_for(sampler: SerialSampler, duration_s, pub_state=None, poll_hz=20.0):
    """
    Average over samples that arrive during this duration.
    Returns (avg, status) where status in {"OK","PAUSED","NO_DATA"}
    """
    if pub_state:
        pub_state.publish(f"STATE=BASELINE_AVG t={float(duration_s):.1f}")

    sampler.clear_buffer()
    t0 = rospy.Time.now().to_sec()

    s = 0.0
    n = 0

    rate = rospy.Rate(float(poll_hz))
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - t0) < float(duration_s):
        if not _run_event.is_set():
            return None, "PAUSED"

        for _, v in sampler.pop_all():
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
    sense_rate_hz=10.0,
    baseline=None,
):
    if pub_state:
        pub_state.publish(f"STATE=FIND_MIN t={float(duration_s):.1f} tail={float(tail_s):.1f}")

    if baseline is None:
        return None, None, None, "NO_BASELINE"

    start_seq, _, _ = sampler.snapshot()
    min_v = None

    tail_sum = 0.0
    tail_n = 0

    rate = rospy.Rate(float(sense_rate_hz))
    t0 = rospy.Time.now()

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            return None, None, None, "PAUSED"

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
        return None, None, None, "NO_DATA"

    drop = float(baseline) - float(min_v)
    threshold = max(0.0, drop) 

    tail_avg = (tail_sum / tail_n) if tail_n > 0 else None
    return threshold, min_v, tail_avg, "OK"

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

    # ---- Requested timing/behavior ----
    WAIT_100               = float(p("wait_100", 100.0))

    PRE_SPRAY_TIME         = float(p("pre_spray_time", 10.0))
    PRE_SET_REPEATS        = int(p("pre_set_repeats", 3))
    PRE_WAIT_290           = float(p("pre_wait_290", 290.0))

    BASELINE_20_CAL        = float(p("baseline_20_cal", 20.0))
    PEAK_300               = float(p("peak_300", 300.0))
    TAIL_20                = float(p("tail_20", 20.0))

    THRESH_FACTOR          = float(p("threshold_factor", 0.5))  # threshold = (baseline - peak_min) * factor
    DROP_THR_FALLBACK      = float(p("drop_threshold_fallback", 300.0))

    WAIT_20_AFTER_THRESH   = float(p("wait_20_after_thresh", 20.0))

    LOOP_WAIT_20           = float(p("loop_wait_20", 20.0))
    LOOP_SPRAY_10          = float(p("loop_spray_10", 10.0))
    LOOP_WAIT_290          = float(p("loop_wait_290", 290.0))

    LOOP_BASELINE_20       = float(p("loop_baseline_20", 20.0))
    LOOP_DETECT_300        = float(p("loop_detect_300", 300.0))

    WAIT_20_IF_DETECTED    = float(p("wait_20_if_detected", 20.0))

    # Sampling
    POLL_HZ                = float(p("poll_hz", 20.0))  # how often we pop the sampler buffer

    # Motion
    SPEED_LIN              = float(p("speed_linear", -0.04))
    SPEED_ANG              = float(p("speed_angular", 0.0))
    MOVE_TIME              = float(p("move_time", 2.0))
    PUB_RATE_HZ            = float(p("publish_rate", 20.0))

    # ROS I/O
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=10)
    pub_raw   = rospy.Publisher(RAW_TOPIC, Float32, queue_size=1000)
    pub_state = rospy.Publisher(STATE_TOPIC, String, queue_size=1000)
    cmd_pub   = rospy.Publisher(CMD_TOPIC, Twist, queue_size=10)

    rospy.loginfo("Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)

    # Twist
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

    # Arduino handshake (before sampler starts)
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

    # Start constant sampler
    sampler = SerialSampler(ser, pub_raw=pub_raw, maxlen=40000)
    sampler.start()

    # -------------------- PRE-PHASE --------------------
    # wait 100s
    rospy.loginfo("[Wait] 100s")
    pub_state.publish(f"STATE=WAIT_100 t={WAIT_100:.1f}")
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue
        if sleep_while_running(WAIT_100, pub_state=pub_state):
            break

    rospy.loginfo("[Spray] 10s, [Wait] 290s: repeat 3 times")
    # spray 10s + wait 290s, repeated 3 times as a set
    pub_state.publish(f"STATE=PRE_SPRAY_SET repeats={PRE_SET_REPEATS}")
    for i in range(int(PRE_SET_REPEATS)):
        if rospy.is_shutdown():
            sampler.stop()
            return
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
        
        rospy.loginfo("[Spray] 10s")
        pub_state.publish(f"STATE=PRE_SPRAY i={i+1}/{PRE_SET_REPEATS} t={PRE_SPRAY_TIME:.1f}")
        if not spray_step(ser, PRE_SPRAY_TIME, pub_state=pub_state):
            continue

        rospy.loginfo("[Wait] 290s")
        pub_state.publish(f"STATE=PRE_WAIT t={PRE_WAIT_290:.1f}")
        if not sleep_while_running(PRE_WAIT_290, pub_state=pub_state):
            continue

    # -------------------- CALIBRATION --------------------
    # baseline avg 20s
    rospy.loginfo("[Baseline] 20s")
    while not rospy.is_shutdown():
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue

        baseline, st = baseline_avg_for(
            sampler, BASELINE_20_CAL, pub_state=pub_state, poll_hz=POLL_HZ
        )
        if st == "PAUSED":
            continue
        if baseline is None:
            pub_state.publish(f"EVENT=BASELINE_NO_DATA status={st}")
            continue
        pub_state.publish(f"EVENT=BASELINE_OK value={baseline:.1f}")
        break
    rospy.loginfo("[Baseline] Set as %.1f", float(baseline))

    # find peak for 300s + tail baseline avg (last 20s)
    thresholds = []
    min_values = []
    tail_avgs = []

    for i in range(CAL_SET_REPEATS):
        rospy.loginfo("[Find peak] 300s / [Find baseline] last 20s of 300s")
        flush_input(ser)

        threshold, min_v, tail_avg, st = find_min_with_tail_baseline_avg(
            sampler,
            PEAK_300,
            tail_s=TAIL_20,
            pub_state=pub_state,
            sense_rate_hz=SAMPLE_HZ,
            baseline=baseline0
        )

        rospy.loginfo(
            "[Find peak %d] st=%s threshold=%.1f min_v=%.1f tail_avg=%.1f",
            i+1,
            st,
            float(threshold) if threshold is not None else float('nan'),
            float(min_v) if min_v is not None else float('nan'),
            float(tail_avg) if tail_avg is not None else float('nan'),
        )

        if st == "OK" and threshold is not None:
            thresholds.append(float(threshold))
            min_values.append(float(min_v))
            if tail_avg is not None:
                tail_avgs.append(float(tail_avg))

    # Decide drop_threshold
    if baseline0 is None or len(thresholds) == 0:
        drop_threshold = DROP_THR_FALLBACK
        pub_state.publish(f"STATE=DROP_THR_FALLBACK dthr={drop_threshold:.1f} reason=NO_DATA")
    else:
        threshold_avg = sum(thresholds) / float(len(thresholds))   # avg drop (baseline - min)
        drop_threshold = threshold_avg * float(THRESH_FACTOR)      # your 0.5 factor

        avg_min = sum(min_values) / float(len(min_values))

        pub_state.publish(
            f"STATE=DROP_THR_OK baseline0={float(baseline0):.1f} "
            f"avg_min={avg_min:.1f} avg_drop={threshold_avg:.1f} "
            f"factor={float(THRESH_FACTOR):.2f} dthr={float(drop_threshold):.1f}"
        )

    rospy.loginfo("[Find threshold] threshold=%.1f", float(drop_threshold))
    rospy.loginfo("[Calibration done]")

    # -------------------- MAIN LOOP --------------------
    pub_state.publish("STATE=MAIN_LOOP")
    rospy.loginfo("Entering main loop...")

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue

        # 1) wait 20s
        rospy.loginfo("[Wait] 20s")
        pub_state.publish(f"STATE=LOOP_WAIT_20 t={LOOP_WAIT_20:.1f}")
        if not sleep_while_running(LOOP_WAIT_20, pub_state=pub_state):
            continue
        rospy.loginfo("[Wait] Done")

        # 2) spray 10s once + wait 290s
        rospy.loginfo("[Spray] 10s")
        pub_state.publish(f"STATE=LOOP_SPRAY t={LOOP_SPRAY_10:.1f}")
        if not spray_step(ser, LOOP_SPRAY_10, pub_state=pub_state):
            continue

        rospy.loginfo("[Wait] 290s")
        pub_state.publish(f"STATE=LOOP_POST_SPRAY_WAIT t={LOOP_WAIT_290:.1f}")
        if not sleep_while_running(LOOP_WAIT_290, pub_state=pub_state):
            continue

        # 3) capture baseline avg for 20s
        rospy.loginfo("[Baseline] 20s")
        baseline, st = baseline_avg_for(
            sampler, LOOP_BASELINE_20, pub_state=pub_state, poll_hz=POLL_HZ
        )
        if st == "PAUSED":
            continue
        if baseline is None:
            pub_state.publish(f"EVENT=LOOP_BASELINE_NO_DATA status={st}")
            continue
        pub_state.publish(f"EVENT=LOOP_BASELINE_OK value={baseline:.1f}")
        rospy.loginfo("[Baseline] Set as %.1f", float(baseline))

        # 4) detect for 300s: exists v where (baseline - v) > drop_threshold
        rospy.loginfo("[Detect] 300s")
        flush_input(ser)
        res = detect_drop_for(
            sampler, LOOP_DETECT_300,
            baseline=baseline,
            drop_threshold=drop_threshold,
            pub_state=pub_state,
            poll_hz=POLL_HZ
        )
        if res is None:
            continue  # paused
        pub_state.publish(f"EVENT=DETECT_RESULT detected={res}")
        rospy.loginfo("[Detect] Done.")

        # 5) if detected: move, then go back to loop wait 20s
        if res:
            pub_state.publish(f"STATE=MOVE t={MOVE_TIME:.1f}")
            rospy.loginfo("[Move] %.1f seconds", MOVE_TIME)
            move_step(cmd_pub, tw_go, tw_stop, MOVE_TIME, PUB_RATE_HZ, pub_state=pub_state)

    sampler.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
