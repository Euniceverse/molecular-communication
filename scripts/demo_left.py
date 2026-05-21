#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
from collections import deque
import time

import rospy
import serial

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


# -------------------- Param helper --------------------
def p(name, default):
    return rospy.get_param("~" + name, default)


# -------------------- Wall-clock time helpers --------------------
def wall_now():
    return time.monotonic()


def wall_sleep(dt):
    # wall-clock sleep (NOT ROS time)
    rospy.rostime.wallsleep(dt)


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
    # wall-clock timeout
    t0 = wall_now()
    while not rospy.is_shutdown() and (wall_now() - t0) < float(timeout):
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            line = ""

        if line == "READY":
            return True

    return False


# -------------------- RUN/PAUSE control --------------------
_run_event = threading.Event()  # set() = running, clear() = paused


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
        wall_sleep(0.1)


def sleep_while_running(duration_s, pub_state=None):
    """
    Wall-clock sleep up to duration_s but return False early if STOP happens.

    True  = slept full duration
    False = stopped/paused during sleep
    """
    t0 = wall_now()

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            if pub_state:
                pub_state.publish("STATE=PAUSED_DURING_SLEEP")
            return False

        if (wall_now() - t0) >= float(duration_s):
            return True

        wall_sleep(0.02)


# -------------------- Robot stop helpers --------------------
def publish_stop(cmd_pub, tw_stop, repeats=12):
    for _ in range(int(repeats)):
        cmd_pub.publish(tw_stop)
        wall_sleep(0.02)


def emergency_stop(ser, cmd_pub, tw_stop, pub_state=None, stop_stream=True):
    # TX OFF + robot stop + optionally STREAM0
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

    # On resume: STREAM1 again
    if pub_state:
        pub_state.publish("STATE=RESUMED")

    try:
        write_cmd(ser, "STREAM1")
        wall_sleep(0.1)
        flush_input(ser)
    except Exception:
        pass


# -------------------- Constant serial reader --------------------
class SerialSampler:
    """
    Continuously reads serial lines of the form:
        R <float>

    It publishes each parsed value and stores recent samples.
    """
    def __init__(self, ser, pub_raw=None, maxlen=20000):
        self.ser = ser
        self.pub_raw = pub_raw

        self._lock = threading.Lock()
        self._buf = deque(maxlen=maxlen)

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


def move_step(cmd_pub, tw_go, tw_stop, move_time, pub_rate_hz, pub_state=None):
    if pub_state:
        pub_state.publish(f"STATE=MOVE t={float(move_time):.1f}")

    period = 1.0 / float(pub_rate_hz) if pub_rate_hz > 0 else 0.05
    t0 = wall_now()

    while not rospy.is_shutdown() and (wall_now() - t0) < float(move_time):
        if not _run_event.is_set():
            break

        cmd_pub.publish(tw_go)
        wall_sleep(period)

    publish_stop(cmd_pub, tw_stop, repeats=12)

    if pub_state:
        pub_state.publish("STATE=MOVE_DONE")


# -------------------- Main --------------------
def main():
    rospy.init_node("molecular_demo_tx_left")

    # Serial params
    PORT = p("port", "/dev/ttyUSB0")
    BAUD = int(p("baud", 9600))
    USE_READY = bool(p("use_ready", True))

    # Topics
    CONTROL_TOPIC = p("control_topic", "/molecular_demo/control")
    RAW_TOPIC = p("raw_topic", "/molecular/tx/raw")
    STATE_TOPIC = p("state_topic", "/molecular/tx/state")
    CMD_TOPIC = p("cmd_vel_topic", "/cmd_vel")

    # Calibration timing
    PRE_SPRAY_TIME = float(p("pre_spray_time", 5.0))
    PRE_SET_REPEATS = int(p("pre_set_repeats", 3))
    PRE_WAIT_290 = float(p("pre_wait_290", 30.0))

    # Left waits before calibration while right calibrates and waits 30 s after right's last calibration pulse
    WAIT_BEFORE_CALIBRATION = float(p("wait_before_calibration", 105.0))

    # Left waits 30 s after its last calibration pulse before entering main loop
    WAIT_AFTER_LEFT_CALIBRATION = float(p("wait_after_left_calibration", 30.0))

    # Main-loop timing
    LOOP_SPRAY_10 = float(p("loop_spray_10", 5.0))
    LOOP_WAIT_290 = float(p("loop_wait_290", 30.0))

    # Sampling
    SAMPLE_HZ = float(p("sample_hz", 20.0))

    # Motion
    SPEED_LIN = float(p("speed_linear", -0.04))
    SPEED_ANG = float(p("speed_angular", 0.0))
    MOVE_TIME = float(p("move_time", 2.0))
    PUB_RATE_HZ = float(p("publish_rate", 20.0))

    # ROS I/O
    rospy.Subscriber(CONTROL_TOPIC, String, control_cb, queue_size=10)
    pub_raw = rospy.Publisher(RAW_TOPIC, Float32, queue_size=1000)
    pub_state = rospy.Publisher(STATE_TOPIC, String, queue_size=1000)
    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=10)

    rospy.loginfo("Control topic: %s  (send 'start' or 'stop')", CONTROL_TOPIC)

    # Twist messages
    tw_go = Twist()
    tw_go.linear.x = SPEED_LIN
    tw_go.angular.z = SPEED_ANG

    tw_stop = Twist()

    # Start paused
    _run_event.clear()
    wait_until_running(pub_state=pub_state)

    # Open serial
    pub_state.publish(f"STATE=SERIAL_OPEN port={PORT} baud={BAUD}")
    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=0.1)

    rospy.on_shutdown(
        lambda: emergency_stop(
            ser,
            cmd_pub,
            tw_stop,
            pub_state=pub_state,
            stop_stream=True,
        )
    )

    # Arduino handshake
    pub_state.publish("STATE=ARDUINO_HANDSHAKE")
    rospy.loginfo("Waiting for Arduino handshake...")

    if USE_READY:
        if not wait_for_ready(ser, 5.0):
            rospy.logwarn("No 'READY' seen, falling back to fixed delay")
            wall_sleep(2.5)
    else:
        wall_sleep(2.5)

    # Start streaming
    pub_state.publish("STATE=STREAM_ON")
    write_cmd(ser, "STREAM1")
    wall_sleep(0.1)
    flush_input(ser)

    # Start constant sampler
    sampler = SerialSampler(ser, pub_raw=pub_raw, maxlen=40000)
    sampler.start()

    # -------------------- WAIT FOR RIGHT ROBOT CALIBRATION --------------------
    pub_state.publish(
        f"STATE=WAIT_FOR_RIGHT_CALIBRATION t={WAIT_BEFORE_CALIBRATION:.1f}"
    )
    rospy.loginfo(
        "[Left robot] Waiting %.1f s before left calibration...",
        WAIT_BEFORE_CALIBRATION,
    )

    if not sleep_while_running(WAIT_BEFORE_CALIBRATION, pub_state=pub_state):
        handle_pause(ser, cmd_pub, tw_stop, pub_state)

    # -------------------- LEFT ROBOT CALIBRATION --------------------
    pub_state.publish(f"STATE=LEFT_CALIBRATION_START repeats={PRE_SET_REPEATS}")
    rospy.loginfo("[Left calibration] Sending %d calibration pulses...", PRE_SET_REPEATS)

    for i in range(int(PRE_SET_REPEATS)):
        if rospy.is_shutdown():
            sampler.stop()
            return

        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue

        rospy.loginfo("[Left calibration] Pulse %d/%d", i + 1, PRE_SET_REPEATS)
        pub_state.publish(
            f"STATE=LEFT_CALIBRATION_SPRAY i={i+1}/{PRE_SET_REPEATS} "
            f"t={PRE_SPRAY_TIME:.1f}"
        )

        if not spray_step(ser, PRE_SPRAY_TIME, pub_state=pub_state):
            continue

        if i < int(PRE_SET_REPEATS) - 1:
            rospy.loginfo(
                "[Left calibration] Waiting %.1f s before next pulse",
                PRE_WAIT_290,
            )
            pub_state.publish(f"STATE=LEFT_CALIBRATION_WAIT t={PRE_WAIT_290:.1f}")

            if not sleep_while_running(PRE_WAIT_290, pub_state=pub_state):
                continue

    pub_state.publish("STATE=LEFT_CALIBRATION_DONE")
    rospy.loginfo("[Left calibration] Calibration pulses done.")

    # Wait 30 s after the last left calibration pulse before entering main loop
    pub_state.publish(
        f"STATE=WAIT_AFTER_LEFT_CALIBRATION t={WAIT_AFTER_LEFT_CALIBRATION:.1f}"
    )
    rospy.loginfo(
        "[Left robot] Waiting %.1f s after left calibration...",
        WAIT_AFTER_LEFT_CALIBRATION,
    )

    if not sleep_while_running(WAIT_AFTER_LEFT_CALIBRATION, pub_state=pub_state):
        handle_pause(ser, cmd_pub, tw_stop, pub_state)

    # -------------------- MAIN LOOP --------------------
    pub_state.publish("STATE=MAIN_LOOP")
    rospy.loginfo("Entering main loop...")

    while not rospy.is_shutdown():
        if not _run_event.is_set():
            handle_pause(ser, cmd_pub, tw_stop, pub_state)
            continue

        # Wait for right spray 5 s + wait 30 s after right spray
        left_initial_wait = LOOP_SPRAY_10 + LOOP_WAIT_290

        rospy.loginfo("[Left Wait before spray] %.1f s", left_initial_wait)
        pub_state.publish(f"STATE=LEFT_WAIT_BEFORE_SPRAY t={left_initial_wait:.1f}")

        if not sleep_while_running(left_initial_wait, pub_state=pub_state):
            continue

        # Left robot spray
        rospy.loginfo("[Left Spray] %.1f s", LOOP_SPRAY_10)
        pub_state.publish(f"STATE=LEFT_LOOP_SPRAY t={LOOP_SPRAY_10:.1f}")

        if not spray_step(ser, LOOP_SPRAY_10, pub_state=pub_state):
            continue

        # Wait after left robot spray
        rospy.loginfo("[Left Wait after spray] %.1f s", LOOP_WAIT_290)
        pub_state.publish(f"STATE=LEFT_WAIT_AFTER_SPRAY t={LOOP_WAIT_290:.1f}")

        if not sleep_while_running(LOOP_WAIT_290, pub_state=pub_state):
            continue

        # Move
        pub_state.publish(f"STATE=MOVE t={MOVE_TIME:.1f}")
        rospy.loginfo("[Left Move] %.1f seconds", MOVE_TIME)

        move_step(cmd_pub, tw_go, tw_stop, MOVE_TIME, PUB_RATE_HZ, pub_state=pub_state)

    sampler.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass