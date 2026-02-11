#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS1 Dual logger (left/right) -> CSV

CSV columns (one row per event):
  t_wall, t_rel, left_raw, right_raw, msg

- t_wall: raw UNIX wall timestamp (float seconds)
- t_rel : seconds since node start (t0 set ONCE at start)
- left_raw/right_raw: latest values seen so far (blank until first message)
- msg   : "RAW left/right", "LEFT_STATE ...", "RIGHT_STATE ...", "CONTROL ...", etc.

Saves on:
  - publishing "stop" to /molecular_demo/control
  - Ctrl+C (SIGINT)
  - any shutdown (SIGTERM, rosnode kill, roslaunch shutdown)

Default topics:
  /left/molecular/tx/raw    (std_msgs/Float32)
  /right/molecular/tx/raw   (std_msgs/Float32)
  /left/molecular/tx/state  (std_msgs/String)
  /right/molecular/tx/state (std_msgs/String)
  /molecular_demo/control   (std_msgs/String)

Params:
  ~out_dir (default: /home/hayeonglee/molecular_logs)
  ~prefix (default: run)
  ~flush_every (default: 1)   # rows between flush+fsync (1 is safest)
"""

import os
import csv
import time
import threading
import signal
import atexit

import rospy
from std_msgs.msg import Float32, String


def _norm_topic(s: str) -> str:
    s = (s or "").strip()
    if not s.startswith("/"):
        s = "/" + s
    while "//" in s:
        s = s.replace("//", "/")
    return s


class DualMolecularLogger:
    def __init__(self):
        self.lock = threading.Lock()
        self._closed = False

        # Start time set once
        self.t0_wall = time.time()

        # Use explicit default (so it doesn't depend on "~" in containers/users)
        self.out_dir = rospy.get_param("~out_dir", "/home/hayeonglee/molecular_logs")
        self.prefix = rospy.get_param("~prefix", "run")

        self.left_raw_topic = _norm_topic(rospy.get_param("~left_raw_topic", "/left/molecular/tx/raw"))
        self.right_raw_topic = _norm_topic(rospy.get_param("~right_raw_topic", "/right/molecular/tx/raw"))
        self.left_state_topic = _norm_topic(rospy.get_param("~left_state_topic", "/left/molecular/tx/state"))
        self.right_state_topic = _norm_topic(rospy.get_param("~right_state_topic", "/right/molecular/tx/state"))
        self.control_topic = _norm_topic(rospy.get_param("~control_topic", "/molecular_demo/control"))

        self.flush_every = int(rospy.get_param("~flush_every", 1))
        if self.flush_every < 1:
            self.flush_every = 1
        self._rows_since_flush = 0

        # Latest sensor values
        self.left_val = None
        self.right_val = None

        # Ensure output dir exists
        self.out_dir = os.path.abspath(os.path.expanduser(self.out_dir))
        os.makedirs(self.out_dir, exist_ok=True)

        start_ts = int(self.t0_wall)
        self.path = os.path.join(self.out_dir, f"{self.prefix}_{start_ts}.csv")

        # Open file
        try:
            # buffering=1 => line-buffered (best effort)
            self.f = open(self.path, "w", newline="", buffering=1)
        except Exception as e:
            rospy.logerr("[LOGGER] Failed to open CSV path: %s (%s)", self.path, str(e))
            raise

        self.w = csv.writer(self.f)
        self.w.writerow(["t_wall", "t_rel", "left_raw", "right_raw", "msg"])
        self._flush(force=True)

        rospy.loginfo("[LOGGER] Writing -> %s", self.path)

    def _flush(self, force: bool = False):
        if self._closed:
            return
        if not force and self._rows_since_flush < self.flush_every:
            return
        try:
            self.f.flush()
            os.fsync(self.f.fileno())
        except Exception:
            # fsync may fail on some filesystems; flush is still helpful
            try:
                self.f.flush()
            except Exception:
                pass
        self._rows_since_flush = 0

    def _write_row_locked(self, msg: str):
        t_wall = time.time()
        t_rel = t_wall - self.t0_wall

        l = "" if self.left_val is None else f"{float(self.left_val):.6f}"
        r = "" if self.right_val is None else f"{float(self.right_val):.6f}"

        self.w.writerow([f"{t_wall:.6f}", f"{t_rel:.6f}", l, r, msg])
        self._rows_since_flush += 1
        self._flush(force=False)

    def log_event(self, msg: str):
        with self.lock:
            self._write_row_locked(msg)

    # ---------- Callbacks ----------
    def on_left_raw(self, m: Float32):
        with self.lock:
            self.left_val = float(m.data)
            self._write_row_locked("RAW left")

    def on_right_raw(self, m: Float32):
        with self.lock:
            self.right_val = float(m.data)
            self._write_row_locked("RAW right")

    def on_left_state(self, m: String):
        self.log_event(f"LEFT_STATE {m.data}")

    def on_right_state(self, m: String):
        self.log_event(f"RIGHT_STATE {m.data}")

    def on_control(self, m: String):
        cmd = (m.data or "").strip()
        self.log_event(f"CONTROL {cmd}")
        if cmd.lower() in ("stop", "pause", "quit", "exit"):
            rospy.signal_shutdown("CONTROL requested shutdown")

    def bind(self):
        rospy.Subscriber(self.left_raw_topic, Float32, self.on_left_raw, queue_size=5000)
        rospy.Subscriber(self.right_raw_topic, Float32, self.on_right_raw, queue_size=5000)
        rospy.Subscriber(self.left_state_topic, String, self.on_left_state, queue_size=1000)
        rospy.Subscriber(self.right_state_topic, String, self.on_right_state, queue_size=1000)
        rospy.Subscriber(self.control_topic, String, self.on_control, queue_size=100)

        rospy.loginfo("[LOGGER] Subscribed:")
        rospy.loginfo("  %s (Float32)", self.left_raw_topic)
        rospy.loginfo("  %s (Float32)", self.right_raw_topic)
        rospy.loginfo("  %s (String)", self.left_state_topic)
        rospy.loginfo("  %s (String)", self.right_state_topic)
        rospy.loginfo("  %s (String)  <-- publish 'stop' to end", self.control_topic)

    def close(self, reason: str = "close()"):
        # Idempotent close
        with self.lock:
            if self._closed:
                return
            self._closed = True
            try:
                # best-effort final marker
                self._write_row_locked(f"LOGGER_STOP {reason}")
            except Exception:
                pass
            try:
                self._flush(force=True)
            except Exception:
                pass
            try:
                self.f.close()
            except Exception:
                pass


def main():
    rospy.init_node("molecular_logger_lr", anonymous=True)
    lg = DualMolecularLogger()
    lg.bind()

    # Start marker row
    lg.log_event("LOGGER_START")

    # Ensure save on any shutdown path
    def _on_shutdown():
        try:
            rospy.loginfo("[LOGGER] Shutdown triggered. Saving CSV...")
        except Exception:
            pass
        lg.close("rospy.on_shutdown")
        try:
            rospy.loginfo("[LOGGER] Saved -> %s", lg.path)
        except Exception:
            pass

    rospy.on_shutdown(_on_shutdown)

    # Extra safety: also save on SIGINT/SIGTERM (Ctrl+C, roslaunch stop, etc.)
    def _signal_handler(signum, frame):
        try:
            rospy.loginfo("[LOGGER] Signal %s received -> shutting down", str(signum))
        except Exception:
            pass
        try:
            rospy.signal_shutdown(f"signal {signum}")
        except Exception:
            # if rospy isn't fully up, still close file
            lg.close(f"signal {signum}")

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    # Last-resort: if interpreter exits unexpectedly
    atexit.register(lambda: lg.close("atexit"))

    rospy.loginfo("[LOGGER] Running. Ctrl+C should save. Also publish 'stop' to %s.", lg.control_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # If for some reason rospy didn't catch it, still save
        lg.close("KeyboardInterrupt")
    finally:
        # Guarantee close even if callbacks fail
        lg.close("finally")


if __name__ == "__main__":
    main()
