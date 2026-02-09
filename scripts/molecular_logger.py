#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, csv, time, threading
import rospy
from std_msgs.msg import Float32, String

class MolecularLogger:
    def __init__(self):
        self.lock = threading.Lock()
        self.t0_ros = None
        self.t0_wall = time.time()

        self.out_dir = rospy.get_param("~out_dir", os.path.expanduser("~/molecular_logs"))
        self.prefix  = rospy.get_param("~prefix", "run")

        self.control_topic = rospy.get_param("~control_topic", "/molecular_demo/control")

        # For two robots (namespaced):
        #   _robots:="tb3_0,tb3_1"
        #   _raw_topic_fmt:="/{robot}/molecular/tx/raw"
        #   _state_topic_fmt:="/{robot}/molecular/tx/state"
        #
        # For single stream:
        #   _robots:="tx"
        #   _raw_topic_fmt:="/molecular/tx/raw"
        #   _state_topic_fmt:="/molecular/tx/state"
        self.robots = rospy.get_param("~robots", ["tx"])
        if isinstance(self.robots, str):
            self.robots = [x.strip() for x in self.robots.split(",") if x.strip()]

        self.raw_topic_fmt   = rospy.get_param("~raw_topic_fmt",   "/molecular/tx/raw")
        self.state_topic_fmt = rospy.get_param("~state_topic_fmt", "/molecular/tx/state")

        os.makedirs(self.out_dir, exist_ok=True)
        start_ts = int(self.t0_wall)
        self.path = os.path.join(self.out_dir, f"{self.prefix}_{start_ts}.csv")

        self.f = open(self.path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_ros", "t_rel", "robot", "type", "value"])
        self.f.flush()

        rospy.loginfo("[LOGGER] Writing -> %s", self.path)

    def add_row(self, robot, kind, value):
        t_ros = rospy.Time.now().to_sec()
        if self.t0_ros is None:
            self.t0_ros = t_ros
        t_rel = t_ros - self.t0_ros

        with self.lock:
            self.w.writerow([f"{t_ros:.6f}", f"{t_rel:.6f}", robot, kind, value])
            self.f.flush()

    def on_control(self, msg: String):
        cmd = msg.data.strip()
        self.add_row("ALL", "CONTROL", cmd)
        rospy.loginfo("[LOGGER] CONTROL: %s", cmd)
        if cmd.lower() in ("stop", "pause"):
            rospy.signal_shutdown("STOP/PAUSE received -> save CSV")

    def bind_robot(self, robot):
        raw_topic = self.raw_topic_fmt.format(robot=robot)
        state_topic = self.state_topic_fmt.format(robot=robot)

        rospy.Subscriber(raw_topic, Float32,
                         lambda m: self._on_raw(robot, m),
                         queue_size=5000)
        rospy.Subscriber(state_topic, String,
                         lambda m: self._on_state(robot, m),
                         queue_size=5000)

        rospy.loginfo("[LOGGER] Subscribed: %s, %s", raw_topic, state_topic)

    def _on_raw(self, robot, m: Float32):
        self.add_row(robot, "RAW", float(m.data))
        rospy.loginfo_throttle(1.0, f"[LOGGER] RAW incoming (sampled) from {robot}")

    def _on_state(self, robot, m: String):
        self.add_row(robot, "STATE", m.data)
        rospy.loginfo("[LOGGER] %s STATE: %s", robot, m.data)

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass

def main():
    rospy.init_node("molecular_logger", anonymous=True)
    lg = MolecularLogger()

    lg.add_row("ALL", "STATE", "LOGGER_START")
    rospy.Subscriber(lg.control_topic, String, lg.on_control, queue_size=100)

    for r in lg.robots:
        lg.bind_robot(r)

    def _shutdown():
        lg.add_row("ALL", "STATE", "LOGGER_STOP")
        rospy.loginfo("[LOGGER] Saved -> %s", lg.path)
        lg.close()

    rospy.on_shutdown(_shutdown)
    rospy.loginfo("[LOGGER] Running. Publish 'stop' to %s to end + save.", lg.control_topic)
    rospy.spin()

if __name__ == "__main__":
    main()
