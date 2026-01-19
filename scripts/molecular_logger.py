#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import threading
import rospy
from std_msgs.msg import Float32, String

lock = threading.Lock()
rows = []
t_start = None

def now_s():
    return rospy.Time.now().to_sec()

def add_row(robot, kind, value):
    with lock:
        rows.append([now_s(), robot, kind, value])

def make_outpath():
    out_dir = rospy.get_param("~out_dir", os.path.expanduser("~/molecular_logs"))
    os.makedirs(out_dir, exist_ok=True)
    prefix = rospy.get_param("~prefix", "run")
    start_ts = int(t_start if t_start is not None else now_s())
    return os.path.join(out_dir, f"{prefix}_{start_ts}.csv")

def save_csv():
    path = make_outpath()
    with lock:
        data = list(rows)

    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "type", "value"])  # t=timestamp, type=RAW/STATE
        w.writerows(data)

    rospy.loginfo("[LOGGER] Saved %d rows -> %s", len(data), path)

def on_control(msg):
    cmd = msg.data.strip().lower()
    add_row("ALL", "CONTROL", cmd)
    if cmd in ("stop", "pause"):
        rospy.signal_shutdown("STOP received -> save CSV")

def bind_robot(robot_name):
    raw_topic   = f"/molecular/{robot_name}/raw"
    state_topic = f"/molecular/{robot_name}/state"

    def on_raw(m):
        add_row(robot_name, "RAW", float(m.data))

    def on_state(m):
        add_row(robot_name, "STATE", m.data)

    rospy.Subscriber(raw_topic, Float32, on_raw, queue_size=5000)
    rospy.Subscriber(state_topic, String, on_state, queue_size=5000)
    rospy.loginfo("[LOGGER] Subscribed: %s, %s", raw_topic, state_topic)

def main():
    global t_start
    rospy.init_node("molecular_logger", anonymous=True)

    t_start = now_s()
    add_row("ALL", "STATE", "LOGGER_START")

    control_topic = rospy.get_param("~control_topic", "/molecular_demo/control")
    rospy.Subscriber(control_topic, String, on_control, queue_size=100)

    # front/back 고정 구독 (원하면 ~robots 파라미터로 바꿔도 됨)
    bind_robot("front")
    bind_robot("back")

    rospy.on_shutdown(lambda: (add_row("ALL", "STATE", "LOGGER_STOP"), save_csv()))
    rospy.loginfo("[LOGGER] Running. When you publish 'stop' to %s, CSV will be saved on this laptop.", control_topic)
    rospy.spin()

if __name__ == "__main__":
    main()
