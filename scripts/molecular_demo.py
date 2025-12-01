#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
from geometry_msgs.msg import Twist

# -------------------- Params --------------------
def p(name, default): return rospy.get_param("~"+name, default)

def readline_float_if_sensor(ser):
    try:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line or not line.startswith("R "):
            return None
        return float(line.split()[1])
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

def write_cmd(ser, s):
    ser.write((s if s.endswith("\n") else s+"\n").encode("ascii"))

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

    # Motion params (direct publish)
    CMD_TOPIC         = p("cmd_vel_topic", "/cmd_vel")
    SPEED_LIN         = float(p("speed_linear", -0.04))
    SPEED_ANG         = float(p("speed_angular", 0.0))
    MOVE_TIME         = float(p("move_time", 2.0))
    PUB_RATE_HZ       = float(p("publish_rate", 20.0))  # continuous stream

    # Publishers
    cmd_pub = rospy.Publisher(CMD_TOPIC, Twist, queue_size=1)

    rospy.loginfo("Opening serial %s @ %d ...", PORT, BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=1.0)

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

    # Prime baseline
    baseline = None
    if PRIME_BASELINE_S > 0.0:
        rospy.loginfo("Priming baseline for %.1f s ...", PRIME_BASELINE_S)
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < PRIME_BASELINE_S and not rospy.is_shutdown():
            val = readline_float_if_sensor(ser)
            if val is not None:
                baseline = val if baseline is None else (1.0-BASELINE_ALPHA)*baseline + BASELINE_ALPHA*val
        if baseline is not None:
            rospy.loginfo("Initial baseline: %.2f", baseline)

    sensor_rate = rospy.Rate(20)
    first_cycle = True

    # Prebuilt Twist messages
    tw_go   = Twist(); tw_go.linear.x = SPEED_LIN; tw_go.linear.y = 0; tw_go.linear.z = 0; tw_go.angular.x= 0; tw_go.angular.y= 0; tw_go.angular.z= 0
    tw_stop = Twist()

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

        # 2) WAIT FOR RECEPTION
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
                baseline = (1.0-BASELINE_ALPHA)*baseline + BASELINE_ALPHA*value

            diff = value - baseline
            rospy.loginfo("value=%.1f, baseline=%.1f, diff=%.1f", value, baseline, diff)

            if diff >= DIFF_THRESHOLD:
                received = True
                break

            sensor_rate.sleep()

        first_cycle = False

        # 3) MOVE: publish /cmd_vel continuously for MOVE_TIME
        if received:
            rospy.loginfo(">>> GAS RECEIVED: moving for %.1f s on %s", MOVE_TIME, CMD_TOPIC)
            t0 = rospy.Time.now()
            rate = rospy.Rate(PUB_RATE_HZ)
            while (rospy.Time.now() - t0).to_sec() < MOVE_TIME and not rospy.is_shutdown():
                cmd_pub.publish(tw_go)
                rate.sleep()
            # stop (publish a few zeros just to be sure)
            for _ in range(3):
                cmd_pub.publish(tw_stop)
                rospy.sleep(0.02)
            rospy.loginfo("Move done.")
        else:
            rospy.loginfo("Cycle ended without reception")

        # 4) COOLDOWN
        rospy.sleep(COOLDOWN_TIME)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
