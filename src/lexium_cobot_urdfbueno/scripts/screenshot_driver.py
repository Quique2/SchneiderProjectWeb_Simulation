#!/usr/bin/env python3
"""Drive joint_states through a sequence of single-joint motions and trigger
a screenshot at each waypoint.

Used standalone (no plant) to validate that the canonical URDF
(Cobot_URDFBUENO.zip) responds to each joint as defined in the file:
  joint_1 axis Y, joint_2 axis Z, joint_3 axis Z,
  joint_4 axis Z, joint_5 axis Y, joint_6 axis Z.

The screenshot tool is gnome-screenshot, capturing the whole screen each
time. We do NOT modify the URDF or joint axes if the result is unexpected
- this script ONLY drives the chain that the URDF defines.
"""
import os
import subprocess
import time

import rospy
from sensor_msgs.msg import JointState


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# Modest amplitude per joint (rad).
# Stay well inside each joint's URDF-defined limits.
WAVE = 0.6  # ~34 deg

EVIDENCE_DIR = os.environ.get(
    "EVIDENCE_DIR",
    os.path.expanduser("~/v33_ws/evidence/standalone_cobot"))


def publish_state(pub, positions):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = list(JOINT_NAMES)
    msg.position = list(positions)
    pub.publish(msg)


def screenshot(name):
    os.makedirs(EVIDENCE_DIR, exist_ok=True)
    path = os.path.join(EVIDENCE_DIR, name + ".png")
    rospy.loginfo("[screenshot] %s", path)
    # gnome-screenshot -f writes whole-screen PNG without prompting.
    try:
        subprocess.run(
            ["gnome-screenshot", "-f", path],
            check=False, timeout=10)
    except Exception as e:  # noqa: BLE001
        rospy.logwarn("gnome-screenshot failed: %s", e)


def settle(pub, positions, hold_s=1.5, rate_hz=50):
    rate = rospy.Rate(rate_hz)
    t_end = rospy.Time.now() + rospy.Duration(hold_s)
    while rospy.Time.now() < t_end and not rospy.is_shutdown():
        publish_state(pub, positions)
        rate.sleep()


def main():
    rospy.init_node("urdfbueno_screenshot_driver")
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.loginfo("Waiting 4 s for RViz/robot_state_publisher startup.")
    rospy.sleep(4.0)

    # 1. Pose 0 (q=0)
    zero = [0.0] * 6
    settle(pub, zero, hold_s=2.5)
    screenshot("01_pose_zero")

    # 2. Each joint individually
    for idx, jname in enumerate(JOINT_NAMES):
        q = [0.0] * 6
        q[idx] = WAVE
        settle(pub, q, hold_s=2.0)
        screenshot(f"{idx+2:02d}_{jname}_pos")
        q[idx] = -WAVE
        settle(pub, q, hold_s=2.0)
        screenshot(f"{idx+2:02d}_{jname}_neg")

    # 3. Pose 0 again at the end
    settle(pub, zero, hold_s=1.5)
    screenshot("99_pose_zero_end")
    rospy.loginfo("standalone screenshot sweep done")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
