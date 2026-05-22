#!/usr/bin/env python3
"""Drive cobot to a few diagnostic wrist orientations and capture
close-up screenshots of the flange + gripper from frontal/lateral/
perspective angles.  We use this to inspect whether the gripper is
coaxial with the flange axis or visually offset.
"""
import os
import subprocess
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty


JOINTS = ["lexium_cobot_joint_1", "lexium_cobot_joint_2",
          "lexium_cobot_joint_3", "lexium_cobot_joint_4",
          "lexium_cobot_joint_5", "lexium_cobot_joint_6"]


SHOT_DIR = os.path.expanduser("~/v34_ws/evidence/screenshots")


def shot(name):
    os.makedirs(SHOT_DIR, exist_ok=True)
    p = os.path.join(SHOT_DIR, name + ".png")
    rospy.loginfo("[shot] %s", p)
    subprocess.run(["gnome-screenshot", "-f", p], check=False, timeout=10)


def publish(pub, q):
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = list(JOINTS)
    msg.position = list(q)
    pub.publish(msg)


def main():
    rospy.init_node("v34_flange_capture", anonymous=False)
    pub = rospy.Publisher("/robot/cobot_joints", JointState, queue_size=4)
    rospy.sleep(2.0)

    # POSE_HOME from resolved_poses.py
    HOME = [-1.457499, -0.047497, 1.747727, -3.141590, 0.815302, -1.312629]

    # WRIST_DOWN: q1=0 q2=0 q3=0 q4=0 q5=pi/2 q6=0 (try to make tool0 point world -Z)
    WRIST_DOWN = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # q=0 (canonical pose)
    Q_ZERO = [0.0] * 6

    # Pose with shoulder up, elbow folded
    Q_TUCK = [0.0, -1.0, 1.5, 0.0, 0.0, 0.0]

    for name, q in [("20_HOME", HOME), ("21_QZERO", Q_ZERO),
                    ("22_TUCK", Q_TUCK)]:
        # Send several times so robot_state_publisher catches it
        for _ in range(60):
            publish(pub, q)
            rospy.sleep(0.04)
        rospy.sleep(0.5)
        shot(f"flange_{name}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
