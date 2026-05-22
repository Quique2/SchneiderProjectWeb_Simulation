#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_visualization.visualization_node  --  V23

Three responsibilities:

  [1] JOINT STATE FUSER
      Collects partial JointState publications from:
        /robot/cobot_joints  (6 cobot joints)
        /gripper/jaw_joint   (1 prismatic jaw)
        /disc/joints         (1 disc + 6 fixture pistons/solenoids)
      Fuses them into one full sensor_msgs/JointState on
      /joint_states_desired which the safe_joint_filter publishes to
      /joint_states for robot_state_publisher.

  [2] DEBUG MARKERS / LABELS
      Optional decorative markers on /visualization/markers.  V22 had a
      hose marker that looked up gripper_pneumatics_inlet_a/b -- those
      frames were removed in V22 along with the gripper_pneumatics
      assembly, so the lookup raised every tick.  V23 removes the dead
      lookup entirely.

  [3] DISC CHAIN TRACE
      The fuser now logs (5 Hz throttled) the disc joint angle as it
      flows through the fuser, so when the user wires up the manual
      `rostopic pub /disc/index_cmd Float32 ...` test they can see the
      angle being received by visualization_node before robot_state_publisher
      consumes the fused /joint_states.

NOTE: Does NOT publish CAFI markers (object_manager does that).
"""

import math
import threading

import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray


# Every joint the V25 URDF expects.  V25: fixture_*_solenoid_right_joint
# REMOVED (only LEFT solenoid per fixture, per user brief).
ALL_JOINTS = [
    "lexium_cobot_joint_1", "lexium_cobot_joint_2", "lexium_cobot_joint_3",
    "lexium_cobot_joint_4", "lexium_cobot_joint_5", "lexium_cobot_joint_6",
    # V43: gripper is part of the new cobot URDF; the prismatic joint is
    # now appendage_prismatic_joint (prefixed by the cobot instance name).
    "lexium_cobot_appendage_prismatic_joint",
    "rotary_table_disc_axis_joint",
    "fixture_A_piston_joint", "fixture_A_solenoid_left_joint",
    "fixture_B_piston_joint", "fixture_B_solenoid_left_joint",
]

DISC_JOINT = "rotary_table_disc_axis_joint"

FUSE_HZ = 50.0


class Visualization(object):

    def __init__(self):
        rospy.init_node("schneider_visualization", anonymous=False)

        self.joint_cache = {n: 0.0 for n in ALL_JOINTS}
        # V35: the user requires the spawn pose to be q=0 (all six
        # cobot joints at zero) so the cabin/mount design is
        # visually validated from the cobot's full reference
        # configuration.  We therefore leave the cobot entries at
        # zero — no seeding from POSE_HOME.  POSE_HOME itself was
        # redefined to q=0 in resolved_poses.py / robot_controller,
        # so the spawn frame, the operative HOME, and the FK base
        # for IK validation all agree.

        self.lock = threading.Lock()
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.pub_jsd     = rospy.Publisher(
            "/joint_states_desired", JointState, queue_size=2)
        self.pub_markers = rospy.Publisher(
            "/visualization/markers", MarkerArray, queue_size=2)

        rospy.Subscriber("/robot/cobot_joints", JointState,
                         self._cb_partial)
        rospy.Subscriber("/gripper/jaw_joint",  JointState,
                         self._cb_partial)
        rospy.Subscriber("/disc/joints",        JointState,
                         self._cb_partial)

        rospy.loginfo("[VIZ] V23 visualization init - joint fuser, disc "
                      "chain tracing")

    def _cb_partial(self, js):
        with self.lock:
            for i, n in enumerate(js.name):
                if i < len(js.position):
                    if n == DISC_JOINT and \
                            abs(js.position[i] - self.joint_cache[n]) > 1e-4:
                        rospy.loginfo_throttle(0.2,
                            "[VIZ] disc joint fused angle=%.3f rad "
                            "(from /disc/joints)", js.position[i])
                    self.joint_cache[n] = js.position[i]

    def publish_joints(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        with self.lock:
            js.name = list(ALL_JOINTS)
            js.position = [self.joint_cache[n] for n in ALL_JOINTS]
        self.pub_jsd.publish(js)

    def run(self):
        rate_j  = rospy.Rate(FUSE_HZ)
        while not rospy.is_shutdown():
            self.publish_joints()
            rate_j.sleep()


if __name__ == "__main__":
    try:
        Visualization().run()
    except rospy.ROSInterruptException:
        pass
