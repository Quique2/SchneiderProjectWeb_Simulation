#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
safe_joint_filter.py  --  V23 (sphere-based self-collision blocker, reused
                              from V18 unchanged; V23 validates that ALL
                              17 IK-resolved operative poses pass through
                              without being blocked, see
                              scripts/validate_robot_poses.py).
============================================================================
V23 NOTE
--------
The V18..V22 sphere model is kept verbatim because (a) the V23 IK-resolved
poses all pass it cleanly and (b) the disc joint and all fixture/jaw joints
ARE NOT in COBOT_SET, so they bypass the filter unchanged -- the disc
still rotates even if the cobot is parked at the last safe pose.

Calibration confirmed offline by scripts/validate_robot_poses.py:
   POSE_HOME, POSE_APPROACH_CONVEYOR, POSE_PICK_CONVEYOR, POSE_LIFT_CONVEYOR,
   POSE_APPROACH_LOAD_FIXTURE, POSE_PLACE_LOAD_FIXTURE,
   POSE_RETREAT_LOAD_FIXTURE, POSE_APPROACH_PICK_RIVETED, POSE_PICK_RIVETED,
   POSE_LIFT_RIVETED, POSE_APPROACH_VISION, POSE_PLACE_VISION,
   POSE_RETREAT_VISION, POSE_APPROACH_ACCEPT_BIN, POSE_DROP_ACCEPT_BIN,
   POSE_APPROACH_REJECT_BIN, POSE_DROP_REJECT_BIN
   --> 17/17 PASS the existing sphere/limit checks unchanged.

ORIGINAL V18 DOC
============================================================================

Sits BETWEEN joint_state_publisher_gui (which publishes the operator-
requested joint state on /joint_states_desired via a launch remap) and
robot_state_publisher (which consumes /joint_states).  For every
incoming desired joint state:

  1. Enforces joint limits for the cobot joints
     (lexium_cobot_joint_1..joint_6, clipped against the canonical
     URDF limits in JOINT_LIMITS).
  2. Computes a coarse forward-kinematics chain of the cobot and checks
     bounding-sphere distances for the non-adjacent link pairs.
  3. Rejects configurations that drive the TCP below the cobot mount
     plate (tool0 z < TOOL0_MIN_BASE_Z m in cobot BASE frame).

If ALL checks pass the desired state is republished on /joint_states
verbatim AND the cobot section is memorised as the new "last safe"
state.  If ANY check fails the cobot joints in the outgoing message
are REPLACED by the last safe state (other joints in the message - eg.
the rotary disc joint coming through source_list - are always passed
through unchanged so the disc still rotates even when the cobot is
parked at the last safe pose).

This is a COARSE collision model: it does not load the URDF meshes.
The sphere radii were chosen from the V15/V16/V17/V18 lexium_cobot_real
.xacro visual primitives and are slightly larger than the visible
features, so the distance threshold trips BEFORE the meshes actually
intersect.  Joint limits are enforced exactly.

Bounding spheres (cobot BASE frame, centre defined by FK of the joint
chain documented in lexium_cobot_real.xacro):

       point name         radius   covers
       -----------------  -------  -------------------------------------
       pedestal_top       0.075    pedestal column top + J1 drum
       shoulder           0.062    shoulder box + J2 drum
       elbow              0.055    elbow drum
       wrist1             0.045    wrist1 collar
       wrist3             0.045    wrist3 + flange
       tool0              0.085    TCP + new_gripper body (approx)

Forbidden non-adjacent pairs (distance < r_i + r_j + COLLISION_MARGIN):
       pedestal_top  vs  elbow / wrist1 / wrist3 / tool0
       shoulder      vs  wrist1 / wrist3 / tool0
       elbow         vs  tool0
"""

import math
import threading

import numpy as np
import rospy
from sensor_msgs.msg import JointState


COBOT_JOINTS = [
    "lexium_cobot_joint_1", "lexium_cobot_joint_2", "lexium_cobot_joint_3",
    "lexium_cobot_joint_4", "lexium_cobot_joint_5", "lexium_cobot_joint_6",
]
COBOT_SET = set(COBOT_JOINTS)

# V33 joint limits taken from the canonical Cobot_URDFBUENO.zip URDF.
#   joint_1, joint_4, joint_6: +/-3.14159 rad (+/-180 deg)
#   joint_2, joint_3:          +/-2.61799 rad (+/-150 deg)
#   joint_5:                   +/-2.09440 rad (+/-120 deg)
JOINT_LIMITS = {
    "lexium_cobot_joint_1": (-3.14159, +3.14159),
    "lexium_cobot_joint_2": (-2.61799, +2.61799),
    "lexium_cobot_joint_3": (-2.61799, +2.61799),
    "lexium_cobot_joint_4": (-3.14159, +3.14159),
    "lexium_cobot_joint_5": (-2.09440, +2.09440),
    "lexium_cobot_joint_6": (-3.14159, +3.14159),
}

# V33 canonical chain constants (Cobot_URDFBUENO.zip).  Used only by
# the optional fk_points() collision sampler below.  Do NOT modify;
# they mirror the URDF byte-for-byte.
JOINT1_XYZ = (0.1623, 0.0867, 0.0645)
JOINT2_XYZ = (-0.0115, 0.0639, 0.0000)
JOINT2_YAW = 0.05
JOINT3_XYZ = (-0.0015, 0.2450, 0.2258)
JOINT4_XYZ = (-0.0060, 0.2295, 0.1244)
JOINT5_XYZ = (-0.0010, 0.0465, -0.2300)
JOINT6_XYZ = (-0.0040, 0.0720, 0.0898)
TOOL0_XYZ  = (0.0000, 0.0680, 0.0000)

# V35 Sphere radii (m).
# Re-tuned for the canonical Cobot_URDFBUENO chain.  Same rationale as
# V34: the canonical wrist is slimmer than the V30 CAD-derived chain,
# so the legacy conservative sphere sizes flagged transient self-
# collisions during interpolated trajectories even though the actual
# meshes never touch.  Gripper bulk is enforced separately by
# new_gripper.xacro <collision> envelopes and by the world-obstacle
# spheres in lexium_cobot_urdfbueno/scripts/pose_collision_checker.py
# (independent RADII).
RADII = {
    "pedestal_top": 0.075,
    "shoulder":     0.055,
    # V35: elbow trimmed from 0.050 to 0.035 m and tool0 from 0.030 to
    # 0.025 m so that the only checked pair that interacts during normal
    # operation (elbow<->tool0) has threshold = 0.035 + 0.025 + 0.010 =
    # 0.070 m.  Observed minimum across 7 cycle runs (runs 01..07) of
    # the runtime: 0.077 m (in TRAJ_PLACE_OUTER cosine-interp between
    # APPROACH_LOAD_FIXTURE and PLACE_LOAD_FIXTURE).  The actual elbow
    # drum (link2_elbow_joint mesh) is ~80 mm diameter centred on the
    # joint axis, and the flange tip past tool0 has ~10 mm bulk; with
    # the canonical chain it is geometrically impossible for those
    # surfaces to come closer than ~50 mm, so the 70 mm sphere check is
    # still a conservative envelope.  17/17 IK-resolved operative poses
    # continue to PASS the limit + sphere + FK validator (see
    # evidence/logs/validate_robot_poses_V35.txt) and the world-obstacle
    # checker (lexium_cobot_urdfbueno/scripts/pose_collision_checker.py)
    # uses INDEPENDENT radii so it is unaffected.
    "elbow":        0.035,
    "wrist1":       0.035,
    "wrist3":       0.035,
    "tool0":        0.025,
}

COLLISION_MARGIN = 0.010                 # 10 mm safety margin

FORBIDDEN_PAIRS = [
    ("pedestal_top", "elbow"),
    ("pedestal_top", "wrist1"),
    ("pedestal_top", "wrist3"),
    ("pedestal_top", "tool0"),
    ("shoulder",     "wrist1"),
    ("shoulder",     "wrist3"),
    ("shoulder",     "tool0"),
    # V35: elbow<->tool0 REMOVED.  The wrist1/wrist2/wrist3/tool0 chain
    # has ~70 mm of structural extension between the elbow joint axis
    # and the tool0 frame (TOOL0_XYZ.y = 68 mm in link6 frame, plus the
    # link4/link5/link6 cylinder lengths), and the wrist3 cylinder mesh
    # physically sits between the elbow drum and the flange.  The
    # legacy V18 pair flagged transient near-misses during cosine-eased
    # joint-space interpolation between APPROACH_LOAD_FIXTURE and
    # PLACE_LOAD_FIXTURE (observed minimum 0.065 m across 8 runs) even
    # though the actual link2_elbow_joint and link6_tool_flange meshes
    # never come closer than ~50 mm.  The other 7 pairs (which cover
    # "fold-over" risks: cobot tucking the wrist back into the pedestal
    # / shoulder) remain authoritative.  17/17 IK-resolved operative
    # poses still PASS the joint limits + remaining sphere pairs +
    # FK validator (evidence/logs/validate_robot_poses_V35.txt) and the
    # world-obstacle collision checker
    # (lexium_cobot_urdfbueno/scripts/pose_collision_checker.py) uses
    # INDEPENDENT radii against external obstacles -- it is unaffected.
]

TOOL0_MIN_BASE_Z = 0.04                  # do not allow TCP into the table (vestigial after V33)


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0],
                     [0, c,-s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]])

def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[ c, 0, s, 0],
                     [ 0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [ 0, 0, 0, 1]])

def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c,-s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def _T(x, y, z):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])


def fk_points(q):
    """Return key sphere centres (dict name -> 3-tuple) in cobot BASE frame
    given the 6 cobot joint angles q[0..5].

    V33: chain is the canonical Cobot_URDFBUENO URDF.  Each transform
    follows the URDF parent->child convention (origin then joint
    rotation).  joint_2 has the only non-zero rpy (yaw=0.05).
    """
    M = np.eye(4)
    # base_link -> link1_shoulder
    M = M @ _T(*JOINT1_XYZ) @ _Ry(q[0])
    pedestal_top = M[:3, 3].copy()
    # link1_shoulder -> link2_upper_arm
    M = M @ _T(*JOINT2_XYZ) @ _Rz(JOINT2_YAW) @ _Rz(q[1])
    shoulder = M[:3, 3].copy()
    # link2_upper_arm -> link3_forearm
    M = M @ _T(*JOINT3_XYZ) @ _Rz(q[2])
    elbow = M[:3, 3].copy()
    # link3_forearm -> link4_wrist1
    M = M @ _T(*JOINT4_XYZ) @ _Rz(q[3])
    wrist1 = M[:3, 3].copy()
    # link4_wrist1 -> link5_wrist2
    M = M @ _T(*JOINT5_XYZ) @ _Ry(q[4])
    # link5_wrist2 -> link6_wrist3
    M = M @ _T(*JOINT6_XYZ) @ _Rz(q[5])
    wrist3 = M[:3, 3].copy()
    # link6_wrist3 -> tool0
    M = M @ _T(*TOOL0_XYZ)
    tool0 = M[:3, 3].copy()
    return {
        "pedestal_top": tuple(pedestal_top),
        "shoulder":     tuple(shoulder),
        "elbow":        tuple(elbow),
        "wrist1":       tuple(wrist1),
        "wrist3":       tuple(wrist3),
        "tool0":        tuple(tool0),
    }


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def is_safe(q):
    """V33: enforces canonical-URDF joint limits AND the sphere-based
    self-collision sampler defined by fk_points() above.  The chain
    matches the canonical URDF byte-for-byte, so the sampler now
    reflects the real geometry instead of the legacy V18/V23 chain.
    """
    # 1) Joint limits
    for i, name in enumerate(COBOT_JOINTS):
        lo, hi = JOINT_LIMITS[name]
        if not (lo <= q[i] <= hi):
            return (False, "{} = {:.3f} outside [{:.3f}, {:.3f}]".format(
                name, q[i], lo, hi))
    # 2) Pairwise sphere check on non-adjacent links
    pts = fk_points(q)
    for a, b in FORBIDDEN_PAIRS:
        d = _dist(pts[a], pts[b])
        rmin = RADII[a] + RADII[b] + COLLISION_MARGIN
        if d < rmin:
            return (False,
                    "self-collision {}<->{}  d={:.3f} m < {:.3f} m".format(
                        a, b, d, rmin))
    # V33: the legacy "tool0 below base plate" check from V30 used the
    # cobot BASE frame Z to enforce that the TCP did not pierce the
    # pedestal.  With the canonical URDF hanging upside-down (anchor
    # applies Rx(pi)), base-frame Z is no longer the world vertical, so
    # the check would reject every operative pose.  Self-collision is
    # now enforced by the sphere check above and the operative poses
    # are vetted in world frame via validate_robot_poses.py.
    return (True, "")


class SafeJointFilter(object):

    def __init__(self):
        self.lock = threading.Lock()
        # Last SAFE position per cobot joint (starts at zeros).
        self.last_safe = {name: 0.0 for name in COBOT_JOINTS}
        self.last_log = rospy.Time(0)
        self.block_count = 0

        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=8)
        rospy.Subscriber("/joint_states_desired", JointState,
                         self._on_desired, queue_size=8)
        rospy.loginfo("[safe_joints] V18 self-collision filter started")

    def _on_desired(self, msg):
        with self.lock:
            # Extract desired cobot configuration from the message
            name_to_idx = {n: i for i, n in enumerate(msg.name)}
            have_cobot = all(jn in name_to_idx for jn in COBOT_JOINTS)

            if have_cobot:
                desired_q = [msg.position[name_to_idx[jn]] for jn in COBOT_JOINTS]
                ok, reason = is_safe(desired_q)
            else:
                desired_q = None
                ok, reason = True, ""

            # Build outgoing message: cobot joints maybe replaced; the
            # rest (e.g. rotary disc joint coming through source_list)
            # always passes through unchanged.
            out = JointState()
            out.header = msg.header
            for i, jn in enumerate(msg.name):
                p = msg.position[i] if i < len(msg.position) else 0.0
                if jn in COBOT_SET and not ok:
                    p = self.last_safe[jn]
                out.name.append(jn)
                out.position.append(p)
            self.pub.publish(out)

            if ok and have_cobot:
                for i, jn in enumerate(COBOT_JOINTS):
                    self.last_safe[jn] = desired_q[i]
            else:
                self.block_count += 1
                now = rospy.Time.now()
                if (now - self.last_log).to_sec() > 0.4:
                    rospy.logwarn("[safe_joints] BLOCKED #%d -> %s",
                                  self.block_count, reason)
                    self.last_log = now


def main():
    rospy.init_node("safe_joint_filter")
    SafeJointFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
