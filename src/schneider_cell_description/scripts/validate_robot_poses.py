#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""validate_robot_poses.py  --  V23

Runs the V23 IK-resolved poses through:
   1. JOINT LIMITS                  (lexium_cobot_real.xacro V17 ranges)
   2. SAFE_JOINT_FILTER             (sphere-based self-collision check)
   3. FK CARTESIAN ERROR            (grasp_center error against the target)

Prints a PASS/FAIL table and exits non-zero if any operative pose fails.

Usage:
    python validate_robot_poses.py
"""
from __future__ import print_function
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Patch rospy so safe_joint_filter can be imported on Windows / outside a
# ROS env.  We only need its is_safe() and fk_points().
fake_rospy = types.SimpleNamespace()
fake_rospy.loginfo = lambda *a, **k: None
fake_rospy.logwarn = lambda *a, **k: None
fake_rospy.Time = type("T", (), {"now": staticmethod(lambda: None)})
fake_rospy.Publisher = lambda *a, **k: None
fake_rospy.Subscriber = lambda *a, **k: None
fake_rospy.init_node = lambda *a, **k: None
fake_rospy.spin = lambda: None
sys.modules["rospy"] = fake_rospy
sys.modules["sensor_msgs"] = types.ModuleType("sensor_msgs")
sys.modules["sensor_msgs.msg"] = types.ModuleType("sensor_msgs.msg")
sys.modules["sensor_msgs.msg"].JointState = object

import math
import numpy as np
import safe_joint_filter as sjf
import resolved_poses
import lexium_kinematics as kin

# Targets must match those in resolve_poses.py.
# V33: HOME is "TCP tucked 250 mm below the cobot anchor" so the cobot
# rests visibly tucked between work moves.  PICK heights unchanged.
TARGETS = {
    # V38: targets in WORLD frame for cobot mounted directly on the
    # mesa at (1.152, 1.049, 1.000).  Conveyor PICK is on the LEFT
    # (west) end of the belt.  HOME = real planta HOME (J4=J5=-90 deg);
    # FK at that pose with the V38 anchor gives world (1.468, 0.868, 1.741).
    "POSE_HOME":                  (1.468, 0.868, 1.741),
    "POSE_APPROACH_CONVEYOR":     (1.235, 1.365, 1.255),
    "POSE_PICK_CONVEYOR":         (1.235, 1.365, 1.135),
    "POSE_LIFT_CONVEYOR":         (1.235, 1.365, 1.285),
    "POSE_APPROACH_LOAD_FIXTURE": (0.692, 1.109, 1.271),
    "POSE_PLACE_LOAD_FIXTURE":    (0.692, 1.109, 1.151),
    "POSE_RETREAT_LOAD_FIXTURE":  (0.692, 1.109, 1.271),
    "POSE_APPROACH_PICK_RIVETED": (0.692, 1.109, 1.271),
    "POSE_PICK_RIVETED":          (0.692, 1.109, 1.151),
    "POSE_LIFT_RIVETED":          (0.692, 1.109, 1.301),
    "POSE_APPROACH_VISION":       (0.824, 0.804, 1.185),
    "POSE_PLACE_VISION":          (0.824, 0.804, 1.065),
    "POSE_RETREAT_VISION":        (0.824, 0.804, 1.185),
    "POSE_APPROACH_ACCEPT_BIN":   (1.411, 0.679, 1.245),
    "POSE_DROP_ACCEPT_BIN":       (1.411, 0.679, 1.125),
    "POSE_APPROACH_REJECT_BIN":   (1.110, 0.684, 1.245),
    "POSE_DROP_REJECT_BIN":       (1.110, 0.684, 1.125),
}

# User-requested tolerance: <= 20 mm cartesian.
TOL_M = 0.020


def main():
    rows = []
    all_pass = True
    for name in TARGETS:
        q = resolved_poses.POSE_LIB[name]
        # 1. Joint limits
        within_limits = True
        for i in range(6):
            lo, hi = kin.JOINT_LIMITS[i]
            if not (lo <= q[i] <= hi):
                within_limits = False
        # 2. Safe filter
        safe_ok, safe_reason = sjf.is_safe(q)
        # 3. FK
        M = kin.fk_world_to_grasp_center(q)
        fk_p = kin.pos(M)
        tgt = np.array(TARGETS[name])
        err_m = float(np.linalg.norm(tgt - fk_p))
        fk_ok = (err_m <= TOL_M)

        row_ok = within_limits and safe_ok and fk_ok
        all_pass = all_pass and row_ok
        rows.append((name, q, tgt, fk_p, err_m,
                     within_limits, safe_ok, fk_ok, safe_reason))

    # Print table
    print()
    print("{:30s}  {:7s} {:7s} {:7s} {:7s} {:7s} {:7s}  {:21s}  {:21s}  {:>6s}  {:>4s} {:>4s} {:>4s}".format(
        "Pose", "q1", "q2", "q3", "q4", "q5", "q6",
        "Target XYZ", "FK XYZ", "Err mm", "Lim", "Safe", "FK"))
    print("-" * 170)
    for name, q, tgt, fk_p, err_m, lim_ok, safe_ok, fk_ok, safe_reason in rows:
        print("{:30s}  {:+7.3f} {:+7.3f} {:+7.3f} {:+7.3f} {:+7.3f} {:+7.3f}  "
              "({:+6.3f},{:+6.3f},{:+6.3f})  ({:+6.3f},{:+6.3f},{:+6.3f})  "
              "{:6.2f}  {:>4s} {:>4s} {:>4s}".format(
            name, q[0], q[1], q[2], q[3], q[4], q[5],
            tgt[0], tgt[1], tgt[2],
            fk_p[0], fk_p[1], fk_p[2],
            err_m * 1000,
            "PASS" if lim_ok  else "FAIL",
            "PASS" if safe_ok else "FAIL",
            "PASS" if fk_ok   else "FAIL"))
        if not safe_ok:
            print("    safe_filter reason: {}".format(safe_reason))

    print()
    n = len(rows)
    n_ok = sum(1 for r in rows if (r[5] and r[6] and r[7]))
    print("Summary: {}/{} poses PASS all checks (cart tol <= {:.0f} mm).".format(
        n_ok, n, TOL_M * 1000))
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
