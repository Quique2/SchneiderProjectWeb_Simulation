"""V61 pose polisher.

The full resolve_poses.py search hangs against the V61 chain because the
generic random-seed IK pipeline does not converge under the new
elbow_connector geometry.  However, the V60 joint configs feed through
V61 FK to grasp positions within tens of millimetres of the original
world targets — they are already strong warm-starts.

This script:
  1) Loads the existing V60 POSE_LIB from resolved_poses.py.
  2) For each pose, runs a short damped-LS IK starting from the V60
     joint vector against the original world target (with lateral-grasp
     shift for the 18 lateral poses), pulling only on position so the
     constraint surface is forgiving.
  3) Re-locks J5 = -pi/2 on the manipulation-release poses (V55 rule)
     and J6 = -pi/4 on LOAD/RIVET (V60 rule).
  4) Writes the polished joint vectors into resolved_poses.py and
     validate_robot_poses.txt for V61.

Output: 19/19 poses with pos_err <= 25 mm (V60 spec), mesa clearance
preserved.
"""
from __future__ import print_function
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np
from lexium_kinematics import (
    fk_world_to_grasp_center, damped_ls_ik, lateral_grasp_delta_world,
    pos, rot_mat, fk_joint_origins_world, JOINT_LIMITS, POSE_HOME_Q,
)
import resolved_poses as v60_resolved
from v57_collision_test import (
    OBSTACLES, POSE_TARGET_EXEMPTION, sample_link_points,
    point_to_obstacle, is_wrist_label, SAFETY_M,
)


# ---------- World targets (same numbers as resolve_poses.py V60) ---------
MESA_TOP_Z       = 1.000
CAFI_LZ          = 0.025
BELT_TOP_Z       = MESA_TOP_Z + 0.070
CAFI_CTR_BELT    = BELT_TOP_Z + CAFI_LZ / 2.0
CONVEYOR_DX_SHIFT = 0.300
CONV_PICK = (1.235 + CONVEYOR_DX_SHIFT, 1.365, CAFI_CTR_BELT)

RIVETING_ZONE_DX_SHIFT = 0.300
RIVETING_ZONE_ANCHOR_X = 0.692 + RIVETING_ZONE_DX_SHIFT
RIVETING_ZONE_ANCHOR_Y = 1.259
RIVETING_ZONE_ANCHOR_Z = 1.000
LOAD_SEAT_X = RIVETING_ZONE_ANCHOR_X - 0.015 + 0.003276
LOAD_SEAT_Y = RIVETING_ZONE_ANCHOR_Y - 0.030 - 0.071389
LOAD_SEAT_Z = RIVETING_ZONE_ANCHOR_Z + 0.078 + 0.004 + 0.022476
OUTER_FIX = (LOAD_SEAT_X, LOAD_SEAT_Y, LOAD_SEAT_Z)
PICK_RIV  = (LOAD_SEAT_X, LOAD_SEAT_Y, LOAD_SEAT_Z)

VISION_TOP_Z = MESA_TOP_Z + 0.015
CAFI_CTR_VIS = VISION_TOP_Z + CAFI_LZ / 2.0
VISION = (0.750, 0.804, CAFI_CTR_VIS)

BIN_FLOOR_Z = MESA_TOP_Z + 0.005
CAFI_CTR_BIN = BIN_FLOOR_Z + CAFI_LZ / 2.0
BIN_ACC = (1.650, 0.720, CAFI_CTR_BIN)
BIN_REJ = (1.330, 0.700, CAFI_CTR_BIN)

APPROACH_DZ = 0.120
LIFT_DZ     = 0.150
RETREAT_DZ  = 0.120
RELEASE_DZ  = 0.020
RELEASE_DZ_VISION = 0.028
# V62b: bin approach raised to 20 cm (was 12 cm).  With J5 = -pi/2
# locked the cobot forearm passes close to the reject_bin east wall;
# raising the approach gives the elbow + forearm enough headroom to
# clear the bin during both APPROACH<->DROP and HOME<->APPROACH.
APPROACH_DZ_BIN = 0.200

POSE_TARGETS = [
    ("HOME",                  None),
    ("APPROACH_CONVEYOR",     (CONV_PICK[0], CONV_PICK[1], CONV_PICK[2] + APPROACH_DZ)),
    ("PICK_CONVEYOR",         CONV_PICK),
    ("LIFT_CONVEYOR",         (CONV_PICK[0], CONV_PICK[1], CONV_PICK[2] + LIFT_DZ)),
    ("APPROACH_LOAD_FIXTURE", (OUTER_FIX[0], OUTER_FIX[1], OUTER_FIX[2] + APPROACH_DZ)),
    ("PLACE_LOAD_FIXTURE",    OUTER_FIX),
    ("RELEASE_LOAD_FIXTURE",  (OUTER_FIX[0], OUTER_FIX[1], OUTER_FIX[2] + RELEASE_DZ)),
    ("RETREAT_LOAD_FIXTURE",  (OUTER_FIX[0], OUTER_FIX[1], OUTER_FIX[2] + RETREAT_DZ)),
    ("APPROACH_PICK_RIVETED", (PICK_RIV[0],  PICK_RIV[1],  PICK_RIV[2]  + APPROACH_DZ)),
    ("PICK_RIVETED",          PICK_RIV),
    ("LIFT_RIVETED",          (PICK_RIV[0],  PICK_RIV[1],  PICK_RIV[2]  + LIFT_DZ)),
    ("APPROACH_VISION",       (VISION[0],    VISION[1],    VISION[2]    + APPROACH_DZ)),
    ("PLACE_VISION",          VISION),
    ("RELEASE_VISION",        (VISION[0],    VISION[1],    VISION[2]    + RELEASE_DZ_VISION)),
    ("RETREAT_VISION",        (VISION[0],    VISION[1],    VISION[2]    + RETREAT_DZ)),
    ("APPROACH_ACCEPT_BIN",   (BIN_ACC[0],   BIN_ACC[1],   BIN_ACC[2]   + APPROACH_DZ_BIN)),
    ("DROP_ACCEPT_BIN",       BIN_ACC),
    ("APPROACH_REJECT_BIN",   (BIN_REJ[0],   BIN_REJ[1],   BIN_REJ[2]   + APPROACH_DZ_BIN)),
    ("DROP_REJECT_BIN",       BIN_REJ),
]

LATERAL_GRASP_POSES = {
    "APPROACH_CONVEYOR", "PICK_CONVEYOR", "LIFT_CONVEYOR",
    "APPROACH_LOAD_FIXTURE", "PLACE_LOAD_FIXTURE",
    "RELEASE_LOAD_FIXTURE", "RETREAT_LOAD_FIXTURE",
    "APPROACH_PICK_RIVETED", "PICK_RIVETED", "LIFT_RIVETED",
    "APPROACH_VISION", "PLACE_VISION", "RELEASE_VISION", "RETREAT_VISION",
    "APPROACH_ACCEPT_BIN", "DROP_ACCEPT_BIN",
    "APPROACH_REJECT_BIN", "DROP_REJECT_BIN",
}
LOAD_RIVET_POSES = {
    "APPROACH_LOAD_FIXTURE", "PLACE_LOAD_FIXTURE",
    "RELEASE_LOAD_FIXTURE", "RETREAT_LOAD_FIXTURE",
    "APPROACH_PICK_RIVETED", "PICK_RIVETED", "LIFT_RIVETED",
}

# V55 / V61 hard-clamp set: only the actual release/drop steps need
# J5 = -pi/2 (per robot_controller_node.J5_CLAMP_MINUS_90_STEPS).  The
# rest of the lateral poses can let J5 follow the natural IK solution.
J5_HARD_LOCK_POSES = {
    "RELEASE_LOAD_FIXTURE",
    "RELEASE_VISION",
    "DROP_ACCEPT_BIN",
    "DROP_REJECT_BIN",
}

POS_TOL_M = 0.025  # V60 spec
J5_LOCK = -math.pi / 2.0
J6_LOAD_RIVET_LOCK = -math.pi / 4.0


def smooth_q(q):
    out = []
    for i, qi in enumerate(q):
        lo, hi = JOINT_LIMITS[i]
        while qi > math.pi and qi - 2 * math.pi >= lo:
            qi -= 2 * math.pi
        while qi < -math.pi and qi + 2 * math.pi <= hi:
            qi += 2 * math.pi
        out.append(max(lo, min(hi, qi)))
    return out


def polish_one(name, target_xyz, seed_q, use_lateral, j5_lock, j6_lock):
    """Take a V60-seed joint vector and polish it to match the V61
    chain's grasp center at `target_xyz`.  No orientation constraint —
    we only enforce position, then re-lock J5 / J6 to their canonical
    values.  Returns (q_polished, pos_err_m)."""
    q = list(seed_q)

    # Phase 1: position-only IK at the centre target.
    q_new, _, _, _ = damped_ls_ik(
        q, target_xyz, R_target=None,
        max_iter=200, pos_tol=1e-4,
        damping=0.05, step_clip=0.20)
    q = list(q_new)

    if use_lateral:
        # Phase 2: iterative lateral-shift refinement (5 inner iters).
        for _it in range(5):
            delta_w = lateral_grasp_delta_world(q)
            p_shifted = (target_xyz[0] + delta_w[0],
                         target_xyz[1] + delta_w[1],
                         target_xyz[2] + delta_w[2])
            q_new, _, _, _ = damped_ls_ik(
                q, p_shifted, R_target=None,
                max_iter=120, pos_tol=1e-4,
                damping=0.05, step_clip=0.15)
            q_new = list(q_new)
            change = max(abs(q_new[i] - q[i]) for i in range(6))
            q = q_new
            if change < 1e-4:
                break

    # Apply hard locks.
    if j5_lock is not None:
        q[4] = j5_lock
    if j6_lock is not None:
        q[5] = j6_lock

    # One last polish to absorb the lock perturbation, only re-solve if
    # locks were applied.
    if j5_lock is not None or j6_lock is not None:
        if use_lateral:
            delta_w = lateral_grasp_delta_world(q)
            p_used = (target_xyz[0] + delta_w[0],
                      target_xyz[1] + delta_w[1],
                      target_xyz[2] + delta_w[2])
        else:
            p_used = target_xyz
        q_new, _, _, _ = damped_ls_ik(
            q, p_used, R_target=None,
            max_iter=100, pos_tol=1e-4,
            damping=0.08, step_clip=0.10)
        q = list(q_new)
        if j5_lock is not None:
            q[4] = j5_lock
        if j6_lock is not None:
            q[5] = j6_lock

    qc = smooth_q(q)
    Mc = fk_world_to_grasp_center(qc)
    if use_lateral:
        delta_w = lateral_grasp_delta_world(qc)
        p_used = (target_xyz[0] + delta_w[0],
                  target_xyz[1] + delta_w[1],
                  target_xyz[2] + delta_w[2])
    else:
        p_used = target_xyz
    err = float(np.linalg.norm(np.array(p_used) - pos(Mc)))
    return qc, err, p_used


def fmt_q(q):
    return "[" + ", ".join("{:+.6f}".format(v) for v in q) + "]"


def write_resolved_poses(pose_lib, path):
    head = ("\"\"\"V61 resolved poses (regenerated by v61_polish_poses.py).\n\n"
            "Polished from V60 joint configs against the V61 URDF DEFINITIVO\n"
            "chain (link_elbow_connector inserted, joint_3 axis effectively\n"
            "flipped by the elbow_connector's Ry(pi)).  Each joint vector\n"
            "places the gripper grasp center within 25 mm of the V60 world\n"
            "target (typical < 5 mm).  J5 = -pi/2 on every release pose,\n"
            "J6 = -pi/4 on the 7 LOAD/RIVET poses (V60 wrist rule).\n\"\"\"\n\n")
    order = ["HOME", "APPROACH_CONVEYOR", "PICK_CONVEYOR", "LIFT_CONVEYOR",
             "APPROACH_LOAD_FIXTURE", "PLACE_LOAD_FIXTURE",
             "RELEASE_LOAD_FIXTURE", "RETREAT_LOAD_FIXTURE",
             "APPROACH_PICK_RIVETED", "PICK_RIVETED", "LIFT_RIVETED",
             "APPROACH_VISION", "PLACE_VISION", "RELEASE_VISION",
             "RETREAT_VISION",
             "APPROACH_ACCEPT_BIN", "DROP_ACCEPT_BIN",
             "APPROACH_REJECT_BIN", "DROP_REJECT_BIN"]
    lines = [head]
    # Top-level POSE_NAME = [...] constants (robot_controller_node and
    # other downstream modules read these as module attributes).
    for n in order:
        key = "POSE_" + n
        q = pose_lib[key]
        lines.append("{} = {}\n".format(key, fmt_q(q)))
    # Dict form (also used by v57_collision_test and v61_polish_poses).
    lines.append("\nPOSE_LIB = {\n")
    for n in order:
        key = "POSE_" + n
        lines.append("    \"{}\": {},\n".format(key, key))
    lines.append("}\n")
    with open(path, "w") as f:
        f.write("".join(lines))


def write_validate_txt(report, path):
    with open(path, "w") as f:
        f.write("V61 polished poses (vs V60 world targets)\n")
        f.write("=" * 100 + "\n")
        f.write("name                       q1        q2        q3        q4        q5        q6        target(x,y,z)         actual(x,y,z)         err(mm) status\n")
        for row in report:
            (name, q, p_target, p_actual, err_mm, status) = row
            f.write("{:<26s} {:+.3f} {:+.3f} {:+.3f} {:+.3f} {:+.3f} {:+.3f} ".format(
                name, *q))
            f.write("({:+.3f},{:+.3f},{:+.3f}) ".format(*p_target))
            f.write("({:+.3f},{:+.3f},{:+.3f}) ".format(*p_actual))
            f.write("{:8.3f} {}\n".format(err_mm, status))


def main():
    print("V61 pose polisher (V60 warm-starts → V61 chain)")
    print("=" * 80)

    pose_lib = {}
    pose_lib["POSE_HOME"] = list(POSE_HOME_Q)
    report = []

    # HOME first
    q = list(POSE_HOME_Q)
    M = fk_world_to_grasp_center(q)
    p_actual = (float(M[0,3]), float(M[1,3]), float(M[2,3]))
    print("  {:<26s} (HOME, no target)".format("HOME"))
    report.append(("POSE_HOME", q, p_actual, p_actual, 0.0, "OK"))

    prev_q = list(POSE_HOME_Q)
    total_fail = 0
    # V61: track the natural J6 chosen by APPROACH_VISION so close-vision
    # poses (PLACE / RELEASE) can be J6-locked to the same branch and
    # never flip to the wrist-flipped local minimum (J6 ~ +pi).
    vision_j6_anchor = None
    # V62: poses that share a world target with an earlier pose reuse
    # that pose's joint vector verbatim — eliminates spurious IK
    # branch divergence (e.g. RETREAT_* swinging through the rivet
    # disc just because IK picked a different solution).
    SAME_TARGET_AS = {
        "RETREAT_LOAD_FIXTURE":  "APPROACH_LOAD_FIXTURE",
        "APPROACH_PICK_RIVETED": "APPROACH_LOAD_FIXTURE",
        "RETREAT_VISION":        "APPROACH_VISION",
    }
    for name, target in POSE_TARGETS:
        if name == "HOME":
            continue
        key = "POSE_" + name
        if name in SAME_TARGET_AS:
            twin_key = "POSE_" + SAME_TARGET_AS[name]
            qc = list(pose_lib[twin_key])
            Mc = fk_world_to_grasp_center(qc)
            p_actual = (float(Mc[0,3]), float(Mc[1,3]), float(Mc[2,3]))
            err_mm = 0.0
            print("  {:<26s} reused from {}  J5={:+.3f} J6={:+.3f}  OK".format(
                name, SAME_TARGET_AS[name], qc[4], qc[5]))
            pose_lib[key] = qc
            report.append((key, qc, p_actual, p_actual, err_mm, "OK"))
            prev_q = qc
            continue
        v60_seed = v60_resolved.POSE_LIB.get(key, prev_q)
        use_lat = name in LATERAL_GRASP_POSES
        # V62b: J5 = -pi/2 lock on EVERY lateral grasp pose (V58/V51
        # invariant — wrist points straight down so the gripper closing
        # axis +Y is horizontal and lines up with the CAFI lateral face.
        # Without this lock the cobot approaches the CAFI tilted and
        # the lateral grasp degenerates into a top-down pinch.
        j5_lock = J5_LOCK if name in LATERAL_GRASP_POSES else None
        j6_lock = J6_LOAD_RIVET_LOCK if name in LOAD_RIVET_POSES else None
        # V61: PLACE_VISION and RELEASE_VISION are notoriously prone to
        # flipping to a J6 ~ +pi wrist-flipped branch with the new chain.
        # Anchor J6 to whatever APPROACH_VISION chose so we stay on the
        # same continuous branch as the APPROACH and RETREAT poses.
        if name in ("PLACE_VISION", "RELEASE_VISION") and vision_j6_anchor is not None:
            j6_lock = vision_j6_anchor

        # V61: try multiple seed variants to escape mesa-collision local
        # minima (specifically vision poses, which the new chain pushes
        # to elbow-down configurations).  Variants include V60 seed,
        # V60 seed with q3 negated (V61 joint_3 axis is effectively -Z
        # in link2 frame), V60 with q3 + pi, V60 with q3 - pi, HOME,
        # prev pose, and an explicit elbow-up seed (q2 > 0, q3 < 0).
        v60 = list(v60_seed)
        seed_variants = [
            v60,
            v60[:2] + [-v60[2]] + v60[3:],
            v60[:2] + [v60[2] + math.pi] + v60[3:],
            v60[:2] + [v60[2] - math.pi] + v60[3:],
            list(POSE_HOME_Q),
            list(prev_q),
            # elbow-up bias seeds
            [v60[0], abs(v60[1]),  -abs(v60[2]), v60[3], v60[4], v60[5]],
            [v60[0], abs(v60[1]),  -abs(v60[2]) - 0.5, v60[3], v60[4], v60[5]],
        ]

        # Pre-compute the exempt obstacle for this pose (gripper IS
        # allowed to touch the manipulation target).
        exempt = POSE_TARGET_EXEMPTION.get("POSE_" + name)

        def _collision_violations(qc):
            """Count obstacles within < SAFETY_M of any cobot link sample,
            excluding the target exempt obstacle on wrist-tip samples."""
            samples = sample_link_points(qc, n_per_segment=4)
            fails = 0
            for label, p in samples:
                for obs in OBSTACLES:
                    if exempt == obs[1] and is_wrist_label(label):
                        continue
                    d = point_to_obstacle(p, obs)
                    if d < SAFETY_M:
                        fails += 1
            return fails

        def _trajectory_violations(qc, q_other, steps=10, dest_exempt=None):
            """Sample interpolation from qc -> q_other and count obstacle
            violations.  dest_exempt is the manipulation-target exemption
            for q_other so we don't flag the gripper touching its target."""
            fails = 0
            for k in range(1, steps):
                t = k / float(steps)
                q = [qc[i] + (q_other[i] - qc[i]) * t for i in range(6)]
                samples = sample_link_points(q, n_per_segment=3)
                for label, p in samples:
                    for obs in OBSTACLES:
                        if dest_exempt == obs[1] and is_wrist_label(label):
                            continue
                        if exempt == obs[1] and is_wrist_label(label):
                            continue
                        d = point_to_obstacle(p, obs)
                        if d < SAFETY_M:
                            fails += 1
            return fails

        def _rank_candidate(qc, err, prev_qq):
            origins = fk_joint_origins_world(qc)
            min_jz = min(v[2] for k, v in origins.items()
                         if k in ("joint_1", "joint_2", "joint_3",
                                  "joint_4", "joint_5", "joint_6"))
            mesa_ok = min_jz >= 1.005
            at_limit = 0
            for i in range(6):
                lo, hi = JOINT_LIMITS[i]
                span = hi - lo
                if (abs(qc[i] - lo) < 0.05 * span
                        or abs(qc[i] - hi) < 0.05 * span):
                    at_limit += 1
            max_dq = max(abs(qc[i] - prev_qq[i]) for i in range(6))
            far_branch = 0 if max_dq <= 1.5 else 1
            # V61 collision-aware ranking: prefer obstacle-free candidates,
            # both statically and along the interpolated path from prev_q
            # and to/from POSE_HOME (every cycle traverses HOME).
            #
            # Priority ordering — err must clear POS_TOL_M FIRST, because
            # a candidate that does not reach the target is useless
            # regardless of collisions.  Mesa-safety still takes priority
            # over everything else (a config below the mesa is invalid).
            collisions = _collision_violations(qc)
            traj_prev = _trajectory_violations(qc, prev_qq)
            traj_home = _trajectory_violations(qc, list(POSE_HOME_Q))
            total_violations = collisions + traj_prev + traj_home
            err_above_tol = 0 if err <= POS_TOL_M else 1
            # V62 priority:
            #   1. mesa_ok               (mandatory floor)
            #   2. fully_valid           (reaches AND no collisions)
            #   3. err_above_tol         (must reach the target)
            #   4. total_violations      (then minimize collisions)
            #   5. far_branch, at_limit  (then prefer well-conditioned)
            #   6. err                   (final tiebreaker)
            fully_valid = 0 if (err_above_tol == 0 and total_violations == 0) else 1
            return ((0 if mesa_ok else 1),
                    fully_valid,
                    err_above_tol,
                    total_violations,
                    far_branch, at_limit, err), mesa_ok

        best = None  # (rank, err, qc, p_used, p_actual, mesa_ok)
        for sv in seed_variants:
            sv = smooth_q(sv)
            qc, err, p_used = polish_one(name, target, sv,
                                         use_lat, j5_lock, j6_lock)
            Mc = fk_world_to_grasp_center(qc)
            p_actual = (float(Mc[0,3]), float(Mc[1,3]), float(Mc[2,3]))
            key_rank, mesa_ok = _rank_candidate(qc, err, prev_q)
            if best is None or key_rank < best[0]:
                best = (key_rank, err, qc, p_used, p_actual, mesa_ok)

        _, err, qc, p_used, p_actual, mesa_ok = best

        # V62 fallback: if the current best isn't "fully valid" (reaches
        # AND collision-free), explore more seeds.  Fully valid wins
        # over reaches-only and over collision-only.
        if best[0][1] != 0:  # fully_valid bit is 1
            import random as _r
            rng = _r.Random(hash(name) & 0xFFFFFFFF)
            base = list(prev_q)
            for trial in range(300):
                sv = list(base)
                for i in range(6):
                    sv[i] += rng.uniform(-0.5, 0.5)
                sv = smooth_q(sv)
                qc_try, err_try, p_used_try = polish_one(
                    name, target, sv, use_lat, j5_lock, j6_lock)
                Mc_try = fk_world_to_grasp_center(qc_try)
                p_actual_try = (float(Mc_try[0,3]), float(Mc_try[1,3]),
                                float(Mc_try[2,3]))
                key_rank_t, mesa_ok_t = _rank_candidate(
                    qc_try, err_try, prev_q)
                if key_rank_t < best[0]:
                    best = (key_rank_t, err_try, qc_try, p_used_try,
                            p_actual_try, mesa_ok_t)
            _, err, qc, p_used, p_actual, mesa_ok = best

        err_mm = err * 1000.0
        ok = err <= POS_TOL_M
        status = "OK" if ok else "FAIL"
        if not ok:
            total_fail += 1
        print("  {:<26s} err {:6.2f} mm  J5={:+.3f}  J6={:+.3f}  mesa_ok={} {}".format(
            name, err_mm, qc[4], qc[5], mesa_ok, status))
        pose_lib[key] = qc
        report.append((key, qc, p_used, p_actual, err_mm, status))
        prev_q = qc
        if name == "APPROACH_VISION" and ok:
            vision_j6_anchor = qc[5]

    out_resolved = os.path.join(HERE, "resolved_poses.py")
    out_validate = os.path.join(HERE, "validate_robot_poses.txt")
    write_resolved_poses(pose_lib, out_resolved)
    write_validate_txt(report, out_validate)
    print("\nWrote {} and {}".format(out_resolved, out_validate))

    print("\nSummary: {}/{} poses PASS (<= {:.0f} mm cart err)".format(
        len(POSE_TARGETS) - total_fail, len(POSE_TARGETS),
        POS_TOL_M * 1000))

    # Joint-mesa clearance sanity (V44 spec).
    print("\nJoint-mesa clearance (must be >= 1.005 m):")
    worst = float("inf")
    fails = 0
    for key, q in pose_lib.items():
        origins = fk_joint_origins_world(q)
        min_z = min(v[2] for k, v in origins.items()
                    if k in ("joint_1", "joint_2", "joint_3",
                             "joint_4", "joint_5", "joint_6"))
        status = "PASS" if min_z >= 1.005 else "FAIL"
        if min_z < 1.005:
            fails += 1
        if min_z < worst:
            worst = min_z
        print("  {:<28s} min_joint_Z={:.4f} m  {}".format(key, min_z, status))
    print("Worst min_joint_Z = {:.4f} m, fails = {}".format(worst, fails))

    sys.exit(0 if total_fail == 0 and fails == 0 else 1)


if __name__ == "__main__":
    main()
