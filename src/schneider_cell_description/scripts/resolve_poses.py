"""V44 pose resolver — release 2 cm + lateral grasp + per-joint mesa check.

V44 deltas vs V43:
  * RELEASE_DZ = 0.020 m (was 0.100 m).  CAFI now falls only 20 mm into
    the fixture / vision cradle after release.
  * Lateral grasp via two-pass IK: solve once with target = CAFI centre
    to get an approximate q, compute the gripper's LOCAL +Y closing
    axis in world frame at that q, then re-solve with target shifted by
    -LATERAL_GRASP_OFFSET * gripper_local_Y_world.  The TCP ends up
    on one lateral face of the CAFI so the appendage clamps that face.
  * Layout changes:
       BIN_ACC (1.580, 0.679) -> (1.650, 0.720)
       BIN_REJ (1.320, 0.684) -> (1.330, 0.720)
       VISION  (0.824, 0.804) -> (0.750, 0.804)
  * Per-joint mesa clearance: each resolved pose is also checked with
    fk_joint_origins_world() so no joint sits below mesa_top + 5 mm.
"""
import math, os, sys, random
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from lexium_kinematics import (
    fk_world_to_grasp_center, fk_world_to_tool0, damped_ls_ik,
    pos, rot_mat, JOINT_LIMITS,
    WORLD_COBOT_XY, WORLD_COBOT_Z, POSE_HOME_Q,
    fk_joint_origins_world, gripper_local_axis_world,
    LATERAL_GRASP_OFFSET, PICK_CONVEYOR_TARGET_DX_WORLD,
)

# V49: poses whose TARGET XYZ is shifted by PICK_CONVEYOR_TARGET_DX_WORLD
# in world +X.  Applied AFTER the lateral-grasp shift, so it is purely
# a Cartesian world-frame correction on the IK target.  Surgical fix
# for V48 visually placing the gripper too far east at PICK_CONVEYOR.
PICK_CONVEYOR_TARGET_DX_POSES = {
    "APPROACH_CONVEYOR", "PICK_CONVEYOR", "LIFT_CONVEYOR",
}


# ---- V44 Cartesian targets in WORLD frame (DXF-driven, mesa cobot) -------
MESA_TOP_Z       = 1.000
CAFI_LZ          = 0.025

BELT_TOP_Z       = MESA_TOP_Z + 0.070              # 1.070
CAFI_CTR_BELT    = BELT_TOP_Z + CAFI_LZ / 2.0      # 1.0825
CONV_PICK        = (1.235, 1.365, CAFI_CTR_BELT)

DISC_TOP_Z   = MESA_TOP_Z + 0.081                  # 1.081
LOAD_SEAT_Z  = DISC_TOP_Z + 0.030                  # 1.111  cradle/body top
CAFI_CTR_FIX = LOAD_SEAT_Z + CAFI_LZ / 2.0         # 1.1235
OUTER_FIX    = (0.737, 1.109, CAFI_CTR_FIX)
PICK_RIV     = (0.737, 1.109, CAFI_CTR_FIX)

# V44: vision moved WEST 0.074 m for cobot clearance.
VISION_TOP_Z  = MESA_TOP_Z + 0.015                 # 1.015
CAFI_CTR_VIS  = VISION_TOP_Z + CAFI_LZ / 2.0       # 1.0275
VISION    = (0.750, 0.804, CAFI_CTR_VIS)

# V49 bins: rechazado pulled BACK inside the mesa to (1.330, 0.700).
# V48 had it at Y=0.580 (off the mesa, half-Y=0.091 spilled to 0.489
# while mesa starts at Y=0.589).  aceptado stays at (1.650, 0.720).
BIN_FLOOR_Z   = MESA_TOP_Z + 0.005                 # 1.005
CAFI_CTR_BIN  = BIN_FLOOR_Z + CAFI_LZ / 2.0        # 1.0175
BIN_ACC   = (1.650, 0.720, CAFI_CTR_BIN)
BIN_REJ   = (1.330, 0.700, CAFI_CTR_BIN)

APPROACH_DZ = 0.120
LIFT_DZ     = 0.150
RETREAT_DZ  = 0.120
# V44: release at +20 mm above the cradle base (was +100 mm in V43).
RELEASE_DZ  = 0.020


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
    ("RELEASE_VISION",        (VISION[0],    VISION[1],    VISION[2]    + RELEASE_DZ)),
    ("RETREAT_VISION",        (VISION[0],    VISION[1],    VISION[2]    + RETREAT_DZ)),
    ("APPROACH_ACCEPT_BIN",   (BIN_ACC[0],   BIN_ACC[1],   BIN_ACC[2]   + APPROACH_DZ)),
    ("DROP_ACCEPT_BIN",       BIN_ACC),
    ("APPROACH_REJECT_BIN",   (BIN_REJ[0],   BIN_REJ[1],   BIN_REJ[2]   + APPROACH_DZ)),
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

POS_TOL_M = 0.025   # 25 mm position tolerance (lateral shift relaxes it slightly)
SEEDS_PER_POSE = 80
RAND_SEED      = 1234

MESA_TOP_Z_HARD = 1.000
JOINT_MESA_CLEARANCE = 0.005


def gen_seed(rng):
    return [rng.uniform(-2.0, 2.0),
            rng.uniform(-2.0, 2.0),
            rng.uniform(-2.0, 2.0),
            rng.uniform(-3.0, 3.0),
            rng.uniform(-1.8, 1.8),
            rng.uniform(-3.0, 3.0)]


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


def joint_dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(6)))


def joint_mesa_clearance(q):
    origins = fk_joint_origins_world(q)
    min_z = float("inf")
    fails = []
    for name, p in origins.items():
        if name == "base":
            continue
        if p[2] < min_z:
            min_z = p[2]
        if p[2] < MESA_TOP_Z_HARD + JOINT_MESA_CLEARANCE:
            fails.append((name, p[2]))
    return min_z, fails


def resolve_all():
    rng = random.Random(RAND_SEED)
    rows = []
    poses = {}

    q_home = list(POSE_HOME_Q)
    M_home = fk_world_to_grasp_center(q_home)
    p_home = pos(M_home)
    rows.append(("HOME", tuple(q_home),
                 (float(p_home[0]), float(p_home[1]), float(p_home[2])),
                 (float(p_home[0]), float(p_home[1]), float(p_home[2])),
                 0.0, True, 0))
    poses["HOME"] = q_home
    prev_q = q_home

    R_top_down = rot_mat(M_home)

    for name, tgt in POSE_TARGETS[1:]:
        p_target_center = np.array(tgt)
        seeds = [list(prev_q)]
        for amt in (0.3, 0.7, 1.2):
            for sgn in (-1, +1):
                s = list(prev_q)
                for i in (1, 2, 4):
                    s[i] = max(JOINT_LIMITS[i][0],
                               min(JOINT_LIMITS[i][1],
                                   s[i] + sgn * amt * (1 + 0.1*i)))
                seeds.append(s)
        seeds.append(list(POSE_HOME_Q))
        for _ in range(SEEDS_PER_POSE):
            seeds.append(gen_seed(rng))

        use_lateral = name in LATERAL_GRASP_POSES

        converged = []
        damping_variants = [(0.05, 0.20), (0.03, 0.30), (0.08, 0.15)]
        early_stop = 8
        for damping_v, clip_v in damping_variants:
            if len(converged) >= early_stop:
                break
            for seed in seeds:
                q, _, _, _ = damped_ls_ik(
                    seed, p_target_center, R_target=R_top_down,
                    max_iter=400, pos_tol=1e-4, rot_tol=2e-1,
                    damping=damping_v, step_clip=clip_v)
                qc_center = smooth_q(q)
                Mc_center = fk_world_to_grasp_center(qc_center)
                err_center = float(np.linalg.norm(p_target_center - pos(Mc_center)))
                R_c = rot_mat(Mc_center)
                axis_world_z = R_c[:, 2]
                axis_target  = R_top_down[:, 2]
                rot_err = float(np.linalg.norm(axis_world_z - axis_target))
                if err_center > 0.030 or rot_err > 0.55:
                    continue
                # PASS 2: lateral shift + re-solve.
                if use_lateral:
                    y_world = gripper_local_axis_world(qc_center, (0.0, 1.0, 0.0))
                    p_target_shifted = p_target_center - LATERAL_GRASP_OFFSET * y_world
                else:
                    p_target_shifted = p_target_center
                # V49: PICK_CONVEYOR target X correction (world frame).
                if name in PICK_CONVEYOR_TARGET_DX_POSES:
                    p_target_shifted = p_target_shifted + np.array(
                        [PICK_CONVEYOR_TARGET_DX_WORLD, 0.0, 0.0])

                q2, _, iters2, _ = damped_ls_ik(
                    qc_center, p_target_shifted, R_target=R_top_down,
                    max_iter=300, pos_tol=1e-4, rot_tol=2e-1,
                    damping=damping_v, step_clip=clip_v)
                qc = smooth_q(q2)
                Mc = fk_world_to_grasp_center(qc)
                err_final = float(np.linalg.norm(p_target_shifted - pos(Mc)))
                axis_world_z2 = rot_mat(Mc)[:, 2]
                rot_err2 = float(np.linalg.norm(axis_world_z2 - axis_target))
                min_jz, fails = joint_mesa_clearance(qc)
                if (err_final <= POS_TOL_M and rot_err2 <= 0.55 and not fails):
                    converged.append((qc, err_final, iters2, min_jz, p_target_shifted))
                    if len(converged) >= early_stop:
                        break

        if not converged:
            print(f"  [warn] {name}: lateral-IK failed, retrying without orientation constraint")
            for seed in seeds[:40]:
                q, _, _, _ = damped_ls_ik(
                    seed, p_target_center, R_target=None,
                    max_iter=400, pos_tol=1e-4,
                    damping=0.05, step_clip=0.20)
                qc_center = smooth_q(q)
                if use_lateral:
                    y_world = gripper_local_axis_world(qc_center, (0.0, 1.0, 0.0))
                    p_target_shifted = p_target_center - LATERAL_GRASP_OFFSET * y_world
                else:
                    p_target_shifted = p_target_center
                # V49: PICK_CONVEYOR target X correction (world frame).
                if name in PICK_CONVEYOR_TARGET_DX_POSES:
                    p_target_shifted = p_target_shifted + np.array(
                        [PICK_CONVEYOR_TARGET_DX_WORLD, 0.0, 0.0])
                q2, _, iters2, _ = damped_ls_ik(
                    qc_center, p_target_shifted, R_target=None,
                    max_iter=300, pos_tol=1e-4,
                    damping=0.05, step_clip=0.20)
                qc = smooth_q(q2)
                Mc = fk_world_to_grasp_center(qc)
                err_final = float(np.linalg.norm(p_target_shifted - pos(Mc)))
                min_jz, _ = joint_mesa_clearance(qc)
                if err_final <= POS_TOL_M:
                    converged.append((qc, err_final, iters2, min_jz, p_target_shifted))
                    if len(converged) >= early_stop:
                        break

        if not converged:
            for seed in seeds[:20]:
                q, _, iters, _ = damped_ls_ik(
                    seed, p_target_center, R_target=None,
                    max_iter=600, pos_tol=1e-4,
                    damping=0.05, step_clip=0.20)
                qc = smooth_q(q)
                Mc = fk_world_to_grasp_center(qc)
                err_final = float(np.linalg.norm(p_target_center - pos(Mc)))
                min_jz, _ = joint_mesa_clearance(qc)
                converged.append((qc, err_final, iters, min_jz, p_target_center))

        # Prefer joint-safe (above mesa) solutions, then nearest in joint
        # space, then lowest position error.
        converged.sort(key=lambda t: (
            0 if t[3] >= MESA_TOP_Z_HARD + JOINT_MESA_CLEARANCE else 1,
            joint_dist(t[0], prev_q),
            t[1]))
        q_best, err_final, iters, min_jz, tgt_used = converged[0]

        M = fk_world_to_grasp_center(q_best)
        fk_p = pos(M)
        rows.append((name, tuple(q_best), tuple(tgt),
                     (float(fk_p[0]), float(fk_p[1]), float(fk_p[2])),
                     err_final, err_final <= POS_TOL_M, iters))
        poses[name] = list(q_best)
        prev_q = q_best
    return rows, poses


def render_table(rows):
    lines = []
    lines.append("Pose                       q1       q2       q3       q4       q5       q6       CenterTgt XYZ           FK XYZ                  Err(mm)  Valid")
    lines.append("-" * 175)
    for name, q, tgt, fk_p, err_m, ok_tol, iters in rows:
        lines.append(f"{name:24s}  "
                     f"{q[0]:+7.3f} {q[1]:+7.3f} {q[2]:+7.3f} "
                     f"{q[3]:+7.3f} {q[4]:+7.3f} {q[5]:+7.3f}  "
                     f"({tgt[0]:+6.3f},{tgt[1]:+6.3f},{tgt[2]:+6.3f})  "
                     f"({fk_p[0]:+6.3f},{fk_p[1]:+6.3f},{fk_p[2]:+6.3f})  "
                     f"{err_m*1000:7.2f}  "
                     f"{'PASS' if ok_tol else 'FAIL'}  "
                     f"(iters={iters})")
    return "\n".join(lines)


def render_python_module(poses):
    lines = [
        "# AUTO-GENERATED by resolve_poses.py — do not edit by hand.",
        "# V44 mesa-mounted Lexium L03S: lateral grasp + 2 cm release +",
        "#      per-joint mesa clearance + V44 layout (bins right, vision left,",
        "#      smaller rivet cabin).",
        "",
    ]
    for name, q in poses.items():
        lines.append(f"POSE_{name} = [{q[0]:+.6f}, {q[1]:+.6f}, "
                     f"{q[2]:+.6f}, {q[3]:+.6f}, {q[4]:+.6f}, {q[5]:+.6f}]")
    lines.append("")
    lines.append("POSE_LIB = {")
    for name in poses:
        lines.append(f"    \"POSE_{name}\": POSE_{name},")
    lines.append("}")
    lines.append("")
    return "\n".join(lines)


if __name__ == "__main__":
    rows, poses = resolve_all()
    table = render_table(rows)
    print(table)
    print()
    fails = [r for r in rows if not r[5]]
    print(f"Summary: {len(rows) - len(fails)}/{len(rows)} poses PASS "
          f"(<= {POS_TOL_M*1000:.0f} mm cart err)")
    if fails:
        print("FAILED:")
        for r in fails:
            print(f"  - {r[0]}  err={r[4]*1000:.1f} mm")

    print()
    print("Per-joint mesa clearance check (must be >= mesa_top + 5 mm = 1.005):")
    overall_min = float("inf")
    mesa_fails = []
    for name, q in poses.items():
        min_jz, joint_fails = joint_mesa_clearance(q)
        if min_jz < overall_min:
            overall_min = min_jz
        status = "PASS" if not joint_fails else "FAIL"
        if joint_fails:
            print(f"  POSE_{name:24s} min_joint_Z={min_jz:.4f} m  {status}  "
                  f"fail={joint_fails}")
            mesa_fails.append(name)
        else:
            print(f"  POSE_{name:24s} min_joint_Z={min_jz:.4f} m  {status}")
    print(f"Worst case across all poses: min_joint_Z = {overall_min:.4f} m")
    print(f"Joint-mesa fails: {len(mesa_fails)}")

    here = os.path.dirname(__file__)
    with open(os.path.join(here, "validate_robot_poses.txt"), "w") as f:
        f.write(table); f.write("\n\nSummary: ")
        f.write(f"{len(rows) - len(fails)}/{len(rows)} poses PASS "
                f"(<= {POS_TOL_M*1000:.0f} mm)\n")
        f.write(f"\nPer-joint mesa clearance worst case: {overall_min:.4f} m\n")
        f.write(f"Joint-mesa fails: {len(mesa_fails)}\n")
    with open(os.path.join(here, "resolved_poses.py"), "w") as f:
        f.write(render_python_module(poses))
    print("Wrote validate_robot_poses.txt and resolved_poses.py")
