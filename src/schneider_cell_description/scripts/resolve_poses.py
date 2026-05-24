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
    lateral_grasp_delta_world,
    LATERAL_GRASP_DELTA, PICK_CONVEYOR_TARGET_DX_WORLD,
)

# V51: PICK_CONVEYOR no longer needs an ad-hoc world-X shift.  The
# lateral grasp delta (LATERAL_GRASP_DELTA, set from the URDF debug
# golden reference) fully defines the offset between TCP and CAFI
# centre.  The set below is kept for backward compatibility but the
# shift constant is now 0.
PICK_CONVEYOR_TARGET_DX_POSES = {
    "APPROACH_CONVEYOR", "PICK_CONVEYOR", "LIFT_CONVEYOR",
}


# ---- V44 Cartesian targets in WORLD frame (DXF-driven, mesa cobot) -------
MESA_TOP_Z       = 1.000
CAFI_LZ          = 0.025

BELT_TOP_Z       = MESA_TOP_Z + 0.070              # 1.070
CAFI_CTR_BELT    = BELT_TOP_Z + CAFI_LZ / 2.0      # 1.0825
CONV_PICK        = (1.235, 1.365, CAFI_CTR_BELT)

# V52: LOAD / RIVETED targets come from the new turntable URDF's
# fixture_1_cafi_lateral_target_frame and fixture_2_cafi_lateral_target_frame.
# The chain in the URDF is:
#   riveting_zone -> base_link             : (0, 0, 0)
#   base_link -> turntable_link            : (-0.015, 0, +0.078)
#   turntable_link -> rivet_fixture_1_link : ( 0.000, -0.030, +0.004)
#   rivet_fixture_1_link -> target_frame   : (+0.003276, -0.071389, +0.022476)
#
# V56: the riveting_zone is shifted EAST by RIVETING_ZONE_DX_SHIFT so
# the cobot can reach the LOAD seat with joint5 = -pi/2 (= -90 deg) as
# the natural IK solution.  The shift is mirrored in
# schneider_cell.urdf.xacro `world_to_riveting_zone` joint origin.
# Anchor base: world (0.692, 1.259, 1.000) → with +0.30 m shift the
# new anchor is (0.992, 1.259, 1.000).
RIVETING_ZONE_DX_SHIFT = 0.300                              # V56 (m)
RIVETING_ZONE_ANCHOR_X = 0.692 + RIVETING_ZONE_DX_SHIFT     # 0.992
RIVETING_ZONE_ANCHOR_Y = 1.259
RIVETING_ZONE_ANCHOR_Z = 1.000
LOAD_SEAT_X  = RIVETING_ZONE_ANCHOR_X - 0.015 + 0.000 + 0.003276
LOAD_SEAT_Y  = RIVETING_ZONE_ANCHOR_Y + 0.000 - 0.030 - 0.071389
LOAD_SEAT_Z  = RIVETING_ZONE_ANCHOR_Z + 0.078 + 0.004 + 0.022476
DISC_TOP_Z   = 1.082                               # informative only
CAFI_CTR_FIX = LOAD_SEAT_Z
OUTER_FIX    = (LOAD_SEAT_X, LOAD_SEAT_Y, CAFI_CTR_FIX)
PICK_RIV     = (LOAD_SEAT_X, LOAD_SEAT_Y, CAFI_CTR_FIX)

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
# V51: vision release needs a bit more clearance because the new
# LATERAL_GRASP_DELTA bends the IK so the TCP lands ~2.5 mm below
# the center target.  +8 mm extra lift keeps tcp_tip clear of the
# vision fixture cradle (margin >= 5 mm in pose_collision_checker).
RELEASE_DZ_VISION = 0.028


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

    # V54: TWO physical truths constrain the LOAD / RIVETED solver:
    #   (a) The LOAD seat at (0.680, 1.158, 1.104) is right at the edge
    #       of the cobot's reach (horizontal 0.484 m vs ~0.49 m max
    #       strict-top-down reach).  The 44 mm lateral grasp shift then
    #       pushes the IK target OUT of the strict-top-down reach
    #       manifold — verified empirically by sweeping 1000 random
    #       seeds with strict pos_tol+rot_tol: 0 solutions converge to
    #       both <5 mm pos and <0.05 rot.  So strict top-down at LOAD
    #       is physically impossible; we must accept some wrist tilt.
    #   (b) Whatever tilt the IK chooses at PICK_CONVEYOR vs PLACE_LOAD
    #       is the rotation the CAFI undergoes while being carried.
    #       Visible "CAFI chueco" = tilt difference between PICK and
    #       PLACE orientations.  V53 had ~30° tilt at PLACE_LOAD with
    #       0° tilt at PICK_CONVEYOR → ~30° relative carry rotation =
    #       the visible chueco.
    # V54 strategy:
    #   * tighten the rot tolerance at LOAD/RIVETED from V53's 0.55
    #     (~32°) to 0.30 (~17°): half the chueco.
    #   * solve PLACE_LOAD_FIXTURE first, then COPY the joint pose to
    #     PICK_RIVETED (same world XYZ + we want zero gripper rotation
    #     between place and re-pick on the same fixture).
    #   * use the PLACE_LOAD wrist as the seed for APPROACH/RELEASE/
    #     RETREAT/LIFT so the entire LOAD/RIVETED group shares the
    #     same wrist orientation and APPROACH→PLACE→RETREAT is a pure
    #     vertical move (no diagonal).
    LOAD_RIVET_POSES = {
        "APPROACH_LOAD_FIXTURE", "PLACE_LOAD_FIXTURE",
        "RELEASE_LOAD_FIXTURE", "RETREAT_LOAD_FIXTURE",
        "APPROACH_PICK_RIVETED", "PICK_RIVETED", "LIFT_RIVETED",
    }
    # V54: rot_tol_pose for LOAD/RIVETED.  Physical constraint: the
    # LOAD seat at (0.680, 1.158, 1.104) plus the 44 mm lateral grasp
    # shift puts the IK target outside the strict-top-down reach
    # manifold.  Achievable rot_err vs IK-converge tradeoff:
    #   0.10  (~6° tilt) — no convergence (warned + fallback w/ no R)
    #   0.30 (~17° tilt) — no convergence (warned + fallback w/ no R)
    #   0.55 (~32° tilt) — V53 baseline, converges, 30° visible tilt
    #   0.45 (~26° tilt) — V54 sweet spot: converges (no fallback) AND
    #                      keeps the visible tilt smaller than V53.
    LOAD_RIVET_ROT_TOL = 0.45

    for name, tgt in POSE_TARGETS[1:]:
        p_target_center = np.array(tgt)
        R_target_pose = R_top_down
        # Build seed pool: prev_q + HOME + perturbations + random.
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

        # V54: LOAD_RIVET shares the PLACE_LOAD wrist.  Once PLACE_LOAD
        # is solved, every other LOAD/RIVET pose seeds from it so the
        # whole group has the same q4-q5-q6 character → APPROACH/RETREAT
        # is purely vertical; PICK_RIVETED is byte-for-byte equal to
        # PLACE_LOAD (they share the same world XYZ).
        if name in LOAD_RIVET_POSES and "PLACE_LOAD_FIXTURE" in poses:
            seeds = [list(poses["PLACE_LOAD_FIXTURE"])] + seeds

        # Pose-specific rot tolerance: LOAD_RIVET gets tighter strict tol.
        rot_tol_pose = (LOAD_RIVET_ROT_TOL
                        if name in LOAD_RIVET_POSES else 0.20)

        converged = []

        # V54 LOAD_RIVET dedicated path: iterative lateral-shift
        # refinement that minimizes wrist tilt.
        #
        # Physical reality: the LOAD seat plus the 44 mm lateral grasp
        # shift puts the IK target ON the strict-top-down reach
        # boundary.  V54 algorithm — for each random seed:
        #   1. Position-only IK to LOAD (no R constraint) -> q.
        #   2. Loop: compute delta_w(q), set p_shifted = LOAD + delta_w,
        #      re-solve position-only IK to p_shifted starting from q.
        #   3. CAFI placement is now EXACT (cafi = LOAD ± 0.02 mm) and
        #      the wrist tilt is whatever the IK selected at the fixed
        #      point.
        #
        # V56 LOAD_RIVET addition: lock joint5 = -pi/2 (= -90 deg) so
        # every PLACE_LOAD / PICK_RIVETED / RELEASE_LOAD / etc. pose
        # comes in with the wrist flat as the user requires.  After
        # each IK call we set q[4] = -pi/2 and re-solve so the other
        # joints absorb the change.  Together with the +0.30 m
        # RIVETING_ZONE_DX_SHIFT this configuration converges to
        # pos_err < 10 mm across hundreds of seeds.
        if name in LOAD_RIVET_POSES:
            local_seeds = [list(prev_q), list(POSE_HOME_Q)]
            # V56: seed J5 hard at -pi/2 so the IK starts on the desired
            # branch.
            for _ in range(300):
                s = gen_seed(rng)
                s[4] = -math.pi / 2 + rng.uniform(-0.05, 0.05)
                local_seeds.append(s)
            scored = []
            for seed in local_seeds:
                # Phase 1: position-only at p_center.
                q, _, _, _ = damped_ls_ik(
                    seed, p_target_center, R_target=None,
                    max_iter=400, pos_tol=1e-5,
                    damping=0.05, step_clip=0.20)
                # V56: lock J5 = -pi/2 after the position pass and let
                # the next IK re-solves absorb it.
                q = list(q); q[4] = -math.pi / 2
                q, _, _, _ = damped_ls_ik(
                    q, p_target_center, R_target=None,
                    max_iter=300, pos_tol=1e-4,
                    damping=0.05, step_clip=0.15)
                q = list(q); q[4] = -math.pi / 2
                M = fk_world_to_grasp_center(q)
                if float(np.linalg.norm(
                        p_target_center - pos(M))) > 5e-3:
                    continue
                if not use_lateral:
                    qc = smooth_q(q); qc[4] = -math.pi / 2
                    Mc = fk_world_to_grasp_center(qc)
                    err_final = float(np.linalg.norm(
                        p_target_center - pos(Mc)))
                    min_jz, fails = joint_mesa_clearance(qc)
                    if err_final <= POS_TOL_M and not fails:
                        rot_err_z = float(np.linalg.norm(
                            rot_mat(Mc)[:, 2] - R_top_down[:, 2]))
                        # V56: score primarily by J5 deviation, then
                        # rot_err, then pos_err — guarantees the
                        # chosen pose actually has J5 at -90.
                        j5_err = abs(qc[4] + math.pi / 2)
                        scored.append((j5_err, rot_err_z, qc, err_final, 0,
                                       min_jz, p_target_center))
                    continue
                # Phase 2: iterative lateral-shift refinement (V56 also
                # re-locks J5 after each iter).
                for _it in range(8):
                    delta_w = lateral_grasp_delta_world(q)
                    p_shifted = p_target_center + delta_w
                    q_new, _, _, _ = damped_ls_ik(
                        q, p_shifted, R_target=None,
                        max_iter=400, pos_tol=1e-5,
                        damping=0.03, step_clip=0.10)
                    q_new = list(q_new); q_new[4] = -math.pi / 2
                    M_new = fk_world_to_grasp_center(q_new)
                    if float(np.linalg.norm(
                            p_shifted - pos(M_new))) > 1e-2:
                        break
                    change = max(abs(q_new[i] - q[i]) for i in range(6))
                    q = q_new
                    if change < 1e-3:
                        break
                qc = smooth_q(q); qc[4] = -math.pi / 2
                Mc = fk_world_to_grasp_center(qc)
                delta_w_final = lateral_grasp_delta_world(qc)
                p_shifted_final = p_target_center + delta_w_final
                err_final = float(np.linalg.norm(
                    p_shifted_final - pos(Mc)))
                rot_err_z = float(np.linalg.norm(
                    rot_mat(Mc)[:, 2] - R_top_down[:, 2]))
                min_jz, fails = joint_mesa_clearance(qc)
                # V56: loosened from 5 mm to 15 mm because the strict
                # top-down + J5=-pi/2 constraint can push the IK to
                # ~10 mm in the worst seeds.
                if err_final <= 0.015 and not fails:
                    j5_err = abs(qc[4] + math.pi / 2)
                    scored.append((j5_err, rot_err_z, qc, err_final, 0,
                                   min_jz, p_shifted_final))
            # V56: prefer solutions where J5 actually landed at -pi/2,
            # then small rot tilt, then small pos err.
            scored.sort(key=lambda s: (s[0], s[1], s[3]))
            for s in scored[:8]:
                j5_err, rot_err_z, qc, err_final, iters_b, min_jz, p_shifted_use = s
                converged.append((qc, err_final, iters_b, min_jz,
                                  p_shifted_use))

        damping_variants = [(0.05, 0.20), (0.03, 0.30), (0.08, 0.15)]
        early_stop = 8
        for damping_v, clip_v in damping_variants:
            if converged:
                break  # V54: LOAD_RIVET succeeded; skip the generic path.
            if len(converged) >= early_stop:
                break
            for seed in seeds:
                q, _, _, _ = damped_ls_ik(
                    seed, p_target_center, R_target=R_target_pose,
                    max_iter=400, pos_tol=1e-4, rot_tol=5e-2,
                    damping=damping_v, step_clip=clip_v)
                qc_center = smooth_q(q)
                Mc_center = fk_world_to_grasp_center(qc_center)
                err_center = float(np.linalg.norm(p_target_center - pos(Mc_center)))
                R_c = rot_mat(Mc_center)
                axis_world_z = R_c[:, 2]
                axis_target  = R_target_pose[:, 2]
                rot_err = float(np.linalg.norm(axis_world_z - axis_target))
                if err_center > 0.030 or rot_err > rot_tol_pose:
                    continue
                if use_lateral:
                    delta_w = lateral_grasp_delta_world(qc_center)
                    p_target_shifted = p_target_center + delta_w
                else:
                    p_target_shifted = p_target_center
                if name in PICK_CONVEYOR_TARGET_DX_POSES and \
                        PICK_CONVEYOR_TARGET_DX_WORLD != 0.0:
                    p_target_shifted = p_target_shifted + np.array(
                        [PICK_CONVEYOR_TARGET_DX_WORLD, 0.0, 0.0])

                q2, _, iters2, _ = damped_ls_ik(
                    qc_center, p_target_shifted, R_target=R_target_pose,
                    max_iter=300, pos_tol=1e-4, rot_tol=5e-2,
                    damping=damping_v, step_clip=clip_v)
                qc = smooth_q(q2)
                Mc = fk_world_to_grasp_center(qc)
                err_final = float(np.linalg.norm(p_target_shifted - pos(Mc)))
                axis_world_z2 = rot_mat(Mc)[:, 2]
                rot_err2 = float(np.linalg.norm(axis_world_z2 - axis_target))
                min_jz, fails = joint_mesa_clearance(qc)
                if (err_final <= POS_TOL_M and rot_err2 <= rot_tol_pose
                        and not fails):
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
                    delta_w = lateral_grasp_delta_world(qc_center)
                    p_target_shifted = p_target_center + delta_w
                else:
                    p_target_shifted = p_target_center
                # V51: PICK_CONVEYOR_TARGET_DX_WORLD is now 0; legacy
                # shift kept for backward compat / future micro-tuning.
                if name in PICK_CONVEYOR_TARGET_DX_POSES and \
                        PICK_CONVEYOR_TARGET_DX_WORLD != 0.0:
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

    # V54 post-process: lock the wrist of every LOAD/RIVET pose to the
    # wrist that PLACE_LOAD_FIXTURE settled on, so APPROACH→PLACE→
    # RETREAT is a pure VERTICAL move (no wrist wobble between
    # poses).  The iterative solver above found the lowest-tilt wrist
    # at PLACE_LOAD (~20°); reusing it for APPROACH/RETREAT/LIFT
    # keeps the whole LOAD/RIVET group consistent.
    if "PLACE_LOAD_FIXTURE" in poses:
        q_place = poses["PLACE_LOAD_FIXTURE"]
        # Lock the wrist orientation to PLACE_LOAD's R so APPROACH /
        # RELEASE / RETREAT / LIFT all hold the same gripper Z axis
        # (same tilt) — the descent / lift is then a PURE vertical
        # move in world.  Without this constraint the IK drifts to
        # whatever local minimum it finds at each Z, producing visible
        # wrist wobble between consecutive poses.
        R_place_lock = rot_mat(fk_world_to_grasp_center(q_place))
        place_targets = {
            "APPROACH_LOAD_FIXTURE": "APPROACH_LOAD_FIXTURE",
            "RELEASE_LOAD_FIXTURE":  "RELEASE_LOAD_FIXTURE",
            "RETREAT_LOAD_FIXTURE":  "RETREAT_LOAD_FIXTURE",
            "APPROACH_PICK_RIVETED": "APPROACH_PICK_RIVETED",
            "PICK_RIVETED":          "PICK_RIVETED",  # same XYZ as PLACE
            "LIFT_RIVETED":          "LIFT_RIVETED",
        }
        # Map name -> target tuple, in case POSE_TARGETS order changes.
        name_to_tgt = {n: t for n, t in POSE_TARGETS}
        for name in place_targets:
            if name not in poses:
                continue
            tgt = name_to_tgt.get(name)
            if tgt is None:
                continue
            p_target_center = np.array(tgt)
            # PICK_RIVETED has the EXACT same XYZ as PLACE_LOAD -> copy.
            if name == "PICK_RIVETED":
                poses[name] = list(q_place)
                continue
            # Iteratively re-solve from q_place seed.  Use lateral shift
            # if applicable.  R_target=None lets the wrist settle into
            # whatever pose the position-only IK finds NEAR q_place's
            # wrist (seeded by it).
            q = list(q_place)
            for _it in range(6):
                delta_w = lateral_grasp_delta_world(q)
                p_shifted = p_target_center + delta_w
                q_new, _, _, _ = damped_ls_ik(
                    q, p_shifted, R_target=R_place_lock,
                    max_iter=400, pos_tol=1e-4, rot_tol=0.40,
                    damping=0.05, step_clip=0.15)
                # V56: re-lock J5 = -pi/2 so APPROACH / RETREAT / LIFT
                # also enforce the wrist-flat constraint, matching
                # PLACE_LOAD.
                q_new = list(q_new); q_new[4] = -math.pi / 2
                if max(abs(q_new[i] - q[i]) for i in range(6)) < 1e-3:
                    q = q_new
                    break
                q = q_new
            qc = smooth_q(q); qc[4] = -math.pi / 2
            Mc = fk_world_to_grasp_center(qc)
            delta_w_final = lateral_grasp_delta_world(qc)
            p_shifted_final = p_target_center + delta_w_final
            if float(np.linalg.norm(
                    p_shifted_final - pos(Mc))) <= POS_TOL_M:
                # Update rows + poses if the wrist-locked solution
                # is better aligned with PLACE_LOAD's wrist.
                poses[name] = list(qc)
                # Update the row entry to reflect the new q.
                for i, r in enumerate(rows):
                    if r[0] == name:
                        fk_p = pos(fk_world_to_grasp_center(qc))
                        rows[i] = (name, tuple(qc), r[2],
                                   (float(fk_p[0]), float(fk_p[1]),
                                    float(fk_p[2])),
                                   float(np.linalg.norm(
                                       p_shifted_final - pos(Mc))),
                                   True, 0)
                        break
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
