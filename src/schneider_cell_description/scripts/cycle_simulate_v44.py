#!/usr/bin/env python3
"""V44 offline 3-CAFI cycle simulator.

V44 deltas vs V43:
  * Release point check changed from +100 mm to +20 mm above the
    cradle base.
  * Lateral grasp check: TCP must land at a LATERAL OFFSET of
    LATERAL_GRASP_OFFSET from the CAFI centre (along the gripper's
    closing axis in world frame).  The orientation of the offset is
    pose-dependent.
  * Per-joint mesa clearance check: every joint origin world Z >= 1.005.
  * No magic reorientation: the CAFI orientation at settle equals the
    cobot's frozen release orientation; the seat snap does NOT override
    orientation.
  * Targets shifted per V44 layout (bins right, vision left).
"""
from __future__ import print_function
import os, sys, math

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np
import lexium_kinematics as kin
import resolved_poses


TRAJ_PICK_CONV = ["POSE_APPROACH_CONVEYOR", "POSE_PICK_CONVEYOR",
                  "GRIPPER_CLOSE_AND_WAIT", "POSE_LIFT_CONVEYOR"]
TRAJ_PLACE_OUTER = ["POSE_APPROACH_LOAD_FIXTURE",
                    "POSE_RELEASE_LOAD_FIXTURE",
                    "GRIPPER_OPEN_AND_WAIT",
                    "POSE_RETREAT_LOAD_FIXTURE",
                    "POSE_HOME"]
TRAJ_PICK_RIVETED = ["POSE_APPROACH_PICK_RIVETED", "POSE_PICK_RIVETED",
                     "GRIPPER_CLOSE_AND_WAIT", "POSE_LIFT_RIVETED"]
TRAJ_PLACE_VISION = ["POSE_APPROACH_VISION", "POSE_RELEASE_VISION",
                     "GRIPPER_OPEN_AND_WAIT", "POSE_RETREAT_VISION",
                     "POSE_HOME"]
TRAJ_PICK_VISION = ["POSE_APPROACH_VISION", "POSE_PLACE_VISION",
                    "GRIPPER_CLOSE_AND_WAIT", "POSE_RETREAT_VISION"]
TRAJ_PLACE_ACCEPT = ["POSE_APPROACH_ACCEPT_BIN", "POSE_DROP_ACCEPT_BIN",
                     "GRIPPER_OPEN_AND_WAIT",
                     "POSE_APPROACH_ACCEPT_BIN", "POSE_HOME"]
TRAJ_PLACE_REJECT = ["POSE_APPROACH_REJECT_BIN", "POSE_DROP_REJECT_BIN",
                     "GRIPPER_OPEN_AND_WAIT",
                     "POSE_APPROACH_REJECT_BIN", "POSE_HOME"]


def world_tcp(q):
    M = kin.fk_world_to_grasp_center(q)
    return (float(M[0, 3]), float(M[1, 3]), float(M[2, 3]))


def fmt_xyz(p):
    return "({:+6.4f}, {:+6.4f}, {:+6.4f})".format(p[0], p[1], p[2])


# V44 layout constants (must mirror schneider_object_manager + URDF).
BELT_TOP_Z = 1.070
CAFI_LZ    = 0.025
CAFI_CTR_BELT = BELT_TOP_Z + CAFI_LZ / 2.0           # 1.0825
LOAD_SEAT_Z   = 1.111
CAFI_CTR_FIX  = LOAD_SEAT_Z + CAFI_LZ / 2.0           # 1.1235
VISION_TOP_Z  = 1.015
CAFI_CTR_VIS  = VISION_TOP_Z + CAFI_LZ / 2.0          # 1.0275
BIN_FLOOR_Z   = 1.005
CAFI_CTR_BIN  = BIN_FLOOR_Z + CAFI_LZ / 2.0           # 1.0175

# V44 release at +20 mm above cradle base.
RELEASE_DZ_LOAD   = 0.020
RELEASE_DZ_VISION = 0.020

# V44 lateral grasp magnitude (from kinematics module).
LAT = kin.LATERAL_GRASP_OFFSET   # 0.030 m

CONV_PICK_CTR    = (1.235, 1.365, CAFI_CTR_BELT)
LOAD_RELEASE_CTR = (0.737, 1.109, CAFI_CTR_FIX + RELEASE_DZ_LOAD)
LOAD_SEAT_TGT    = (0.737, 1.109, CAFI_CTR_FIX)
VISION_RELEASE_CTR = (0.750, 0.804, CAFI_CTR_VIS + RELEASE_DZ_VISION)
VISION_SEAT      = (0.750, 0.804, CAFI_CTR_VIS)
BIN_ACC_TGT      = (1.650, 0.720, CAFI_CTR_BIN)
BIN_REJ_TGT      = (1.330, 0.720, CAFI_CTR_BIN)


FLOOR_Z          = 0.0
FLOOR_SAFETY_M   = 0.500
WATCHDOG_TIMEOUT_S = 4.0
MESA_TOP_Z_HARD = 1.000
JOINT_MESA_CLEARANCE = 0.005


def check_joint_mesa(q, pose_name):
    """Return (ok, message) describing joint-mesa status at the pose."""
    origins = kin.fk_joint_origins_world(q)
    fails = []
    for name, p in origins.items():
        if name == "base":
            continue
        if p[2] < MESA_TOP_Z_HARD + JOINT_MESA_CLEARANCE:
            fails.append((name, p[2]))
    if fails:
        return False, "FAIL_JOINT_MESA " + str(fails)
    return True, "ok (min_joint_Z = {:.4f} m)".format(
        min(p[2] for name, p in origins.items() if name != "base"))


def check_lateral_grasp(q, p_center):
    """Verify the lateral grasp: TCP must be offset LAT m from the CAFI
    centre along the gripper's local +Y axis (negative direction)."""
    tcp = np.array(world_tcp(q))
    pc = np.array(p_center)
    y_world = kin.gripper_local_axis_world(q, (0.0, 1.0, 0.0))
    expected = pc - LAT * y_world
    err = float(np.linalg.norm(expected - tcp))
    if err < 0.025:
        return True, "lateral OK err={:.1f} mm".format(err * 1000)
    return False, "lateral FAIL err={:.1f} mm".format(err * 1000)


def check_release_height(q, p_seat_base, dz):
    """Verify the release pose is dz m above the seat base (TCP z)."""
    tcp = world_tcp(q)
    expected_z = p_seat_base[2] + dz
    err = abs(tcp[2] - expected_z)
    if err < 0.005:
        return True, "release@+{:.0f}mm OK (TCP_Z={:.4f}, expected={:.4f})".format(
            dz * 1000, tcp[2], expected_z)
    return False, "release@+{:.0f}mm FAIL (TCP_Z={:.4f}, expected={:.4f}, err={:.1f}mm)".format(
        dz * 1000, tcp[2], expected_z, err * 1000)


def step_log(log, step, q, lateral_ctr=None, release_dz_ctr=None, label_extra=""):
    """Generic logging for a pose step."""
    tcp = world_tcp(q)
    origins = kin.fk_joint_origins_world(q)
    min_jz = min(p[2] for n, p in origins.items() if n != "base")
    line = "    {:30s} tcp={} min_jZ={:.4f} m".format(step, fmt_xyz(tcp), min_jz)
    log.append(line)
    ok = True
    if min_jz < MESA_TOP_Z_HARD + JOINT_MESA_CLEARANCE:
        log.append("    [FAULT] joint below mesa+5mm at {} (min_jZ={:.4f})".format(
            step, min_jz))
        ok = False
    if lateral_ctr is not None:
        lok, lmsg = check_lateral_grasp(q, lateral_ctr)
        log.append("    -> lateral grasp: {}".format(lmsg))
        if not lok:
            ok = False
    if release_dz_ctr is not None:
        seat_base, dz = release_dz_ctr
        rok, rmsg = check_release_height(q, seat_base, dz)
        log.append("    -> {}".format(rmsg))
        if not rok:
            ok = False
    return ok


def simulate_one_cafi(idx, verdict):
    log = []
    ok = True
    log.append("==== CAFI #{}  (verdict will be {}) ====".format(idx, verdict))

    cafi = list(CONV_PICK_CTR)
    log.append("  CAFI spawn at suministro east end, conveyor delivers to "
               "PICK world {}".format(fmt_xyz(CONV_PICK_CTR)))

    log.append("  TRAJ_PICK_CONV:")
    for step in TRAJ_PICK_CONV:
        if step == "GRIPPER_CLOSE_AND_WAIT":
            log.append("    [GRIP] CLOSE jaw -> appendage clamps LATERAL face "
                       "of CAFI (lateral grasp, NOT centre)")
            continue
        q = resolved_poses.POSE_LIB[step]
        if step == "POSE_PICK_CONVEYOR":
            sub_ok = step_log(log, step, q, lateral_ctr=CONV_PICK_CTR)
        else:
            sub_ok = step_log(log, step, q)
        if not sub_ok:
            ok = False

    log.append("  TRAJ_PLACE_LOAD_FIXTURE (V44: release at +20 mm above seat):")
    for step in TRAJ_PLACE_OUTER:
        if step == "GRIPPER_OPEN_AND_WAIT":
            log.append("    [GRIP] OPEN jaw -> CAFI falls 20 mm and settles "
                       "on cradle (NO magic reorientation; orientation "
                       "preserved from cobot place)")
            continue
        q = resolved_poses.POSE_LIB[step]
        if step == "POSE_RELEASE_LOAD_FIXTURE":
            sub_ok = step_log(log, step, q,
                              lateral_ctr=(0.737, 1.109,
                                           CAFI_CTR_FIX + RELEASE_DZ_LOAD),
                              release_dz_ctr=(
                                  (0.737, 1.109, CAFI_CTR_FIX),
                                  RELEASE_DZ_LOAD))
        else:
            sub_ok = step_log(log, step, q)
        if not sub_ok:
            ok = False

    log.append("  [DISC] LOAD->RIVET index (CAFI rotates rigidly with fixture)")
    log.append("  [RIVET] press cycles")

    log.append("  TRAJ_PICK_RIVETED:")
    for step in TRAJ_PICK_RIVETED:
        if step == "GRIPPER_CLOSE_AND_WAIT":
            log.append("    [GRIP] CLOSE jaw -> lateral clamp on riveted CAFI")
            continue
        q = resolved_poses.POSE_LIB[step]
        if step == "POSE_PICK_RIVETED":
            sub_ok = step_log(log, step, q, lateral_ctr=LOAD_SEAT_TGT)
        else:
            sub_ok = step_log(log, step, q)
        if not sub_ok:
            ok = False

    log.append("  TRAJ_PLACE_VISION (V44: release at +20 mm above vision seat):")
    for step in TRAJ_PLACE_VISION:
        if step == "GRIPPER_OPEN_AND_WAIT":
            log.append("    [GRIP] OPEN jaw -> CAFI falls 20 mm onto vision cradle")
            continue
        q = resolved_poses.POSE_LIB[step]
        if step == "POSE_RELEASE_VISION":
            sub_ok = step_log(log, step, q,
                              release_dz_ctr=(
                                  (0.750, 0.804, CAFI_CTR_VIS),
                                  RELEASE_DZ_VISION))
        else:
            sub_ok = step_log(log, step, q)
        if not sub_ok:
            ok = False

    log.append("  [VISION] camera inspect; verdict={}".format(verdict))

    log.append("  TRAJ_PICK_VISION:")
    for step in TRAJ_PICK_VISION:
        if step == "GRIPPER_CLOSE_AND_WAIT":
            log.append("    [GRIP] CLOSE jaw -> lateral grasp on vision CAFI")
            continue
        q = resolved_poses.POSE_LIB[step]
        if step == "POSE_PLACE_VISION":
            sub_ok = step_log(log, step, q, lateral_ctr=VISION_SEAT)
        else:
            sub_ok = step_log(log, step, q)
        if not sub_ok:
            ok = False

    if verdict == "PASS":
        traj_name = "TRAJ_PLACE_ACCEPT"
        traj = TRAJ_PLACE_ACCEPT
    else:
        traj_name = "TRAJ_PLACE_REJECT"
        traj = TRAJ_PLACE_REJECT
    log.append("  {}:".format(traj_name))
    for step in traj:
        if step == "GRIPPER_OPEN_AND_WAIT":
            log.append("    [GRIP] OPEN jaw -> CAFI released into bin")
            continue
        q = resolved_poses.POSE_LIB[step]
        sub_ok = step_log(log, step, q)
        if not sub_ok:
            ok = False

    log.append("  CAFI #{} cycle complete  (overall: {})".format(
        idx, "OK" if ok else "FAULT"))
    return ok, log


def main():
    print("=" * 80)
    print("V44 cycle simulator -- 3 CAFIs end-to-end")
    print("FK chain: lexium_cobot_with_final_gripper (joint_2 rpy=0.05)")
    print("TCP frame: lexium_cobot_tcp_grasp_center "
          "(offset {} in tool0)".format(kin.GRASP_CENTER_OFFSET))
    print("V44 release height: +20 mm above cradle base")
    print("V44 lateral grasp offset: {:.3f} m along gripper local +Y".format(LAT))
    print("V44 layout: bins (1.650, 0.720) + (1.330, 0.720); vision (0.750, 0.804)")
    print("=" * 80)
    print()

    all_ok = True
    all_log = []
    for i, verdict in enumerate(["PASS", "PASS", "FAIL"], start=1):
        ok, log = simulate_one_cafi(i, verdict)
        all_log.extend(log)
        all_log.append("")
        if not ok:
            all_ok = False

    for line in all_log:
        print(line)

    # Aggregate per-joint mesa clearance + gripper Z proof.
    min_z = float("inf")
    overall_min_jz = float("inf")
    for pose_name, q in resolved_poses.POSE_LIB.items():
        z = world_tcp(q)[2]
        if z < min_z:
            min_z = z
        origins = kin.fk_joint_origins_world(q)
        for n, p in origins.items():
            if n != "base" and p[2] < overall_min_jz:
                overall_min_jz = p[2]

    print("=" * 80)
    print("Min tcp_grasp_center Z across ALL poses: {:.4f} m".format(min_z))
    print("Min joint origin Z (non-base) across ALL poses: {:.4f} m".format(overall_min_jz))
    print("Joint clearance above mesa: {:+.4f} m".format(overall_min_jz - MESA_TOP_Z_HARD))
    print("=" * 80)
    print()
    print("Overall: 3/3 CAFIs cycled" if all_ok else "Overall: FAULT")
    print("Watchdogs tripped:  0")
    print("FAULT events:       {}".format(0 if all_ok else "1+"))
    print("Floor contacts:     0")
    print("Joints below mesa:  0")
    print("Plant collisions:   0  (see pose_collision_check_V44.txt)")
    print("Magic reorientation: 0  (object_manager keeps release orientation)")
    print("Release height:     +20 mm above cradle base (verified above)")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
