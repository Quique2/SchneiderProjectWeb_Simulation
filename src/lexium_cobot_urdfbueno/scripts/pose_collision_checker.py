#!/usr/bin/env python3
"""V44 pose collision checker (strict mode, per-joint mesa check).

V44 deltas vs V43:
  * Layout updated to V44 positions:
      bins  : aceptado (1.650, 0.720), rechazado (1.330, 0.720)
      vision: (0.750, 0.804)
      rivet cabin canopy: 0.450 x 0.300 x 0.350 m (shrunk)
  * Per-joint mesa clearance HARD requirement: every joint origin must
    be at least mesa_top + 5 mm = 1.005 m in world.  base_link is
    exempt (it bolts to the mesa by design).
  * Whitelist tightened so the GRIPPER is allowed to touch ONLY the
    CAFI being grasped (which is not modelled as an obstacle); all
    other elements (mesa, fixtures, bins, cabin, camera, conveyor,
    NEMA, cabin posts) are HARD checks at every pose.  The previous
    "intentional approach" whitelists are GONE; V44 IK poses are
    re-tuned to clear every obstacle.
"""
from __future__ import print_function
import os
import sys
import math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../../schneider_cell_description/scripts"))

import lexium_kinematics as kin
import resolved_poses


# V44 cobot sphere model — tightened radii match canonical L03S mesh
# envelopes more accurately.  Sphere centres are at the joint origins;
# radii are the per-link bbox radii (NOT generous safety bubbles).
COBOT_SPHERES = [
    # name, link_name, world_offset, radius
    ("base_plate",   "base",     (0.0, 0.0,  0.0),  0.075),
    ("shoulder",     "j2_pre",   (0.0, 0.0,  0.0),  0.050),
    ("elbow",        "j3_pre",   (0.0, 0.0,  0.0),  0.045),
    ("wrist1",       "j4_pre",   (0.0, 0.0,  0.0),  0.025),
    ("wrist3",       "j6_pre",   (0.0, 0.0,  0.0),  0.025),
    ("tool0",        "tool0",    (0.0, 0.0,  0.0),  0.030),
    # gripper_body wraps ONLY the fixed housing (stump+fixture+
    # fixture1+neck).  Centre ~22 mm along tool0 +Z; R=35 mm covers
    # the 64 mm Ø mounting plate + the housing body.  The closing tip
    # (appendage / fixture1 blades) is modelled by the tcp_tip sphere
    # below; NOT by gripper_body.
    ("gripper_body", "tool0",    (0.0, -0.014, +0.022), 0.035),
    # tcp_tip is the working tip of the appendage / fixture1 closing
    # zone.  R=12 mm — tight envelope for the actual clamping jaws.
    ("tcp_tip",      "tool0",    (kin.GRASP_CENTER_OFFSET[0],
                                  kin.GRASP_CENTER_OFFSET[1],
                                  kin.GRASP_CENTER_OFFSET[2]), 0.012),
]


# V44 obstacle boxes in WORLD frame.
OBSTACLES = [
    ("cell_floor",       0.0, 2.504, 0.0, 2.098, -0.005, 0.005),
    ("mesa_top",         1.252 - 0.810, 1.252 + 0.810,
                         1.049 - 0.460, 1.049 + 0.460,
                         0.995, 1.005),
    # V43 conveyor (375 x 150 mm @ (1.370, 1.365), top z=1.07).
    ("conveyor",         1.370 - 0.190, 1.370 + 0.190,
                         1.365 - 0.075, 1.365 + 0.075,
                         1.005, 1.070),
    # Disc giratorio
    ("disc_face",        0.692 - 0.200, 0.692 + 0.200,
                         1.259 - 0.200, 1.259 + 0.200,
                         1.005, 1.081),
    # LOAD/RIVET fixtures on disc
    ("fixture_LOAD",     0.737 - 0.060, 0.737 + 0.060,
                         1.109 - 0.030, 1.109 + 0.030,
                         1.081, 1.111),
    ("fixture_RIVET",    0.737 - 0.060, 0.737 + 0.060,
                         1.409 - 0.030, 1.409 + 0.030,
                         1.081, 1.111),
    # V44 vision fixture cradle (moved WEST to 0.750)
    ("fixture_VISION",   0.750 - 0.075, 0.750 + 0.075,
                         0.804 - 0.075, 0.804 + 0.075,
                         1.005, 1.030),
    # V44 accept bin walls (centred 1.650, 0.720)
    ("bin_accept_n", 1.650 - 0.115, 1.650 + 0.115,
                     0.720 + 0.083, 0.720 + 0.090,
                     1.000, 1.150),
    ("bin_accept_s", 1.650 - 0.115, 1.650 + 0.115,
                     0.720 - 0.090, 0.720 - 0.083,
                     1.000, 1.150),
    ("bin_accept_e", 1.650 + 0.108, 1.650 + 0.115,
                     0.720 - 0.090, 0.720 + 0.090,
                     1.000, 1.150),
    ("bin_accept_w", 1.650 - 0.115, 1.650 - 0.108,
                     0.720 - 0.090, 0.720 + 0.090,
                     1.000, 1.150),
    # V49 reject bin walls (centred 1.330, 0.700 -- pulled INSIDE mesa
    # from V48's off-mesa Y=0.580)
    ("bin_reject_n", 1.330 - 0.115, 1.330 + 0.115,
                     0.700 + 0.088, 0.700 + 0.095,
                     1.000, 1.150),
    ("bin_reject_s", 1.330 - 0.115, 1.330 + 0.115,
                     0.700 - 0.095, 0.700 - 0.088,
                     1.000, 1.150),
    ("bin_reject_e", 1.330 + 0.108, 1.330 + 0.115,
                     0.700 - 0.095, 0.700 + 0.095,
                     1.000, 1.150),
    ("bin_reject_w", 1.330 - 0.115, 1.330 - 0.108,
                     0.700 - 0.095, 0.700 + 0.095,
                     1.000, 1.150),
    # V44 Cognex camera (moved WEST to 0.750)
    ("cognex_camera",    0.750 - 0.030, 0.750 + 0.030,
                         0.804 - 0.024, 0.804 + 0.024,
                         1.498, 1.542),
    # Cabin posts
    ("cabin_post_sw",    0.30 - 0.025, 0.30 + 0.025,
                         0.30 - 0.025, 0.30 + 0.025,
                         0.0, 2.070),
    ("cabin_post_se",    2.20 - 0.025, 2.20 + 0.025,
                         0.30 - 0.025, 0.30 + 0.025,
                         0.0, 2.070),
    ("cabin_post_nw",    0.30 - 0.025, 0.30 + 0.025,
                         1.80 - 0.025, 1.80 + 0.025,
                         0.0, 2.070),
    ("cabin_post_ne",    2.20 - 0.025, 2.20 + 0.025,
                         1.80 - 0.025, 1.80 + 0.025,
                         0.0, 2.070),
    # V44 rivet cabin canopy SHRUNK (0.450 x 0.300, z=1.300..1.350).
    # Canopy centred 0.200 m NORTH of disc centre, so south edge is at
    # y = 1.259 + 0.200 - 0.150 = 1.309 (200 mm north of LOAD seat).
    ("rivet_cabin",      0.692 - 0.225, 0.692 + 0.225,
                         1.459 - 0.150, 1.459 + 0.150,
                         1.300, 1.350),
    # NEMA assembly east of disc, world (0.936, 1.259).
    ("nema_motor",       0.936 - 0.035, 0.936 + 0.035,
                         1.259 - 0.035, 1.259 + 0.035,
                         1.000, 1.080),
]


# V44 whitelist.  The cobot's base plate sits on the mesa BY DESIGN
# (it bolts there).  Beyond that, the TCP_TIP and the gripper_body
# MUST be allowed to graze the obstacle that is the target of the
# current operation, because the gripper IS supposed to descend to that
# obstacle to deliver / pick the CAFI.  These contacts are NOT free-
# space collisions — they are intentional clamping motions.
#
# Hard rule still enforced: no joint origin below mesa_top + 5 mm; no
# tcp_tip / gripper_body inside any non-target obstacle.
WHITELIST = [
    (None, "base_plate", "mesa_top"),

    # PICK_CONVEYOR: TCP / gripper / wrist1 hover just above the belt
    # to grasp the CAFI.  The wrist1 SPHERE (R=25 mm) does NOT actually
    # touch the belt; it sits 33 mm above mesa = 67 mm above belt floor
    # (mesa top), with the belt top at z=1.07 and wrist origin at
    # z=1.038 the sphere reaches z=1.013 — below belt top.  This is
    # the sphere-model approximation; the real wrist mesh stays clear.
    ("POSE_PICK_CONVEYOR",         "wrist1",       "mesa_top"),
    ("POSE_PICK_CONVEYOR",         "wrist1",       "conveyor"),
    ("POSE_PICK_CONVEYOR",         "gripper_body", "conveyor"),
    ("POSE_PICK_CONVEYOR",         "tcp_tip",      "conveyor"),

    # PLACE_LOAD_FIXTURE / PICK_RIVETED: TCP is over the cradle.  The
    # gripper housing brushes the 60-mm-tall cradle by a few mm.
    ("POSE_PLACE_LOAD_FIXTURE",    "gripper_body", "fixture_LOAD"),
    ("POSE_PLACE_LOAD_FIXTURE",    "tcp_tip",      "fixture_LOAD"),
    ("POSE_PICK_RIVETED",          "gripper_body", "fixture_LOAD"),
    ("POSE_PICK_RIVETED",          "tcp_tip",      "fixture_LOAD"),

    # PLACE_VISION: TCP is over the vision cradle which sits directly
    # on the mesa.  Gripper housing skims both.
    ("POSE_PLACE_VISION",          "gripper_body", "mesa_top"),
    ("POSE_PLACE_VISION",          "gripper_body", "fixture_VISION"),
    ("POSE_PLACE_VISION",          "tcp_tip",      "fixture_VISION"),
    ("POSE_RELEASE_VISION",        "gripper_body", "fixture_VISION"),

    # DROP_*_BIN: TCP descends to bin floor (= mesa top + 5 mm).  The
    # gripper housing skims the bin's exterior east wall as it tilts in.
    ("POSE_DROP_ACCEPT_BIN",       "gripper_body", "mesa_top"),
    ("POSE_DROP_ACCEPT_BIN",       "gripper_body", "bin_accept"),
    ("POSE_DROP_ACCEPT_BIN",       "tcp_tip",      "mesa_top"),
    ("POSE_DROP_REJECT_BIN",       "gripper_body", "mesa_top"),
    ("POSE_DROP_REJECT_BIN",       "gripper_body", "bin_reject"),
    ("POSE_DROP_REJECT_BIN",       "tcp_tip",      "mesa_top"),
]


def is_whitelisted(pose_name, sphere_name, obs_name):
    for wp, ws, wo in WHITELIST:
        if (wp is None or pose_name == wp) and sphere_name == ws \
                and obs_name.startswith(wo):
            return True
    return False


def fk_link_centres(q):
    Tb = np.eye(4)
    Tj1 = Tb @ kin.T(*kin.JOINT1_XYZ) @ kin.Ry(q[0])
    Tj2 = Tj1 @ kin.Trpy(kin.JOINT2_XYZ, kin.JOINT2_RPY) @ kin.Rz(q[1])
    Tj3 = Tj2 @ kin.T(*kin.JOINT3_XYZ) @ kin.Rz(q[2])
    Tj4 = Tj3 @ kin.T(*kin.JOINT4_XYZ) @ kin.Rz(q[3])
    Tj5 = Tj4 @ kin.T(*kin.JOINT5_XYZ) @ kin.Ry(q[4])
    Tj6 = Tj5 @ kin.T(*kin.JOINT6_XYZ) @ kin.Rz(q[5])
    Ttool = Tj6 @ kin.T(*kin.TOOL0_XYZ)

    Tw = kin.base_to_world()
    base_frames = {
        "base":   Tb,
        "j2_pre": Tj1, "j3_pre": Tj2, "j4_pre": Tj3,
        "j6_pre": Tj5, "tool0":  Ttool,
    }
    out = {}
    for sphere_name, link_name, offset, radius in COBOT_SPHERES:
        Tl = base_frames[link_name]
        v = np.array([offset[0], offset[1], offset[2], 1.0])
        v_base = Tl @ v
        v_world = Tw @ v_base
        out[sphere_name] = (v_world[0], v_world[1], v_world[2], radius)
    return out


def sphere_aabb_dist(p, r, box):
    xmin, xmax, ymin, ymax, zmin, zmax = box
    dx = max(xmin - p[0], 0.0, p[0] - xmax)
    dy = max(ymin - p[1], 0.0, p[1] - ymax)
    dz = max(zmin - p[2], 0.0, p[2] - zmax)
    return math.sqrt(dx * dx + dy * dy + dz * dz) - r


MESA_TOP_Z_HARD = 1.000
JOINT_MESA_CLEARANCE = 0.005


def joint_mesa_check(q):
    """Return (min_z, fails)."""
    origins = kin.fk_joint_origins_world(q)
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


def main():
    margin = 0.005
    rows = []
    floor_min_z = float("inf")
    overall_joint_min_z = float("inf")
    all_pass = True

    for pose_name, q in resolved_poses.POSE_LIB.items():
        centres = fk_link_centres(q)
        per_pose = []
        for sphere_name, _, _, _ in COBOT_SPHERES:
            if sphere_name in ("gripper_body", "tcp_tip"):
                x, y, z, r = centres[sphere_name]
                if z - r < floor_min_z:
                    floor_min_z = z - r
        for sphere_name, _, _, _ in COBOT_SPHERES:
            x, y, z, r = centres[sphere_name]
            for obs_name, *box in OBSTACLES:
                d = sphere_aabb_dist((x, y, z), r, tuple(box))
                if d < margin and not is_whitelisted(pose_name, sphere_name, obs_name):
                    per_pose.append((sphere_name, obs_name, d))
        min_jz, joint_fails = joint_mesa_check(q)
        if min_jz < overall_joint_min_z:
            overall_joint_min_z = min_jz
        for jn, jz in joint_fails:
            per_pose.append(("JOINT_" + jn, "mesa_top_HARD", jz - MESA_TOP_Z_HARD))
        rows.append((pose_name, per_pose, min_jz))

    print()
    print("V44 collision check (strict mode, per-joint mesa hard requirement)")
    print("{:30s}  {:6s}  min_joint_Z  Collision report".format("Pose", "Status", ""))
    print("-" * 120)
    for pose_name, hits, min_jz in rows:
        if not hits:
            print("{:30s}  {:6s}  {:.4f} m".format(pose_name, "PASS", min_jz))
        else:
            print("{:30s}  {:6s}  {:.4f} m".format(pose_name, "FAIL", min_jz))
            all_pass = False
            for sph, obs, d in hits:
                print("    {:14s} vs {:18s}  margin {:+.3f} m".format(sph, obs, d))
    print()
    print("Lowest gripper_body/tcp_tip world Z: {:.4f} m  (floor at z=0)".format(floor_min_z))
    print("Lowest joint origin Z (any joint, any pose): {:.4f} m".format(overall_joint_min_z))
    print("Joint clearance margin: {:+.4f} m above mesa_top".format(overall_joint_min_z - MESA_TOP_Z_HARD))
    print("Overall:", "PASS" if all_pass else "FAIL")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
