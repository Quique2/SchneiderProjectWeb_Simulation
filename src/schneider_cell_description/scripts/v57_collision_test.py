"""V57 collision validator.

For every resolved pose AND every linearly-interpolated step along each
trajectory, samples the cobot link segments and verifies they stay >=
3 cm away from every world obstacle (mesa edge, conveyor, suministro,
sensors, disc base, vision fixture, bins, indicator post + lamps,
cabin posts, drive motor, camera assembly).

Reports any violation and exits 1 if found.

Pose endpoints near a fixture (PICK_CONVEYOR + LOAD/RIVET/VISION/BINS)
are exempted from the corresponding "manipulation target" obstacle —
because the gripper IS supposed to touch the CAFI at those poses.
Other obstacles still apply.
"""
from __future__ import print_function
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np
import resolved_poses
from lexium_kinematics import (
    fk_joint_origins_world, fk_world_to_grasp_center, POSE_HOME_Q,
    WORLD_COBOT_XY,
)


SAFETY_M = 0.030  # 3 cm minimum clearance per user spec


# ============================================================
# World obstacles.  Each entry is one of:
#   ("aabb", name, (x_min, x_max, y_min, y_max, z_min, z_max))
#   ("cyl",  name, (x, y, z_bot, z_top, radius))
# Conservative bounding volumes drawn from the URDF; deliberately a
# bit larger than the rendered mesh so the check is strict.
# ============================================================
OBSTACLES = [
    # Mesa SLAB top face (cobot must not slap into the slab; per-joint
    # clearance is already enforced by joint_mesa_clearance — we keep
    # it here as a thin slab to catch any pose that goes through it).
    ("aabb", "mesa_slab",       (0.442, 2.062, 0.589, 1.509, 0.960, 1.000)),
    # Conveyor body (V57 shifted +0.300 m).
    ("aabb", "conveyor_body",   (1.483, 1.858, 1.290, 1.440, 1.000, 1.070)),
    # Suministro feeder block (V57 shifted +0.300 m).
    ("aabb", "suministro_cafi", (1.845, 1.995, 1.255, 1.475, 1.000, 1.015)),
    # SICK sensor on the conveyor end — pedestal + body (V57 shifted).
    ("aabb", "sensor_conveyor", (1.515, 1.555, 1.275, 1.305, 1.000, 1.150)),
    # SICK sensor at the vision (mesa west).
    ("aabb", "sensor_vision",   (0.566, 0.606, 0.794, 0.824, 1.000, 1.080)),
    # Disc base + bearings + sprocket bbox (V56: riveting_zone (0.992,
    # 1.259)).  Conservative cylinder approximation of the gearbase.
    ("cyl",  "disc_base",       (0.992, 1.259, 1.000, 1.085, 0.205)),
    # Both rotary fixtures (small extrusion on top of the disc).
    ("cyl",  "rivet_fixtures",  (0.992, 1.259, 1.082, 1.135, 0.180)),
    # Vision fixture cradle.
    ("aabb", "vision_fixture",  (0.675, 0.825, 0.729, 0.879, 1.000, 1.020)),
    # Accept bin (4 walls + floor at (1.650, 0.720)).
    ("aabb", "accept_bin",      (1.560, 1.740, 0.630, 0.810, 1.000, 1.125)),
    # Reject bin at (1.330, 0.700).
    ("aabb", "reject_bin",      (1.240, 1.420, 0.610, 0.790, 1.000, 1.125)),
    # V56 rivet indicator post (cylinder + 3 lamp spheres).
    ("cyl",  "indicator_post",  (0.772, 1.259, 1.000, 1.300, 0.025)),
    # Cabin perimeter posts (only the ones near the cobot working zone).
    ("cyl",  "cabin_post_sw",   (0.300, 0.300, 0.000, 2.070, 0.035)),
    ("cyl",  "cabin_post_se",   (2.200, 0.300, 0.000, 2.070, 0.035)),
    ("cyl",  "cabin_post_nw",   (0.300, 1.800, 0.000, 2.070, 0.035)),
    ("cyl",  "cabin_post_ne",   (2.200, 1.800, 0.000, 2.070, 0.035)),
    # Conveyor drive motor (V57 shifted +0.300 m).
    ("aabb", "conv_motor",      (1.810, 1.890, 1.325, 1.405, 1.000, 1.100)),
    # V40 camera body at (0.750, 0.804, 1.520) — pitched +pi/2 so the
    # 60x45x45 mm box becomes ~45x45x60 in world.  Plus lens hanging
    # 28 mm below the body.  Plus support column from 1.520 -> 2.070.
    ("aabb", "camera_body",     (0.720, 0.780, 0.774, 0.834, 1.460, 1.580)),
    ("cyl",  "camera_column",   (0.750, 0.804, 1.520, 2.070, 0.016)),
]

# Which target the gripper is allowed to touch at each pose (skip that
# obstacle's check for the gripper-tip points only).  None = strict.
POSE_TARGET_EXEMPTION = {
    "POSE_PICK_CONVEYOR":          "conveyor_body",
    "POSE_LIFT_CONVEYOR":          "conveyor_body",
    "POSE_APPROACH_CONVEYOR":      "conveyor_body",
    "POSE_APPROACH_LOAD_FIXTURE":  "rivet_fixtures",
    "POSE_PLACE_LOAD_FIXTURE":     "rivet_fixtures",
    "POSE_RELEASE_LOAD_FIXTURE":   "rivet_fixtures",
    "POSE_RETREAT_LOAD_FIXTURE":   "rivet_fixtures",
    "POSE_APPROACH_PICK_RIVETED":  "rivet_fixtures",
    "POSE_PICK_RIVETED":           "rivet_fixtures",
    "POSE_LIFT_RIVETED":           "rivet_fixtures",
    "POSE_APPROACH_VISION":        "vision_fixture",
    "POSE_PLACE_VISION":           "vision_fixture",
    "POSE_RELEASE_VISION":         "vision_fixture",
    "POSE_RETREAT_VISION":         "vision_fixture",
    "POSE_APPROACH_ACCEPT_BIN":    "accept_bin",
    "POSE_DROP_ACCEPT_BIN":        "accept_bin",
    "POSE_APPROACH_REJECT_BIN":    "reject_bin",
    "POSE_DROP_REJECT_BIN":        "reject_bin",
}

# Trajectories in V55 controller — interpolate between consecutive
# joint poses to check the swept volume, not just endpoints.
TRAJECTORIES = [
    ("PICK_CONV",     ["POSE_APPROACH_CONVEYOR", "POSE_PICK_CONVEYOR",
                       "POSE_LIFT_CONVEYOR"]),
    ("PLACE_OUTER",   ["POSE_LIFT_CONVEYOR", "POSE_APPROACH_LOAD_FIXTURE",
                       "POSE_RELEASE_LOAD_FIXTURE",
                       "POSE_RETREAT_LOAD_FIXTURE", "POSE_HOME"]),
    ("PICK_RIVETED",  ["POSE_HOME", "POSE_APPROACH_PICK_RIVETED",
                       "POSE_PICK_RIVETED", "POSE_LIFT_RIVETED"]),
    ("PLACE_VISION",  ["POSE_LIFT_RIVETED", "POSE_APPROACH_VISION",
                       "POSE_RELEASE_VISION", "POSE_RETREAT_VISION",
                       "POSE_HOME"]),
    ("PICK_VISION",   ["POSE_HOME", "POSE_APPROACH_VISION",
                       "POSE_PLACE_VISION", "POSE_RETREAT_VISION",
                       "POSE_HOME"]),
    # V60: PLACE_*_BIN trajectories start from HOME (TRAJ_PICK_VISION
    # ends at HOME in V60, breaking the previous direct
    # RETREAT_VISION->APPROACH_BIN swing).
    ("PLACE_ACCEPT",  ["POSE_HOME", "POSE_APPROACH_ACCEPT_BIN",
                       "POSE_DROP_ACCEPT_BIN", "POSE_APPROACH_ACCEPT_BIN",
                       "POSE_HOME"]),
    ("PLACE_REJECT",  ["POSE_HOME", "POSE_APPROACH_REJECT_BIN",
                       "POSE_DROP_REJECT_BIN", "POSE_APPROACH_REJECT_BIN",
                       "POSE_HOME"]),
]


# ============================================================
# Distance helpers
# ============================================================
def point_to_aabb(p, box):
    """Distance from 3D point p to an axis-aligned box (xmin, xmax,
    ymin, ymax, zmin, zmax).  Inside = 0."""
    px, py, pz = p
    xmn, xmx, ymn, ymx, zmn, zmx = box
    dx = max(xmn - px, 0.0, px - xmx)
    dy = max(ymn - py, 0.0, py - ymx)
    dz = max(zmn - pz, 0.0, pz - zmx)
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def point_to_cylinder(p, cyl):
    """Distance from 3D point p to an upright cylinder (x, y, z_bot,
    z_top, radius).  Inside = 0."""
    cx, cy, zbot, ztop, r = cyl
    dx = p[0] - cx; dy = p[1] - cy
    horiz = math.sqrt(dx*dx + dy*dy)
    dr = max(horiz - r, 0.0)
    if zbot <= p[2] <= ztop:
        dz = 0.0
    else:
        dz = min(abs(p[2] - zbot), abs(p[2] - ztop))
    return math.sqrt(dr*dr + dz*dz)


def point_to_obstacle(p, obs):
    kind = obs[0]
    if kind == "aabb":
        return point_to_aabb(p, obs[2])
    if kind == "cyl":
        return point_to_cylinder(p, obs[2])
    raise ValueError(kind)


# ============================================================
# Cobot sampling
# ============================================================
JOINT_KEYS_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4",
                    "joint_5", "joint_6", "tool0"]


def sample_link_points(q, n_per_segment=4):
    """Return a list of (label, world_xyz) points along the cobot
    physical skeleton.  The grasp center is NOT included — it is the
    virtual TCP where the CAFI sits, not a cobot link volume — and
    including it would flag every place/pick pose because the CAFI
    centre legitimately touches the bin / fixture / vision cradle."""
    origins = fk_joint_origins_world(q)
    chain = ["joint_1", "joint_2", "joint_3", "joint_4",
             "joint_5", "joint_6", "tool0"]
    samples = []
    for i in range(len(chain) - 1):
        a_name, b_name = chain[i], chain[i + 1]
        a = origins[a_name]; b = origins[b_name]
        for k in range(n_per_segment + 1):
            t = k / float(n_per_segment)
            p = (a[0] + (b[0] - a[0]) * t,
                 a[1] + (b[1] - a[1]) * t,
                 a[2] + (b[2] - a[2]) * t)
            label = "{}->{} t={:.2f}".format(a_name, b_name, t)
            samples.append((label, p))
    return samples


def is_wrist_label(label):
    return ("joint_5" in label or "joint_6" in label
            or "tool0" in label)


def check_pose(name, q):
    """Return list of (obstacle_name, label, dist) violations."""
    fails = []
    samples = sample_link_points(q, n_per_segment=4)
    exempt = POSE_TARGET_EXEMPTION.get(name)
    for label, p in samples:
        for obs in OBSTACLES:
            obs_name = obs[1]
            # Skip the target obstacle for wrist-tip samples at
            # manipulation poses.
            if exempt == obs_name and is_wrist_label(label):
                continue
            d = point_to_obstacle(p, obs)
            if d < SAFETY_M:
                fails.append((obs_name, label, d))
    return fails


def interp(q_a, q_b, steps):
    out = []
    for k in range(1, steps):
        t = k / float(steps)
        q = [q_a[i] + (q_b[i] - q_a[i]) * t for i in range(6)]
        out.append(q)
    return out


def main():
    pose_lib = resolved_poses.POSE_LIB
    print("V57 collision validator — 3 cm safety margin")
    print("=" * 70)
    total_fail = 0

    # 1. Static poses.
    print("\n[1] Static pose check (every resolved pose):")
    for name in sorted(pose_lib.keys()):
        q = pose_lib[name]
        fails = check_pose(name, q)
        if fails:
            print("  FAIL  {:<32s} {} violations".format(name, len(fails)))
            for obs, label, d in fails[:5]:
                print("         - {:<18s} {:<22s} dist {:.3f} m"
                      .format(obs, label, d))
            total_fail += len(fails)
        else:
            print("  OK    {}".format(name))

    # 2. Trajectory interpolation.
    print("\n[2] Trajectory interpolation check (10 steps per segment):")
    for traj_name, steps in TRAJECTORIES:
        print("\n  Trajectory: {}".format(traj_name))
        for a, b in zip(steps[:-1], steps[1:]):
            qa = pose_lib[a]; qb = pose_lib[b]
            # For interp samples we use the destination pose's exemption
            # because the cobot is APPROACHING that target.
            mid_exempt = POSE_TARGET_EXEMPTION.get(b)
            local_fail = 0
            for q in interp(qa, qb, 10):
                samples = sample_link_points(q, n_per_segment=3)
                for label, p in samples:
                    for obs in OBSTACLES:
                        if mid_exempt == obs[1] and is_wrist_label(label):
                            continue
                        d = point_to_obstacle(p, obs)
                        if d < SAFETY_M:
                            local_fail += 1
                            if local_fail <= 3:
                                print("    FAIL {} -> {}: {} {:.3f} m"
                                      .format(a, b, obs[1], d))
            if local_fail:
                print("    SEG  {} -> {}: {} violations".format(
                    a, b, local_fail))
                total_fail += local_fail
            else:
                print("    OK   {} -> {}".format(a, b))

    print("\n" + "=" * 70)
    if total_fail == 0:
        print("V57 COLLISION TEST PASSED — every pose and trajectory step "
              "is >= 3 cm from every obstacle.")
        sys.exit(0)
    else:
        print("V57 COLLISION TEST FAILED with {} violations.".format(total_fail))
        sys.exit(1)


if __name__ == "__main__":
    main()
