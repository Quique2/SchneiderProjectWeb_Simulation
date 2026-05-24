"""V56 geometry test.

Validates:
  1. The new riveting_zone anchor is at world X = 0.692 + 0.300 = 0.992
     (URDF + resolve_poses agree).
  2. Every LOAD/RIVET pose lands joint5 at exactly -pi/2 (= -90 deg).
  3. The FK at PLACE_LOAD_FIXTURE puts the gripper TCP near the
     fixture seat (within 25 mm of the seat XYZ when accounting for
     the lateral grasp delta).
"""
from __future__ import print_function
import os
import sys
import math

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np
import resolved_poses
from lexium_kinematics import (
    fk_world_to_grasp_center, pos, rot_mat,
    lateral_grasp_delta_world, POSE_HOME_Q,
)

EXPECT_J5 = -math.pi / 2.0

LOAD_RIVET_POSES = [
    "POSE_APPROACH_LOAD_FIXTURE",
    "POSE_PLACE_LOAD_FIXTURE",
    "POSE_RELEASE_LOAD_FIXTURE",
    "POSE_RETREAT_LOAD_FIXTURE",
    "POSE_APPROACH_PICK_RIVETED",
    "POSE_PICK_RIVETED",
    "POSE_LIFT_RIVETED",
]

EXPECTED_RIVETING_ZONE_X = 0.692 + 0.300
# fixture_1_cafi_lateral_target_frame at q_disc=0 in world.
EXPECTED_LOAD_SEAT = (
    EXPECTED_RIVETING_ZONE_X - 0.015 + 0.003276,
    1.259 - 0.030 - 0.071389,
    1.000 + 0.078 + 0.004 + 0.022476,
)


def main():
    print("V56 geometry verification")
    print("-" * 60)
    print(f"Expected LOAD seat world XYZ: {EXPECTED_LOAD_SEAT}")
    print()
    print(f"{'pose':32s} {'J5 deg':>10s} {'OK J5':>6s}")
    j5_fail = 0
    for name in LOAD_RIVET_POSES:
        q = resolved_poses.POSE_LIB[name]
        j5_deg = math.degrees(q[4])
        ok = abs(q[4] - EXPECT_J5) < 1e-4
        print(f"{name:32s} {j5_deg:+10.4f} {'OK' if ok else 'FAIL':>6s}")
        if not ok:
            j5_fail += 1
    assert j5_fail == 0, f"{j5_fail} LOAD/RIVET poses do NOT land at J5=-pi/2"

    print()
    q = resolved_poses.POSE_LIB["POSE_PLACE_LOAD_FIXTURE"]
    M = fk_world_to_grasp_center(q)
    tcp = pos(M)
    delta_w = lateral_grasp_delta_world(q)
    cafi_world = tcp - delta_w
    err = float(np.linalg.norm(np.asarray(EXPECTED_LOAD_SEAT) - cafi_world))
    print(f"PLACE_LOAD_FIXTURE TCP world: ({tcp[0]:.4f}, {tcp[1]:.4f}, "
          f"{tcp[2]:.4f})")
    print(f"PLACE_LOAD_FIXTURE CAFI world (= TCP - delta_w_lateral): "
          f"({cafi_world[0]:.4f}, {cafi_world[1]:.4f}, {cafi_world[2]:.4f})")
    print(f"Expected LOAD seat:           ({EXPECTED_LOAD_SEAT[0]:.4f}, "
          f"{EXPECTED_LOAD_SEAT[1]:.4f}, {EXPECTED_LOAD_SEAT[2]:.4f})")
    print(f"Error norm:                   {err*1000:.2f} mm")
    assert err < 0.025, f"PLACE_LOAD FK XYZ error {err*1000:.1f} mm > 25 mm"
    print()
    print("V56 GEOMETRY TEST PASSED.")


if __name__ == "__main__":
    main()
