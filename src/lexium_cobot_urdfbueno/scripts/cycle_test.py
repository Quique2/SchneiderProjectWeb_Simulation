#!/usr/bin/env python3
"""Trigger the V33 plant through one full CAFI cycle and capture
screenshots + a /joint_states log for post-mortem analysis.

This script is intentionally non-invasive: it only publishes
/operator/spawn_cafi (and optionally /robot/request_home for the
opening shot).  The state_manager / object_manager drive the rest of
the cycle.
"""
import argparse
import os
import subprocess
import sys
import time

import rospy
from std_msgs.msg import Empty, Bool, String
from sensor_msgs.msg import JointState


SCREENSHOT_DIR = os.environ.get(
    "EVIDENCE_DIR",
    os.path.expanduser("~/v34_ws/evidence/screenshots"))


def shot(name):
    os.makedirs(SCREENSHOT_DIR, exist_ok=True)
    path = os.path.join(SCREENSHOT_DIR, name + ".png")
    rospy.loginfo("[shot] %s", path)
    try:
        subprocess.run(["gnome-screenshot", "-f", path],
                       check=False, timeout=10)
    except Exception as e:  # noqa: BLE001
        rospy.logwarn("gnome-screenshot failed: %s", e)
    return path


class CycleTester(object):
    def __init__(self, n_cycles=3):
        rospy.init_node("v33_cycle_tester", anonymous=False)
        self.n_cycles = n_cycles
        self.last_stage = ""
        self.last_task = ""
        self.last_motion_done = False
        self.shots_per_stage = {}
        rospy.Subscriber("/cell/cycle_stage", String, self._on_stage)
        rospy.Subscriber("/robot/current_task", String, self._on_task)
        rospy.Subscriber("/robot/motion_done", Bool, self._on_motion)
        self.pub_spawn = rospy.Publisher(
            "/operator/spawn_cafi", Empty, queue_size=2)
        self.pub_home  = rospy.Publisher(
            "/robot/request_home", Empty, queue_size=2)

    def _on_stage(self, m):
        new_stage = m.data
        if new_stage != self.last_stage:
            rospy.loginfo("[CYCLE] cycle_stage %s -> %s",
                          self.last_stage, new_stage)
            self.last_stage = new_stage
            if new_stage and new_stage != "IDLE":
                idx = self.shots_per_stage.get(new_stage, 0)
                self.shots_per_stage[new_stage] = idx + 1
                # Rate-limit screenshots per stage
                if idx < 3:
                    shot(f"30_cycle_{new_stage}_{idx}")

    def _on_task(self, m):
        if m.data != self.last_task:
            rospy.loginfo("[CYCLE] robot/current_task %s -> %s",
                          self.last_task, m.data)
            self.last_task = m.data

    def _on_motion(self, m):
        self.last_motion_done = bool(m.data)

    def run(self):
        rospy.loginfo("[CYCLE] waiting 4 s for nodes to settle")
        rospy.sleep(4.0)
        shot("20_t0_home_pose")

        # Request HOME (idempotent if already there)
        self.pub_home.publish(Empty())
        rospy.sleep(3.0)
        shot("21_after_home_request")

        # Spawn N CAFIs spaced out
        for i in range(self.n_cycles):
            rospy.loginfo("[CYCLE] spawn CAFI #%d", i + 1)
            self.pub_spawn.publish(Empty())
            shot(f"22_spawn_{i:02d}")
            # Let it travel down the conveyor and start cycling
            rospy.sleep(35.0)  # rough budget per cycle

        # Final settle
        rospy.sleep(15.0)
        shot("90_cycle_end_state")
        rospy.loginfo("[CYCLE] done; stage hits: %s", self.shots_per_stage)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cycles", type=int, default=3,
                   help="how many CAFIs to spawn (default 3)")
    args, _ = p.parse_known_args()
    tester = CycleTester(n_cycles=args.cycles)
    tester.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
