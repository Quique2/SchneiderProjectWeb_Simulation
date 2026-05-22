#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_gripper_sim.gripper_sim_node  --  V23

V22 declared grasp_confirmed purely from current_task + the existence of
a CAFI in the matching logical location. That produced the teletransport-
feel pick the user is rejecting (the CAFI snapped to the gripper even
when the cobot TCP was nowhere near it).

V23 reverts to a STRICTLY GEOMETRIC grasp confirmation against the
gripper_base_grasp_center TF frame. This works in V23 because
robot_controller_node.py now uses IK-validated poses that physically
place grasp_center on the CAFI at the PICK pose (V20/V21/V22 had
hand-tuned joint poses that landed elsewhere -- that was the original
cause of the failed geometric check, which V22 chose to bypass instead
of fix).

V23 grasp_confirmed rules (ALL must be true):
  1. The robot's current_task is a pick action  (gate: never grasp idle)
  2. The TF gripper_base_grasp_center is available
  3. There is at least one CAFI whose world-XYZ is within tolerance of
     grasp_center                                  (GRASP_TOL_X/Y/Z)
  4. That CAFI is in a graspable location           (on_conveyor +
     at_sensor for pick_conv; fixture_outer + riveted for pick_riveted)
  5. The jaw is closing or closed                  (jaw_pos > -0.005)

If any condition fails, grasp_confirmed stays False -- the cobot never
"steals" the CAFI from afar.

V23 also publishes the chosen CAFI's id to /objects/attach so the
object_manager can capture the full T_gripper_cafi transform and follow
the gripper rigidly thereafter.
"""

import json
import math
import threading

import rospy
import tf2_ros
from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import JointState


# V43: the new cobot URDF (lexium_cobot_with_final_gripper) replaces the
# old gripper_base_jaw_joint with appendage_prismatic_joint.  After macro
# prefixing the world name is lexium_cobot_appendage_prismatic_joint.
# Stroke: 0 (closed) ... +0.028 m (open).  Axis (0,1,0) per the new URDF.
# V48 raises the URDF upper limit from 0.015 to 0.028 m so the open jaw
# leaves ~28 mm clearance from the CAFI east face during approach.
JAW_JOINT_NAME = "lexium_cobot_appendage_prismatic_joint"
JAW_OPEN_POS   = 0.028           # 28 mm open  (V48 URDF upper limit; was 0.015)
JAW_CLOSED_POS = 0.000           # 0 mm closed (URDF lower limit)
JAW_RATE       = 0.040           # m/s (28 mm in ~0.7 s, smooth visible motion)

# Jaw thresholds (interpreted under the new closed=0/open=+ convention)
JAW_CLOSED_THRESHOLD = 0.004     # < 4 mm -> jaw is closing or closed
JAW_OPEN_THRESHOLD   = 0.020     # > 20 mm -> jaw is open enough to release (V48: was 0.012)

# V43: TCP frame name in the new URDF.  The whole planta addresses the
# gripper's working centre by this name; it is the alias for tcp_link
# co-located with the user-tuned working centre of the gripper.
GRASP_TCP_FRAME = "lexium_cobot_tcp_grasp_center"

# V25 geometric grasp tolerance (m).  The CAFI bbox is 123 x 88 x 25 (mm)
# centered on (c.x, c.y, c.z) after the marker-rendering fix.  At PICK
# pose the grasp_center is 40 mm above the CAFI top in the Z axis (the
# gripper hangs above the part to avoid conveyor collision), and the IK
# delivers ~ 0.1 mm XY error.  Tolerance +- 0.100 m in each axis covers
# all of this.  This is a SIM-grade clamp envelope -- the real physical
# clamp would not need this much, but the sim sees only the IK target
# vs CAFI position, not the actual jaw closure dynamics.
GRASP_TOL_X = 0.100
GRASP_TOL_Y = 0.100
GRASP_TOL_Z = 0.100

# Which current_task values are pick actions
# V28: added "pick_vision" so the gripper can confirm grasp on the CAFI
# that is sitting at the vision fixture after inspection (post-rivet).
PICK_TASKS = ("pick_conv", "pick_riveted", "pick_vision")

# V26: CAFI locations are now "in_fixture_A" / "in_fixture_B" instead of
# the old outer/inner labels.  The gripper only checks distance to
# grasp_center, so both fixture ids are graspable for the riveted pick.
# V28: pick_vision targets CAFIs whose location is "at_vision" (the
# state set by object_manager when the CAFI is released onto the
# vision fixture).
GRASPABLE_LOCATIONS = {
    "pick_conv":    {"on_conveyor"},
    "pick_riveted": {"in_fixture_A", "in_fixture_B"},
    "pick_vision":  {"at_vision"},
}

TICK_HZ = 30.0


class GripperSim(object):

    def __init__(self):
        rospy.init_node("schneider_gripper_sim", anonymous=False)

        self.jaw_pos       = JAW_OPEN_POS
        self.jaw_target    = JAW_OPEN_POS
        self.state         = "OPEN"
        self.grasp_confirmed = False

        self.current_task = "idle"

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.cafis = []
        self.lock = threading.Lock()

        # Pubs
        self.pub_jaw_joint = rospy.Publisher(
            "/gripper/jaw_joint", JointState, queue_size=2)
        self.pub_state     = rospy.Publisher(
            "/gripper/state", String, queue_size=1, latch=True)
        self.pub_grasp     = rospy.Publisher(
            "/gripper/grasp_confirmed", Bool, queue_size=1, latch=True)
        self.pub_release   = rospy.Publisher(
            "/gripper/release_done", Empty, queue_size=2)
        self.pub_attach    = rospy.Publisher(
            "/objects/attach", String, queue_size=2)
        self.pub_detach    = rospy.Publisher(
            "/objects/detach", Empty, queue_size=2)

        # Subs
        rospy.Subscriber("/gripper/open_cmd",  Empty, self._cb_open)
        rospy.Subscriber("/gripper/close_cmd", Empty, self._cb_close)
        rospy.Subscriber("/objects/cafi_states", String, self._cb_cafis)
        rospy.Subscriber("/robot/current_task", String, self._cb_task)

        self.pub_state.publish(String(data="OPEN"))
        self.pub_grasp.publish(Bool(data=False))
        rospy.loginfo("[GRIP] V23 gripper_sim init - GEOMETRIC grasp "
                      "(task gate: %s, tol +/- (%.0f, %.0f, %.0f) mm)",
                      "|".join(PICK_TASKS),
                      GRASP_TOL_X * 1000, GRASP_TOL_Y * 1000,
                      GRASP_TOL_Z * 1000)

    def _cb_cafis(self, m):
        try:
            self.cafis = json.loads(m.data) or []
        except Exception:
            self.cafis = []

    def _cb_task(self, m):
        new_task = m.data
        if new_task != self.current_task:
            rospy.loginfo("[GRIP] current_task: %s -> %s",
                          self.current_task, new_task)
            self.current_task = new_task

    def _cb_open(self, _m):
        with self.lock:
            self.jaw_target = JAW_OPEN_POS
            self.state = "MOVING"
            self.pub_state.publish(String(data=self.state))
            rospy.loginfo("[GRIP] OPEN cmd received -> jaw_target=%.3f "
                          "(task=%s)", JAW_OPEN_POS, self.current_task)

    def _cb_close(self, _m):
        with self.lock:
            self.jaw_target = JAW_CLOSED_POS
            self.state = "MOVING"
            self.pub_state.publish(String(data=self.state))
            rospy.loginfo("[GRIP] CLOSE cmd received -> jaw_target=%.3f "
                          "(task=%s)", JAW_CLOSED_POS, self.current_task)

    # =========================================================
    # GEOMETRIC GRASP CANDIDATE (V23)
    # =========================================================
    def _grasp_candidate(self):
        """Return (cafi_id, reason) of the CAFI the gripper currently
        EARNS by being physically over it, or (None, None).

        V43: TF frame name now resolves to GRASP_TCP_FRAME
        ("lexium_cobot_tcp_grasp_center") which is the user-defined
        working centre of the new gripper.
        """
        if self.current_task not in PICK_TASKS:
            return None, "gate-closed (task=%s)" % self.current_task

        try:
            t = self.tf_buf.lookup_transform(
                "world", GRASP_TCP_FRAME,
                rospy.Time(0), rospy.Duration(0.05))
        except Exception as e:
            rospy.logdebug("[GRIP] grasp_center TF unavailable: %s", e)
            return None, "no-tf"

        gx = t.transform.translation.x
        gy = t.transform.translation.y
        gz = t.transform.translation.z

        graspable = GRASPABLE_LOCATIONS.get(self.current_task, set())
        best = None
        best_d = float("inf")
        nearest = None
        nearest_d = float("inf")
        for c in self.cafis:
            if c.get("location") not in graspable:
                continue
            cx = float(c.get("x", 0.0))
            cy = float(c.get("y", 0.0))
            cz = float(c.get("z", 0.0))
            dx = cx - gx; dy = cy - gy; dz = cz - gz
            dnorm = math.sqrt(dx*dx + dy*dy + dz*dz)
            if dnorm < nearest_d:
                nearest_d = dnorm
                nearest = (c.get("id"), dx, dy, dz)
            if (abs(dx) < GRASP_TOL_X and abs(dy) < GRASP_TOL_Y
                    and abs(dz) < GRASP_TOL_Z):
                if dnorm < best_d:
                    best = c; best_d = dnorm
        if best is None:
            reason = ("no CAFI in tol  grasp=(%.3f,%.3f,%.3f)" %
                      (gx, gy, gz))
            if nearest:
                reason += ("  nearest id=%s dist=%.3f m d=(%.3f,%.3f,%.3f)" %
                           (nearest[0], nearest_d,
                            nearest[1], nearest[2], nearest[3]))
            return None, reason
        return (best.get("id"),
                "geom: |dx,dy,dz|=(%.3f,%.3f,%.3f) m  dnorm=%.3f" %
                (best["x"] - gx, best["y"] - gy, best["z"] - gz, best_d))

    # =========================================================
    # TICK
    # =========================================================
    def tick(self, dt):
        with self.lock:
            # 1. Jaw kinematic step
            diff = self.jaw_target - self.jaw_pos
            step = JAW_RATE * dt
            if abs(diff) <= step:
                self.jaw_pos = self.jaw_target
            else:
                self.jaw_pos += step if diff > 0 else -step

            was_closed = (self.state == "CLOSED")
            was_open   = (self.state == "OPEN")
            if abs(self.jaw_pos - JAW_CLOSED_POS) < 1e-4:
                self.state = "CLOSED"
            elif abs(self.jaw_pos - JAW_OPEN_POS) < 1e-4:
                self.state = "OPEN"
            else:
                self.state = "MOVING"

            # 2. Geometric grasp evaluation
            # V43: under the new "closed=0 / open=+0.015" convention the
            # jaw is closing/closed when jaw_pos is BELOW JAW_CLOSED_THRESHOLD
            # (V22-V42 had it inverted because the old axis was -Y).
            cand_id, cand_reason = self._grasp_candidate()
            jaw_closing = (self.jaw_pos < JAW_CLOSED_THRESHOLD)
            new_grasp = jaw_closing and (cand_id is not None)

            if new_grasp and not self.grasp_confirmed:
                rospy.loginfo("[GRIP] jaw closing (pos=%.4f) AND geometric "
                              "match -> grasp CONFIRMED cafi_id=%s reason=%s",
                              self.jaw_pos, cand_id, cand_reason)
                self.grasp_confirmed = True
                self.pub_grasp.publish(Bool(data=True))
                self.pub_attach.publish(String(data=json.dumps({
                    "cafi_id": cand_id,
                    "link": GRASP_TCP_FRAME,
                })))
                rospy.loginfo("[GRIP] /objects/attach published cafi_id=%s "
                              "(object_manager will capture T_gripper_cafi)",
                              cand_id)
            elif not new_grasp and self.grasp_confirmed:
                if self.jaw_pos > JAW_OPEN_THRESHOLD:
                    rospy.loginfo("[GRIP] jaw opening (pos=%.4f) -> grasp "
                                  "RELEASED", self.jaw_pos)
                    self.grasp_confirmed = False
                    self.pub_grasp.publish(Bool(data=False))
                    self.pub_detach.publish(Empty())
                    self.pub_release.publish(Empty())

            # 3. Throttle a debug log when waiting (helps diagnose missed
            # grasp during tuning).  V43: jaw_pos < threshold means closed.
            if (self.current_task in PICK_TASKS
                    and self.jaw_pos < JAW_CLOSED_THRESHOLD
                    and not self.grasp_confirmed):
                rospy.logwarn_throttle(2.0,
                    "[GRIP] jaw closed but no geometric match yet (%s)",
                    cand_reason)

            # 4. State change announcement
            if self.state == "CLOSED" and not was_closed:
                self.pub_state.publish(String(data=self.state))
                rospy.loginfo("[GRIP] jaw CLOSED (pos=%.4f)", self.jaw_pos)
            elif self.state == "OPEN" and not was_open:
                self.pub_state.publish(String(data=self.state))
                rospy.loginfo("[GRIP] jaw OPEN (pos=%.4f)", self.jaw_pos)

            # 5. Joint publish for fuser
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = [JAW_JOINT_NAME]
            js.position = [self.jaw_pos]
            self.pub_jaw_joint.publish(js)

    def run(self):
        rate = rospy.Rate(TICK_HZ)
        dt = 1.0 / TICK_HZ
        while not rospy.is_shutdown():
            self.tick(dt)
            rate.sleep()


if __name__ == "__main__":
    try:
        GripperSim().run()
    except rospy.ROSInterruptException:
        pass
