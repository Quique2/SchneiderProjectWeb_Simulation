#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_object_manager.object_manager_node  --  V23

V22 tracked each CAFI as (x, y, z, yaw) only.  On attach it set
location="in_gripper" and then on every tick copied the gripper's world
TRANSLATION into the CAFI position -- the CAFI inherited the gripper's
position but not its orientation, and a yaw-only re-render gave the
twisted/teleport feel the user is rejecting.

V23 fixes that at the root:

  * Cafi now carries a full SE(3) pose (px, py, pz, qx, qy, qz, qw).
  * On /objects/attach, the manager looks up T_world_gripper at that
    instant and captures the FROZEN relative transform
        T_gripper_cafi = inv(T_world_gripper) * T_world_cafi
    Every subsequent tick the CAFI's world pose is recomputed as
        T_world_cafi = T_world_gripper * T_gripper_cafi
    so the CAFI moves AS A RIGID BODY with the gripper -- correct
    orientation, no teleport, no yaw drift.
  * On /objects/detach the CAFI is snapped to the destination TF
    (fixture_A_cafi_seat / fixture_B_cafi_seat / vision_fixture_seat)
    using BOTH position AND orientation from that frame.  Bins use a
    gravity drop.
  * On spawn, the CAFI gets a deterministic conveyor orientation that
    matches how the gripper will side-grasp it later (long axis along
    belt = world +X).
  * /objects/markers now publishes the full quaternion so the mesh
    rotates correctly in RViz.
"""

import json
import math
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray


# ============================================================
# GEOMETRY constants -- V39 layout (mirror schneider_cell.urdf.xacro
# and resolve_poses.py).  These MUST match the URDF; V38 inherited
# stale V20 constants here, which caused the CAFI to spawn off the
# belt (the V38 runtime log showed CAFI @ (0.810, 0.861) -- the V20
# spawn point, NOT the V38 belt centre).  V39 fixes them in lock-step
# with the URDF DXF-driven layout.
# ============================================================
BELT_TOP_Z      = 1.070      # mesa top (1.000) + belt rise (0.070)
BELT_Y          = 1.365      # belt centre Y per DXF (LWPOLYLINE #47)
# V39 belt direction: CAFI enters from "Suministro CAFI" at the EAST
# (xyz="1.620 1.365 1.0075" in URDF) and travels WEST to the pick.
SPAWN_X         = 1.620      # suministro east end
PICK_X          = 1.235      # west pick (belt centre - 0.135 notch)

# V39 disc layout (DXF #41 + r=200 circle centred at (0.692, 1.259)).
# LOAD mount is on the SOUTH (cobot side) at y=1.109; RIVET on NORTH at
# y=1.409.  The rotary_index_table_v37 macro defines MOUNT_RADIUS=0.150.
DISC_CENTER_X   = 0.692
DISC_CENTER_Y   = 1.259
DISC_TOP_Z      = 1.081      # mesa top + 0.081 (disco_1 STL top face)
MOUNT_RADIUS    = 0.150
FIXTURE_TOP_Z   = DISC_TOP_Z + 0.030     # 1.111  cradle top
CAFI_REST_Z     = FIXTURE_TOP_Z + 0.012  # 1.123  CAFI centre on cradle

# V44 vision fixture moved 74 mm WEST (DXF + cobot clearance).
VISION_X        = 0.750
VISION_Y        = 0.804
VISION_TOP_Z    = 1.015      # mesa top + 0.015 (cradle top)

# V49 bins: accept at (1.650, 0.720) (V44/V46 position).  Reject pulled
# INSIDE the mesa to (1.330, 0.700) — V48 had it at 0.580 which spilled
# off the mesa south edge (mesa Y_min = 0.589, bin half-Y = 0.091).
# This file, the URDF, the resolve_poses targets and the collision
# checker are now all aligned to 0.700.
BIN_ACCEPT_XY   = (1.650, 0.720)
BIN_REJECT_XY   = (1.330, 0.700)
BIN_FLOOR_Z     = 1.005      # mesa top + 0.005 (bin floor on mesa)
BIN_HALF_X      = 0.090
BIN_HALF_Y      = 0.090

# V25 CAFI dimensions: the user's CAFI mesh is Y-up CAD with mesh bbox
# X[151,274] Y[10,35] Z[12,100] mm  =>  X = 123 mm long (world X)
#                                       Y =  25 mm tall (world Z after Y-up rot)
#                                       Z =  88 mm deep (world Y after Y-up rot)
# V20-V24 incorrectly used LY=0.025, LZ=0.087 (height and depth swapped).
# V25 fixes to the real world dimensions.
CAFI_LX = 0.123    # along world X (long axis on belt)
CAFI_LY = 0.088    # along world Y (deep on belt)
CAFI_LZ = 0.025    # along world Z (VERTICAL height -- 25 mm)

# Mesh bbox center in MESH frame (mm -> m).  Used to compensate the STL's
# baked-in assembly origin offset when publishing the visualization
# marker (the STL origin is NOT at the bbox corner).
# V29 FIX: the V25 hardcoded value (0.2125, 0.0225, 0.056) was wrong by
# 151 mm in X.  Recomputed directly from the cafi.STL bbox:
#   X[0, 122.94] -> center 61.47 mm = 0.0615 m
#   Y[0,  25.15] -> center 12.57 mm = 0.0126 m
#   Z[0,  87.31] -> center 43.65 mm = 0.0436 m
# This bug was causing the CAFI marker to render 151 mm WEST of where
# the rest of the system thought it was - the root cause of the "CAFI
# queda desplazado a la izquierda" visual the user reported.
CAFI_MESH_BBOX_CENTER = (0.0615, 0.0126, 0.0436)

# Y-up to Z-up base rotation for the CAFI mesh: Rx(-pi/2) maps mesh +Y
# to world +Z, so the CAD vertical becomes RViz vertical.
import math as _m
_q_yup_to_zup = (-_m.sin(_m.pi / 4), 0.0, 0.0, _m.cos(_m.pi / 4))

CAFI_MESH = "package://schneider_cell_description/meshes/cafi/cafi.STL"

GRAVITY = 9.81
SETTLE_VZ_THRESHOLD = 0.05
MAX_FALL_VZ = 5.0
TICK_HZ = 30.0


# ============================================================
# Quaternion / transform helpers (pure-Python, NO tf transformations dep)
# ============================================================
def q_identity():
    return (0.0, 0.0, 0.0, 1.0)


def q_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll/2),  math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2),   math.sin(yaw/2)
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    return (qx, qy, qz, qw)


def q_mul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def q_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return q_identity()
    return (x/n, y/n, z/n, w/n)


def q_rotate_vec(q, v):
    """Rotate vector v=(x,y,z) by quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    r = q_mul(q_mul(q, qv), q_conj(q))
    return (r[0], r[1], r[2])


def pose_compose(p_a, q_a, p_b, q_b):
    """Return (p, q) = (T_a * T_b) applied to translation p and rotation q."""
    rotated = q_rotate_vec(q_a, p_b)
    p = (p_a[0] + rotated[0], p_a[1] + rotated[1], p_a[2] + rotated[2])
    q = q_normalize(q_mul(q_a, q_b))
    return p, q


def pose_inverse(p, q):
    qc = q_conj(q)
    pneg = (-p[0], -p[1], -p[2])
    rotated = q_rotate_vec(qc, pneg)
    return rotated, qc


def tf_msg_to_pose(t):
    p = (t.transform.translation.x,
         t.transform.translation.y,
         t.transform.translation.z)
    q = (t.transform.rotation.x,
         t.transform.rotation.y,
         t.transform.rotation.z,
         t.transform.rotation.w)
    return p, q_normalize(q)


# ============================================================
# CAFI object (V23 full SE(3) pose)
# ============================================================
class Cafi(object):
    NEXT_ID = 1

    def __init__(self, p, q):
        self.id = Cafi.NEXT_ID
        Cafi.NEXT_ID += 1
        self.x, self.y, self.z = p
        self.qx, self.qy, self.qz, self.qw = q
        self.vz = 0.0
        self.location = "on_conveyor"
        self.attached_to = None         # gripper frame name when in_gripper
        self.t_gripper_cafi_p = None    # frozen relative translation
        self.t_gripper_cafi_q = None    # frozen relative rotation
        # V26: fixture-id occupancy.  When the CAFI is placed on a
        # rotary-table fixture, this records WHICH physical fixture
        # ("A" or "B") it sits on, independent of which station that
        # fixture is currently at.
        self.fixture_id = None
        self.target_fixture = None
        self.target_bin = None
        # V44 frozen rigid-body offset CAFI-vs-seat captured at settle.
        # While the CAFI is in_fixture_* / at_vision the rendered pose
        # is seat_pose * t_seat_cafi (analogous to the gripper attach
        # behaviour), so the CAFI rotates rigidly with the disc fixture
        # while preserving the orientation it had at release.  None
        # means "not yet settled".
        self.t_seat_cafi_p = None
        self.t_seat_cafi_q = None
        # V28: target seat frame for the settling state machine.
        # When location starts with 'settling_', the CAFI falls under
        # gravity until its bottom reaches the seat top (or the seat
        # XY is within tolerance) and then snaps to the seat pose.
        self.target_seat = None
        self.riveted = False
        self.verdict = None
        self.last_at_sensor = False
        self.spawn_t = rospy.Time.now().to_sec()

    @property
    def pos(self):
        return (self.x, self.y, self.z)

    @property
    def quat(self):
        return (self.qx, self.qy, self.qz, self.qw)

    def set_pose(self, p, q):
        self.x, self.y, self.z = p
        self.qx, self.qy, self.qz, self.qw = q_normalize(q)

    def snapshot(self):
        return {
            "id": self.id,
            "x": round(self.x, 4), "y": round(self.y, 4), "z": round(self.z, 4),
            "qx": round(self.qx, 5), "qy": round(self.qy, 5),
            "qz": round(self.qz, 5), "qw": round(self.qw, 5),
            "location": self.location,
            "fixture_id": self.fixture_id,   # V26
            "attached_to": self.attached_to,
            "riveted": self.riveted,
            "verdict": self.verdict,
            "at_sensor": self.last_at_sensor,
        }


# ============================================================
# Object manager
# ============================================================
class ObjectManager(object):

    def __init__(self):
        rospy.init_node("schneider_object_manager", anonymous=False)

        self.cafis = []
        self.lock = threading.RLock()

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self._last_published_ids = set()

        self.pub_markers = rospy.Publisher(
            "/objects/markers",     MarkerArray, queue_size=10)
        self.pub_states  = rospy.Publisher(
            "/objects/cafi_states", String,      queue_size=2, latch=True)

        rospy.Subscriber("/objects/spawn_request", Empty, self._cb_spawn)
        rospy.Subscriber("/objects/attach", String, self._cb_attach)
        rospy.Subscriber("/objects/detach", Empty, self._cb_detach)
        rospy.Subscriber("/objects/mark_riveted", String,
                         self._cb_mark_riveted)
        rospy.Subscriber("/objects/mark_verdict", String,
                         self._cb_mark_verdict)
        rospy.Subscriber("/conveyor/belt_step", String,
                         self._cb_belt_step)
        rospy.Subscriber("/disc/state", String, self._cb_disc_state)
        rospy.Subscriber("/cell/cycle_stage", String, self._cb_stage)
        rospy.Subscriber("/conveyor/part_present_pick", Bool,
                         self._cb_part_present)
        # V26: fixture-ID model.  We need to know which fixture is at the
        # outer (LOAD) station to set CAFI.fixture_id on detach.
        rospy.Subscriber("/disc/station_assignment", String,
                         self._cb_station_assignment)

        self.disc_state = "IDLE"
        self.stage = "IDLE"
        self.outer_id = "A"      # initial default; refreshed by station msg
        self.inner_id = "B"

        self._publish_states()
        rospy.loginfo("[OBJ] V26 object_manager init - real CAFI dims "
                      "(LX=%.3f LY=%.3f LZ=%.3f), Y-up to Z-up mesh "
                      "rotation, mesh-origin compensation, full SE(3) "
                      "pose, frozen T_gripper_cafi while attached, "
                      "fixture occupancy by fixture-id (A/B).",
                      CAFI_LX, CAFI_LY, CAFI_LZ)

    # =========================================================
    # CALLBACKS
    # =========================================================
    def _cb_spawn(self, _m):
        with self.lock:
            # idempotent guard: dont create two CAFIs in the spawn zone
            for c in self.cafis:
                if c.location == "on_conveyor" and \
                        abs(c.x - SPAWN_X) < 0.150:
                    rospy.logwarn("[OBJ] spawn IGNORED: spawn zone busy")
                    return
            # V46: spawn orientation = rpy(0, pi, pi).  V45 used yaw=pi
            # only; the user found that the CAFI's vertical orientation
            # was wrong for the fixture (rivet hole was facing the wrong
            # way).  V46 adds a 180 deg flip around Y on top of the V45
            # 180 deg around Z.
            #
            # URDF rpy convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
            # For rpy=(0, pi, pi) the net rotation is Rz(pi) * Ry(pi)
            # which equals Rx(pi) (the two 180 deg rotations around
            # perpendicular axes compose to a 180 deg rotation around
            # the third axis).  Quaternion: q_from_rpy(0,pi,pi) =
            # (-1, 0, 0, 0)  ==  (1, 0, 0, 0)  (Rx pi).
            #
            # Effect on the marker render (object_manager already applies
            # an Rx(-pi/2) Y-up->Z-up correction on top of c.quat):
            #   q_render = Rx(pi) * Rx(-pi/2) = Rx(+pi/2)
            #   mesh +Y in world  = Rx(+pi/2) * (0,1,0) = (0, 0, -1)
            # So mesh +Y (was world +Z in V45) now points to world -Z =
            # the CAFI is upside-down.  The 180 deg yaw is preserved
            # because Rx(pi) flips Y and Z but leaves X unchanged.
            spawn_p = (SPAWN_X, BELT_Y, BELT_TOP_Z + CAFI_LZ / 2.0)
            spawn_q = q_from_rpy(0.0, math.pi, math.pi)
            c = Cafi(spawn_p, spawn_q)
            self.cafis.append(c)
            rospy.loginfo("[OBJ] CAFI %d SPAWN @ world (%.3f, %.3f, %.3f) "
                          "rpy=(0,pi,pi) (q=%.3f,%.3f,%.3f,%.3f); "
                          "LZ=%.3f -> top z=%.3f",
                          c.id, spawn_p[0], spawn_p[1], spawn_p[2],
                          spawn_q[0], spawn_q[1], spawn_q[2], spawn_q[3],
                          CAFI_LZ, spawn_p[2] + CAFI_LZ / 2.0)
    def _cb_attach(self, msg):
        with self.lock:
            try:
                data = json.loads(msg.data)
            except Exception as e:
                rospy.logwarn("[OBJ] attach msg parse fail: %s", e)
                return
            cid = int(data.get("cafi_id", -1))
            # V43: default attach link follows the new TCP alias defined
            # inside lexium_cobot_with_final_gripper.xacro.
            link = data.get("link", "lexium_cobot_tcp_grasp_center")
            for c in self.cafis:
                if c.id != cid:
                    continue
                # Capture T_gripper_cafi NOW.
                try:
                    t = self.tf_buf.lookup_transform(
                        "world", link,
                        rospy.Time(0), rospy.Duration(0.2))
                    p_wg, q_wg = tf_msg_to_pose(t)
                except Exception as e:
                    rospy.logwarn("[OBJ] attach: TF lookup for %s failed: %s "
                                  "-> CAFI %d will snap to grasp frame "
                                  "(orientation may not be preserved)",
                                  link, e, cid)
                    p_wg, q_wg = (c.x, c.y, c.z), q_identity()
                # T_gripper_cafi = inv(T_world_gripper) * T_world_cafi
                p_inv, q_inv = pose_inverse(p_wg, q_wg)
                p_rel, q_rel = pose_compose(p_inv, q_inv, c.pos, c.quat)
                # V40: rigid-body attach with NO extra shift.  V30 added
                # a +0.025 m offset along grasp_center +Y to "press the
                # CAFI against the jaw blade", but this caused the CAFI
                # to visibly TELEPORT 25 mm in world frame the moment
                # /objects/attach was published.  V40 keeps the CAFI at
                # its exact pre-attach world pose; the freeze-frame
                # T_gripper_cafi (= identity when grasp_center FK lands
                # on the CAFI centre at PICK) keeps the CAFI rigidly
                # at grasp_center for the rest of the carry.  If a small
                # visual gap between the CAFI and the jaw blade appears,
                # it's because grasp_center is not exactly at the jaw
                # midpoint -- the fix for that is to move the
                # gripper_base_grasp_center frame in new_gripper.xacro,
                # NOT to shift the CAFI on attach.
                c.t_gripper_cafi_p = p_rel
                c.t_gripper_cafi_q = q_rel
                # V44 clear the seat offset — once the CAFI is back in
                # the gripper any stale seat-relative transform must NOT
                # be reused when it is later released onto a different
                # seat.
                c.t_seat_cafi_p = None
                c.t_seat_cafi_q = None
                c.attached_to = link
                prev = c.location
                c.location = "in_gripper"
                rospy.loginfo("[OBJ] CAFI %d attached to %s "
                              "(prev=%s, frozen T_gripper_cafi "
                              "p=(%.3f,%.3f,%.3f) q=(%.3f,%.3f,%.3f,%.3f))",
                              cid, link, prev,
                              p_rel[0], p_rel[1], p_rel[2],
                              q_rel[0], q_rel[1], q_rel[2], q_rel[3])
                self._publish_states()
                return
            rospy.logwarn("[OBJ] attach IGNORED: no CAFI id=%d in snapshot",
                          cid)

    def _cb_detach(self, _m):
        """V28: replace V27's instant snap with a settling state.

        On detach the CAFI keeps the world pose it had in the gripper
        (no teleport), is marked as 'settling_*' with a target seat
        frame, and falls under gravity in tick() until its bottom
        reaches the seat top.  At that point a small XY assist snaps
        the CAFI center to the seat XY so it sits cleanly in the
        cradle, then the state transitions to 'in_fixture_*' /
        'at_vision' so downstream logic still sees the CAFI in its
        functional location.

        For bins the existing 'falling' free-fall is preserved.
        """
        with self.lock:
            for c in self.cafis:
                if c.location != "in_gripper":
                    continue
                dest = self._classify_drop_destination(c.x, c.y)
                # V28: do NOT teleport; keep current world pose and
                # start the settling state machine.  The CAFI gets a
                # small upward bump (+30 mm) so it's clearly ABOVE the
                # seat at release time, giving a visible fall.
                if dest == "fixture_outer":
                    c.fixture_id = self.outer_id
                    c.location = "settling_fixture"
                    c.target_seat = "fixture_" + c.fixture_id + "_cafi_seat"
                    c.vz = 0.0
                    c.z = c.z + 0.030
                elif dest == "fixture_inner":
                    c.fixture_id = self.inner_id
                    c.location = "settling_fixture"
                    c.target_seat = "fixture_" + c.fixture_id + "_cafi_seat"
                    c.vz = 0.0
                    c.z = c.z + 0.030
                elif dest == "vision":
                    c.location = "settling_vision"
                    # The vision fixture link is named "fixture_2" in
                    # schneider_cell.urdf.xacro.  Its cafi_seat frame
                    # was added in V28 fixture_vision.xacro.
                    c.target_seat = "fixture_2_cafi_seat"
                    c.vz = 0.0
                    c.z = c.z + 0.030
                elif dest == "bin_accept":
                    c.location = "falling"
                    c.target_bin = "accept"
                    c.vz = 0.0
                elif dest == "bin_reject":
                    c.location = "falling"
                    c.target_bin = "reject"
                    c.vz = 0.0
                else:
                    c.location = "falling"
                    c.vz = 0.0
                c.attached_to = None
                c.t_gripper_cafi_p = None
                c.t_gripper_cafi_q = None
                rospy.loginfo("[OBJ] CAFI %d detach -> %s "
                              "(fixture_id=%s station_outer=%s "
                              "start_pose=(%.3f,%.3f,%.3f))",
                              c.id, c.location, c.fixture_id,
                              self.outer_id, c.x, c.y, c.z)
            self._publish_states()

    def _cb_station_assignment(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        with self.lock:
            new_outer = data.get("outer", self.outer_id)
            new_inner = data.get("inner", self.inner_id)
            if new_outer != self.outer_id or new_inner != self.inner_id:
                rospy.loginfo("[OBJ] station_assignment update outer=%s inner=%s",
                              new_outer, new_inner)
            self.outer_id = new_outer
            self.inner_id = new_inner

    def _cb_mark_riveted(self, msg):
        with self.lock:
            try:
                cid = int(msg.data)
            except Exception:
                return
            for c in self.cafis:
                if c.id == cid:
                    c.riveted = True
                    rospy.loginfo("[OBJ] CAFI %d marked RIVETED", cid)
                    self._publish_states()
                    return

    def _cb_mark_verdict(self, msg):
        with self.lock:
            try:
                data = json.loads(msg.data)
                cid  = int(data["cafi_id"])
                verd = data["verdict"]
            except Exception:
                return
            for c in self.cafis:
                if c.id == cid:
                    c.verdict = verd
                    rospy.loginfo("[OBJ] CAFI %d verdict=%s", cid, verd)
                    self._publish_states()
                    return

    def _cb_belt_step(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        speed = float(data.get("speed", 0.0))
        dt    = float(data.get("dt", 1.0 / TICK_HZ))
        # V39: belt moves CAFIs WEST (from suministro at high X to pick
        # at low X).  x_min is the pick position; do not advance past it.
        x_min = float(data.get("x_min", PICK_X))
        with self.lock:
            for c in self.cafis:
                if c.location == "on_conveyor":
                    c.x = max(x_min, c.x - speed * dt)

    def _cb_disc_state(self, m): self.disc_state = m.data
    def _cb_stage(self, m):      self.stage = m.data
    def _cb_part_present(self, _m): pass

    # =========================================================
    # SNAP HELPERS - full pose from TF (V23)
    # =========================================================
    def _snap_to_frame(self, c, frame_name):
        """V44 rigid-body follow: pose = seat_pose * t_seat_cafi.

        The CAFI is treated as a rigid body bolted to the seat.  When
        the disc indexes, the seat rotates in world and the CAFI
        rotates with it — orientation tracks the seat (correct physics)
        but the small relative offset captured at settle is preserved
        so there is NO "magic reorientation" at the settling instant.
        """
        try:
            t = self.tf_buf.lookup_transform(
                "world", frame_name,
                rospy.Time(0), rospy.Duration(0.2))
            sp, sq = tf_msg_to_pose(t)
            if c.t_seat_cafi_p is not None and c.t_seat_cafi_q is not None:
                p, q = pose_compose(sp, sq,
                                    c.t_seat_cafi_p, c.t_seat_cafi_q)
                c.set_pose(p, q)
            else:
                # Should not happen (settle always sets the offset), but
                # if it does, fall back to pure position snap.
                c.set_pose(sp, (c.qx, c.qy, c.qz, c.qw))
        except Exception as e:
            rospy.logwarn("[OBJ] snap_to_frame(%s) TF fail: %s -> using "
                          "fallback XY only", frame_name, e)
            if "fixture_A" in frame_name:
                c.set_pose((DISC_CENTER_X, DISC_CENTER_Y - MOUNT_RADIUS,
                            CAFI_REST_Z),
                           (c.qx, c.qy, c.qz, c.qw))
            elif "fixture_B" in frame_name:
                c.set_pose((DISC_CENTER_X, DISC_CENTER_Y + MOUNT_RADIUS,
                            CAFI_REST_Z),
                           (c.qx, c.qy, c.qz, c.qw))

    def _snap_to_vision(self, c):
        # V44 rigid-body follow: pose = seat_pose * t_seat_cafi.
        try:
            t = self.tf_buf.lookup_transform(
                "world", "fixture_2_cafi_seat",
                rospy.Time(0), rospy.Duration(0.2))
            sp, sq = tf_msg_to_pose(t)
            if c.t_seat_cafi_p is not None and c.t_seat_cafi_q is not None:
                p, q = pose_compose(sp, sq,
                                    c.t_seat_cafi_p, c.t_seat_cafi_q)
                c.set_pose(p, q)
            else:
                c.set_pose(sp, (c.qx, c.qy, c.qz, c.qw))
        except Exception:
            c.set_pose((VISION_X, VISION_Y, VISION_TOP_Z + CAFI_LZ / 2.0),
                       (c.qx, c.qy, c.qz, c.qw))

    def _classify_drop_destination(self, x, y):
        if abs(x - VISION_X) < 0.15 and abs(y - VISION_Y) < 0.15:
            return "vision"
        if (abs(x - BIN_ACCEPT_XY[0]) < BIN_HALF_X and
                abs(y - BIN_ACCEPT_XY[1]) < BIN_HALF_Y):
            return "bin_accept"
        if (abs(x - BIN_REJECT_XY[0]) < BIN_HALF_X and
                abs(y - BIN_REJECT_XY[1]) < BIN_HALF_Y):
            return "bin_reject"
        outer_y = DISC_CENTER_Y - MOUNT_RADIUS
        if abs(x - DISC_CENTER_X) < 0.10 and abs(y - outer_y) < 0.12:
            return "fixture_outer"
        inner_y = DISC_CENTER_Y + MOUNT_RADIUS
        if abs(x - DISC_CENTER_X) < 0.10 and abs(y - inner_y) < 0.12:
            return "fixture_inner"
        return "free_fall"

    # =========================================================
    # TICK
    # =========================================================
    def tick(self, dt):
        with self.lock:
            for c in self.cafis:
                if c.location == "in_gripper" and c.attached_to:
                    # V23: rigid-body follow using frozen T_gripper_cafi
                    try:
                        t = self.tf_buf.lookup_transform(
                            "world", c.attached_to,
                            rospy.Time(0), rospy.Duration(0.05))
                        p_wg, q_wg = tf_msg_to_pose(t)
                        if c.t_gripper_cafi_p is not None:
                            p, q = pose_compose(
                                p_wg, q_wg,
                                c.t_gripper_cafi_p,
                                c.t_gripper_cafi_q)
                            c.set_pose(p, q)
                        else:
                            c.set_pose(p_wg, q_wg)
                    except Exception:
                        pass
                elif c.location in ("in_fixture_A", "in_fixture_B"):
                    # V26: snap by fixture_id (independent of outer/inner)
                    seat = "fixture_" + c.fixture_id + "_cafi_seat"
                    self._snap_to_frame(c, seat)
                elif c.location == "at_vision":
                    self._snap_to_vision(c)
                elif c.location in ("settling_fixture", "settling_vision"):
                    # V28: gravity-based settling - visible drop until the
                    # CAFI bottom reaches the seat top, then small XY
                    # assist to center on the cradle.  This replaces the
                    # V27 instant snap that the user rejected.
                    try:
                        t = self.tf_buf.lookup_transform(
                            "world", c.target_seat,
                            rospy.Time(0), rospy.Duration(0.05))
                        sp, sq = tf_msg_to_pose(t)
                    except Exception:
                        # If TF unavailable, fall back to settled state
                        c.location = ("in_fixture_" + (c.fixture_id or "A")
                                      if c.location == "settling_fixture"
                                      else "at_vision")
                        continue

                    # Apply gravity
                    c.vz = max(-MAX_FALL_VZ, c.vz - GRAVITY * dt)
                    c.z += c.vz * dt

                    # Seat target height (CAFI center should land on
                    # seat XYZ, which already accounts for cradle top
                    # plus half CAFI height).
                    target_z = sp[2]
                    # V44 capture rigid-body offset CAFI-vs-seat ONCE on
                    # the way down (first time TF lookup succeeded).
                    # We use the LATEST seat_q here so the offset
                    # records the orientation difference at settle.
                    seat_q = sq
                    if c.z <= target_z:
                        # V44: release height shrunk to +20 mm and the
                        # cobot IK places the gripper coaxial with the
                        # seat (sub-mm error), so the XY mismatch at
                        # release is ALREADY tiny.  V44 limits the
                        # snap assist to 5 mm (was 100 mm) so the
                        # settle does NOT visually look like a snap.
                        # If the CAFI is further than 5 mm from the
                        # seat in XY at settle time it stays where it
                        # is — that's a bug to fix upstream, not to
                        # mask with a magic snap.
                        dx = sp[0] - c.x
                        dy = sp[1] - c.y
                        c.x += max(-0.005, min(0.005, dx))
                        c.y += max(-0.005, min(0.005, dy))
                        c.z = target_z
                        c.vz = 0.0
                        # V44 capture rigid-body offset CAFI-vs-seat
                        # at settle.  Position offset is computed AFTER
                        # the 5 mm assist so it stays tiny; orientation
                        # offset is the orientation difference between
                        # the CAFI (as released) and the seat at this
                        # instant.  From now on the CAFI follows the
                        # seat as a rigid body — disc rotation moves
                        # both position AND orientation in lockstep,
                        # but the orientation set at release is
                        # PRESERVED (no magic snap).
                        p_inv, q_inv = pose_inverse(sp, seat_q)
                        p_rel, q_rel = pose_compose(
                            p_inv, q_inv, c.pos, c.quat)
                        c.t_seat_cafi_p = p_rel
                        c.t_seat_cafi_q = q_rel
                        if c.location == "settling_fixture":
                            c.location = "in_fixture_" + (c.fixture_id or "A")
                            rospy.loginfo(
                                "[OBJ] CAFI %d settled into %s at "
                                "world (%.3f, %.3f, %.3f) "
                                "(orientation preserved from release)",
                                c.id, c.location, c.x, c.y, c.z)
                        else:
                            c.location = "at_vision"
                            rospy.loginfo(
                                "[OBJ] CAFI %d settled at vision station "
                                "world (%.3f, %.3f, %.3f) "
                                "(orientation preserved from release)",
                                c.id, c.x, c.y, c.z)
                elif c.location == "falling":
                    c.vz = max(-MAX_FALL_VZ, c.vz - GRAVITY * dt)
                    c.z += c.vz * dt
                    floor = self._surface_below(c)
                    if c.z <= floor:
                        c.z = floor; c.vz = 0.0
                        if c.target_bin == "accept":
                            c.location = "in_bin"
                            c.x = BIN_ACCEPT_XY[0]
                            c.y = BIN_ACCEPT_XY[1]
                            c.z = BIN_FLOOR_Z + CAFI_LZ / 2.0 + 0.005
                        elif c.target_bin == "reject":
                            c.location = "in_bin"
                            c.x = BIN_REJECT_XY[0]
                            c.y = BIN_REJECT_XY[1]
                            c.z = BIN_FLOOR_Z + CAFI_LZ / 2.0 + 0.005
                        else:
                            c.location = "settled"
                if c.location == "on_conveyor":
                    c.last_at_sensor = self._is_at_pick_sensor(c)
                else:
                    c.last_at_sensor = False

            now = rospy.Time.now().to_sec()
            to_remove = [c for c in self.cafis
                         if c.location == "in_bin" and (now - c.spawn_t) > 30.0]
            for c in to_remove:
                self.cafis.remove(c)

        self._publish_states()
        self._publish_markers()

    def _surface_below(self, c):
        if (abs(c.x - BIN_ACCEPT_XY[0]) < BIN_HALF_X and
                abs(c.y - BIN_ACCEPT_XY[1]) < BIN_HALF_Y):
            return BIN_FLOOR_Z + CAFI_LZ / 2.0 + 0.005
        if (abs(c.x - BIN_REJECT_XY[0]) < BIN_HALF_X and
                abs(c.y - BIN_REJECT_XY[1]) < BIN_HALF_Y):
            return BIN_FLOOR_Z + CAFI_LZ / 2.0 + 0.005
        # V39: floor is mesa top (1.000 m) -- not 1.200 (V20 platform).
        return 1.000 + CAFI_LZ / 2.0 + 0.005

    def _is_at_pick_sensor(self, c):
        # V39 sensor at the WEST end (face_x=1.235, face_y=1.290 per URDF
        # photoelectric_sensor_sick_grte18s_p2312 instance).  The CAFI
        # passes through the beam when its centre lies inside the small
        # window around the sensor face on the belt.
        SENSOR_X = PICK_X; HW_X = 0.040
        Y_LO, Y_HI = 1.290, 1.440
        SENSOR_Z = BELT_TOP_Z + CAFI_LZ / 2.0; HW_Z = 0.060
        return (abs(c.x - SENSOR_X) <= HW_X and
                Y_LO <= c.y <= Y_HI and
                abs(c.z - SENSOR_Z) <= HW_Z)

    # =========================================================
    # PUBLISH
    # =========================================================
    def _publish_states(self):
        snap = [c.snapshot() for c in self.cafis]
        self.pub_states.publish(String(data=json.dumps(snap)))

    def _publish_markers(self):
        ma = MarkerArray()
        now = rospy.Time.now()

        current_ids = set(c.id for c in self.cafis)
        stale_ids   = self._last_published_ids - current_ids
        for sid in stale_ids:
            d = Marker()
            d.header.frame_id = "world"
            d.header.stamp = now
            d.ns = "cafi"
            d.id = sid
            d.action = Marker.DELETE
            ma.markers.append(d)
        self._last_published_ids = current_ids

        for c in self.cafis:
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = now
            m.ns = "cafi"
            m.id = c.id
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.mesh_resource = CAFI_MESH
            m.mesh_use_embedded_materials = False
            m.scale.x = 0.001; m.scale.y = 0.001; m.scale.z = 0.001
            # V25: TWO corrections vs V20-V24.
            # (1) Y-up to Z-up: marker orientation = c.quat * Rx(-pi/2)
            #     so the mesh +Y axis (CAD vertical) maps to world +Z.
            # (2) Mesh-origin compensation: the STL has its bbox CENTER
            #     at mesh-frame (212.5, 22.5, 56) mm because it was
            #     exported from the assembly origin, NOT at the bbox
            #     corner.  We translate the marker pose so the FINAL
            #     bbox center coincides with (c.x, c.y, c.z), matching
            #     what the rest of the system thinks the CAFI is.
            #     Without this fix the rendered CAFI was ~152 mm OFF
            #     from where the IK targeted (root of PICK_CONV watchdog).
            q_out = q_normalize(q_mul(c.quat, _q_yup_to_zup))
            bbox_center_world = q_rotate_vec(q_out, CAFI_MESH_BBOX_CENTER)
            m.pose.position.x = c.x - bbox_center_world[0]
            m.pose.position.y = c.y - bbox_center_world[1]
            m.pose.position.z = c.z - bbox_center_world[2]
            m.pose.orientation.x = q_out[0]
            m.pose.orientation.y = q_out[1]
            m.pose.orientation.z = q_out[2]
            m.pose.orientation.w = q_out[3]
            # V42: brighter, FULLY opaque CAFI palette.  V20-V41 used
            # alpha=0.85 which made the CAFI render with a "ghost"
            # translucent look (the user kept reporting it as "naranja
            # transparente").  V42 sets alpha=1.0 and increases the
            # saturation of the default orange so the part reads as
            # a real, opaque solid.
            if c.location == "in_bin":
                m.color.r = 0.40; m.color.g = 0.40; m.color.b = 0.40
            elif c.verdict == "FAIL":
                m.color.r = 0.95; m.color.g = 0.15; m.color.b = 0.15
            elif c.verdict == "PASS":
                m.color.r = 0.15; m.color.g = 0.85; m.color.b = 0.15
            elif c.riveted:
                m.color.r = 0.95; m.color.g = 0.55; m.color.b = 0.10
            else:
                m.color.r = 1.00; m.color.g = 0.45; m.color.b = 0.00
            m.color.a = 1.0
            ma.markers.append(m)
        self.pub_markers.publish(ma)

    def run(self):
        rate = rospy.Rate(TICK_HZ)
        dt = 1.0 / TICK_HZ
        while not rospy.is_shutdown():
            self.tick(dt)
            rate.sleep()


if __name__ == "__main__":
    try:
        ObjectManager().run()
    except rospy.ROSInterruptException:
        pass
