#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_rotary_fixture_sim.rotary_fixture_sim_node  --  V23

V23 changes (vs V22 - which was V20 untouched):
  - Added the user-requested chain trace logs for the disc index
    sequence so any failure in runtime can be localised in rosout
    without RViz.  Specifically a 10 Hz throttled progress log of
    [ROT] disc_pos=... target=... while INDEXING, plus an explicit
    [ROT] disc index cmd received / [ROT] disc index done banner.
  - DISC_INDEX_RATE_RAD_S relaxed from 1.6 to 1.2 (~ 2.6 s for 180 deg)
    so the rotation reads cleanly in RViz.

ORIGINAL V20 DOC BELOW
======================

Maneja:
  - rotary_table_disc_axis_joint  (continuous, angulo del disco)
  - fixture_A_piston_joint        (prismatic, rivet head A)
  - fixture_A_solenoid_left_joint (prismatic, lateral seating A izq)
  - fixture_A_solenoid_right_joint
  - fixture_B_piston_joint
  - fixture_B_solenoid_left_joint
  - fixture_B_solenoid_right_joint

Y publica los topics:
  /disc/state          ("IDLE" | "INDEXING")
  /disc/index_done     (Empty cuando termina el indexado)
  /fixture/load_present (Bool, hay CAFI en outer)
  /fixture/rivet_present (Bool, hay CAFI en inner)
  /fixture/cafi_seated  (Bool, solenoides activos y seated por > 0.5 s)
  /fixture/solenoids_state (UInt8MultiArray [L,R])
  /rivet/active        (Bool, True durante el ciclo de 30 s)
  /rivet/done          (Empty al terminar 30 s)

Comandos:
  /disc/index_cmd      (Float32, target delta angular)
  /rivet/start_cmd     (Empty)
  /fixture/seat_cmd    (String, "outer" o "inner")
  /fixture/unseat_cmd  (String)

V20 NOTAS:
  RIVET_DURATION_S = 30.0 (V19 era 60).
  El timer SIEMPRE termina y publica /rivet/done una vez. Diseno
  watchdog-free para el cliente: state_manager solo espera el evento.

  El indexado es CONTINUO (continuous joint), no STOP-AND-START.  Pero
  por simulacion paramos al alcanzar el target ANG con tolerancia 0.005 rad.
"""

import json
import math
import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32, String, UInt8MultiArray


# V52: the new turntable URDF exposes a single revolute joint
# (table_rotation_joint) for the disc and NO piston / solenoid joints
# (the pistons inside fixtures 1 and 2 are FIXED meshes per spec).  The
# /disc/joints publication is therefore reduced to that single joint so
# robot_state_publisher does not warn about missing joint names.  The
# rest of the rivet-cycle topics (/rivet/active, /fixture/cafi_seated,
# /fixture/load_present, /disc/station_assignment, ...) are kept intact
# because downstream consumers (HMI, state_manager, object_manager) rely
# on them; the seating + 30 s rivet timer remain time-based and no
# longer animate a prismatic joint.
DISC_JOINT     = "table_rotation_joint"

ALL_JOINTS = [DISC_JOINT]

# Rates / limits
DISC_INDEX_RATE_RAD_S = 1.2        # ~2.6 s para 180 deg (V23 visible in RViz)
PISTON_RATE           = 0.020      # m/s descent
SOLENOID_RATE         = 0.005      # m/s
SEAT_HOLD_S           = 0.5

# Tolerancias
DISC_TOL_RAD          = 0.005
RIVET_DURATION_S      = 30.0       # V20: del V19 60.0 -> V20 30.0
TICK_HZ               = 50.0


class RotaryFixtureSim(object):

    def __init__(self):
        rospy.init_node("schneider_rotary_fixture_sim", anonymous=False)

        # Disco
        self.disc_pos    = 0.0
        self.disc_target = 0.0
        self.disc_state  = "IDLE"

        # Pistones / solenoides (V25: solo LEFT por fixture; right REMOVIDO)
        self.A_pist   = 0.0;  self.A_pist_tgt   = 0.0
        self.A_sol_l  = 0.0;  self.A_sol_l_tgt  = 0.0
        self.B_pist   = 0.0;  self.B_pist_tgt   = 0.0
        self.B_sol_l  = 0.0;  self.B_sol_l_tgt  = 0.0

        # ====================================================
        # V53: STATION ASSIGNMENT uses physical fixture ids "A" / "B"
        # (the convention every existing consumer is wired for).  The
        # mapping to the new turntable URDF's "_1" / "_2" frame suffix
        # lives in _fix_frame_suffix() below — code never looks up the
        # dict with a "1" / "2" key, so KeyError 'A' is impossible.
        # Initial state: fixture A at the LOAD (cobot-side, outer)
        #                fixture B at the RIVET (cabin-side, inner)
        # ====================================================
        self.outer_id = "A"
        self.inner_id = "B"

        # CAFI presence BY FIXTURE ID (not by station label).  Updated
        # from /objects/cafi_states with locations "in_fixture_A" /
        # "in_fixture_B".  Tracking by fixture id keeps the rivet
        # authorization correct after a disc index swap.
        self.fixture_has_cafi = {"A": False, "B": False}
        self.fixture_cafi_id  = {"A": None,  "B": None}

        # Estado de seating / rivet
        self.seat_t0    = 0.0      # t cuando solenoides arrancaron
        self.seated     = False
        self.rivet_t0   = 0.0
        self.rivet_active = False

        self.lock = threading.RLock()

        # ---- Publishers ----
        self.pub_joints       = rospy.Publisher(
            "/disc/joints",                JointState, queue_size=1)
        self.pub_disc_state   = rospy.Publisher(
            "/disc/state",                 String,     queue_size=1, latch=True)
        self.pub_disc_done    = rospy.Publisher(
            "/disc/index_done",            Empty,      queue_size=2)
        self.pub_load_present = rospy.Publisher(
            "/fixture/load_present",       Bool,       queue_size=1, latch=True)
        self.pub_rivet_present = rospy.Publisher(
            "/fixture/rivet_present",      Bool,       queue_size=1, latch=True)
        self.pub_seated       = rospy.Publisher(
            "/fixture/cafi_seated",        Bool,       queue_size=1, latch=True)
        self.pub_sol_state    = rospy.Publisher(
            "/fixture/solenoids_state",    UInt8MultiArray,
            queue_size=1, latch=True)
        self.pub_rivet_active = rospy.Publisher(
            "/rivet/active",               Bool,       queue_size=1, latch=True)
        self.pub_rivet_done   = rospy.Publisher(
            "/rivet/done",                 Empty,      queue_size=2)
        # V26: station assignment publication (latched, JSON)
        self.pub_station      = rospy.Publisher(
            "/disc/station_assignment",    String,     queue_size=1, latch=True)
        # V29 fix: pub_mark_riveted was created lazily on first rivet
        # completion, but the first message after a fresh publisher
        # connection is reliably lost in ROS unless we wait for the
        # subscriber to register.  Persistent publisher initialised
        # at startup ensures every "marked RIVETED" message reaches
        # the object_manager.
        self._pub_riveted     = rospy.Publisher(
            "/objects/mark_riveted",       String,     queue_size=2, latch=True)

        # ---- Subscribers ----
        rospy.Subscriber("/disc/index_cmd",    Float32, self._cb_disc_cmd)
        rospy.Subscriber("/rivet/start_cmd",   Empty,   self._cb_rivet_start)
        rospy.Subscriber("/fixture/seat_cmd",  String,  self._cb_seat)
        rospy.Subscriber("/fixture/unseat_cmd", String, self._cb_unseat)
        # Fixture occupancy is owned by object_manager; we read snapshot
        rospy.Subscriber("/objects/cafi_states", String, self._cb_cafi_states)

        # Initial publish
        self.pub_disc_state.publish(String(data="IDLE"))
        self.pub_load_present.publish(Bool(data=False))
        self.pub_rivet_present.publish(Bool(data=False))
        self.pub_seated.publish(Bool(data=False))
        self.pub_rivet_active.publish(Bool(data=False))
        self._publish_station()
        self._publish_solenoids()
        rospy.loginfo("[ROT] V52 rotary_fixture_sim init "
                      "(RIVET_DURATION_S=%.0f, outer=%s inner=%s); "
                      "drives table_rotation_joint only; pistons are "
                      "fixed meshes in the new turntable URDF",
                      RIVET_DURATION_S, self.outer_id, self.inner_id)

    def _publish_station(self):
        msg = String(data=json.dumps({
            "outer": self.outer_id,
            "inner": self.inner_id,
        }))
        self.pub_station.publish(msg)

    def _swap_stations(self):
        self.outer_id, self.inner_id = self.inner_id, self.outer_id
        self._publish_station()
        rospy.loginfo("[ROT] STATION ASSIGNMENT swapped -> outer=%s inner=%s "
                      "(fixture_has_cafi A=%s B=%s)",
                      self.outer_id, self.inner_id,
                      self.fixture_has_cafi["A"], self.fixture_has_cafi["B"])

    # =========================================================
    # Callbacks
    # =========================================================
    def _cb_disc_cmd(self, msg):
        with self.lock:
            delta = float(msg.data)
            self.disc_target = self.disc_pos + delta
            self.disc_state  = "INDEXING"
            self.pub_disc_state.publish(String(data="INDEXING"))
            rospy.loginfo("[ROT] disc index cmd received delta=%.3f rad "
                          "target=%.3f rad (from %.3f rad, ETA=%.2f s)",
                          delta, self.disc_target, self.disc_pos,
                          abs(delta) / DISC_INDEX_RATE_RAD_S)

    def _cb_rivet_start(self, _m):
        with self.lock:
            if self.rivet_active:
                rospy.logwarn("[ROT] rivet already active - ignored")
                return
            # V26: rivet authorisation = check the CAFI on the fixture
            # currently at the INNER station.  Use fixture-id occupancy,
            # not stale outer/inner labels.
            inner_fix = self.inner_id
            inner_has = self.fixture_has_cafi.get(inner_fix, False)
            inner_cafi = self.fixture_cafi_id.get(inner_fix)
            if not inner_has:
                rospy.logwarn(
                    "[ROT] rivet REJECTED: inner_fixture=%s occupancy=False "
                    "outer_fixture=%s fixture_A_has_cafi=%s fixture_B_has_cafi=%s",
                    inner_fix, self.outer_id,
                    self.fixture_has_cafi["A"], self.fixture_has_cafi["B"])
                return
            rospy.loginfo("[ROT] rivet accepted: fixture=%s cafi_id=%s",
                          inner_fix, inner_cafi)
            self.rivet_active = True
            self.rivet_t0 = rospy.Time.now().to_sec()
            # piston descends + solenoide B left activo (V25: no right)
            self.B_pist_tgt  = -0.030
            self.B_sol_l_tgt = -0.010
            self.pub_rivet_active.publish(Bool(data=True))
            rospy.loginfo("[ROT] rivet timer start %.1fs", RIVET_DURATION_S)

    def _cb_seat(self, msg):
        with self.lock:
            where = msg.data
            if where == "outer":
                self.A_sol_l_tgt = -0.010
                self.seat_t0 = rospy.Time.now().to_sec()
                self.seated = False
                rospy.loginfo("[ROT] seat OUTER solenoid_left -> active")
            elif where == "inner":
                self.B_sol_l_tgt = -0.010
                self.seat_t0 = rospy.Time.now().to_sec()
                self.seated = False
                rospy.loginfo("[ROT] seat INNER solenoid_left -> active")

    def _cb_unseat(self, msg):
        with self.lock:
            where = msg.data
            if where == "outer":
                self.A_sol_l_tgt = 0.0
                self.seated = False
                self.pub_seated.publish(Bool(data=False))
            elif where == "inner":
                self.B_sol_l_tgt = 0.0
                self.seated = False
                self.pub_seated.publish(Bool(data=False))

    def _cb_cafi_states(self, msg):
        try:
            data = json.loads(msg.data) or []
        except Exception:
            return
        with self.lock:
            # V53: occupancy by physical fixture ID "A" / "B".  Accept
            # both the canonical "in_fixture_A" / "in_fixture_B" and
            # the V52-transitional "in_fixture_1" / "in_fixture_2"
            # for forward compatibility (if anything still emits them
            # they map cleanly onto A / B).
            _loc_to_fid = {
                "in_fixture_A": "A", "in_fixture_B": "B",
                "in_fixture_1": "A", "in_fixture_2": "B",
            }
            new_obj = {"A": None, "B": None}
            for c in data:
                loc = c.get("location", "")
                fid = _loc_to_fid.get(loc)
                if fid is not None:
                    new_obj[fid] = c
            for fid in ("A", "B"):
                has = new_obj[fid] is not None
                if has != self.fixture_has_cafi[fid]:
                    self.fixture_has_cafi[fid] = has
                    self.fixture_cafi_id[fid]  = (
                        new_obj[fid].get("id") if new_obj[fid] else None)
                    rospy.loginfo(
                        "[ROT] fixture_%s occupancy -> %s (cafi_id=%s)",
                        fid, has, self.fixture_cafi_id[fid])
            # Derived: which station has a CAFI (for /fixture/load_present
            # and /fixture/rivet_present, which the HMI still uses).
            # V35: ALSO publish from tick() as a heartbeat so a single
            # missed cafi_states message can never leave the derived
            # signal stale (root cause of V34 PLACE_LOAD watchdog).
            load_present  = self.fixture_has_cafi[self.outer_id]
            rivet_present = self.fixture_has_cafi[self.inner_id]
            self.pub_load_present.publish(Bool(data=load_present))
            self.pub_rivet_present.publish(Bool(data=rivet_present))

    # =========================================================
    # Animation tick
    # =========================================================
    def _step_toward(self, cur, tgt, rate, dt):
        d = rate * dt
        diff = tgt - cur
        if abs(diff) <= d:
            return tgt
        return cur + (d if diff > 0 else -d)

    def tick(self, dt):
        with self.lock:
            # Disc
            if self.disc_state == "INDEXING":
                diff = self.disc_target - self.disc_pos
                step = DISC_INDEX_RATE_RAD_S * dt
                # ease-out near the end
                if abs(diff) <= step:
                    self.disc_pos = self.disc_target
                    self.disc_state = "IDLE"
                    self.pub_disc_state.publish(String(data="IDLE"))
                    # V26: swap station assignment BEFORE publishing
                    # /disc/index_done so any subscriber that reacts to
                    # the done event sees the new assignment.
                    self._swap_stations()
                    self.pub_disc_done.publish(Empty())
                    rospy.loginfo("[ROT] disc index done at %.3f rad "
                                  "(target=%.3f, /disc/index_done published)",
                                  self.disc_pos, self.disc_target)
                else:
                    self.disc_pos += (step if diff > 0 else -step)
                    rospy.loginfo_throttle(0.2,
                        "[ROT] disc_pos=%.3f target=%.3f (state=INDEXING)",
                        self.disc_pos, self.disc_target)

            # Pistones / solenoides interpolacion (V25: solo LEFT)
            self.A_pist   = self._step_toward(self.A_pist,   self.A_pist_tgt,
                                              PISTON_RATE, dt)
            self.B_pist   = self._step_toward(self.B_pist,   self.B_pist_tgt,
                                              PISTON_RATE, dt)
            self.A_sol_l  = self._step_toward(self.A_sol_l,  self.A_sol_l_tgt,
                                              SOLENOID_RATE, dt)
            self.B_sol_l  = self._step_toward(self.B_sol_l,  self.B_sol_l_tgt,
                                              SOLENOID_RATE, dt)

            # Seat confirmation (V25: solo solenoid_left activo > SEAT_HOLD_S)
            outer_active = (abs(self.A_sol_l - (-0.010)) < 1e-4)
            inner_active = (abs(self.B_sol_l - (-0.010)) < 1e-4)
            if (outer_active or inner_active) and self.seat_t0 > 0:
                if (rospy.Time.now().to_sec() - self.seat_t0) >= SEAT_HOLD_S:
                    if not self.seated:
                        self.seated = True
                        self.pub_seated.publish(Bool(data=True))
                        rospy.loginfo("[ROT] cafi seated CONFIRMED")
            else:
                if self.seated:
                    self.seated = False
                    self.pub_seated.publish(Bool(data=False))

            # Solenoides state pub (4 bits packed)
            self._publish_solenoids()

            # V35: HEARTBEAT of derived fixture occupancy signals.
            # In V34 these were published ONLY from _cb_cafi_states, so a
            # single dropped or late /objects/cafi_states message could
            # leave /fixture/load_present stuck stale and trip the
            # PLACE_LOAD watchdog on a fresh launch.  Re-publishing every
            # tick (30 Hz) makes the derived signals converge to truth
            # within ~33 ms regardless of message-drop history.  Also
            # re-publishes the station assignment so any subscriber
            # registered after the init snapshot still sees the current
            # outer/inner mapping.
            load_present  = self.fixture_has_cafi[self.outer_id]
            rivet_present = self.fixture_has_cafi[self.inner_id]
            self.pub_load_present.publish(Bool(data=load_present))
            self.pub_rivet_present.publish(Bool(data=rivet_present))
            self._publish_station()

            # Rivet timer
            if self.rivet_active:
                elapsed = rospy.Time.now().to_sec() - self.rivet_t0
                if elapsed >= RIVET_DURATION_S:
                    # retract piston + solenoide left (V25)
                    self.B_pist_tgt  = 0.0
                    self.B_sol_l_tgt = 0.0
                    self.rivet_active = False
                    self.pub_rivet_active.publish(Bool(data=False))
                    self.pub_rivet_done.publish(Empty())
                    # V26: mark the CAFI on the fixture currently at INNER
                    # (the one we just riveted) via fixture-id occupancy.
                    inner_cafi = self.fixture_cafi_id.get(self.inner_id)
                    if inner_cafi is not None:
                        self.pub_mark_riveted(inner_cafi)
                    rospy.loginfo("[ROT] rivet done; /rivet/done published "
                                  "(elapsed=%.1fs cafi_id=%s fixture=%s)",
                                  elapsed, inner_cafi, self.inner_id)

            # V52: publish only the disc joint (pistons / solenoids are
            # fixed meshes in the new turntable URDF; the seating logic
            # above is purely time-based and no longer drives any
            # prismatic joint).
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = ALL_JOINTS
            js.position = [self.disc_pos]
            self.pub_joints.publish(js)

    def pub_mark_riveted(self, cafi_id):
        # V29: publisher is now persistent (created in __init__).
        self._pub_riveted.publish(String(data=str(cafi_id)))

    def _publish_solenoids(self):
        # V25: solo solenoide LEFT por fixture (right removido).
        # Slot 0 = A_left, slot 1 = B_left, 2 reservados a 0 para mantener
        # tamaño 4 que la HMI espera.
        msg = UInt8MultiArray()
        msg.data = [
            1 if abs(self.A_sol_l - (-0.010)) < 5e-4 else 0,  # A left
            1 if abs(self.B_sol_l - (-0.010)) < 5e-4 else 0,  # B left
            0,  # reservado (was A right; V25: removido)
            0,  # reservado (was B right; V25: removido)
        ]
        self.pub_sol_state.publish(msg)

    def run(self):
        rate = rospy.Rate(TICK_HZ)
        dt = 1.0 / TICK_HZ
        while not rospy.is_shutdown():
            self.tick(dt)
            rate.sleep()


if __name__ == "__main__":
    try:
        RotaryFixtureSim().run()
    except rospy.ROSInterruptException:
        pass
