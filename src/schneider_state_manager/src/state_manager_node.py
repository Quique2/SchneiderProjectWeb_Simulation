#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_state_manager.state_manager_node  --  V23 cell FSM

V23 changes vs V22 (which was V20 untouched):
  - Adds the user-requested explicit "[SM] INDEX command +/-180 published"
    log right after publishing /disc/index_cmd, paired with a final
    "[SM] disc index done -> RIVETING" / "-> IDLE" log on completion.
    Together with the [ROT] disc_pos progress trace and the [VIZ] disc
    joint fused trace, the entire disc indexing chain is observable in
    rosout without RViz.
  - The cell-level FSM itself is unchanged: the V23 robot_controller is
    self-sequencing (approach -> pick -> close -> lift), and only fires
    /robot/motion_done=True after grasp_confirmed has also fired, so the
    "motion_done AND grasp_confirmed" condition the V22 SM already used
    still serves as the correct PICK_CONV completion criterion.
  - The PICK watchdog explanation log added in V22 is retained.

ORIGINAL V20 DOC BELOW
======================

Orquestador global de la celda. Implementa la FSM de la celda (no la del
ciclo de produccion - esa la lleva /cell/cycle_stage publicado por
state_manager tambien, pero usando los eventos de los nodos por dominio).

Estados de la celda (/cell/state):
  IDLE     - todo en home, listo para arrancar
  RUNNING  - ciclo automatico corriendo
  PAUSED   - STOP del operador (conveyor off, sin nuevas acciones)
  FAULT    - falla detectada (acumulacion, sensor stuck, etc.)

Etapas del ciclo (/cell/cycle_stage):
  IDLE | PICK_CONV | INDEX_DISC | RIVETING | PICK_RIVETED |
  INSPECT | PLACE_BIN | RETURN_HOME | STOPPED

El state_manager NO mueve actuadores directamente. Publica REQUESTS al
robot_controller / disc_controller / etc., suscribe sus DONE/STATE topics,
y avanza la FSM por EVENTOS.

Esto rompe la condicion muerta del V19: si una etapa no se completa, el
state_manager publica /cell/fault con la razon y la HMI lo expone. NO
hay timeouts ocultos que esconden silenciosamente errores.

----------------------------------------------------------------------------
ARQUITECTURA POR EVENTOS
----------------------------------------------------------------------------

Subs (eventos del mundo):
  /operator/spawn_cafi          std_msgs/Empty
  /operator/stop                std_msgs/Bool        toggle PAUSE/RESUME

  /conveyor/part_present_pick   std_msgs/Bool        DI1 sensor
  /conveyor/spawn_allowed       std_msgs/Bool        latched
  /robot/motion_done            std_msgs/Bool
  /robot/current_task           std_msgs/String      lo que el robot esta haciendo
  /gripper/grasp_confirmed      std_msgs/Bool        firme grasp
  /gripper/release_done         std_msgs/Empty
  /fixture/load_present         std_msgs/Bool
  /fixture/rivet_present        std_msgs/Bool
  /fixture/cafi_seated          std_msgs/Bool
  /disc/index_done              std_msgs/Empty
  /disc/state                   std_msgs/String      IDLE/INDEXING
  /rivet/done                   std_msgs/Empty
  /vision/presence              std_msgs/Bool
  /camera/result                std_msgs/String      PASS/FAIL/""
  /objects/cafi_states          std_msgs/String      JSON

Pubs (comandos al resto):
  /robot/request_pick_conveyor      std_msgs/Empty
  /robot/request_place_fixture      std_msgs/String
  /robot/request_pick_riveted       std_msgs/String
  /robot/request_place_vision       std_msgs/Empty
  /robot/request_place_accept_bin   std_msgs/Empty
  /robot/request_place_reject_bin   std_msgs/Empty
  /robot/request_home               std_msgs/Empty
  /disc/index_cmd                   std_msgs/Float32   target rad delta
  /rivet/start_cmd                  std_msgs/Empty
  /camera/trigger                   std_msgs/Empty
  /fixture/seat_cmd                 std_msgs/String    "load"/"rivet"

State publish:
  /cell/state                       std_msgs/String      latched
  /cell/cycle_stage                 std_msgs/String      latched
  /cell/fault                       std_msgs/String      latched ("" = no fault)
"""

import json
import threading

import rospy
from std_msgs.msg import Bool, Empty, Float32, String


# ============================================================
# CONSTANTES
# ============================================================
CELL_IDLE     = "IDLE"
CELL_RUNNING  = "RUNNING"
CELL_PAUSED   = "PAUSED"
CELL_FAULT    = "FAULT"

STAGE_IDLE          = "IDLE"
STAGE_PICK_CONV     = "PICK_CONV"
STAGE_PLACE_LOAD    = "PLACE_LOAD"
STAGE_SEAT          = "SEAT"           # solenoides acomodando CAFI
STAGE_INDEX         = "INDEX_DISC"
STAGE_RIVET         = "RIVETING"
STAGE_INDEX_BACK    = "INDEX_DISC_BACK"
STAGE_PICK_RIVETED  = "PICK_RIVETED"
STAGE_PLACE_VISION  = "PLACE_VISION"
STAGE_INSPECT       = "INSPECT"
# V28: NEW stage between INSPECT and PLACE_BIN.  V27 went straight from
# INSPECT to PLACE_BIN, but the CAFI was on the vision fixture, not in
# the gripper - the robot moved to the bin with an empty gripper and
# the CAFI was stranded at vision.  V28 inserts a PICK_VISION stage so
# the CAFI is physically picked up before being placed in the bin.
STAGE_PICK_VISION   = "PICK_VISION"
STAGE_PLACE_BIN     = "PLACE_BIN"
STAGE_RETURN_HOME   = "RETURN_HOME"
STAGE_STOPPED       = "STOPPED"

# Disco: angulo de indexado
DISC_INDEX_RAD = 3.141592653589793  # 180 deg

# Watchdogs (no esconden errores; si vencen, FAULT explicito)
# V30: PICK and PLACE expanded from 12s to 18s.  The new Lexium L03S
# chain produces longer joint paths (the elbow flip between picks at
# the conveyor and the LOAD fixture spans ~5 rad on J1) and the
# robot_controller's cosine-eased interpolation runs at JOINT_VEL_MAX
# = 1.4 rad/s, capping each segment at 4 s.  A typical trajectory
# (approach -> pose -> wait grip -> retreat -> HOME) needs ~16 s
# end-to-end.  V30 also has a 1.6 s settling delay after detach
# before the vision sensor reports presence, which used to overflow
# the 12 s budget.
WD_PICK_S       = 18.0
WD_PLACE_S      = 18.0
WD_SEAT_S       = 6.0
WD_INDEX_S      = 6.0
WD_RIVET_S      = 40.0   # 30s rivet + 10s margen
WD_INSPECT_S    = 6.0


# ============================================================
# FSM
# ============================================================
class StateManager(object):

    def __init__(self):
        rospy.init_node("schneider_state_manager", anonymous=False)

        # ---- Estado celda ----
        self.cell_state    = CELL_IDLE
        self.cycle_stage   = STAGE_IDLE
        self.fault_reason  = ""

        # ---- Evento de mundo ----
        self.part_present_pick = False
        # V27: PICK_CONV is gated on this TIGHT signal (CAFI center within
        # +- 20 mm of PICK_X), not on the wide DI1 sensor.
        self.part_ready_for_pick = False
        self.spawn_allowed     = True

        self.robot_motion_done = True
        self.robot_task        = "idle"
        self.gripper_grasp     = False
        self.gripper_releasing = False

        self.fixture_load_present  = False
        self.fixture_rivet_present = False
        self.fixture_cafi_seated   = False

        self.disc_state            = "IDLE"
        self.disc_index_done_flag  = False
        self.rivet_done_flag       = False

        self.vision_presence       = False
        self.camera_result         = ""

        self.cafi_states           = []  # list[dict]

        # V26: fixture-id station assignment, kept fresh by
        # /disc/station_assignment.  Initial: A at LOAD, B at RIVET.
        self.outer_id = "A"
        self.inner_id = "B"

        # ---- Watchdog timestamp ----
        self.stage_t0 = rospy.Time.now().to_sec()

        # ---- Concurrency ----
        self.lock = threading.RLock()

        # ---- Publishers ----
        self.pub_cell_state   = rospy.Publisher("/cell/state",        String,
                                                queue_size=1, latch=True)
        self.pub_cycle_stage  = rospy.Publisher("/cell/cycle_stage",  String,
                                                queue_size=1, latch=True)
        self.pub_fault        = rospy.Publisher("/cell/fault",        String,
                                                queue_size=1, latch=True)

        self.pub_req_pick_conv      = rospy.Publisher(
            "/robot/request_pick_conveyor",   Empty,  queue_size=1)
        self.pub_req_place_load     = rospy.Publisher(
            "/robot/request_place_fixture",   String, queue_size=1)
        self.pub_req_pick_riveted   = rospy.Publisher(
            "/robot/request_pick_riveted",    String, queue_size=1)
        self.pub_req_place_vision   = rospy.Publisher(
            "/robot/request_place_vision",    Empty,  queue_size=1)
        # V28: pick CAFI back from vision station after verdict
        self.pub_req_pick_vision    = rospy.Publisher(
            "/robot/request_pick_vision",     Empty,  queue_size=1)
        self.pub_req_place_accept   = rospy.Publisher(
            "/robot/request_place_accept_bin", Empty, queue_size=1)
        self.pub_req_place_reject   = rospy.Publisher(
            "/robot/request_place_reject_bin", Empty, queue_size=1)
        self.pub_req_home           = rospy.Publisher(
            "/robot/request_home",            Empty,  queue_size=1)

        self.pub_disc_index   = rospy.Publisher(
            "/disc/index_cmd",         Float32, queue_size=1)
        self.pub_rivet_start  = rospy.Publisher(
            "/rivet/start_cmd",        Empty,   queue_size=1)
        self.pub_camera_trig  = rospy.Publisher(
            "/camera/trigger",         Empty,   queue_size=1)
        self.pub_fixture_seat = rospy.Publisher(
            "/fixture/seat_cmd",       String,  queue_size=1)
        self.pub_fixture_unseat = rospy.Publisher(
            "/fixture/unseat_cmd",     String,  queue_size=1)

        # ---- Subscribers ----
        rospy.Subscriber("/operator/spawn_cafi",       Empty,
                         self._cb_op_spawn)
        rospy.Subscriber("/operator/stop",             Bool,
                         self._cb_op_stop)
        rospy.Subscriber("/conveyor/part_present_pick", Bool,
                         self._cb_part_present)
        rospy.Subscriber("/conveyor/part_ready_for_pick", Bool,
                         self._cb_part_ready)
        rospy.Subscriber("/conveyor/spawn_allowed",    Bool,
                         self._cb_spawn_allowed)
        rospy.Subscriber("/robot/motion_done",         Bool,
                         self._cb_motion_done)
        rospy.Subscriber("/robot/current_task",        String,
                         self._cb_robot_task)
        rospy.Subscriber("/gripper/grasp_confirmed",   Bool,
                         self._cb_grasp_confirmed)
        rospy.Subscriber("/gripper/release_done",      Empty,
                         self._cb_release_done)
        rospy.Subscriber("/fixture/load_present",      Bool,
                         self._cb_fixture_load)
        rospy.Subscriber("/fixture/rivet_present",     Bool,
                         self._cb_fixture_rivet)
        rospy.Subscriber("/fixture/cafi_seated",       Bool,
                         self._cb_cafi_seated)
        rospy.Subscriber("/disc/index_done",           Empty,
                         self._cb_disc_done)
        rospy.Subscriber("/disc/state",                String,
                         self._cb_disc_state)
        # V26: station assignment from rotary_fixture_sim
        rospy.Subscriber("/disc/station_assignment",   String,
                         self._cb_station_assignment)
        rospy.Subscriber("/rivet/done",                Empty,
                         self._cb_rivet_done)
        rospy.Subscriber("/vision/presence",           Bool,
                         self._cb_vision_presence)
        rospy.Subscriber("/camera/result",             String,
                         self._cb_camera_result)
        rospy.Subscriber("/objects/cafi_states",       String,
                         self._cb_cafi_states)

        # ---- Initial publish (latched) ----
        self._publish_state()

        rospy.loginfo("[SM] V35 state_manager init - cell=IDLE; "
                      "PLACE_LOAD/PLACE_VISION use source-of-truth "
                      "/objects/cafi_states as parallel gate (defense "
                      "against derived-signal staleness)")

    # =========================================================
    # CALLBACKS
    # =========================================================
    def _cb_op_spawn(self, _m):
        with self.lock:
            if self.cell_state == CELL_IDLE:
                self._transition_cell(CELL_RUNNING)

    def _cb_op_stop(self, msg):
        with self.lock:
            if bool(msg.data):
                rospy.logwarn("[SM] STOP del operador")
                if self.cell_state == CELL_RUNNING:
                    self._transition_cell(CELL_PAUSED)
                    self._set_stage(STAGE_STOPPED)
            else:
                rospy.loginfo("[SM] RESUME del operador")
                if self.cell_state in (CELL_PAUSED, CELL_FAULT):
                    self.fault_reason = ""
                    self.pub_fault.publish(String(data=""))
                    self._transition_cell(CELL_RUNNING)
                    self._set_stage(STAGE_IDLE)

    def _cb_part_present(self, m):    self.part_present_pick = bool(m.data)
    def _cb_part_ready(self, m):      self.part_ready_for_pick = bool(m.data)
    def _cb_spawn_allowed(self, m):   self.spawn_allowed     = bool(m.data)
    def _cb_motion_done(self, m):     self.robot_motion_done = bool(m.data)
    def _cb_robot_task(self, m):      self.robot_task        = m.data
    def _cb_grasp_confirmed(self, m): self.gripper_grasp     = bool(m.data)
    def _cb_release_done(self, _m):
        self.gripper_releasing = False

    def _cb_fixture_load(self, m):    self.fixture_load_present  = bool(m.data)
    def _cb_fixture_rivet(self, m):   self.fixture_rivet_present = bool(m.data)
    def _cb_cafi_seated(self, m):     self.fixture_cafi_seated   = bool(m.data)
    def _cb_disc_done(self, _m):      self.disc_index_done_flag  = True
    def _cb_disc_state(self, m):      self.disc_state = m.data

    def _cb_station_assignment(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        new_outer = d.get("outer", self.outer_id)
        new_inner = d.get("inner", self.inner_id)
        if new_outer != self.outer_id or new_inner != self.inner_id:
            rospy.loginfo("[SM] station_assignment update outer=%s inner=%s",
                          new_outer, new_inner)
        self.outer_id = new_outer
        self.inner_id = new_inner
    def _cb_rivet_done(self, _m):     self.rivet_done_flag = True
    def _cb_vision_presence(self, m): self.vision_presence = bool(m.data)
    def _cb_camera_result(self, m):   self.camera_result = m.data

    def _cb_cafi_states(self, msg):
        try:
            self.cafi_states = json.loads(msg.data) or []
        except Exception:
            self.cafi_states = []

    # =========================================================
    # HELPERS DE PUBLICACION
    # =========================================================
    def _publish_state(self):
        self.pub_cell_state.publish(String(data=self.cell_state))
        self.pub_cycle_stage.publish(String(data=self.cycle_stage))
        self.pub_fault.publish(String(data=self.fault_reason))

    def _transition_cell(self, new):
        if new == self.cell_state:
            return
        rospy.loginfo("[SM] cell %s -> %s", self.cell_state, new)
        self.cell_state = new
        self.pub_cell_state.publish(String(data=new))

    def _set_stage(self, new):
        if new != self.cycle_stage:
            rospy.loginfo("[SM] stage %s -> %s", self.cycle_stage, new)
            self.cycle_stage = new
            self.pub_cycle_stage.publish(String(data=new))
        self.stage_t0 = rospy.Time.now().to_sec()
        # limpia flags volatiles
        self.disc_index_done_flag = False
        self.rivet_done_flag      = False
        self.robot_motion_done    = False

    def _enter_fault(self, reason):
        rospy.logerr("[SM] FAULT: %s", reason)
        self.fault_reason = reason
        self.pub_fault.publish(String(data=reason))
        self._transition_cell(CELL_FAULT)
        self._set_stage(STAGE_STOPPED)

    def _stage_elapsed(self):
        return rospy.Time.now().to_sec() - self.stage_t0

    # =========================================================
    # CAFI helpers (sobre snapshot publicado por object_manager)
    # =========================================================
    def _cafi_on_conveyor_at_pick(self):
        """V27: dispatch PICK_CONV only when the CAFI center is at the
        tight pick window (NOT just inside DI1's wide window).  This is
        what eliminates the 110 mm IK/CAFI misalignment that caused
        the V26 PICK_CONV watchdog."""
        if not self.part_ready_for_pick:
            return None
        for c in self.cafi_states:
            if c.get("location") == "on_conveyor" and c.get("at_sensor"):
                return c
        return None

    def _outer_cafi(self):
        """V26: CAFI at the outer (LOAD) station = CAFI whose fixture_id
        matches the fixture currently at outer."""
        for c in self.cafi_states:
            if (c.get("location") == "in_fixture_" + self.outer_id and
                    c.get("fixture_id") == self.outer_id):
                return c
        return None

    def _inner_cafi(self):
        for c in self.cafi_states:
            if (c.get("location") == "in_fixture_" + self.inner_id and
                    c.get("fixture_id") == self.inner_id):
                return c
        return None

    def _vision_cafi(self):
        for c in self.cafi_states:
            if c.get("location") == "at_vision":
                return c
        return None

    # =========================================================
    # FSM TICK
    # =========================================================
    def tick(self):
        with self.lock:
            if self.cell_state != CELL_RUNNING:
                return

            stage = self.cycle_stage
            outer = self._outer_cafi()
            inner = self._inner_cafi()
            at_vis = self._vision_cafi()

            # --------- IDLE: dispatcher por prioridad ---------
            #
            # V29: extended priority list to support BUFFER LOGIC with
            # two fixtures + conveyor wait + parallel rivet/load.
            # Added:
            #   P4 (inner riveted AND outer unriveted) -> INDEX swap
            #   P6 (outer empty + conveyor CAFI ready) regardless of
            #      whether inner has unriveted (riveting) or is empty
            if stage == STAGE_IDLE:
                outer_unrivet = outer and not outer.get("riveted")
                outer_rivet   = outer and outer.get("riveted")
                inner_unrivet = inner and not inner.get("riveted")
                inner_rivet   = inner and inner.get("riveted")
                conv = self._cafi_on_conveyor_at_pick()

                # P1: V28 - CAFI en vision con verdict -> PICK_VISION
                if at_vis and at_vis.get("verdict") in ("PASS", "FAIL"):
                    rospy.loginfo("[SM] dispatch: PICK_VISION "
                                  "(verdict=%s, will then place in bin)",
                                  at_vis.get("verdict"))
                    self._set_stage(STAGE_PICK_VISION)
                    self.pub_req_pick_vision.publish(Empty())
                    return

                # P2: outer tiene riveted -> PICK_RIVETED -> VISION
                if outer_rivet:
                    rospy.loginfo("[SM] dispatch: PICK_RIVETED")
                    self._set_stage(STAGE_PICK_RIVETED)
                    self.pub_req_pick_riveted.publish(
                        String(data="outer"))
                    return

                # P3: inner riveted + outer vacio -> INDEX_BACK (-180)
                if inner_rivet and not outer:
                    rospy.loginfo("[SM] dispatch: INDEX_BACK (rotar -180 deg, "
                                  "inner has riveted, outer empty)")
                    self._set_stage(STAGE_INDEX_BACK)
                    self.pub_disc_index.publish(
                        Float32(data=-DISC_INDEX_RAD))
                    rospy.loginfo("[SM] INDEX command -180 published on "
                                  "/disc/index_cmd (delta=%.3f rad)",
                                  -DISC_INDEX_RAD)
                    return

                # P4 (V29): inner riveted + outer unriveted -> INDEX +180
                # SWAP both fixtures: riveted moves to outer (for pickup),
                # unriveted moves to inner (rivet starts on the new one).
                # This handles the buffer case where a 2nd CAFI was loaded
                # on outer while the 1st was being riveted at inner.
                if inner_rivet and outer_unrivet:
                    rospy.loginfo("[SM] dispatch: INDEX swap (rotar +180 deg, "
                                  "inner has riveted, outer has unriveted - "
                                  "buffer swap)")
                    self._set_stage(STAGE_INDEX)
                    self.pub_disc_index.publish(
                        Float32(data=+DISC_INDEX_RAD))
                    rospy.loginfo("[SM] INDEX command +180 published on "
                                  "/disc/index_cmd (delta=%.3f rad)",
                                  +DISC_INDEX_RAD)
                    return

                # P5: outer unriveted + inner vacio -> INDEX +180 + rivet
                if outer_unrivet and not inner:
                    rospy.loginfo("[SM] dispatch: INDEX (rotar +180 deg, "
                                  "outer has unriveted, inner empty)")
                    self._set_stage(STAGE_INDEX)
                    self.pub_disc_index.publish(
                        Float32(data=+DISC_INDEX_RAD))
                    rospy.loginfo("[SM] INDEX command +180 published on "
                                  "/disc/index_cmd (delta=%.3f rad)",
                                  +DISC_INDEX_RAD)
                    return

                # P6 (V29 relaxed): outer empty + CAFI on conveyor ready
                # -> PICK_CONV.  Allowed even if inner is being riveted
                # (inner has unriveted CAFI).  This is the BUFFER load
                # case the user requires.
                if conv and not outer:
                    if inner_unrivet:
                        rospy.loginfo("[SM] dispatch: PICK_CONV BUFFERED "
                                      "(CAFI %s at conveyor pick, outer empty,"
                                      " inner CAFI %s still being riveted)",
                                      conv.get("id"), inner.get("id"))
                    else:
                        rospy.loginfo("[SM] dispatch: PICK_CONV (CAFI %s at "
                                      "conveyor pick, outer fixture empty)",
                                      conv.get("id"))
                    self._set_stage(STAGE_PICK_CONV)
                    self.pub_req_pick_conv.publish(Empty())
                    rospy.loginfo("[SM] /robot/request_pick_conveyor published")
                    return

                # Sin trabajo -> RETURN_HOME una vez si no esta home
                if (self.robot_task != "idle" and
                        self.robot_task != "home"):
                    self._set_stage(STAGE_RETURN_HOME)
                    self.pub_req_home.publish(Empty())
                    return

                return

            # --------- PICK_CONV ---------
            if stage == STAGE_PICK_CONV:
                if self._stage_elapsed() > WD_PICK_S:
                    # V22: log WHY before falling into FAULT
                    rospy.logerr("[SM] PICK_CONV watchdog FIRED at %.1fs. "
                                 "motion_done=%s gripper_grasp=%s task=%s "
                                 "outer_present=%s",
                                 self._stage_elapsed(),
                                 self.robot_motion_done,
                                 self.gripper_grasp,
                                 self.robot_task,
                                 self.fixture_load_present)
                    self._enter_fault("watchdog PICK_CONV")
                    return
                if self.robot_motion_done and self.gripper_grasp:
                    rospy.loginfo("[SM] PICK_CONV complete -> PLACE_LOAD "
                                  "(motion_done=True grasp_confirmed=True)")
                    self._set_stage(STAGE_PLACE_LOAD)
                    self.pub_req_place_load.publish(String(data="outer"))
                return

            # --------- PLACE_LOAD ---------
            # V35: precondition uses SOURCE-OF-TRUTH (cafi_states) as a
            # parallel gate next to the derived /fixture/load_present.
            # In V34 the derived signal was published only on the
            # rotary_fixture_sim cafi_states callback; a single missed
            # message could leave it stuck stale and trip this watchdog
            # even though SM already knew (from its own /objects/cafi_states
            # subscription) that the CAFI had landed.  Either gate now
            # advances the FSM.  The V35 rotary heartbeat keeps the
            # derived signal fresh too, so this is defence in depth.
            if stage == STAGE_PLACE_LOAD:
                outer_cafi_now = outer  # already resolved above from cafi_states
                if self._stage_elapsed() > WD_PLACE_S:
                    rospy.logerr("[SM] PLACE_LOAD watchdog FIRED at %.1fs. "
                                 "motion_done=%s gripper_grasp=%s "
                                 "fixture_load_present=%s "
                                 "outer_cafi_from_cafi_states=%s "
                                 "cafi_states_size=%d outer_id=%s",
                                 self._stage_elapsed(),
                                 self.robot_motion_done,
                                 self.gripper_grasp,
                                 self.fixture_load_present,
                                 (outer_cafi_now.get("id")
                                  if outer_cafi_now else None),
                                 len(self.cafi_states),
                                 self.outer_id)
                    self._enter_fault("watchdog PLACE_LOAD")
                    return
                # Robot ha colocado y liberado, sensor del fixture confirma
                # presencia.  V35: o bien el derivado o bien la fuente
                # directa /objects/cafi_states confirman que la CAFI está
                # en el fixture outer.
                placed = (self.fixture_load_present
                          or outer_cafi_now is not None)
                if (self.robot_motion_done
                        and not self.gripper_grasp
                        and placed):
                    rospy.loginfo(
                        "[SM] place load ok -> SEAT "
                        "(fixture_load_present=%s outer_cafi=%s)",
                        self.fixture_load_present,
                        (outer_cafi_now.get("id")
                         if outer_cafi_now else None))
                    self._set_stage(STAGE_SEAT)
                    self.pub_fixture_seat.publish(String(data="outer"))
                return

            # --------- SEAT (solenoides clamping CAFI) ---------
            if stage == STAGE_SEAT:
                if self._stage_elapsed() > WD_SEAT_S:
                    self._enter_fault("watchdog SEAT solenoides")
                    return
                if self.fixture_cafi_seated:
                    rospy.loginfo("[SM] CAFI seated -> IDLE (dispatcher)")
                    self._set_stage(STAGE_IDLE)
                return

            # --------- INDEX (rotar +180) ---------
            # V29: after INDEX completes, publish rivet_start_cmd AND
            # return to IDLE (NOT to STAGE_RIVET).  This frees the SM
            # to dispatch other actions (PICK_CONV for a buffered CAFI)
            # while the rivet runs in parallel for 30 s.  /rivet/done
            # callback (self.rivet_done_flag) and object_manager's
            # mark_riveted on /objects/cafi_states drive the next
            # dispatch via the IDLE priority list (P2 / P3 / P4).
            if stage == STAGE_INDEX:
                if self._stage_elapsed() > WD_INDEX_S:
                    rospy.logerr("[SM] INDEX watchdog FIRED at %.1fs. "
                                 "disc_index_done_flag=%s disc_state=%s",
                                 self._stage_elapsed(),
                                 self.disc_index_done_flag, self.disc_state)
                    self._enter_fault("watchdog INDEX")
                    return
                if self.disc_index_done_flag:
                    rospy.loginfo("[SM] V29 disc index done -> rivet_start + "
                                  "IDLE (rivet runs in parallel, dispatcher "
                                  "free to pick a buffered CAFI from conveyor)"
                                  "  outer=%s inner=%s",
                                  self.outer_id, self.inner_id)
                    rospy.loginfo("[SM] request rivet start for fixture=%s",
                                  self.inner_id)
                    self.pub_rivet_start.publish(Empty())
                    self._set_stage(STAGE_IDLE)
                return

            # --------- INDEX_BACK (rotar -180) ---------
            if stage == STAGE_INDEX_BACK:
                if self._stage_elapsed() > WD_INDEX_S:
                    rospy.logerr("[SM] INDEX_BACK watchdog FIRED at %.1fs. "
                                 "disc_index_done_flag=%s disc_state=%s",
                                 self._stage_elapsed(),
                                 self.disc_index_done_flag, self.disc_state)
                    self._enter_fault("watchdog INDEX_BACK")
                    return
                if self.disc_index_done_flag:
                    rospy.loginfo("[SM] disc index done -> IDLE "
                                  "(received /disc/index_done from rotary "
                                  "sim after %.1fs)", self._stage_elapsed())
                    self._set_stage(STAGE_IDLE)
                return

            # --------- RIVETING ---------
            if stage == STAGE_RIVET:
                if self._stage_elapsed() > WD_RIVET_S:
                    rospy.logerr("[SM] RIVET watchdog FIRED at %.1fs. "
                                 "rivet_done_flag=%s rivet_present=%s "
                                 "outer=%s inner=%s",
                                 self._stage_elapsed(),
                                 self.rivet_done_flag,
                                 self.fixture_rivet_present,
                                 self.outer_id, self.inner_id)
                    self._enter_fault("watchdog RIVET (no /rivet/done)")
                    return
                if self.rivet_done_flag:
                    rospy.loginfo("[SM] rivet done -> next stage "
                                  "(IDLE dispatcher will route)")
                    self._set_stage(STAGE_IDLE)
                return

            # --------- PICK_RIVETED ---------
            if stage == STAGE_PICK_RIVETED:
                if self._stage_elapsed() > WD_PICK_S:
                    self._enter_fault("watchdog PICK_RIVETED")
                    return
                if self.robot_motion_done and self.gripper_grasp:
                    rospy.loginfo("[SM] pick riveted ok -> PLACE_VISION")
                    self._set_stage(STAGE_PLACE_VISION)
                    self.pub_req_place_vision.publish(Empty())
                return

            # --------- PLACE_VISION ---------
            # V35: same robust precondition pattern as PLACE_LOAD.
            if stage == STAGE_PLACE_VISION:
                at_vis_now = at_vis  # resolved above from cafi_states
                if self._stage_elapsed() > WD_PLACE_S:
                    rospy.logerr("[SM] PLACE_VISION watchdog FIRED at %.1fs. "
                                 "motion_done=%s gripper_grasp=%s "
                                 "vision_presence=%s "
                                 "at_vision_cafi=%s cafi_states_size=%d",
                                 self._stage_elapsed(),
                                 self.robot_motion_done,
                                 self.gripper_grasp,
                                 self.vision_presence,
                                 (at_vis_now.get("id")
                                  if at_vis_now else None),
                                 len(self.cafi_states))
                    self._enter_fault("watchdog PLACE_VISION")
                    return
                placed_at_vision = (self.vision_presence
                                    or at_vis_now is not None)
                if (self.robot_motion_done
                        and not self.gripper_grasp
                        and placed_at_vision):
                    rospy.loginfo(
                        "[SM] place vision ok -> INSPECT "
                        "(vision_presence=%s at_vision_cafi=%s)",
                        self.vision_presence,
                        (at_vis_now.get("id")
                         if at_vis_now else None))
                    self._set_stage(STAGE_INSPECT)
                    self.pub_camera_trig.publish(Empty())
                return

            # --------- INSPECT ---------
            if stage == STAGE_INSPECT:
                if self._stage_elapsed() > WD_INSPECT_S:
                    self._enter_fault("watchdog INSPECT")
                    return
                if self.camera_result in ("PASS", "FAIL"):
                    rospy.loginfo("[SM] vision verdict=%s -> IDLE "
                                  "(dispatcher will route to PICK_VISION)",
                                  self.camera_result)
                    self._set_stage(STAGE_IDLE)
                return

            # --------- PICK_VISION (V28) ---------
            # The cobot moves to the vision fixture, closes the gripper on
            # the CAFI sitting there, and lifts.  Then the IDLE dispatcher
            # picks up "CAFI in_gripper with verdict" and routes to
            # PLACE_BIN with the CAFI actually in hand.
            if stage == STAGE_PICK_VISION:
                if self._stage_elapsed() > WD_PICK_S:
                    rospy.logerr("[SM] PICK_VISION watchdog FIRED at %.1fs. "
                                 "motion_done=%s gripper_grasp=%s "
                                 "vision_present=%s",
                                 self._stage_elapsed(),
                                 self.robot_motion_done,
                                 self.gripper_grasp,
                                 self.vision_presence)
                    self._enter_fault("watchdog PICK_VISION")
                    return
                if self.robot_motion_done and self.gripper_grasp:
                    # Look up verdict from any CAFI now in gripper
                    verdict = ""
                    for c in self.cafi_states:
                        if c.get("location") == "in_gripper":
                            verdict = c.get("verdict", "") or ""
                            break
                    if verdict not in ("PASS", "FAIL"):
                        # Fallback to camera_result (latched)
                        verdict = self.camera_result
                    rospy.loginfo("[SM] PICK_VISION complete -> PLACE_BIN "
                                  "(verdict=%s)", verdict)
                    self._set_stage(STAGE_PLACE_BIN)
                    if verdict == "PASS":
                        self.pub_req_place_accept.publish(Empty())
                    else:
                        self.pub_req_place_reject.publish(Empty())
                return

            # --------- PLACE_BIN ---------
            if stage == STAGE_PLACE_BIN:
                if self._stage_elapsed() > WD_PLACE_S * 2:
                    self._enter_fault("watchdog PLACE_BIN")
                    return
                if self.robot_motion_done and not self.gripper_grasp:
                    rospy.loginfo("[SM] place bin ok -> IDLE")
                    # V28: reset camera verdict so the IDLE dispatcher
                    # does not loop into PICK_VISION again.  The CAFI is
                    # in the bin; the cycle is done.
                    self.camera_result = ""
                    self._set_stage(STAGE_IDLE)
                return

            # --------- RETURN_HOME ---------
            if stage == STAGE_RETURN_HOME:
                if self.robot_motion_done:
                    self._set_stage(STAGE_IDLE)
                return

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()


if __name__ == "__main__":
    try:
        StateManager().run()
    except rospy.ROSInterruptException:
        pass
