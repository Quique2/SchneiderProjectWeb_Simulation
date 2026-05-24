#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_conveyor_sim.conveyor_sim_node  --  V27

V27 PICK_CONV SYNC FIX
======================
V26 stopped the belt as soon as DI1 (the SICK photoelectric sensor) went
HIGH, but the sensor window is +- 110 mm wide around the pick X. The CAFI
center stopped 110 mm SHORT of the IK pick target, so the geometric
grasp check (+- 100 mm tolerance) failed and the PICK_CONV watchdog
fired.

V27 separates two concepts:
  * /conveyor/part_present_pick   - the REAL sensor (wide window, kept
                                    so the HMI DI1 lamp and the spawn
                                    interlock still reflect physical
                                    presence).
  * /conveyor/part_ready_for_pick - NEW: True only when the CAFI center
                                    is within PICK_TOL_X of PICK_X (tight,
                                    20 mm).  The belt motor stops only
                                    on this tight condition.  The
                                    state_manager waits for this before
                                    requesting PICK_CONV.

No tolerances were inflated; no fake grasp shortcut was added.

V20 ORIGINAL DOC BELOW
======================

Simulacion del conveyor + sensor SICK + spawn interlock.

ARCHITECTURE NOTE (vs V19 monolith):
  El conveyor V19 vivia mezclado dentro de process_simulator. V20 lo
  pone en su propio nodo que solo se ocupa de:
    - velocidad del belt
    - posicion de bolsas en el belt (publica al object_manager)
    - sensor de presencia DI1 en el pick
    - gating de spawn (anti-acumulacion)
    - fault por doble pieza

  El nodo NO crea CAFIs (eso lo hace object_manager). Solo le PIDE al
  object_manager que cree uno cuando el operador pulsa spawn y el gating
  lo permite.

GEOMETRIA (mirror de schneider_cell.urdf.xacro V20):
  belt top z = 1.277
  belt X span = [0.719, 1.419]  Y centre = 0.860821
  spawn X = 0.769 (oeste)
  pick   X = 1.380 (este, alineado con sensor face)
  sensor face X = 1.400
  sensor window = +- 0.110 m en X centrado en sensor_x
                  Y en [0.770, 0.950]
                  Z en sensor_z +- 0.060

SPAWN INTERLOCK (per user brief, seccion 11.1):
  El spawn NO se permite cuando:
    - DI1 sensor conveyor activo (parte presente en pick)
    - spawn zone ocupada (CAFI < 0.4 m de spawn_x)
    - hay >= 1 CAFI a menos de MIN_SEPARATION_M de otra
    - cell_state in {PAUSED, FAULT, IDLE}  (solo permite en RUNNING)

ANTI-ACUMULACION (per user brief, seccion 11.3):
  Si en cualquier tick se detectan 2 CAFIs separados < MIN_SEPARATION_M,
  se publica:
    - /cell/fault con razon "conveyor doble pieza / acumulacion"
    - /conveyor/motor_state False
    - el state_manager hara HOLD/FAULT
"""

import json
import math
import threading

import rospy
from std_msgs.msg import Bool, Empty, String


# ============================================================
# GEOMETRIA V39 -- exact match con schneider_cell.urdf.xacro DXF
# layout.  V38 inherited V20 belt coords (the CAFI spawned at
# (0.810, 0.861) -- the V20 belt centre, completely off the V37/V38
# belt at (1.370, 1.365)).  V39 fixes them.
# ============================================================
BELT_TOP_Z      = 1.070          # mesa top (1.000) + belt rise (0.070)
BELT_X_WEST     = 1.183          # belt centre (1.370) - half length (0.187)
BELT_X_EAST     = 1.620          # suministro east face
BELT_Y          = 1.365          # belt centre Y per DXF
# V39: CAFI enters from "Suministro CAFI" at the EAST end and travels
# WEST to the pick at x=1.235 (south-rail notch).
SPAWN_X         = 1.620
PICK_X          = 1.235
PICK_TOL_X      = 0.020          # +- 20 mm para considerar "en pick"

# V39 sensor: SICK GRTE18S at (1.235, 1.290), beam crosses belt south->north.
CONV_SENSOR_X       = 1.235
CONV_SENSOR_HW_X    = 0.040
CONV_SENSOR_Y_RANGE = (1.290, 1.440)
CONV_SENSOR_Z       = BELT_TOP_Z + 0.0125    # CAFI centre height on belt
CONV_SENSOR_HW_Z    = 0.060

BELT_SPEED          = 0.10        # m/s nominal
SPAWN_GUARD_RADIUS  = 0.150       # m   no se permite spawn si hay CAFI a < 0.15 m
MIN_SEPARATION_M    = 0.250       # m   < 0.25 entre CAFI = acumulacion -> fault
TICK_HZ             = 30.0
PUBLISH_HZ          = 10.0

# Cell states from /cell/state
CELL_RUNNING = "RUNNING"


class ConveyorSimNode(object):

    def __init__(self):
        rospy.init_node("schneider_conveyor_sim", anonymous=False)

        # ---- Cell state mirror ----
        self.cell_state    = "IDLE"
        self.motor_enabled = True
        # V26: cycle_stage mirror -- spawn is BLOCKED during INDEX_DISC
        # and INDEX_DISC_BACK to avoid a second CAFI arriving on the
        # belt mid-rotation (the user reported HMI allowed it in V25).
        self.cycle_stage   = "IDLE"

        # ---- CAFI snapshot from object_manager ----
        self.cafis = []   # list of dict {id, x, y, z, location}

        self.lock = threading.RLock()

        # ---- Publishers ----
        self.pub_present       = rospy.Publisher(
            "/conveyor/part_present_pick",  Bool,   queue_size=1, latch=True)
        # V27: NEW topic; True only when CAFI center is at PICK_X (tight).
        self.pub_ready         = rospy.Publisher(
            "/conveyor/part_ready_for_pick", Bool,  queue_size=1, latch=True)
        self.pub_spawn_allowed = rospy.Publisher(
            "/conveyor/spawn_allowed",      Bool,   queue_size=1, latch=True)
        self.pub_motor_state   = rospy.Publisher(
            "/conveyor/motor_state",        Bool,   queue_size=1, latch=True)
        self.pub_occupancy     = rospy.Publisher(
            "/conveyor/occupancy",          String, queue_size=2)
        # spawn request to object_manager (gated)
        self.pub_spawn_req     = rospy.Publisher(
            "/objects/spawn_request",       Empty,  queue_size=2)
        self.pub_spawn_result  = rospy.Publisher(
            "/objects/spawn_result",        String, queue_size=2)
        # fault to cell state_manager
        self.pub_fault         = rospy.Publisher(
            "/cell/fault",                  String, queue_size=1, latch=True)
        # belt step request to object_manager (advance bolsas on belt)
        self.pub_belt_step     = rospy.Publisher(
            "/conveyor/belt_step",          String, queue_size=5)

        # ---- Subscribers ----
        rospy.Subscriber("/operator/spawn_cafi",   Empty,  self._cb_op_spawn)
        rospy.Subscriber("/cell/state",            String, self._cb_cell_state)
        rospy.Subscriber("/cell/cycle_stage",      String, self._cb_cycle_stage)
        rospy.Subscriber("/objects/cafi_states",   String, self._cb_cafi_states)

        # initial latched publish
        self.pub_present.publish(Bool(data=False))
        self.pub_ready.publish(Bool(data=False))
        self.pub_spawn_allowed.publish(Bool(data=False))
        self.pub_motor_state.publish(Bool(data=True))

        rospy.loginfo("[CONV] V27 conveyor_sim init; pick_x=%.3f tol=%.3f "
                      "speed=%.2f m/s  (DI1 sensor window +-%.3f m, belt "
                      "stops only when CAFI center within +-%.3f m of "
                      "pick_x)",
                      PICK_X, PICK_TOL_X, BELT_SPEED,
                      CONV_SENSOR_HW_X, PICK_TOL_X)

    # =========================================================
    # CALLBACKS
    # =========================================================
    def _cb_op_spawn(self, _m):
        """Spawn interlock - rechaza si no es permitido."""
        with self.lock:
            ok, reason = self._spawn_can_proceed()
            if not ok:
                self.pub_spawn_result.publish(String(data=json.dumps({
                    "success": False, "reason": reason})))
                rospy.logwarn("[CONV] spawn DENIED: %s", reason)
                return
            self.pub_spawn_req.publish(Empty())
            self.pub_spawn_result.publish(String(data=json.dumps({
                "success": True, "reason": "spawn granted"})))
            rospy.loginfo("[CONV] spawn granted -> object_manager")

    def _cb_cell_state(self, m):
        with self.lock:
            self.cell_state = m.data
            self.motor_enabled = (self.cell_state == CELL_RUNNING)
            self.pub_motor_state.publish(Bool(data=self.motor_enabled))

    def _cb_cycle_stage(self, m):
        with self.lock:
            self.cycle_stage = m.data

    def _cb_cafi_states(self, m):
        try:
            self.cafis = json.loads(m.data) or []
        except Exception:
            self.cafis = []

    # =========================================================
    # SPAWN GATING
    # =========================================================
    def _spawn_can_proceed(self):
        """Return (ok, reason). Backend-side enforcement: aunque el HMI
        publique al topic, este nodo decide si el object_manager spawnea.

        V55: el spawn solo se acepta cuando cell == RUNNING.  IDLE ya
        no autoarranca el ciclo (eso lo hace /operator/start desde el
        HMI); un spawn en IDLE seria un click previo a START y debe
        rechazarse para no dejar una CAFI parada en el conveyor antes
        de que el dispatcher pueda recogerla."""
        if self.cell_state != CELL_RUNNING:
            return False, "cell in {} (need RUNNING; press START first)".format(
                self.cell_state)
        # V29 relaxed: only short blocking stages stop spawn.  The
        # disc indexing takes ~2.6 s so we keep it blocked.  RIVETING
        # no longer occurs (V29 SM publishes rivet_start and returns
        # to IDLE; rivet runs in parallel), so don't block on it.
        # SEAT (2 s) and INDEX_DISC* (2.6 s) are still blocked so the
        # spawn isn't reset to a half-second-old transient state.
        if self.cycle_stage in ("INDEX_DISC", "INDEX_DISC_BACK", "SEAT"):
            return False, "stage {} (bloqueado durante index/seat)".format(
                self.cycle_stage)
        # V29 user requirement E.3: no permitir spawn si ya hay CAFI
        # detenido en el sensor / pick point.
        if self._part_at_pick_sensor():
            return False, ("CAFI ya presente en sensor/pick point "
                           "(DI1 activo)")
        # spawn zone ocupada
        for c in self.cafis:
            if c.get("location") != "on_conveyor":
                continue
            if abs(c.get("x", 0) - SPAWN_X) < SPAWN_GUARD_RADIUS:
                return False, "spawn zone ocupada por id={}".format(c.get("id"))
        # acumulacion?
        if self._accumulation_detected():
            return False, "acumulacion en conveyor (otra falla activa)"
        return True, "ok"

    def _part_at_pick_sensor(self):
        """DI1 wide window (real SICK sensor: +- 110 mm around pick X)."""
        for c in self.cafis:
            if c.get("location") != "on_conveyor":
                continue
            x = c.get("x", 0)
            y = c.get("y", BELT_Y)
            z = c.get("z", BELT_TOP_Z)
            if (abs(x - CONV_SENSOR_X) <= CONV_SENSOR_HW_X and
                    CONV_SENSOR_Y_RANGE[0] <= y <= CONV_SENSOR_Y_RANGE[1] and
                    abs(z - CONV_SENSOR_Z) <= CONV_SENSOR_HW_Z):
                return True
        return False

    def _part_ready_for_pick(self):
        """V27 tight pick-ready: CAFI center within PICK_TOL_X of PICK_X.
        Only this triggers belt stop and PICK_CONV dispatch."""
        for c in self.cafis:
            if c.get("location") != "on_conveyor":
                continue
            x = c.get("x", 0)
            y = c.get("y", BELT_Y)
            if (abs(x - PICK_X) <= PICK_TOL_X and
                    CONV_SENSOR_Y_RANGE[0] <= y <= CONV_SENSOR_Y_RANGE[1]):
                return True
        return False

    def _accumulation_detected(self):
        on_belt = sorted([c for c in self.cafis
                          if c.get("location") == "on_conveyor"],
                         key=lambda c: c.get("x", 0))
        for i in range(len(on_belt) - 1):
            dx = abs(on_belt[i + 1]["x"] - on_belt[i]["x"])
            if dx < MIN_SEPARATION_M:
                return True
        return False

    # =========================================================
    # TICK
    # =========================================================
    def tick(self):
        with self.lock:
            # Update DI1 latched (real sensor presence, wide window)
            present = self._part_at_pick_sensor()
            self.pub_present.publish(Bool(data=present))

            # V27: pick-ready (CAFI center at PICK_X within +- PICK_TOL_X)
            ready = self._part_ready_for_pick()
            self.pub_ready.publish(Bool(data=ready))

            # Update spawn_allowed latched
            ok, _ = self._spawn_can_proceed()
            self.pub_spawn_allowed.publish(Bool(data=ok))

            # Acumulacion -> fault (one-shot per detection)
            if self._accumulation_detected():
                self.pub_fault.publish(String(
                    data="ERROR: doble pieza / acumulacion en conveyor"))
                self.pub_motor_state.publish(Bool(data=False))

            # V27: belt stops only when the CAFI center is at PICK_X
            # (NOT when DI1 first triggers).  Stopping on DI1 left the
            # CAFI 110 mm short of the pick target and caused the
            # PICK_CONV watchdog in V26.
            motor_on = self.motor_enabled and not ready and (
                self.cell_state == CELL_RUNNING)
            self.pub_motor_state.publish(Bool(data=motor_on))

            # V39: belt steps WEST (object_manager subtracts speed*dt
            # from CAFI.x and clamps at x_min = PICK_X).
            self.pub_belt_step.publish(String(data=json.dumps({
                "speed":      BELT_SPEED if motor_on else 0.0,
                "dt":         1.0 / TICK_HZ,
                "belt_top_z": BELT_TOP_Z,
                "belt_y":     BELT_Y,
                "x_min":      PICK_X,        # stop here
                "x_max":      BELT_X_EAST,   # informational
            })))

    def publish_occupancy(self):
        with self.lock:
            occ = {
                "n_on_belt": sum(1 for c in self.cafis
                                 if c.get("location") == "on_conveyor"),
                "pick_present":   self._part_at_pick_sensor(),
                "motor":          self.motor_enabled,
                "spawn_allowed":  self._spawn_can_proceed()[0],
                "cell":           self.cell_state,
            }
        self.pub_occupancy.publish(String(data=json.dumps(occ)))

    def run(self):
        tick_rate    = rospy.Rate(TICK_HZ)
        publish_each = int(TICK_HZ / PUBLISH_HZ)
        counter = 0
        while not rospy.is_shutdown():
            self.tick()
            counter += 1
            if counter >= publish_each:
                self.publish_occupancy()
                counter = 0
            tick_rate.sleep()


if __name__ == "__main__":
    try:
        ConveyorSimNode().run()
    except rospy.ROSInterruptException:
        pass
