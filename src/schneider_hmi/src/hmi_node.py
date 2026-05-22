#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_hmi.hmi_node  --  V25

Two-button operator HMI + monitoring panel.

OPERATOR BUTTONS:
  - "Colocar CAFI"   -> /operator/spawn_cafi (Empty)
                        DESHABILITADO si /conveyor/spawn_allowed=False
  - "STOP / RESUME"  -> /operator/stop (Bool toggle)

MONITOR (read-only):
  DI lamps (4):
    1. Sensor presencia conveyor
    2. Sensor presencia remachado (load fixture)
    3. Sensor presencia vision
    4. Cobot busy

  DO lamps (8):
    1. Conveyor motor
    2. Disco indexing
    3. Remachado start (rivet active)
    4. Trigger camara (1s pulse)
    5. Gripper open
    6. Gripper close
    7. Solenoid fixture left  (current station)
    8. RESERVADO  (V25: solo solenoide LEFT por fixture; right removido)

  Cycle stage label
  Camera verdict label
  Spawn allowed / Spawn blocked label
  Fault active + reason label

ARCHITECTURE NOTE:
  V19 HMI hacia ademas algunas decisiones de control (publica a topics
  /process/cmd/*). V20 HMI es DUMB - solo publica los 2 commands del
  operador y lee estado. Toda la logica de control esta en los nodos
  por dominio.
"""

import threading

import rospy
from std_msgs.msg import Bool, Empty, String, UInt8MultiArray


try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except Exception:
    HAS_TK = False


COLOR_OFF = "#444444"
COLOR_ON  = "#22dd22"
COLOR_WARN = "#dd2222"


class HMI(object):

    def __init__(self):
        rospy.init_node("schneider_hmi", anonymous=False)
        self.lock = threading.Lock()

        # State mirrors
        self.spawn_allowed   = False
        self.cell_state      = "IDLE"
        self.fault_reason    = ""
        self.cycle_stage     = "IDLE"
        self.camera_verdict  = ""
        self.di_conv         = False
        self.di_rivet        = False
        self.di_vision       = False
        self.cobot_busy      = False
        self.do_motor        = False
        self.do_disc         = False
        self.do_rivet        = False
        self.do_camera_trig  = False
        self.do_grip_open    = False
        self.do_grip_close   = False
        self.do_sol_l        = False
        self.do_sol_r        = False
        self.stopped         = False

        # Publishers
        self.pub_spawn = rospy.Publisher(
            "/operator/spawn_cafi", Empty, queue_size=2)
        self.pub_stop  = rospy.Publisher(
            "/operator/stop",       Bool,  queue_size=2)

        # Subscribers
        rospy.Subscriber("/conveyor/spawn_allowed",  Bool,
                         self._cb_spawn_allowed)
        rospy.Subscriber("/cell/state",              String,
                         self._cb_cell_state)
        rospy.Subscriber("/cell/fault",              String,
                         self._cb_fault)
        rospy.Subscriber("/cell/cycle_stage",        String,
                         self._cb_stage)
        rospy.Subscriber("/camera/result",           String,
                         self._cb_verdict)
        rospy.Subscriber("/conveyor/part_present_pick", Bool,
                         self._cb_di_conv)
        rospy.Subscriber("/fixture/load_present",    Bool,
                         self._cb_di_rivet)
        rospy.Subscriber("/vision/presence",         Bool,
                         self._cb_di_vision)
        rospy.Subscriber("/robot/busy",              Bool,
                         self._cb_busy)
        rospy.Subscriber("/conveyor/motor_state",    Bool,
                         self._cb_motor)
        rospy.Subscriber("/disc/state",              String,
                         self._cb_disc_state)
        rospy.Subscriber("/rivet/active",            Bool,
                         self._cb_rivet_active)
        rospy.Subscriber("/gripper/state",           String,
                         self._cb_grip_state)
        rospy.Subscriber("/fixture/solenoids_state", UInt8MultiArray,
                         self._cb_sol_state)

    def _cb_spawn_allowed(self, m): self.spawn_allowed = bool(m.data)
    def _cb_cell_state(self, m):    self.cell_state = m.data
    def _cb_fault(self, m):         self.fault_reason = m.data
    def _cb_stage(self, m):         self.cycle_stage = m.data
    def _cb_verdict(self, m):       self.camera_verdict = m.data
    def _cb_di_conv(self, m):       self.di_conv = bool(m.data)
    def _cb_di_rivet(self, m):      self.di_rivet = bool(m.data)
    def _cb_di_vision(self, m):     self.di_vision = bool(m.data)
    def _cb_busy(self, m):          self.cobot_busy = bool(m.data)
    def _cb_motor(self, m):         self.do_motor = bool(m.data)
    def _cb_disc_state(self, m):    self.do_disc = (m.data == "INDEXING")
    def _cb_rivet_active(self, m):  self.do_rivet = bool(m.data)

    def _cb_grip_state(self, m):
        self.do_grip_open  = (m.data == "OPEN")
        self.do_grip_close = (m.data == "CLOSED")

    def _cb_sol_state(self, m):
        # [A_left, A_right, B_left, B_right]
        if len(m.data) >= 4:
            self.do_sol_l = bool(m.data[0] or m.data[2])
            self.do_sol_r = bool(m.data[1] or m.data[3])

    # =========================================================
    # Operator actions
    # =========================================================
    def on_spawn(self):
        """V21: el HMI NO censura el primer spawn cuando la celda esta
        en IDLE. Razon: spawn_allowed depende de conveyor_sim, conveyor_sim
        solo lo pone True con cell=RUNNING, y state_manager solo pasa a
        RUNNING al recibir /operator/spawn_cafi -> deadlock de arranque.

        Regla V21:
          cell=IDLE                                 -> publica (arranca el ciclo)
          cell=RUNNING + spawn_allowed=True         -> publica
          cell=RUNNING + spawn_allowed=False        -> bloquea (gating runtime)
          cell in {PAUSED, FAULT}                   -> bloquea
        El backend (conveyor_sim._spawn_can_proceed) tiene la palabra final.
        """
        if self.cell_state in ("PAUSED", "FAULT"):
            rospy.logwarn("[HMI] spawn IGNORED: cell %s", self.cell_state)
            return
        if self.cell_state == "RUNNING" and not self.spawn_allowed:
            rospy.logwarn("[HMI] spawn IGNORED: spawn_allowed=False "
                          "(runtime gating)")
            return
        # cell=IDLE -> arranca; cell=RUNNING + allowed -> spawn normal
        self.pub_spawn.publish(Empty())
        rospy.loginfo("[HMI] operator: Colocar CAFI (cell=%s)",
                      self.cell_state)

    def on_stop_toggle(self):
        with self.lock:
            self.stopped = not self.stopped
        self.pub_stop.publish(Bool(data=self.stopped))
        rospy.loginfo("[HMI] operator: %s",
                      "STOP" if self.stopped else "RESUME")

    # =========================================================
    # GUI
    # =========================================================
    def run(self):
        if not HAS_TK:
            rospy.logwarn("[HMI] tkinter not available - running headless")
            rospy.spin()
            return

        root = tk.Tk()
        root.title("Schneider Cell V25 - HMI")
        root.geometry("700x600")

        # Header
        hdr = ttk.Frame(root, padding=10)
        hdr.pack(fill="x")
        ttk.Label(hdr, text="Schneider Riveting Cell - HMI V25",
                  font=("Helvetica", 16, "bold")).pack()

        # Operator buttons
        btn_frame = ttk.Frame(root, padding=10)
        btn_frame.pack(fill="x")
        self.btn_spawn = tk.Button(
            btn_frame, text="Colocar CAFI",
            command=self.on_spawn, width=20, height=2,
            bg="#3399ff", fg="white",
            font=("Helvetica", 12, "bold"))
        self.btn_spawn.pack(side="left", padx=10)
        self.btn_stop = tk.Button(
            btn_frame, text="STOP / RESUME",
            command=self.on_stop_toggle, width=20, height=2,
            bg="#dd5500", fg="white",
            font=("Helvetica", 12, "bold"))
        self.btn_stop.pack(side="left", padx=10)

        # Cell state
        st_frame = ttk.LabelFrame(root, text="Cell State", padding=10)
        st_frame.pack(fill="x", padx=10, pady=5)
        self.lbl_cell  = tk.Label(st_frame, text="IDLE",
                                  font=("Helvetica", 14, "bold"))
        self.lbl_cell.pack(side="left", padx=10)
        self.lbl_stage = tk.Label(st_frame, text="IDLE",
                                  font=("Helvetica", 14))
        self.lbl_stage.pack(side="left", padx=20)
        self.lbl_verdict = tk.Label(st_frame, text="--",
                                    font=("Helvetica", 14, "bold"),
                                    width=8)
        self.lbl_verdict.pack(side="right", padx=10)
        ttk.Label(st_frame, text="Camera:").pack(side="right")

        # Spawn / fault row
        sf_frame = tk.Frame(root, padx=10, pady=4)
        sf_frame.pack(fill="x")
        self.lbl_spawn = tk.Label(sf_frame, text="SPAWN BLOCKED",
                                  bg=COLOR_WARN, fg="white",
                                  font=("Helvetica", 10, "bold"),
                                  width=20)
        self.lbl_spawn.pack(side="left", padx=5)
        self.lbl_fault = tk.Label(sf_frame, text="", fg="red",
                                  font=("Helvetica", 10, "bold"))
        self.lbl_fault.pack(side="left", padx=10)

        # DI lamps
        di_frame = ttk.LabelFrame(root, text="Digital Inputs (4)",
                                  padding=10)
        di_frame.pack(fill="x", padx=10, pady=5)
        self.di_lamps = {}
        for i, n in enumerate(["DI1 Conveyor", "DI2 Remachado",
                                "DI3 Vision", "DI4 Cobot ready"]):
            lf = tk.Frame(di_frame)
            lf.pack(side="left", padx=8)
            lamp = tk.Label(lf, text="●", font=("Helvetica", 20),
                            fg=COLOR_OFF, width=2)
            lamp.pack()
            ttk.Label(lf, text=n).pack()
            self.di_lamps[n] = lamp

        # DO lamps
        do_frame = ttk.LabelFrame(root, text="Digital Outputs (8)",
                                  padding=10)
        do_frame.pack(fill="x", padx=10, pady=5)
        self.do_lamps = {}
        for i, n in enumerate([
                "DO1 Conv Motor", "DO2 Disco", "DO3 Remachado",
                "DO4 Camara", "DO5 Grip Open", "DO6 Grip Close",
                "DO7 Sol Left", "DO8 (reservado)"]):
            row = 0 if i < 4 else 1
            col = i % 4
            lf = tk.Frame(do_frame)
            lf.grid(row=row, column=col, padx=5, pady=2)
            lamp = tk.Label(lf, text="●", font=("Helvetica", 20),
                            fg=COLOR_OFF, width=2)
            lamp.pack()
            ttk.Label(lf, text=n).pack()
            self.do_lamps[n] = lamp

        def refresh():
            # V21: en IDLE el boton DEBE estar habilitado para arrancar
            # el ciclo (rompe el deadlock de arranque). Solo se deshabilita
            # cuando cell=RUNNING + spawn_allowed=False (gating de runtime)
            # o cuando cell in {PAUSED, FAULT}.
            enable_first = (self.cell_state == "IDLE")
            enable_run   = (self.cell_state == "RUNNING" and self.spawn_allowed)
            if enable_first or enable_run:
                self.btn_spawn.config(state="normal", bg="#3399ff")
                label = "SPAWN ALLOWED" if enable_run else "ARRANCAR CICLO"
                self.lbl_spawn.config(text=label, bg=COLOR_ON)
            else:
                self.btn_spawn.config(state="disabled", bg="#888888")
                self.lbl_spawn.config(text="SPAWN BLOCKED",
                                      bg=COLOR_WARN)
            self.btn_stop.config(
                text="RESUME" if self.stopped else "STOP",
                bg="#22aa55" if self.stopped else "#dd5500")
            # cell / stage / verdict
            self.lbl_cell.config(text=self.cell_state,
                                 fg=("red" if self.cell_state == "FAULT"
                                     else "black"))
            self.lbl_stage.config(text=self.cycle_stage)
            v = self.camera_verdict or "--"
            v_color = ("#22aa22" if v == "PASS"
                       else "#dd2222" if v == "FAIL"
                       else "#444444")
            self.lbl_verdict.config(text=v, fg=v_color)
            # fault
            self.lbl_fault.config(text=self.fault_reason)
            # DI lamps
            self.di_lamps["DI1 Conveyor"].config(
                fg=COLOR_ON if self.di_conv else COLOR_OFF)
            self.di_lamps["DI2 Remachado"].config(
                fg=COLOR_ON if self.di_rivet else COLOR_OFF)
            self.di_lamps["DI3 Vision"].config(
                fg=COLOR_ON if self.di_vision else COLOR_OFF)
            self.di_lamps["DI4 Cobot ready"].config(
                fg=COLOR_OFF if self.cobot_busy else COLOR_ON)
            # DO lamps
            self.do_lamps["DO1 Conv Motor"].config(
                fg=COLOR_ON if self.do_motor else COLOR_OFF)
            self.do_lamps["DO2 Disco"].config(
                fg=COLOR_ON if self.do_disc else COLOR_OFF)
            self.do_lamps["DO3 Remachado"].config(
                fg=COLOR_ON if self.do_rivet else COLOR_OFF)
            self.do_lamps["DO4 Camara"].config(
                fg=COLOR_ON if (self.camera_verdict == "" and self.cycle_stage == "INSPECT")
                else COLOR_OFF)
            self.do_lamps["DO5 Grip Open"].config(
                fg=COLOR_ON if self.do_grip_open else COLOR_OFF)
            self.do_lamps["DO6 Grip Close"].config(
                fg=COLOR_ON if self.do_grip_close else COLOR_OFF)
            self.do_lamps["DO7 Sol Left"].config(
                fg=COLOR_ON if self.do_sol_l else COLOR_OFF)
            # V25: DO8 reservado (sol_right fue removido del URDF).
            self.do_lamps["DO8 (reservado)"].config(fg=COLOR_OFF)
            root.after(150, refresh)

        refresh()
        root.protocol("WM_DELETE_WINDOW", root.destroy)
        rospy.on_shutdown(root.destroy)
        try:
            root.mainloop()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        HMI().run()
    except rospy.ROSInterruptException:
        pass
