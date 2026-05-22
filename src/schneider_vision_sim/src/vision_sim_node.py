#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""schneider_vision_sim.vision_sim_node  --  V40

- Maneja el sensor de presencia en la zona de vision (publica /vision/presence)
- Maneja el trigger de la camara (sub /camera/trigger)
- Despues de INSPECT_DURATION_S simula la respuesta TCP/IP de la camara
  (PASS 70%, FAIL 30%) y publica /camera/result

REGLA CRITICA (per user brief seccion 14):
  El trigger de la camara solo dispara si /vision/presence=True. Si llega
  /camera/trigger sin presencia, se rechaza el trigger y se loguea
  WARN. NO se publica resultado.

V40 fix:
  La causa raiz del watchdog INSPECT en V39 era que estas constantes
  seguian apuntando al layout V20 (VISION=(2.050, 0.990, 1.250)) - el
  CAFI llegaba a (0.824, 0.804, 1.042) y caia 1.32 m fuera del window.
  Vision_sim publicaba presence=False y el trigger se rechazaba.
  V40 sincroniza las constantes con la fixture_vision real del URDF
  (DXF #46) y deja un window mas amplio en Z para tolerar la mecanica
  de settling del object_manager.
"""

import json
import random
import threading

import rospy
from std_msgs.msg import Bool, Empty, String


# V40: vision fixture per DXF #46 (schneider_cell.urdf.xacro V38/V39
# instances fixture_2 at world (0.824, 0.804)).  CAFI seat sits on top
# of the fixture; object_manager settles the CAFI center near z=1.042-
# 1.058 depending on TF lookup vs fallback path.  Z window +-90 mm is
# generous enough to cover both paths without false positives outside
# the fixture.
VISION_X       = 0.824
VISION_Y       = 0.804
VISION_Z       = 1.050
VISION_HX      = 0.120
VISION_HY      = 0.120
VISION_HZ      = 0.090

INSPECT_DURATION_S = 1.6
PASS_PROB          = 0.70
TICK_HZ            = 20.0


class VisionSim(object):

    def __init__(self):
        rospy.init_node("schneider_vision_sim", anonymous=False)

        self.cafis = []
        self.lock = threading.Lock()

        self.inspect_active = False
        self.inspect_t0 = 0.0
        self.inspect_cafi_id = None

        self.pub_presence = rospy.Publisher("/vision/presence",
                                            Bool, queue_size=1, latch=True)
        self.pub_result   = rospy.Publisher("/camera/result",
                                            String, queue_size=1, latch=True)
        self.pub_mark_verdict = rospy.Publisher(
            "/objects/mark_verdict", String, queue_size=2)

        rospy.Subscriber("/objects/cafi_states", String,
                         self._cb_cafi_states)
        rospy.Subscriber("/camera/trigger", Empty, self._cb_trigger)

        self.pub_presence.publish(Bool(data=False))
        self.pub_result.publish(String(data=""))
        rospy.loginfo("[VIS] V20 vision_sim init (INSPECT %.2fs, PASS %.0f%%)",
                      INSPECT_DURATION_S, PASS_PROB * 100)

    def _cb_cafi_states(self, msg):
        try:
            self.cafis = json.loads(msg.data) or []
        except Exception:
            self.cafis = []

    def _cb_trigger(self, _m):
        with self.lock:
            cid = self._cafi_at_vision()
            if cid is None:
                rospy.logwarn("[VIS] trigger DENIED: no CAFI presence in vision zone")
                return
            if self.inspect_active:
                rospy.logwarn("[VIS] trigger DENIED: inspection already active")
                return
            self.inspect_active = True
            self.inspect_t0 = rospy.Time.now().to_sec()
            self.inspect_cafi_id = cid
            self.pub_result.publish(String(data=""))
            rospy.loginfo("[VIS] trigger ACCEPTED cafi_id=%s", cid)

    def _cafi_at_vision(self):
        for c in self.cafis:
            if c.get("location") != "at_vision":
                continue
            x = c.get("x", 0)
            y = c.get("y", 0)
            z = c.get("z", 0)
            if (abs(x - VISION_X) <= VISION_HX and
                    abs(y - VISION_Y) <= VISION_HY and
                    abs(z - VISION_Z) <= VISION_HZ):
                return c.get("id")
        return None

    def tick(self):
        with self.lock:
            present = self._cafi_at_vision() is not None
            self.pub_presence.publish(Bool(data=present))

            if self.inspect_active:
                elapsed = rospy.Time.now().to_sec() - self.inspect_t0
                if elapsed >= INSPECT_DURATION_S:
                    verdict = "PASS" if random.random() < PASS_PROB else "FAIL"
                    self.pub_result.publish(String(data=verdict))
                    if self.inspect_cafi_id is not None:
                        self.pub_mark_verdict.publish(String(
                            data=json.dumps({
                                "cafi_id": self.inspect_cafi_id,
                                "verdict": verdict})))
                    rospy.loginfo("[VIS] verdict=%s cafi_id=%s",
                                  verdict, self.inspect_cafi_id)
                    self.inspect_active = False
                    self.inspect_cafi_id = None

    def run(self):
        rate = rospy.Rate(TICK_HZ)
        while not rospy.is_shutdown():
            self.tick()
            rate.sleep()


if __name__ == "__main__":
    try:
        VisionSim().run()
    except rospy.ROSInterruptException:
        pass
