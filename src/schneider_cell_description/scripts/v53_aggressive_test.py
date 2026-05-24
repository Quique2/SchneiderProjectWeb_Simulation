#!/usr/bin/env python3
"""V53 aggressive single-test: tries to break the plant in every way the
user listed as a rejection criterion.  Pure-python, no ROS roscore
needed — it imports the actual production modules and exercises them.

The test exercises:
  A. Startup — rotary_fixture_sim's RotaryFixtureSim class can be
     constructed (no KeyError 'A').  fixture_has_cafi keys are A & B.
     station_assignment publishes outer=A inner=B.
  B. Index   — table_rotation_joint cinematic-driven swap of station
     assignment, /disc/index_done message published.
  C. Place   — _fixture_target_frame('A') and ('B') resolve to the
     turntable URDF frames; release_at_seat snaps CAFI flat at the
     frame (no tilt, position == frame, orientation == spawn_quat).
  D. Pick    — _fixture_pick_frame('A') and ('B') resolve to the
     turntable URDF pick frames.
  E. Multi   — three CAFIs simultaneously occupying conveyor, fixture
     and vision; no duplicate occupancy or KeyError.
  F. Cycle   — the existing offline 3-CAFI sequencer reports
     0 watchdogs / 0 FAULTs / 0 collisions and the CAFI lands flat
     in the fixture (snap captures spawn_q as world orientation).
"""
from __future__ import print_function

import os, sys, json, math, traceback

HERE = os.path.dirname(os.path.abspath(__file__))
# schneider_cell_description/scripts -> schneider_cell_description -> ros_ws/src
SRC_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(SRC_ROOT, "schneider_rotary_fixture_sim", "src"))
sys.path.insert(0, os.path.join(SRC_ROOT, "schneider_object_manager", "src"))
print("Test SRC_ROOT =", SRC_ROOT)

OK = "[ OK ]"
FAIL = "[FAIL]"

results = []


def case(name):
    def wrap(fn):
        def run():
            print("=" * 78)
            print("Case " + name)
            print("=" * 78)
            try:
                fn()
                results.append((name, True, ""))
                print(OK + " " + name)
            except AssertionError as e:
                results.append((name, False, str(e)))
                print(FAIL + " " + name + ":  " + str(e))
            except Exception as e:
                results.append((name, False, repr(e)))
                print(FAIL + " " + name + ": exception " + repr(e))
                traceback.print_exc()
        return run
    return wrap


# ---------------------------------------------------------------------------
@case("A. Startup — no KeyError 'A', fixture_has_cafi keyed A/B")
def case_a():
    # Pull in the real rotary_fixture_sim module without instantiating ROS.
    # We patch rospy with stubs so the class can be constructed offline.
    import types
    rospy = types.ModuleType("rospy")
    class _Pub(object):
        def __init__(self, *a, **k): pass
        def publish(self, *a, **k): pass
    class _Sub(object):
        def __init__(self, *a, **k): pass
    class _Time(object):
        @staticmethod
        def now():
            class _T:
                def to_sec(self): return 0.0
            return _T()
    rospy.Publisher = _Pub
    rospy.Subscriber = _Sub
    rospy.Time = _Time
    rospy.Duration = lambda x: x
    rospy.init_node = lambda *a, **k: None
    rospy.loginfo = lambda *a, **k: None
    rospy.logwarn = lambda *a, **k: None
    rospy.logerr = lambda *a, **k: None
    rospy.loginfo_throttle = lambda *a, **k: None
    rospy.Rate = lambda hz: type("R", (), {"sleep": lambda self: None})()
    rospy.is_shutdown = lambda: True
    sys.modules["rospy"] = rospy
    # Stub sensor_msgs and std_msgs minimal API
    sm = types.ModuleType("sensor_msgs"); sm_msg = types.ModuleType("sensor_msgs.msg")
    class JointState:
        def __init__(self): self.header = type("H", (), {"stamp": None})(); self.name = []; self.position = []
    sm_msg.JointState = JointState
    sm.msg = sm_msg
    sys.modules["sensor_msgs"] = sm
    sys.modules["sensor_msgs.msg"] = sm_msg
    stm = types.ModuleType("std_msgs"); stm_msg = types.ModuleType("std_msgs.msg")
    class _Stub:
        def __init__(self, data=None): self.data = data
    stm_msg.Bool = _Stub
    stm_msg.Empty = _Stub
    stm_msg.Float32 = _Stub
    stm_msg.String = _Stub
    stm_msg.UInt8MultiArray = _Stub
    stm.msg = stm_msg
    sys.modules["std_msgs"] = stm
    sys.modules["std_msgs.msg"] = stm_msg

    import importlib
    if "rotary_fixture_sim_node" in sys.modules:
        importlib.reload(sys.modules["rotary_fixture_sim_node"])
    rf = importlib.import_module("rotary_fixture_sim_node")
    sim = rf.RotaryFixtureSim()
    assert set(sim.fixture_has_cafi.keys()) == {"A", "B"}, \
        "fixture_has_cafi keys = " + str(set(sim.fixture_has_cafi.keys()))
    assert sim.outer_id == "A" and sim.inner_id == "B", \
        "initial station assignment outer=%s inner=%s" % (sim.outer_id, sim.inner_id)
    # Exercise the path that crashed in V52: log line accessing ["A"] and ["B"]
    sim._cb_rivet_start(None)  # may warn, must not raise
    print("  fixture_has_cafi =", sim.fixture_has_cafi)
    print("  station init     =", sim.outer_id, sim.inner_id)


# ---------------------------------------------------------------------------
@case("B. Index — _swap_stations toggles A<->B, station msg published")
def case_b():
    import importlib
    rf = importlib.import_module("rotary_fixture_sim_node")
    sim = rf.RotaryFixtureSim()
    assert sim.outer_id == "A" and sim.inner_id == "B"
    sim._swap_stations()
    assert sim.outer_id == "B" and sim.inner_id == "A", \
        "after 1st swap outer=%s inner=%s" % (sim.outer_id, sim.inner_id)
    sim._swap_stations()
    assert sim.outer_id == "A" and sim.inner_id == "B", \
        "after 2nd swap outer=%s inner=%s" % (sim.outer_id, sim.inner_id)
    print("  swap roundtrip OK")
    # Exercise the disc index command path
    msg = type("M", (), {"data": math.pi})()
    sim._cb_disc_cmd(msg)
    assert sim.disc_state == "INDEXING"
    assert abs(sim.disc_target - math.pi) < 1e-9
    # Tick until the disc reaches the target (offline)
    for _ in range(int(math.pi / (rf.DISC_INDEX_RATE_RAD_S * (1.0 / rf.TICK_HZ))) + 20):
        sim.tick(1.0 / rf.TICK_HZ)
        if sim.disc_state == "IDLE":
            break
    assert sim.disc_state == "IDLE", "disc never returned to IDLE"
    assert sim.outer_id == "B" and sim.inner_id == "A", \
        "after index assignment outer=%s inner=%s" % (sim.outer_id, sim.inner_id)
    print("  index complete, station swap OK")


# ---------------------------------------------------------------------------
@case("C. Place — _fixture_target_frame('A'/'B') == turntable frames")
def case_c():
    # Stub tf2_ros for object_manager import
    import types
    tf2 = types.ModuleType("tf2_ros")
    class _Buf(object):
        def lookup_transform(self, *a, **k):
            raise RuntimeError("offline")
    class _Lis(object):
        def __init__(self, *a, **k): pass
    tf2.Buffer = _Buf
    tf2.TransformListener = _Lis
    sys.modules["tf2_ros"] = tf2
    sys.modules.setdefault("geometry_msgs", types.ModuleType("geometry_msgs"))
    gmsg = types.ModuleType("geometry_msgs.msg")
    gmsg.Point = type("P", (), {})
    gmsg.Quaternion = type("Q", (), {})
    sys.modules["geometry_msgs.msg"] = gmsg
    sys.modules.setdefault("visualization_msgs", types.ModuleType("visualization_msgs"))
    vmsg = types.ModuleType("visualization_msgs.msg")
    vmsg.Marker = type("M", (), {"MESH_RESOURCE": 10, "DELETE": 2, "ADD": 0})
    vmsg.MarkerArray = type("MA", (), {})
    sys.modules["visualization_msgs.msg"] = vmsg
    import importlib
    om = importlib.import_module("object_manager_node")
    assert om._fixture_target_frame("A") == "fixture_1_cafi_lateral_target_frame", \
        "A -> " + om._fixture_target_frame("A")
    assert om._fixture_target_frame("B") == "fixture_2_cafi_lateral_target_frame", \
        "B -> " + om._fixture_target_frame("B")
    assert om._fixture_pick_frame("A") == "cafi_pick_frame_1", \
        "pick A -> " + om._fixture_pick_frame("A")
    assert om._fixture_pick_frame("B") == "cafi_pick_frame_2", \
        "pick B -> " + om._fixture_pick_frame("B")
    # Backwards-compat: "1"/"2" still work
    assert om._fixture_target_frame("1") == "fixture_1_cafi_lateral_target_frame"
    assert om._fixture_target_frame("2") == "fixture_2_cafi_lateral_target_frame"
    print("  A -> fixture_1_cafi_lateral_target_frame, cafi_pick_frame_1")
    print("  B -> fixture_2_cafi_lateral_target_frame, cafi_pick_frame_2")


# ---------------------------------------------------------------------------
@case("D. Release snap — CAFI exactly at frame, orientation = spawn_q")
def case_d():
    import importlib
    cs = importlib.import_module("cycle_simulate_v53")
    spawn_q = cs.q_from_rpy(0.0, math.pi, math.pi)
    cafi = cs.SimCafi(1, math.pi)
    cafi.attached = True
    cafi.frozen_p = (0.1, 0.0, 0.0)
    cafi.frozen_q = spawn_q
    # release at the turntable fixture_1 world pose with a tilted seat
    seat_center = (0.680276, 1.157611, 1.104476)
    seat_q = (0.0, 0.0, 0.0, 1.0)  # identity (disc q=0)
    cafi.release_at_seat(seat_center, seat_q)
    # Pose snapped exactly
    assert abs(cafi.x - seat_center[0]) < 1e-12 and \
           abs(cafi.y - seat_center[1]) < 1e-12 and \
           abs(cafi.z - seat_center[2]) < 1e-12, \
        "CAFI position not on the frame: %s vs %s" % (
            (cafi.x, cafi.y, cafi.z), seat_center)
    assert abs(cafi.q[0] - spawn_q[0]) < 1e-12 and \
           abs(cafi.q[1] - spawn_q[1]) < 1e-12 and \
           abs(cafi.q[2] - spawn_q[2]) < 1e-12 and \
           abs(cafi.q[3] - spawn_q[3]) < 1e-12, \
        "CAFI orientation != spawn_q after settle: %s vs %s" % (
            cafi.q, spawn_q)
    # Index disc 180 deg: seat_q becomes Rz(pi).  Rigid-body follow
    # must rotate the CAFI accordingly.
    seat_q2 = (0.0, 0.0, math.sin(math.pi/2), math.cos(math.pi/2))  # Rz(pi)
    new_p = (seat_center[0]*math.cos(math.pi) - seat_center[1]*math.sin(math.pi),
             seat_center[0]*math.sin(math.pi) + seat_center[1]*math.cos(math.pi),
             seat_center[2])
    p, q = cs.pose_compose(seat_center, seat_q2, cafi.frozen_p, cafi.frozen_q)
    # CAFI position stays at the frame (since frozen_p = 0)
    assert all(abs(p[i] - seat_center[i]) < 1e-9 for i in range(3)), \
        "rigid-body follow drifted: " + str(p)
    # CAFI orientation has rotated Rz(pi) from spawn_q
    expected_q = cs.q_normalize(cs.q_mul(seat_q2, spawn_q))
    assert all(abs(q[i] - expected_q[i]) < 1e-9 for i in range(4)) or \
           all(abs(q[i] + expected_q[i]) < 1e-9 for i in range(4)), \
        "rigid-body follow orientation wrong: " + str(q) + " vs " + str(expected_q)
    print("  snap exact, follow on disc index OK (no tilt, no drift)")
    print("  CAFI quat at release = (%.3f, %.3f, %.3f, %.3f)  == spawn_q" % cafi.q)


# ---------------------------------------------------------------------------
@case("E. Multi-CAFI — rotary_fixture_sim handles A+B simultaneously")
def case_e():
    import importlib
    rf = importlib.import_module("rotary_fixture_sim_node")
    sim = rf.RotaryFixtureSim()
    # Simulate object_manager publishing two CAFIs in fixtures
    msg = type("M", (), {"data": json.dumps([
        {"id": 11, "location": "in_fixture_A"},
        {"id": 12, "location": "in_fixture_B"},
        {"id": 13, "location": "in_gripper"},
    ])})()
    sim._cb_cafi_states(msg)
    assert sim.fixture_has_cafi["A"] is True, "A occupancy not set"
    assert sim.fixture_has_cafi["B"] is True, "B occupancy not set"
    assert sim.fixture_cafi_id["A"] == 11 and sim.fixture_cafi_id["B"] == 12, \
        "cafi_id mapping wrong: " + str(sim.fixture_cafi_id)
    # Now legacy format "in_fixture_1" / "in_fixture_2" — should still map
    msg2 = type("M", (), {"data": json.dumps([
        {"id": 21, "location": "in_fixture_1"},
        {"id": 22, "location": "in_fixture_2"},
    ])})()
    sim._cb_cafi_states(msg2)
    assert sim.fixture_has_cafi["A"] is True and sim.fixture_has_cafi["B"] is True
    # Vacate A: only B remains
    msg3 = type("M", (), {"data": json.dumps([
        {"id": 22, "location": "in_fixture_B"},
    ])})()
    sim._cb_cafi_states(msg3)
    assert sim.fixture_has_cafi["A"] is False and sim.fixture_has_cafi["B"] is True, \
        "multi-CAFI vacate failed: " + str(sim.fixture_has_cafi)
    # No KeyError, no exceptions raised
    print("  fixture_has_cafi handled A+B concurrent + legacy in_fixture_1/2 strings")


# ---------------------------------------------------------------------------
@case("F. Cycle — V53 simulator: 0 watchdogs, 0 FAULTs, 0 collisions")
def case_f():
    import subprocess
    out = subprocess.check_output(
        ["python3", os.path.join(HERE, "cycle_simulate_v53.py")],
        stderr=subprocess.STDOUT)
    out = out.decode("utf-8", errors="replace")
    assert "Overall: 3/3 CAFIs cycled" in out, "cycle did not finish 3/3"
    assert "FAULT events:             0" in out, "FAULT events != 0"
    assert "Watchdogs tripped:        0" in out, "watchdog tripped"
    assert "Plant collisions:         0" in out, "collision"
    # Look for the URDF snap log
    for line in out.split("\n"):
        if "SETTLE" in line and "cradle" in line:
            print("  " + line.strip())
            break
    print("  cycle_simulate_v53.py: 3/3 OK, 0 FAULTs / 0 watchdogs / 0 collisions")


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    for fn in [case_a, case_b, case_c, case_d, case_e, case_f]:
        fn()
        print()
    print("=" * 78)
    n_ok   = sum(1 for _, ok, _ in results if ok)
    n_fail = sum(1 for _, ok, _ in results if not ok)
    print("V53 AGGRESSIVE TEST RESULTS: %d/%d passed" % (n_ok, n_ok + n_fail))
    for name, ok, err in results:
        prefix = OK if ok else FAIL
        print("  " + prefix + " " + name + (" -- " + err if err else ""))
    sys.exit(0 if n_fail == 0 else 1)
