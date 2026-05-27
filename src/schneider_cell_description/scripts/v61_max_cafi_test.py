"""V61 max-CAFI throughput stress test.

Two scenarios, both must pass with zero faults, zero watchdogs, and
zero unsafe cabin transitions.  The point of V61 is to push the cell
to a regime far beyond the V55 baseline (52 CAFIs) and prove the FSM,
delivery dispatcher and cabin interlock survive it.

  TEST A — sustained max-CAFI burst (200 parts, alternating verdicts)
    * 200 CAFIs back-to-back, 1 every 5 s sim-time.
    * Alternates PASS / FAIL so the bins both fill.
    * Asserts: no fault, no watchdog ever, cabin always RAISED on
      delivery transitions, FSM ends back in IDLE.

  TEST B — extreme sustained run (400 parts, mixed cadence)
    * 400 CAFIs.  First 100 at 4 s/CAFI (over-spawn pressure), then
      300 at 5 s/CAFI (steady state).
    * Verdicts: every 3rd FAIL, rest PASS, so REJECT bin is exercised
      ~133 times and ACCEPT bin ~267 times.
    * Asserts: same invariants as TEST A; also checks the cell never
      drifts into "FAULT" or "PAUSED" mid-stream.

The V61 collision and geometry layers do NOT change — V60 poses and
V60 trajectories are reused as-is.  V61 only exercises stress on the
FSM / dispatcher / cabin interlock.
"""
from __future__ import print_function
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import rospy_stub
rospy_stub.install()
import rospy  # noqa: E402  resolves to the stub
from std_msgs.msg import Bool, Empty, String  # noqa: E402


# Load the real state_manager source (same trick as V55).
SM_PATH = os.path.abspath(os.path.join(
    HERE, "..", "..", "schneider_state_manager", "src",
    "state_manager_node.py"))
sm_globals = {"__name__": "__sm__", "__file__": SM_PATH}
with open(SM_PATH) as f:
    exec(compile(f.read(), SM_PATH, "exec"), sm_globals)
StateManager = sm_globals["StateManager"]


def _drive_full_cycle(sm, cafi_id, with_verdict="PASS"):
    """Drive one CAFI through every stage of the FSM.  Mirrors
    `_drive_full_cycle` in v55_aggressive_test.py but lifted here so
    V61 is self-contained and survives small V55 refactors."""
    cafi_state = {
        "id": cafi_id, "x": 1.235, "y": 1.365, "z": 1.083,
        "location": "on_conveyor", "at_sensor": True, "riveted": False,
        "verdict": None, "fixture_id": None,
    }
    sm.cafi_states = [cafi_state]
    sm.part_ready_for_pick = True
    sm.part_present_pick = True

    sm.tick()  # IDLE -> PICK_CONV
    assert sm.cycle_stage == "PICK_CONV", \
        "[id=%d] expected PICK_CONV got %s" % (cafi_id, sm.cycle_stage)

    def _advance(**fields):
        for k, v in fields.items():
            setattr(sm, k, v)
        sm.robot_motion_done = True
        sm.tick()

    cafi_state["location"] = "in_gripper"
    _advance(gripper_grasp=True)
    assert sm.cycle_stage == "PLACE_LOAD"
    assert sm.cabin_state == "RAISED", \
        "[id=%d] cabin must be RAISED before PLACE_LOAD" % cafi_id

    cafi_state["location"] = "in_fixture_" + sm.outer_id
    cafi_state["fixture_id"] = sm.outer_id
    sm.gripper_grasp = False
    sm.fixture_load_present = True
    sm._cb_robot_task(String(data="home"))
    _advance()
    assert sm.cycle_stage == "SEAT"
    assert sm.cabin_state == "LOWERED", \
        "[id=%d] cabin must lower after cobot returns to home" % cafi_id

    sm.fixture_cafi_seated = True
    sm.tick()
    assert sm.cycle_stage == "IDLE"

    sm.tick()
    assert sm.cycle_stage == "INDEX_DISC"

    sm.disc_index_done_flag = True
    sm.tick()
    assert sm.cycle_stage == "IDLE"

    sm.outer_id, sm.inner_id = sm.inner_id, sm.outer_id
    cafi_state["fixture_id"] = sm.inner_id
    cafi_state["location"] = "in_fixture_" + sm.inner_id

    cafi_state["riveted"] = True
    sm.tick()
    assert sm.cycle_stage == "INDEX_DISC_BACK"
    sm.disc_index_done_flag = True
    sm.tick()
    assert sm.cycle_stage == "IDLE"

    sm.outer_id, sm.inner_id = sm.inner_id, sm.outer_id
    cafi_state["fixture_id"] = sm.outer_id
    cafi_state["location"] = "in_fixture_" + sm.outer_id
    sm.tick()
    assert sm.cycle_stage == "PICK_RIVETED"

    cafi_state["location"] = "in_gripper"
    _advance(gripper_grasp=True)
    assert sm.cycle_stage == "PLACE_VISION"
    assert sm.cabin_state == "RAISED"

    cafi_state["location"] = "at_vision"
    sm.gripper_grasp = False
    sm.vision_presence = True
    sm._cb_robot_task(String(data="home"))
    _advance()
    assert sm.cycle_stage == "INSPECT"

    sm.camera_result = with_verdict
    sm.tick()
    assert sm.cycle_stage == "IDLE"

    cafi_state["verdict"] = with_verdict
    sm.tick()
    assert sm.cycle_stage == "PICK_VISION"

    cafi_state["location"] = "in_gripper"
    _advance(gripper_grasp=True)
    assert sm.cycle_stage == "PLACE_BIN"
    assert sm.cabin_state == "RAISED"

    sm.gripper_grasp = False
    cafi_state["location"] = "in_bin"
    sm._cb_robot_task(String(data="home"))
    _advance()
    assert sm.cycle_stage == "IDLE"

    sm.cafi_states = []
    sm.fixture_load_present = False
    sm.vision_presence = False


def _assert_clean_log(label):
    bad = [l for l in rospy_stub.LOG_ERR if "watchdog" in l[2].lower()]
    assert not bad, "%s watchdog lines: %s" % (label, bad)
    fault = [l for l in rospy_stub.LOG_ERR if "fault" in l[2].lower()]
    assert not fault, "%s fault lines: %s" % (label, fault)


def test_a_sustained_burst(n_cafi=200, dt=5.0):
    """TEST A — 200 CAFI sustained burst."""
    rospy_stub.reset()
    sm = StateManager()
    sm._cb_op_start(Empty())
    assert sm.cell_state == "RUNNING"

    for cid in range(1, n_cafi + 1):
        rospy_stub.CLOCK.advance(dt)
        _drive_full_cycle(sm, cid,
                          with_verdict=("PASS" if cid % 2 else "FAIL"))
        assert sm.cell_state == "RUNNING", \
            "cell drifted out of RUNNING at cafi %d -> %s" % (cid, sm.cell_state)
        assert sm.fault_reason == "", \
            "fault at cafi %d: %r" % (cid, sm.fault_reason)

    _assert_clean_log("TEST A")
    assert sm.fault_reason == ""

    cabin_evts = sum(1 for _, _, m in rospy_stub.LOG
                     if "cabin RAISE" in m or "cabin LOWER" in m)
    print("TEST A OK: %d CAFIs at %.1fs cadence, "
          "cell still RUNNING, cabin transitions=%d, "
          "no faults, no watchdogs." % (n_cafi, dt, cabin_evts))


def test_b_extreme_mixed(n_burst=100, n_steady=300):
    """TEST B — 400 CAFI extreme run, mixed cadence."""
    rospy_stub.reset()
    sm = StateManager()
    sm._cb_op_start(Empty())
    assert sm.cell_state == "RUNNING"

    for cid in range(1, n_burst + 1):
        rospy_stub.CLOCK.advance(4.0)
        verdict = "FAIL" if (cid % 3 == 0) else "PASS"
        _drive_full_cycle(sm, cid, with_verdict=verdict)
        assert sm.cell_state == "RUNNING"
        assert sm.fault_reason == ""

    for cid in range(n_burst + 1, n_burst + n_steady + 1):
        rospy_stub.CLOCK.advance(5.0)
        verdict = "FAIL" if (cid % 3 == 0) else "PASS"
        _drive_full_cycle(sm, cid, with_verdict=verdict)
        assert sm.cell_state == "RUNNING"
        assert sm.fault_reason == ""

    _assert_clean_log("TEST B")

    total = n_burst + n_steady
    fails = sum(1 for cid in range(1, total + 1) if cid % 3 == 0)
    passes = total - fails

    print("TEST B OK: %d CAFIs (%d burst @4s + %d steady @5s), "
          "PASS=%d, FAIL=%d, cell still RUNNING, no faults, no watchdogs."
          % (total, n_burst, n_steady, passes, fails))


if __name__ == "__main__":
    test_a_sustained_burst()
    test_b_extreme_mixed()
    print()
    print("ALL V61 MAX-CAFI TESTS PASSED.")
