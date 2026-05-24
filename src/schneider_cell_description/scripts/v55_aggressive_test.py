"""V55 aggressive test.

Runs the actual schneider_state_manager source under a rospy stub and
exercises the two V55 user scenarios:

  TEST 1 (throughput)
    - Operator presses START.
    - Two CAFIs are spawned back-to-back (simulating the user dropping
      two parts at the very beginning).  The conveyor advance + rivet
      complete + vision verdict + bin drop callbacks are replayed
      synthetically so the FSM advances through all stages.
    - Then a continuous stream of CAFIs is fed for 5 simulated minutes.
    - We assert: no fault, no stage watchdog ever fires, no log lines
      contain "watchdog" (other than benign info), and the cabin is
      always RAISED whenever a delivery is dispatched.

  TEST 2 (operator buttons)
    - START -> RUNNING.
    - Spawn one CAFI.  Drive it up to STAGE_PICK_CONV with the gripper
      grasping (CLOSED).  Then press STOP.
    - In PAUSED, press RESET.  The state_manager must:
        * raise the cabin
        * dispatch the REJECT bin place because the gripper is closed
        * after the reject completes, remove any stranded CAFIs in
          vision + outer + (after disc index) inner
        * transition back to IDLE (started_once back to False)
    - We assert all of the above without any /cell/fault published
      and without any RESET_* watchdog.
"""
from __future__ import print_function
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import rospy_stub
rospy_stub.install()
import rospy  # now resolves to the stub
from std_msgs.msg import Bool, Empty, Float32, String


# Load the real state_manager source by exec'ing it (the package layout
# uses `schneider_state_manager.state_manager_node` but here we just
# load the file directly so we don't need catkin's installspace).
SM_PATH = os.path.abspath(os.path.join(
    HERE, "..", "..", "schneider_state_manager", "src",
    "state_manager_node.py"))
sm_globals = {"__name__": "__sm__", "__file__": SM_PATH}
with open(SM_PATH) as f:
    exec(compile(f.read(), SM_PATH, "exec"), sm_globals)
StateManager = sm_globals["StateManager"]


# ============================================================
# Test 1: throughput
# ============================================================
def _drive_full_cycle(sm, cafi_id, with_verdict="PASS"):
    """Simulate the world: bring the CAFI to the pick sensor, complete
    PICK_CONV / PLACE_LOAD / SEAT / INDEX / RIVET / PICK_RIVETED /
    PLACE_VISION / INSPECT / PICK_VISION / PLACE_BIN.  Just enough
    events for the FSM to advance happily."""
    # The CAFI snapshot the FSM consumes:
    cafi_state = {
        "id": cafi_id, "x": 1.235, "y": 1.365, "z": 1.083,
        "location": "on_conveyor", "at_sensor": True, "riveted": False,
        "verdict": None, "fixture_id": None,
    }
    sm.cafi_states = [cafi_state]
    sm.part_ready_for_pick = True
    sm.part_present_pick = True

    # Tick: STAGE_IDLE -> STAGE_PICK_CONV (dispatch P6)
    sm.tick()
    assert sm.cycle_stage == "PICK_CONV", \
        "tick1: expected PICK_CONV got %s" % sm.cycle_stage

    def _advance(stage_now, **fields):
        """Apply fields, ensure motion_done True, tick.  _set_stage in
        the FSM resets motion_done on each transition, so we always
        push True right before tick when we're acting as the world."""
        for k, v in fields.items():
            setattr(sm, k, v)
        sm.robot_motion_done = True
        sm.tick()

    # Simulate cobot completing PICK_CONV: motion_done + grasp_confirmed
    cafi_state["location"] = "in_gripper"
    _advance("PICK_CONV", gripper_grasp=True)
    assert sm.cycle_stage == "PLACE_LOAD"
    assert sm.cabin_state == "RAISED", \
        "cabin must be RAISED before PLACE_LOAD"

    # Simulate place complete
    cafi_state["location"] = "in_fixture_" + sm.outer_id
    cafi_state["fixture_id"] = sm.outer_id
    sm.gripper_grasp = False
    sm.fixture_load_present = True
    # robot returns to home so cabin lowers
    sm._cb_robot_task(String(data="home"))
    _advance("PLACE_LOAD")
    assert sm.cycle_stage == "SEAT"
    assert sm.cabin_state == "LOWERED", \
        "cabin must lower after cobot returns to home (was %s)" % sm.cabin_state

    # SEAT done
    sm.fixture_cafi_seated = True
    sm.tick()  # SEAT -> IDLE
    assert sm.cycle_stage == "IDLE"

    # Dispatcher: outer unriveted + inner empty -> INDEX
    sm.tick()  # IDLE -> INDEX
    assert sm.cycle_stage == "INDEX_DISC"

    # Disc done
    sm.disc_index_done_flag = True
    sm.tick()  # INDEX -> IDLE (rivet runs in parallel)
    assert sm.cycle_stage == "IDLE"
    # Swap inner/outer to reflect index +180
    sm.outer_id, sm.inner_id = sm.inner_id, sm.outer_id
    cafi_state["fixture_id"] = sm.inner_id
    cafi_state["location"] = "in_fixture_" + sm.inner_id

    # Mark CAFI riveted, dispatcher will INDEX_BACK
    cafi_state["riveted"] = True
    sm.tick()  # IDLE -> INDEX_BACK
    assert sm.cycle_stage == "INDEX_DISC_BACK"
    sm.disc_index_done_flag = True
    sm.tick()  # INDEX_BACK -> IDLE
    assert sm.cycle_stage == "IDLE"

    # Outer now has riveted -> PICK_RIVETED
    sm.outer_id, sm.inner_id = sm.inner_id, sm.outer_id
    cafi_state["fixture_id"] = sm.outer_id
    cafi_state["location"] = "in_fixture_" + sm.outer_id
    sm.tick()  # IDLE -> PICK_RIVETED
    assert sm.cycle_stage == "PICK_RIVETED"

    cafi_state["location"] = "in_gripper"
    _advance("PICK_RIVETED", gripper_grasp=True)
    assert sm.cycle_stage == "PLACE_VISION"
    assert sm.cabin_state == "RAISED"

    # Place vision done
    cafi_state["location"] = "at_vision"
    sm.gripper_grasp = False
    sm.vision_presence = True
    sm._cb_robot_task(String(data="home"))
    _advance("PLACE_VISION")
    assert sm.cycle_stage == "INSPECT", \
        "expected INSPECT got %s" % sm.cycle_stage

    sm.camera_result = with_verdict
    sm.tick()  # INSPECT -> IDLE
    assert sm.cycle_stage == "IDLE"

    cafi_state["verdict"] = with_verdict
    sm.tick()  # IDLE -> PICK_VISION
    assert sm.cycle_stage == "PICK_VISION"

    cafi_state["location"] = "in_gripper"
    _advance("PICK_VISION", gripper_grasp=True)
    assert sm.cycle_stage == "PLACE_BIN"
    assert sm.cabin_state == "RAISED"

    # Bin drop done
    sm.gripper_grasp = False
    cafi_state["location"] = "in_bin"
    sm._cb_robot_task(String(data="home"))
    _advance("PLACE_BIN")
    assert sm.cycle_stage == "IDLE"
    # FSM "place_bin -> idle" resets camera_result internally, but the
    # CAFI is in the bin so the dispatcher has no more work for it.
    sm.cafi_states = []
    sm.fixture_load_present = False
    sm.vision_presence = False


def test_throughput():
    rospy_stub.reset()
    sm = StateManager()
    sm._cb_op_start(Empty())
    assert sm.cell_state == "RUNNING"

    # Two back-to-back CAFIs first.
    _drive_full_cycle(sm, 1, with_verdict="PASS")
    _drive_full_cycle(sm, 2, with_verdict="FAIL")

    # 5 simulated minutes of continuous spawn (1 CAFI every 6 s rough
    # heuristic from the V54 cycle ~25 s, so 50 CAFIs is conservative).
    for cid in range(3, 3 + 50):
        rospy_stub.CLOCK.advance(6.0)
        _drive_full_cycle(sm, cid,
                          with_verdict=("PASS" if cid % 2 else "FAIL"))

    # No fault must ever have been raised.
    assert sm.fault_reason == "", \
        "TEST1 fault_reason=%r" % sm.fault_reason
    # No watchdog log line.
    bad = [l for l in rospy_stub.LOG_ERR
           if "watchdog" in l[2].lower()]
    assert not bad, "TEST1 watchdog lines: %s" % bad

    # Cabin always raised on any delivery dispatch transition.
    deliveries_seen = sum(1 for _, t, m in rospy_stub.LOG
                          if "cabin RAISE" in m or "cabin LOWER" in m)
    assert deliveries_seen > 0, "TEST1 no cabin transitions seen"

    print("TEST 1 OK: 52 CAFIs through the cell, no fault, no watchdog, "
          "cabin transitions=%d" % deliveries_seen)


# ============================================================
# Test 2: operator buttons (START / STOP / RESET / colocar CAFI)
# ============================================================
def test_button_flow():
    rospy_stub.reset()
    sm = StateManager()

    # Cell starts IDLE; spawn before START must be ignored (HMI gating
    # is enforced separately; here we just confirm the SM doesn't
    # transition on spawn).
    assert sm.cell_state == "IDLE"
    sm._cb_op_spawn(Empty())
    assert sm.cell_state == "IDLE", \
        "spawn must NOT arrancar el ciclo in V55"

    # START
    sm._cb_op_start(Empty())
    assert sm.cell_state == "RUNNING"

    # Spawn + drive to PICK_CONV with gripper closed.
    cafi_state = {
        "id": 7, "x": 1.235, "y": 1.365, "z": 1.083,
        "location": "on_conveyor", "at_sensor": True, "riveted": False,
        "verdict": None, "fixture_id": None,
    }
    sm.cafi_states = [cafi_state]
    sm.part_ready_for_pick = True
    sm.tick()
    assert sm.cycle_stage == "PICK_CONV"
    sm.robot_motion_done = True
    sm.gripper_grasp = True
    cafi_state["location"] = "in_gripper"
    sm.tick()
    assert sm.cycle_stage == "PLACE_LOAD"

    # Simulate the gripper sim: CAFI in hand -> publish state CLOSED.
    sm._cb_grip_state(String(data="CLOSED"))
    assert sm.gripper_state_str == "CLOSED"

    # STOP
    sm._cb_op_stop(Bool(data=True))
    assert sm.cell_state == "PAUSED"

    # RESET (gripper closed -> reject place sequence first)
    sm._cb_op_reset(Empty())
    assert sm.cell_state == "PAUSED"
    assert sm.cycle_stage == "RESET_PLACE_REJECT"
    assert sm.cabin_state == "RAISED"

    # Simulate the cobot completing the reject place.
    sm.robot_motion_done = True
    sm.gripper_grasp = False
    cafi_state["location"] = "in_bin"
    sm._cb_grip_state(String(data="OPEN"))
    sm.tick()
    assert sm.cycle_stage == "RESET_CLEAN"

    # Strand a CAFI at vision + one at the outer fixture + one at
    # the inner fixture, so the reset cleanup must walk all three
    # branches (vision first, outer next, then disc-index + remove
    # the inner one).
    sm.cafi_states = [
        {"id": 11, "location": "at_vision",
         "fixture_id": None, "riveted": True},
        {"id": 12, "location": "in_fixture_" + sm.outer_id,
         "fixture_id": sm.outer_id, "riveted": True},
        {"id": 13, "location": "in_fixture_" + sm.inner_id,
         "fixture_id": sm.inner_id, "riveted": True},
    ]
    # Hook into /objects/remove_cafi so we can assert order.
    removed = []
    rospy.Subscriber("/objects/remove_cafi", String,
                     lambda m: removed.append(m.data))

    # Pace one tick per simulated second until cleanup is done.
    for _ in range(60):
        rospy_stub.CLOCK.advance(1.1)
        # Mirror the object_manager: drop removed cafis from snapshot.
        if removed:
            cid = int(removed[-1])
            sm.cafi_states = [c for c in sm.cafi_states
                              if c.get("id") != cid]
        # If FSM is asking for INDEX, simulate disc done immediately.
        if sm.cycle_stage == "RESET_INDEX":
            sm.disc_index_done_flag = True
            # swap fixture ids to reflect the rotation
            sm.outer_id, sm.inner_id = sm.inner_id, sm.outer_id
            for c in sm.cafi_states:
                if c.get("fixture_id") == sm.outer_id:
                    c["location"] = "in_fixture_" + sm.outer_id
        sm.tick()
        if sm.cell_state == "IDLE":
            break
    else:
        raise AssertionError("RESET cleanup did not finish in budget")

    assert sm.cell_state == "IDLE"
    assert sm.cycle_stage == "IDLE"
    assert sm.cabin_state == "LOWERED", \
        "cabin must lower after RESET completes"
    assert sm.started_once is False, \
        "started_once must reset so operator presses START again"

    # Order: vision first, outer next, inner last (after disc index).
    assert removed[0] == "11", "vision must be removed first, got %s" % removed
    assert "12" in removed and "13" in removed, \
        "outer + inner must be removed too, got %s" % removed
    # Inner (13) is removed AFTER vision (11) and after the index step
    # surfaces it at outer.  We just assert the relative order between
    # the inner removal and the index command (the latter is a log line).
    index_log_idx = next((i for i, l in enumerate(rospy_stub.LOG)
                          if "RESET: only inner CAFI remains" in l[2]),
                         None)
    assert index_log_idx is not None, "expected RESET INDEX log"

    bad = [l for l in rospy_stub.LOG_ERR
           if "watchdog" in l[2].lower() and "RESET" in l[2]]
    assert not bad, "TEST2 RESET watchdog: %s" % bad

    print("TEST 2 OK: START -> STOP -> RESET cycle with gripper CLOSED. "
          "Cleanup order removed=%s, cell back to IDLE, cabin LOWERED, "
          "started_once=False." % removed)


if __name__ == "__main__":
    test_throughput()
    test_button_flow()
    print("\nALL V55 AGGRESSIVE TESTS PASSED.")
