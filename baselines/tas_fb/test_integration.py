"""Small model/policy/export checks. Run: python test_integration.py"""

import copy
import time
from joint_model import solve, tiny_instances, validate
from run_joint import export_sipp


def main():
    case, _ = tiny_instances()[0]
    result = solve(case, seconds=5, workers=1, wall_deadline=time.monotonic() + 10, first_after_base=True)
    assert result["status"] == "OPTIMAL" and result["solver_seconds"] < 5 and result["validated"]
    extended = solve(case, seconds=0, workers=1, wall_deadline=time.monotonic() + 10, first_after_base=True)
    assert extended["extended"] and len(extended["incumbent_updates"]) == 1 and extended["validated"]
    # Multiple service waypoints and a PRM distance differing from Euclidean.
    case = copy.deepcopy(case)
    case["locations"] = [
        dict(name="w0", position=[0, 0]),
        dict(name="w1", position=[3, 0]),
        dict(name="w2", position=[4, 0]),
    ]
    case["tasks"][0]["locations"] = [0]
    case["tasks"][1]["locations"] = [1, 2]
    case["travel_ticks"] = [[0, 900, 400], [900, 0, 500], [400, 500, 0]]
    case["start_travel_ticks"] = [[0, 900, 400]]
    result = solve(case, seconds=5, workers=1)
    assert result["status"] == "OPTIMAL" and result["validated"]
    assert result["makespan"] == 8.5
    assert next(a for a in result["actions"] if a["kind"] == "service" and a["task"] == 1)["goal"] == 2
    plans, events = export_sipp(case, result)
    assert len(events) == 1 and events[0]["kind"] == "pickup_tool"
    assert sum(len(x) == 4 and x[2] == 2 for seq in plans.values() for x in seq) == 1
    # Precedence-only model must not create any tool actions.
    for t in case["tasks"]:
        t["tool"] = False
    result = solve(case, seconds=5, workers=1)
    assert result["validated"] and all(a["kind"] == "service" for a in result["actions"])
    print("PASS: early optimum; extension first feasible; PRM matrix; waypoint choice; export; no tool on precedence.")


if __name__ == "__main__":
    main()
