# Baselines

Every baseline uses the same roadmap (PRM), the same 100 s budget and 4-core allocation, the same motion standard (prioritized SIPP, no endpoint
retreat, no hand-over wait-aside detour, a finished robot keeps occupying its last location) and the same validator as Ours. Below: where each
method comes from and what we changed. The method names are the ones of the paper (`experiments/run_batch.py --method <name>`).

| Method | Code |
|---|---|
| tp (TP) | `tp/tp_loop.py` |
| tp_sipp (TP+SIPP) | `tp_sipp/tp_sipp_loop.py` |
| cbtamp (CBTAMP) | `cbtamp/cbtamp_loop.py` |
| tas_sipp (TAS+SIPP) | `tas_sipp/` |
| tas_fb (TAS+FB, supplementary) | `tas_fb/` |
| itags (ITAGS) | `itags/` |

## TP (temporal planner only)

- Origin: POPF2, Coles, Coles, Fox and Long, "POPF2: A forward-chaining partial order planner", IPC 2011. The planning model (PDDL domain and
  problem encoding) is the one of the CBTAMP paper.
- Code: the POPF2 binary (`common/code/planners/popf2`) and the PDDL generators of the CBTAMP implementation (`common/code/task_planning`).
- Adaptation: temporal planning on the roadmap; when a collision is detected the roadmap is resampled and the planner is called again. No schedule
  guidance, no collision regions, no SIPP. Budget, cores and the planner call policy (two profile phases, early/late variants) follow the common protocol.

## TP+SIPP

- Origin: POPF2 (as above) and SIPP, Phillips and Likhachev, "SIPP: Safe interval path planning for dynamic environments", ICRA 2011.
- Code: POPF2 binary; prioritized SIPP in `common/code/motion/prioritized.py` (our implementation, the same code Ours uses).
- Adaptation: one temporal plan, task assignment and action order kept fixed, prioritized SIPP adjusts routes and waits, one validation, no feedback.
  Tool missions go through the hand-over adapter (`common/baseline_tool_adapter.py`) that propagates drop -> pickup times.

## CBTAMP

- Origin: Lee and Long, "Conflict-Based Task and Motion Planning for Multi-Robot, Multi-Goal Problems", ICAR 2023.
- Code: the implementation of the CBTAMP paper. The roadmap, PDDL generators, collision checker (`common/code/motion/collision_vor.py`) and
  validator in `common/code/` come from that implementation and are shared by every method including Ours.
- Adaptation: the algorithm is unchanged: collision regions found in candidate trajectories are added to the temporal planning problem and the planner
  is called again until a plan passes validation or the budget ends. Only the protocol differs: 100 s budget, POPF2 backend, the two-phase planner
  profiles of the common protocol; the candidate adoption rules are the original ones. No SIPP.

## TAS+SIPP (task allocation and scheduling + SIPP)

- Origin: not an external implementation; a CP-SAT (OR-tools) formulation we wrote for the comparison ("TAS+SIPP" in Section 5.1 of the paper).
- Code: `tas_sipp/joint_model.py` (model), `tas_sipp/run_joint.py` (PRM -> CP-SAT -> SIPP -> validation).
- Model scope: task assignment, per-robot action order, start and end times, precedence, no overlapping services at a goal, the tool circuit
  (pickup / drop / hand-over only at the previous tool-service waypoint), vertex occupancy (arrival and departure times, NoOverlap per vertex) and
  explicit parking (stay, own start vertex or any service location; parking arrival counts in the makespan). Travel time = shortest roadmap path /
  speed. Stops at a relative gap of 5 % or 60 s. One prioritized SIPP attempt per schedule; a SIPP failure is a failure of the run.

## TAS+FB (supplementary)

- TAS+SIPP with the collision-region feedback of CBTAMP and rescheduling. When SIPP fails, the schedule is replayed literally on the roadmap, the
  shared `collision_vor.collision_check` produces regions, and the regions enter the model as spatial resources (NoOverlap on the crossing windows,
  stationary occupancy of nodes and parking inside a region); assignment, order, tool circuit and parking are re-chosen. The CP-SAT cap starts at
  5 s and doubles to 10, 20, 40 s when no schedule is found. Code `tas_fb/run_joint_fb.py`, model changes in `tas_fb/joint_model.fb.patch`.

## ITAGS

- Origin: Neville, Messing, Ravichandar, Hutchinson and Chernova, "An Interleaved Approach to Trait-Based Task Allocation and Scheduling", IROS 2021.
- Code: the public D-ITAGS repository, https://github.com/gneville6/D-ITAGS (commit e779ca8b6c9877ca51a5ffa5b4550dcdb3e0cd38), GPL-3.
  The copy we used is `itags/package/upstream/` (licence in `itags/package/LICENSE.upstream`); our changes are `itags/package/compatibility.patch`
  and `upstream_cpsat.patch` (both apply to that commit).
- Adaptation:
  - Scheduler: the Gurobi MILP of the original is replaced by CP-SAT (OR-tools C++, `itags/package/cpsat_scheduler.cpp`, `cpsat_model.cpp`). The
    allocation search (Itags) is the original code.
  - Motion environment: the point graph given to ITAGS is the run's PRM roadmap (vertices = samples + robot starts + goal service points, edge cost =
    Euclidean length), so travel times are PRM path lengths / speed.
  - Fix: the original point-graph motion query did not accumulate costs in the shared A*; it now runs a graph-specific Dijkstra and returns a duration
    of -1 on failure so that the scheduler reads a failed transition (`itags/package/compatibility.patch`). Normalisation constants follow the PRM.
  - Motion stage (our adapter, `itags/itags_adapter.py`): the first task-complete allocation and the per-robot orders are fixed; the shared
    prioritized SIPP plans the robots once, in robot id order (one attempt, no priority rotation); the plan goes through the shared collision
    checker and validator.
  - Scope: tool (hand-over) missions are not in the original model and are reported as N/A.
  - Stopping policy: CP-SAT per refinement capped at min(10 s, remaining budget), relative gap 0.1, 4 workers, seed = run seed; 100 s run budget.
  - Build: `itags/package/build.sh` (OR-tools C++ 9.14, release 9.14.6206); the binary is `itags/package/build/itags_cpsat` and is not shipped.
