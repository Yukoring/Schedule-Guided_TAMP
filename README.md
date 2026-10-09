# Schedule-Guided Task and Motion Planning for Multi-Robot Multi-Goal Problems

Code, inputs and results of the experiments in the paper (AAMAS 2027 submission).

```
common/        shared stack used by every method: PRM roadmap, temporal planner interface (POPF2 binary in common/code/planners), CP-SAT
               scheduling model, prioritized SIPP motion layer, tool hand-over adapter, validator, run worker (run_worker.py), budget / planner pool
common/variants/   overlays: diverse_u (ablation), hw (hardware planning inputs with non-integer arena bounds)
ours/          Ours: the schedule-guided control layer (ours_loop.py)
baselines/     cbtamp/, tp/, tp_sipp/ (temporal-planning baselines on the shared stack), tas_sipp/ (TAS+SIPP), tas_fb/ (TAS+FB), itags/ (ITAGS)
inputs/        struct (80: MP, SP, MT, ST), scal (80 room-map instances, 2 to 8 robots), maps (20: den312d, warehouse), hw (10 hardware inputs)
experiments/   run_batch.py (every experiment), paper_tables.py (tables from results/paper), make_tables.py (tables from new batches),
               EXPERIMENTS.md (paper table -> command)
visualization/ scripts for the supplementary videos (trajectory extraction, rendering)
results/paper/ the measured results of the paper (per-run CSVs and the tables); new batches go to results/<exp>/
```

## Setup

Linux x86_64, Python 3.12, `pip install -r requirements.txt`. The POPF2 binary is included in `common/code/planners`. The ITAGS
binary is not included (a compiled binary carries machine-specific library paths): build it with `baselines/itags/package/build.sh`
(OR-tools C++ 9.14, OMPL, GEOS, yaml-cpp, fmt, spdlog, Eigen; the binary is written to `baselines/itags/package/build/itags_cpsat`).

## Running the experiments

```bash
PY=python   # the venv above
$PY experiments/run_batch.py single --method ours --input inputs/struct/MP_s6.yaml --cores 0,1,2,3        # one run of Ours
$PY experiments/run_batch.py single --method cbtamp --input inputs/struct/MP_s6.yaml --cores 0,1,2,3      # one run of a baseline
$PY experiments/run_batch.py run --exp main --seeds 0 1 2 --slots A:0,1,2,3 B:6,7,8,9   # Table 1: 80 inputs x 6 methods x 3 seeds -> results/main
$PY experiments/run_batch.py run --exp ablation                                         # Table 3: 80 inputs x {Ours, U, Diverse U, U+AD+PD, U+RT, No FB}
$PY experiments/run_batch.py run --exp scalability                                      # Fig. 4 / Table 2: room maps + den312d + warehouse
$PY experiments/run_batch.py run --exp tasfb                                            # supplementary: TAS+FB on all 180 inputs
$PY experiments/run_batch.py run --exp hw                                               # planning on the hardware inputs (no results shipped)
$PY experiments/paper_tables.py                                                          # the tables of the paper from results/paper/*.csv
$PY experiments/paper_ci.py                                                              # Table 1 approximate 95 % CIs (instance-cluster bootstrap)
$PY experiments/make_tables.py results/main results/ablation results/scalability        # tables of a new batch (per seed and pooled, runtime, paired)
```

Methods: ours, u, diverse_u, u_ad_pd, u_rt, no_fb, cbtamp, tp, tp_sipp, tas_sipp, tas_fb, itags (names as in the paper). A batch is resumable (completed runs are skipped); a `STOP_REQUESTED` file in the results
directory stops it after the current runs. Every run leaves its planner calls, candidate trajectories, validation output and `result_row.json`
under `results/<exp>/runs/<run_id>/`; `results/<exp>/output/runs.csv` aggregates the batch.
Inside a run folder, `pddl_domain/` holds the domain used (`ours_domain_{early,late}.pddl` for Ours and its ablations,
`tp_domain_{early,late}.pddl` for TP, TP+SIPP and CBTAMP; tool missions add the tool actions), and `pddl_prob_plan/` holds one problem file
per planner call: `problem_round<k>_<guidance>.pddl` with guidance U, AD, PD or RT for Ours (`problem_beta<j>_round<k>_<guidance>.pddl` for
the retries with a larger beta), `problem_round<k>.pddl` for the baselines. `calls/<id>/` keeps the domain, problem and returned plan of every
POPF2 call.

## Protocol

Budget 100 s per run from roadmap construction to the last validation, 4 dedicated cores per run, two runs in parallel on disjoint physical
cores, PRM seed = seed (0, 1, 2 for Table 1; 0 elsewhere); the run seed is also the CP-SAT seed of TAS+SIPP, TAS+FB and ITAGS, while the
scheduling CP-SAT of Ours and its ablations uses seed 0. CP-SAT: 4 workers, 5 s per call (Ours), gap 5 % / 60 s cap (TAS+SIPP), 10 s per
refinement / gap 0.1 (ITAGS).
Motion layer for every method: prioritized SIPP on the shared roadmap, no endpoint retreat, no hand-over wait-aside detour, a finished robot keeps
occupying its last location; the same validator accepts or rejects every candidate. A run is successful when a validated plan exists within the budget.

## License and third-party components

This repository is released under the GNU General Public License v3 (`LICENSE`). It redistributes:
- D-ITAGS / GRSTAPSE (ITAGS baseline), https://github.com/gneville6/D-ITAGS, commit e779ca8b6c9877ca51a5ffa5b4550dcdb3e0cd38, GPL-3
  (`baselines/itags/package/upstream`, `LICENSE.upstream`), with our patches `compatibility.patch` and `upstream_cpsat.patch`.
- POPF2 temporal planner binary (`common/code/planners/popf2`), GPL, King's College London planning group.
- OR-tools (CP-SAT) as a dependency, Apache-2.0 (Python package `ortools`; the ITAGS binary links the C++ release 9.14).
