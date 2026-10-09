# Experiments used in the paper and how to run them

All runs: 100 s budget per run (roadmap construction to validation), 4 dedicated cores, PRM seed = seed (the scheduling CP-SAT of Ours and its ablations uses seed 0; TAS and ITAGS use the run seed), final motion standard
(endpoint retreat off, hand-over wait-aside off, parked robots keep their last location, prioritized SIPP, the same validator).
`experiments/run_batch.py` plans a manifest, runs it on two 4-core slots (resumable, `STOP_REQUESTED` file stops after the current runs)
and writes `results/<exp>/output/runs.csv`. Tables are produced by `experiments/make_tables.py`.

| Paper table / figure | `--exp` | Inputs | Methods | Seeds |
|---|---|---|---|---|
| Main table (Tab. 1) | `main` | 80 structured (MP, SP, MT, ST x 20) | Ours, CBTAMP, TP, TP+SIPP, TAS+SIPP, ITAGS (MP/SP) | 0, 1, 2 |
| Ablation | `ablation` | 80 structured | Ours, No FB, U, Diverse U, U+AD+PD, U+RT | 0 |
| Scalability | `scalability` | rooms R2J8..R8J32 (20 each) + den312d, warehouse (10 each) | Ours, CBTAMP, TP, TP+SIPP, TAS+SIPP, ITAGS | 0 |
| TAS+FB (feedback on the TAS+SIPP baseline) | `tasfb` | structured 80 + rooms 80 + maps 20 | TAS+FB | 0 |
| Hardware planning (Tab. hardware) | `hw` | 10 HW inputs (2.9 m arena) | Ours, U, CBTAMP, TAS+SIPP | 0 |

Commands

```bash
PY=/path/to/venv/bin/python          # Python 3.12 with requirements.txt installed; the POPF2 binary is in common/code/planners
$PY experiments/run_batch.py plan --exp main --seeds 0 1 2
$PY experiments/run_batch.py run  --exp main --slots A:0,1,2,3 B:6,7,8,9      # resumable; results/main/
$PY experiments/run_batch.py run  --exp ablation
$PY experiments/run_batch.py run  --exp scalability
$PY experiments/run_batch.py run  --exp tasfb
$PY experiments/run_batch.py run  --exp hw
$PY experiments/run_batch.py single --method ours --input inputs/struct/MP_s6.yaml --cores 0,1,2,3   # one run
$PY experiments/make_tables.py results/main results/ablation results/scalability results/tasfb results/hw
```

Notes
- `diverse_u` and the `hw` experiment use stack variants (`common/variants/diverse_u`, `common/variants/hw`); the launcher materialises
  `common/_build/<variant>` on first use (common + overlay files). The variant diffs are `common/variants/*/*.patch`.
- Method code: Ours in `ours/ours_loop.py`; CBTAMP, TP, TP+SIPP in `baselines/{cbtamp,tp,tp_sipp}/`; shared pieces in `common/control_common.py`;
  the worker `common/run_worker.py` loads them through `common/methods.py`.
- ITAGS needs `baselines/itags/package/build_seed/itags_cpsat` (included, built from `baselines/itags/package` with OR-tools C++ 9.14; `build.sh`).
- Hardware execution (RoboMaster EP, NOKOV motion capture): the plans of `--exp hw` were executed as 0.03 s command trajectories; export and
  robot-side tooling are not part of this repository. Measured makespans are in the paper (Table 4).
