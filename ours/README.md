# Ours — schedule-guided control layer

`ours_loop.py` implements Algorithm 1 of the paper on top of the shared stack in `common/`. The routing-and-scheduling solution (CP-SAT,
`common/code/vrp`, `common/cpsat_limits.py`) is turned into task deadlines (AD, PD) and robot task-type restrictions (RT) for the temporal planner
(POPF2); guided and unguided calls run as a portfolio (U / AD / PD / RT, two phases, early/late variants). When the guided calls return nothing,
beta is increased and they are retried. Candidate plans are refined and repaired with prioritized SIPP (`common/sipp_repair.py`) and validated.
After a valid plan up to two more rounds are run, and collision regions are fed back to both the scheduling and the temporal planning model.

Methods: ours, u, u_ad_pd, u_rt, no_fb (ours stopped after the first round), diverse_u (the `common/variants/diverse_u` overlay).
Run: `experiments/run_batch.py single --method ours --input inputs/struct/MP_s6.yaml`.
