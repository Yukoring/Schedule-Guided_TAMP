# Hardware inputs (2.9 x 2.9 m workspace, four RoboMaster EP)

`planner/` holds HW_MP_s0..s4 (precedence missions) and HW_MT_s0..s4 (tool missions), the inputs of `run_batch.py --exp hw`.
Lengths are in planner units, the physical layout divided by 0.30 m (workspace +-4.833, robot radius 0.833, goal regions 2 x 2); times are not
scaled. Service durations are 1 / 15 / 30 / 60 s per task type, permuted per robot; starts are near the corners with a +-0.05 m jitter per seed.
