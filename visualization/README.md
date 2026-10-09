# Supplementary videos

Animations of planned and replayed trajectories for the instances shown in the supplementary video (main table, scalability, ablation; seed 0).

```bash
PY=python
$PY visualization/sim_extract.py main scal abl        # trajectories from results/<exp>/runs/<EXP>__<case>__<method>__seed0 -> visualization/traj/
$PY visualization/render_all.py                       # one mp4 per (instance, method) in visualization/videos/, one playback speed-up per instance
$PY visualization/organize.py                         # visualization/by_instance/<scene>/<instance>/<method>.mp4 + README with speed-ups
```

Instances: MP_s6, SP_s3, MT_s0, ST_s1 (main table); R4J16_s2, R6J24_s7, R8J32_s8, warehouse_s107 (scalability); MP_s5, SP_s11, MT_s6, ST_s8
(ablation); edit `SCENES` in sim_extract.py for others. Success videos replay the validated trajectory. Failed runs replay the method's last
candidate (TAS+SIPP and ITAGS: the schedule executed on the roadmap without SIPP) and stop one step before the first collision or precedence
violation; runs without any plan are 3 s static cards. Visual grammar: teal goal regions, task chips ▲ ■ ● ◆ in precedence order, hammer = tool,
countdown ring during a service, "park" during a final move. Requires ffmpeg and matplotlib. The supplementary videos are distributed separately.
