# Measured results

Numbers reported in the paper (budget 100 s, 4 cores per run).

CSV columns: `experiment, case, family, method, seed, success, makespan_s, runtime_s, validation_end_s, end_reason`.
`validation_end_s` is the time (from the start of the run) at which the returned plan finished validation; it is below 100 s for every
successful run. `runtime_s` is defined per method as in the paper tables: for Ours and its ablations the run time capped at the 100 s budget (a process
that ended after the budget, for example while an extra round was still running, counts as 100 s and keeps the plan validated earlier);
for TAS+SIPP, TAS+FB and ITAGS the worker time from its start to the end of validation (capped at 100 s for ITAGS); for CBTAMP, TP and
TP+SIPP the process wall time, so a failed run stopped by the budget can show slightly more than 100 s. `end_reason` is the stage at which a
run ended: `VALID` (validated plan returned); for Ours and the POPF2 baselines `BUDGET_EXHAUSTED`, `SEARCH_EXHAUSTED` (no planner
candidate left), `NO_REPAIRABLE_CONFLICT`, `NO_PATH` / `MOTION_REJECTED` (TP+SIPP: SIPP found no path / rejected the plan); for TAS+SIPP
`NO_SCHEDULE` (CP-SAT found no schedule within its cap), `SCHEDULE_INVALID`, `SIPP_FAILED` (schedule found, SIPP produced no trajectories);
for ITAGS `MOTION_FAILED`, `TIMEOUT`. For example, TAS+SIPP on the tool missions: MT 51 x SIPP_FAILED + 9 x NO_SCHEDULE, ST 60 x
SIPP_FAILED (no run failed in validation or by timeout). Cell = success/n · mean makespan of successes (s); runtime cells = mean runtime (all runs) / first valid plan / final validated plan (successes), s; paired = Ours shorter-tied-longer on jointly solved rows. Per-run records: main.csv, ablation.csv, scalability.csv, tasfb.csv, hw.csv.

## Ablation (Table 3, seed 0)

|Mission|Ours|No FB|U|Diverse U|U+AD+PD|U+RT|
|---|---|---|---|---|---|---|
|MP|20/20 · 42.3|15/20 · 39.3|20/20 · 50.8|20/20 · 51.1|20/20 · 47.5|20/20 · 44.4|
|SP|20/20 · 39.8|17/20 · 43.0|20/20 · 52.1|20/20 · 50.1|20/20 · 43.8|20/20 · 49.0|
|MT|20/20 · 61.3|20/20 · 69.2|20/20 · 74.5|20/20 · 73.3|20/20 · 74.5|20/20 · 64.2|
|ST|19/20 · 62.9|17/20 · 67.5|19/20 · 66.8|19/20 · 67.1|19/20 · 66.8|18/20 · 62.8|

|Mission|Ours vs No FB|Ours vs U|Ours vs Diverse U|Ours vs U+AD+PD|Ours vs U+RT|
|---|---|---|---|---|---|
|MP|n=15 41.0/39.3 0-12-3|n=20 42.3/50.8 15-5-0|n=20 42.3/51.1 16-3-1|n=20 42.3/47.5 10-9-1|n=20 42.3/44.4 5-12-3|
|SP|n=17 39.9/43.0 3-14-0|n=20 39.8/52.1 12-8-0|n=20 39.8/50.1 12-8-0|n=20 39.8/43.8 6-13-1|n=20 39.8/49.0 8-12-0|
|MT|n=20 61.3/69.2 8-12-0|n=20 61.3/74.5 14-6-0|n=20 61.3/73.3 15-3-2|n=20 61.3/74.5 14-6-0|n=20 61.3/64.2 5-15-0|
|ST|n=17 62.4/67.5 6-11-0|n=19 62.9/66.8 10-8-1|n=18 62.7/66.9 11-6-1|n=19 62.9/66.8 10-8-1|n=18 62.8/62.8 0-18-0|

ABLATION  cell = mean runtime(all) / first valid(success) / final validated(success), s
|Mission|Ours|No FB|U|U+AD+PD|U+RT|
|---|---|---|---|---|---|
|MP|16.9 / 10.6 / 10.9|12.3 / 7.5 / 7.7|9.5 / 5.4 / 6.1|14.7 / 10.7 / 11.8|14.6 / 9.3 / 10.1|
|SP|17.7 / 10.1 / 14.3|11.9 / 7.9 / 8.9|10.4 / 4.9 / 6.6|14.7 / 8.8 / 11.7|19.4 / 8.9 / 11.7|
|MT|31.3 / 11.4 / 20.4|21.2 / 13.5 / 14.4|24.7 / 10.8 / 14.6|25.4 / 10.6 / 14.5|32.8 / 14.0 / 19.5|
|ST|54.6 / 22.7 / 35.0|44.3 / 16.7 / 20.2|38.7 / 21.1 / 28.3|38.2 / 20.4 / 27.6|55.1 / 19.1 / 32.2|


## Scalability (Fig. 4 / Table 2, seed 0)

|Family|Ours|CBTAMP|TP|TP+SIPP|TAS+SIPP|ITAGS|
|---|---|---|---|---|---|---|
|R2J8|20/20 · 95.7|20/20 · 122.7|20/20 · 127.3|20/20 · 126.5|18/20 · 78.6|10/20 · 79.8|
|R4J16|20/20 · 107.5|18/20 · 161.3|20/20 · 146.5|10/20 · 128.2|18/20 · 74.8|2/20 · 68.7|
|R6J24|20/20 · 127.4|17/20 · 167.8|20/20 · 147.7|6/20 · 142.6|10/20 · 76.4|0/20|
|R8J32|20/20 · 137.7|11/20 · 200.4|18/20 · 151.1|6/20 · 139.2|10/20 · 83.2|0/20|
|den312d|10/10 · 179.2|10/10 · 232.6|10/10 · 188.2|4/10 · 167.2|3/10 · 137.3|0/10|
|warehouse|10/10 · 234.5|10/10 · 276.8|10/10 · 263.6|6/10 · 246.1|7/10 · 165.0|0/10|

|Family|vs CBTAMP|vs TP|vs TP+SIPP|vs TAS+SIPP|vs ITAGS|
|---|---|---|---|---|---|
|R2J8|n=20 95.7/122.7 18-0-2|n=20 95.7/127.3 19-1-0|n=20 95.7/126.5 18-0-2|n=18 95.4/78.6 1-0-17|n=10 83.7/79.8 2-0-8|
|R4J16|n=18 107.5/161.3 18-0-0|n=20 107.5/146.5 20-0-0|n=10 109.7/128.2 10-0-0|n=18 108.5/74.8 0-0-18|n=2 85.6/68.7 0-0-2|
|R6J24|n=17 129.3/167.8 17-0-0|n=20 127.4/147.7 17-0-3|n=6 127.3/142.6 5-1-0|n=10 129.4/76.4 0-0-10|n=0|
|R8J32|n=11 141.7/200.4 9-1-1|n=18 141.0/151.1 10-1-7|n=6 130.4/139.2 2-4-0|n=10 144.1/83.2 0-0-10|n=0|
|den312d|n=10 179.2/232.6 7-0-3|n=10 179.2/188.2 8-0-2|n=4 160.8/167.2 2-2-0|n=3 182.3/137.3 0-0-3|n=0|
|warehouse|n=10 234.5/276.8 6-4-0|n=10 234.5/263.6 6-4-0|n=6 229.2/246.1 4-1-1|n=7 242.6/165.0 0-0-7|n=0|

|Family|Ours|CBTAMP|TP|TP+SIPP|TAS+SIPP|ITAGS|
|---|---|---|---|---|---|---|
|R2J8|5.3 / 5.1 / 5.1|1.5 / 1.3 / 1.3|1.6 / 1.4 / 1.4|1.2 / 1.1 / 1.1|1.1 / — / 1.1|1.9 / 1.8 / 1.8|
|R4J16|13.0 / 9.2 / 10.4|14.7 / 8.1 / 8.1|2.6 / 2.5 / 2.5|2.6 / 2.8 / 2.8|4.8 / — / 4.8|10.0 / 9.3 / 9.3|
|R6J24|36.6 / 20.1 / 33.1|40.2 / 29.5 / 29.5|5.2 / 5.0 / 5.0|2.3 / 2.5 / 2.5|38.4 / — / 43.5|80.1 / — / —|
|R8J32|41.8 / 24.6 / 33.9|65.9 / 37.6 / 37.6|36.2 / 29.0 / 29.0|4.1 / 4.1 / 4.1|61.9 / — / 62.5|100.0 / — / —|
|den312d|43.9 / 27.0 / 30.8|34.0 / 33.9 / 33.9|16.0 / 15.9 / 15.9|7.5 / 6.5 / 6.5|68.3 / — / 68.6|100.0 / — / —|
|warehouse|40.3 / 27.3 / 38.2|18.6 / 18.5 / 18.5|12.7 / 12.5 / 12.5|17.6 / 13.4 / 13.4|75.1 / — / 73.0|100.0 / — / —|

## Main table (Table 1, seeds 0, 1, 2)

Table 1 intervals are approximate 95 % confidence intervals: mean +- 1.96 x SE, where SE is the standard deviation of 100,000
instance-cluster bootstrap means (20 instances drawn with replacement per family, the three seed runs of an instance kept together,
makespan averaged over the successful runs of a sample). Computed by `experiments/paper_ci.py`.

seed 0
|Mission|Ours|CBTAMP|TP|TP+SIPP|TAS+SIPP|ITAGS|
|---|---|---|---|---|---|---|
|MP|20/20 · 42.8|17/20 · 79.4|20/20 · 66.4|5/20 · 55.5|20/20 · 26.1|12/20 · 25.8|
|SP|20/20 · 39.8|18/20 · 87.6|20/20 · 65.2|6/20 · 49.3|19/20 · 26.3|14/20 · 25.9|
|MT|20/20 · 64.0|17/20 · 113.0|16/20 · 106.5|8/20 · 86.7|0/20|—|
|ST|18/20 · 62.8|18/20 · 109.2|15/20 · 97.4|8/20 · 77.7|0/20|—|

seed 1
|Mission|Ours|CBTAMP|TP|TP+SIPP|TAS+SIPP|ITAGS|
|---|---|---|---|---|---|---|
|MP|20/20 · 41.3|17/20 · 80.2|18/20 · 64.5|5/20 · 55.1|20/20 · 26.2|12/20 · 25.9|
|SP|20/20 · 41.6|19/20 · 79.3|19/20 · 62.2|7/20 · 49.3|20/20 · 26.5|14/20 · 26.1|
|MT|20/20 · 62.9|16/20 · 115.6|19/20 · 101.4|8/20 · 86.9|0/20|—|
|ST|20/20 · 62.3|13/20 · 117.2|17/20 · 96.0|8/20 · 77.6|0/20|—|

seed 2
|Mission|Ours|CBTAMP|TP|TP+SIPP|TAS+SIPP|ITAGS|
|---|---|---|---|---|---|---|
|MP|20/20 · 43.1|19/20 · 79.7|20/20 · 68.0|5/20 · 55.1|20/20 · 26.2|12/20 · 25.8|
|SP|20/20 · 39.9|17/20 · 77.8|20/20 · 64.5|7/20 · 49.5|19/20 · 26.2|14/20 · 25.9|
|MT|20/20 · 63.5|14/20 · 129.7|19/20 · 117.6|8/20 · 86.7|0/20|—|
|ST|18/20 · 60.7|11/20 · 125.9|15/20 · 98.2|7/20 · 79.1|0/20|—|

seed pool
|Mission|Ours|CBTAMP|TP|TP+SIPP|TAS+SIPP|ITAGS|
|---|---|---|---|---|---|---|
|MP|60/60 · 42.4|53/60 · 79.8|58/60 · 66.4|15/60 · 55.2|60/60 · 26.2|36/60 · 25.8|
|SP|60/60 · 40.5|54/60 · 81.6|59/60 · 64.0|20/60 · 49.4|58/60 · 26.3|42/60 · 26.0|
|MT|60/60 · 63.5|47/60 · 118.9|54/60 · 108.6|24/60 · 86.8|0/60|—|
|ST|56/60 · 61.9|42/60 · 116.1|47/60 · 97.1|23/60 · 78.1|0/60|—|

pooled paired (Ours vs X, same case+seed, both solved): n, mean Ours/X, S-T-L
|Mission|vs CBTAMP|vs TP|vs TP+SIPP|vs TAS+SIPP|vs ITAGS|
|---|---|---|---|---|---|
|MP|n=53 41.9/79.8 53-0-0|n=58 42.6/66.4 58-0-0|n=15 40.7/55.2 15-0-0|n=60 42.4/26.2 0-0-60|n=36 41.5/25.8 0-0-36|
|SP|n=54 41.0/81.6 53-0-1|n=59 40.7/64.0 56-0-3|n=20 38.0/49.4 13-7-0|n=58 40.6/26.3 0-0-58|n=42 40.7/26.0 0-0-42|
|MT|n=47 62.0/118.9 47-0-0|n=54 64.0/108.6 54-0-0|n=24 61.4/86.8 23-1-0|n=0|n=0|
|ST|n=39 62.1/117.8 39-0-0|n=45 62.1/98.2 44-1-0|n=19 65.9/80.0 15-3-1|n=0|n=0|

## TAS+FB (supplementary, seed 0)

|Family|TAS+SIPP (60 s, one shot)|TAS+FB round 1 (5 s cap)|TAS+FB final|Ours|
|---|---|---|---|---|
|MP|20/20 · 26.1|20/20 · 26.0|20/20 · 26.0|20/20 · 42.8|
|SP|19/20 · 26.3|20/20 · 26.5|20/20 · 26.5|20/20 · 39.8|
|MT|0/20|0/20|0/20|20/20 · 64.0|
|ST|0/20|0/20|1/20 · 73.5|18/20 · 62.8|
|R2J8|18/20 · 78.6|18/20 · 78.6|20/20 · 80.3|20/20 · 95.7|
|R4J16|18/20 · 74.8|15/20 · 74.3|20/20 · 73.9|20/20 · 107.5|
|R6J24|10/20 · 76.4|4/20 · 179.5|12/20 · 215.4|20/20 · 127.4|
|R8J32|10/20 · 83.2|1/20 · 237.9|2/20 · 199.2|20/20 · 137.7|
|den312d|3/10 · 137.3|0/10|0/10|10/10 · 179.2|
|warehouse|7/10 · 165.0|1/10 · 510.8|1/10 · 510.8|10/10 · 234.5|
