#!/usr/bin/env python3
"""Approximate 95 % confidence intervals of Table 1 (instance-cluster bootstrap).

For each mission family (MP, SP, MT, ST; 20 instances x seeds 0, 1, 2):
  1. draw 20 instances with replacement (the same draws are used for every method of the family);
  2. take the three seed runs of every drawn instance together (an instance drawn twice contributes its runs twice);
  3. makespan of a sample = sum of the makespans of its successful runs / number of successful runs
     (failures are left out; a sample with no success is skipped); runtime = mean over all runs of the sample;
  4. repeat 100,000 times;
  5. SE = standard deviation (ddof=1) of the bootstrap means; the table shows mean +- 1.95996 x SE.
This is a symmetric normal-approximation interval with a bootstrap standard error, not a percentile interval.
Random generator: numpy Generator(PCG64(20261008)), created once, families processed in the order MP, SP, MT, ST,
instances sorted by case name, indices rng.integers(0, 20, size=(100000, 20)) per family.

Usage: paper_ci.py [results/paper/main.csv]
"""
import csv, sys
from pathlib import Path
import numpy as np

B = 100_000
Z = 1.9599639845400536
FAMILIES = ["MP", "SP", "MT", "ST"]
METHODS = ["ours", "cbtamp", "tp", "tp_sipp", "tas_sipp", "itags"]
LABEL = {"ours": "Ours", "cbtamp": "CBTAMP", "tp": "TP", "tp_sipp": "TP+SIPP", "tas_sipp": "TAS+SIPP", "itags": "ITAGS"}


def main(path):
    rows = list(csv.DictReader(open(path)))
    rng = np.random.Generator(np.random.PCG64(20261008))
    out = {}
    for fam in FAMILIES:
        cases = sorted({r["case"] for r in rows if r["family"] == fam})
        idx = rng.integers(0, len(cases), size=(B, len(cases)))
        for m in METHODS:
            runs = [r for r in rows if r["family"] == fam and r["method"] == m]
            if not runs:
                continue
            # per instance: sum of successful makespans, number of successes, sum of runtimes, number of runs
            ms_sum = np.zeros(len(cases))
            ok_n = np.zeros(len(cases))
            rt_sum = np.zeros(len(cases))
            run_n = np.zeros(len(cases))
            for r in runs:
                i = cases.index(r["case"])
                run_n[i] += 1
                rt_sum[i] += float(r["runtime_s"])
                if r["success"] == "True":
                    ok_n[i] += 1
                    ms_sum[i] += float(r["makespan_s"])
            n_ok = ok_n[idx].sum(1)
            keep = n_ok > 0
            ms_boot = ms_sum[idx].sum(1)[keep] / n_ok[keep]
            rt_boot = rt_sum[idx].sum(1) / run_n[idx].sum(1)
            ms_mean = ms_sum.sum() / ok_n.sum() if ok_n.sum() else None
            out[(fam, m)] = dict(
                success="%d/%d" % (ok_n.sum(), run_n.sum()),
                makespan=ms_mean,
                makespan_hw=Z * ms_boot.std(ddof=1) if ms_mean is not None else None,
                runtime=rt_sum.sum() / run_n.sum(),
                runtime_hw=Z * rt_boot.std(ddof=1),
            )
    print("|Family|" + "|".join(LABEL[m] for m in METHODS) + "|")
    print("|---|" + "---|" * len(METHODS))
    for fam in FAMILIES:
        cells = []
        for m in METHODS:
            c = out.get((fam, m))
            if c is None:
                cells.append("—")
            elif c["makespan"] is None:
                cells.append("%s · — · T %.1f ± %.1f" % (c["success"], c["runtime"], c["runtime_hw"]))
            else:
                cells.append(
                    "%s · %.1f ± %.1f · T %.1f ± %.1f"
                    % (c["success"], c["makespan"], c["makespan_hw"], c["runtime"], c["runtime_hw"])
                )
        print("|%s|" % fam + "|".join(cells) + "|")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "results/paper/main.csv")
