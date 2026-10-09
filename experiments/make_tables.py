#!/usr/bin/env python3
"""Tables from run_batch results. usage: make_tables.py results/<exp> [results/<exp> ...] [--tie 0.01]
For every results dir: success/n and mean makespan of successes per family x method, runtime (mean runtime / first valid / final validated),
paired comparison of FULL against every other method on jointly solved rows (shorter-tied-longer), per seed and pooled over seeds."""
import csv, sys, statistics, collections
from pathlib import Path

LABEL = {
    "ours": "Ours",
    "no_fb": "No FB",
    "u": "U",
    "diverse_u": "Diverse U",
    "u_ad_pd": "U+AD+PD",
    "u_rt": "U+RT",
    "cbtamp": "CBTAMP",
    "tp": "TP",
    "tp_sipp": "TP+SIPP",
    "tas_sipp": "TAS+SIPP",
    "tas_fb": "TAS+FB",
    "itags": "ITAGS",
    "FULL": "Ours",
    "NOFB": "No FB",
    "U": "U",
    "U_MATCH": "Diverse U",
    "U_AD_PD": "U+AD+PD",
    "U_RT": "U+RT",
    "CBTAMP": "CBTAMP",
    "PLANNER_ONLY": "TP",
    "TASK_PRIORITIZED": "TP+SIPP",
    "JOINT_GAP": "TAS+SIPP",
    "JOINT_FB": "TAS+FB",
    "ITAGS": "ITAGS",
}
FAM = {"HW_MP": "HW MP", "HW_MT": "HW MT"}


def fl(x):
    try:
        return float(x)
    except Exception:
        return None


def load(d):
    rows = list(csv.DictReader(open(Path(d) / "output/runs.csv")))
    for r in rows:
        r["ok"] = r["success"] == "True"
        r["mk"] = fl(r["makespan"])
        r["rt"] = fl(r.get("runtime_s"))
        r["fv"] = fl(r.get("first_valid_s"))
        r["ve"] = fl(r.get("validation_end_s"))
        r["seed"] = int(r.get("seed") or 0)
    return rows


def table(rows, tie):
    fams = list(dict.fromkeys(r["family"] for r in rows))
    methods = list(dict.fromkeys(r["method"] for r in rows))
    seeds = sorted({r["seed"] for r in rows})
    for sd in (seeds if len(seeds) > 1 else []) + ["pool"]:
        sel = [r for r in rows if sd == "pool" or r["seed"] == sd]
        print(
            f'\n### {"all seeds pooled" if sd=="pool" else "seed "+str(sd)}   (success/n · mean makespan of successes, s)'
        )
        print("|Family|" + "|".join(LABEL.get(m, m) for m in methods) + "|")
        print("|---|" + "---|" * len(methods))
        for f in fams:
            cells = []
            for m in methods:
                g = [r for r in sel if r["family"] == f and r["method"] == m]
                ok = [r["mk"] for r in g if r["ok"] and r["mk"] is not None]
                cells.append(
                    "—"
                    if not g
                    else (
                        "%d/%d · %.1f" % (len(ok), len(g), statistics.mean(ok)) if ok else "%d/%d" % (len(ok), len(g))
                    )
                )
            print("|" + FAM.get(f, f) + "|" + "|".join(cells) + "|")
    print("\n### runtime pooled: mean runtime (all) / first valid (successes) / final validated (successes), s")
    print("|Family|" + "|".join(LABEL.get(m, m) for m in methods) + "|")
    print("|---|" + "---|" * len(methods))
    for f in fams:
        cells = []
        for m in methods:
            g = [r for r in rows if r["family"] == f and r["method"] == m]
            if not g:
                cells.append("—")
                continue
            rt = [r["rt"] for r in g if r["rt"] is not None]
            fv = [r["fv"] for r in g if r["ok"] and r["fv"] is not None]
            ve = [r["ve"] for r in g if r["ok"] and r["ve"] is not None]
            cells.append(
                "%s / %s / %s"
                % (
                    "%.1f" % statistics.mean(rt) if rt else "—",
                    "%.1f" % statistics.mean(fv) if fv else "—",
                    "%.1f" % statistics.mean(ve) if ve else "—",
                )
            )
        print("|" + FAM.get(f, f) + "|" + "|".join(cells) + "|")
    ours = "ours" if "ours" in methods else "FULL"
    if ours in methods and len(methods) > 1:
        print(
            "\n### paired, pooled over seeds: Ours vs X on jointly solved rows (n, mean Ours / X, shorter-tied-longer)"
        )
        others = [m for m in methods if m != ours]
        print("|Family|" + "|".join("vs " + LABEL.get(m, m) for m in others) + "|")
        print("|---|" + "---|" * len(others))
        idx = {(r["case"], r["method"], r["seed"]): r for r in rows}
        for f in fams:
            cells = []
            for m in others:
                pr = [
                    (a["mk"], b["mk"])
                    for (c, mm, s), a in idx.items()
                    if mm == ours and a["family"] == f and a["ok"]
                    for b in [idx.get((c, m, s))]
                    if b and b["ok"]
                ]
                cells.append(
                    "n=%d %.1f/%.1f %d-%d-%d"
                    % (
                        len(pr),
                        statistics.mean(x for x, _ in pr),
                        statistics.mean(y for _, y in pr),
                        sum(x < y - tie for x, y in pr),
                        sum(abs(x - y) <= tie for x, y in pr),
                        sum(x > y + tie for x, y in pr),
                    )
                    if pr
                    else "n=0"
                )
            print("|" + FAM.get(f, f) + "|" + "|".join(cells) + "|")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tie = float(sys.argv[sys.argv.index("--tie") + 1]) if "--tie" in sys.argv else 0.01
    for d in args:
        print(f"\n## {d}")
        table(load(d), tie)
