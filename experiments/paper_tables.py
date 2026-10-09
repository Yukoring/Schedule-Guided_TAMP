#!/usr/bin/env python3
"""Tables of the paper from the measured results in results/paper/*.csv.

Usage: paper_tables.py [results/paper]
Cell = success/n · mean makespan of the successes (s). Paired cell = n jointly solved · Ours shorter-tied-longer (tie 0.01 s).
"""
import csv, statistics, sys
from collections import defaultdict
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
}
ORDER = ["ours", "no_fb", "u", "diverse_u", "u_ad_pd", "u_rt", "cbtamp", "tp", "tp_sipp", "tas_sipp", "tas_fb", "itags"]
FAMILIES = ["MP", "SP", "MT", "ST", "R2J8", "R4J16", "R6J24", "R8J32", "den312d", "warehouse", "HW_MP", "HW_MT"]
TIE = 0.01


def load(path):
    rows = list(csv.DictReader(open(path)))
    for r in rows:
        r["success"] = r["success"] == "True"
        r["makespan_s"] = float(r["makespan_s"]) if r["success"] else None
    return rows


def cell(rows):
    ok = [r["makespan_s"] for r in rows if r["success"]]
    if not rows:
        return "—"
    return "%d/%d · %.1f" % (len(ok), len(rows), statistics.mean(ok)) if ok else "%d/%d" % (0, len(rows))


def paired(ours, other):
    by = {(r["case"], r["seed"]): r for r in other if r["success"]}
    pairs = [
        (r["makespan_s"], by[(r["case"], r["seed"])]["makespan_s"])
        for r in ours
        if r["success"] and (r["case"], r["seed"]) in by
    ]
    if not pairs:
        return "n=0"
    s = sum(a < b - TIE for a, b in pairs)
    l = sum(a > b + TIE for a, b in pairs)
    return "n=%d %.1f/%.1f %d-%d-%d" % (
        len(pairs),
        statistics.mean(a for a, _ in pairs),
        statistics.mean(b for _, b in pairs),
        s,
        len(pairs) - s - l,
        l,
    )


def table(rows, title):
    fams = [f for f in FAMILIES if any(r["family"] == f for r in rows)]
    methods = [m for m in ORDER if any(r["method"] == m for r in rows)]
    seeds = sorted({r["seed"] for r in rows})
    print("\n## %s (seeds %s)\n" % (title, ", ".join(seeds)))
    print("|Family|" + "|".join(LABEL[m] for m in methods) + "|")
    print("|---|" + "---|" * len(methods))
    for f in fams:
        print(
            "|%s|" % f
            + "|".join(cell([r for r in rows if r["family"] == f and r["method"] == m]) for m in methods)
            + "|"
        )
    if "ours" in methods and len(methods) > 1:
        others = [m for m in methods if m != "ours"]
        print("\n|Family|" + "|".join("Ours vs " + LABEL[m] for m in others) + "|")
        print("|---|" + "---|" * len(others))
        for f in fams:
            fr = [r for r in rows if r["family"] == f]
            print(
                "|%s|" % f
                + "|".join(
                    paired([r for r in fr if r["method"] == "ours"], [r for r in fr if r["method"] == m])
                    for m in others
                )
                + "|"
            )


if __name__ == "__main__":
    d = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent / "results/paper")
    for name, title in [
        ("main", "Table 1"),
        ("ablation", "Table 3"),
        ("scalability", "Table 2 / Fig. 4"),
        ("hw", "Hardware"),
        ("tasfb", "TAS+FB (supplementary)"),
    ]:
        if (d / (name + ".csv")).exists():
            table(load(d / (name + ".csv")), title)
