#!/usr/bin/env python3
"""Independent exhaustive-order oracle for the native C++ CP-SAT scheduler.
Enumerates every mutex orientation, then computes earliest starts by longest
paths. No optimization package is used by the oracle.
"""
import itertools
import json
from pathlib import Path
import random
import subprocess

ROOT = Path(__file__).resolve().parent


def oracle(p):
    n = len(p['durations'])
    best = None
    for bits in itertools.product([0, 1], repeat=len(p['mutexes'])):
        edges = list(p['precedences'])
        for bit, (a, b, ab, ba) in zip(bits, p['mutexes']):
            edges.append([a, b, ab] if bit else [b, a, ba])
        adj = [[] for _ in range(n)]
        indegree = [0]*n
        for a, b, travel in edges:
            adj[a].append((b, p['durations'][a]+travel))
            indegree[b] += 1
        ready = [i for i in range(n) if indegree[i] == 0]
        starts = p['releases'].copy()
        seen = 0
        while ready:
            a = ready.pop()
            seen += 1
            for b, lag in adj[a]:
                starts[b] = max(starts[b], starts[a]+lag)
                indegree[b] -= 1
                if indegree[b] == 0:
                    ready.append(b)
        if seen != n:
            continue
        key = (max(s+d for s, d in zip(starts, p['durations'])), sum(starts))
        best = key if best is None else min(best, key)
    return best


def main():
    cases = []
    rng = random.Random(771)
    # Includes varying service durations, releases, asymmetric transition times,
    # cross-robot precedence, partial-allocation relaxations and infeasibility.
    for k in range(36):
        n = 3+k % 3
        owner = [rng.choice([-1, 0, 1]) for _ in range(n)]
        mutexes = [[a, b, rng.randrange(6), rng.randrange(6)]
                   for a in range(n) for b in range(a+1, n)
                   if owner[a] >= 0 and owner[a] == owner[b]]
        precedence = [[a, b, rng.randrange(4) if owner[a] == owner[b] >= 0 else 0]
                      for a in range(n) for b in range(a+1, n) if rng.random() < .22]
        if k % 9 == 0:
            precedence += [[0, 1, 0], [1, 0, 0]]
        cases.append({'durations': [rng.randrange(1, 13) for _ in range(n)],
                      'releases': [rng.randrange(8) if r >= 0 else 0 for r in owner],
                      'precedences': precedence, 'mutexes': mutexes,
                      'relative_gap': 0.0, 'hierarchical': True, 'workers': 1, 'timeout': 5.0})
    proc = subprocess.run([str(ROOT/'build/cpsat_numeric')],
                          input=''.join(json.dumps(c)+'\n' for c in cases),
                          text=True, capture_output=True, timeout=180)
    if proc.returncode:
        raise RuntimeError(proc.stderr)
    results = [json.loads(line) for line in proc.stdout.splitlines() if line.startswith('{')]
    assert len(results) == len(cases), proc.stdout
    checks = []
    for k, (p, r) in enumerate(zip(cases, results)):
        expected = oracle(p)
        observed = (r['makespan'], sum(r['starts'])) if r['accepted'] else None
        ok = observed == expected and (expected is not None or r['primary_status'] == 'INFEASIBLE')
        checks.append({'case': k, 'passed': ok, 'expected': expected, 'observed': observed,
                       'problem': p, 'result': r})
    report = {'passed': sum(x['passed'] for x in checks), 'total': len(checks), 'checks': checks}
    (ROOT/'results').mkdir(exist_ok=True)
    (ROOT/'results/numeric_verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(f"Exhaustive oracle: {report['passed']}/{report['total']}")
    assert report['passed'] == report['total']


if __name__ == '__main__':
    main()
