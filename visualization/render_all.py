#!/usr/bin/env python3
"""Render all traj files with ONE playback speed per instance: k = ceil(T_max / 60 s) over the methods of that instance (static
"no plan" cards excluded), so the videos of one instance can be shown side by side. Writes SPEEDS.json. 4 parallel workers."""
import json,glob,math,subprocess,collections,sys
from pathlib import Path
H=Path(__file__).resolve().parent;import sys as _s;PY=_s.executable;HOLD=2.0
files=sorted(glob.glob(str(H/'traj/*.json')));by=collections.defaultdict(list)
for f in files:
    d=json.load(open(f))
    if d.get('outcome') in ('none','missing'):T=0.0
    else:T=max(bp[-1][2] for bp in d['breakpoints'].values())-(HOLD if d.get('stop') else 0.0)
    by[(d['scene'],d['case'])].append((f,T))
speeds={f'{sc}__{case}':max(1,int(math.ceil(max(T for _,T in v)/60.0))) for (sc,case),v in by.items()}
json.dump({'rule':'k = ceil(T_max/60 s) per instance, T_max = longest run of that instance (no-plan cards excluded)','speeds':speeds,'T_max':{f'{sc}__{case}':round(max(T for _,T in v),1) for (sc,case),v in by.items()}},open(H/'SPEEDS.json','w'),indent=1)
jobs=[(f,speeds[f'{sc}__{case}']) for (sc,case),v in by.items() for f,_ in v]
procs=[]
for i in range(4):
    chunk=jobs[i::4];cmd=';'.join(f'{PY} {H}/sim_render.py {f} --speed {k}' for f,k in chunk)
    procs.append(subprocess.Popen(['bash','-c',cmd],stdout=open(H/f'render_{i}.log','w'),stderr=subprocess.STDOUT))
for p in procs:p.wait()
print('done',len(jobs));print(speeds)
