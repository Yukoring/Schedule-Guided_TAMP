#!/usr/bin/env python3
"""Arrange the rendered videos per instance: by_instance/<scene>/<case>/<method>.mp4 and write README.md with one table per
instance (playback speed-up, outcome, makespan, stop time). Scene folders: main_table, scalability, ablation."""
import json,glob,shutil,csv
from pathlib import Path
H=Path(__file__).resolve().parent;OUT=H/'by_instance';shutil.rmtree(OUT,ignore_errors=True)
SC={'main':'main_table','scal':'scalability','abl':'ablation'};sp=json.load(open(H/'SPEEDS.json'))
ORDER={'main':['Ours','CBTAMP','TP','TP+SIPP','TAS+SIPP','ITAGS'],'scal':['Ours','CBTAMP','TP','TP+SIPP','TAS+SIPP','ITAGS'],'abl':['Ours','No FB','U','Diverse U','U+AD+PD','U+RT']}
CASES={'main':['MP_s6','SP_s3','MT_s0','ST_s1'],'scal':['R4J16_s2','R6J24_s7','R8J32_s8','warehouse_s107'],'abl':['MP_s5','SP_s11','MT_s6','ST_s8']}
rows=list(csv.DictReader(open(H/'INDEX.csv')));md=['# Supplementary animations (simulation, seed 0)','',
'One folder per instance; inside, one mp4 per method. Every video of an instance uses the SAME playback speed-up k (k = ceil of the',
'longest run of that instance / 60 s), so they can be played side by side; the header clock shows simulation time. 20 fps.',
'Success videos replay the adopted trajectory. Failed runs replay the method\'s last candidate (or, for SD+SIPP / ITAGS, the schedule',
'without SIPP) and stop one step before the first collision (centre distance < 2 r) or precedence violation, red marker + banner, 2 s hold.',
'"No plan" videos are 3 s static cards (NO_PATH / NO_SCHEDULE / ITAGS timeout).','']
for sc in ['main','scal','abl']:
    md.append(f'## {SC[sc]}');md.append('')
    for case in CASES[sc]:
        k=sp['speeds'][f'{sc}__{case}'];Tm=sp['T_max'][f'{sc}__{case}'];d=OUT/SC[sc]/case;d.mkdir(parents=True)
        md.append(f'### {case}   (playback x{k}; longest run {Tm:.1f} s)');md.append('');md.append('|method|file|outcome|makespan (s)|stops at (s)|note|');md.append('|---|---|---|---|---|---|')
        for m in ORDER[sc]:
            r=next((r for r in rows if r['scene']==sc and r['case']==case and r['method']==m),None)
            if r is None:continue
            src=H/r['video'];name=m.replace('+','').replace('/','').replace(' ','')+'.mp4';shutil.copy(src,d/name)
            oc={'success':'success','fail_stop':'FAILED (stopped at first violation)','fail_full':'FAILED (full replay, no violation found)','none':'FAILED (no plan)','missing':'missing'}[r['outcome']]
            md.append(f"|{m}|{name}|{oc}|{r['makespan_s'] if r['outcome']=='success' else ''}|{(r['stop_s']+' '+r['stop_kind']) if r['stop_s'] else ''}|{r['note'][:90]}|")
        md.append('')
(OUT/'README.md').write_text('\n'.join(md)+'\n');print('organized',sum(1 for _ in OUT.rglob('*.mp4')),'videos')
