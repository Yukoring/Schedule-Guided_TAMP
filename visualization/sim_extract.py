#!/usr/bin/env python3
"""Trajectory extraction for the supplementary videos (main table, scalability, ablation; seed 0 runs under results/<exp>/runs).

For every (scene, case, method) a traj JSON is written to traj/<scene>__<case>__<method>.json with the environment (bounds,
obstacles, goal regions, precedence, tool), per-robot breakpoints [(x, y, t)], planned events and, for failed runs, the replayed
trajectory truncated at the first violation (collision = centre distance < 2 r, or precedence) plus a hold.
Success: the validated trajectory. Failure with a last candidate: replay stopped one step before the first collision or precedence violation.
TAS+SIPP / ITAGS failures after a schedule: the schedule replayed on the roadmap without SIPP. No plan: static card.
Usage: sim_extract.py [scene ...]   (scenes: main scal abl)"""
import json,math,sys,glob,heapq
from pathlib import Path
import yaml
H=Path(__file__).resolve().parent;R=H.parent;RES=R/'results';OUT=H/'traj';OUT.mkdir(exist_ok=True)
BUDGET=100.0;DT=0.05;HOLD=2.0;TASKS=['taska','taskb','taskc','taskd']
LABEL={'ours':'Ours','no_fb':'No FB','u':'U','diverse_u':'Diverse U','u_ad_pd':'U+AD+PD','u_rt':'U+RT','cbtamp':'CBTAMP','tp':'TP','tp_sipp':'TP+SIPP','tas_sipp':'TAS+SIPP','tas_fb':'TAS+FB','itags':'ITAGS'}
EXP={'main':'main','scal':'scalability','abl':'ablation'}
SCENES={'main':{'cases':['MP_s6','SP_s3','MT_s0','ST_s1'],'methods':['ours','cbtamp','tp','tp_sipp','tas_sipp','itags']},
        'scal':{'cases':['R4J16_s2','R6J24_s7','R8J32_s8','warehouse_s107'],'methods':['ours','cbtamp','tp','tp_sipp','tas_sipp','itags']},
        'abl':{'cases':['MP_s5','SP_s11','MT_s6','ST_s8'],'methods':['ours','no_fb','u','diverse_u','u_ad_pd','u_rt']}}
dist=lambda a,b:math.hypot(a[0]-b[0],a[1]-b[1])
def run_dir(scene,case,m):
    return RES/EXP[scene]/'runs'/f'{EXP[scene].upper()}__{case}__{m}__seed0'
def input_yaml(case):
    for p in [R/'inputs/struct',R/'inputs/scal',R/'inputs/maps']:
        f=p/f'{case}.yaml'
        if f.exists():return f
    raise FileNotFoundError(case)
def walk(path_cost):
    """replay (unit speed): breakpoints [(pos,t)] and event records [(pos,t0,t1,part)]"""
    bps=[];recs=[];t=0.0;prev=None
    for i,ele in enumerate(path_cost):
        if isinstance(ele,list) and len(ele)==3 and isinstance(ele[0],(list,tuple)):
            pos=tuple(ele[0]);t+=dist(prev,pos) if prev is not None else 0.0;bps.append((pos,t))
            for part in ele[2]:recs.append((pos,t,t+part[0],part));t+=part[0];bps.append((pos,t))
            prev=pos
        else:
            pos=tuple(ele);t+=dist(prev,pos) if prev is not None else 0.0;bps.append((pos,t));prev=pos
    return bps,recs
def events_from(recs,robot,ri):
    ev=[]
    for pos,t0,t1,part in recs:
        if len(part)==3 and part[1]==2:kind=part[2]            # pickup_tool / drop_tool
        elif len(part)==2 and part[1]==2:kind='wait'
        else:kind='service'
        e=dict(robot=robot,robot_index=ri,kind=kind,t0=t0,t1=t1,pos=list(pos))
        if kind=='service':e.update(task=part[1],goal=part[2])
        ev.append(e)
    return ev
def env_info(case,tool_pos):
    d=yaml.safe_load(open(input_yaml(case)));e=d['environment'];b=[float(x) for x in str(e['bounds']).split(',')]
    obs=[dict(center=o['center'],length=o['length'],width=o['width'],rotation=o.get('rotation',0)) for o in (e.get('obstacles') or {}).values() if o.get('shape')=='rectangle']
    goals={}
    for g,gd in d['goals']['goal'].items():
        tasks=[s.strip() for s in gd['service'].split(',')]
        goals[g]=dict(corners=gd['corners'],tasks=tasks,order=[tuple(o.split('-')) for o in (gd.get('order') or [])])
    robots={r:dict(start=rd['state'][:2],radius=rd['robot_radius']) for r,rd in d['robots']['robot'].items()}
    tool=None
    if case.split('_s')[0] in ('MT','ST'):
        tool=dict(initial_position=tool_pos,required_for=['taska','taskb'])
    return dict(bounds=b,obstacles=obs,goals=goals,robots=robots,tool=tool)
def sample(bps,T):
    n=int(math.ceil(T/DT))+1;xy=[];j=0
    for k in range(n):
        t=k*DT
        while j+1<len(bps) and bps[j+1][1]<=t:j+=1
        if j+1>=len(bps):xy.append(bps[-1][0]);continue
        (p0,t0),(p1,t1)=bps[j],bps[j+1];a=0.0 if t1-t0<1e-12 else min(1.0,(t-t0)/(t1-t0));xy.append((p0[0]+a*(p1[0]-p0[0]),p0[1]+a*(p1[1]-p0[1])))
    return xy
def first_violation(bpsd,events,env,radius):
    """earliest collision (centre distance < 2r) or precedence violation; returns (t, info) or (None,None)"""
    names=list(bpsd);T=max(b[-1][1] for b in bpsd.values());XY={r:sample(bpsd[r],T) for r in names};n=int(math.ceil(T/DT))+1
    tc=None;ci=None
    for k in range(n):
        for i in range(len(names)):
            for j in range(i+1,len(names)):
                p=XY[names[i]][k];q=XY[names[j]][k]
                if dist(p,q)<2*radius-1e-9:tc=k*DT;ci=dict(kind='collision',pair=[names[i],names[j]],at=[(p[0]+q[0])/2,(p[1]+q[1])/2],distance=dist(p,q));break
            if tc is not None:break
        if tc is not None:break
    tp=None;pi=None
    for g,gd in env['goals'].items():
        for a,b in gd['order']:
            ea=[e for e in events if e['kind']=='service' and e['goal']==g and e['task']==a];eb=[e for e in events if e['kind']=='service' and e['goal']==g and e['task']==b]
            if ea and eb and eb[0]['t0']<ea[0]['t1']-1e-9 and (tp is None or eb[0]['t0']<tp):tp=eb[0]['t0'];pi=dict(kind='precedence',goal=g,task=b,before=a,robot=eb[0]['robot'],at=eb[0]['pos'])
    cands=[(t,i) for t,i in [(tc,ci),(tp,pi)] if t is not None]
    return min(cands) if cands else (None,None)
def truncate(bpsd,t_stop):
    out={}
    for r,b in bpsd.items():
        nb=[(p,t) for p,t in b if t<=t_stop]
        if not nb:nb=[b[0]]
        # position at t_stop
        xy=sample(b,max(t_stop,b[-1][1]));k=min(len(xy)-1,int(round(t_stop/DT)));nb.append((tuple(xy[k]),t_stop));nb.append((tuple(xy[k]),t_stop+HOLD));out[r]=nb
    return out
# ---------- sources ----------
def best_candidate(d):
    best=None
    for f in sorted(glob.glob(str(d/'candidates'/'*.json'))):
        c=json.load(open(f))
        if c.get('valid') and c.get('validated_elapsed_s',1e9)<=BUDGET and (best is None or c['trajectory_makespan']<best['trajectory_makespan']):best=c
    return best
def from_robot_path(rp,names):
    bpsd={};events=[]
    for i,r in enumerate(names):
        p=rp[r];bps,recs=walk(p['path_cost']);bpsd[r]=bps;events+=events_from(recs,r,i)
    return bpsd,events
def dijkstra(graph,pts,s):
    ds={s:0.0};prev={};todo=[(0.0,s)]
    while todo:
        c,u=heapq.heappop(todo)
        if c>ds[u]:continue
        for v in graph[u]:
            nd=c+dist(pts[u],pts[v])
            if nd<ds.get(v,math.inf):ds[v]=nd;prev[v]=u;heapq.heappush(todo,(nd,v))
    return ds,prev
def path_of(prev,s,t):
    p=[t]
    while p[-1]!=s:p.append(prev[p[-1]])
    return p[::-1]
def naive_joint(d):
    inst=json.load(open(d/'joint/instance.json'));sch=json.load(open(d/'joint/schedule.json'))
    # rebuild the task roadmap of the run (same input, PRM seed 0)
    import random,hashlib,numpy as np
    stack=R/'common';sys.path.insert(0,str(stack));sys.path.insert(0,str(stack/'code'))
    from world.environment import Environment
    from world.prm import PRMPlanning
    random.seed(0);np.random.seed(0)
    env_=Environment(str(input_yaml(d.name.split('__')[1])),False);prm=PRMPlanning(env_);samples,graph0,goals0=prm.ConstructPhase(goals_map=env_.goals_map)
    pts=[tuple(x) for x in prm.task_samples];graph=prm.task_roadmap;idx={p:i for i,p in enumerate(pts)}
    G=len(inst['locations']);targets=[idx[tuple(l['position'])] for l in inst['locations']];starts=[idx[tuple(r['start'])] for r in inst['robots']]
    cache={s:dijkstra(graph,pts,s) for s in set(targets+starts)}
    def route(a,b):return [pts[i] for i in (path_of(cache[a][1],a,b) if a!=b else [a])]
    bpsd={};events=[]
    for ri,r in enumerate(inst['robots']):
        acts=sorted((a for a in sch['actions'] if a['robot']==ri),key=lambda a:a['start']);t=0.0;pos=tuple(r['start']);cur=starts[ri];bps=[(pos,0.0)]
        for a in acts:
            if a['kind']=='park':dest_idx=starts[a['goal']-G] if a['goal']>=G else targets[a['goal']]
            else:dest_idx=targets[a['goal']]
            pts_=route(cur,dest_idx);travel=sum(dist(pts_[i],pts_[i+1]) for i in range(len(pts_)-1))
            if a['kind']!='park':
                dep=max(t,a['start']/100.0-travel)
                if dep>t+1e-9:t=dep;bps.append((pos,t))
            for q in pts_[1:]:t+=dist(pos,q);pos=q;bps.append((pos,t))
            cur=dest_idx
            if a['kind']=='park':continue
            s=a['start']/100.0
            if s>t+1e-9:t=s;bps.append((pos,t))
            dur=(a['end']-a['start'])/100.0;task=inst['tasks'][a['task']]
            e=dict(robot=r['name'],robot_index=ri,kind=('service' if a['kind']=='service' else a['kind']+'_tool'),t0=t,t1=t+dur,pos=list(pos))
            if a['kind']=='service':e.update(task=task['type'],goal=inst['goals'][task['goal']]['name'])
            events.append(e);t+=dur;bps.append((pos,t))
        bpsd[r['name']]=bps
    return bpsd,events,sch.get('makespan')
def naive_itags(d,env):
    p=json.load(open(d/'itags_plan.json'));m=json.load(open(d/'itags_mapping.json'));tasks=m['tasks']
    bpsd={};events=[]
    for rp in p['robot_plans']:
        r=rp['name'];ri=rp['robot'];pos=tuple(env['robots'][r]['start']);t=0.0;bps=[(pos,0.0)];trans={tr['to_task']:tr for tr in rp['transitions']}
        for tk in rp['tasks']:
            tr=trans.get(tk);wps=[(w['x'],w['y']) for w in tr['waypoints']] if tr else [pos,tuple(tasks[tk]['loc'])]
            if dist(wps[0],pos)>1e-6:wps=[pos]+wps
            travel=sum(dist(wps[i],wps[i+1]) for i in range(len(wps)-1));s,e_=p['timepoints'][tk]
            dep=max(t,s-travel)
            if dep>t+1e-9:t=dep;bps.append((pos,t))
            for q in wps[1:]:t+=dist(pos,q);pos=q;bps.append((pos,t))
            if s>t+1e-9:t=s;bps.append((pos,t))
            dur=max(e_-s,tasks[tk]['durations_by_robot'].get(r,e_-s))
            events.append(dict(robot=r,robot_index=ri,kind='service',t0=t,t1=t+dur,pos=list(pos),task=tasks[tk]['service'],goal=tasks[tk]['goal']));t+=dur;bps.append((pos,t))
        bpsd[r]=bps
    return bpsd,events
# ---------- main ----------
def extract(scene,case,m):
    d=run_dir(scene,case,m);out=dict(scene=scene,case=case,method=m,label=LABEL[m],run_dir=str(d))
    if not d.exists():out.update(outcome='missing');return out
    names=None;tool_pos=None;pr={}
    if m=='tas_sipp':
        res=json.load(open(d/'joint/result.json'));ok=bool(res.get('solved'));reason=res.get('end_reason')
        inst=json.load(open(d/'joint/instance.json'));names=[r['name'] for r in inst['robots']]
        if case.split('_s')[0] in ('MT','ST'):tool_pos=inst['locations'][inst['tool_initial_location']]['position']
    else:
        res=json.load(open(d/'result_row.json'));ok=bool(res.get('success'));reason=res.get('end_reason')
        pr=json.load(open(d/'pipeline_return.json')) if (d/'pipeline_return.json').exists() else {}
        rm=json.load(open(d/'roadmap.json')) if (d/'roadmap.json').exists() else {};tool_pos=rm.get('tool_initial_position')
    env=env_info(case,tool_pos);radius=list(env['robots'].values())[0]['radius'];names=names or list(env['robots'])
    out.update(env=env,radius=radius,success=ok,end_reason=reason)
    bpsd=events=None;note=''
    if ok:
        if m=='tas_sipp':mo=json.load(open(d/'joint/motion.json'));bpsd,events=from_robot_path(mo['robot_path'],names);out['makespan']=res['makespan']
        else:
            c=best_candidate(d)
            if c is None:out.update(outcome='missing');return out
            bpsd,events=from_robot_path(c['robot_path'],names);out['makespan']=c['trajectory_makespan']
        out['outcome']='success'
    else:
        if m=='tas_sipp' and (d/'joint/schedule.json').exists() and json.load(open(d/'joint/schedule.json')).get('actions'):
            bpsd,events,T_s=naive_joint(d);note=f'schedule replayed without SIPP (T_sched {T_s:.1f} s); SIPP: {reason}'
        elif m=='itags' and (d/'itags_plan.json').exists() and reason=='MOTION_FAILED':
            bpsd,events=naive_itags(d,env);note='ITAGS schedule replayed without SIPP; motion planning failed'
        elif (d/'motion_candidates/last.json').exists():
            c=json.load(open(d/'motion_candidates/last.json'));bpsd,events=from_robot_path(c['robot_path'],names)
            sub=str(pr.get('sub_reason') or '')
            note=f"last candidate (iteration {c.get('iteration')}, {c.get('collision_count')} conflict(s)); {reason}"+(f': {sub[:70]}' if sub else '')
        else:
            out.update(outcome='none',note=f'no plan: {reason}'+((': '+str(pr.get('sub_reason'))[:80]) if pr.get('sub_reason') else ''));return out
        t_stop,info=first_violation(bpsd,events,env,radius)
        if t_stop is None:
            # rejected for another reason (validator); show the whole replay flagged as failed
            T=max(b[-1][1] for b in bpsd.values());out.update(outcome='fail_full',note=note,stop=None,makespan=T)
        else:
            k=max(1,int(t_stop/DT)-1);t_cut=k*DT;bpsd=truncate(bpsd,t_cut);out.update(outcome='fail_stop',note=note,stop=dict(t=t_cut,info=info),makespan=t_cut+HOLD)
            events=[e for e in events if e['t0']<t_cut]
    out['robots']=names;out['breakpoints']={r:[[p[0],p[1],t] for p,t in b] for r,b in bpsd.items()};out['events']=events
    return out
if __name__=='__main__':
    scenes=sys.argv[1:] or list(SCENES)
    for sc in scenes:
        for case in SCENES[sc]['cases']:
            for m in SCENES[sc]['methods']:
                if m=='itags' and case.split('_s')[0] in ('MT','ST'):continue
                o=extract(sc,case,m);f=OUT/f'{sc}__{case}__{m}.json';json.dump(o,open(f,'w'))
                print(sc,case,LABEL[m],o.get('outcome'),('%.1f'%o['makespan']) if o.get('makespan') else '',(o.get('stop') or {}).get('info',{}).get('kind','') if o.get('stop') else '',o.get('note','')[:90],flush=True)
