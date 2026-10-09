#!/usr/bin/env python3
"""Renderer for the supplementary videos: goal regions as teal rounded squares, task
chips (▲ ■ ● ◆) in precedence order with "›" between levels (outside the wall for the structured arena, under the goal on maps),
hammer marker for the tool, robots as coloured discs with index, countdown ring + remaining seconds during a service, pause glyph
during waits, no label after the last task. Failed runs: replay stopped at the first violation, red marker + banner, then a hold.
Playback: dt 0.05 s, 20 fps, speed-up k = ceil(T / 60 s) shown in the header. Usage: sim_render.py traj/<file>.json ... [--dpi 110]"""
import json,sys,math,argparse
from pathlib import Path
import numpy as np
import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle,FancyBboxPatch,Wedge,Polygon
from matplotlib.path import Path as MPath
import matplotlib.transforms as mtr
from matplotlib.animation import FFMpegWriter
H=Path(__file__).resolve().parent;OUT=H/'videos';OUT.mkdir(exist_ok=True)
_h=[(-0.08,-1.0),(0.08,-1.0),(0.08,0.35),(-0.08,0.35),(-0.08,-1.0)];_d=[(-0.65,0.3),(0.65,0.3),(0.65,0.85),(-0.65,0.85),(-0.65,0.3)]
HAMMER=MPath([*_h,*_d],[MPath.MOVETO,MPath.LINETO,MPath.LINETO,MPath.LINETO,MPath.CLOSEPOLY]*2).transformed(mtr.Affine2D().rotate_deg(-35))
COL=['#d62728','#1f77b4','#2ca02c','#ff7f0e','#9467bd','#8c564b','#e377c2','#17becf'];GOAL_FC='#d7efe9';GOAL_EC='#5aa58f';GOAL_ACT='#bfe6d8';CHIP_DONE='#7f7f7f'
LET={'taska':'a','taskb':'b','taskc':'c','taskd':'d'};SHAPE={'a':'▲','b':'■','c':'●','d':'◆'};DT=0.05;FPS=20;HOLD=2.0
def sample(bps,T):
    n=int(round(T/DT))+1;xy=np.zeros((n,2));j=0
    for k in range(n):
        t=k*DT
        while j+1<len(bps) and bps[j+1][2]<=t:j+=1
        if j+1>=len(bps):xy[k]=bps[-1][:2];continue
        p0,p1=bps[j],bps[j+1];a=0.0 if p1[2]-p0[2]<1e-12 else min(1.0,(t-p0[2])/(p1[2]-p0[2]));xy[k]=(p0[0]+a*(p1[0]-p0[0]),p0[1]+a*(p1[1]-p0[1]))
    return xy
def render(f,dpi=110,speed=None):
    d=json.load(open(f));env=d['env'];b=env['bounds'];W,Hh=b[2]-b[0],b[3]-b[1];ext=max(W,Hh);structured=(b[0]<0)   # structured arena is centred at 0
    names=d.get('robots') or list(env['robots']);R=len(names);r=d.get('radius',0.3);r=max(r,0.008*ext);tool=env.get('tool');label=d['label'];case=d['case']
    outcome=d.get('outcome');ev=d.get('events',[])
    if outcome in ('none','missing'):
        T=3.0;XY=np.repeat(np.array([[env['robots'][n]['start'] for n in names]]),int(T/DT)+1,axis=0);stop=None
    else:
        bps=d['breakpoints'];T=max(bp[-1][2] for bp in bps.values());XY=np.stack([sample(bps[n],T) for n in names],axis=1);stop=d.get('stop')
    k_speed=speed or max(1,int(math.ceil((T-(HOLD if stop else 0))/60.0)));step=k_speed
    if outcome in ('none','missing'):T=3.0*k_speed;XY=np.repeat(XY[:1],int(T/DT)+1,axis=0)   # static card: 3 s of video at any speed-up
    # figure: map aspect, header strip on top
    fw=7.2 if W<=Hh*1.15 else min(12.0,7.2*W/Hh);fh=fw*Hh/W*0.92+0.9;PAD=0.06*ext if not structured else 0.75+0.9
    fig=plt.figure(figsize=(fw,fh),dpi=dpi);ax=fig.add_axes([0.02,0.01,0.96,1-0.95/fh]);ax.set_xlim(b[0]-PAD,b[2]+PAD);ax.set_ylim(b[1]-PAD,b[3]+PAD);ax.set_aspect('equal');ax.axis('off')
    ax.add_patch(Rectangle((b[0],b[1]),W,Hh,fill=False,lw=2,color='k',zorder=1))
    for o in env['obstacles']:
        cx,cy=o['center'];L,Wd=o['length'],o['width'];th=math.radians(o.get('rotation',0) or 0)
        ax.add_patch(Rectangle((cx-L/2,cy-Wd/2),L,Wd,angle=math.degrees(th),rotation_point='center',fc='#555555',ec='none',zorder=2))
    chips={};gpatch={};prec={g:[(LET[a],LET[c]) for a,c in gd['order']] for g,gd in env['goals'].items()}
    cs=0.5 if structured else max(0.55,0.028*ext)   # chip size in map units
    for g,gd in env['goals'].items():
        xs=[p[0] for p in gd['corners']];ys=[p[1] for p in gd['corners']];x0,y0,w,h=min(xs),min(ys),max(xs)-min(xs),max(ys)-min(ys);cx,cy=x0+w/2,y0+h/2
        gpatch[g]=FancyBboxPatch((x0,y0),w,h,boxstyle='round,pad=0,rounding_size=%.3f'%(0.08*w),fc=GOAL_FC,ec=GOAL_EC,lw=1.5,zorder=3);ax.add_patch(gpatch[g])
        letters=[LET[t] for t in gd['tasks']];preds={L:[u for u,v in prec[g] if v==L] for L in letters};level={}
        def lv(L):
            if L not in level:level[L]=0 if not preds[L] else 1+max(lv(u) for u in preds[L])
            return level[L]
        order=sorted(letters,key=lambda L:(lv(L),L));gap=cs*0.2;sep=cs*0.45;xs_=[];x=0.0
        for i,L in enumerate(order):
            if i>0 and lv(L)!=lv(order[i-1]):x+=sep
            xs_.append(x);x+=cs+gap
        width=x-gap
        if structured:
            nx,ny=cx,cy;nn=math.hypot(nx,ny);nx,ny=(nx/nn,ny/nn) if nn>1e-9 else (0,1);px_,py_=((1,0) if abs(ny)>=abs(nx) else (0,-1));off=(b[2]-max(abs(cx),abs(cy)))+0.75;base=(cx+nx*off,cy+ny*off)
        else:
            # maps: enlarged goal marker scaled with the map, 2x2 chip grid inside (a b / c d), no rows outside
            sg=max(2.4,0.045*ext);gpatch[g].set(x=cx-sg/2,y=cy-sg/2,width=sg,height=sg,alpha=0.92,zorder=4);gpatch[g].set_boxstyle('round,pad=0,rounding_size=%.3f'%(0.08*sg))
            cs=sg*0.40;gp=sg*0.067;grid={'a':(-1,1),'b':(1,1),'c':(-1,-1),'d':(1,-1)}
            for L in letters:
                gx,gy=grid[L];px,py=cx+gx*(cs/2+gp/2),cy+gy*(cs/2+gp/2)
                box=FancyBboxPatch((px-cs/2,py-cs/2),cs,cs,boxstyle='round,pad=0,rounding_size=%.3f'%(0.15*cs),fc='white',ec='#444444',lw=1.0,zorder=5);ax.add_patch(box)
                txt=ax.text(px,py,SHAPE[L],ha='center',va='center',fontsize=max(6,min(10,9*cs/ext*32)),color='#444444',zorder=6)
                chips[(g,L)]={'box':box,'txt':txt,'pos':(px,py)}
            continue
        for i,L in enumerate(order):
            u=xs_[i]-width/2+cs/2;px,py=base[0]+px_*u,base[1]+py_*u
            box=FancyBboxPatch((px-cs/2,py-cs/2),cs,cs,boxstyle='round,pad=0,rounding_size=%.3f'%(0.15*cs),fc='white',ec='#444444',lw=1.0,zorder=5);ax.add_patch(box)
            txt=ax.text(px,py,SHAPE[L],ha='center',va='center',fontsize=10 if structured else 7,color='#444444',zorder=6)
            if tool and 'task'+L in tool['required_for']:ax.plot(px+cs*0.38,py+cs*0.38,marker=HAMMER,ms=8,mfc='#8c5a2b',mec='k',mew=0.4,ls='none',zorder=7)
            chips[(g,L)]={'box':box,'txt':txt,'pos':(px,py)}
            if i>0 and lv(L)!=lv(order[i-1]):
                u2=(xs_[i-1]+cs+xs_[i])/2-width/2;ax.text(base[0]+px_*u2,base[1]+py_*u2,'›',ha='center',va='center',fontsize=12 if structured else 9,weight='bold',color='#333333',zorder=6,rotation=0 if px_ else -90)
    trails=[ax.plot([],[],'-',color=COL[i%8],lw=1.0,alpha=0.45,zorder=8)[0] for i in range(R)]
    bodies=[Circle((0,0),r,fc=COL[i%8],ec='k',lw=0.8,zorder=10) for i in range(R)]
    for bd in bodies:ax.add_patch(bd)
    rings=[Wedge((0,0),r*1.3,0,0,width=r*0.18,fc=COL[i%8],ec='none',zorder=11,visible=False) for i in range(R)]
    for rg in rings:ax.add_patch(rg)
    fs_lab=9 if ext<=12 else 7;labels=[ax.text(0,0,str(i),fontsize=fs_lab,ha='center',va='center',color='w',weight='bold',zorder=12) for i in range(R)]
    evtext=[ax.text(0,0,'',fontsize=8 if ext<=12 else 7,ha='center',va='bottom',color=COL[i%8],weight='bold',zorder=12,bbox=dict(boxstyle='round,pad=0.15',fc='white',ec=COL[i%8],lw=0.8,alpha=0.9)) for i in range(R)]
    toolmark=ax.plot([],[],marker=HAMMER,ms=20,mfc='#8c5a2b',mec='k',mew=0.8,ls='none',zorder=13)[0] if tool else None
    cmark=Circle((0,0),r*2.2,fill=False,ec='red',lw=3,zorder=14,visible=False);ax.add_patch(cmark)
    hx=fig.add_axes([0.04,1-0.85/fh,0.92,0.8/fh]);hx.axis('off');hx.set_xlim(0,1);hx.set_ylim(0,1)
    scene_name={'main':'Main table','scal':'Scalability','abl':'Ablation'}[d['scene']]
    head=f"{scene_name}   {case}   {label}";sub=''
    if outcome=='success':head+=f"   makespan {d['makespan']:.1f} s";tc='k'
    elif outcome in ('none','missing'):head+="   FAILED";sub=d.get('note','no plan');tc='#b00000'
    elif outcome=='fail_full':head+="   FAILED";sub=d.get('note','');tc='#b00000'
    else:
        info=stop['info'];what='collision' if info['kind']=='collision' else 'precedence violation'
        head+=f"   FAILED — {what} at {stop['t']:.1f} s";sub=d.get('note','');tc='#b00000'
    title=head+(('   '+sub) if sub else '')
    hx.text(0,1.0,head,fontsize=10,weight='bold',va='top',color=tc)
    if sub:
        maxc=int(fw*9.5);sub=sub if len(sub)<=maxc else sub[:maxc-1].rstrip()+'…'   # keep clear of the clock on the right
        hx.text(0,0.62,sub,fontsize=7.5,va='bottom',color='#666666')
    hx.add_patch(Rectangle((0,0.3),1,0.18,fc='#eeeeee',ec='#999999',lw=0.8));bar=Rectangle((0,0.3),0,0.18,fc='#333333',ec='none');hx.add_patch(bar)
    clock=hx.text(1.0,0.62,'',fontsize=9,ha='right',va='bottom',family='monospace');done_t=hx.text(0.0,0.0,'',fontsize=8.5,ha='left',va='bottom',color='#444444');banner=hx.text(1.0,0.0,'',fontsize=9,ha='right',va='bottom',color='red',weight='bold')
    services=[e for e in ev if e['kind']=='service'];ntask=sum(len(gd['tasks']) for gd in env['goals'].values())
    writer=FFMpegWriter(fps=FPS,bitrate=2500,metadata={'title':title});out=OUT/f"{d['scene']}__{case}__{label.replace('+','').replace('/','')}.mp4"
    with writer.saving(fig,str(out),dpi):
        for k in range(0,len(XY),step):
            t=k*DT;xy=XY[k]
            done_set={(e['goal'],LET[e['task']]) for e in services if e['t1']<=t}
            for (g,L),ch in chips.items():
                run=next((e for e in services if e['goal']==g and LET[e['task']]==L and e['t0']<=t<e['t1']),None)
                locked=any((g,u) not in done_set for u,v in prec.get(g,[]) if v==L)
                if (g,L) in done_set:ch['box'].set(fc=CHIP_DONE,ec=CHIP_DONE,ls='-');ch['txt'].set_color('white')
                elif run:ch['box'].set(fc=COL[run['robot_index']%8],ec='k',ls='-');ch['txt'].set_color('white')
                else:ch['box'].set(fc='white',ec='#444444',ls=(0,(2,2)) if locked else '-');ch['txt'].set_color('#444444' if not locked else '#999999')
            active_goals={e['goal'] for e in services if e['t0']<=t<e['t1']}
            for g,pt in gpatch.items():pt.set(fc=GOAL_ACT if g in active_goals else GOAL_FC,lw=3 if g in active_goals else 1.5)
            for i in range(R):
                bodies[i].center=(xy[i,0],xy[i,1]);labels[i].set_position((xy[i,0],xy[i,1]));rings[i].set_center((xy[i,0],xy[i,1]))
                k0=max(0,k-int(8/DT));trails[i].set_data(XY[k0:k+1,i,0],XY[k0:k+1,i,1])
                act=[e for e in ev if e['robot_index']==i and e['t0']<=t<e['t1']]
                above=xy[i,1]<(b[1]+b[3])/2
                if act:
                    e=act[0];rem=e['t1']-t;frac=rem/max(1e-9,e['t1']-e['t0'])
                    if e['kind']=='service':txt=f"{SHAPE[LET[e['task']]]} {rem:.0f}s";rings[i].set(theta1=90-360*frac,theta2=90,visible=True)
                    elif e['kind']=='wait':txt=f"‖ {rem:.0f}s" if rem>=0.5 else '';rings[i].set_visible(False)
                    else:txt='pick up' if e['kind']=='pickup_tool' else 'put down';rings[i].set_visible(False)
                else:
                    rings[i].set_visible(False)
                    txt=''  # no label after the last task
                evtext[i].set_text(txt);evtext[i].set_position((xy[i,0],xy[i,1]+r*1.6) if above else (xy[i,0],xy[i,1]-r*1.6));evtext[i].set_va('bottom' if above else 'top')
            if tool:
                holder=None;pos=tool['initial_position'] or [0,0]
                for e in sorted((e for e in ev if e['kind'] in ('pickup_tool','drop_tool')),key=lambda e:e['t1']):
                    if t>=e['t1']:
                        if e['kind']=='pickup_tool':holder=e['robot_index']
                        else:holder=None;pos=e['pos']
                toolmark.set_data([xy[holder,0]+r*0.55],[xy[holder,1]+r*0.55]) if holder is not None else toolmark.set_data([pos[0]],[pos[1]])
            if stop and t>=stop['t']-1e-9:
                info=stop['info'];cmark.center=tuple(info['at']);cmark.set_visible(True)
                banner.set_text(('COLLISION  robots %s'%' - '.join(str(names.index(n)) for n in info['pair'])) if info['kind']=='collision' else f"PRECEDENCE  {info['goal']}: {SHAPE[LET[info['task']]]} before {SHAPE[LET[info['before']]]} done")
                if info['kind']=='precedence':chips[(info['goal'],LET[info['task']])]['box'].set(ec='red',lw=2.5)
            Tshow=T-(HOLD if stop else 0)
            bar.set_width(min(1,t/max(1e-9,Tshow)));clock.set_text(f"t = {min(t,Tshow):5.1f} s / {Tshow:.1f} s");done_t.set_text(f"tasks done {len(done_set)}/{ntask}")
            if outcome in ('none','missing'):banner.set_text('NO PLAN')
            writer.grab_frame()
    plt.close(fig);return out
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('files',nargs='+');p.add_argument('--dpi',type=int,default=110);p.add_argument('--speed',type=int,default=None,help='playback speed-up (frames skipped); default per video = ceil(T/60)');a=p.parse_args()
    for f in a.files:
        out=render(Path(f),a.dpi,a.speed);print('->',out.name,round(out.stat().st_size/1e6,1),'MB','x%d'%(a.speed or 0),flush=True)
