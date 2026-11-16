"""Hard-parameterized non-return output experiments; isolated from repo/CAD."""
import argparse,json,time,math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
import thumb_vibration_preview as v
from thumb_collision_screen import clearances,summarize
t=v.t; model=t.model
ROOT=t.ROOT; OUT=ROOT/'generated/thumb_no_return';OUT.mkdir(parents=True,exist_ok=True)
WORK=ROOT/'work/thumb_no_return';WORK.mkdir(parents=True,exist_ok=True)
GRID=np.unique(np.r_[np.linspace(0,1,51),np.linspace(0,.1,11)])
REF=t.geometry(t.X0,GRID)[0];HGREF=REF['g']-REF['h'];HGREF/=np.linalg.norm(HGREF,axis=-1)[:,None]
U0=(model.setup(t.X0)[3]-model.H0);PHI0=np.arctan2(U0[1],U0[0])
# z: AB, phase, Cx,Cy, Ex,Ey, GH, H->G angle, FG, bend, travel, Dx,Dy.
NAMES=['straight','gentle','relayout']
def setup(z):
    ab,phase,cx,cy,ex,ey,gh,phi,fg,bend,travel,dx,dy=z
    a=model.A;d=np.array([dx,dy]);b=a+ab*np.array([np.cos(phase),np.sin(phase)])
    c=np.array([cx,cy]);e=np.array([ex,ey]);g=model.H0+gh*np.array([np.cos(phi),np.sin(phi)])
    f=g+fg*np.array([np.cos(phi+bend),np.sin(phi+bend)])
    return dict(a=a,b=b,c=c,d=d,e=e,f=f,g=g,h=model.H0)

def geometry(z,s):
    p=setup(z);a,b,c,d,e,f,g,h=[p[k] for k in 'abcdefgh'];ab,phase=z[:2]
    bc=np.linalg.norm(c-b);cd=np.linalg.norm(c-d);ef=np.linalg.norm(f-e);fg=z[8]
    bs=a+ab*np.stack([np.cos(phase-z[10]*s),np.sin(phase-z[10]*s)],-1)
    cs,vi1=model.circle(bs,bc,d,cd,np.sign(model.cross(d-b,c-b)))
    co=((cs-d)@(c-d))/(cd*cd+1e-20);si=model.cross(c-d,cs-d)/(cd*cd+1e-20)
    gs=d+co[...,None]*(g-d)+si[...,None]*model.perp(g-d)
    fs,vi2=model.circle(e,ef,gs,fg,np.sign(model.cross(g-e,f-e)))
    gf=f-g;hs=h-g;u=fs-gs
    hp=gs+(hs@gf)/(gf@gf)*u+model.cross(gf,hs)/(gf@gf)*model.perp(u)
    raw=dict(a=np.broadcast_to(a,bs.shape),b=bs,c=cs,d=np.broadcast_to(d,bs.shape),e=np.broadcast_to(e,bs.shape),f=fs,g=gs,h=hp)
    pts={k:p@model.R for k,p in raw.items()};m,ip,r=model.ik(hp)
    return pts,np.r_[vi1,vi2],m,ip,r

def lengths(z):
    p=setup(z)
    return {'L_'+k:float(np.linalg.norm(p[k[0]]-p[k[1]])) for k in ['ab','bc','cd','ad','ae','de','cg','dg','ef','fg','gh','fh']}

def free_bounds(kind,sign=1):
    lo=np.array([8,-1.5,-55,-35,t.X0[4]-5,t.X0[5]-5,8,PHI0-np.deg2rad(14.9),8,-np.pi/6,.15,0,0.],float)
    hi=np.array([40,4.5,65,75,t.X0[4]+5,t.X0[5]+5,65,PHI0+np.deg2rad(14.9),65,np.pi/6,1.8,0,0.],float)
    free=[0,1,2,3,4,5,6,7,8,10]
    if kind=='gentle':free+=[9]
    if kind=='relayout':
        lo[4:6]=[-45,-35];hi[4:6]=[65,75];lo[11:13]=[-25,-25];hi[11:13]=[25,25];free+=[11,12]
        if sign<0:lo[10]=-1.8;hi[10]=-.15
    return np.array(free),lo,hi

def residual(z,kind,weight=.003):
    pts,viol,m,ip,r=geometry(z,GRID);ls=np.array(list(lengths(z).values()))
    err=pts['h']-model.target(GRID)@model.R
    penalties=[err.ravel()*2,err[-1]*80,np.array([m[-1]-45,ip[-1]-60])*50,
      viol*4,np.maximum(r-49.995,0)*120,np.maximum(10.01-r,0)*120,
      np.minimum(np.diff(m)/np.diff(GRID)-.1,0)*3,np.minimum(np.diff(ip)/np.diff(GRID)-.1,0)*3,
      np.maximum(m-45,0)*10,np.maximum(ip-60,0)*10,
      np.maximum(8.1-ls,0)*35,np.maximum(ls-84.8,0)*35]
    _,_,mi,ii,_=geometry(z,np.array([0.,1e-5]));penalties.append(np.minimum(np.array([mi[1]-mi[0],ii[1]-ii[0]])/1e-5-.5,0)*3)
    hg=pts['g']-pts['h'];hgu=hg/np.linalg.norm(hg,axis=-1)[:,None];_,normal=v.axes(m,ip)
    penalties += [np.maximum(np.cos(np.deg2rad(24))-np.sum(hgu*HGREF,axis=-1),0)*150,
       np.maximum(3-np.sum(hg*normal,axis=-1),0)*20,np.array([max(3-hg[0,0],0)])*20]
    for a,b,c in [('b','c','d'),('e','f','g')]:
        u=pts[a]-pts[b];w=pts[c]-pts[b];sine=np.abs(model.cross(u,w))/(np.linalg.norm(u,axis=-1)*np.linalg.norm(w,axis=-1)+1e-20)
        penalties.append(np.maximum(.06-sine,0)*80)
    ix=np.unique(np.r_[np.arange(0,len(GRID),2),len(GRID)-1]);small={k:p[ix] for k,p in pts.items()}
    gaps,_=clearances(small,m[ix],ip[ix]);reserve,_=v.reserve_clearances(small,m[ix],ip[ix],guidance=True)
    penalties += [np.maximum(.3-gaps,0).ravel()*30,np.maximum(2.2-reserve,0).ravel()*20]
    if kind!='relayout':penalties.append(np.array([max(np.linalg.norm(z[4:6]-t.X0[4:6])-4.9,0)])*50)
    else:penalties += [np.array([max(np.linalg.norm(z[11:13])-25,0),max(np.linalg.norm(z[4:6]-t.X0[4:6])-50,0)])*50]
    penalties += [ls*weight]
    return np.concatenate(penalties)

def evaluate(z,kind,n=5001):
    s=np.linspace(0,1,n);pts,viol,m,ip,r=geometry(z,s);ls=lengths(z)
    target=model.target(s)@model.R;track=np.linalg.norm(pts['h']-target,axis=-1)
    dim=max(float(np.max(np.abs(np.linalg.norm(pts[k[2]]-pts[k[3]],axis=-1)-length))) for k,length in ls.items())
    signs=[];mins=[]
    for a,b,c in [('b','c','d'),('e','f','g')]:
        u=pts[a]-pts[b];w=pts[c]-pts[b];si=model.cross(u,w)/(np.linalg.norm(u,axis=-1)*np.linalg.norm(w,axis=-1)+1e-20)
        signs.append(bool(np.all(si>0) or np.all(si<0)));mins.append(float(np.min(abs(si))))
    ref=t.geometry(t.X0,s)[0];hg=pts['g']-pts['h'];dev=v.angular_deviation(hg,ref['g']-ref['h']);_,normal=v.axes(m,ip)
    a=pts['f']-pts['g'];b=pts['h']-pts['g'];angle=v.angular_deviation(a,b)
    col=summarize(pts,m,ip);res,names=v.reserve_clearances(pts,m,ip);ri,rj=np.unravel_index(np.argmin(res),res.shape)
    mr=np.radians(m);ir=np.radians(ip);joint=np.array([0,-7])+30*np.stack([np.cos(mr),-np.sin(mr)],-1)
    expected=joint+20*np.stack([np.cos(mr+ir),-np.sin(mr+ir)],-1);closure=float(np.max(np.linalg.norm(pts['h']-expected,axis=-1)))
    es=float(np.linalg.norm(z[4:6]-t.X0[4:6]));ds=float(np.linalg.norm(z[11:13]));
    checks=dict(assembly=float(max(viol))<1e-8,rigid_lengths=dim<1e-6,direct_closure=closure<1e-6,
      endpoint=max(abs(m[-1]-45),abs(ip[-1]-60))<=.25,trajectory=max(track)<=1,
      monotone_mcp=min(np.diff(m))>=-1e-6,monotone_ip=min(np.diff(ip))>=-1e-6,
      continuous_branches=all(signs),intersection_margin=min(mins)>=.04,
      length_bounds=min(ls.values())>=7.999 and max(ls.values())<=85.001,
      no_return=float(min(angle))>=(150-1e-5 if kind=='gentle' else 180-1e-5),
      initial_HG_direction=dev[0]<=15 and hg[0,0]>=2.99,
      all_pose_HG_direction=max(dev)<=25 and min(np.sum(hg*normal,axis=-1))>=2.99,
      vibration_and_strap_reserve=float(res.min())>=2-1e-6,
      layered_clearance=col['minimum_clearance_mm']>=.1,
      ground_bounds=es<=(50.001 if kind=='relayout' else 5.001) and ds<=(25.001 if kind=='relayout' else 1e-7))
    checks={k:bool(value) for k,value in checks.items()};arr=np.stack(list(pts.values()))
    return dict(kind=kind,z=z.tolist(),passed=all(checks.values()),checks=checks,samples=n,
      lengths_mm=ls,FGH_angle_deg=float(angle[0]),minimum_FGH_angle_deg=float(min(angle)),
      trajectory_max_error_mm=float(max(track)),trajectory_rms_mm=float(np.sqrt(np.mean(track**2))),
      endpoint_deg=[float(m[-1]),float(ip[-1])],motor_travel_deg=float(np.degrees(z[10])),
      maximum_closure_mm=closure,maximum_dimension_error_mm=dim,minimum_intersection_sine=min(mins),
      minimum_mcp_step_deg=float(min(np.diff(m))),minimum_ip_step_deg=float(min(np.diff(ip))),
      initial_HG_deviation_deg=float(dev[0]),maximum_HG_deviation_deg=float(max(dev)),
      collision_screen=col,reserve_clearance_mm=float(res.min()),reserve_worst_pair=names[ri],reserve_worst_index=int(rj),
      ground_E_shift_mm=es,ground_D_shift_mm=ds,
      physical_moving_link_sum_mm=sum(ls['L_'+k] for k in ['ab','bc','cd','cg','ef','fg','gh']),
      bounds_mm={axis:[float(arr[:,:,i].min()),float(arr[:,:,i].max())] for i,axis in enumerate(['x','y'])})

def rank(r):
    # Feasible first, then minimize continuous residual to escape near-feasible traps.
    return (not r['passed'],r.get('residual_norm',1e9) if not r['passed'] else r['physical_moving_link_sum_mm'])

def run(kind,trials,nfev):
    path=WORK/(kind+'_search.json');results=json.loads(path.read_text()) if path.exists() else []
    rng=np.random.default_rng(20261118+NAMES.index(kind))
    for i in range(len(results),trials):
        sign=-1 if kind=='relayout' and i%4==3 else 1
        free,lo,hi=free_bounds(kind,sign)
        if kind=='gentle' and 15<=i<18 and (OUT/'straight.json').exists():
            z=np.array(json.loads((OUT/'straight.json').read_text())['z']);z[9]=[0,-.15,.15][i-15]
        elif kind!='relayout' and 9<=i<12 and (OUT/'relayout.json').exists():
            z=np.array(json.loads((OUT/'relayout.json').read_text())['z'])
            if kind=='gentle':z[9]=[0,-.5,.5][i-9]
            if i>9:z[6]+=5*(i-9)
        elif results and i%3!=0:
            eligible=[r for r in results if np.sign(r['z'][10])==sign]
            z=np.array(min(eligible or results,key=rank)['z'])
            z+=rng.normal(size=13)*np.array([2,.18,5,5,3,3,5,.07,5,.1,.12,3,3])*(.3 if i%3==1 else 1.)
        else:
            gh=[20,30,12,40][i%4];fg=[16,28,35,12][i%4]
            z=np.r_[t.X0[:6],gh,PHI0,fg,0,sign*.9,0,0]
            z[0]=rng.uniform(12,28);z[1]=rng.uniform(1.,2.7);z[2:4]=[rng.uniform(-8,20),rng.uniform(12,28)]
            if kind=='gentle':z[9]=rng.uniform(-.5,.5)
            if kind=='relayout':z[4:6]+=[rng.uniform(-10,20),rng.uniform(0,25)];z[11:13]=rng.uniform(-10,10,2)
        if kind!='gentle':z[9]=0
        if kind!='relayout':z[11:13]=0
        z[free]=np.clip(z[free],lo[free]+1e-7,hi[free]-1e-7)
        def unpack(q):
            full=z.copy();full[free]=q;return full
        start=time.time();fit=least_squares(lambda q:residual(unpack(q),kind),z[free],bounds=(lo[free],hi[free]),
          x_scale='jac',max_nfev=nfev,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        rec=evaluate(unpack(fit.x),kind,n=1001);rec.update(trial=i,seconds=time.time()-start,nfev=fit.nfev,residual_norm=float(np.linalg.norm(fit.fun)))
        results.append(rec);path.write_text(json.dumps(results,indent=2),encoding='utf-8')
        best=min(results,key=rank);(OUT/(kind+'.json')).write_text(json.dumps(best,indent=2),encoding='utf-8')
        print(kind,i,rec['passed'],'loss',round(rec['residual_norm'],3),'angle',round(rec['FGH_angle_deg'],2),'error',round(rec['trajectory_max_error_mm'],3),'gap',round(rec['collision_screen']['minimum_clearance_mm'],3),'fails',[k for k,value in rec['checks'].items() if not value],flush=True)
    best=min(results,key=rank);checked=evaluate(np.array(best['z']),kind);checked.update(trials=len(results),selected_trial=best['trial'],residual_norm=best['residual_norm'])
    (OUT/(kind+'.json')).write_text(json.dumps(checked,indent=2),encoding='utf-8')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('kind',choices=NAMES);p.add_argument('--trials',type=int,default=9);p.add_argument('--nfev',type=int,default=650);args=p.parse_args();run(args.kind,args.trials,args.nfev)
