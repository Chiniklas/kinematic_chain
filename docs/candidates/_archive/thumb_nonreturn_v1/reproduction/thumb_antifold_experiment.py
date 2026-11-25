"""Isolated thumb shape experiments. Never updates the repository candidates."""
import argparse, json, math, sys, time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from thumb_collision_screen import clearances,summarize

ROOT=Path(__file__).resolve().parent
SOURCE=ROOT/'reference_thumb'
sys.path.insert(0,str(SOURCE))
import optimize_direct as model
OUT=ROOT/'generated/thumb_antifold'
OUT.mkdir(parents=True,exist_ok=True)
P=json.loads((SOURCE/'parameters.json').read_text())
X0=np.array(P['direct_synthesis'],float)
# Keep all ground pivots A,D,E and the original human/cuff geometry fixed.
FREE=np.array([0,1,2,3,6,7,8,9,10])
LOW=np.array([8,-.5,-35,-15,-30,-20,-35,-30,.15])
HIGH=np.array([32,3.6,30,42,95,95,95,95,1.55])
SCALE=np.array([8,.6,15,15,35,35,35,35,.4])
GRID=np.unique(np.r_[np.linspace(0,1,61),np.linspace(0,.12,41)])
EARLY=GRID<=.25
NAMES=['order','angle','compact','combined']

def unpack(z):
    x=X0.copy();x[FREE]=z;return x

def geometry(x,s):
    raw,viol=model.poses(x,s)
    world={k:v@model.R for k,v in raw.items()}
    m,ip,r=model.ik(raw['h'])
    return world,viol,m,ip,r

def shape_metrics(x,points):
    u=points['f'][0]-points['g'][0];v=points['h'][0]-points['g'][0]
    cos=float(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)))
    angle=math.degrees(math.acos(np.clip(cos,-1,1)))
    return cos,angle

def order_values(points,s):
    # Near extension, compare F/G against H along MCP->H, not screen left/right.
    axis=points['h']-np.array([0.,-7.]);axis/=np.linalg.norm(axis,axis=1)[:,None]
    early=np.asarray(s)<=.25+1e-12
    return np.concatenate([np.sum((points[k]-points['h'])*axis,axis=1)[early] for k in ['f','g']])

def envelope_values(points):
    arr=np.stack(list(points.values()))
    return np.concatenate([(arr[:,:,0]-65).ravel(),(-40-arr[:,:,0]).ravel(),(arr[:,:,1]-50).ravel(),(-65-arr[:,:,1]).ravel()])

def residual(z,variant,strength=1.):
    x=unpack(z);pts,viol,m,ip,r=geometry(x,GRID)
    raw_h=pts['h']@model.R.T
    err=raw_h-model.target(GRID)
    err[-1]*=80
    lengths=np.array(list(model.lengths(x).values()))
    pieces=[err.ravel(),(m-(2+43*GRID))*.10,(ip-(3+57*GRID))*.10,
            viol*2,np.maximum(r-49.995,0)*80,np.maximum(10.01-r,0)*80,
            np.minimum(np.diff(m)/np.diff(GRID)-.2,0)*2,np.minimum(np.diff(ip)/np.diff(GRID)-.2,0)*2,
            np.maximum(m-45,0)*8,np.maximum(ip-60,0)*8,
            np.maximum(8.1-lengths,0)*20,np.maximum(lengths-84.9,0)*20]
    # Near full extension the IK is sensitive: enforce the actual initial
    # derivative as well as finite differences on the collocation grid.
    _,_,mi,ii,_=geometry(x,np.array([0.,1e-5]))
    pieces.append(np.array([min((mi[1]-mi[0])/1e-5-1.,0),min((ii[1]-ii[0])/1e-5-1.,0)])*3)
    # Keep intersection triangles away from tangency; signed branches validated later.
    for a,b,c in [('b','c','d'),('e','f','g')]:
        u=pts[a]-pts[b];v=pts[c]-pts[b]
        sine=np.abs(u[:,0]*v[:,1]-u[:,1]*v[:,0])/(np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1)+1e-12)
        pieces.append(np.maximum(.06-sine,0)*30)
    small={k:v[::4] for k,v in pts.items()}
    clearance,_=clearances(small,m[::4],ip[::4],guidance=True)
    pieces.append(np.maximum(.3-clearance,0).ravel()*8)
    cos,angle=shape_metrics(x,pts)
    pieces.append(np.array([max(cos-math.cos(math.radians(15.5)),0)*80,max(-.985-cos,0)*80]))
    if variant in ['order','combined']:
        # Aim slightly inside the 2 mm final gate.
        pieces.append(np.maximum(order_values(pts,GRID)-1.5,0)*10*strength)
    if variant in ['angle','combined']:
        pieces.append(np.array([max(cos+.01,0)*80*strength,max(-.985-cos,0)*80*strength]))
    if variant in ['compact','combined']:
        pieces.append(np.maximum(envelope_values(pts)+.2,0)*10*strength)
        # This is a soft objective, separate from the envelope gates.
        moving=np.array([v for k,v in model.lengths(x).items() if k not in ['L_ad','L_ae','L_de']])
        pieces.append(moving*.012)
    return np.concatenate(pieces)

def evaluate(x,variant,n=1001):
    s=np.linspace(0,1,n);pts,viol,m,ip,r=geometry(x,s)
    ls=model.lengths(x)
    errors=[np.max(np.abs(np.linalg.norm(pts[k[2]]-pts[k[3]],axis=1)-v)) for k,v in ls.items()]
    cosine,angle=shape_metrics(x,pts)
    axis_over=float(np.max(order_values(pts,s)))
    env=max(0.,float(np.max(envelope_values(pts))))
    target=model.target(s)@model.R
    track=np.linalg.norm(pts['h']-target,axis=1)
    root=np.tile([0.,-7.],(n,1));mr=np.radians(m);ir=np.radians(ip)
    joint=root+30*np.stack([np.cos(mr),-np.sin(mr)],axis=1)
    tip=joint+40*np.stack([np.cos(mr+ir),-np.sin(mr+ir)],axis=1)
    closure=float(np.max(np.linalg.norm(pts['h']-(joint+tip)/2,axis=1)))
    signs=[];minsin=[]
    for a,b,c in [('b','c','d'),('e','f','g')]:
        u=pts[a]-pts[b];v=pts[c]-pts[b]
        sine=(u[:,0]*v[:,1]-u[:,1]*v[:,0])/(np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1)+1e-12)
        signs.append(bool(np.all(sine>0) or np.all(sine<0)));minsin.append(float(np.min(np.abs(sine))))
    maxlen=max(ls.values());minlen=min(ls.values())
    checks=dict(assembly=float(np.max(viol))<1e-8,rigid_lengths=max(errors)<1e-6,
       direct_closure=closure<1e-6,endpoint=max(abs(m[-1]-45),abs(ip[-1]-60))<=.25,
       trajectory=float(max(track))<=1.0,monotone_mcp=float(np.min(np.diff(m)))>=-1e-6,
       monotone_ip=float(np.min(np.diff(ip)))>=-1e-6,continuous_branches=all(signs),
       intersection_margin=min(minsin)>=.04,length_bounds=minlen>=7.999 and maxlen<=85.001,
       rigid_shape_margin=15<=angle<=170)
    if variant in ['order','combined']:checks['node_order']=axis_over<=2.0
    if variant in ['angle','combined']:checks['FGH_angle']=90<=angle<=170
    if variant in ['compact','combined']:checks['envelope']=env<=.01
    collision=summarize(pts,m,ip)
    checks['layered_clearance_screen']=collision['minimum_clearance_mm']>=.01
    checks={k:bool(v) for k,v in checks.items()}
    arr=np.stack(list(pts.values()))
    result=dict(variant=variant,passed=all(checks.values()),checks=checks,
      x=x.tolist(),lengths_mm=ls,FGH_angle_deg=angle,early_FG_overshoot_mm=axis_over,
      envelope_excess_mm=env,trajectory_max_error_mm=float(max(track)),trajectory_rms_mm=float(np.sqrt(np.mean(track**2))),
      endpoint_deg=[float(m[-1]),float(ip[-1])],motor_travel_deg=float(np.degrees(x[-1])),
      minimum_mcp_step_deg=float(np.min(np.diff(m))),minimum_ip_step_deg=float(np.min(np.diff(ip))),
      minimum_intersection_sine=min(minsin),maximum_closure_mm=closure,maximum_dimension_error_mm=max(errors),
      moving_length_sum_mm=sum(v for k,v in ls.items() if k not in ['L_ad','L_ae','L_de']),
      bounds_mm={'x':[float(arr[:,:,0].min()),float(arr[:,:,0].max())],'y':[float(arr[:,:,1].min()),float(arr[:,:,1].max())]},samples=n)
    result['collision_screen']=collision
    return result

def ranking(r):
    failure=sum(not v for v in r['checks'].values())
    loss=r['trajectory_rms_mm']+.001*r['moving_length_sum_mm']
    if r['variant'] in ['order','combined']:loss+=max(r['early_FG_overshoot_mm']-2,0)*2
    if r['variant'] in ['angle','combined']:loss+=max(90-r['FGH_angle_deg'],0)*.2
    if r['variant'] in ['compact','combined']:loss+=r['envelope_excess_mm']
    return (not r['passed'],failure,loss)

def run(variant,trials,max_nfev):
    rng=np.random.default_rng(20261116+NAMES.index(variant))
    results=[];best=None
    saved=OUT/(variant+'_search.json')
    if saved.exists():
        results=json.loads(saved.read_text())['trials']
        results=[dict(r,**evaluate(np.array(r['x']),variant)) for r in results]
        best=min(results,key=ranking)
    for number in range(len(results),trials):
        if number==0:z=X0[FREE].copy()
        elif number%4==0:
            z=rng.uniform(LOW,HIGH);z[1]=rng.uniform(.8,2.5);z[-1]=rng.uniform(.5,1.3)
        else:
            seed=np.array(best['x']) if best else X0.copy()
            # Also reuse other already found shape families as alternate starts.
            others=[p for p in OUT.glob('*_search.json') if p!=saved]
            if number%3==0 and others:
                other=json.loads(rng.choice(others).read_text())
                seed=np.array(other['best']['x'])
            z=seed[FREE]+rng.normal(size=len(FREE))*SCALE*(.05 if number%2 else .5)
        z=np.clip(z,LOW+1e-6,HIGH-1e-6)
        began=time.time()
        fit=least_squares(lambda v:residual(v,variant),z,bounds=(LOW,HIGH),max_nfev=max_nfev,
                          x_scale='jac',ftol=2e-8,xtol=2e-8,gtol=2e-8)
        candidate=evaluate(unpack(fit.x),variant)
        candidate.update(trial=number,nfev=int(fit.nfev),seconds=time.time()-began,residual_norm=float(np.linalg.norm(fit.fun)))
        results.append(candidate);best=min(results,key=ranking)
        saved.write_text(json.dumps({'best':best,'trials':results},indent=2),encoding='utf-8')
        print(variant,number,'PASS' if candidate['passed'] else 'FAIL',
              'angle',round(candidate['FGH_angle_deg'],2),'over',round(candidate['early_FG_overshoot_mm'],2),
              'err',round(candidate['trajectory_max_error_mm'],3),
              'failed',[k for k,v in candidate['checks'].items() if not v],flush=True)
    return best

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--variant',choices=NAMES+['all'],default='all');parser.add_argument('--trials',type=int,default=12);parser.add_argument('--nfev',type=int,default=800)
    args=parser.parse_args()
    (OUT/'baseline.json').write_text(json.dumps(evaluate(X0,'baseline'),indent=2))
    for name in NAMES if args.variant=='all' else [args.variant]:run(name,args.trials,args.nfev)
