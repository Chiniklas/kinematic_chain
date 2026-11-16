"""Joint length/phase optimization; fixed A,D,E installation, unchanged upstream code.

Direct collocation is used to propose designs; upstream crank-driven evaluation
and denser continuation validate them independently afterward.
"""
from pathlib import Path
from dataclasses import replace, asdict
import argparse, copy, json, math, sys, time
import numpy as np
import yaml
from scipy.optimize import least_squares

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'work/kinematic_chain/src/co-optimization'))
import run_adam as m
ORIGINAL=m._mechanism_workspace_poses
OUT=ROOT/'outputs/finger_reoptimization'

def load(finger):
    profile='short_ad' if finger=='ring' else 'long_ad'
    cfg=ROOT/'work/kinematic_chain/src/co-optimization/config'/profile
    p=m.load_problem(cfg/'objectives.yaml',cfg/'optimizable_variables.yaml')
    p=replace(p,objectives=tuple(o for o in p.objectives if o.finger==finger))
    c=yaml.safe_load((ROOT/f'outputs/finger_optimization/{finger}_{profile}/fingers/{finger}/candidate_0001/candidate.yaml').read_text(encoding='utf-8'))
    vals=np.array([c['candidate_variables'][v.id]['value'] for v in p.variables])
    return p,vals

def anchored(lengths,qs,tilt,phase=0):
    qs=np.asarray(qs,dtype=float)
    if not len(qs):return ()
    first=float(qs[0]+phase)
    prefix=np.linspace(0,first,max(2,int(math.ceil(abs(first)/.25))+1))[:-1] if abs(first)>1e-12 else np.array([])
    # Track every input interval densely, including within coarse collocation.
    # Large jumps can otherwise select the other circle-intersection branch.
    track=list(prefix);keep=[]
    for value in qs+phase:
        if track:
            track.extend(np.linspace(track[-1],value,max(1,int(math.ceil(abs(value-track[-1])/.25)))+1)[1:].tolist())
        else:track.append(float(value))
        keep.append(len(track)-1)
    poses=ORIGINAL(lengths,np.array(track),tilt)
    if len(poses)<len(prefix):return ()
    return tuple((float(q),poses[i][1]) for q,i in zip(qs,keep) if i<len(poses))

def clearances(p,pos,hand,r4,with_names=False):
    c=p.component_config['hand_mechanism_non_collision'];rows=[]
    for name,a,b in m._mechanism_segments(p,pos,c['dorsal_mount_exclusion_radius_mm']):
        for hn,hs,he,hr in hand:
            rows.append((name,hn,m._segment_distance(a,b,hs,he)-c['mechanism_link_radius_mm']-hr))
    for name,poly in m._mechanism_polygons(p,pos,c['dorsal_mount_exclusion_radius_mm']):
        for hn,hs,he,hr in hand:
            rows.append(('body:'+name,hn,m._segment_polygon_signed_distance(hs,he,poly,c['link_axis_samples'])-c['mechanism_link_radius_mm']-hr))
    rod=m._trim_segment_endpoint(pos['h'],r4,c['r4_contact_exclusion_radius_mm'],False)
    for hn,hs,he,hr in hand:
        rows.append(('output_rod',hn,m._segment_distance(*rod,hs,he)-c['output_rod_radius_mm']-hr if rod else 100.0))
    return rows if with_names else np.array([r[2] for r in rows])

def evaluate(p,vals,phase,n=301):
    comps=copy.deepcopy(p.component_config);cons=copy.deepcopy(p.constraint_config)
    comps['task_space_reachability']['curled_input_samples']=901
    for k in ['hand_mechanism_non_collision','output_link_perpendicularity']:comps[k]['evaluation_poses']=n
    for k in ['closure_error','hand_motion_admissibility']:cons[k]['evaluation_poses']=n
    dense=replace(p,component_config=comps,constraint_config=cons)
    m._mechanism_workspace_poses=lambda lengths,qs,tilt_deg=0:anchored(lengths,qs,tilt_deg,phase)
    try:ev=m.evaluate(dense,vals)
    finally:m._mechanism_workspace_poses=ORIGINAL
    return ev

def run(finger,max_nfev=180,resume=False):
    dest=OUT/finger;dest.mkdir(parents=True,exist_ok=True)
    p,base=load(finger);o=p.objectives[0];base_ev=m.evaluate(p,base)
    indices=[i for i,v in enumerate(p.variables) if v.id not in ['L_ae','L_de']]
    ids=[p.variables[i].id for i in indices];nvar=len(indices)
    bounds_lo=np.array([p.variables[i].minimum for i in indices]);bounds_hi=np.array([p.variables[i].maximum for i in indices])
    for j,variable_id in enumerate(ids):
        if variable_id in ['L_cd','L_dg']:
            # Do not exploit upstream behavior which drops an entire rigid-body
            # polygon when an edge fits inside the 12mm mount exclusion.
            bounds_lo[j]=max(bounds_lo[j],p.component_config['hand_mechanism_non_collision']['dorsal_mount_exclusion_radius_mm']+.5)
    startphase=-2.1789087011401636 if finger=='ring' else 0.0
    ev=evaluate(p,base,startphase,31)
    saved=json.loads((dest/'trials.json').read_text(encoding='utf-8')) if resume else []
    begin=time.time()
    def optimize(start,qs,ns,shorten,tag):
        s=np.linspace(0,1,ns)
        hands=[m._hand_capsules(p,o,float(si)) for si in s]
        contacts=[m._hand_distal_contact_progress(p,o,float(si)) for si in s]
        part_ids=[d['id'] for d in p.nominal_design['dimensions'] if d['body']!='ground']
        part_ids+=['body:'+b['id'] for b in p.nominal_design['bodies'] if b['kind']!='ground' and len(b['nodes'])>=3]
        part_ids+=['output_rod']
        pair_ids=[(part,hn) for part in part_ids for hn,_,_,_ in hands[0]]
        x0=np.r_[start[indices],qs]
        lo=np.r_[bounds_lo,[-15.0]*ns];hi=np.r_[bounds_hi,[105.0]*ns];hi[nvar]=15.0
        state={'calls':0,'best':float('inf'),'last_print':0.0}
        residual_size=None
        def fun(x):
            nonlocal residual_size
            vals=base.copy();vals[indices]=x[:nvar];q=x[nvar:];lengths=m._candidate_lengths(p,vals)
            poses=anchored(lengths,q,p.ad_tilt_deg)
            if len(poses)!=ns:
                return np.full(residual_size,1e3+np.linalg.norm(x-x0))
            clos=np.array([math.dist(pos['h'],r4)-lengths['L_tip_rod'] for (_,pos),r4 in zip(poses,contacts)])
            pose_clearances=[]
            for (_,pos),h,r4 in zip(poses,hands,contacts):
                pair_values={(a,b):value for a,b,value in clearances(p,pos,h,r4,True)}
                # The upstream mount trim can omit a short edge/polygon. Keep
                # residual layout stable, while retaining the same exemptions.
                pose_clearances.append([pair_values.get(pair,100.) for pair in pair_ids])
            cl=np.array(pose_clearances).ravel()
            # Fixed 1mm design margin; actual upstream rejection remains 0mm.
            r=np.r_[clos*10,np.minimum(cl-1.0,0)*4,np.minimum(np.diff(q)-.01,0)*10,
                    max(q[-1]-q[0]-90,0)*10,(x[:nvar]-base[indices])*.003,
                    x[:nvar]*shorten]
            residual_size=len(r);state['calls']+=1
            cost=float(r@r)
            if cost<state['best']:
                state['best']=cost;state['bestx']=x.copy()
            if time.time()-state['last_print']>25:
                print(json.dumps(dict(finger=finger,stage=tag,calls=state['calls'],cost=cost,closure=float(np.max(abs(clos))),clearance=float(min(cl)),phase=float(q[0]),elapsed_s=time.time()-begin)),flush=True)
                state['last_print']=time.time()
            return r
        fun(x0)
        result=least_squares(fun,x0,bounds=(lo,hi),x_scale='jac',max_nfev=max_nfev,ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
        vals=base.copy();vals[indices]=result.x[:nvar];q=result.x[nvar:]
        validation=evaluate(p,vals,float(q[0]))
        record=dict(tag=tag,solver='scipy.optimize.least_squares / direct collocation',nfev=result.nfev,calls=state['calls'],solver_message=result.message,shortening_weight=shorten,collocation_samples=ns,values=vals.tolist(),lengths_mm=m._candidate_lengths(p,vals),phase_deg=float(q[0]),q_original_deg=q.tolist(),curl_s=s.tolist(),evaluation=asdict(validation),elapsed_s=time.time()-begin)
        saved.append(record)
        (dest/'trials.json').write_text(json.dumps(saved,ensure_ascii=False,indent=2),encoding='utf-8')
        print('STAGE_RESULT '+json.dumps({k:v for k,v in record.items() if k not in ['evaluation','q_original_deg','curl_s','values','lengths_mm']})+' '+json.dumps({k:getattr(validation,k) for k in ['collision_free','minimum_clearance_mm','rod_closure_feasible','maximum_rod_closure_error_mm','hand_motion_feasible']}),flush=True)
        return vals,q,s,validation
    if resume:
        prior=json.loads((dest/'result.json').read_text(encoding='utf-8'))['selected']
        sweep=json.loads((dest/'exact_sweep.json').read_text(encoding='utf-8'))['records']
        qs=np.interp(np.linspace(0,1,25),[r['curl_s'] for r in sweep],[r['q_original_deg'] for r in sweep])
        vals,q,s,v=optimize(np.array(prior['values']),qs,25,.08,'compact')
    else:
        qs=np.interp(np.linspace(0,1,13),ev.hand_progress_schedules[o.id],np.array(ev.input_schedules_deg[o.id])+startphase)
        vals,q,s,v=optimize(base,qs,13,0,'feasibility')
    if not resume and v.collision_free and v.rod_closure_feasible and v.hand_motion_feasible:
        # Length squared sum discourages a few very long dimensions dominating.
        vals2,q2,s2,v2=optimize(vals,np.interp(np.linspace(0,1,25),s,q),25,.015,'shorten')
        if v2.collision_free and v2.rod_closure_feasible and v2.hand_motion_feasible:
            vals,q,s,v=vals2,q2,s2,v2
    feasible=[r for r in saved if all(r['evaluation'][k] for k in ['collision_free','rod_closure_feasible','hand_motion_feasible'])]
    selected=min(feasible,key=lambda r:sum(r['values'][i]**2 for i in indices)) if feasible else saved[0]
    doc=dict(finger=finger,profile='short_ad' if finger=='ring' else 'long_ad',variable_ids=[v.id for v in p.variables],fixed_dimensions_mm={k:m._candidate_lengths(p,base)[k] for k in ['L_ad','L_ae','L_de']},mount_tilt_deg=p.ad_tilt_deg,dorsal_clearance_mm=p.dorsal_clearance_mm,baseline_values=base.tolist(),baseline_evaluation=asdict(evaluate(p,base,0)),selected=selected,feasible_found=bool(feasible),source_unchanged=True,cad_or_hardware_modified=False)
    (dest/'result.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2),encoding='utf-8')
    print('DONE '+finger,flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('finger',choices=['ring','little']);parser.add_argument('--max-nfev',type=int,default=180);parser.add_argument('--resume',action='store_true')
    a=parser.parse_args();run(a.finger,a.max_nfev,a.resume)
