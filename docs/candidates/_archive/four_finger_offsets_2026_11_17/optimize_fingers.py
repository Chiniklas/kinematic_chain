"""Reproducible planar middle-finger solve for 50 mm motor offset + 6 mm root offset."""
from pathlib import Path
import sys,copy,math,time,json
from dataclasses import replace,asdict
import numpy as np
import yaml
from scipy.optimize import least_squares
import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[3])
parser.add_argument('--finger',choices=['index','middle','ring','little'],required=True)
parser.add_argument('--out',type=Path,required=True)
args=parser.parse_args();ROOT=args.repo.resolve();OUT=args.out.resolve();OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(ROOT/'tools'))
from _common import load_design, initial_vector, report_row
FINGER=args.finger
motor_offsets={'index':0,'middle':50,'ring':0,'little':50}
root_offsets={'index':0,'middle':6,'ring':2,'little':-3}
profile='short_ad' if FINGER in ('index','ring') else 'long_ad'
m,p=load_design(profile,FINGER,weights={'closure':5,'collision':20,'rod':20})
ORIGINAL=m._mechanism_workspace_poses
old=p;old_values=initial_vector(old);phase=0.0
if FINGER in ('ring','little'):
    prior=json.loads((ROOT/f'docs/candidates/ring_little_reoptimization/{FINGER}/parameters.json').read_text())
    old_values=np.array([prior['lengths_mm'][v.id] for v in p.variables]);phase=prior['phase_deg']
original_phase=phase
short=yaml.safe_load((ROOT/'designs/mechanism_2/short_ad/mechanism.yaml').read_text())
short_ad=next(v['value'] for v in short['dimensions'] if v['id']=='L_ad')
short_mount=next(a for a in short['exoskeleton_attachments'] if a['id']=='dorsal_input_mount')
a=math.radians(short_mount['ad_tilt_deg'])
dx=short_ad*math.cos(a)+motor_offsets[FINGER]+root_offsets[FINGER];dy=short_ad*math.sin(a)
nominal=copy.deepcopy(old.nominal_design)
mount=next(a for a in nominal['exoskeleton_attachments'] if a['id']=='dorsal_input_mount')
mount['ad_tilt_deg']=math.degrees(math.atan2(dy,dx))
next(v for v in nominal['dimensions'] if v['id']=='L_ad')['value']=math.hypot(dx,dy)
p=replace(old,nominal_design=nominal,ad_tilt_deg=mount['ad_tilt_deg'],fixed_mechanism_dimensions={'L_ad':math.hypot(dx,dy)})
# Keep parameterization topology stable at the D mount exclusion, as the archived
# ring/little optimizer also did. Bounds remain documented per finger.
p=replace(p,variables=tuple(replace(v,minimum=max(v.minimum,12.5)) if v.id in ('L_cd','L_dg') else v for v in p.variables))
ids=[v.id for v in p.variables]
old_pose=ORIGINAL(m._candidate_lengths(old,old_values),np.array([phase]),old.ad_tilt_deg)[0][1]
values=old_values.copy()
p=replace(p,variables=tuple(replace(v,maximum=v.maximum+max(0,root_offsets[FINGER])) if v.id=='L_ae' else v for v in p.variables))
values[ids.index('L_ae')]=math.dist((-dx,dy),old_pose['e'])
phase=phase+old.ad_tilt_deg-p.ad_tilt_deg
crank=math.radians(90-phase-p.ad_tilt_deg)
b_new=(-dx+values[ids.index('L_ab')]*math.cos(crank),dy+values[ids.index('L_ab')]*math.sin(crank))
values[ids.index('L_bc')]=math.dist(b_new,old_pose['c'])
values=np.clip(values,[v.minimum for v in p.variables],[v.maximum for v in p.variables])
print('MOUNT',json.dumps({'finger':FINGER,'motor_offset_mm':motor_offsets[FINGER],'root_offset_mm':root_offsets[FINGER],'horizontal_mm':dx,'vertical_mm_unchanged':dy,'AD_mm':math.hypot(dx,dy),'tilt_deg':p.ad_tilt_deg}),flush=True)
objective=p.objectives[0]

def tracked(lengths,qs,tilt):
    qs=np.asarray(qs,dtype=float)
    if not len(qs):return ()
    first=float(qs[0]);path=list(np.linspace(0,first,max(2,int(math.ceil(abs(first)/.25))+1))[:-1]) if abs(first)>1e-12 else []
    keep=[]
    for q in qs:
        if path:path.extend(np.linspace(path[-1],q,max(1,int(math.ceil(abs(q-path[-1])/.25)))+1)[1:].tolist())
        else:path.append(float(q))
        keep.append(len(path)-1)
    poses=ORIGINAL(lengths,np.array(path),tilt)
    return tuple((float(q),poses[i][1]) for q,i in zip(qs,keep) if i<len(poses))

def evaluate(problem, vals, phase):
    m._mechanism_workspace_poses=lambda lengths,qs,tilt_deg=0:tuple((float(q),pose) for q,(_,pose) in zip(qs,tracked(lengths,np.asarray(qs)+phase,tilt_deg)))
    try:return m.evaluate(problem,vals)
    finally:m._mechanism_workspace_poses=ORIGINAL

print('BASELINE',report_row('previous candidate',evaluate(old,old_values,original_phase),m._candidate_lengths(old,old_values)),flush=True)
seed_evaluation=evaluate(p,values,phase)
print('INITIAL',report_row('new mount',seed_evaluation,m._candidate_lengths(p,values)),flush=True)

def clearance(pos,hand,r4):
    cfg=p.component_config['hand_mechanism_non_collision'];rows=[]
    for _,u,v in m._mechanism_segments(p,pos,cfg['dorsal_mount_exclusion_radius_mm']):
        rows.extend(m._segment_distance(u,v,hs,he)-cfg['mechanism_link_radius_mm']-hr for _,hs,he,hr in hand)
    for _,poly in m._mechanism_polygons(p,pos,cfg['dorsal_mount_exclusion_radius_mm']):
        rows.extend(m._segment_polygon_signed_distance(hs,he,poly,cfg['link_axis_samples'])-cfg['mechanism_link_radius_mm']-hr for _,hs,he,hr in hand)
    rod=m._trim_segment_endpoint(pos['h'],r4,cfg['r4_contact_exclusion_radius_mm'],False)
    rows.extend(m._segment_distance(*rod,hs,he)-cfg['output_rod_radius_mm']-hr if rod else 100 for _,hs,he,hr in hand)
    return np.array(rows)

started=time.monotonic()
feasible_candidates=[]
if seed_evaluation.collision_free and seed_evaluation.rod_closure_feasible and seed_evaluation.hand_motion_feasible:
    feasible_candidates.append((values.copy(),phase,'geometric seed'))
for n,max_nfev in [(7,60),(17,60)]:
    s=np.linspace(0,1,n);hands=[m._hand_capsules(p,objective,float(t)) for t in s]
    contacts=[m._hand_distal_contact_progress(p,objective,float(t)) for t in s]
    angles=(np.interp(s,seed_evaluation.hand_progress_schedules[objective.id],np.array(seed_evaluation.input_schedules_deg[objective.id])+phase) if seed_evaluation.hand_progress_schedules[objective.id] else np.linspace(phase,phase+(40 if FINGER in ('index','ring') else (54 if FINGER=='middle' else 69.4)),n)) if n==7 else np.interp(s,old_s,angles)
    lower=np.r_[[v.minimum for v in p.variables],-15.,np.zeros(n-1)]
    upper=np.r_[[v.maximum for v in p.variables],15.,np.full(n-1,90.)]
    z0=np.r_[values,angles]
    calls=0;last=0;size=None;best=float('inf')
    def fun(z):
        global calls,last,size,best
        calls+=1
        lengths=m._candidate_lengths(p,z[:len(values)]);q=z[len(values):]
        poses=tracked(lengths,q,p.ad_tilt_deg) if np.all(np.diff(q)>0) else ()
        if len(poses)!=n:
            if size is None:raise RuntimeError('Initial seed does not assemble')
            return np.full(size,1000+np.linalg.norm(z-z0))
        errors=np.array([math.dist(pos['h'],r4)-lengths['L_tip_rod'] for (_,pos),r4 in zip(poses,contacts)])
        cl=np.concatenate([clearance(pos,h,r4) for (_,pos),h,r4 in zip(poses,hands,contacts)])
        # Stable residual layout: trim/polygon helpers must not omit changed topology.
        res=np.r_[errors*20,np.minimum(cl-.5,0)*4,(z[:len(values)]-old_values)*.002, lengths['L_tip_rod']*.008, (q[0]-phase)*.001]
        if size is None:size=len(res)
        if len(res)!=size:raise RuntimeError('Collision residual layout changed')
        cost=float(res@res)
        if time.monotonic()-last>20:
            print('ITER',json.dumps({'poses':n,'calls':calls,'cost':cost,'closure_mm':float(max(abs(errors))),'clearance_mm':float(min(cl)),'elapsed_s':round(time.monotonic()-started,1)}),flush=True);last=time.monotonic()
        return res
    fun(z0)
    result=least_squares(fun,z0,bounds=(lower,upper),max_nfev=max_nfev,x_scale='jac',ftol=1e-7,xtol=1e-7,gtol=1e-7,diff_step=1e-5)
    values=result.x[:len(values)];angles=result.x[len(values):];old_s=s
    print('STAGE',n,'nfev',result.nfev,'cost',result.cost,flush=True)
    np.save(OUT/f'vector_{n}.npy',values)
    (OUT/f'angles_{n}.json').write_text(json.dumps(angles.tolist()))
    # Validate with the existing crank-driven passive-hand evaluator, independent
    # of collocation angles. Do not claim feasibility solely from optimizer cost.
    stage_evaluation=evaluate(p,values,float(angles[0]))
    print('CHECK',report_row('candidate',stage_evaluation,m._candidate_lengths(p,values)),flush=True)
    if stage_evaluation.collision_free and stage_evaluation.rod_closure_feasible and stage_evaluation.hand_motion_feasible:
        feasible_candidates.append((values.copy(),float(angles[0]),f'optimized {n} poses'))
# Independently recompute crank-driven passive motion, densely. Optimize the rod
# and geometry without pretending collocation's prescribed q/s is a physical control law.
components=copy.deepcopy(p.component_config);constraints=copy.deepcopy(p.constraint_config)
components['task_space_reachability']['curled_input_samples']=1801
for k in ['hand_mechanism_non_collision','output_link_perpendicularity']:components[k]['evaluation_poses']=1201
for k in ['closure_error','hand_motion_admissibility']:constraints[k]['evaluation_poses']=1201
dense=replace(p,component_config=components,constraint_config=constraints)
feasible_candidates.sort(key=lambda row:row[0][ids.index('L_tip_rod')])
for values,selected_phase,selected_stage in feasible_candidates:
    ev=evaluate(dense,values,selected_phase)
    if ev.collision_free and ev.rod_closure_feasible and ev.hand_motion_feasible:break
else:raise SystemExit('No candidate passed dense validation; results not adopted')
angles[0]=selected_phase
print('SELECTED',selected_stage,flush=True)
report={'finger':FINGER,'status':'sampled_planar_candidate','input':{'reference_finger':'index','motor_mount_offset_mm':motor_offsets[FINGER],'finger_root_forward_offset_mm':root_offsets[FINGER],'vertical_geometry_changed':False},'mount':{'horizontal_mm':dx,'vertical_mm':dy,'L_ad_mm':math.hypot(dx,dy),'ad_tilt_deg':p.ad_tilt_deg,'dorsal_clearance_mm':p.dorsal_clearance_mm},'lengths_mm':m._candidate_lengths(p,values),'selected_stage':selected_stage,'phase_deg':float(angles[0]),'optimizer':'scipy.optimize.least_squares / direct collocation','pose_count':1201,'evaluated':asdict(ev),'bounds':{v.id:[v.minimum,v.maximum] for v in p.variables},'objectives':'rod closure and hand clearance, weak output-rod shortening and prior-length regularization; perpendicularity measured, not optimized'}
report['passed']=bool(ev.collision_free and ev.rod_closure_feasible and ev.hand_motion_feasible)
(OUT/'result.json').write_text(json.dumps(report,indent=2))
print('DENSE',report_row(FINGER,ev,m._candidate_lengths(p,values)),flush=True)
if not report['passed']:raise SystemExit('DENSE VALIDATION FAILED: candidate not adopted')
# Use the same phase-aware FK in the project materializer. No global source edits.
m._mechanism_workspace_poses=lambda lengths,qs,tilt_deg=0:tuple((float(q),pose) for q,(_,pose) in zip(qs,tracked(lengths,np.asarray(qs)+angles[0],tilt_deg)))
try:
    materialized=m.OptimizationResult(values,ev,ev,0,True,(),'scipy.optimize.least_squares','cpu')
    m.write_outputs(OUT/'materialized',dense,materialized)
finally:m._mechanism_workspace_poses=ORIGINAL
print('DONE',FINGER,flush=True)
