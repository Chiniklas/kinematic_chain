"""Independent exact passive closure continuation and reproducible result files."""
from pathlib import Path
import json,math,csv,shutil,copy
from dataclasses import asdict
import numpy as np
from scipy.optimize import brentq
from reoptimize_fingers import ROOT,OUT,load,anchored,clearances,m,evaluate

def polish_phases(p,trial):
    vals=np.array(trial['values']);lengths=m._candidate_lengths(p,vals)
    r4=m._hand_distal_contact_progress(p,p.objectives[0],0.)
    def f(q):
        poses=anchored(lengths,[q],p.ad_tilt_deg)
        return math.dist(poses[0][1]['h'],r4)-lengths['L_tip_rod'] if poses else float('nan')
    phase=trial['phase_deg'];grid=np.linspace(max(-15,phase-3),min(15,phase+3),301)
    errors=[f(q) for q in grid];roots=[]
    for a,b,fa,fb in zip(grid[:-1],grid[1:],errors[:-1],errors[1:]):
        if fa*fb<0:roots.append(brentq(f,a,b,xtol=1e-12))
    return sorted(roots,key=lambda q:abs(q-phase))

def exact_check(p,values,phase,upstream,n=1201):
    o=p.objectives[0];lengths=m._candidate_lengths(p,values)
    approx_end=upstream['input_schedules_deg'][o.id][-1]
    r4end=m._hand_distal_contact_progress(p,o,1)
    def terminal(q):
        poses=anchored(lengths,[q],p.ad_tilt_deg,phase)
        if not poses:return float('nan')
        return math.dist(poses[0][1]['h'],r4end)-lengths['L_tip_rod']
    qend=brentq(terminal,max(.001,approx_end-.3),approx_end+.3,xtol=1e-12)
    qs=np.linspace(0,qend,n);poses=anchored(lengths,qs,p.ad_tilt_deg,phase)
    grid=np.linspace(0,1,1001)
    contacts=np.array([m._hand_distal_contact_progress(p,o,float(si)) for si in grid])
    records=[];previous=0;missing=[]
    for i,(q,pos) in enumerate(poses):
        def residual(s):return math.dist(pos['h'],m._hand_distal_contact_progress(p,o,s))-lengths['L_tip_rod']
        errors=np.linalg.norm(contacts-np.array(pos['h']),axis=1)-lengths['L_tip_rod']
        roots=[]
        for k in np.flatnonzero(errors[:-1]*errors[1:]<0):
            roots.append(brentq(residual,grid[k],grid[k+1],xtol=1e-12))
        if abs(errors[0])<1e-9:roots.append(0.)
        if abs(errors[-1])<1e-9:roots.append(1.)
        if i==0:s=0.
        else:
            candidates=[r for r in roots if r>=previous-1e-8]
            if candidates:s=min(candidates,key=lambda r:abs(r-previous))
            else:s=float(grid[np.argmin(abs(errors))]);missing.append(i)
        r4=m._hand_distal_contact_progress(p,o,s)
        cls=clearances(p,pos,m._hand_capsules(p,o,s),r4,True)
        worst=min(cls,key=lambda r:r[2])
        records.append(dict(index=i,q_relative_deg=float(q),q_original_deg=float(q+phase),curl_s=s,closure_mm=abs(residual(s)),minimum_clearance_mm=worst[2],nearest_mechanism_part=worst[0],nearest_hand_part=worst[1],points={k:list(v) for k,v in pos.items()}))
        previous=s
    progress=np.array([r['curl_s'] for r in records]);worst=min(records,key=lambda r:r['minimum_clearance_mm'])
    closure=max(r['closure_mm'] for r in records)
    reverse=anchored(lengths,qs[::-1],p.ad_tilt_deg,phase)
    reverse_error=max(math.dist(pos[k],rev[k]) for (_,pos),(_,rev) in zip(poses,reversed(reverse)) for k in pos) if len(reverse)==n else float('inf')
    dimension_error=max(abs(math.dist(pos[k[2]],pos[k[3]])-value) for _,pos in poses for k,value in lengths.items() if k!='L_tip_rod')
    dropped_body_guard=min(lengths['L_cd'],lengths['L_dg'])>=p.component_config['hand_mechanism_non_collision']['dorsal_mount_exclusion_radius_mm']+.5-1e-8
    ok=len(records)==n and not missing and closure<=.1 and worst['minimum_clearance_mm']>=0 and progress[-1]>=.999 and min(np.diff(progress))>=-1e-8 and max(np.diff(progress))<.05 and reverse_error<1e-7 and dimension_error<1e-7 and dropped_body_guard and abs(phase)<=15 and 0<qend<=90
    return dict(passed=bool(ok),samples=n,assembly_samples=len(records),exact_passive_root_missing_indices=missing,maximum_closure_mm=closure,minimum_clearance_mm=worst['minimum_clearance_mm'],worst_pose={k:v for k,v in worst.items() if k!='points'},terminal_curl_s=float(progress[-1]),maximum_progress_step=float(max(np.diff(progress))),minimum_progress_step=float(min(np.diff(progress))),phase_deg=phase,stroke_deg=qend,reverse_tracking_error_mm=reverse_error,maximum_dimension_error_mm=dimension_error,mount_polygon_preservation_passed=bool(dropped_body_guard),records=records)

def main():
    rows=[];summary=[]
    for finger in ['ring','little']:
        p,base=load(finger);dest=OUT/finger
        doc=json.loads((dest/'result.json').read_text(encoding='utf-8'))
        trials=json.loads((dest/'trials.json').read_text(encoding='utf-8'))
        validations=[]
        for trial in trials:
            ev=trial['evaluation']
            if not all(ev[k] for k in ['collision_free','rod_closure_feasible','hand_motion_feasible']):continue
            for phase in polish_phases(p,trial):
                refined=copy.deepcopy(trial)
                refined['phase_before_exact_polish_deg']=trial['phase_deg']
                refined['phase_deg']=phase
                refined['collocation_proposal_q_original_deg']=refined.pop('q_original_deg',[])
                refined['collocation_curl_s']=refined.pop('curl_s',[])
                refined['evaluation']=asdict(evaluate(p,np.array(trial['values']),phase))
                check=exact_check(p,np.array(trial['values']),phase,refined['evaluation'])
                validations.append((refined,check))
                print(finger,trial['tag'],{k:v for k,v in check.items() if k!='records'},flush=True)
                if check['passed']:break
        viable=[pair for pair in validations if pair[1]['passed']]
        if not viable:raise RuntimeError('No independently validated candidate: '+finger)
        selected,check=min(viable,key=lambda pair:sum(v*v for k,v in pair[0]['lengths_mm'].items() if k not in ['L_ad','L_ae','L_de']))
        doc['selected']=selected;doc['exact_validation']={k:v for k,v in check.items() if k!='records'}
        (dest/'validation_trials.json').write_text(json.dumps([dict(tag=t['tag'],phase_deg=t['phase_deg'],validation={k:v for k,v in c.items() if k!='records'}) for t,c in validations],ensure_ascii=False,indent=2),encoding='utf-8')
        (dest/'result.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2),encoding='utf-8')
        (dest/'exact_sweep.json').write_text(json.dumps(check,ensure_ascii=False,indent=2),encoding='utf-8')
        with (dest/'exact_sweep.csv').open('w',newline='',encoding='utf-8-sig') as f:
            fields=[k for k in check['records'][0] if k!='points'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows({k:r[k] for k in fields} for r in check['records'])
        lengths=m._candidate_lengths(p,base)
        for k,before in lengths.items():
            after=selected['lengths_mm'][k]
            rows.append(dict(finger=finger,dimension=k,before_mm=before,after_mm=after,change_mm=after-before,change_percent=(after/before-1)*100,fixed=k in ['L_ad','L_ae','L_de']))
        summary.append(dict(finger=finger,stage=selected['tag'],baseline=doc['baseline_evaluation'],after=doc['exact_validation'],lengths_before=lengths,lengths_after=selected['lengths_mm']))
    with (OUT/'length_comparison.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
    (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    for name in ['reoptimize_fingers.py','validate_reoptimization.py']:
        shutil.copy2(ROOT/'work'/name,OUT/name)
    for finger in ['ring','little']:shutil.copy2(ROOT/'work'/f'reopt_{finger}.log',OUT/finger/'optimization.log')

if __name__=='__main__':main()
