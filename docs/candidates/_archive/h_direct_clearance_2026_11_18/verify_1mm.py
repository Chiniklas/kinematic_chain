"""Optimize and independently validate direct H=R4 candidates at 1 mm tolerance.

Collision constraints use the actual nearest-progress hand pose during search,
the same pose definition as independent validation. No hand trajectory, mount,
attachment exclusion, material radius or finger measurement is relaxed.
"""
from base_geometry import *
import copy


def hand_curve(p, s, derivatives=False):
    objective=p.objectives[0].data
    ph=objective['phalanx_lengths_mm'];jr=objective['joint_flexion_ranges_deg']
    lengths=np.array([ph['proximal'],ph['middle'],ph['distal']/2.])
    omega=-np.radians(np.cumsum([jr[k]['max'] for k in ['mcp','pip','dip']]))
    angle=np.asarray(s)[...,None]*omega
    unit=np.stack((np.cos(angle),np.sin(angle)),axis=-1)
    positions=np.sum(lengths[:,None]*unit,axis=-2)+[0.,-p.dorsal_clearance_mm]
    if not derivatives:return positions
    tangent=np.sum((lengths*omega)[:,None]*rot90(unit),axis=-2)
    acceleration=np.sum((-lengths*omega**2)[:,None]*unit,axis=-2)
    return positions,tangent,acceleration


def project_hand(p, points):
    """Global coarse bracket followed by vector Newton projection.

    The final report is independently checked with the archived scalar bounded
    minimizer, so this acceleration does not establish feasibility by itself.
    """
    grid=np.linspace(0,1,101);curve=hand_curve(p,grid)
    indices=np.argmin(np.sum((points[:,None,:]-curve[None,:,:])**2,axis=-1),axis=1)
    lo=grid[np.maximum(indices-1,0)];hi=grid[np.minimum(indices+1,100)]
    s=grid[indices]
    for _ in range(8):
        pos,tangent,acc=hand_curve(p,s,True);delta=pos-points
        denominator=np.sum(tangent*tangent+delta*acc,axis=-1)
        step=np.sum(delta*tangent,axis=-1)/np.maximum(denominator,1e-9)
        s=np.clip(s-step,lo,hi)
    options=np.stack((s,lo,hi,np.zeros_like(s),np.ones_like(s)),axis=-1)
    targets=hand_curve(p,options)
    costs=np.sum((targets-points[:,None,:])**2,axis=-1)
    choice=np.argmin(costs,axis=1)
    s=options[np.arange(len(s)),choice]
    return s,hand_curve(p,s)


def fast_hands(p,s):
    """Same dorsal-surface capsule construction as the archived evaluator."""
    obj=p.objectives[0].data;ph=obj['phalanx_lengths_mm'];jr=obj['joint_flexion_ranges_deg']
    lengths=np.array([ph[k] for k in ['proximal','middle','distal']])
    omega=-np.radians(np.cumsum([jr[k]['max'] for k in ['mcp','pip','dip']]))
    angles=s[:,None]*omega;unit=np.stack((np.cos(angles),np.sin(angles)),axis=-1)
    segments=unit*lengths[None,:,None]
    joint0=np.tile([0.,-p.dorsal_clearance_mm],(len(s),1))
    joints=np.concatenate((joint0[:,None,:],joint0[:,None,:]+np.cumsum(segments,axis=1)),axis=1)
    hand=p.nominal_design['human_hand_model'];rows={r['id']:r for r in hand['phalanges']}
    widths=np.array([rows[k+'_phalanx']['width_mm'] for k in ['proximal','middle','distal']])
    offsets=-rot90(unit)*widths[None,:,None]/2
    starts=joints[:,:-1,:]+offsets;ends=joints[:,1:,:]+offsets
    radii=np.broadcast_to(widths[None,:,None]/2,(len(s),3,2))
    fingers=np.stack((starts,ends,radii),axis=2)
    palm=hand['palm'];r=palm['width_mm']/2;y=-p.dorsal_clearance_mm-r
    palmrow=np.tile([[[-palm['length_mm'],y],[0.,y],[r,r]]],(len(s),1,1))[:,None,:,:]
    return np.concatenate((palmrow,fingers),axis=1)


def independent_check(candidate,count=4801):
    """Recompute geometry, projection and collisions with archived evaluator."""
    report,rows,nodes=validate(candidate,count)
    report['endpoint_tolerance_mm']=1.
    report['endpoint_error_passed']=report['maximum_endpoint_error_mm']<=1.
    report['motion_passed']=bool(report['start_progress']<.001 and report['terminal_progress']>=.99 and report['monotonic_progress'])
    report['trajectory_passed']=report['endpoint_error_passed'] and report['motion_passed']
    report['passed']=bool(report['trajectory_passed'] and report['collision_passed'] and report['assembly_valid'])
    m,p=load_design(candidate['finger'],candidate['finger'])
    actual_progress=np.array([r['progress'] for r in rows])
    projected,_=project_hand(p,nodes['h'])
    report['maximum_projection_progress_difference']=float(np.max(abs(projected-actual_progress)))
    report['maximum_projection_position_difference_mm']=float(np.max(np.linalg.norm(hand_curve(p,projected)-hand_curve(p,actual_progress),axis=-1)))
    candidate=copy.deepcopy(candidate);candidate['validation']=report
    candidate['status']='sampled_planar_candidate_1mm' if report['passed'] else 'rejected_search_candidate_1mm'
    return candidate,rows,nodes


def optimize(seed,out,stages=(25,49),maxiter=220,clearance_target=.5,error_limit=.95,terminal_progress=.99,dg_max=100.):
    out.mkdir(parents=True,exist_ok=True)
    finger=seed['finger'];m,p=load_design(finger,finger);search_bounds=BOUNDS.copy()
    search_bounds['L_dg']=(BOUNDS['L_dg'][0],dg_max)
    ids=list(search_bounds);branch=seed.get('output_branch',1)
    fixed=seed['lengths_mm']['L_ad'];shape='l_bracket'
    values=np.array([seed['lengths_mm'][k] for k in ids])
    original_values=values.copy();q0=seed['q_deg'][0];q1=seed['q_deg'][-1]
    start=time.monotonic();last=0;history=[];best_archive=[]
    for n in stages:
        fractions=np.linspace(0,1,n);cache={}
        def evaluate(z):
            nonlocal cache
            if 'z' in cache and np.array_equal(z,cache['z']):return cache
            L={'L_ad':fixed,**dict(zip(ids,z[:len(ids)]))}
            q=z[-3]+(z[-2]-z[-3])*fractions
            nodes,margins=fk(L,q,p.ad_tilt_deg,branch)
            s,targets=project_hand(p,nodes['h']);errors=np.linalg.norm(nodes['h']-targets,axis=-1)
            clear=collision(p,nodes,fast_hands(p,s),samples=7,output_shape=shape)
            endtargets,tangents,_=hand_curve(p,np.array([0.,1.]),True)
            tangents/=np.linalg.norm(tangents,axis=-1,keepdims=True)
            endpoint_tangent=np.sum((nodes['h'][[0,-1]]-endtargets)*tangents,axis=-1)
            cache=dict(z=z.copy(),lengths=L,q=q,s=s,nodes=nodes,margins=margins,errors=errors,clear=clear,end_tangent=endpoint_tangent)
            return cache
        def constraints(z):
            ev=evaluate(z)
            return np.r_[z[-1]-ev['errors'],ev['clear']-clearance_target,
                         (ev['margins']-.001).ravel(),z[-2]-z[-3]-2.,
                         np.diff(ev['s']),.0005-ev['s'][0],ev['s'][-1]-terminal_progress]
        # Minimax error while ensuring collision and travel constraints. Feasible
        # candidates below 0.95 mm have a reserve against the 1 mm acceptance.
        best_z=None;best_t=float('inf')
        def callback(z):
            nonlocal last,best_z,best_t
            conmin=float(min(constraints(z)));ev=evaluate(z)
            if conmin>=-1e-7 and z[-1]<best_t:
                best_t=float(z[-1]);best_z=z.copy()
            if time.monotonic()-last>25:
                print('ITER',finger,n,round(time.monotonic()-start,1),json.dumps({'max_error':float(max(ev['errors'])),'clearance':float(min(ev['clear'])),'constraint_min':conmin}),flush=True);last=time.monotonic()
        z0=np.r_[values,q0,q1,1.]
        z0[-1]=max(evaluate(z0)['errors'])+.001
        bounds=[search_bounds[k] for k in ids]+[(-25,115),(-25,115),(0,10)]
        z0=np.clip(z0,np.array(bounds)[:,0],np.array(bounds)[:,1]);callback(z0)
        result=minimize(lambda z:z[-1],z0,method='SLSQP',bounds=bounds,constraints=[{'type':'ineq','fun':constraints}],callback=callback,
                        options={'maxiter':maxiter,'ftol':1e-9})
        callback(result.x);chosen=best_z if best_z is not None else result.x;ev=evaluate(chosen)
        values=chosen[:len(ids)];q0=float(chosen[-3]);q1=float(chosen[-2])
        history.append({'poses':n,'optimizer_success':bool(result.success),'message':str(result.message),
                        'search_maximum_error_mm':float(max(ev['errors'])),'search_minimum_clearance_mm':float(min(ev['clear'])),
                        'constraint_min':float(min(constraints(chosen)))})
        candidate={**seed,'lengths_mm':{k:float(v) for k,v in ev['lengths'].items()},'q_deg':ev['q'].tolist(),'progress':ev['s'].tolist(),
                   'output_shape':shape,'output_branch':branch,'endpoint_tolerance_mm':1.,'collision_search_clearance_mm':clearance_target,
                   'search_terminal_progress_minimum':terminal_progress,
                   'bounds_mm':search_bounds,'dg_search_upper_bound_mm':dg_max,
                   'optimizer':'nearest-hand-pose constrained minimax, uniform motor-angle samples',
                   'search_history':history.copy(),'elapsed_s':time.monotonic()-start,
                   'hand_path_modified':False,'source_seed_dimensions_mm':dict(zip(ids,original_values.tolist()))}
        for key in ['validation','status','selected_stage','conversion_max_endpoint_change_mm','changed_dimensions']:
            candidate.pop(key,None)
        checked,rows,_=independent_check(candidate,1201)
        (out/f'stage_{n}.json').write_text(json.dumps(checked,indent=2)+'\n')
        best_archive.append(checked)
        print('STAGE',finger,n,json.dumps(checked['validation']),flush=True)
        # More input samples are always checked independently; stop optimization
        # after a validated design meets the requested threshold with reserve.
        if checked['validation']['passed'] and checked['validation']['maximum_endpoint_error_mm']<=error_limit:
            break
    def score(c):
        v=c['validation'];return (not v['passed'],not v['collision_passed'],not v['assembly_valid'],v['maximum_endpoint_error_mm'])
    chosen=min(best_archive,key=score)
    checked,rows,_=independent_check(chosen,4801)
    (out/'result.json').write_text(json.dumps(checked,indent=2)+'\n');(out/'sweep.json').write_text(json.dumps(rows,indent=2)+'\n')
    print('FINAL',finger,json.dumps(checked['validation']),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['optimize','validate'])
    parser.add_argument('--source',required=True,type=Path);parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--stages',default='25,49');parser.add_argument('--maxiter',type=int,default=220)
    parser.add_argument('--clearance',type=float,default=.5)
    parser.add_argument('--terminal-progress',type=float,default=.99,help='Same 99%% coverage threshold as archived validation')
    parser.add_argument('--dg-max',type=float,default=100.,help='Exploratory DG search upper bound; record any enlargement explicitly')
    args=parser.parse_args();candidate=json.loads(args.source.read_text())
    if args.action=='optimize':optimize(candidate,args.out,tuple(map(int,args.stages.split(','))),args.maxiter,args.clearance,terminal_progress=args.terminal_progress,dg_max=args.dg_max)
    else:
        args.out.mkdir(parents=True,exist_ok=True);c,rows,_=independent_check(candidate)
        (args.out/'result.json').write_text(json.dumps(c,indent=2)+'\n');(args.out/'sweep.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(json.dumps(c['validation']),flush=True)


if __name__=='__main__':main()
