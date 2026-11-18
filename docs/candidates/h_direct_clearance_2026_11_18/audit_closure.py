"""Audit actual H=R4 closure with three independent bounded hand angles.

This does not change synthesis inputs or candidate dimensions. It asks whether
the endpoint mismatch can be absorbed by the assumed human joint ranges.
All errors and unsuccessful solves are retained, not classified as a proof of
global infeasibility. Nonnegative flexion is retained; no MCP hyperextension.
"""
from verify_1mm import *
from scipy.optimize import least_squares


def hand_parameters(p):
    obj=p.objectives[0].data
    lengths=np.array([obj['phalanx_lengths_mm'][k] for k in ['proximal','middle','distal']],dtype=float)
    limits=np.radians([obj['joint_flexion_ranges_deg'][k]['max'] for k in ['mcp','pip','dip']])
    return lengths,limits


def endpoint(p,angles,jacobian=False):
    lengths,_=hand_parameters(p);lengths[-1]/=2
    phi=np.cumsum(angles)
    pos=np.array([np.sum(lengths*np.cos(phi)), -np.sum(lengths*np.sin(phi))-p.dorsal_clearance_mm])
    if not jacobian:return pos
    contributions=np.array([-lengths*np.sin(phi),-lengths*np.cos(phi)])
    return np.flip(np.cumsum(np.flip(contributions,axis=1),axis=1),axis=1)


def angle_hands(p,angles):
    lengths,_=hand_parameters(p);phi=np.cumsum(angles,axis=1)
    units=np.stack((np.cos(phi),-np.sin(phi)),axis=-1)
    base=np.tile([0.,-p.dorsal_clearance_mm],(len(angles),1))
    joints=np.concatenate((base[:,None,:],base[:,None,:]+np.cumsum(units*lengths[None,:,None],axis=1)),axis=1)
    model=p.nominal_design['human_hand_model'];rows={r['id']:r for r in model['phalanges']}
    widths=np.array([rows[k+'_phalanx']['width_mm'] for k in ['proximal','middle','distal']])
    offsets=-rot90(units)*widths[None,:,None]/2
    starts=joints[:,:-1,:]+offsets;ends=joints[:,1:,:]+offsets
    radii=np.broadcast_to(widths[None,:,None]/2,(len(angles),3,2))
    fingers=np.stack((starts,ends,radii),axis=2)
    palm=model['palm'];r=palm['width_mm']/2;y=-p.dorsal_clearance_mm-r
    palmrow=np.tile([[[-palm['length_mm'],y],[0.,y],[r,r]]],(len(angles),1,1))[:,None,:,:]
    return np.concatenate((palmrow,fingers),axis=1)


def audit(candidate,count=241):
    finger=candidate['finger'];m,p=load_design(finger,finger);lengths,limits=hand_parameters(p)
    q=np.linspace(candidate['q_deg'][0],candidate['q_deg'][-1],count)
    nodes,_=fk(candidate['lengths_mm'],q,p.ad_tilt_deg,candidate.get('output_branch',1))
    s,_=project_hand(p,nodes['h']);reference=s[:,None]*limits
    rows=[];chosen=[];previous=None
    for i,target in enumerate(nodes['h']):
        ref=reference[i]
        def fun(a):return endpoint(p,a)-target
        def jac(a):return endpoint(p,a,True)
        starts=[ref,limits*np.array([.02,.9,.5]),limits*np.array([.9,.02,.02])]
        if previous is not None:starts.append(previous)
        fits=[least_squares(fun,np.clip(a,1e-9,limits-1e-9),jac=jac,bounds=(np.zeros(3),limits),
                            ftol=1e-12,xtol=1e-12,gtol=1e-12,max_nfev=250) for a in starts]
        best=min(fits,key=lambda r:np.linalg.norm(r.fun));angles=best.x
        if np.linalg.norm(best.fun)<1e-5:
            close=minimize(lambda a:float(np.sum((a-ref)**2)),angles,jac=lambda a:2*(a-ref),method='SLSQP',
                bounds=list(zip(np.zeros(3),limits)),constraints=[{'type':'eq','fun':fun,'jac':jac}],
                options={'ftol':1e-12,'maxiter':150})
            if np.linalg.norm(fun(close.x))<1e-5:angles=close.x
        residual=float(np.linalg.norm(fun(angles)));previous=angles.copy();chosen.append(angles)
        rows.append({'q_deg':float(q[i]),'reference_progress':float(s[i]),'residual_mm':residual,
                     'reference_angles_deg':np.degrees(ref).tolist(),'fitted_angles_deg':np.degrees(angles).tolist(),
                     'angle_change_deg':np.degrees(angles-ref).tolist(),'position_closed':residual<=1e-4,
                     'target_h_mm':target.tolist(),'fitted_r4_mm':endpoint(p,angles).tolist()})
    angles=np.array(chosen);clear=collision(p,nodes,angle_hands(p,angles),samples=21,output_shape='l_bracket')
    closed=np.array([r['position_closed'] for r in rows]);errors=np.array([r['residual_mm'] for r in rows])
    for row,value in zip(rows,clear):row['fitted_pose_clearance_mm']=float(value)
    report={'poses':count,'closure_tolerance_mm':1e-4,'hand_joint_ranges_deg':np.degrees(limits).tolist(),
            'all_positions_closed':bool(np.all(closed)), 'closed_pose_count':int(sum(closed)),
            'maximum_remaining_closure_error_mm':float(max(errors)),
            'start_closure_error_mm':float(errors[0]),'terminal_closure_error_mm':float(errors[-1]),
            'maximum_joint_change_deg':float(np.max(abs(np.degrees(angles-reference)))),
            'maximum_joint_change_for_closed_poses_deg':float(np.max(abs(np.degrees(angles[closed]-reference[closed])))) if any(closed) else None,
            'minimum_clearance_for_closed_poses_mm':float(min(clear[closed])) if any(closed) else None,
            'minimum_fitted_joint_increment_deg':np.min(np.diff(np.degrees(angles),axis=0),axis=0).tolist(),
            'independent_joint_fit_not_a_certified_passive_hand_motion':True,
            'global_infeasibility_proven':False,'negative_flexion_allowed':False}
    return report,rows


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--poses',type=int,default=241)
    args=parser.parse_args();candidate=json.loads(args.source.read_text());report,rows=audit(candidate,args.poses)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'closure_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.out/'closure_sweep.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(candidate['finger'],json.dumps(report),flush=True)


if __name__=='__main__':main()
