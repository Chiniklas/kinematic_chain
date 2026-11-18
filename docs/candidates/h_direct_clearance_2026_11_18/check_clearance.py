"""Fixed-hole GH outline relief, retaining 1 mm trajectory tolerance.

No optimization of link lengths, human measurements or hole coordinates.
The existing FG arm stays. GH is replaced by two capsule segments G-P-H;
P is a material-contour corner, not a new joint. Radius and H exclusion stay
2 mm and 4 mm respectively. This uses exactly the old collision evaluator
for all unchanged components, replacing only its GH contribution.
"""
from audit_closure import *
import copy

HERE=Path(__file__).resolve().parent
BASE=HERE/'baseline'


def outline(nodes,offset_mm,terminal_run_mm=10.):
    g,h=nodes['g'],nodes['h'];unit=(h-g)/np.linalg.norm(h-g,axis=-1,keepdims=True)
    point=h-terminal_run_mm*unit+offset_mm*rot90(unit)
    return np.stack((g,point,h),axis=1)


def relief_clearances(nodes,hands,offset_mm,terminal_run_mm=10.,radius_mm=2.,attachment_mm=4.):
    points=outline(nodes,offset_mm,terminal_run_mm)
    g,point,h=points[:,0,:],points[:,1,:],points[:,2,:]
    last_length=np.linalg.norm(point-h,axis=-1,keepdims=True)
    if np.any(last_length<=attachment_mm):raise ValueError('Relief must not fit inside the attachment exemption.')
    trimmed_h=h+attachment_mm*(point-h)/last_length
    # No additional segment is exempted. The old 4 mm clipping remains only
    # on the H-incident segment, within the same H-centred disk.
    start=hands[:,:,0,:];end=hands[:,:,1,:];radii=hands[:,:,2,0]
    values=[];names=[]
    for key,a,b in [('GH_G_P',g,point),('GH_P_H',point,trimmed_h)]:
        values.append(segment_distance(a[:,None,:],b[:,None,:],start,end)-radii-radius_mm)
        names += [key+'/'+f for f in ['palm','proximal','middle','distal']]
    return np.concatenate(values,axis=1),names


def result_metric(matrix,names,q,s,alpha):
    flat=int(np.argmin(matrix));i,j=np.unravel_index(flat,matrix.shape)
    return {'minimum_clearance_mm':float(matrix[i,j]),'pair':names[j],
            'q_deg':float(q[i]),'reference_progress':float(s[i]),
            'hand_angle_blend_fraction':float(alpha),'pose_index':int(i),
            'interfering_pose_count':int(np.count_nonzero(matrix.min(axis=1)<0))}


def validate(finger,output,pose_count=4801,blend_count=11,offset_mm=2.,terminal_run_mm=10.):
    folder=BASE/finger;c=json.loads((folder/'result.json').read_text())
    reference_rows=json.loads((folder/'sweep.json').read_text())
    fitted_rows=json.loads((folder/'closure_sweep.json').read_text())
    m,p=load_design(finger,finger);_,limits=hand_parameters(p)
    q=np.linspace(c['q_deg'][0],c['q_deg'][-1],pose_count)
    nodes,margins=fk(c['lengths_mm'],q,p.ad_tilt_deg,c['output_branch'])
    s,targets=project_hand(p,nodes['h'])
    reference_angles=s[:,None]*limits
    fitted_q=np.array([r['q_deg'] for r in fitted_rows])
    fitted_old=np.radians([r['fitted_angles_deg'] for r in fitted_rows])
    fitted_angles=np.column_stack([np.interp(q,fitted_q,fitted_old[:,j]) for j in range(3)])
    radius=float(p.component_config['hand_mechanism_non_collision']['mechanism_link_radius_mm'])
    assert abs(radius-2.)<1e-12
    original_errors=np.linalg.norm(nodes['h']-targets,axis=-1)
    blends=[];witnesses={};new_worst=None;old_worst=None;all_sweeps=[]
    max_mismatch=0.;best_offset_metrics={o:None for o in [0.,.5,1.,1.5,2.,3.]}
    for alpha in np.linspace(0,1,blend_count):
        angles=reference_angles+(fitted_angles-reference_angles)*alpha
        hands=angle_hands(p,angles)
        old,names=collision(p,nodes,hands,samples=21,details=True,output_shape='l_bracket')
        keep=[i for i,name in enumerate(names) if not name.startswith('L_gh/')]
        untouched=old[:,keep];untouched_names=[names[i] for i in keep]
        replacement,replacement_names=relief_clearances(nodes,hands,offset_mm,terminal_run_mm,radius)
        new=np.concatenate((untouched,replacement),axis=1);newnames=untouched_names+replacement_names
        old_record=result_metric(old,names,q,s,alpha);new_record=result_metric(new,newnames,q,s,alpha)
        pos=np.array([endpoint(p,a) for a in angles]);mismatch=np.linalg.norm(pos-nodes['h'],axis=-1)
        max_mismatch=max(max_mismatch,float(max(mismatch)))
        record={'blend_fraction':float(alpha),'old':old_record,'new':new_record,
                'maximum_H_R4_mismatch_mm':float(max(mismatch))}
        blends.append(record)
        if old_worst is None or old_record['minimum_clearance_mm']<old_worst['minimum_clearance_mm']:old_worst=old_record
        if new_worst is None or new_record['minimum_clearance_mm']<new_worst['minimum_clearance_mm']:new_worst=new_record
        # Keep per-pose minima, not only a summary of favorable endpoints.
        all_sweeps.append({'blend_fraction':float(alpha),'q_deg':q.tolist(),'reference_progress':s.tolist(),
                           'old_minimum_clearance_mm':old.min(axis=1).tolist(),
                           'new_minimum_clearance_mm':new.min(axis=1).tolist(),
                           'H_R4_mismatch_mm':mismatch.tolist()})
        for label,metric in [('old',old_record),('new',new_record)]:
            if label not in witnesses or metric['minimum_clearance_mm']<witnesses[label]['minimum_clearance_mm']:
                idx=metric['pose_index'];witnesses[label]={**metric,'angles_deg':np.degrees(angles[idx]).tolist(),
                    'nodes_mm':{key:value[idx].tolist() for key,value in nodes.items()},
                    'hand_capsules_mm':hands[idx].tolist(),
                    'new_contour_mm':outline(nodes,offset_mm,terminal_run_mm)[idx].tolist()}
        # Explore only six contour offsets; do not change any kinematic dimensions.
        for candidate_offset in best_offset_metrics:
            replacement,cnames=relief_clearances(nodes,hands,candidate_offset,terminal_run_mm,radius)
            candidate=np.concatenate((untouched,replacement),axis=1)
            metric=result_metric(candidate,untouched_names+cnames,q,s,alpha)
            if best_offset_metrics[candidate_offset] is None or metric['minimum_clearance_mm']<best_offset_metrics[candidate_offset]['minimum_clearance_mm']:
                best_offset_metrics[candidate_offset]=metric
    c=copy.deepcopy(c)
    report={'finger':finger,'poses_per_blend':pose_count,'angle_blend_count':blend_count,
            'total_tested_configurations':pose_count*blend_count,
            'old':old_worst,'new':new_worst,'by_hand_pose_blend':blends,
            'maximum_reference_trajectory_error_mm':float(max(original_errors)),
            'maximum_H_R4_mismatch_over_tested_hand_poses_mm':max_mismatch,
            'trajectory_tolerance_mm':1.,'trajectory_passed':bool(max(original_errors)<=1.),
            'collision_passed':bool(new_worst['minimum_clearance_mm']>=0),
            'strict_closure_required':False,'all_holes_and_lengths_unchanged':True,
            'new_contour':{'type':'rigid_G_P_H_outline','terminal_run_mm':terminal_run_mm,
                'normal_offset_mm':offset_mm,'segment_radius_mm':radius,'H_exclusion_mm':4.,
                'P_is_new_joint':False,'FG_retained':True},
            'outline_offset_sweep':{str(key):value for key,value in best_offset_metrics.items()},
            'reference_path_policy':'Original proportional-flexion hand path retained.',
            'supplementary_pose_policy':'Linear interpolation in motor angle through previous fitted-hand samples; 11 angular blends from reference to fitted posture.',
            'sampled_planar_only':True,'arbitrary_human_poses_certified':False,
            '3d_collision_certified':False,'self_collision_certified':False,'strength_certified':False}
    report['passed_user_criteria']=report['trajectory_passed'] and report['collision_passed']
    c['baseline_validation']=c.pop('validation',None)
    c['validation']=report
    c['outline_relief_validation']=report;c['output_material_contour']=report['new_contour']
    c['output_shape']='rigid_G_P_H_outline'
    c['geometry_evaluator']='check_clearance.py (legacy triangle/L-bracket evaluator is not applicable)'
    c['status']='sampled_clearance_candidate_with_1mm_trajectory_tolerance' if report['passed_user_criteria'] else 'outline_candidate_failed'
    output.mkdir(parents=True,exist_ok=True)
    (output/'result.json').write_text(json.dumps(c,indent=2)+'\n')
    (output/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'witnesses.json').write_text(json.dumps(witnesses,indent=2)+'\n')
    (output/'clearance_sweeps.json').write_text(json.dumps(all_sweeps,indent=2)+'\n')
    print(json.dumps({'finger':finger,'old':old_worst,'new':new_worst,
          'max_reference_error':float(max(original_errors)),'max_pose_family_mismatch':max_mismatch,
          'offsets':best_offset_metrics,'passed':report['passed_user_criteria']}),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--finger',choices=FINGERS,required=True)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--poses',type=int,default=4801)
    parser.add_argument('--blends',type=int,default=11);parser.add_argument('--offset',type=float,default=2.)
    parser.add_argument('--terminal-run',type=float,default=10.)
    args=parser.parse_args();validate(args.finger,args.out,args.poses,args.blends,args.offset,args.terminal_run)


if __name__=='__main__':main()
