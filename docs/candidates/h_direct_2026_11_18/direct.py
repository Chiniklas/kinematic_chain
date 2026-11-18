"""Planar synthesis with H directly pinned to the distal hand attachment R4.

There is no output rod or perpendicularity objective in this model.
Reads repository profiles; never writes repository or hardware settings.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import minimize, minimize_scalar

from functools import lru_cache

HERE = Path(__file__).resolve().parent
if os.environ.get('KINEMATIC_CHAIN_REPO'):
    REPO = Path(os.environ['KINEMATIC_CHAIN_REPO']).resolve()
else:
    REPO = next((p for p in HERE.parents if (p / 'tools/_common.py').is_file()), None)
    if REPO is None:
        raise RuntimeError('Run inside kinematic_chain or set KINEMATIC_CHAIN_REPO.')
sys.path.insert(0, str(REPO / 'tools'))
from _common import load_run_adam


@lru_cache(maxsize=1)
def evaluator():
    return load_run_adam()


@lru_cache(maxsize=4)
def load_design(design, finger):
    if design != finger:
        raise ValueError('This experiment uses one independent profile per finger.')
    snapshot = HERE / 'input_snapshot' / finger
    module = evaluator()
    problem = module.load_problem(snapshot / 'objectives.yaml', snapshot / 'optimizable_variables.yaml')
    return module, problem

FINGERS = ('index', 'middle', 'ring', 'little')
BOUNDS = {'L_ab':(8,50),'L_bc':(8,100),'L_cd':(12.5,55),
          'L_ae':(8,130),'L_de':(7,55),'L_cg':(8,100),'L_dg':(12.5,100),
          'L_ef':(8,80),'L_fg':(8,65),'L_gh':(8,130),'L_fh':(8,130)}


def rot90(v):
    return np.stack((-v[...,1],v[...,0]),axis=-1)


def circle(a,ra,b,rb,sign):
    delta=b-a;distance=np.maximum(np.linalg.norm(delta,axis=-1),1e-12)
    unit=delta/distance[...,None]
    along=(ra*ra-rb*rb+distance*distance)/(2*distance)
    height=np.sqrt(np.maximum(ra*ra-along*along,1e-16))
    point=a+along[...,None]*unit+sign*height[...,None]*rot90(unit)
    margin=np.minimum(ra+rb-distance,distance-abs(ra-rb))
    return point,margin


def fk(lengths,q,tilt,branch=1):
    q=np.atleast_1d(q);n=len(q);L=lengths
    a=np.tile([-L['L_ad'],0.],(n,1));d=np.zeros((n,2))
    e,em=circle(a,L['L_ae'],d,L['L_de'],1)
    theta=np.radians(90-q)
    b=a+L['L_ab']*np.stack((np.cos(theta),np.sin(theta)),axis=-1)
    c,cm=circle(b,L['L_bc'],d,L['L_cd'],1)
    g,gm=circle(c,L['L_cg'],d,L['L_dg'],1)
    f,fm=circle(e,L['L_ef'],g,L['L_fg'],-1)
    h,hm=circle(g,L['L_gh'],f,L['L_fh'],branch)
    angle=math.radians(-tilt)
    rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    nodes={k:v@rotation.T for k,v in dict(a=a,b=b,c=c,d=d,e=e,f=f,g=g,h=h).items()}
    return nodes,np.stack((em,cm,gm,fm,hm),axis=-1)


def point_segment(p,a,b):
    axis=b-a;fraction=np.sum((p-a)*axis,axis=-1)/np.maximum(np.sum(axis*axis,axis=-1),1e-12)
    return np.linalg.norm(p-(a+np.clip(fraction,0,1)[...,None]*axis),axis=-1)


def segment_distance(a,b,c,d):
    result=np.minimum.reduce((point_segment(a,c,d),point_segment(b,c,d),point_segment(c,a,b),point_segment(d,a,b)))
    def cross(u,v):return u[...,0]*v[...,1]-u[...,1]*v[...,0]
    crossed=(cross(b-a,c-a)*cross(b-a,d-a)<=0)&(cross(d-c,a-c)*cross(d-c,b-c)<=0)
    boxes=np.all(np.maximum(np.minimum(a,b),np.minimum(c,d))<=np.minimum(np.maximum(a,b),np.maximum(c,d))+1e-12,axis=-1)
    return np.where(crossed&boxes,0,result)


def get_hands(m,p,s):
    return np.array([[[a,b,[radius,radius]] for _,a,b,radius in m._hand_capsules(p,p.objectives[0],float(t))] for t in s])


def collision(p,nodes,hands,attachment_mm=4.,samples=7,details=False,output_shape='filled_triangle'):
    """All moving bars and filled bodies against all hand capsules.

    The existing 12-mm D mount exemption is retained. The old 4-mm R4
    attachment exemption moves to H, trimming BOTH incident edges and only
    the triangle corner. No entire body or hand segment is excluded.
    """
    cfg=p.component_config['hand_mechanism_non_collision'];radius=cfg['mechanism_link_radius_mm']
    starts=hands[:,:,0,:];ends=hands[:,:,1,:];radii=hands[:,:,2,0]
    names=[];values=[]
    def trim(a,b,distance):
        delta=b-a;length=np.linalg.norm(delta,axis=-1,keepdims=True)
        return a+np.minimum(distance/np.maximum(length,1e-12),1)*delta
    for row in p.nominal_design['dimensions']:
        if row['body']=='ground':continue
        if output_shape=='l_bracket' and row['id']=='L_fh':continue
        u,v=row['nodes'];a=nodes[u].copy();b=nodes[v].copy()
        if u=='d':a=trim(a,b,cfg['dorsal_mount_exclusion_radius_mm'])
        if v=='d':b=trim(b,a,cfg['dorsal_mount_exclusion_radius_mm'])
        if u=='h':a=trim(a,b,attachment_mm)
        if v=='h':b=trim(b,a,attachment_mm)
        value=segment_distance(a[:,None,:],b[:,None,:],starts,ends)-radii-radius
        values.append(value);names.extend(f'{row["id"]}/{k}' for k in ['palm','proximal','middle','distal'])
    fractions=np.linspace(0,1,samples)
    hand_samples=starts[:,:,None,:]+fractions[None,None,:,None]*(ends-starts)[:,:,None,:]
    for row in p.nominal_design['bodies']:
        if row['kind']=='ground' or len(row['nodes'])<3:continue
        if output_shape=='l_bracket' and row['id']=='distal_body':continue
        ids=row['nodes'];poly=[]
        for j,key in enumerate(ids):
            distance=cfg['dorsal_mount_exclusion_radius_mm'] if key=='d' else attachment_mm if key=='h' else 0
            if distance:
                for other in (ids[j-1],ids[(j+1)%len(ids)]):poly.append(trim(nodes[key],nodes[other],distance))
            else:poly.append(nodes[key])
        inside=np.zeros(hand_samples.shape[:-1],dtype=bool);distances=[];exact=[]
        for j in range(len(poly)):
            a=poly[j][:,None,None,:];b=poly[j-1][:,None,None,:]
            distances.append(point_segment(hand_samples,a,b))
            x,y=hand_samples[...,0],hand_samples[...,1];ax,ay=a[...,0],a[...,1];bx,by=b[...,0],b[...,1]
            denominator=by-ay;denominator=np.where(abs(denominator)>1e-12,denominator,1e-12)
            inside^=((ay>y)!=(by>y))&(x<(bx-ax)*(y-ay)/denominator+ax)
            exact.append(segment_distance(poly[j][:,None,:],poly[j-1][:,None,:],starts,ends)-radii-radius)
        signed=np.min(distances,axis=0)*np.where(inside,-1,1)-radii[:,:,None]-radius
        value=np.minimum(np.min(signed,axis=-1),np.min(exact,axis=0))
        values.append(value);names.extend(f'{row["id"]}/{k}' for k in ['palm','proximal','middle','distal'])
    matrix=np.concatenate(values,axis=1)
    if details:return matrix,names
    return np.min(matrix,axis=1)


def nearest_progress(m,p,points):
    grid=np.linspace(0,1,1001)
    targets=np.array([m._hand_distal_contact_progress(p,p.objectives[0],float(s)) for s in grid])
    progress=[];errors=[]
    for point in points:
        j=int(np.argmin(np.sum((targets-point)**2,axis=-1)))
        def objective(s):
            delta=np.array(m._hand_distal_contact_progress(p,p.objectives[0],float(s)))-point
            return float(delta@delta)
        result=minimize_scalar(objective,bounds=(grid[max(0,j-1)],grid[min(1000,j+1)]),method='bounded',options={'xatol':1e-13})
        s,cost=min([(float(result.x),float(result.fun)),(0.,objective(0)),(1.,objective(1))],key=lambda v:v[1])
        progress.append(s);errors.append(math.sqrt(cost))
    return np.array(progress),np.array(errors)


def validate(candidate,count=1201):
    m,p=load_design(candidate['finger'],candidate['finger']);L=candidate['lengths_mm']
    q=np.linspace(candidate['q_deg'][0],candidate['q_deg'][-1],count)
    nodes,margins=fk(L,q,p.ad_tilt_deg,candidate.get('output_branch',1))
    s,errors=nearest_progress(m,p,nodes['h']);hands=get_hands(m,p,s)
    shape=candidate.get('output_shape','filled_triangle')
    matrix,names=collision(p,nodes,hands,samples=21,details=True,output_shape=shape)
    clear=matrix.min(axis=1);where=matrix.argmin(axis=1)
    link_errors={row['id']:float(np.max(abs(np.linalg.norm(nodes[row['nodes'][0]]-nodes[row['nodes'][1]],axis=-1)-L[row['id']]))) for row in p.nominal_design['dimensions']}
    max_step=max(float(np.max(np.linalg.norm(np.diff(nodes[k],axis=0),axis=-1))) for k in 'bcfgh')
    nearest_target=np.array([m._hand_distal_contact_progress(p,p.objectives[0],float(t)) for t in s])
    report={'poses':count,'maximum_endpoint_error_mm':float(max(errors)),'minimum_clearance_mm':float(min(clear)),
            'worst_clearance_pair':names[where[int(np.argmin(clear))]],'worst_clearance_progress':float(s[int(np.argmin(clear))]),
            'worst_error_progress':float(s[int(np.argmax(errors))]),'maximum_link_length_error_mm':max(link_errors.values()),
            'minimum_circle_triangle_margin_mm':float(margins.min()),'maximum_node_step_mm':max_step,
            'assembly_valid':bool(margins.min()>=.001-1e-7 and max(link_errors.values())<1e-6 and max_step<2),
            'monotonic_progress':bool(np.min(np.diff(s))>=-1e-7),'start_progress':float(s[0]),'terminal_progress':float(s[-1]),
            'endpoint_tolerance_mm':.1,'attachment_exclusion_mm':4.,'mechanism_radius_mm':2.,'output_shape':shape,
            'sampled_only':True,'continuous_certification':False,'three_dimensional_collision_checked':False,
            'joint_force_or_singularity_analysis_complete':False}
    report['trajectory_passed']=bool(max(errors)<=.1 and s[0]<.001 and s[-1]>=.99 and report['monotonic_progress'])
    report['collision_passed']=bool(min(clear)>=0)
    report['passed']=bool(report['trajectory_passed'] and report['collision_passed'] and report['assembly_valid'])
    rows=[{'q_deg':float(q[i]),'progress':float(s[i]),'endpoint_error_mm':float(errors[i]),'clearance_mm':float(clear[i]),'closest_pair':names[where[i]],'h_mm':nodes['h'][i].tolist(),'r4_target_mm':nearest_target[i].tolist()} for i in range(count)]
    return report,rows,nodes


def write_validated(candidate,out):
    out.mkdir(parents=True,exist_ok=True)
    report,rows,_=validate(candidate)
    candidate['validation']=report;candidate['status']='sampled_planar_candidate' if report['passed'] else 'rejected_search_candidate'
    (out/'result.json').write_text(json.dumps(candidate,indent=2))
    (out/'sweep.json').write_text(json.dumps(rows,indent=2))
    print(candidate['finger'],json.dumps(report),flush=True)


def convert(source,out):
    """Replace rigid F-G-H-R4 output by the triangle F-G-(new H=old R4)."""
    original=json.loads(source.read_text());finger=original['finger'];m,p=load_design(finger,finger)
    oldL=original['lengths_mm'];q=np.linspace(original['q_deg'][0],original['q_deg'][-1],1201)
    oldnodes,_=fk(oldL,q,p.ad_tilt_deg)
    gh=oldnodes['h']-oldnodes['g'];unit=gh/np.linalg.norm(gh,axis=-1,keepdims=True)
    oldR=oldnodes['h']+original['output_side']*oldL['L_tip_rod']*rot90(unit)
    L={k:v for k,v in oldL.items() if k!='L_tip_rod'}
    L['L_gh']=float(np.linalg.norm(oldR[0]-oldnodes['g'][0]));L['L_fh']=float(np.linalg.norm(oldR[0]-oldnodes['f'][0]))
    matches=[]
    for branch in [-1,1]:
        newnodes,_=fk(L,q,p.ad_tilt_deg,branch)
        matches.append((float(np.max(np.linalg.norm(newnodes['h']-oldR,axis=-1))),branch))
    difference,branch=min(matches)
    assert difference<1e-7, (finger,difference)
    candidate={'finger':finger,'lengths_mm':L,'q_deg':original['q_deg'],'progress':original['progress'],
               'output_branch':branch,'model':'H is the direct revolute attachment to distal hand point R4',
               'output_rod_present':False,'perpendicularity_constraint':False,'fixed_mount':original['fixed_mount'],
               'source':str(source),'conversion_max_endpoint_change_mm':difference,
               'changed_dimensions':{k:{'old_mm':oldL[k],'new_mm':L[k]} for k in ['L_gh','L_fh']},
               'removed_output_rod_mm':oldL['L_tip_rod']}
    write_validated(candidate,out)


def refine(source,out,maxiter=150,stages=(17,33,65),output_shape=None):
    seed=json.loads(source.read_text());finger=seed['finger'];m,p=load_design(finger,finger)
    if output_shape:seed['output_shape']=output_shape
    shape=seed.get('output_shape','filled_triangle')
    ids=list(BOUNDS);values=np.array([seed['lengths_mm'][k] for k in ids]);fixed=seed['lengths_mm']['L_ad']
    qold=np.array(seed['q_deg']);sold=np.array(seed['progress']);branch=seed['output_branch']
    out.mkdir(parents=True,exist_ok=True);last=0;start=time.monotonic();history=[]
    for n in stages:
        s=np.linspace(0,1,n);q=np.interp(s,sold,qold)
        targets=np.array([m._hand_distal_contact_progress(p,p.objectives[0],float(t)) for t in s]);hands=get_hands(m,p,s)
        cache={};best_z=None;best_t=float('inf')
        def evaluate(z):
            nonlocal cache
            if 'z' in cache and np.array_equal(z,cache['z']):return cache
            L={'L_ad':fixed,**dict(zip(ids,z[:len(ids)]))}
            nodes,margins=fk(L,z[len(ids):-1],p.ad_tilt_deg,branch)
            errors=np.linalg.norm(nodes['h']-targets,axis=-1);clear=collision(p,nodes,hands,output_shape=shape)
            cache=dict(z=z.copy(),lengths=L,nodes=nodes,margins=margins,errors=errors,clear=clear)
            return cache
        def constraints(z):
            ev=evaluate(z)
            return np.r_[z[-1]-ev['errors'],ev['clear']-.1,(ev['margins']-.001).ravel(),np.diff(z[len(ids):-1])-.01]
        def callback(z):
            nonlocal last,best_z,best_t
            conmin=float(min(constraints(z)))
            if conmin>=-1e-7 and z[-1]<best_t:best_z=z.copy();best_t=float(z[-1])
            if time.monotonic()-last>25:
                ev=evaluate(z);print('ITER',finger,n,round(time.monotonic()-start,1),json.dumps({'error':float(max(ev['errors'])),'clearance':float(min(ev['clear'])),'constraint_min':conmin}),flush=True);last=time.monotonic()
        z0=np.r_[values,q,1.]
        z0[-1]=max(evaluate(z0)['errors'])+.001
        bounds=[BOUNDS[k] for k in ids]+[(-25,115)]*n+[(0,10)]
        z0=np.clip(z0,np.array(bounds)[:,0],np.array(bounds)[:,1]);callback(z0)
        result=minimize(lambda z:z[-1],z0,method='SLSQP',bounds=bounds,constraints=[{'type':'ineq','fun':constraints}],callback=callback,options={'maxiter':maxiter,'ftol':1e-10})
        callback(result.x);chosen=best_z if best_z is not None else result.x;ev=evaluate(chosen)
        values=chosen[:len(ids)];qold=chosen[len(ids):-1];sold=s
        record={'n':n,'optimizer_success':bool(result.success),'message':str(result.message),'max_error_mm':float(max(ev['errors'])),'minimum_clearance_mm':float(min(ev['clear'])),'constraint_min':float(min(constraints(chosen)))}
        history.append(record);print('STAGE',finger,json.dumps(record),flush=True)
        candidate={**seed,'lengths_mm':{k:float(v) for k,v in ev['lengths'].items()},'q_deg':qold.tolist(),'progress':s.tolist(),
                   'optimizer':'SLSQP minimax endpoint error with collision and assembly constraints','search_history':history,
                   'source':str(source),'bounds_mm':BOUNDS,'elapsed_s':time.monotonic()-start}
        for k in ['validation','status','conversion_max_endpoint_change_mm','changed_dimensions']:candidate.pop(k,None)
        (out/f'stage_{n}.json').write_text(json.dumps(candidate,indent=2))
    write_validated(candidate,out)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['convert','refine','validate'])
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--maxiter',type=int,default=150);parser.add_argument('--stages',default='17,33,65')
    parser.add_argument('--output-shape',choices=['filled_triangle','l_bracket'])
    args=parser.parse_args()
    if args.action=='convert':convert(args.source,args.out)
    elif args.action=='refine':refine(args.source,args.out,args.maxiter,tuple(map(int,args.stages.split(','))),args.output_shape)
    else:
        candidate=json.loads(args.source.read_text())
        if args.output_shape:candidate['output_shape']=args.output_shape
        write_validated(candidate,args.out)


if __name__=='__main__':main()
