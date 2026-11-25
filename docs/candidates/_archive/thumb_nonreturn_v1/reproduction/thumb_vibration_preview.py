"""Scheme 1: retain original output direction; reserve distal-pad module/strap.
Module size is a declared placeholder, not supplied hardware dimensions.
"""
from pathlib import Path
import json,math,time
import numpy as np
from scipy.optimize import least_squares
import thumb_antifold_experiment as t
from thumb_collision_screen import PLATES,PINS,point_distance,segments,clearances

WORK=t.ROOT/'work/thumb_vibration';WORK.mkdir(parents=True,exist_ok=True)
OUT=t.ROOT/'generated/thumb_vibration_preview';OUT.mkdir(parents=True,exist_ok=True)
ASSUMPTIONS={
 'confirmed_location':'distal thumb pad / palmar side',
 'confirmed_attachment':'strap connects vibration unit to linkage',
 'module_dimensions_mm':{'along_finger':14,'across_finger':12,'height':6},
 'dimensions_status':'ASSUMED PLACEHOLDER for preview, user has not supplied sizes',
 'module_center':'distal phalanx midpoint, directly beneath H on palmar side',
 'distal_depth_mm':10,'strap_width_mm':8,'strap_thickness_mm':1,
 'clearance_target_mm':2,
 'screen':'conservative side-view projected capsule/rectangle reserve across all linkage layers',
 'allowed_contact':'last 12 mm of GH and H pin may meet strap; module itself always excluded',
 'ground':'original A,D,E, AD length and tilt fixed',
 'orientation':'initial H->G within 15deg and G->F within 30deg of original; all-stroke H->G within 25deg of original at equal normalized motor progress',
 'not_checked':'real module, full strap routing/contact, motor/carrier/cuff solids, tolerances and 3D CAD',
}
MODULE=(-7.,7.,-16.,-10.)
BELT=(-4.,4.,-17.,1.)
GRID=t.GRID

def axes(m,ip):
    a=np.radians(m+ip)
    return np.stack([np.cos(a),-np.sin(a)],axis=-1),np.stack([np.sin(a),np.cos(a)],axis=-1)

def local(p,h,e,n):
    v=p-h
    return np.stack([np.sum(v*e,axis=-1),np.sum(v*n,axis=-1)],axis=-1)

def signed_box(p,box):
    x0,x1,y0,y1=box;c=np.array([(x0+x1)/2,(y0+y1)/2]);half=np.array([(x1-x0)/2,(y1-y0)/2])
    q=np.abs(p-c)-half
    return np.linalg.norm(np.maximum(q,0),axis=-1)+np.minimum(np.max(q,axis=-1),0)

def segment_box(a,b,box):
    x0,x1,y0,y1=box;verts=[np.tile(p,(len(a),1)) for p in [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]]
    ds=[segments(a,b,verts[i],verts[(i+1)%4]) for i in range(4)]
    return np.minimum.reduce(ds+[signed_box(a,box),signed_box(b,box)])

def reserve_clearances(pts,m,ip,guidance=False):
    e,n=axes(m,ip);p={k:local(v,pts['h'],e,n) for k,v in pts.items()}
    values=[];names=[]
    for label,box in [('module',MODULE),('strap',BELT)]:
        for name,nodes,r,_,_ in PLATES:
            for a,b in zip(nodes,nodes[1:]):
                pa,pb=p[a],p[b]
                if label=='strap' and 'h' in (a,b):
                    if b!='h':pa,pb=pb,pa
                    v=pa-pb;length=np.linalg.norm(v,axis=-1)
                    pb=pb+v*np.minimum(12/(length+1e-12),1)[:,None]
                if guidance:
                    u=np.linspace(0,1,13)[:,None,None]
                    samples=pa[None]*(1-u)+pb[None]*u
                    distance=np.min(signed_box(samples,box),axis=0)
                else:distance=segment_box(pa,pb,box)
                values.append(distance-r);names.append(name+'/'+label)
        for name,r,_,_ in PINS:
            if label=='strap' and name=='h':continue
            values.append(signed_box(p[name],box)-r);names.append('pin_'+name+'/'+label)
    return np.array(values),names

REF=t.geometry(t.X0,GRID)[0]
HG_REF=REF['g']-REF['h'];HG_REF/=np.linalg.norm(HG_REF,axis=-1)[:,None]
GF_REF=REF['f'][0]-REF['g'][0];GF_REF/=np.linalg.norm(GF_REF)

def angular_deviation(u,v):
    u=u/(np.linalg.norm(u,axis=-1)[...,None]+1e-15)
    v=v/(np.linalg.norm(v,axis=-1)[...,None]+1e-15)
    return np.degrees(np.arccos(np.clip(np.sum(u*v,axis=-1),-1,1)))

def residual(z,weight):
    base=t.residual(z,'orientation')
    x=t.unpack(z);pts,_,m,ip,_=t.geometry(x,GRID)
    hg=pts['g']-pts['h'];hgu=hg/np.linalg.norm(hg,axis=-1)[:,None]
    gf=pts['f'][0]-pts['g'][0];gfu=gf/np.linalg.norm(gf)
    e,n=axes(m,ip)
    penalties=[base,np.array([m[-1]-45,ip[-1]-60])*100,
      np.array([max(math.cos(math.radians(14))-hgu[0]@HG_REF[0],0),
                max(math.cos(math.radians(29))-gfu@GF_REF,0)])*150,
      np.maximum(math.cos(math.radians(24))-np.sum(hgu*HG_REF,axis=-1),0)*100,
      np.maximum(3-np.sum(hg*n,axis=-1),0)*15,
      np.array([max(3-hg[0,0],0),max(2-gf[0],0)])*15]
    ix=np.unique(np.r_[np.arange(0,len(GRID),3),len(GRID)-1])
    small={k:v[ix] for k,v in pts.items()}
    real_clearance,_=clearances(small,m[ix],ip[ix],guidance=False)
    penalties.append(np.maximum(.3-real_clearance,0).ravel()*120)
    distances,_=reserve_clearances(small,m[ix],ip[ix],guidance=True)
    penalties.append(np.maximum(2.2-distances,0).ravel()*12)
    # Soft size objective only. Never reverse the output to meet a size cap.
    length=t.model.lengths(x)
    physical=np.array([length['L_'+p] for p in ['ab','bc','cd','cg','ef','fg','gh']])
    penalties.append(physical*weight)
    arr=np.stack([pts[k] for k in ['b','c','f','g','h']])
    penalties.append(np.maximum(arr[:,:,0]-65,0).ravel()*weight*.7)
    penalties.append(np.maximum(arr[:,:,1]-35,0).ravel()*weight*.7)
    return np.concatenate(penalties)

def evaluate(x,n=1201):
    r=t.evaluate(x,'orientation',n=n)
    s=np.linspace(0,1,n);pts,_,m,ip,_=t.geometry(x,s);ref=t.geometry(t.X0,s)[0]
    hg=pts['g']-pts['h'];gf=pts['f'][0]-pts['g'][0]
    hgdev=angular_deviation(hg,ref['g']-ref['h']);gfdev=float(angular_deviation(gf,GF_REF))
    _,normal=axes(m,ip);dorsal=np.sum(hg*normal,axis=-1)
    values,names=reserve_clearances(pts,m,ip);idx,j=np.unravel_index(np.argmin(values),values.shape)
    r['checks'].update(initial_output_direction=bool(hgdev[0]<=15 and gfdev<=30 and hg[0,0]>=2.99 and gf[0]>=1.99),
                       all_pose_output_direction=bool(max(hgdev)<=25 and min(dorsal)>=2.99),
                       vibration_and_strap_reserve=bool(np.min(values)>=2.-1e-6),
                       preview_layer_clearance_margin=bool(r['collision_screen']['minimum_clearance_mm']>=.1))
    r['passed']=all(r['checks'].values())
    r['initial_HG_deviation_deg']=float(hgdev[0]);r['initial_GF_deviation_deg']=gfdev
    r['maximum_HG_deviation_deg']=float(max(hgdev));r['minimum_dorsal_HG_mm']=float(min(dorsal))
    r['reserve_clearance_mm']=float(values[idx,j]);r['reserve_worst_pair']=names[idx];r['reserve_worst_index']=int(j)
    r['physical_moving_link_sum_mm']=sum(r['lengths_mm']['L_'+p] for p in ['ab','bc','cd','cg','ef','fg','gh'])
    return r

def rank(r):
    return (not r['passed'],sum(not v for v in r['checks'].values()),
            r['physical_moving_link_sum_mm']+.3*r['trajectory_max_error_mm']+max(2-r['reserve_clearance_mm'],0)*50)

def main():
    (OUT/'assumptions.json').write_text(json.dumps(ASSUMPTIONS,indent=2),encoding='utf-8')
    old=evaluate(t.X0,n=5001);(OUT/'original_checked.json').write_text(json.dumps(old,indent=2),encoding='utf-8')
    rng=np.random.default_rng(20261117);results=[];saved=WORK/'search.json'
    if saved.exists():results=json.loads(saved.read_text())['trials']
    for trial in range(len(results),10):
        x=t.X0.copy();weight=[.004,.009,.015,.025][trial%4]
        if trial<5:
            k=[1,.85,.70,.55,.40][trial]
            x[6:8]=t.model.H0+k*(x[6:8]-t.model.H0)
            x[8:10]=t.model.H0+k*(x[8:10]-t.model.H0)
        else:
            x=np.array(min(results,key=rank)['x']);x[t.FREE]+=rng.normal(size=9)*t.SCALE*(.03 if trial%3 else .15)
        z=np.clip(x[t.FREE],t.LOW+1e-6,t.HIGH-1e-6);start=time.time()
        fit=least_squares(lambda z:residual(z,weight),z,bounds=(t.LOW,t.HIGH),x_scale='jac',max_nfev=1000,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        r=evaluate(t.unpack(fit.x));r.update(trial=trial,size_weight=weight,seconds=time.time()-start)
        results.append(r);best=min(results,key=rank)
        saved.write_text(json.dumps({'best':best,'trials':results},indent=2),encoding='utf-8')
        print(trial,r['passed'],'direction',round(r['initial_HG_deviation_deg'],2),'gap',round(r['reserve_clearance_mm'],3),
              'length',round(r['physical_moving_link_sum_mm'],1),'error',round(r['trajectory_max_error_mm'],3),
              [k for k,v in r['checks'].items() if not v],flush=True)
    chosen=evaluate(np.array(min(results,key=rank)['x']),n=5001)
    (OUT/'selected.json').write_text(json.dumps(chosen,indent=2),encoding='utf-8')

if __name__=='__main__':main()
