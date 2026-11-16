"""Direct H-to-distal-cuff synthesis. All dimensions mm; no output rod.
Seed geometry is reparameterized in the initial pose, so initial closure is exact.
Target angular interpolation is only a synthesis objective; delivered FK uses IK.
"""
import json, math
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parent
P=json.loads((ROOT/'seed_parameters.json').read_text())
tilt=np.deg2rad(P['fixed']['ad_tilt_deg'])
R=np.array([[np.cos(tilt),-np.sin(tilt)],[np.sin(tilt),np.cos(tilt)]])
A=np.array([-21.112,0.]);D=np.zeros(2)
def target(s):
    m=np.deg2rad(2+43*s);i=np.deg2rad(3+57*s)
    return np.stack([30*np.cos(m)+20*np.cos(m+i),-7-30*np.sin(m)-20*np.sin(m+i)],-1)@R.T
H0=target(0.)
def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]
def perp(a):return np.stack([-a[...,1],a[...,0]],-1)
def circle(a,ra,b,rb,sign):
    delta=b-a;d=np.linalg.norm(delta,axis=-1);u=delta/np.maximum(d[...,None],1e-10)
    x=(ra*ra-rb*rb+d*d)/(2*np.maximum(d,1e-10));h2=ra*ra-x*x
    return a+x[...,None]*u+sign*np.sqrt(np.maximum(h2,0))[...,None]*perp(u),np.maximum(-h2,0)
def setup(x):
    ab,angle,cx,cy,ex,ey,gx,gy,fx,fy,travel=x
    b=A+ab*np.array([np.cos(angle),np.sin(angle)])
    c=np.array([cx,cy]);e=np.array([ex,ey]);g=np.array([gx,gy]);f=np.array([fx,fy])
    return b,c,e,g,f
def poses(x,s):
    b,c,e,g,f=setup(x);ab,angle=x[:2]
    bc=np.linalg.norm(c-b);cd=np.linalg.norm(c);ef=np.linalg.norm(f-e);fg=np.linalg.norm(f-g)
    bs=A+ab*np.stack([np.cos(angle-x[-1]*s),np.sin(angle-x[-1]*s)],-1)
    cs,v1=circle(bs,bc,D,cd,np.sign(cross(D-b,c-b)))
    # CDG rotates rigidly around D; FGH rotates rigidly with GF.
    co=(cs@c)/(cd*cd);si=cross(c,cs)/(cd*cd)
    gs=co[...,None]*g+si[...,None]*perp(g)
    fs,v2=circle(e,ef,gs,fg,np.sign(cross(g-e,f-e)))
    gf=f-g;hs=H0-g;u=fs-gs
    hp=gs+(hs@gf)/(gf@gf)*u+cross(gf,hs)/(gf@gf)*perp(u)
    return dict(a=np.broadcast_to(A,bs.shape),b=bs,c=cs,d=np.zeros_like(bs),e=np.broadcast_to(e,bs.shape),f=fs,g=gs,h=hp),np.concatenate([v1,v2])
def ik(h):
    h=h@R;v=h-np.array([0,-7]);r2=np.sum(v*v,axis=-1)
    ip=np.arccos(np.clip((r2-1300)/1200,-1,1))
    m=-np.arctan2(v[...,1],v[...,0])-np.arctan2(20*np.sin(ip),30+20*np.cos(ip))
    return np.rad2deg(m),np.rad2deg(ip),np.sqrt(r2)
def lengths(x):
    b,c,e,g,f=setup(x);pts=dict(a=A,b=b,c=c,d=D,e=e,g=g,f=f,h=H0)
    return {'L_'+k:float(np.linalg.norm(pts[k[0]]-pts[k[1]])) for k in ['ab','bc','cd','ad','ae','de','cg','dg','ef','fg','gh','fh']}
grid=np.linspace(0,1,41)
def residual(x):
    ps,viol=poses(x,grid);h=ps['h'];m,ip,r=ik(h)
    err=(h-target(grid));err[-1]*=30
    ls=np.array(list(lengths(x).values()))
    return np.concatenate([err.ravel(),viol*3,np.maximum(r-49.999,0)*100,
        np.maximum(10.001-r,0)*100,np.minimum(np.diff(m),0)*4,np.minimum(np.diff(ip),0)*4,
        np.maximum(m-45,0)*5,np.maximum(ip-60,0)*5,
        np.maximum(8-ls,0)*3,np.maximum(ls-85,0)*3])
def main():
    # Original reference assembly evaluated without importing CAD.
    l=P['lengths'];b=A+np.array([0,l['L_ab']]);c,_=circle(b,l['L_bc'],D,l['L_cd'],-1)
    # Select original branch explicitly by comparing circle solutions.
    c=max([circle(b,l['L_bc'],D,l['L_cd'],s)[0] for s in [-1,1]],key=lambda p:p[1])
    e=max([circle(A,l['L_ae'],D,l['L_de'],s)[0] for s in [-1,1]],key=lambda p:p[1])
    g=max([circle(c,l['L_cg'],D,l['L_dg'],s)[0] for s in [-1,1]],key=lambda p:p[0])
    f=min([circle(e,l['L_ef'],g,l['L_fg'],s)[0] for s in [-1,1]],key=lambda p:p[1])
    x0=np.r_[l['L_ab'],np.pi/2,c,e,g,f,.45]
    lo=[10,.3,-30,3,-35,3,5,5,-30,-35,.12];hi=[28,2.7,25,35,25,45,90,90,80,70,1.4]
    rng=np.random.default_rng(20261018);best=None
    for n in range(12):
        start=x0 if n==0 else np.clip(best.x+rng.normal(size=11)*np.array([2,.15,4,4,4,4,6,6,6,6,.12]),np.array(lo)+1e-5,np.array(hi)-1e-5)
        start=np.clip(start,np.array(lo)+1e-5,np.array(hi)-1e-5)
        fit=least_squares(residual,start,bounds=(lo,hi),max_nfev=1400,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        if best is None or np.linalg.norm(fit.fun)<np.linalg.norm(best.fun):best=fit
        print(n,float(np.linalg.norm(best.fun)),flush=True)
        (ROOT/'search_result.json').write_text(json.dumps({'x':best.x.tolist(),'residual_norm':float(np.linalg.norm(best.fun)),'lengths':lengths(best.x)},indent=2))
        if np.linalg.norm(best.fun)<.02:break
    s=np.linspace(0,1,1001);ps,v=poses(best.x,s);m,i,r=ik(ps['h'])
    report={'mcp_range':[float(m.min()),float(m.max())],'ip_range':[float(i.min()),float(i.max())],
      'endpoint_angles':[float(m[-1]),float(i[-1])],'minimum_mcp_increment':float(np.diff(m).min()),
      'minimum_ip_increment':float(np.diff(i).min()),'r_range':[float(r.min()),float(r.max())],
      'circle_violation':float(v.max()),'maximum_target_position_error':float(np.linalg.norm(ps['h']-target(s),axis=1).max()),'sample_count':1001}
    (ROOT/'kinematic_validation.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
if __name__=='__main__':main()
