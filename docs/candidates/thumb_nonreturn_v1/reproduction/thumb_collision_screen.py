"""Sampled capsule/pin clearance screen retaining upstream axial plate layers.

Conservative for rods around holes. Does not model servo/carrier boxes, cuff,
spacers, screw heads, tolerances or swept solids. Not a CAD collision certificate.
"""
import numpy as np

PLATES=[('AB','ab',3.,3.5,6.),('BC','bc',4.,15.9,18.9),
        ('CDG','dcg',4.,12.8,15.3),('EF','ef',4.,6.6,9.1),('FGH','fgh',4.,20.,22.5),
        ('ADE','adea',5.,0.,3.)]
PINS=[('b',1.5,3.2,19.1),('c',2.,12.5,19.1),('d',2.,-.3,15.5),
      ('e',2.,-.3,9.3),('f',2.,6.3,22.8),('g',2.,12.5,22.8),('h',2.,19.7,25.8)]

def point_distance(p,a,b):
    v=b-a;t=np.clip(np.sum((p-a)*v,axis=-1)/(np.sum(v*v,axis=-1)+1e-20),0,1)
    return np.linalg.norm(p-(a+t[...,None]*v),axis=-1)

def cross(u,v):return u[...,0]*v[...,1]-u[...,1]*v[...,0]

def segments(a,b,c,d):
    distances=np.minimum.reduce([point_distance(a,c,d),point_distance(b,c,d),point_distance(c,a,b),point_distance(d,a,b)])
    crosses=(cross(b-a,c-a)*cross(b-a,d-a)<0)&(cross(d-c,a-c)*cross(d-c,b-c)<0)
    return np.where(crosses,0.,distances)

def clearances(points,m,ip,guidance=False):
    rows=[];names=[]
    for joint,r,z0,z1 in PINS:
        for name,nodes,radius,bottom,top in PLATES:
            if joint in nodes or min(z1,top)<=max(z0,bottom):continue
            for a,b in zip(nodes,nodes[1:]):
                rows.append(point_distance(points[joint],points[a],points[b])-r-radius)
                names.append('pin_'+joint.upper()+' / '+name)
    for i,(a,ra,za,ta) in enumerate(PINS):
        for b,rb,zb,tb in PINS[i+1:]:
            if min(ta,tb)>max(za,zb):
                rows.append(np.linalg.norm(points[a]-points[b],axis=1)-ra-rb);names.append('pin_'+a.upper()+' / pin_'+b.upper())
    m=np.radians(m);ip=np.radians(ip);root=np.tile([0.,-7.],(len(m),1))
    j=root+30*np.stack([np.cos(m),-np.sin(m)],axis=1)
    tip=j+40*np.stack([np.cos(m+ip),-np.sin(m+ip)],axis=1)
    hands=[]
    for name,a,b,angle,depth in [('proximal',root,j,-m,20),('distal',j,tip,-m-ip,10)]:
        normal=np.stack([-np.sin(angle),np.cos(angle)],axis=1)
        hands.append((name,a-depth/2*normal,b-depth/2*normal,depth/2))
    for name,nodes,r,z0,z1 in PLATES:
        if min(z1,10)<=max(z0,-10):continue
        for a,b in zip(nodes,nodes[1:]):
            for hn,ha,hb,hr in hands:
                if guidance:
                    for u in np.linspace(0,1,9):
                        sample=points[a]*(1-u)+points[b]*u
                        rows.append(point_distance(sample,ha,hb)-r-hr);names.append(name+' / hand_'+hn)
                else:
                    rows.append(segments(points[a],points[b],ha,hb)-r-hr);names.append(name+' / hand_'+hn)
    for joint,r,z0,z1 in PINS:
        if joint=='h' or min(z1,10)<=max(z0,-10):continue
        for hn,ha,hb,hr in hands:
            rows.append(point_distance(points[joint],ha,hb)-r-hr);names.append('pin_'+joint.upper()+' / hand_'+hn)
    return np.array(rows),names

def summarize(points,m,ip):
    values,names=clearances(points,m,ip);i,j=np.unravel_index(np.argmin(values),values.shape)
    return dict(minimum_clearance_mm=float(values[i,j]),worst_pair=names[i],worst_pose_index=int(j),
                passed=bool(np.min(values)>=-.01),sampled_poses=values.shape[1],
                exclusions='servo/carrier boxes, cuff, spacers, screw heads, tolerances and continuous swept solids')
