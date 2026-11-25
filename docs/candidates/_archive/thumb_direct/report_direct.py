"""Independent closed-loop checks, motion CSV and GIF for the direct thumb."""
import json,csv,io
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from optimize_direct import poses,ik,R,target
ROOT=Path(__file__).resolve().parent
p=json.loads((ROOT/'parameters.json').read_text());x=np.array(p['direct_synthesis'])
s=np.linspace(0,1,1001);ps,violation=poses(x,s);m,ip,r=ik(ps['h'])
mr=np.deg2rad(m);ir=np.deg2rad(ip)
o=np.tile([0.,-7.],(len(s),1))
j=o+30*np.stack([np.cos(mr),-np.sin(mr)],-1)
tip=j+40*np.stack([np.cos(mr+ir),-np.sin(mr+ir)],-1)
h=ps['h']@R
closure=np.linalg.norm(h-(j+tip)/2,axis=1)
pitch=[]
for k,l in p['lengths'].items():
    a,b=k[2:];pitch.extend(abs(np.linalg.norm(ps[a]-ps[b],axis=1)-l))
assert np.max(violation)<1e-9 and np.max(closure)<1e-9 and max(pitch)<1e-9
assert np.min(np.diff(m))>0 and np.min(np.diff(ip))>0
assert np.max(abs(np.array([m[-1],ip[-1]])-[45,60]))<.01
# Singularities: intersections must remain strictly separated from tangency.
minimum_sine={}
for name,a,b,c in [('BCD','b','c','d'),('EFG','e','f','g')]:
    u=ps[a]-ps[b];v=ps[c]-ps[b]
    sine=abs(u[:,0]*v[:,1]-u[:,1]*v[:,0])/(np.linalg.norm(u,axis=1)*np.linalg.norm(v,axis=1))
    minimum_sine[name]=float(sine.min())
assert min(minimum_sine.values())>.01
report=json.loads((ROOT/'kinematic_validation.json').read_text())
report.update({'maximum_direct_closure_error_mm':float(closure.max()),'maximum_rigid_pitch_error_mm':float(max(pitch)),
 'minimum_transmission_triangle_sine':minimum_sine,'motor_travel_deg':float(np.rad2deg(x[-1])),
 'thumb_minimum_jacobian_determinant_mm2':float((600*np.sin(ir)).min()),
 'model':'fixed MCP root; distal cuff midpoint; downward continuous 2R branch; no HR4 rod',
 'initial_AB_angle_relative_AD_deg':float(np.rad2deg(x[1]))})
(ROOT/'kinematic_validation.json').write_text(json.dumps(report,indent=2))
with (ROOT/'motion.csv').open('w',newline='',encoding='utf-8-sig') as f:
    writer=csv.writer(f);writer.writerow(['motor_travel_deg','MCP_deg','IP_deg','H_x_mm','H_y_mm','tip_x_mm','tip_y_mm'])
    writer.writerows(np.c_[np.rad2deg(x[-1]*s),m,ip,h,tip])
with (ROOT/'link_dimensions.csv').open('w',newline='',encoding='utf-8-sig') as f:
    writer=csv.writer(f);writer.writerow(['pivot_pair','length_mm']);writer.writerows((k[2:].upper(),v) for k,v in p['lengths'].items())
frames=[]
for n in np.linspace(0,1000,41,dtype=int):
    fig,ax=plt.subplots(figsize=(7,6));pts={k:v[n]@R for k,v in ps.items()}
    for nodes,col in [('adea','#8c939c'),('ab','#426da9'),('bc','#74a2b5'),('dcg','#727d88'),('ef','#d08b35'),('fgh','#ce714d')]:
        xy=np.array([pts[k] for k in nodes]);ax.plot(*xy.T,'o-',color=col,lw=3,ms=4)
    ax.plot(*np.array([o[n],j[n],tip[n]]).T,'o-',color='#a37553',lw=7,alpha=.55)
    ax.plot(*h[n],'o',color='#7f5293',ms=10)
    for k,v in pts.items():ax.annotate(k.upper(),v,xytext=(4,5),textcoords='offset points',fontsize=9)
    ax.set(xlim=(-55,105),ylim=(-85,75),aspect='equal',xlabel='mm',ylabel='mm',
        title=f'Direct H = cuff pivot | no output rod\nMotor {np.rad2deg(x[-1]*s[n]):.1f} deg   MCP {m[n]:.1f} deg   IP {ip[n]:.1f} deg')
    ax.grid(alpha=.2);fig.tight_layout();buf=io.BytesIO();fig.savefig(buf,format='png',dpi=100);plt.close(fig)
    frames.append(Image.open(buf).convert('RGB'))
frames[0].save(ROOT/'thumb_direct.gif',save_all=True,append_images=frames[1:]+frames[-2:0:-1],duration=85,loop=0)
print(json.dumps(report,indent=2))
