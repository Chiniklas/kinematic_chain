"""Plot the four measured mounts in a common horizontal coordinate system."""
from pathlib import Path
import sys,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools'))
from _common import load_design, initial_vector
HERE=Path(__file__).resolve().parent
summary=json.loads((HERE/'summary.json').read_text())
fingers=('index','middle','ring','little');items={}
for f in fingers:
 m,p=load_design(f,f);r=json.loads((HERE/f/'result.json').read_text());ev=r['evaluated'];key=p.objectives[0].id
 q=np.array(ev['input_schedules_deg'][key]);s=np.array(ev['hand_progress_schedules'][key]);L=m._candidate_lengths(p,initial_vector(p))
 states=m._phase_workspace_poses(L,q,p.ad_tilt_deg,p.input_phase_deg)
 assert len(states)==1201
 items[f]=(m,p,r,states,s)

edges=[('a','b'),('b','c'),('c','d'),('c','g'),('d','g'),('e','f'),('f','g'),('g','h'),('f','h')]
def draw(ax,f,indices):
 m,p,r,states,progress=items[f];offset=r['input']['finger_root_forward_offset_mm']
 ax.clear();ax.set_xlim(-80,118);ax.set_ylim(-90,86);ax.set_aspect('equal');ax.grid(alpha=.15)
 ax.axvline(0,color='#adb5bd',lw=.8,ls=':');ax.axhline(0,color='#adb5bd',lw=.8,ls=':')
 for order,i in enumerate(indices):
  alpha=1 if order==0 else .32
  nodes={k:np.array(v)+[offset,0] for k,v in states[i][1].items()}
  hand,_=m._hand_joint_geometry(p,p.objectives[0],float(progress[i]));hand=np.array(hand)+[offset,0]
  r4=np.array(m._hand_distal_contact_progress(p,p.objectives[0],float(progress[i])))+[offset,0]
  ax.plot(hand[:,0],hand[:,1],'-o',color='#85939f',lw=8,markersize=4,alpha=alpha*.6,zorder=1)
  for u,v in [('a','d'),('d','e'),('e','a')]:
   xy=np.array([nodes[u],nodes[v]]);ax.plot(*xy.T,color='#495057',lw=2,alpha=alpha)
  for u,v in edges:
   xy=np.array([nodes[u],nodes[v]]);color='#e29528' if (u,v)==('a','b') else ('#278c66' if u in ('f','g') else '#3478b9')
   ax.plot(*xy.T,'-o',color=color,lw=1.6,markersize=3,alpha=alpha)
  xy=np.array([nodes['h'],r4]);ax.plot(*xy.T,color='#d74747',lw=2,alpha=alpha)
  if order==0:
   for name in ('a','d','e','h'):
    xy=nodes[name];ax.annotate(name.upper(),xy,xytext=(4,5),textcoords='offset points',fontsize=8)
   ax.annotate('MCP',hand[0],xytext=(4,-12),textcoords='offset points',fontsize=8)
 ax.set_title(f'{f.title()} | root {offset:+g} mm | AD {r["mount"]["L_ad_mm"]:.3f} mm',fontsize=11,fontweight='bold')
 ax.set_xlabel('Horizontal position relative to index root (mm)',fontsize=9)
 ax.set_ylabel('Existing vertical coordinate (mm)',fontsize=9)
 e=r['evaluated'];ax.text(.02,.03,f'Rod {r["lengths_mm"]["L_tip_rod"]:.2f} mm | clearance {e["minimum_clearance_mm"]:.3f} mm\nMax closure error {e["maximum_rod_closure_error_mm"]:.4f} mm',transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))

fig,axes=plt.subplots(2,2,figsize=(12,10),layout='constrained')
fig.suptitle('Independent four-finger horizontal fit\nMotor-hole groups remain 50 mm apart; vertical geometry unchanged',fontsize=14,fontweight='bold')
for ax,f in zip(axes.flat,fingers):draw(ax,f,[0,600,1200])
fig.savefig(HERE/'comparison.png',dpi=160)
def frame(k):
 for ax,f in zip(axes.flat,fingers):draw(ax,f,[k])
forward=np.linspace(0,1200,32,dtype=int);sequence=np.r_[forward,forward[-2:0:-1]]
anim=FuncAnimation(fig,frame,frames=sequence,interval=90)
anim.save(HERE/'motion.gif',writer=PillowWriter(fps=12),dpi=80)
plt.close(fig)
print('Rendered comparison.png and motion.gif')
