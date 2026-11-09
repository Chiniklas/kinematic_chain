from __future__ import annotations
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def render(parts,path,state,exploded=False):
    figure=plt.figure(figsize=(12,9),facecolor='#f3f5f8')
    ax=figure.add_subplot(111,projection='3d')
    ax.set_proj_type('ortho')
    all_points=[]
    for index,part in enumerate(parts):
        vertices,triangles=part['shape'].val().tessellate(.16,.2)
        points=np.array([v.toTuple() for v in vertices])
        if exploded:
            # Separate axial plate layers, preserving every part's own shape.
            z=points[:,2].mean()
            points[:,2]+=z*1.8
            if part['name'].startswith('11_') or part['name'].startswith('12_'):
                points[:,0]-=15
        all_points.append(points)
        faces=points[np.array(triangles)]
        normals=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0])
        norms=np.linalg.norm(normals,axis=1)
        normals/=np.maximum(norms[:,None],1e-12)
        light=np.array([-.4,-.2,1.]);light/=np.linalg.norm(light)
        strength=.55+.4*np.clip(normals@light,0,1)
        colors=strength[:,None]*np.array(to_rgb(part['color']))
        alpha=.14 if part['kind']=='human_context' else 1
        collection=Poly3DCollection(faces,facecolors=colors,edgecolors='none',alpha=alpha,zorder=index)
        ax.add_collection3d(collection)
    points=np.concatenate(all_points)
    lo,hi=points.min(0),points.max(0)
    center=(lo+hi)/2;ranges=(hi-lo)*1.12
    ax.set_xlim(center[0]-ranges[0]/2,center[0]+ranges[0]/2)
    ax.set_ylim(center[1]-ranges[1]/2,center[1]+ranges[1]/2)
    ax.set_zlim(center[2]-ranges[2]/2,center[2]+ranges[2]/2)
    ax.set_box_aspect(ranges,zoom=1.0);ax.view_init(elev=27,azim=-65,roll=0)
    ax.set_axis_off()
    ax.set_facecolor('#f3f5f8')
    figure.suptitle('THUMB MODULE  /  '+('NEAR STRAIGHT 2 / 3 DEG' if state=='straight' else state.upper().replace('_',' ')),fontsize=18,fontweight='bold',y=.94)
    figure.text(.5,.075,'30 + 40 mm thumb  |  optimized pivot pitches  |  side-mounted servo  |  STEP solid prototype',
                ha='center',fontsize=10,color='#495467')
    figure.text(.5,.045,'Mounting, axial stack, saddle and catalog hardware remain provisional.',
                ha='center',fontsize=10,color='#7b4f28')
    figure.subplots_adjust(left=0,right=1,top=.91,bottom=.1)
    figure.savefig(path,dpi=160,facecolor=figure.get_facecolor())
    plt.close(figure)
