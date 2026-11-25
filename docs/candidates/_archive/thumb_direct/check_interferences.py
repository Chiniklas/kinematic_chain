"""Check prototype B-rep intersections at sampled poses, excluding human context."""
from pathlib import Path
import json
import itertools
import numpy as np

from build_thumb import P,L,ROOT,mechanism,make_hardware


def overlap(a,b):
    aa=a.val().BoundingBox();bb=b.val().BoundingBox()
    return all(min(getattr(aa,k+'max'),getattr(bb,k+'max'))-
               max(getattr(aa,k+'min'),getattr(bb,k+'min'))>1e-6 for k in 'xyz')


def main():
    poses=mechanism(P['q_schedule_deg'])
    indices=np.linspace(0,len(poses)-1,13,dtype=int)
    collisions=[];checked=0;dimensions=[]
    for index in indices:
        parts=make_hardware(poses[index],P['hand_progress'][index])
        for left,right in itertools.combinations(parts,2):
            if not overlap(left['shape'],right['shape']):
                continue
            checked+=1
            common=left['shape'].val().intersect(right['shape'].val())
            volume=sum(s.Volume() for s in common.Solids())
            if volume>1e-4:
                collisions.append({'pose_index':int(index),'q_deg':P['q_schedule_deg'][index],
                                   'part1':left['name'],'part2':right['name'],'intersection_volume_mm3':volume})
        for name,a,b in [('L_ab','a','b'),('L_bc','b','c'),('L_cd','c','d'),('L_ad','a','d'),
                         ('L_ae','a','e'),('L_de','d','e'),('L_cg','c','g'),('L_dg','d','g'),
                         ('L_ef','e','f'),('L_fg','f','g'),('L_gh','g','h'),('L_fh','f','h')]:
            measured=float(np.linalg.norm(poses[index][a]-poses[index][b]))
            dimensions.append({'pose_index':int(index),'id':name,'distance_mm':measured,'error_mm':abs(measured-L[name])})
        print(f'checked CAD pose {index}: cumulative overlaps {len(collisions)}',flush=True)
    report={'sampled_poses':13,'narrow_phase_pair_checks':checked,
            'interference_volume_threshold_mm3':1e-4,'interferences':collisions,
            'maximum_joint_pitch_error_mm':max(r['error_mm'] for r in dimensions),
            'excluded':'human context; no real threads, deformation, joint tolerances, continuous swept-volume certification',
            'note':'Hardware pins / spacers and motor envelope are conceptual dimensions.'}
    (ROOT/'interference_check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    main()
