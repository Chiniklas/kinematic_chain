"""Regression checks for shape-only replacement of the GH collision envelope."""
import unittest
from check_clearance import *


class OutlineTests(unittest.TestCase):
    def test_zero_offset_recovers_original_collision_geometry(self):
        for f in FINGERS:
            c=json.loads((BASE/f/'result.json').read_text());_,p=load_design(f,f)
            nodes,_=fk(c['lengths_mm'],np.linspace(c['q_deg'][0],c['q_deg'][-1],91),p.ad_tilt_deg,c['output_branch'])
            s,_=project_hand(p,nodes['h']);hands=fast_hands(p,s)
            matrix,names=collision(p,nodes,hands,samples=21,details=True,output_shape='l_bracket')
            old=matrix[:,[i for i,name in enumerate(names) if name.startswith('L_gh/')]]
            new,_=relief_clearances(nodes,hands,0.)
            np.testing.assert_allclose(np.minimum(new[:,:4],new[:,4:]),old,atol=1e-10)

    def test_contour_is_rigid_without_moving_holes(self):
        for f in FINGERS:
            c=json.loads((BASE/f/'result.json').read_text());_,p=load_design(f,f)
            nodes,_=fk(c['lengths_mm'],np.linspace(c['q_deg'][0],c['q_deg'][-1],91),p.ad_tilt_deg,c['output_branch'])
            saved={k:v.copy() for k,v in nodes.items()};points=outline(nodes,2.)
            for k in nodes:np.testing.assert_array_equal(nodes[k],saved[k])
            for k in ['f','g','h']:
                distance=np.linalg.norm(points[:,1,:]-nodes[k],axis=1)
                np.testing.assert_allclose(distance,distance[0],atol=1e-10)


if __name__=='__main__':unittest.main()
