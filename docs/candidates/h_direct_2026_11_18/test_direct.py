"""Numerical checks for direct-joint kinematics and local contact exemptions."""
import copy
import json
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
from direct import collision, fk, load_design, rot90, segment_distance


class DirectJointTests(unittest.TestCase):
    def test_segment_distance_crossing_and_random_reference(self):
        m,_=load_design('index','index')
        rng=np.random.default_rng(61);rows=rng.normal(size=(500,4,2))*20
        actual=segment_distance(*[rows[:,i,:] for i in range(4)])
        expected=[m._segment_distance(*[tuple(v) for v in row]) for row in rows]
        np.testing.assert_allclose(actual,expected,atol=1e-9)
        np.testing.assert_allclose(segment_distance(np.array([[0.,0.],[0.,0.]]),np.array([[1.,0.],[1.,0.]]),np.array([[2.,0.],[.5,-1.]]),np.array([[3.,0.],[.5,1.]])),[1,0])

    def test_h_matches_old_r4_over_entire_assembly_motion(self):
        base=Path(__file__).resolve().parent
        for finger in ['index','middle','ring','little']:
            converted=base/(finger+'_converted')/'result.json'
            if not converted.exists():converted=base/'direct_conversion'/finger/'result.json'
            candidate=json.loads(converted.read_text())
            source=Path(candidate['source'])
            if not source.exists():source=base/'source_candidates'/finger/'result.json'
            original=json.loads(source.read_text());_,p=load_design(finger,finger)
            q=np.linspace(original['q_deg'][0],original['q_deg'][-1],1201)
            old,_=fk(original['lengths_mm'],q,p.ad_tilt_deg)
            direction=old['h']-old['g'];direction/=np.linalg.norm(direction,axis=-1,keepdims=True)
            expected=old['h']+original['output_side']*original['lengths_mm']['L_tip_rod']*rot90(direction)
            new,_=fk(candidate['lengths_mm'],q,p.ad_tilt_deg,candidate['output_branch'])
            np.testing.assert_allclose(new['h'],expected,atol=1e-8)
            self.assertNotIn('L_tip_rod',candidate['lengths_mm'])

    def test_attachment_exemption_only_removes_local_corner(self):
        _,p=load_design('index','index');nominal=copy.deepcopy(p.nominal_design)
        nominal['dimensions']=[row for row in nominal['dimensions'] if row['body']=='distal_body']
        nominal['bodies']=[row for row in nominal['bodies'] if row['id']=='distal_body']
        p=replace(p,nominal_design=nominal)
        nodes={'f':np.array([[-10.,10.]]),'g':np.array([[10.,10.]]),'h':np.array([[0.,0.]])}
        # Four identical capsules make the plane y=0 the dorsal surface.
        hands=np.tile(np.array([[[[-30.,-5.],[30.,-5.],[5.,5.]]]]),(1,4,1,1))
        self.assertLess(float(collision(p,nodes,hands,attachment_mm=0)[0]),-1.9)
        self.assertGreater(float(collision(p,nodes,hands,attachment_mm=4)[0]),.8)
        # A remote piece intersects the hand; moving the contact exemption
        # must not hide this collision.
        nodes['f']=np.array([[-15.,-3.]])
        self.assertLess(float(collision(p,nodes,hands,attachment_mm=4)[0]),0)


if __name__=='__main__':unittest.main()
