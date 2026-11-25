"""Independent measured mounts and signed installed phases, without PyTorch."""
from pathlib import Path
import copy
import json
import math
import sys
import unittest
from dataclasses import replace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from _common import load_design, initial_vector

class FingerMountTests(unittest.TestCase):
    def test_mounts_retain_motor_holes_and_individual_root_offsets(self):
        expected={'index':(0,0),'middle':(50,6),'ring':(0,2),'little':(50,-3)}
        motor_world={}
        for finger,(rear,forward) in expected.items():
            m,p=load_design(finger,finger)
            self.assertEqual([o.finger for o in p.objectives],[finger])
            mount=p.nominal_design['exoskeleton_attachments'][0]
            self.assertEqual(mount['placement_reference']['finger_root_forward_offset_mm'],forward)
            self.assertEqual(mount['placement_reference']['motor_mount_rearward_offset_mm'],rear)
            length=p.fixed_mechanism_dimensions['L_ad'];tilt=math.radians(p.ad_tilt_deg)
            dx=length*math.cos(tilt);dy=length*math.sin(tilt)
            self.assertAlmostEqual(dx,16.84881073214355+rear+forward,places=8)
            self.assertAlmostEqual(dy,12.721404046425228,places=8)
            self.assertEqual(p.dorsal_clearance_mm,7)
            motor_world[finger]=forward-dx
        self.assertAlmostEqual(motor_world['index'],motor_world['ring'])
        self.assertAlmostEqual(motor_world['middle'],motor_world['little'])
        self.assertAlmostEqual(motor_world['index']-motor_world['middle'],50)

    def test_signed_phases_survive_materialization_and_preserve_relative_input(self):
        phases=[]
        for finger in ('index','middle','ring','little'):
            m,p=load_design(finger,finger);phases.append(p.input_phase_deg)
            lengths=m._candidate_lengths(p,initial_vector(p))
            q=np.linspace(0,5,11)
            poses=m._phase_workspace_poses(lengths,q,p.ad_tilt_deg,p.input_phase_deg)
            np.testing.assert_allclose([value for value,_ in poses],q)
            first=poses[0][1];angle=-math.radians(p.ad_tilt_deg)
            for node in p.nominal_design['nodes']:
                x,y=node['initial_position_mm']
                expected=(x*math.cos(angle)-y*math.sin(angle),x*math.sin(angle)+y*math.cos(angle))
                np.testing.assert_allclose(first[node['id']],expected,atol=1e-8)
        self.assertTrue(any(p<0 for p in phases))
        self.assertTrue(any(p>0 for p in phases))

    def test_saved_candidates_pass_dense_checks_after_loading_new_profiles(self):
        summary=json.loads((ROOT/'docs/candidates/_archive/four_finger_offsets_2026_11_17/summary.json').read_text())
        for finger in ('index','middle','ring','little'):
            with self.subTest(finger=finger):
                m,p=load_design(finger,finger)
                components=copy.deepcopy(p.component_config);constraints=copy.deepcopy(p.constraint_config)
                components['task_space_reachability']['curled_input_samples']=1801
                for name in ('hand_mechanism_non_collision','output_link_perpendicularity'):
                    components[name]['evaluation_poses']=1201
                for name in ('closure_error','hand_motion_admissibility'):
                    constraints[name]['evaluation_poses']=1201
                p=replace(p,component_config=components,constraint_config=constraints)
                ev=m.evaluate(p,initial_vector(p));reference=summary['results'][finger]
                self.assertTrue(ev.collision_free and ev.rod_closure_feasible and ev.hand_motion_feasible)
                self.assertAlmostEqual(ev.minimum_clearance_mm,reference['minimum_clearance_mm'],places=6)
                self.assertAlmostEqual(ev.maximum_rod_closure_error_mm,reference['maximum_rod_closure_error_mm'],places=6)

    def test_legacy_phase_zero_is_unchanged(self):
        for design in ('long_ad','short_ad'):
            m,p=load_design(design,'index');self.assertEqual(p.input_phase_deg,0)
            lengths=m._candidate_lengths(p,initial_vector(p));q=np.linspace(0,5,11)
            self.assertEqual(m._phase_workspace_poses(lengths,q,p.ad_tilt_deg,0),m._mechanism_workspace_poses(lengths,q,p.ad_tilt_deg))

if __name__=='__main__':unittest.main()
