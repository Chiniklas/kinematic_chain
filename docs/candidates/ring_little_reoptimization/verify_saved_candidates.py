"""Recheck the archived candidates without rerunning optimization or writing results."""
from dataclasses import replace
from pathlib import Path
import hashlib
import json
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
# Use the archived evaluator, preserving the model used for these results.
sys.path.insert(0, str(HERE / 'source_snapshot'))
sys.path.insert(0, str(HERE))
from validate_reoptimization import exact_check
from reoptimize_fingers import m


def main():
    provenance = json.loads((HERE / 'provenance.json').read_text(encoding='utf-8'))
    for name, expected in provenance['sha256'].items():
        assert hashlib.sha256((HERE / name).read_bytes()).hexdigest() == expected, name
    for finger, profile, design in [('ring', 'short_ad', 'short_ad'),
                                    ('little', 'long_ad', 'nominal')]:
        snapshot = HERE / 'input_snapshot' / finger
        cfg = REPO / 'src/co-optimization/config' / profile
        pairs = [(snapshot / name, cfg / name) for name in
                 ['objectives.yaml', 'optimizable_variables.yaml']]
        pairs += [(snapshot / (finger + '.yaml'), cfg.parent / 'objectives' / (finger + '.yaml')),
                  (snapshot / 'mechanism.yaml', REPO / 'designs/mechanism_2' / design / 'mechanism.yaml')]
        for saved, current in pairs:
            assert saved.read_text(encoding='utf-8') == current.read_text(encoding='utf-8'), str(current)
        problem = m.load_problem(cfg / 'objectives.yaml', cfg / 'optimizable_variables.yaml')
        problem = replace(problem, objectives=tuple(o for o in problem.objectives if o.finger == finger))
        result = json.loads((HERE / finger / 'result.json').read_text(encoding='utf-8'))
        params = json.loads((HERE / finger / 'parameters.json').read_text(encoding='utf-8'))
        selected = result['selected']
        assert params['lengths_mm'] == selected['lengths_mm']
        assert params['phase_deg'] == selected['phase_deg']
        check = exact_check(problem, np.array(selected['values']), selected['phase_deg'], selected['evaluation'])
        assert check['passed'], finger
        for key in ['samples', 'assembly_samples', 'minimum_clearance_mm', 'maximum_closure_mm', 'stroke_deg']:
            assert abs(check[key] - params['validation'][key]) < 1e-8, (finger, key)
        print(f"{finger}: PASS, {check['samples']} poses, minimum clearance {check['minimum_clearance_mm']:.6f} mm", flush=True)


if __name__ == '__main__':
    main()
