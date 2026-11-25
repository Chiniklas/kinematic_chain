# Ring and little finger length/starting-angle candidates

Results from 2026-11-13, based on upstream commit
`879f499cdf6fa16e822807b4534cc1cc403f4ab5`, archived on 2026-11-16.
The ring finger uses the short mount; the little finger uses the extended mount.
Grounded A/D/E geometry stays fixed while moving-link lengths, output-rod length
and starting angle are jointly optimized.

| Candidate | AD (mm) | DG (mm) | GH (mm) | Model starting offset (deg) | Minimum sampled clearance (mm) |
|---|---:|---:|---:|---:|---:|
| Ring / finger 4 | 21.112 | 54.808599 | 52.438513 | +0.732924 | 0.397909 |
| Little / finger 5 | 68.049 | 64.259272 | 58.036314 | -1.078496 | 1.987311 |

Both candidates passed 1,201 planar poses with continuous passive closure tracking.
These are sampled 2D candidates, not a manufacturing or full 3D collision release.
The ring candidate has only about 0.40 mm model clearance. Starting offsets follow
the convention in each `parameters.json`; they are not calibrated servo commands.
Output-rod perpendicularity was not an objective in this local search.

- [Ring parameters](ring/parameters.json) and [little parameters](little/parameters.json)
  include all dimensions, starting angles, motion ranges and validation metrics.
- [Full comparison](all_length_comparison.csv) includes previous glove geometry,
  the earlier optimization baseline and the retained candidates.
- [HTML report](report.html) includes motion and size comparisons (download to view).
- Each finger directory includes selected results, trials and the 1,201-pose sweep.
- `input_snapshot`, `source_snapshot` and `provenance.json` preserve inputs,
  the evaluator and original script hashes. Original optimization/report scripts
  are archived unchanged and retain the historical `work/` and `outputs/` layout;
  their original commands are recorded in `provenance.json`.

To independently recheck the saved candidates from a fresh repository checkout,
use the project Python environment with SciPy installed:

```bash
python docs/candidates/ring_little_reoptimization/verify_saved_candidates.py
```

The verifier checks the archived script hashes and input correspondence, then
recomputes the 1,201-pose checks without changing the saved results. Library versions
used originally are recorded in `provenance.json`.

The archived `cad_or_hardware_modified: false` describes the numerical run at the
time it was made. Subsequent Onshape length previews are separate; this commit
does not claim that all optimized angles, base geometry and output rods have been
transferred to CAD or hardware.
