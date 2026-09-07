"""Report a design's geometry and say why it fails, without running an optimisation.

    python3 tools/design_probe.py --design short_ad --finger index

Prints link lengths, rigid-body interior angles, node coordinates, how much of the
crank sweep assembles (and which loop breaks first if it does not), and the current
constraint status.
"""
from __future__ import annotations

import argparse
import math

import numpy as np

from _common import (bounds, fgh_angle, initial_vector, load_design, mark, passes,
                     triangle_angle, variable_ids)

ORDER = ["L_ab", "L_bc", "L_cd", "L_ad", "L_ae", "L_de",
         "L_cg", "L_dg", "L_ef", "L_fg", "L_gh", "L_fh", "L_tip_rod"]


def first_break(module, lengths, tilt, samples=91):
    """Walk the sweep and name the first loop that cannot close."""
    ci, nearest = module._circle_intersections, module._nearest
    a, d = (-lengths["L_ad"], 0.0), (0.0, 0.0)
    e_candidates = ci(a, lengths["L_ae"], d, lengths["L_de"])
    if not e_candidates:
        return 0.0, ("ground triangle A-D-E",
                     f"|AE-DE|={abs(lengths['L_ae']-lengths['L_de']):.2f} "
                     f"<= AD={lengths['L_ad']:.2f} <= {lengths['L_ae']+lengths['L_de']:.2f}")
    e = max(e_candidates, key=lambda p: p[1])
    previous = None
    for q_deg in np.linspace(0, 90, samples):
        angle = math.radians(90.0 - float(q_deg))
        b = (a[0] + lengths["L_ab"] * math.cos(angle),
             a[1] + lengths["L_ab"] * math.sin(angle))
        c_candidates = ci(b, lengths["L_bc"], d, lengths["L_cd"])
        if not c_candidates:
            return q_deg, ("loop 1 A-B-C-D (point C)",
                           f"|BD|={math.dist(b, d):.2f} but BC+CD="
                           f"{lengths['L_bc']+lengths['L_cd']:.2f}")
        c = nearest(c_candidates, previous["c"]) if previous else max(c_candidates, key=lambda p: p[1])
        g_candidates = ci(c, lengths["L_cg"], d, lengths["L_dg"])
        if not g_candidates:
            return q_deg, ("rigid body C-D-G (point G)", "")
        g = nearest(g_candidates, previous["g"]) if previous else max(g_candidates, key=lambda p: p[0])
        f_candidates = ci(e, lengths["L_ef"], g, lengths["L_fg"])
        if not f_candidates:
            return q_deg, ("loop 2 D-G-F-E (point F)",
                           f"|EG|={math.dist(e, g):.2f} but EF+FG="
                           f"{lengths['L_ef']+lengths['L_fg']:.2f}")
        f = nearest(f_candidates, previous["f"]) if previous else min(f_candidates, key=lambda p: p[1])
        h_candidates = ci(g, lengths["L_gh"], f, lengths["L_fh"])
        if not h_candidates:
            return q_deg, ("rigid body F-G-H (point H)", "")
        h = nearest(h_candidates, previous["h"]) if previous else max(h_candidates, key=lambda p: p[0])
        previous = {"c": c, "g": g, "f": f, "h": h}
    return None, None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", default="short_ad")
    parser.add_argument("--finger", default="index")
    args = parser.parse_args()

    module, problem = load_design(args.design, args.finger)
    ids = variable_ids(problem)
    values = initial_vector(problem)
    lengths = module._candidate_lengths(problem, values)
    tilt = problem.ad_tilt_deg

    print(f"design {args.design}  finger {args.finger}")
    print(f"  L_ad={lengths['L_ad']:.3f} mm  tilt={tilt:.3f} deg  "
          f"dorsal clearance={problem.dorsal_clearance_mm:.1f} mm")

    print("\nlink lengths (mm), with bounds")
    low, high = bounds(problem)
    limits = dict(zip(ids, zip(low, high)))
    for name in ORDER:
        if name == "L_ad":
            print(f"  {name:<10}{lengths[name]:9.3f}   fixed")
        else:
            lo, hi = limits[name]
            at = " <- at bound" if min(abs(lengths[name]-lo), abs(lengths[name]-hi)) < 1e-6 else ""
            print(f"  {name:<10}{lengths[name]:9.3f}   [{lo:.1f}, {hi:.1f}]{at}")

    print("\nrigid-body interior angles (deg)")
    for label, (x, y, z) in {
        "ground  A-D-E": ("L_ad", "L_ae", "L_de"),
        "central C-D-G": ("L_cd", "L_dg", "L_cg"),
        "distal  F-G-H": ("L_fg", "L_gh", "L_fh"),
    }.items():
        print(f"  {label}: at first vertex {triangle_angle(lengths[x], lengths[y], lengths[z]):6.2f}")
    print(f"  angle FGH (at G, the one that swings H toward the finger): {fgh_angle(lengths):.2f}")

    poses = module._mechanism_workspace_poses(lengths, np.linspace(0, 90, 91), tilt)
    print(f"\nassembly across a 0-90 deg sweep: {len(poses)}/91 poses")
    if len(poses) < 91:
        q_deg, (where, detail) = first_break(module, lengths, tilt)
        print(f"  first failure at q={q_deg:.1f} deg in {where}")
        if detail:
            print(f"    {detail}")
    if poses:
        print("\nnode coordinates at q=0 (D at origin, tilt applied)")
        for node in "abcdefgh":
            x, y = poses[0][1][node]
            print(f"  {node.upper()}: ({x:8.3f}, {y:8.3f})")

    evaluation = module.evaluate(problem, values)
    print(f"\nconstraints: {passes(evaluation)}/3")
    print(f"  collision   {mark(evaluation.collision_free):>4}  "
          f"min clearance {evaluation.minimum_clearance_mm:+.3f} mm (need >= 0)")
    print(f"  rod closure {mark(evaluation.rod_closure_feasible):>4}  "
          f"max error {evaluation.maximum_rod_closure_error_mm:.4f} mm (need <= 0.1)")
    print(f"  curl        {mark(evaluation.hand_motion_feasible):>4}")
    schedule = evaluation.hand_progress_schedules[problem.objectives[0].id]
    if schedule:
        print(f"    curl progress 0 -> {schedule[-1]:.4f} (need >= 0.99)")


if __name__ == "__main__":
    main()
