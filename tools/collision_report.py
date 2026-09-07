"""Name the mechanism part, hand part and pose behind a collision failure.

    python3 tools/collision_report.py --design short_ad --finger index

`evaluate()` reports only the worst signed clearance, which tells you a design
collides but not where. This walks the same primitive pairs the optimiser uses and
keeps the labels, so a -12.000 mm result can be traced to, say, the central body
passing through the proximal phalanx rather than a link grazing the palm.
"""
from __future__ import annotations

import argparse
import math

import numpy as np

from _common import initial_vector, load_design


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", default="short_ad")
    parser.add_argument("--finger", default="index")
    parser.add_argument("--top", type=int, default=15, help="worst N pairs to list")
    parser.add_argument("--vector", help="optional .npy design vector to inspect instead")
    args = parser.parse_args()

    module, problem = load_design(args.design, args.finger)
    values = np.load(args.vector) if args.vector else initial_vector(problem)
    lengths = module._candidate_lengths(problem, values)
    objective = problem.objectives[0]
    tilt = problem.ad_tilt_deg

    config = problem.component_config["hand_mechanism_non_collision"]
    mount_exclusion = float(config["dorsal_mount_exclusion_radius_mm"])
    link_radius = float(config["mechanism_link_radius_mm"])
    rod_radius = float(config["output_rod_radius_mm"])
    r4_exclusion = float(config["r4_contact_exclusion_radius_mm"])
    axis_samples = int(config["link_axis_samples"])

    evaluation = module.evaluate(problem, values)
    schedule = evaluation.hand_progress_schedules[objective.id]
    crank = evaluation.input_schedules_deg[objective.id]
    if not schedule:
        raise SystemExit("this design has no motion solution; run design_probe.py first")

    poses = module._mechanism_workspace_poses(lengths, np.asarray(crank), tilt)
    records = []
    for progress, (q_deg, positions) in zip(schedule, poses):
        capsules = module._hand_capsules(problem, objective, float(progress))

        for seg_id, start, end in module._mechanism_segments(
                problem, positions, mount_exclusion):
            for hand_id, hs, he, hr in capsules:
                clearance = module._segment_distance(start, end, hs, he) - link_radius - hr
                records.append((clearance, q_deg, progress, seg_id, hand_id))

        for body_id, polygon in module._mechanism_polygons(
                problem, positions, mount_exclusion):
            for hand_id, hs, he, hr in capsules:
                centre = module._segment_polygon_signed_distance(hs, he, polygon, axis_samples)
                records.append((centre - link_radius - hr, q_deg, progress,
                                f"body:{body_id}", hand_id))

        contact = module._hand_distal_contact_progress(problem, objective, float(progress))
        trimmed = module._trim_segment_endpoint(positions["h"], contact, r4_exclusion, False)
        if trimmed is not None:
            for hand_id, hs, he, hr in capsules:
                clearance = module._segment_distance(*trimmed, hs, he) - rod_radius - hr
                records.append((clearance, q_deg, progress, "output_rod", hand_id))

    records.sort(key=lambda r: r[0])
    print(f"design {args.design}  finger {args.finger}  "
          f"worst clearance {records[0][0]:+.4f} mm\n")
    print(f"{'clearance_mm':>13} {'q_deg':>8} {'curl_s':>8}  "
          f"{'mechanism part':<22} {'hand part'}")
    for clearance, q_deg, progress, part, hand_part in records[:args.top]:
        print(f"{clearance:13.4f} {q_deg:8.2f} {progress:8.3f}  "
              f"{part:<22} {hand_part}")

    worst = records[0]
    print(f"\nA clearance of exactly -(link radius + hand-part radius) means the hand "
          f"segment passes clean through a rigid body, not that it grazes it.")
    print(f"Link radius {link_radius} mm; hand radii come from the phalanx widths.")
    print(f"Worst pair: {worst[3]} vs {worst[4]} at q={worst[1]:.2f} deg.")


if __name__ == "__main__":
    main()
