# Repository maintenance rules

- Treat `FINGER_DIMENSIONS.txt` as the human-readable index of the 2–5 finger root coordinates and anatomical phalanx lengths.
- Whenever a change modifies any index, middle, ring, or little finger root X/Y coordinate or proximal/middle/distal phalanx length, update `FINGER_DIMENSIONS.txt` in the same change.
- Keep unknown assembly-derived Y coordinates marked as pending; do not infer them as zero.
- Do not confuse anatomical phalanx lengths with exoskeleton linkage lengths such as AB, BC, CD, or GH.
