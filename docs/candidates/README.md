# Candidate versions

The supported human-readable candidate layout is version based:

```text
Vn/
  manifest.yaml
  <finger>.yaml
  preview.png
  motion.gif
```

Each `<finger>.yaml` uses the same wrapper fields: `schema_version`, `version`,
`finger`, `source`, and `candidate`. The source payload is retained under `candidate`
so the historical result is not silently altered during format conversion.

| Version | Date | Optimized fingers | Description | Status |
|---|---|---|---|---|
| V1 | 2026-08-25 | index | Initial index-finger candidate | historical |
| V2 | 2026-09-06 | index | Tilted-mount index-finger candidate | historical |
| V3 | 2026-09-08 | middle | Extended-mount middle-finger candidate | historical |
| V4 | 2026-11-09 | thumb | Direct-drive thumb candidate | historical |
| V5 | 2026-11-16 | ring, little | Ring/little reoptimization | historical |
| V6 | 2026-11-16 | thumb | Non-returning straight-output thumb | historical |
| V7 | 2026-11-17 | index, middle, ring, little | Independent X-offset optimization | historical |
| V8 | 2026-11-18 | index, middle, ring, little | Direct H-to-R4 feasibility study | historical study |
| V9 | 2026-11-18 | index, middle, ring, little | GH-clearance contour candidate | **current** |

`_archive/` contains the original reports, JSON files, logs, CAD assets, and
reproduction scripts. It is intentionally not part of the supported presentation
format, but remains available for validation and provenance.

Rebuild the standardized folders after changing an archived source:

```bash
python3 tools/normalize_candidate_versions.py
```
