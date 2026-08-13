# Mechanism 2 nominal design

`mechanism.yaml` is the only editable source of truth for the nominal mechanism,
self-contained four-finger analysis targets, human-hand context, attachment
interfaces, analysis settings, and nominal mass model. Refresh every generated file
below with `./run_analysis.sh` from the repository root.

```text
nominal/
├── README.md
├── mechanism.yaml
├── sources/
│   ├── abstraction.jpg
│   └── real_thing.jpg
└── artifacts/
    ├── mechanism/
    │   ├── abstraction.png
    │   ├── link_lengths.csv
    │   ├── mechanism_tables.md
    │   ├── workspace/
    │   └── torque/
    └── combined/
        ├── combined_abstraction.png
        ├── combined_workspace_report.png
        ├── combined_workspace_samples.csv
        ├── combined_workspace_summary.yaml
        └── fingers/
            └── {index,middle,ring,little}/
                ├── combined_abstraction.png
                ├── combined_workspace_report.png
                ├── combined_workspace_samples.csv
                └── combined_workspace_summary.yaml
```

Files under `sources/` are measurement evidence. Files under `artifacts/` are generated
artifacts and should not be edited independently. Optimization runs remain under
`runs/`; every materialized candidate follows the same `mechanism.yaml` + `artifacts/`
design contract.
