#!/usr/bin/env bash
set -euo pipefail

# Read-only entry point for analysing the current abstraction. It may write
# reports under output_dir, but it never modifies the abstraction YAML.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="${script_dir}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  printf 'Usage: %s [abstraction.yaml] [output-dir]\n' "$0"
  printf 'Validate and render the current mechanism without modifying it.\n'
  exit 0
fi

abstraction="${1:-${project_dir}/designs/mechanism_2/nominal/mechanism.yaml}"
analysis_timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
output_dir="${2:-${project_dir}/runs/analysis_${analysis_timestamp}}"

if ! python3 -c 'import yaml, matplotlib, numpy' >/dev/null 2>&1; then
  printf '%s\n' 'Missing Python dependencies in the active environment.' >&2
  printf '%s\n' \
    "Run: conda env update --name kinematic-chain --file ${project_dir}/environment.yml" >&2
  exit 3
fi

mkdir -p "${output_dir}"
mechanism_dir="${output_dir}/mechanism"
mechanism_workspace_dir="${mechanism_dir}/workspace"
mechanism_torque_dir="${mechanism_dir}/torque"
combined_dir="${output_dir}/combined"
finger_dir="${combined_dir}/fingers"
mkdir -p \
  "${mechanism_workspace_dir}" \
  "${mechanism_torque_dir}" \
  "${finger_dir}"

PYTHONPATH="${project_dir}/src/analysis" python3 "${project_dir}/src/analysis/mechanism_table.py" \
  "${abstraction}" \
  --markdown "${mechanism_dir}/mechanism_tables.md" \
  --csv "${mechanism_dir}/link_lengths.csv"

PYTHONPATH="${project_dir}/src/analysis" MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
  python3 "${project_dir}/src/analysis/plot_linkage.py" \
  "${abstraction}" \
  --output "${mechanism_dir}/abstraction.png"

PYTHONPATH="${project_dir}/src/analysis" MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
  python3 "${project_dir}/src/analysis/plot_combined_suite.py" \
  "${abstraction}" \
  --objectives-dir "${project_dir}/src/co-optimization/config/objectives" \
  --output-dir "${finger_dir}" \
  --overview "${combined_dir}/combined_abstraction.png" \
  --group-by-finger

PYTHONPATH="${project_dir}/src/analysis" MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
  python3 "${project_dir}/src/analysis/workspace_sweep.py" \
  "${abstraction}" \
  --output "${mechanism_workspace_dir}/workspace_report.png" \
  --csv "${mechanism_workspace_dir}/workspace_samples.csv"

PYTHONPATH="${project_dir}/src/analysis" MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
  python3 "${project_dir}/src/analysis/combined_analysis.py" \
  "${abstraction}" \
  --objectives-dir "${project_dir}/src/co-optimization/config/objectives" \
  --output "${combined_dir}/combined_workspace_report.png" \
  --csv "${combined_dir}/combined_workspace_samples.csv" \
  --summary "${combined_dir}/combined_workspace_summary.yaml" \
  --per-finger-dir "${finger_dir}"

PYTHONPATH="${project_dir}/src/analysis" MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
  python3 "${project_dir}/src/analysis/torque_analysis.py" \
  "${abstraction}" \
  --output "${mechanism_torque_dir}/torque_report.png" \
  --csv "${mechanism_torque_dir}/torque_samples.csv"

printf 'Analysis outputs written to %s\n' "${output_dir}"
