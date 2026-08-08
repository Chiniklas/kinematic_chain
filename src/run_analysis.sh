#!/usr/bin/env bash
set -euo pipefail

# Read-only entry point for analysing the current abstraction. It may write
# reports under output_dir, but it never modifies the abstraction YAML.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  printf 'Usage: %s [abstraction.yaml] [output-dir]\n' "$0"
  printf 'Validate and render the current mechanism without modifying it.\n'
  exit 0
fi

abstraction="${1:-${project_dir}/sources/mechanism_2/mechanism.yaml}"
output_dir="${2:-${project_dir}/runs/mechanism_2}"

if ! python3 -c 'import yaml, matplotlib' >/dev/null 2>&1; then
  printf '%s\n' 'Missing Python dependencies in the active environment.' >&2
  printf '%s\n' \
    "Run: conda env update --name kinematic-chain --file ${project_dir}/environment.yml" >&2
  exit 3
fi

mkdir -p "${output_dir}"

PYTHONPATH="${project_dir}/src" python3 "${project_dir}/src/mechanism_table.py" \
  "${abstraction}" \
  --markdown "${output_dir}/mechanism_tables.md" \
  --csv "${output_dir}/link_lengths.csv"

PYTHONPATH="${project_dir}/src" MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
  python3 "${project_dir}/src/plot_linkage.py" \
  "${abstraction}" \
  --output "${output_dir}/abstraction.png"

printf 'Analysis outputs written to %s\n' "${output_dir}"
