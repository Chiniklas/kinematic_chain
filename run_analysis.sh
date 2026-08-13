#!/usr/bin/env bash
set -euo pipefail

# Analyze one or more self-contained mechanism.yaml design documents. Unless an
# explicit destination is supplied for a single design, each design writes beside
# itself into an artifacts/ directory.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="${script_dir}"
default_mechanism="${project_dir}/designs/mechanism_2/nominal/mechanism.yaml"

usage() {
  printf 'Usage: %s [--finger NAME] [--output-dir DIR] [mechanism.yaml|directory ...]\n' "$0"
  printf '%s\n' 'Analyze one YAML, several YAMLs, or every mechanism.yaml below a directory.'
  printf '%s\n' 'Default output: <directory-containing-mechanism.yaml>/artifacts/'
  printf '%s\n' '--output-dir is supported only when exactly one mechanism.yaml is selected.'
}

finger=""
output_dir=""
inputs=()
while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --finger)
      if (($# < 2)); then
        printf '%s\n' '--finger requires index, middle, ring, or little' >&2
        exit 2
      fi
      finger="$2"
      shift 2
      ;;
    --finger=*)
      finger="${1#--finger=}"
      shift
      ;;
    --output-dir)
      if (($# < 2)); then
        printf '%s\n' '--output-dir requires a path' >&2
        exit 2
      fi
      output_dir="$2"
      shift 2
      ;;
    --output-dir=*)
      output_dir="${1#--output-dir=}"
      shift
      ;;
    --*)
      printf 'Unknown analysis option: %s\n' "$1" >&2
      exit 2
      ;;
    *)
      inputs+=("$1")
      shift
      ;;
  esac
done

if [[ -n "${finger}" ]]; then
  case "${finger}" in
    index|middle|ring|little) ;;
    *) printf 'Invalid --finger value: %s\n' "${finger}" >&2; exit 2 ;;
  esac
fi

if ((${#inputs[@]} == 0)); then
  inputs=("${default_mechanism}")
fi

mechanisms=()
declare -A seen_mechanisms=()
for input in "${inputs[@]}"; do
  if [[ -f "${input}" ]]; then
    candidates=("${input}")
  elif [[ -d "${input}" ]]; then
    mapfile -d '' candidates < <(find "${input}" -type f -name mechanism.yaml -print0 | sort -z)
    if ((${#candidates[@]} == 0)); then
      printf 'No mechanism.yaml found below %s\n' "${input}" >&2
      exit 2
    fi
  else
    printf 'Analysis input does not exist: %s\n' "${input}" >&2
    exit 2
  fi
  for candidate in "${candidates[@]}"; do
    candidate="$(cd -- "$(dirname -- "${candidate}")" && pwd)/$(basename -- "${candidate}")"
    if [[ -z "${seen_mechanisms[${candidate}]+present}" ]]; then
      mechanisms+=("${candidate}")
      seen_mechanisms["${candidate}"]=1
    fi
  done
done

if [[ -n "${output_dir}" && ${#mechanisms[@]} -ne 1 ]]; then
  printf '%s\n' '--output-dir requires exactly one selected mechanism.yaml' >&2
  exit 2
fi

if ! python3 -c 'import yaml, matplotlib, numpy' >/dev/null 2>&1; then
  printf '%s\n' 'Missing Python dependencies in the active environment.' >&2
  printf '%s\n' \
    "Run: conda env update --name kinematic-chain --file ${project_dir}/environment.yml" >&2
  exit 3
fi

analyze_design() {
  local abstraction="$1"
  local artifacts="$2"
  local mechanism_dir="${artifacts}/mechanism"
  local mechanism_workspace_dir="${mechanism_dir}/workspace"
  local mechanism_torque_dir="${mechanism_dir}/torque"
  local combined_dir="${artifacts}/combined"
  local finger_dir="${combined_dir}/fingers"
  mkdir -p \
    "${mechanism_workspace_dir}" \
    "${mechanism_torque_dir}" \
    "${finger_dir}"

  PYTHONPATH="${project_dir}/src/analysis" python3 \
    "${project_dir}/src/analysis/mechanism_table.py" \
    "${abstraction}" \
    --markdown "${mechanism_dir}/mechanism_tables.md" \
    --csv "${mechanism_dir}/link_lengths.csv"

  PYTHONPATH="${project_dir}/src/analysis" \
    MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
    python3 "${project_dir}/src/analysis/plot_linkage.py" \
    "${abstraction}" \
    --output "${mechanism_dir}/abstraction.png"

  local combined_suite_args=(
    "${abstraction}"
    --output-dir "${finger_dir}"
    --overview "${combined_dir}/combined_abstraction.png"
    --group-by-finger
  )
  local combined_analysis_args=(
    "${abstraction}"
    --output "${combined_dir}/combined_workspace_report.png"
    --csv "${combined_dir}/combined_workspace_samples.csv"
    --summary "${combined_dir}/combined_workspace_summary.yaml"
    --per-finger-dir "${finger_dir}"
  )
  if [[ -n "${finger}" ]]; then
    combined_suite_args+=(--finger "${finger}")
    combined_analysis_args+=(--finger "${finger}")
  fi

  PYTHONPATH="${project_dir}/src/analysis" \
    MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
    python3 "${project_dir}/src/analysis/plot_combined_suite.py" \
    "${combined_suite_args[@]}"

  PYTHONPATH="${project_dir}/src/analysis" \
    MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
    python3 "${project_dir}/src/analysis/workspace_sweep.py" \
    "${abstraction}" \
    --output "${mechanism_workspace_dir}/workspace_report.png" \
    --csv "${mechanism_workspace_dir}/workspace_samples.csv"

  PYTHONPATH="${project_dir}/src/analysis" \
    MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
    python3 "${project_dir}/src/analysis/combined_analysis.py" \
    "${combined_analysis_args[@]}"

  PYTHONPATH="${project_dir}/src/analysis" \
    MPLCONFIGDIR="${TMPDIR:-/tmp}/kinematic-chain-matplotlib" \
    python3 "${project_dir}/src/analysis/torque_analysis.py" \
    "${abstraction}" \
    --output "${mechanism_torque_dir}/torque_report.png" \
    --csv "${mechanism_torque_dir}/torque_samples.csv"

  printf 'Analysis artifacts for %s written to %s\n' "${abstraction}" "${artifacts}"
}

for mechanism in "${mechanisms[@]}"; do
  if [[ -n "${output_dir}" ]]; then
    artifacts="${output_dir}"
  else
    artifacts="$(dirname -- "${mechanism}")/artifacts"
  fi
  analyze_design "${mechanism}" "${artifacts}"
done
