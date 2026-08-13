#!/usr/bin/env bash
set -euo pipefail

# Canonical top-level entry point for co-optimization and candidate analysis.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="${script_dir}"
entry_point="${project_dir}/src/co-optimization/run_adam.py"

if ! python3 -c 'import matplotlib, numpy, torch, yaml' >/dev/null 2>&1; then
  printf '%s\n' 'Missing Python optimization dependencies.' >&2
  printf '%s\n' \
    "Run: conda env update --name kinematic-chain --file ${project_dir}/environment.yml" >&2
  exit 3
fi

for argument in "$@"; do
  if [[ "${argument}" == "-h" || "${argument}" == "--help" ]]; then
    exec python3 "${entry_point}" "$@"
  fi
done

optimization_timestamp="$(date -u +%Y%m%dT%H%M%SZ)"
output_dir=""
arguments=("$@")
for ((index = 0; index < ${#arguments[@]}; index++)); do
  case "${arguments[index]}" in
    --output-dir)
      if ((index + 1 >= ${#arguments[@]})); then
        printf '%s\n' '--output-dir requires a path' >&2
        exit 2
      fi
      output_dir="${arguments[index + 1]}"
      ;;
    --output-dir=*)
      output_dir="${arguments[index]#--output-dir=}"
      ;;
  esac
done

if [[ -z "${output_dir}" ]]; then
  output_dir="${project_dir}/runs/co-optimization_${optimization_timestamp}"
  arguments+=(--output-dir "${output_dir}")
fi

python3 "${entry_point}" "${arguments[@]}"

if [[ "${output_dir}" != /* ]]; then
  output_dir="$(pwd)/${output_dir}"
fi
candidate_dirs=("${output_dir}"/fingers/*/candidate_*)
if [[ ! -d "${candidate_dirs[0]}" ]]; then
  printf 'No per-finger retained candidate directories found under %s\n' "${output_dir}" >&2
  exit 4
fi
for candidate_dir in "${candidate_dirs[@]}"; do
  if [[ ! -f "${candidate_dir}/mechanism.yaml" ]]; then
    printf 'Candidate is missing materialized mechanism: %s\n' "${candidate_dir}" >&2
    exit 4
  fi
done

analysis_failures=0
for candidate_dir in "${candidate_dirs[@]}"; do
  finger="$(basename -- "$(dirname -- "${candidate_dir}")")"
  analysis_dir="${candidate_dir}/artifacts"
  printf 'Analyzing materialized %s design: %s\n' \
    "${finger}" "${candidate_dir}/mechanism.yaml"
  if ! "${project_dir}/run_analysis.sh" \
      --output-dir "${analysis_dir}" --finger "${finger}" \
      "${candidate_dir}/mechanism.yaml"; then
    printf 'Analysis failed for %s: %s\n' "${finger}" "${candidate_dir}" >&2
    analysis_failures=$((analysis_failures + 1))
  fi
done

if ((analysis_failures > 0)); then
  printf '%d candidate analysis job(s) failed under %s\n' \
    "${analysis_failures}" "${output_dir}" >&2
  exit 5
fi

printf 'Co-optimization outputs written to %s\n' "${output_dir}"
