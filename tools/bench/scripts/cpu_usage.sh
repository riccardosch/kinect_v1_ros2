#!/usr/bin/env bash
# Logs CPU usage of a process once per second with pidstat (package sysstat).
# Usage: cpu_usage.sh <process-name-pattern> [seconds]
#   e.g. cpu_usage.sh kinect_container 60 > cpu_A.txt
set -euo pipefail

pattern="${1:?usage: $0 <process-name-pattern> [seconds]}"
seconds="${2:-60}"

# pgrep -f also matches shells/wrappers whose command line merely contains the
# pattern (including the one that launched this script): skip those.
pid=""
for candidate in $(pgrep -f -- "${pattern}" || true); do
  comm="$(cat "/proc/${candidate}/comm" 2>/dev/null || true)"
  case "${comm}" in
    ""|bash|sh|zsh|timeout|ros2|python3) continue ;;
  esac
  pid="${candidate}"
done
if [[ -z "${pid}" ]]; then
  echo "No process matching '${pattern}'" >&2
  exit 1
fi

echo "# pid ${pid}: $(tr '\0' ' ' < "/proc/${pid}/cmdline")"
# -u: CPU, -h: one line per sample, -t: per-thread breakdown
pidstat -u -h -t -p "${pid}" 1 "${seconds}"
