#!/usr/bin/env bash
# Print the jog/path trace from the NEWEST ur_servo_controller runtime log.
# Robust "newest" = grep all matching per-process logs, then sort by mtime.
#   setup/jog_log.sh            # tail the jog/TCP/path trace
#   setup/jog_log.sh -f path    # show only path-angle lines
#   setup/jog_log.sh -p         # just print the log file path
#   setup/jog_log.sh -c         # show the 125 Hz TCP path CSV (path + tail)
set -euo pipefail
dir="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ "${1:-}" = "-c" ]; then
  csv="$dir/tcp_path.csv"
  [ -f "$csv" ] || { echo "no $csv yet — run the robot first" >&2; exit 1; }
  echo "# $csv ($(wc -l < "$csv") rows)"; tail -40 "$csv"
  exit 0
fi
f=$(grep -lZ ur_servo_controller "$dir"/python3_*.log 2>/dev/null \
      | xargs -0 ls -t 2>/dev/null | head -1)
[ -n "${f:-}" ] || { echo "no ur_servo_controller log found in $dir" >&2; exit 1; }
echo "# $f  ($(stat -c %y "$f" | cut -d. -f1))"
case "${1:-}" in
  -p) ;;                                            # path only (already printed)
  -f) grep -E "${2:-path }" "$f" | tail -60 ;;
  *)  grep -E '\[jog\] (moving|stopped|TCP rpy|cmd=)' "$f" | tail -60 ;;
esac
