#!/usr/bin/env bash
# Full SITL run in WSL: ArduCopter SITL + simulated ArUco camera + mission.
#   bash sim/run_sim.sh
# ARDUPILOT=/path/to/ardupilot overrides where the SITL build lives.
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(dirname "$HERE")"
ARDUPILOT="${ARDUPILOT:-$HOME/sim-deps/ardupilot}"
SITL_BIN="$ARDUPILOT/build/sitl/bin/arducopter"
PY="${PY:-/usr/bin/python3}"
OUT="$HERE/output"

[ -x "$SITL_BIN" ] || { echo "No SITL build at $SITL_BIN (set ARDUPILOT=...)"; exit 1; }
if pgrep -x arducopter >/dev/null; then echo "Another arducopter SITL is running; stop it first."; exit 1; fi

rm -rf "$OUT"; mkdir -p "$OUT"
HOME_LOC="$("$PY" -c "import json;h=json.load(open('$HERE/sim_config.json'))['sim']['home'];print(','.join(map(str,h)))")"
RUNDIR="$(mktemp -d /tmp/medical_sitl.XXXX)"

echo "Starting ArduCopter SITL at $HOME_LOC (work dir $RUNDIR)"
( cd "$RUNDIR" && exec "$SITL_BIN" -w --model + --speedup 1 -I0 --home "$HOME_LOC" \
    --defaults "$ARDUPILOT/Tools/autotest/default_params/copter.parm,$HERE/sim.parm" ) > "$OUT/sitl.log" 2>&1 &
SITL_PID=$!
REC_PID=""
cleanup() { [ -n "$REC_PID" ] && kill "$REC_PID" 2>/dev/null; kill "$SITL_PID" 2>/dev/null; wait 2>/dev/null; }
trap cleanup EXIT

"$PY" "$HERE/recorder.py" tcp:127.0.0.1:5762 "$OUT/track.csv" > "$OUT/recorder.log" 2>&1 &
REC_PID=$!
sleep 2

cd "$ROOT"
# mission.log is the plain output; mission_timed.log adds the exact time of each
# line so make_video.py can sync the terminal panel with the flight.
"$PY" -u mission_pymavlink.py --config "$HERE/sim_config.json" 2>&1 | while IFS= read -r line; do
    printf '%s\t%s\n' "$EPOCHREALTIME" "$line" >> "$OUT/mission_timed.log"
    printf '%s\n' "$line"
done | tee "$OUT/mission.log"
STATUS=${PIPESTATUS[0]}
sleep 2
kill "$REC_PID" 2>/dev/null; REC_PID=""

"$PY" "$HERE/plot_run.py" "$HERE/sim_config.json" "$OUT/track.csv" "$OUT/flight_map.png" | tee "$OUT/summary.json"
if [ "${VIDEO:-0}" = "1" ]; then
    "$PY" "$HERE/make_video.py" "$HERE/sim_config.json" "$OUT"
fi
echo "Mission exit code: $STATUS. Results in $OUT"
exit "$STATUS"
