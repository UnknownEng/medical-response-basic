#!/usr/bin/env python3
"""Draws the simulated flight and prints how accurate the landing and drop were.

    python3 plot_run.py sim_config.json output/track.csv output/flight_map.png
"""
import csv
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from camera_model import offset_ne  # noqa: E402
from mission_common import parse_qgc_wpl  # noqa: E402

cfg_path, track_path, out_png = sys.argv[1:4]
cfg = json.load(open(cfg_path))
home = cfg["sim"]["home"][:2]
# Skip rows from before the GPS fix (lat/lon 0, 0).
rows = [r for r in csv.DictReader(open(track_path)) if float(r["lat"]) != 0 or float(r["lon"]) != 0]
if not rows:
    sys.exit("no track recorded")


def ne(lat, lon):
    n, e = offset_ne(home[0], home[1], float(lat), float(lon))
    return e, n  # plot x = east, y = north


markers = cfg["aruco"]["sim_world"]["markers"]
real = next(m for m in markers if m["id"] in cfg["aruco"]["allowed_ids"])
target = cfg["target"]
wps = parse_qgc_wpl(os.path.join(os.path.dirname(cfg_path), cfg["mission"]["waypoint_file"]))

colors = {"GUIDED": "tab:blue", "AUTO": "tab:orange", "LAND": "tab:purple", "RTL": "tab:green"}
fig, ax = plt.subplots(figsize=(9, 9))

route = [ne(home[0], home[1])] + [ne(w.x, w.y) for w in wps[1:] if w.command == 16]
ax.plot(*zip(*route), "--", color="grey", lw=1, label="planned search route")

seg, seg_mode = [], None
for r in rows:
    p = ne(r["lat"], r["lon"])
    if r["mode"] != seg_mode and seg:
        ax.plot(*zip(*seg), color=colors.get(seg_mode, "black"), lw=2)
        seg = [seg[-1]]
    seg_mode = r["mode"]
    seg.append(p)
if seg:
    ax.plot(*zip(*seg), color=colors.get(seg_mode, "black"), lw=2)
for mode, c in colors.items():
    ax.plot([], [], color=c, lw=2, label=f"flown in {mode}")

for m in markers:
    x, y = ne(m["lat"], m["lon"])
    good = m is real
    ax.add_patch(plt.Rectangle((x - m["size_m"] / 2, y - m["size_m"] / 2), m["size_m"], m["size_m"],
                               color="black" if good else "grey"))
    ax.annotate(f"ArUco {m['id']}" + ("" if good else " (decoy)"), (x, y), xytext=(6, 6),
                textcoords="offset points")
tx, ty = ne(target["lat"], target["lon"])
ax.plot(tx, ty, "rx", ms=14, mew=3, label="delivery target")
ax.plot(0, 0, "k^", ms=10, label="home")

summary = {}
# Landing on the marker = first time it goes from armed to disarmed.
for prev, cur in zip(rows, rows[1:]):
    if prev["armed"] == "1" and cur["armed"] == "0":
        x, y = ne(cur["lat"], cur["lon"])
        mx, my = ne(real["lat"], real["lon"])
        d = ((x - mx) ** 2 + (y - my) ** 2) ** 0.5
        summary["landing_error_from_marker_m"] = round(d, 2)
        ax.plot(x, y, "o", mfc="none", mec="magenta", ms=14, mew=2, label=f"landed ({d:.2f} m from marker)")
        break
# Servo changes.
last = rows[0]["servo9"]
for r in rows[1:]:
    if r["servo9"] != last and r["servo9"] not in ("0", ""):
        x, y = ne(r["lat"], r["lon"])
        if int(r["servo9"]) == cfg["payload"]["pwm_at_target"]:
            d = ((x - tx) ** 2 + (y - ty) ** 2) ** 0.5
            summary["release_error_from_target_m"] = round(d, 2)
            summary["release_alt_m"] = round(float(r["rel_alt"]), 1)
            ax.plot(x, y, "*", color="gold", mec="black", ms=18,
                    label=f"payload released ({d:.2f} m from target, {float(r['rel_alt']):.1f} m up)")
        last = r["servo9"]
summary["flight_time_s"] = round(float(rows[-1]["t"]) - float(rows[0]["t"]), 1)
summary["ended_disarmed_at_home_m"] = round(sum(v ** 2 for v in ne(rows[-1]["lat"], rows[-1]["lon"])) ** 0.5, 2)

ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("east of home (m)")
ax.set_ylabel("north of home (m)")
ax.set_title("SITL medical delivery run")
ax.legend(loc="lower right", fontsize=8)
fig.tight_layout()
fig.savefig(out_png, dpi=120)
print(json.dumps(summary, indent=2))
