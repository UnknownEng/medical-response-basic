#!/usr/bin/env python3
"""Builds a 1920x1080 replay video of a SITL run.

  +---------------------------+----------------------+--------+
  | chase view (3rd person)   | drone camera         | flight |
  |                           | (downward, ArUco)    | data   |
  +---------------------------+----------------------+--------+
  | terminal (script output)  | map (top-down)                |
  +---------------------------+-------------------------------+

    python3 make_video.py sim_config.json output [--fps 20] [--speed 1]
    python3 make_video.py sim_config.json output --still 95     # one PNG at T+95 s

Everything is rebuilt from the recorded run: track.csv (pose at 10 Hz) and
mission_timed.log (each printed line with the time it was printed). The
camera panel is re-rendered with the same sim camera the mission used, and
the ArUco detector is run on it again.
"""
import argparse
import bisect
import csv
import math
import multiprocessing as mp
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from aruco_watcher import _make_detector  # noqa: E402
from camera_model import CameraModel, offset_ne, rot_body_to_ned  # noqa: E402
from mission_common import load_config, parse_qgc_wpl  # noqa: E402
from sim_camera import SimCamera  # noqa: E402

W, H = 1920, 1080
CH_W, CH_H = 960, 540     # chase view
CAM_W, CAM_H = 720, 540   # drone camera panel
HUD_W = 240
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
MODE_BGR = {"GUIDED": (235, 150, 50), "AUTO": (40, 160, 255), "LAND": (210, 100, 190),
            "RTL": (70, 200, 70)}
DRONE_SCALE = 3.0  # drawn 3x real size so it's visible from the chase camera
G = 9.81


def bgr2rgb(c):
    return (c[2], c[1], c[0])


# ------------------------------------------------------------------ data

def load_track(path):
    rows = []
    for r in csv.DictReader(open(path)):
        lat, lon = float(r["lat"]), float(r["lon"])
        if lat == 0 and lon == 0:
            continue
        rows.append({
            "wall": float(r["wall"]), "lat": lat, "lon": lon, "alt": float(r["rel_alt"]),
            "mode": r["mode"], "armed": r["armed"] == "1", "servo": int(r["servo9"] or 0),
            "roll": float(r["roll"]), "pitch": float(r["pitch"]), "yaw": float(r["yaw"]),
            "speed": float(r["groundspeed"]), "batt": int(r["battery_pct"]),
        })
    if not rows:
        sys.exit("track.csv has no positions")
    return rows


def load_log(path):
    lines = []
    for raw in open(path, encoding="utf-8", errors="replace"):
        t, _, text = raw.rstrip("\n").partition("\t")
        try:
            lines.append((float(t), text))
        except ValueError:
            continue
    return lines


def phase_timeline(log):
    """(time, label) changes, worked out from what the script printed."""
    out, landed = [], False
    rules = [
        ("Waiting for flight controller", "CONNECTING"),
        ("Waiting for GPS", "PRE-FLIGHT CHECKS"),
        ("Taking off", "TAKING OFF"),
        ("Flying the search route", "SEARCHING FOR MARKER"),
        ("confirmed", "MARKER FOUND: CENTERING"),
        ("Landing.", "LANDING ON MARKER"),
        ("Landed and disarmed", "ON GROUND AT MARKER"),
        ("Over the target", "PAYLOAD RELEASED"),
        ("Mode -> RTL", "RETURNING HOME"),
        ("Mission complete", "MISSION COMPLETE"),
        ("ABORT", "ABORTED"),
    ]
    for t, text in log:
        if "Landed and disarmed" in text:
            landed = True
        if landed and "Flying to" in text:
            out.append((t, "FLYING TO DELIVERY TARGET"))
            continue
        for key, label in rules:
            if key in text:
                out.append((t, label))
                break
    return out


def first_time(log, needle):
    return next((t for t, text in log if needle in text), None)


class Track:
    def __init__(self, rows):
        self.rows = rows
        self.t = [r["wall"] for r in rows]

    def index(self, t):
        return bisect.bisect_right(self.t, t)

    def at(self, t):
        i = self.index(t)
        if i <= 0:
            return dict(self.rows[0])
        if i >= len(self.rows):
            return dict(self.rows[-1])
        a, b = self.rows[i - 1], self.rows[i]
        f = (t - a["wall"]) / max(b["wall"] - a["wall"], 1e-6)
        out = dict(a)
        for k in ("lat", "lon", "alt", "roll", "pitch", "speed"):
            out[k] = a[k] + (b[k] - a[k]) * f
        dy = (b["yaw"] - a["yaw"] + math.pi) % (2 * math.pi) - math.pi
        out["yaw"] = a["yaw"] + dy * f
        return out


# ------------------------------------------------------------------ renderer

class Renderer:
    def __init__(self, cfg, out_dir):
        self.cfg = cfg
        self.track = Track(load_track(os.path.join(out_dir, "track.csv")))
        self.log = load_log(os.path.join(out_dir, "mission_timed.log"))
        self.phases = phase_timeline(self.log)
        a = cfg["aruco"]
        self.cam_model = CameraModel(*a["frame_size"], a["hfov_deg"])
        self.sim = SimCamera(a["sim_world"] | {"save_dir": None}, lambda: None, self.cam_model, a["dictionary"])
        self.detect = _make_detector(a["dictionary"])
        self.allowed = set(a["allowed_ids"])
        self.home = tuple(cfg["sim"]["home"][:2])
        self.sim.origin = self.home
        self.target = (cfg["target"]["lat"], cfg["target"]["lon"])
        self.markers = a["sim_world"]["markers"]
        self.real = next(m for m in self.markers if m["id"] in self.allowed)
        items = parse_qgc_wpl(cfg["mission"]["waypoint_file"])
        self.route = [self.ne(*self.home)] + [self.ne(w.x, w.y) for w in items[1:] if w.command == 16]
        self.chase_k = CameraModel(CH_W, CH_H, 75).K
        self.pwm_marker = cfg["payload"]["pwm_at_marker"]
        self.pwm_target = cfg["payload"]["pwm_at_target"]

        # Key moments from the log and track.
        self.t0 = self.log[0][0] if self.log else self.track.t[0]
        self.watch = []
        s, d, land = (first_time(self.log, k) for k in ("Flying the search route", "confirmed", "Landing."))
        if s:
            self.watch.append((s, d or 1e18))
        if d and land:
            self.watch.append((d, land))
        self.release_t, self.release_pt, self.land_pt = None, None, None
        rows = self.track.rows
        for p, c in zip(rows, rows[1:]):
            if self.land_pt is None and p["armed"] and not c["armed"]:
                self.land_pt = c
            if self.release_t is None and c["servo"] == self.pwm_target and p["servo"] != self.pwm_target:
                self.release_t, self.release_pt = c["wall"], c
        self.loaded_t = next((r["wall"] for r in rows if r["servo"] == self.pwm_marker), None)

        self.font = {s: ImageFont.truetype(FONT, s) for s in (13, 15, 17, 22)}
        self.font_b = {s: ImageFont.truetype(FONT_B, s) for s in (14, 17, 20, 26)}
        self.map_bg, self.map_tf = self._map_background()
        self._term_cache = {}

    def ne(self, lat, lon):
        return offset_ne(self.home[0], self.home[1], lat, lon)

    def phase(self, t):
        label = "STARTING"
        for pt, name in self.phases:
            if pt <= t:
                label = name
        return label

    def watching(self, t):
        return any(a <= t < b for a, b in self.watch)

    # ---------------------------------------------------- chase view

    def chase(self, p, yaw_c, t):
        alt = max(p["alt"], 0.0)
        back, up = 16.0, 9.0
        cam = np.array([-back * math.cos(yaw_c), -back * math.sin(yaw_c), -(alt + up)])
        look = np.array([0.0, 0.0, -alt]) - cam
        z = look / np.linalg.norm(look)
        x = np.cross([0.0, 0.0, 1.0], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        r_cn = np.vstack([x, y, z])
        hmat = self.chase_k @ r_cn @ np.array([[1, 0, -cam[0]], [0, 1, -cam[1]], [0, 0, -cam[2]]])
        img = self.sim.render_view(p["lat"], p["lon"], hmat, (CH_W, CH_H), marker_radius_m=250)

        # distance haze towards the top of the frame
        haze = np.linspace(0.55, 0.0, int(CH_H * 0.45), dtype=np.float32)[:, None, None]
        top = img[:len(haze)].astype(np.float32)
        img[:len(haze)] = (top * (1 - haze) + np.array([235, 215, 190], np.float32) * haze).astype(np.uint8)

        dn, de = self.ne(p["lat"], p["lon"])

        def project(pts):
            pts = np.asarray(pts, float).reshape(-1, 3)
            pc = (r_cn @ (pts - cam).T).T
            ok = pc[:, 2] > 0.5
            uv = (self.chase_k @ pc.T).T
            uv = uv[:, :2] / np.maximum(uv[:, 2:3], 1e-6)
            return uv, ok

        def line3d(pts, color, th=2, step=2.0, dashed=False):
            pts = np.asarray(pts, float)
            dense = [pts[0]]
            for a, b in zip(pts, pts[1:]):
                n = max(1, int(np.linalg.norm(b - a) / step))
                dense += [a + (b - a) * (k / n) for k in range(1, n + 1)]
            uv, ok = project(dense)
            for k in range(len(dense) - 1):
                if ok[k] and ok[k + 1] and (not dashed or k % 2 == 0):
                    cv2.line(img, tuple(np.int32(uv[k])), tuple(np.int32(uv[k + 1])), color, th, cv2.LINE_AA)

        def rel(n, e, d=0.0):
            return [n - dn, e - de, d]

        # planned route on the ground, flown trail at altitude
        line3d([rel(n, e) for n, e in self.route], (230, 230, 230), 1, dashed=True)
        i = self.track.index(t)
        trail = self.track.rows[:i:2]
        for a_, b_ in zip(trail, trail[1:]):
            pa, pb = rel(*self.ne(a_["lat"], a_["lon"]), -a_["alt"]), rel(*self.ne(b_["lat"], b_["lon"]), -b_["alt"])
            line3d([pa, pb], MODE_BGR.get(b_["mode"], (200, 200, 200)), 2, step=5)

        # target and home rings with labels
        def ring(lat, lon, radius, color, label):
            n, e = self.ne(lat, lon)
            pts = [rel(n + radius * math.cos(a), e + radius * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 25)]
            line3d(pts, color, 3, step=1)
            line3d([rel(n, e), rel(n, e, -5)], color, 2, step=1)
            uv, ok = project([rel(n, e, -5.5)])
            if ok[0]:
                cv2.putText(img, label, tuple(np.int32(uv[0]) + [-20, 0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2, cv2.LINE_AA)
        ring(*self.target, 2.0, (40, 40, 230), "TARGET")
        ring(*self.home, 2.0, (40, 220, 240), "HOME")
        for m in self.markers:
            n, e = self.ne(m["lat"], m["lon"])
            uv, ok = project([rel(n, e, -2.0)])
            if ok[0]:
                lbl = f"ID {m['id']}" + ("" if m["id"] in self.allowed else " decoy")
                cv2.putText(img, lbl, tuple(np.int32(uv[0]) + [-25, 0]), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (255, 255, 255), 1, cv2.LINE_AA)

        # payload: under the drone while loaded, falls at release, then lies at the drop point
        if self.release_pt and t >= self.release_t:
            rn, re_ = self.ne(self.release_pt["lat"], self.release_pt["lon"])
            fall = 0.5 * G * (t - self.release_t) ** 2
            h0 = self.release_pt["alt"]
            self._box(img, project, rel(rn, re_, -max(h0 - fall, 0.0) - 0.3), 0.6)
        elif self.loaded_t and t >= self.loaded_t:
            self._box(img, project, [0, 0, -alt + 0.25 * DRONE_SCALE], 0.6)

        self._drone(img, project, p, alt, t)
        self._label(img, "CHASE CAM  (3rd person view, drone drawn 3x size)")
        return img

    def _box(self, img, project, centre, size):
        c = np.array(centre, float)
        s = size / 2
        pts = [c + [dx, dy, 0] for dx, dy in ((-s, -s), (-s, s), (s, s), (s, -s))]
        uv, ok = project(pts)
        if ok.all():
            cv2.fillConvexPoly(img, np.int32(uv), (40, 140, 240), cv2.LINE_AA)
            cv2.polylines(img, [np.int32(uv)], True, (20, 60, 120), 1, cv2.LINE_AA)

    def _drone(self, img, project, p, alt, t):
        r_bn = rot_body_to_ned(p["roll"], p["pitch"], p["yaw"])
        s = DRONE_SCALE
        arm_ends = [np.array([dx, dy, 0.0]) * 0.3 * s for dx, dy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        ang = np.linspace(0, 2 * math.pi, 17)
        centre = np.array([0.0, 0.0, -alt])

        def world(pb, ground=False):
            w_ = centre + r_bn @ pb
            if ground:
                w_ = w_.copy()
                w_[2] = 0.0
            return w_

        for ground in (True, False):  # shadow first, then the drone
            overlay = img.copy() if ground else img
            for k, end in enumerate(arm_ends):
                uv, ok = project([world(np.zeros(3), ground), world(end, ground)])
                if ok.all():
                    cv2.line(overlay, tuple(np.int32(uv[0])), tuple(np.int32(uv[1])),
                             (20, 20, 20) if not ground else (30, 40, 30), 4, cv2.LINE_AA)
                rotor = [world(end + 0.16 * s * np.array([math.cos(a), math.sin(a), -0.02]), ground) for a in ang]
                uv, ok = project(rotor)
                if ok.all():
                    if ground:
                        cv2.fillPoly(overlay, [np.int32(uv)], (30, 40, 30), cv2.LINE_AA)
                    else:
                        front = end[0] > 0
                        col = (60, 60, 230) if front else (210, 210, 210)
                        if p["armed"]:
                            disc = img.copy()
                            cv2.fillPoly(disc, [np.int32(uv)], col, cv2.LINE_AA)
                            cv2.addWeighted(disc, 0.35, img, 0.65, 0, img)
                        cv2.polylines(img, [np.int32(uv)], True, col, 2, cv2.LINE_AA)
            body = [world(np.array([dx, dy, 0.0]) * 0.09 * s, ground) for dx, dy in ((1.4, 0), (0, 1), (-1, 0), (0, -1))]
            uv, ok = project(body)
            if ok.all():
                cv2.fillConvexPoly(overlay, np.int32(uv), (30, 40, 30) if ground else (25, 25, 25), cv2.LINE_AA)
            if ground:
                cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)

    # ---------------------------------------------------- drone camera

    def camera(self, p, t):
        pose = (p["lat"], p["lon"], p["alt"], p["roll"], p["pitch"], p["yaw"])
        img = self.sim.render_pose(pose)
        # painted drop-zone and home rings on the ground (visual only, not ArUco)
        hmat = self.cam_model.ground_to_pixel_h(max(p["alt"], 0.3), p["roll"], p["pitch"], p["yaw"])
        dn, de = self.ne(p["lat"], p["lon"])
        for (lat, lon), col in ((self.target, (40, 40, 220)), (self.home, (40, 220, 240))):
            n, e = self.ne(lat, lon)
            if math.hypot(n - dn, e - de) > 40:
                continue
            ring = np.array([[n - dn + 2.0 * math.cos(a), e - de + 2.0 * math.sin(a), 1.0]
                             for a in np.linspace(0, 2 * math.pi, 40)])
            uv = (hmat @ ring.T).T
            if (uv[:, 2] > 0).all():
                cv2.polylines(img, [np.int32(uv[:, :2] / uv[:, 2:3])], True, col, 3, cv2.LINE_AA)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)
        if ids is not None:
            for c, mid in zip(corners, ids.flatten()):
                pts = np.int32(c.reshape(-1, 2))
                good = int(mid) in self.allowed
                col = (60, 220, 60) if good else (0, 165, 255)
                cv2.polylines(img, [pts], True, col, 2, cv2.LINE_AA)
                txt = f"ID {int(mid)}" + (" TARGET MARKER" if good else " ignored (not allowed)")
                org = tuple(pts.min(axis=0) + [0, -8])
                cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        cx, cy = img.shape[1] // 2, img.shape[0] // 2
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)
        img = cv2.resize(img, (CAM_W, CAM_H), interpolation=cv2.INTER_LINEAR)
        on = self.watching(t)
        badge = "ArUco detector: WATCHING" if on else "ArUco detector: idle"
        col = (60, 220, 60) if on else (170, 170, 170)
        cv2.rectangle(img, (CAM_W - 265, 8), (CAM_W - 8, 34), (0, 0, 0), -1)
        cv2.putText(img, badge, (CAM_W - 258, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)
        self._label(img, "DRONE CAMERA  (downward Pi camera, 62 deg FOV)")
        return img

    # ---------------------------------------------------- HUD

    def hud(self, p, t):
        im = Image.new("RGB", (HUD_W, CH_H), (18, 20, 24))
        d = ImageDraw.Draw(im)
        y = 12
        d.text((12, y), "FLIGHT DATA", font=self.font_b[14], fill=(150, 160, 175))
        el = max(0, t - self.t0)
        d.text((128, y - 1), f"T+{int(el // 60):02d}:{el % 60:04.1f}", font=self.font_b[17], fill=(235, 238, 242))
        y += 34
        mode = p["mode"] or "-"
        d.text((12, y), mode, font=self.font_b[26], fill=bgr2rgb(MODE_BGR.get(mode, (220, 220, 220))))
        y += 36
        d.text((12, y), "ARMED" if p["armed"] else "DISARMED", font=self.font_b[17],
               fill=(235, 80, 70) if p["armed"] else (140, 200, 140))
        y += 34
        n, e = self.ne(p["lat"], p["lon"])

        def dist(lat, lon):
            n2, e2 = self.ne(lat, lon)
            return math.hypot(n2 - n, e2 - e)

        servo = p["servo"]
        servo_txt = {self.pwm_marker: f"{servo} LOADED", self.pwm_target: f"{servo} RELEASED"}.get(servo, str(servo or "-"))
        rows = [
            ("ALT", f"{p['alt']:6.1f} m"),
            ("SPEED", f"{p['speed']:6.1f} m/s"),
            ("HEADING", f"{math.degrees(p['yaw']) % 360:6.0f} deg"),
            ("BATTERY", f"{p['batt']:6d} %" if p["batt"] >= 0 else "     - %"),
            ("SERVO 9", servo_txt),
            ("", ""),
            ("TO MARKER", f"{dist(self.real['lat'], self.real['lon']):6.1f} m"),
            ("TO TARGET", f"{dist(*self.target):6.1f} m"),
            ("TO HOME", f"{dist(*self.home):6.1f} m"),
        ]
        for k, v in rows:
            d.text((12, y), k, font=self.font[13], fill=(130, 140, 155))
            d.text((12, y + 15), v, font=self.font_b[17], fill=(235, 238, 242))
            y += 40 if k else 10
        d.rectangle((0, CH_H - 78, HUD_W, CH_H), fill=(32, 36, 44))
        d.text((12, CH_H - 70), "MISSION PHASE", font=self.font[13], fill=(130, 140, 155))
        words, line, yy = self.phase(t).split(), "", CH_H - 52
        for wd in words:
            if len(line) + len(wd) + 1 > 17:
                d.text((12, yy), line, font=self.font_b[17], fill=(255, 210, 90))
                yy += 21
                line = wd
            else:
                line = (line + " " + wd).strip()
        d.text((12, yy), line, font=self.font_b[17], fill=(255, 210, 90))
        return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)

    # ---------------------------------------------------- terminal

    def terminal(self, t):
        n = bisect.bisect_right([lt for lt, _ in self.log], t)
        blink = int(t * 2) % 2
        key = (n, blink)
        if key in self._term_cache:
            return self._term_cache[key]
        im = Image.new("RGB", (960, 540), (12, 12, 14))
        d = ImageDraw.Draw(im)
        d.rectangle((0, 0, 960, 28), fill=(44, 44, 50))
        for k, c in enumerate(((237, 106, 94), (245, 191, 79), (98, 197, 84))):
            d.ellipse((12 + k * 20, 9, 24 + k * 20, 21), fill=c)
        d.text((80, 6), "user@wsl: ~/medical-response-basic-main  (terminal)", font=self.font[15], fill=(200, 200, 205))
        f = self.font[15]
        lh, max_chars = 19, 104
        lines = [("$ python3 mission_pymavlink.py --config sim/sim_config.json", (110, 220, 110))]
        for _, text in self.log[:n]:
            low = text
            if "ABORT" in low or "WARNING" in low or "rejected" in low or "refused" in low:
                col = (240, 110, 100)
            elif "FC:" in low:
                col = (140, 145, 155)
            elif any(k in low for k in ("confirmed", "complete", "Armed.", "locked", "Landed", "OK")):
                col = (120, 230, 120)
            elif "Mode ->" in low:
                col = (110, 200, 240)
            elif "Servo" in low or "Over the target" in low:
                col = (250, 210, 90)
            else:
                col = (225, 225, 230)
            while len(text) > max_chars:
                lines.append((text[:max_chars], col))
                text = "    " + text[max_chars:]
            lines.append((text, col))
        visible = (540 - 40) // lh
        y = 36
        for text, col in lines[-visible:]:
            d.text((10, y), text, font=f, fill=col)
            y += lh
        if blink:
            d.rectangle((10, y + 2, 19, y + 17), fill=(200, 200, 200))
        out = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
        if len(self._term_cache) > 8:
            self._term_cache.clear()
        self._term_cache[key] = out
        return out

    # ---------------------------------------------------- map

    def _map_background(self):
        pts = list(self.route) + [self.ne(*self.target), (0.0, 0.0)] + \
              [self.ne(m["lat"], m["lon"]) for m in self.markers]
        ns, es = [p[0] for p in pts], [p[1] for p in pts]
        n0, n1, e0, e1 = min(ns) - 15, max(ns) + 15, min(es) - 25, max(es) + 25
        mw, mh = 960, 540
        sc = min((mw - 40) / (e1 - e0), (mh - 60) / (n1 - n0))
        cx, cy = mw / 2, mh / 2 + 10
        nc, ec = (n0 + n1) / 2, (e0 + e1) / 2

        def tf(n, e):
            return int(cx + (e - ec) * sc), int(cy - (n - nc) * sc)

        img = np.full((mh, mw, 3), (34, 38, 32), np.uint8)
        for g in range(int(e0 // 20) * 20, int(e1) + 20, 20):
            cv2.line(img, tf(n0 - 50, g), tf(n1 + 50, g), (48, 54, 46), 1)
        for g in range(int(n0 // 20) * 20, int(n1) + 20, 20):
            cv2.line(img, tf(g, e0 - 50), tf(g, e1 + 50), (48, 54, 46), 1)
        for a, b in zip(self.route, self.route[1:]):
            n_seg = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 3))
            for k in range(0, n_seg, 2):
                p0 = (a[0] + (b[0] - a[0]) * k / n_seg, a[1] + (b[1] - a[1]) * k / n_seg)
                p1 = (a[0] + (b[0] - a[0]) * (k + 1) / n_seg, a[1] + (b[1] - a[1]) * (k + 1) / n_seg)
                cv2.line(img, tf(*p0), tf(*p1), (150, 150, 150), 1, cv2.LINE_AA)
        for m in self.markers:
            x, y = tf(*self.ne(m["lat"], m["lon"]))
            good = m["id"] in self.allowed
            cv2.rectangle(img, (x - 6, y - 6), (x + 6, y + 6), (255, 255, 255) if good else (140, 140, 140), -1)
            cv2.rectangle(img, (x - 3, y - 3), (x + 3, y + 3), (0, 0, 0), -1)
            cv2.putText(img, f"ArUco {m['id']}" + ("" if good else " (decoy)"), (x + 10, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        x, y = tf(*self.ne(*self.target))
        cv2.drawMarker(img, (x, y), (60, 60, 240), cv2.MARKER_TILTED_CROSS, 18, 3)
        cv2.putText(img, "TARGET", (x + 12, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 250), 1, cv2.LINE_AA)
        x, y = tf(0, 0)
        cv2.drawMarker(img, (x, y), (40, 220, 240), cv2.MARKER_TRIANGLE_UP, 16, 3)
        cv2.putText(img, "HOME", (x + 12, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 240), 1, cv2.LINE_AA)
        # scale bar + legend
        bx, by = 20, mh - 20
        cv2.line(img, (bx, by), (bx + int(20 * sc), by), (230, 230, 230), 2)
        cv2.putText(img, "20 m", (bx, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        lx = mw - 120
        for k, (mode, col) in enumerate(MODE_BGR.items()):
            cv2.line(img, (lx, 50 + k * 20), (lx + 25, 50 + k * 20), col, 3)
            cv2.putText(img, mode, (lx + 32, 55 + k * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        return img, tf

    def map(self, p, t):
        img = self.map_bg.copy()
        tf = self.map_tf
        pose = (p["lat"], p["lon"], max(p["alt"], 0.3), p["roll"], p["pitch"], p["yaw"])
        # camera footprint on the ground
        w_, h_ = self.cam_model.size
        foot = []
        for u, v in ((0, 0), (w_, 0), (w_, h_), (0, h_)):
            ll = self.cam_model.pixel_to_latlon(u, v, pose)
            if ll:
                foot.append(tf(*self.ne(*ll)))
        if len(foot) == 4 and p["alt"] > 0.5:
            ov = img.copy()
            cv2.fillConvexPoly(ov, np.int32(foot), (200, 200, 60))
            cv2.addWeighted(ov, 0.25, img, 0.75, 0, img)
            cv2.polylines(img, [np.int32(foot)], True, (230, 230, 90), 1, cv2.LINE_AA)
        i = self.track.index(t)
        trail = self.track.rows[:i]
        for a, b in zip(trail, trail[1:]):
            cv2.line(img, tf(*self.ne(a["lat"], a["lon"])), tf(*self.ne(b["lat"], b["lon"])),
                     MODE_BGR.get(b["mode"], (200, 200, 200)), 2, cv2.LINE_AA)
        if self.land_pt and t >= self.land_pt["wall"]:
            cv2.circle(img, tf(*self.ne(self.land_pt["lat"], self.land_pt["lon"])), 11, (230, 80, 230), 2, cv2.LINE_AA)
        if self.release_pt and t >= self.release_t:
            x, y = tf(*self.ne(self.release_pt["lat"], self.release_pt["lon"]))
            cv2.drawMarker(img, (x, y), (40, 200, 255), cv2.MARKER_STAR, 22, 2)
        # drone arrow
        x, y = tf(*self.ne(p["lat"], p["lon"]))
        yw = p["yaw"]
        tip = (x + 14 * math.sin(yw), y - 14 * math.cos(yw))
        left = (x + 8 * math.sin(yw - 2.5), y - 8 * math.cos(yw - 2.5))
        right = (x + 8 * math.sin(yw + 2.5), y - 8 * math.cos(yw + 2.5))
        cv2.fillConvexPoly(img, np.int32([tip, left, right]), MODE_BGR.get(p["mode"], (255, 255, 255)), cv2.LINE_AA)
        cv2.polylines(img, [np.int32([tip, left, right])], True, (255, 255, 255), 1, cv2.LINE_AA)
        self._label(img, "MAP  (top-down; yellow box = what the camera sees)")
        return img

    # ---------------------------------------------------- compose

    @staticmethod
    def _label(img, text):
        cv2.rectangle(img, (0, 0), (len(text) * 9 + 16, 26), (0, 0, 0), -1)
        cv2.putText(img, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def frame(self, t, yaw_c):
        p = self.track.at(t)
        out = np.zeros((H, W, 3), np.uint8)
        out[:CH_H, :CH_W] = self.chase(p, yaw_c, t)
        out[:CH_H, CH_W:CH_W + CAM_W] = self.camera(p, t)
        out[:CH_H, CH_W + CAM_W:] = self.hud(p, t)
        out[CH_H:, :960] = self.terminal(t)
        out[CH_H:, 960:] = self.map(p, t)
        cv2.line(out, (0, CH_H), (W, CH_H), (70, 70, 70), 2)
        cv2.line(out, (CH_W, 0), (CH_W, H), (70, 70, 70), 2)
        cv2.line(out, (CH_W + CAM_W, 0), (CH_W + CAM_W, CH_H), (70, 70, 70), 2)
        return out


# ------------------------------------------------------------------ main

_R = None


def _init(cfg_path, out_dir):
    global _R
    cv2.setNumThreads(1)
    _R = Renderer(load_config(cfg_path), out_dir)


def _render(spec):
    t, yaw_c = spec
    return _R.frame(t, yaw_c)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config")
    ap.add_argument("out_dir")
    ap.add_argument("--fps", type=float, default=20)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (2 = twice as fast)")
    ap.add_argument("--still", type=float, help="write one PNG at this many seconds into the run")
    ap.add_argument("--workers", type=int, default=max(1, min(6, (os.cpu_count() or 2) - 1)))
    args = ap.parse_args()

    r = Renderer(load_config(args.config), args.out_dir)
    t_start = r.t0 - 0.5
    t_end = max(r.track.t[-1], r.log[-1][0] if r.log else 0) + 3.0

    # Chase camera heading follows the drone's heading, smoothed (time constant 1.5 s).
    n = int((t_end - t_start) * args.fps / args.speed)
    times = [t_start + k * args.speed / args.fps for k in range(n)]
    yaws, yc = [], None
    for t in times:
        target = r.track.at(t)["yaw"]
        if yc is None:
            yc = target
        dy = (target - yc + math.pi) % (2 * math.pi) - math.pi
        yc += dy * min(1.0, (args.speed / args.fps) / 1.5)
        yaws.append(yc)

    if args.still is not None:
        k = min(n - 1, int(args.still * args.fps / args.speed))
        path = os.path.join(args.out_dir, f"still_{int(args.still):03d}s.png")
        cv2.imwrite(path, r.frame(times[k], yaws[k]))
        print(path)
        return

    path = os.path.join(args.out_dir, "mission_replay.mp4")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"avc1"), args.fps, (W, H))
    if not writer.isOpened():
        path = path.replace(".mp4", "_mp4v.mp4")
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (W, H))
    print(f"Rendering {n} frames ({n / args.fps:.0f} s of video) with {args.workers} workers -> {path}", flush=True)
    with mp.Pool(args.workers, initializer=_init, initargs=(args.config, args.out_dir)) as pool:
        for k, img in enumerate(pool.imap(_render, zip(times, yaws), chunksize=8)):
            writer.write(img)
            if k % max(1, n // 20) == 0:
                print(f"  {100 * k // n:3d}%", flush=True)
    writer.release()
    print(f"Done: {path}")


if __name__ == "__main__":
    main()
