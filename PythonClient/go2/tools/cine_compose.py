#!/usr/bin/env python3
"""Compose the "walking together" video: cinematic frame + both robots' onboard sensor panels, then encode.

  python3 cine_compose.py RUN_DIR OUT.mp4 [--workers 8] [--limit N] [--frame K (write one PNG only)]
RUN_DIR is the output of cine_walk.py (cine/, sens/, telemetry.jsonl, run.json).
"""
import argparse
import json
import math
import os
import subprocess
import sys
from multiprocessing import Pool

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

W, H = 1920, 1080
PW = 312                      # side panel width
FONT_DIR = "/usr/share/fonts/truetype/dejavu"
ROBOT_META = {"go2": ("UNITREE GO2", "quadruped · HIMLoco policy", (90, 200, 255)),
              "g1": ("UNITREE G1", "humanoid · Unitree RL policy", (255, 170, 60))}


def font(size, bold=False):
    try:
        return ImageFont.truetype(os.path.join(FONT_DIR, "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"), size)
    except Exception:
        return ImageFont.load_default()


F = {k: font(*v) for k, v in {"h": (15, True), "s": (12, False), "t": (11, False), "T": (22, True), "cap": (13, False), "big": (34, True), "sub": (18, False)}.items()}


def turbo_lut():
    try:
        import matplotlib
        cm = matplotlib.colormaps["turbo"]
        return (cm(np.linspace(0, 1, 256))[:, :3] * 255).astype(np.uint8)
    except Exception:
        g = np.linspace(0, 255, 256).astype(np.uint8)
        return np.stack([g, g, g], 1)


LUT = turbo_lut()


def depth_to_rgb(d16):
    d = d16.astype(np.float32) / 1000.0
    d = np.clip(d, 0.4, 30.0)
    v = (np.log(d) - math.log(0.4)) / (math.log(30.0) - math.log(0.4))
    idx = (255 - np.clip(v * 255, 0, 255)).astype(np.uint8)   # near = warm
    return LUT[idx]


SENSOR_H = {"go2": 0.42, "g1": 0.39}      # lidar height above ground (m): Go2 body 0.30 + 0.12, G1 pelvis 0.79 - 0.40


def lidar_panel(pts, w, h, rng=15.0, color=(90, 200, 255), sensor_h=0.4):
    """Top-down, robot-centred (forward up, right right). pts in the sensor frame (NED: x fwd, y right, z down)."""
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (12, 14, 20)
    cx, cy = w // 2, h // 2
    ppm = min(w, h) / 2 / rng
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    for r in (5, 10, 15):
        rr = r * ppm
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=(48, 54, 70))
    d.line([(cx, 0), (cx, h)], fill=(40, 46, 60))
    d.line([(0, cy), (w, cy)], fill=(40, 46, 60))
    img = np.asarray(im).copy()
    if len(pts):
        p = pts.astype(np.float32)
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        px = (cx + y * ppm).astype(int)
        py = (cy - x * ppm).astype(int)
        hgt = np.clip((-z + sensor_h) / 2.5, 0, 1)            # height above ground (NED z down) -> hue
        col = LUT[(40 + hgt * 215).astype(np.uint8)]           # ground = blue, objects warm
        for dx in (0, 1):
            for dy in (0, 1):
                qx, qy = px + dx, py + dy
                ok = (qx >= 0) & (qx < w) & (qy >= 0) & (qy < h)
                img[qy[ok], qx[ok]] = col[ok]
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    d.polygon([(cx, cy - 7), (cx - 5, cy + 5), (cx + 5, cy + 5)], fill=color)
    d.text((w - 34, h - 16), "15 m", font=F["t"], fill=(120, 130, 150))
    return im


def imu_panel(hist, k, w, h, color):
    """hist: dict of arrays gyro (N,3), acc (N,3); draws the last 4 s up to frame k."""
    im = Image.new("RGB", (w, h), (12, 14, 20))
    d = ImageDraw.Draw(im)
    fps = hist["fps"]
    n = int(4 * fps)
    k0 = max(0, k - n + 1)
    g = hist["gyro"][k0:k + 1]
    a = hist["acc"][k0:k + 1]
    half = h // 2
    d.line([(0, half), (w, half)], fill=(40, 46, 60))
    d.text((4, 2), "gyro  rad/s", font=F["t"], fill=(150, 160, 180))
    d.text((4, half + 2), "accel m/s²", font=F["t"], fill=(150, 160, 180))
    cols = [(255, 90, 90), (90, 230, 90), (110, 140, 255)]
    if len(g) >= 2:
        xs = np.linspace(w - 1 - (len(g) - 1) * (w - 1) / (n - 1), w - 1, len(g))
        for j in range(3):
            ys = half / 2 - np.clip(g[:, j], -2.5, 2.5) / 2.5 * (half / 2 - 4)
            d.line(list(zip(xs, ys)), fill=cols[j], width=1)
            ys = half + half / 2 - np.clip(a[:, j] + (9.81 if j == 2 else 0.0), -8, 8) / 8 * (half / 2 - 4)
            d.line(list(zip(xs, ys)), fill=cols[j], width=1)
    for j, lab in enumerate("xyz"):
        d.text((w - 40 + j * 12, 2), lab, font=F["t"], fill=cols[j])
    return im


def rounded_panel(base, box, radius=10, alpha=150):
    x0, y0, x1, y1 = box
    ov = Image.new("RGBA", (x1 - x0, y1 - y0), (0, 0, 0, 0))
    d = ImageDraw.Draw(ov)
    d.rounded_rectangle([0, 0, x1 - x0 - 1, y1 - y0 - 1], radius=radius, fill=(8, 10, 16, alpha), outline=(255, 255, 255, 60))
    base.alpha_composite(ov, (x0, y0))


def side_panel(base, run, hist, k, robot, side, rec):
    """Draw one robot's sensor column on the RGBA base."""
    name, sub, col = ROBOT_META[robot]
    x0 = 20 if side == "left" else W - 20 - PW
    y = 150
    iw = PW - 16
    ih = int(iw * 480 / 752)
    sm_w = (iw - 6) // 2
    sm_h = int(sm_w * 480 / 752)
    total = 30 + ih + 6 + sm_h + 6 + 180 + 6 + 92 + 6 + 50 + 16
    rounded_panel(base, (x0, y, x0 + PW, y + total))
    d = ImageDraw.Draw(base)
    d.text((x0 + 10, y + 7), name, font=F["h"], fill=col)
    d.text((x0 + 10, y + 24 - 2), sub, font=F["t"], fill=(170, 180, 200))
    yy = y + 36
    sens = os.path.join(run, "sens")
    try:
        rgb = Image.open(os.path.join(sens, "%s_rgb_%05d.jpg" % (robot, k))).convert("RGB").resize((iw, ih), Image.BILINEAR)
    except Exception:
        rgb = Image.new("RGB", (iw, ih), (20, 20, 20))
    base.paste(rgb, (x0 + 8, yy))
    d.text((x0 + 12, yy + ih - 16), "front_center · RGB 752×480", font=F["t"], fill=(230, 230, 230))
    yy += ih + 6
    try:
        d16 = np.asarray(Image.open(os.path.join(sens, "%s_depth_%05d.png" % (robot, k))))
        dep = Image.fromarray(depth_to_rgb(d16)).resize((sm_w, sm_h), Image.BILINEAR)
    except Exception:
        dep = Image.new("RGB", (sm_w, sm_h), (20, 20, 20))
    try:
        seg = Image.open(os.path.join(sens, "%s_seg_%05d.png" % (robot, k))).convert("RGB").resize((sm_w, sm_h), Image.NEAREST)
    except Exception:
        seg = Image.new("RGB", (sm_w, sm_h), (20, 20, 20))
    base.paste(dep, (x0 + 8, yy))
    base.paste(seg, (x0 + 8 + sm_w + 6, yy))
    d.text((x0 + 12, yy + sm_h - 15), "depth", font=F["t"], fill=(230, 230, 230))
    d.text((x0 + 12 + sm_w + 6, yy + sm_h - 15), "segmentation", font=F["t"], fill=(230, 230, 230))
    yy += sm_h + 6
    try:
        pts = np.load(os.path.join(sens, "%s_lidar_%05d.npy" % (robot, k))).astype(np.float32)
    except Exception:
        pts = np.zeros((0, 3), np.float32)
    lp = lidar_panel(pts, iw, 180, color=col, sensor_h=SENSOR_H[robot])
    base.paste(lp, (x0 + 8, yy))
    d.text((x0 + 12, yy + 3), "LiDAR 16 ch · %d pts" % len(pts), font=F["t"], fill=(230, 230, 230))
    yy += 180 + 6
    base.paste(imu_panel(hist[robot], k, iw, 92, col), (x0 + 8, yy))
    yy += 92 + 6
    r = rec["robots"][robot]
    spd = r.get("speed", math.hypot(r["vel"][0], r["vel"][1]))
    d.text((x0 + 10, yy), "odom  x %+6.2f  y %+6.2f m   yaw %+4.0f°" % (r["pos"][0], r["pos"][1], math.degrees(r["yaw_ned"])), font=F["t"], fill=(220, 225, 235))
    d.text((x0 + 10, yy + 15), "speed %.2f m/s   cmd vx %.2f vy %+.2f ω %+.2f" % (spd, r["cmd"][0], r["cmd"][1], r["cmd"][2]), font=F["t"], fill=(220, 225, 235))
    d.text((x0 + 10, yy + 30), "GPS %.6f, %.6f  alt %.1f m" % (r["gps"][0], r["gps"][1], r["gps"][2]), font=F["t"], fill=(220, 225, 235))


def minimap(base, recs, k, box):
    x0, y0, x1, y1 = box
    rounded_panel(base, box, alpha=140)
    d = ImageDraw.Draw(base)
    w, h = x1 - x0 - 16, y1 - y0 - 30
    pts = {r: np.array([[rec["robots"][r]["pos"][0], rec["robots"][r]["pos"][1]] for rec in recs]) for r in ("go2", "g1")}
    allp = np.concatenate(list(pts.values()))
    mn, mx = allp.min(0), allp.max(0)
    span = max(mx - mn)
    span = max(span, 6.0)
    sc = min(w, h) / span * 0.9
    cxy = (mn + mx) / 2
    def P(p):   # NED x north (up), y east (right)
        return x0 + 8 + w / 2 + (p[1] - cxy[1]) * sc, y0 + 22 + h / 2 - (p[0] - cxy[0]) * sc
    d.text((x0 + 10, y0 + 6), "formation · ground truth odometry", font=F["t"], fill=(170, 180, 200))
    for r, col in (("go2", ROBOT_META["go2"][2]), ("g1", ROBOT_META["g1"][2])):
        tr = pts[r][:k + 1]
        if len(tr) >= 2:
            d.line([P(p) for p in tr], fill=col, width=2)
        px, py = P(tr[-1])
        d.ellipse([px - 4, py - 4, px + 4, py + 4], fill=col)
    d.text((x1 - 60, y1 - 18), "%d m" % round(span), font=F["t"], fill=(120, 130, 150))


def compose_frame(args):
    run, k, out_path, meta = args
    recs, hist, fps = meta["recs"], meta["hist"], meta["fps"]
    rec = recs[k]
    try:
        base = Image.open(os.path.join(run, "cine", "f%05d.png" % k)).convert("RGBA")
    except Exception:
        base = Image.new("RGBA", (W, H), (0, 0, 0, 255))
    if base.size != (W, H):
        base = base.resize((W, H), Image.LANCZOS)
    side_panel(base, run, hist, k, "go2", "left", rec)
    side_panel(base, run, hist, k, "g1", "right", rec)
    # header
    rounded_panel(base, (20, 20, 20 + 520, 20 + 62), alpha=130)
    d = ImageDraw.Draw(base)
    d.text((34, 28), "HERCULES  ·  Go2 + G1 walking together", font=F["T"], fill=(245, 245, 245))
    d.text((34, 56), "MuJoCo physics + RL locomotion policies  ·  Unreal Engine rendering  ·  AirSim sensors", font=F["cap"], fill=(200, 205, 215))
    rounded_panel(base, (W - 20 - 300, 20, W - 20, 20 + 62), alpha=130)
    d.text((W - 20 - 288, 28), "sim time  %6.2f s" % rec["t_sim"], font=F["h"], fill=(245, 245, 245))
    d.text((W - 20 - 288, 50), "lockstep %d fps · frame %05d · %s" % (fps, k, rec["shot"]), font=F["t"], fill=(200, 205, 215))
    minimap(base, recs, k, (W // 2 - 150, H - 20 - 170, W // 2 + 150, H - 20))
    base.convert("RGB").save(out_path, compress_level=1)
    return k


def title_card(text, sub, path, fade=1.0):
    im = Image.new("RGB", (W, H), (6, 8, 12))
    d = ImageDraw.Draw(im)
    tw = d.textlength(text, font=F["big"])
    d.text(((W - tw) / 2, H / 2 - 40), text, font=F["big"], fill=(245, 245, 245))
    sw = d.textlength(sub, font=F["sub"])
    d.text(((W - sw) / 2, H / 2 + 12), sub, font=F["sub"], fill=(170, 180, 200))
    im.save(path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("out")
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--frame", type=int, default=-1, help="compose a single frame to OUT (png) and exit")
    ap.add_argument("--no-cards", action="store_true")
    a = ap.parse_args()
    recs = [json.loads(l) for l in open(os.path.join(a.run, "telemetry.jsonl")) if l.strip()]
    runmeta = json.load(open(os.path.join(a.run, "run.json")))
    fps = runmeta["fps"]
    hist = {"fps": fps}
    for r in ("go2", "g1"):
        hist[r] = {"fps": fps, "gyro": np.array([rec["robots"][r]["imu_gyro"] for rec in recs]),
                   "acc": np.array([rec["robots"][r]["imu_acc"] for rec in recs])}
    meta = {"recs": recs, "hist": hist, "fps": fps}
    if a.frame >= 0:
        compose_frame((a.run, a.frame, a.out, meta))
        print("wrote", a.out)
        return
    n = len(recs) if not a.limit else min(a.limit, len(recs))
    comp = os.path.join(a.run, "composed")
    os.makedirs(comp, exist_ok=True)
    jobs = [(a.run, k, os.path.join(comp, "c%05d.png" % k), meta) for k in range(n)
            if not os.path.exists(os.path.join(comp, "c%05d.png" % k))]
    with Pool(a.workers) as p:
        for i, k in enumerate(p.imap_unordered(compose_frame, jobs, chunksize=4)):
            if i % 100 == 0:
                print("composed %d/%d" % (i, len(jobs)), flush=True)
    # encode: title card (3 s) + body + end card (3 s), fades
    cards = os.path.join(a.run, "cards")
    os.makedirs(cards, exist_ok=True)
    title_card("Go2 + G1: walking together", "HERCULES · MuJoCo + RL policies · Unreal Engine · AirSim onboard sensors", os.path.join(cards, "title.png"))
    title_card("HERCULES", "Georgia Tech · lockstep capture, 1 world tick per frame, no dropped frames", os.path.join(cards, "end.png"))
    body = os.path.join(a.run, "body.mp4")
    subprocess.check_call(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", os.path.join(comp, "c%05d.png"), "-frames:v", str(n),
                           "-c:v", "libx264", "-preset", "slow", "-crf", "17", "-pix_fmt", "yuv420p", "-movflags", "+faststart", body])
    if a.no_cards:
        os.replace(body, a.out)
        print("wrote", a.out)
        return
    dur = n / fps
    cmd = ["ffmpeg", "-y", "-loglevel", "error",
           "-loop", "1", "-t", "3", "-framerate", str(fps), "-i", os.path.join(cards, "title.png"),
           "-i", body,
           "-loop", "1", "-t", "3", "-framerate", str(fps), "-i", os.path.join(cards, "end.png"),
           "-filter_complex",
           "[0:v]fade=t=in:st=0:d=0.8,fade=t=out:st=2.2:d=0.8,format=yuv420p[a];"
           "[1:v]fade=t=in:st=0:d=0.8,fade=t=out:st=%.2f:d=0.8,format=yuv420p[b];"
           "[2:v]fade=t=in:st=0:d=0.8,fade=t=out:st=2.2:d=0.8,format=yuv420p[c];"
           "[a][b][c]concat=n=3:v=1:a=0[v]" % (dur - 0.8),
           "-map", "[v]", "-c:v", "libx264", "-preset", "slow", "-crf", "17", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out]
    subprocess.check_call(cmd)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
