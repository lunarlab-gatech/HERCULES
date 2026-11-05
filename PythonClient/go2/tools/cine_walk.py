#!/usr/bin/env python3
"""Frame-exact "walking together" capture for the Go2 dog + G1 humanoid (Unreal lockstep, 1 world tick per frame).

Pipeline per frame k:
  camera pose for the shot  ->  rig.step(shot)  [applies the pose, runs exactly one world tick at 1/fps, pauses,
  HighResShot]  ->  while the PNG is written, read both robots' onboard sensors through AirSim (RGB / depth /
  segmentation / lidar / IMU / GPS / ground-truth kinematics)  ->  formation controller -> velocity commands to
  both MuJoCo bridges (--pacing frames: exactly 1/fps of MuJoCo time per world tick).
The two bridges, the AirSim client and the editor rig (ue_cine_rig.py, running inside the editor) are all driven
from this one process. Outputs: OUT/cine/f%05d.png, OUT/sens/<robot>_{rgb,depth,seg}_%05d.{jpg,png}, lidar .npy,
OUT/telemetry.jsonl (one line per frame), OUT/run.json.

  python3 cine_walk.py OUT --rig RIGDIR --path "X,Y X,Y ..." [--lane 1.7] [--speed 0.4] [--fps 25] [--shots shots.json]
Path points are Unreal world cm; the robots are aligned to the path during an unrecorded pre-roll.
"""
import argparse
import json
import math
import os
import queue
import shutil
import sys
import threading
import time

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PC = os.environ.get("HERCULES_PYTHONCLIENT", os.path.join(os.path.expanduser("~"), "multi-robot-coordination", "Cosys-AirSim", "PythonClient"))
for p in (HERE, os.path.join(HERE, ".."), os.path.join(HERE, "..", ".."), PC, os.path.join(PC, "go2")):
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
from rigclient import Rig  # noqa: E402


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ease(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


# ----------------------------------------------------------------------------- planar frame helpers
# Planar frame: metres, x = X_ue/100, y = -Y_ue/100 (right-handed, z up), yaw CCW = -yaw_ue.
def ue_to_plane(X, Y):
    return X / 100.0, -Y / 100.0


def plane_to_ue(x, y):
    return x * 100.0, -y * 100.0


class Path:
    """Polyline with closest-point projection, arc length and lane offsets."""

    def __init__(self, pts):
        self.p = np.array(pts, float)
        d = np.diff(self.p, axis=0)
        self.seg_len = np.hypot(d[:, 0], d[:, 1])
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.tan = d / self.seg_len[:, None]
        self.length = float(self.cum[-1])

    def project(self, x, y):
        """-> (s, e, heading): arc length of the closest point, signed cross-track (left +), tangent heading."""
        best = None
        for i in range(len(self.seg_len)):
            a, t = self.p[i], self.tan[i]
            u = (x - a[0]) * t[0] + (y - a[1]) * t[1]
            u = min(max(u, 0.0), self.seg_len[i])
            cx, cy = a[0] + t[0] * u, a[1] + t[1] * u
            dist = math.hypot(x - cx, y - cy)
            if best is None or dist < best[0]:
                e = -(x - cx) * t[1] + (y - cy) * t[0]
                best = (dist, self.cum[i] + u, e, math.atan2(t[1], t[0]))
        return best[1], best[2], best[3]

    def point(self, s, lane=0.0):
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.cum, s, side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        u = s - self.cum[i]
        a, t = self.p[i], self.tan[i]
        return a[0] + t[0] * u - t[1] * lane, a[1] + t[1] * u + t[0] * lane

    def heading(self, s):
        s = min(max(s, 0.0), self.length)
        i = int(np.searchsorted(self.cum, s, side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        return math.atan2(self.tan[i][1], self.tan[i][0])


# ----------------------------------------------------------------------------- robots
class Walker:
    """One robot: MuJoCo bridge (frames pacing) + planar state from the editor rig + speed/lane controller."""
    GAIN0 = {"go2": 0.78, "g1": 0.94}       # m/s of body speed per unit vx command (measured 2026-11-02)

    def __init__(self, robot, vehicle, lane, fps):
        from go2_commander import RobotCommander
        self.robot, self.vehicle, self.lane, self.fps = robot, vehicle, lane, fps
        self.cmd = RobotCommander(robot=robot, pacing="frames", terrain=False, verbose=False)
        self.sim = self.cmd.sim
        self.gain = self.GAIN0[robot]
        self.x = self.y = self.yaw = 0.0
        self.hist = []                       # (x, y) per frame for speed estimation
        self.speed = 0.0
        self.vcmd = (0.0, 0.0, 0.0)
        self.s = self.e = 0.0

    def update_state(self, st):
        X, Y, Z, yaw_ue, stand = st["robots"][self.vehicle]
        self.x, self.y = ue_to_plane(X, Y)
        self.yaw = -math.radians(yaw_ue)
        self.z_ground_ue = Z - stand
        self.hist.append((self.x, self.y))
        if len(self.hist) > int(self.fps) + 1:
            self.hist.pop(0)
        if len(self.hist) >= 2:
            (x0, y0), (x1, y1) = self.hist[0], self.hist[-1]
            self.speed = math.hypot(x1 - x0, y1 - y0) / ((len(self.hist) - 1) / self.fps)

    def set_velocity(self, vx, vy, vyaw):
        self.vcmd = (vx, vy, vyaw)
        self.sim.set_velocity(vx, vy, vyaw)

    def close(self):
        try:
            self.set_velocity(0, 0, 0)
            self.cmd.close()
        except Exception:
            pass


class Formation:
    """Keeps both robots abreast on their lanes along the path (pure pursuit heading + sidestep lane trim +
    along-track speed sync)."""
    KP_YAW, MAX_YAW = 1.2, 0.5
    KP_LAT, MAX_LAT = 0.25, 0.12
    KV_SYNC = 0.35
    LOOK = 1.6

    def __init__(self, path, walkers, speed):
        self.path, self.w, self.v_nom = path, walkers, speed
        self.s_mean = 0.0
        self.stopped = False

    def update(self, walking=True):
        for w in self.w:
            w.s, w.e, _ = self.path.project(w.x, w.y)
        self.s_mean = sum(w.s for w in self.w) / len(self.w)
        out = {}
        for w in self.w:
            tx, ty = self.path.point(w.s + self.LOOK, w.lane)
            h_des = math.atan2(ty - w.y, tx - w.x)
            err = wrap(h_des - w.yaw)
            vyaw = max(-self.MAX_YAW, min(self.MAX_YAW, self.KP_YAW * err))
            lat = w.lane - w.e                    # + = robot must move left
            vy = max(-self.MAX_LAT, min(self.MAX_LAT, self.KP_LAT * lat))
            if walking and not self.stopped:
                v = self.v_nom + self.KV_SYNC * (self.s_mean - w.s)
                v = max(0.55 * self.v_nom, min(1.45 * self.v_nom, v))
                vx = v / w.gain
                if abs(err) > math.radians(35):   # badly misaligned: turn first
                    vx *= 0.3
            else:
                vx = 0.0
            w.set_velocity(vx, vy, vyaw)
            out[w.robot] = (round(vx, 3), round(vy, 3), round(vyaw, 3), round(err, 3), round(lat, 3))
        return out

    def needs_swap(self):
        return all((w.e - w.lane) * w.lane < -0.5 * abs(w.lane) for w in self.w)

    def swap_step(self):
        """One control step of the swap manoeuvre; returns True while still swapping."""
        for w in self.w:
            w.s, w.e, _ = self.path.project(w.x, w.y)
        go2 = next(w for w in self.w if w.robot == "go2")
        g1 = next(w for w in self.w if w.robot == "g1")
        h = self.path.heading((go2.s + g1.s) / 2)
        for w in self.w:                       # both hold the path heading
            w._err = wrap(h - w.yaw)
        go2.set_velocity(0.0, 0.0, max(-self.MAX_YAW, min(self.MAX_YAW, self.KP_YAW * go2._err)))
        if g1.s - go2.s < 2.5:                 # G1 walks ahead first
            g1.set_velocity(0.45, 0.0, max(-self.MAX_YAW, min(self.MAX_YAW, self.KP_YAW * g1._err)))
            return True
        lat = g1.lane - g1.e
        if abs(lat) > 0.3:                     # then sidesteps across, well ahead of the Go2
            g1.set_velocity(0.0, 0.3 if lat > 0 else -0.3, max(-self.MAX_YAW, min(self.MAX_YAW, self.KP_YAW * g1._err)))
            return True
        g1.set_velocity(0.0, 0.0, 0.0)
        return False

    def aligned(self):
        ds = abs(self.w[0].s - self.w[1].s)
        ok = ds < 0.35
        for w in self.w:
            _, _, h = self.path.project(w.x, w.y)
            ok = ok and abs(w.lane - w.e) < 0.3 and abs(wrap(h - w.yaw)) < math.radians(12)
        return ok

    def stop(self):
        self.stopped = True
        for w in self.w:
            w.set_velocity(0.0, 0.0, 0.0)


# ----------------------------------------------------------------------------- camera
class Camera:
    """Shot timeline evaluated per frame in the formation frame (az 0 = ahead of the robots, + = their left)."""

    def __init__(self, shots, fps):
        self.shots, self.fps = shots, fps
        self.starts = np.concatenate([[0.0], np.cumsum([s["dur"] for s in shots])])
        self.total_frames = int(round(self.starts[-1] * fps))
        self.pos = None
        self.aim = None
        self.cur = -1
        self.locked = None

    @staticmethod
    def lerp(a, b, u):
        return a + (b - a) * u

    def shot_at(self, k):
        t = k / self.fps
        i = int(np.searchsorted(self.starts, t, side="right") - 1)
        i = min(max(i, 0), len(self.shots) - 1)
        u = (t - self.starts[i]) / max(1e-6, self.shots[i]["dur"])
        return i, min(1.0, u)

    # robot bounding boxes for the framing guard: (half x, half y, z0, z1) in metres above ground
    BOXES = {"go2": (0.5, 0.4, 0.0, 0.5), "g1": (0.5, 0.6, 0.0, 1.5)}     # G1 walks with the arms spread
    MARGIN = 0.08

    @staticmethod
    def fits(p, aim, fov, boxes, margin):
        """True when all box corners project inside the frame (16:9) with the given margin."""
        f = np.array(aim) - np.array(p)
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        th = math.tan(math.radians(fov) / 2)
        tv = th * 9 / 16
        for (x, y, hx, hy, z0, z1) in boxes:
            for dx in (-hx, hx):
                for dy in (-hy, hy):
                    for z in (z0, z1):
                        v = np.array([x + dx, y + dy, z]) - p
                        d = v @ f
                        if d < 0.3:
                            return False
                        if abs(v @ r) / (d * th) > 1 - margin or abs(v @ u) / (d * tv) > 1 - margin:
                            return False
        return True

    def pose(self, k, cx, cy, heading, gz_ue, robots=None):
        """-> (cam [X,Y,Z,pitch,yaw,roll] in UE, fov, shot name). robots: {name: (x, y)} planar, for the guard."""
        i, u = self.shot_at(k)
        s = self.shots[i]
        ue = ease(u)
        g = lambda key: self.lerp(s[key][0], s[key][1], ue) if isinstance(s[key], (list, tuple)) else s[key]
        az, dist, h, aim_h, fov = (math.radians(g("az")), g("dist"), g("h"), g("aim"), g("fov"))
        gz = gz_ue / 100.0
        ax, ay, az_ = cx, cy, gz + aim_h

        def place(dist):
            fx, fy = dist * math.cos(az), dist * math.sin(az)
            return (cx + fx * math.cos(heading) - fy * math.sin(heading),
                    cy + fx * math.sin(heading) + fy * math.cos(heading), gz + h)
        px, py, pz = place(dist)
        if robots and not s.get("lock"):
            boxes = [(x, y, bx, by, gz + z0, gz + z1) for n, (x, y) in robots.items() for (bx, by, z0, z1) in [self.BOXES[n]]]
            dd = dist
            while dd < dist * 3 and not self.fits(np.array([px, py, pz]), (ax, ay, az_), fov, boxes, self.MARGIN):
                dd += 0.1
                px, py, pz = place(dd)
            self.last_fit = dd
        tau_p, tau_a = s.get("tau", 0.5), s.get("tau_aim", 0.6)
        if i != self.cur:                     # cut: no smoothing lag at the start of a shot
            self.cur, self.pos, self.aim, self.locked = i, [px, py, pz], [ax, ay, az_], None
            if s.get("lock"):
                self.locked = [px, py, pz]
        if self.locked is not None:
            self.pos = list(self.locked)
        else:
            a = 1 - math.exp(-1.0 / (self.fps * tau_p))
            self.pos = [self.pos[j] + (v - self.pos[j]) * a for j, v in enumerate((px, py, pz))]
        a = 1 - math.exp(-1.0 / (self.fps * tau_a))
        self.aim = [self.aim[j] + (v - self.aim[j]) * a for j, v in enumerate((ax, ay, az_))]
        dx, dy, dz = self.aim[0] - self.pos[0], self.aim[1] - self.pos[1], self.aim[2] - self.pos[2]
        yaw_ue = -math.degrees(math.atan2(dy, dx))
        pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        X, Y = plane_to_ue(self.pos[0], self.pos[1])
        return [X, Y, self.pos[2] * 100.0, pitch, yaw_ue, 0.0], fov, s["name"]


DEFAULT_SHOTS = [
    {"name": "approach", "dur": 8, "az": 28, "dist": 9.0, "h": 1.0, "aim": 0.55, "fov": 36, "lock": True},
    {"name": "front_dolly", "dur": 12, "az": [0, 18], "dist": 4.6, "h": 0.8, "aim": 0.6, "fov": 50},
    {"name": "side_track", "dur": 8, "az": 90, "dist": 5.0, "h": 0.9, "aim": 0.6, "fov": 45},
    {"name": "rear_follow", "dur": 8, "az": [150, 178], "dist": 5.0, "h": 1.6, "aim": 0.6, "fov": 48},
    {"name": "orbit", "dur": 12, "az": [178, 20], "dist": 5.0, "h": 1.2, "aim": 0.6, "fov": 48},   # via the sun-lit left side (Go2 nearer: no occlusion)
    {"name": "low_front", "dur": 6, "az": -20, "dist": 4.5, "h": 0.3, "aim": 0.55, "fov": 55},
    {"name": "crane_out", "dur": 9, "az": [30, 40], "dist": [4.8, 9.5], "h": [1.0, 4.0], "aim": 0.5, "fov": 48, "stop": 3.0, "sit": 1.5},
    {"name": "ending", "dur": 6, "az": [40, 70], "dist": 4.8, "h": 1.0, "aim": 0.6, "fov": 45},
]


# ----------------------------------------------------------------------------- sensors
class Sensors:
    def __init__(self, port=41453):
        import hercules_cosysairsim as airsim
        self.airsim = airsim
        self.c = airsim.VehicleClient(port=port)
        self.c.confirmConnection()
        IT = airsim.ImageType
        self.req_rgb = [airsim.ImageRequest("front_center", IT.Scene, False, False),
                        airsim.ImageRequest("front_center", IT.Segmentation, False, False)]
        self.req_depth = [airsim.ImageRequest("front_center", IT.DepthPlanar, True, False)]

    def grab(self, vehicle):
        c = self.c
        rs = c.simGetImages(self.req_rgb, vehicle_name=vehicle)
        rd = c.simGetImages(self.req_depth, vehicle_name=vehicle)
        rgb = np.frombuffer(rs[0].image_data_uint8, np.uint8).reshape(rs[0].height, rs[0].width, 3).copy()
        seg = np.frombuffer(rs[1].image_data_uint8, np.uint8).reshape(rs[1].height, rs[1].width, 3).copy()
        depth = np.array(rd[0].image_data_float, np.float32).reshape(rd[0].height, rd[0].width)
        ld = c.getLidarData(lidar_name="LidarSensor1", vehicle_name=vehicle)
        pts = np.array(ld.point_cloud, np.float32).reshape(-1, 3) if len(ld.point_cloud) >= 3 else np.zeros((0, 3), np.float32)
        imu = c.getImuData(imu_name="imu", vehicle_name=vehicle)
        gps = c.getGpsData(gps_name="gps", vehicle_name=vehicle)
        kin = c.simGetGroundTruthKinematics(vehicle_name=vehicle)
        po = c.simGetObjectPose(vehicle)
        q = po.orientation
        yaw = math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val), 1 - 2 * (q.y_val ** 2 + q.z_val ** 2))
        tele = {
            "pos": [po.position.x_val, po.position.y_val, po.position.z_val], "yaw_ned": yaw,
            "imu_gyro": [imu.angular_velocity.x_val, imu.angular_velocity.y_val, imu.angular_velocity.z_val],
            "imu_acc": [imu.linear_acceleration.x_val, imu.linear_acceleration.y_val, imu.linear_acceleration.z_val],
            "imu_ts": int(imu.time_stamp),
            "gps": [gps.gnss.geo_point.latitude, gps.gnss.geo_point.longitude, gps.gnss.geo_point.altitude],
            "vel": [kin.linear_velocity.x_val, kin.linear_velocity.y_val, kin.linear_velocity.z_val],
            "lidar_n": int(len(pts)), "lidar_ts": int(ld.time_stamp),
        }
        return rgb, seg, depth, pts, tele


class SensorWorker(threading.Thread):
    """Owns the AirSim client (msgpack-rpc needs an asyncio loop in the calling thread) and grabs all robots."""

    def __init__(self, vehicles):
        super().__init__(daemon=True)
        self.vehicles = vehicles
        self.jobs, self.results = queue.Queue(), queue.Queue()
        self.ready = threading.Event()
        self.start()
        self.ready.wait()

    def run(self):
        import asyncio
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.sens = Sensors()
        self.ready.set()
        while True:
            job = self.jobs.get()
            if job is None:
                return
            out = {}
            try:
                for robot, vehicle in self.vehicles:
                    out[robot] = self.sens.grab(vehicle)
            except Exception as e:
                out = e
            self.results.put(out)

    def grab_async(self):
        self.jobs.put(1)

    def wait(self):
        r = self.results.get()
        if isinstance(r, Exception):
            raise r
        return r


class Saver(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue(maxsize=64)
        self.start()

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            kind, path, data = item
            try:
                if kind == "jpg":
                    Image.fromarray(data).save(path, quality=92)
                elif kind == "png":
                    Image.fromarray(data).save(path, compress_level=1)
                elif kind == "depth":
                    Image.fromarray(np.clip(data * 1000.0, 0, 65535).astype(np.uint16)).save(path, compress_level=1)
                elif kind == "npy":
                    np.save(path, data.astype(np.float16))
                elif kind == "move":
                    shutil.move(data, path)
            except Exception as e:
                print("saver error", path, e, flush=True)
            self.q.task_done()

    def put(self, kind, path, data):
        self.q.put((kind, path, data))

    def finish(self):
        self.q.put(None)
        self.join()


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--rig", required=True)
    ap.add_argument("--path", required=True, help="'X,Y X,Y ...' Unreal cm")
    ap.add_argument("--lane", type=float, default=1.7, help="lateral spacing between the robots (m)")
    ap.add_argument("--left", default="go2", choices=("go2", "g1"), help="which robot walks on the left lane")
    ap.add_argument("--speed", type=float, default=0.4)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--shots", default="")
    ap.add_argument("--preroll-max", type=float, default=25.0, help="max unrecorded pre-roll (sim s)")
    ap.add_argument("--settle", type=float, default=3.0, help="stand still before walking (sim s)")
    ap.add_argument("--res", default="1920x1080")
    ap.add_argument("--no-record", action="store_true", help="drive only (no shots, no sensors)")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after this many recorded frames (test runs)")
    ap.add_argument("--preview", type=int, default=0, help="framing preview: robots stand, HighResShot every Nth frame, no sensors")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    for d in ("cine", "sens"):
        os.makedirs(os.path.join(a.out, d), exist_ok=True)
    log = open(os.path.join(a.out, "cine_walk.log"), "a")

    def say(msg):
        line = time.strftime("%H:%M:%S ") + msg
        print(line, flush=True)
        log.write(line + "\n")
        log.flush()

    shots = json.load(open(a.shots)) if a.shots else DEFAULT_SHOTS
    pts = [ue_to_plane(*map(float, p.split(","))) for p in a.path.split()]
    path = Path(pts)
    os.environ["GO2_FRAME_DT"] = str(1.0 / a.fps)
    rig = Rig(a.rig)
    rig.req(hold=True, fps=a.fps, res=a.res, view="cam")
    st = rig.status()
    say("rig ok t=%.3f paused=%s; path %.1f m, %d shots, %.1f s" % (st["t"], st["paused"], path.length, len(shots),
                                                                   sum(s["dur"] for s in shots)))
    half = a.lane / 2.0
    lanes = {"go2": half, "g1": -half} if a.left == "go2" else {"go2": -half, "g1": half}
    walkers = [Walker("go2", "Go2", lanes["go2"], a.fps), Walker("g1", "G1", lanes["g1"], a.fps)]
    say("bridges connected (frames pacing, dt=%.4f)" % (1.0 / a.fps))
    form = Formation(path, walkers, a.speed)
    cam = Camera(shots, a.fps)
    if a.preview:
        a.no_record = True
    sensors = None if a.no_record else SensorWorker([(w.robot, w.vehicle) for w in walkers])
    saver = Saver()
    tele_f = open(os.path.join(a.out, "telemetry.jsonl"), "a")
    json.dump({"path_ue": a.path, "lane": a.lane, "left": a.left, "speed": a.speed, "fps": a.fps, "shots": shots,
               "res": a.res, "started": time.strftime("%Y-%m-%d %H:%M:%S")}, open(os.path.join(a.out, "run.json"), "w"), indent=1)

    def refresh():
        s = rig.status()
        for w in walkers:
            w.update_state(s)
        return s

    frame = 0            # recorded frame index
    tick = 0             # world ticks since start
    phase = "settle"
    t_phase0 = 0.0
    swap_checked = False
    sit_at = None
    try:
        s = refresh()
        # ---------------- pre-roll: settle, then walk the formation until aligned (unrecorded) ----------------
        while True:
            t_sim = tick / a.fps
            if phase == "settle":
                if t_sim >= a.settle:
                    phase, t_phase0 = "turn", t_sim
                    say("pre-roll: turning to the path heading")
            elif phase == "turn":
                # face the first segment before walking: Go2 = kinematic pivot (slow policy yaw), G1 = policy yaw rate
                h0 = path.heading(0.0)
                done = True
                for w in walkers:
                    err = wrap(h0 - w.yaw)
                    if abs(err) > math.radians(5):
                        done = False
                        if w.robot == "go2":
                            w.sim.set_pivot(0.8 if err > 0 else -0.8)
                        else:
                            w.set_velocity(0.0, 0.0, max(-0.8, min(0.8, 1.5 * err)))
                    else:
                        if w.robot == "go2":
                            w.sim.set_pivot(0.0)
                        w.set_velocity(0.0, 0.0, 0.0)
                if tick % a.fps == 0:
                    say("turn t=%.1f yaw_err=%s" % (t_sim, [round(math.degrees(wrap(h0 - w.yaw)), 1) for w in walkers]))
                if done or t_sim - t_phase0 > 15.0:
                    for w in walkers:
                        if w.robot == "go2":
                            w.sim.set_pivot(0.0)
                        w.set_velocity(0.0, 0.0, 0.0)
                    phase, t_phase0 = "align", t_sim
                    say("pre-roll: walking to align (turn done=%s)" % done)
            elif phase == "swap":
                if not form.swap_step():
                    phase, t_phase0 = "align", t_sim
                    say("swap done at t=%.1f: e=%.2f/%.2f" % (t_sim, walkers[0].e, walkers[1].e))
                elif tick % a.fps == 0:
                    say("swap t=%.1f s=%.2f/%.2f e=%.2f/%.2f" % (t_sim, walkers[0].s, walkers[1].s, walkers[0].e, walkers[1].e))
                if t_sim - t_phase0 > 40.0:
                    say("swap timed out")
                    phase, t_phase0 = "align", t_sim
            elif phase == "align":
                if a.preview:
                    say("preview: robots stand still, recording camera path only")
                    break
                if not swap_checked:
                    swap_checked = True
                    form.update(walking=False)
                    if form.needs_swap():
                        phase = "swap"
                        say("robots are on the wrong lanes (e=%.2f/%.2f): swapping" % (walkers[0].e, walkers[1].e))
                        rig.step(); tick += 1; s = refresh(); continue
                cmds = form.update(walking=True)
                if tick % a.fps == 0:
                    say("align t=%.1f s_go2=%.2f s_g1=%.2f e=%.2f/%.2f spd=%.2f/%.2f cmds=%s" % (
                        t_sim, walkers[0].s, walkers[1].s, walkers[0].e, walkers[1].e, walkers[0].speed, walkers[1].speed, cmds))
                if (form.aligned() and t_sim - t_phase0 > 4.0) or t_sim - t_phase0 > a.preroll_max:
                    say("pre-roll done at t=%.1f (aligned=%s): recording starts" % (t_sim, form.aligned()))
                    break
            rig.step()
            tick += 1
            s = refresh()
        # ---------------- recorded frames ----------------
        t_rec0 = tick / a.fps
        heading_f = path.heading(form.s_mean)
        n_frames = cam.total_frames if not a.max_frames else min(a.max_frames, cam.total_frames)
        while frame < n_frames:
            # formation centre + heading for the camera
            cx, cy = (walkers[0].x + walkers[1].x) / 2, (walkers[0].y + walkers[1].y) / 2
            heading_f += wrap(path.heading(form.s_mean) - heading_f) * (1 - math.exp(-1.0 / (a.fps * 1.5)))
            gz = min(w.z_ground_ue for w in walkers)
            cam_pose, fov, shot_name = cam.pose(frame, cx, cy, heading_f, gz, {w.robot: (w.x, w.y) for w in walkers})
            name = "cw_%05d" % frame
            t_before = s["t"]
            # step + shot; grab sensors as soon as the world has ticked and frozen again
            t0 = time.time()
            if a.preview:
                shot = name if frame % a.preview == 0 else None
                if shot and os.path.exists(rig.shot_path(shot)):
                    os.remove(rig.shot_path(shot))
                rig.step(cam=cam_pose, fov=fov, shot=shot)
                if shot:
                    saver.put("move", os.path.join(a.out, "cine", "f%05d.png" % frame), rig.shot_path(shot))
            elif a.no_record:
                rig.step(cam=cam_pose, fov=fov)
            else:
                p = rig.shot_path(name)
                if os.path.exists(p):
                    os.remove(p)
                seq = rig.send(step=True, shot=name, cam=cam_pose, fov=fov)
                while True:                       # the world has ticked once and is frozen again
                    s2 = rig.status()
                    if s2["t"] > t_before and s2["paused"]:
                        break
                    if time.time() - t0 > 20:
                        raise TimeoutError("step %s" % name)
                    time.sleep(0.002)
                sensors.grab_async()              # onboard sensors while the PNG is being written
                ack = rig.wait_ack(seq, timeout=20)
                grabbed = sensors.wait()
                if ack.get("ticks") != 1:
                    say("WARNING frame %d: %s world ticks" % (frame, ack.get("ticks")))
                while not (os.path.exists(p) and os.path.getsize(p) > 1000):
                    time.sleep(0.005)
                    if time.time() - t0 > 20:
                        raise TimeoutError("shot %s" % name)
            tick += 1
            s = refresh()
            # controller for the next tick
            t_sim = tick / a.fps
            si, su = cam.shot_at(frame)
            if a.preview:
                cmds = None
            elif form.stopped:
                cmds = form.update(walking=False)
            else:
                cmds = form.update(walking=True)
                stop_shot = shots[si].get("stop")
                if form.s_mean >= path.length - 0.25 or (stop_shot is not None and su * shots[si]["dur"] >= stop_shot):
                    form.stop()
                    sit_at = tick + int(shots[si].get("sit", 1.0) * a.fps)
                    say("frame %d: stop (%s)" % (frame, "path end" if form.s_mean >= path.length - 0.25 else shots[si]["name"]))
            if sit_at is not None and tick >= sit_at and not a.preview:
                sit_at = None
                walkers[0].sim.sit(1.5)
                say("frame %d: Go2 sits" % frame)
            if not a.no_record:
                saver.put("move", os.path.join(a.out, "cine", "f%05d.png" % frame), p)
                rec = {"frame": frame, "t_sim": round((tick - 1) / a.fps - t_rec0, 4), "t_world": s["t"], "shot": shot_name,
                       "cam": [round(v, 2) for v in cam_pose], "fov": fov, "robots": {}}
                for w in walkers:
                    rgb, seg, depth, pts, tele = grabbed[w.robot]
                    base = os.path.join(a.out, "sens", "%s_%%s_%05d" % (w.robot, frame))
                    saver.put("jpg", base % "rgb" + ".jpg", rgb)
                    saver.put("png", base % "seg" + ".png", seg)
                    saver.put("depth", base % "depth" + ".png", depth)
                    saver.put("npy", base % "lidar" + ".npy", pts)
                    tele.update({"ue": s["robots"][w.vehicle][:4], "cmd": list(w.vcmd), "speed": round(w.speed, 3),
                                 "s": round(w.s, 3), "e": round(w.e, 3), "gain": round(w.gain, 3)})
                    rec["robots"][w.robot] = tele
                tele_f.write(json.dumps(rec) + "\n")
            # online gain adaptation (slow): measured body speed vs commanded vx
            for w in walkers:
                if not form.stopped and w.vcmd[0] > 0.25 and len(w.hist) > a.fps and w.speed > 0.05:
                    w.gain += 0.01 * (w.speed / w.vcmd[0] - w.gain)
            if frame % a.fps == 0:
                say("frame %d/%d %s t=%.1f ds=%.2f e=%.2f/%.2f spd=%.2f/%.2f gain=%.2f/%.2f %.2fs/frame" % (
                    frame, n_frames, shot_name, t_sim, walkers[0].s - walkers[1].s, walkers[0].e, walkers[1].e,
                    walkers[0].speed, walkers[1].speed, walkers[0].gain, walkers[1].gain, time.time() - t0))
            frame += 1
        say("capture complete: %d frames" % frame)
    finally:
        for w in walkers:
            w.close()
        saver.finish()
        tele_f.close()
        try:
            rig.req(hold=False, view="ext")
        except Exception as e:
            say("rig release failed: %s" % e)
        say("done")


if __name__ == "__main__":
    main()
