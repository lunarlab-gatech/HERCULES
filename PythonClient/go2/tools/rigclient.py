"""Client side of ue_cine_rig.py: request/ack/status files in a shared directory."""
import json
import os
import time


class Rig:
    def __init__(self, d, screenshots="/home/sgarimella34/Documents/Unreal Projects/AustraliaLandscape/Saved/Screenshots/LinuxEditor"):
        self.d = d
        self.req_path, self.ack_path, self.status_path = (os.path.join(d, n) for n in ("req.json", "ack.json", "status.json"))
        self.screenshots = screenshots
        self.seq = int(time.time()) % 100000 * 10

    def status(self):
        for _ in range(200):
            try:
                with open(self.status_path) as f:
                    return json.load(f)
            except Exception:
                time.sleep(0.01)
        raise RuntimeError("rig status unreadable")

    def send(self, **kw):
        """Write a request and return its seq without waiting."""
        self.seq += 1
        kw["seq"] = self.seq
        tmp = self.req_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(kw, f)
        os.replace(tmp, self.req_path)
        return self.seq

    def wait_ack(self, seq, timeout=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                with open(self.ack_path) as f:
                    a = json.load(f)
                if a.get("seq") == seq:
                    return a
            except Exception:
                pass
            time.sleep(0.005)
        raise TimeoutError("rig: no ack for seq %d" % seq)

    def req(self, timeout=10.0, **kw):
        return self.wait_ack(self.send(**kw), timeout)

    def shot_path(self, name):
        return os.path.join(self.screenshots, name + ".png")

    def step(self, shot=None, cam=None, fov=None, timeout=15.0):
        """Apply cam/fov, run exactly one world tick, optionally HighResShot it. Returns the ack (t, ticks)."""
        if shot and os.path.exists(self.shot_path(shot)):
            os.remove(self.shot_path(shot))
        a = self.req(timeout=timeout, step=True, shot=shot, cam=cam, fov=fov)
        if shot:
            p = self.shot_path(shot)
            t0 = time.time()
            while time.time() - t0 < timeout:
                if os.path.exists(p) and os.path.getsize(p) > 1000:
                    return a
                time.sleep(0.01)
            raise TimeoutError("rig: shot %s not written" % shot)
        return a
