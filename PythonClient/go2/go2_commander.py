#!/usr/bin/env python3
"""High-level Go2 API for HERCULES: forward(m), turn_left(deg), stand(), sit(), get_camera(), get_lidar().

Locomotion goes through go2_mujoco_bridge (MuJoCo + policy -> Unreal pawn); sensors come from AirSim,
because the dog is an AirSim ComputerVision vehicle (default name "Go2") with cameras/lidar/IMU in settings.json.

    from go2_commander import Go2Commander
    with Go2Commander(port=12345) as dog:
        dog.forward(2.0)
        dog.turn_left(90)
        rgb = dog.get_camera()          # HxWx3 uint8 from the AirSim camera "front_center"
"""
import math
import time

import numpy as np

from go2_mujoco_bridge import Go2Bridge


class Go2Commander:
    def __init__(self, port=12345, host="127.0.0.1", pacing="wall", terrain=True, bridge=None,
                 vehicle_name="Go2", airsim_port=41452, airsim_ip="127.0.0.1", verbose=True):
        self.bridge = bridge or Go2Bridge(host, port, pacing, terrain, verbose=verbose).start()
        self.sim = self.bridge.sim
        self.vehicle_name, self.airsim_port, self.airsim_ip = vehicle_name, airsim_port, airsim_ip
        self._client = None

    # ---- context ----
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        self.bridge.stop()

    # ---- time helpers (sim-time based, so they work in lockstep too) ----
    def wait(self, seconds):
        t_end = self.sim.time + seconds
        while self.bridge.running and self.sim.time < t_end:
            time.sleep(0.01)

    def _wait_until(self, pred, timeout_s):
        t_end = self.sim.time + timeout_s
        while self.bridge.running and self.sim.time < t_end:
            if pred():
                return True
            time.sleep(0.01)
        return False

    # ---- locomotion ----
    def set_velocity(self, vx, vy=0.0, vyaw=0.0):
        self.sim.set_velocity(vx, vy, vyaw)

    def stop(self):
        self.sim.set_velocity(0.0, 0.0, 0.0)

    def forward(self, meters, speed=0.5):
        """Walk forward (negative = backward) by odometry distance, then stop."""
        x0, y0 = self.sim.body_pose()[:2]
        v = abs(speed) * (1 if meters >= 0 else -1)
        self.sim.set_velocity(v, 0.0, 0.0)
        ok = self._wait_until(lambda: math.hypot(self.sim.body_pose()[0] - x0, self.sim.body_pose()[1] - y0) >= abs(meters),
                              timeout_s=abs(meters) / max(0.05, abs(speed)) * 3 + 3)
        self.stop()
        self.wait(0.5)
        return ok

    def turn(self, degrees, rate=0.8, mode="assist"):
        """Turn in place by integrated yaw (positive = left / counter-clockwise), then stop.
        mode="assist": walking policy + yaw torque assist (physical, slow: ~7 deg/s at rate 0.8);
        mode="pivot": kinematic pivot at `rate` rad/s (fast, feet slide)."""
        if mode == "pivot":
            return self._pivot(degrees, rate)
        yaw0 = self.sim.yaw()
        acc = [0.0]
        last = [yaw0]

        def progressed():
            y = self.sim.yaw()
            d = (y - last[0] + math.pi) % (2 * math.pi) - math.pi
            acc[0] += d
            last[0] = y
            return abs(acc[0]) >= math.radians(abs(degrees))
        self.sim.set_velocity(0.0, 0.0, abs(rate) * (1 if degrees >= 0 else -1))
        ok = self._wait_until(progressed, timeout_s=math.radians(abs(degrees)) / max(0.1, abs(rate)) * 3 + 3)
        self.stop()
        self.wait(0.5)
        return ok

    def _pivot(self, degrees, rate):
        acc = [0.0]; last = [self.sim.yaw()]

        def progressed():
            y = self.sim.yaw(); d = (y - last[0] + math.pi) % (2 * math.pi) - math.pi
            acc[0] += d; last[0] = y
            return abs(acc[0]) >= math.radians(abs(degrees))
        self.sim.set_pivot(abs(rate) * (1 if degrees >= 0 else -1))
        ok = self._wait_until(progressed, timeout_s=math.radians(abs(degrees)) / max(0.1, abs(rate)) + 3)
        self.sim.set_pivot(0.0)
        self.wait(0.5)
        self.sim.set_velocity(0.0, 0.0, 0.0)
        return ok

    def turn_left(self, degrees=90, rate=0.8, mode="assist"):
        return self.turn(abs(degrees), rate, mode)

    def turn_right(self, degrees=90, rate=0.8, mode="assist"):
        return self.turn(-abs(degrees), rate, mode)

    def stand(self):
        self.sim.stand(1.5)
        self.wait(1.7)
        self.sim.set_velocity(0.0, 0.0, 0.0)   # hand back to the policy (stands in place)

    def sit(self):
        self.sim.sit(1.5)
        self.wait(1.7)

    def pose(self):
        return self.sim.body_pose()

    def run_script(self, script):
        """'forward:2.0;turn:90;sit;wait:2;stand' style mini-language."""
        for item in [s.strip() for s in script.split(";") if s.strip()]:
            name, _, arg = item.partition(":")
            arg = float(arg) if arg else None
            if name == "forward":
                self.forward(arg if arg is not None else 1.0)
            elif name == "turn":
                self.turn(arg if arg is not None else 90.0)
            elif name == "pivot":
                self.turn(arg if arg is not None else 90.0, rate=1.0, mode="pivot")
            elif name == "sit":
                self.sit()
            elif name == "stand":
                self.stand()
            elif name == "wait":
                self.wait(arg if arg is not None else 1.0)
            elif name == "vel":
                self.set_velocity(arg or 0.0)
            else:
                raise ValueError("unknown script item: " + item)

    # ---- sensors via AirSim ----
    @property
    def client(self):
        if self._client is None:
            import hercules_cosysairsim as airsim
            self._client = airsim.CarClient(ip=self.airsim_ip, port=self.airsim_port)
            self._client.confirmConnection()
            self._airsim = airsim
        return self._client

    def get_camera(self, camera="front_center", image_type=None):
        """RGB image (HxWx3 uint8) from an AirSim camera on the dog; image_type = airsim.ImageType.*"""
        c = self.client
        it = self._airsim.ImageType.Scene if image_type is None else image_type
        req = self._airsim.ImageRequest(camera, it, False, False)
        r = c.simGetImages([req], vehicle_name=self.vehicle_name)[0]
        img = np.frombuffer(r.image_data_uint8, dtype=np.uint8).reshape(r.height, r.width, 3)
        return img

    def get_depth(self, camera="front_center"):
        c = self.client
        req = self._airsim.ImageRequest(camera, self._airsim.ImageType.DepthPlanar, True, False)
        r = c.simGetImages([req], vehicle_name=self.vehicle_name)[0]
        return np.array(r.image_data_float, dtype=np.float32).reshape(r.height, r.width)

    def get_lidar(self, lidar="LidarSensor1"):
        d = self.client.getLidarData(lidar_name=lidar, vehicle_name=self.vehicle_name)
        return np.array(d.point_cloud, dtype=np.float32).reshape(-1, 3) if len(d.point_cloud) >= 3 else np.zeros((0, 3), np.float32)

    def get_imu(self, imu="imu"):
        return self.client.getImuData(imu_name=imu, vehicle_name=self.vehicle_name)

    def get_airsim_pose(self):
        return self.client.simGetVehiclePose(vehicle_name=self.vehicle_name)
