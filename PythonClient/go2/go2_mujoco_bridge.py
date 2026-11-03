#!/usr/bin/env python3
"""MuJoCo locomotion bridge for HERCULES: Unitree Go2 (quadruped) and Unitree G1 (humanoid).

MuJoCo simulates the robot at 500 Hz, a pretrained TorchScript walking policy runs at 50 Hz, and after every
policy step the base pose (+ 12 joint angles) goes to the AirSim pawn listening inside Unreal over TCP
(80-byte "GO2" packets, compatible with S2R_Go2Sim). The pawn (AGo2Pawn / AG1Pawn, both AMujocoPawn) poses the
robot's link meshes to match, so every AirSim sensor on the vehicle works.

Robots (--robot):
  go2  menagerie unitree_go2 model + rl_sar HIMLoco policy (Unreal rig driven by the 12 joint angles)
  g1   Unitree's official g1_12dof model + Unitree's pretrained policy (unitree_rl_gym deploy/pre_train/g1,
       the policy Unitree ships for sim2sim and the real robot); the bridge also streams the pose of every
       link mesh relative to the pelvis (MESHES / LINKS packets) and the pawn renders them generically.

HERCULES extensions (negotiated by a HELLO packet, ignored by older receivers):
  * terrain: the pawn traces the Unreal ground around the robot and streams it back as a heightfield window;
    the bridge feeds it to a MuJoCo hfield so the robot really stands on / bumps into the Unreal terrain;
  * lockstep: with --pacing sim the bridge advances MuJoCo to the Unreal sim time it receives, so pausing
    the simulator (AirSim simPause / lockstep nodes) pauses the robot too. --pacing wall = real time.
    --pacing frames --frame-dt 0.04 advances exactly frame_dt of MuJoCo time per feedback packet (the pawn sends
    one per world tick when no terrain grid is requested) -- for frame-exact video capture with t.OverrideFPS,
    immune to the float32 time stamp in the feedback header (8 ms granularity after a day of editor uptime).

Usage:
  python3 go2_mujoco_bridge.py [--robot go2|g1] [--port 12345] [--pacing wall|sim] [--no-terrain] [--vx 0.5 ...]
or from Python: see go2_commander.RobotCommander.
"""
import argparse
import math
import os
import socket
import struct
import threading
import time

import numpy as np

MAGIC_POSE = 0x474F3200
MAGIC_FEEDBACK = 0x474F3201
MAGIC_HELLO = 0x474F3202
MAGIC_MESHES = 0x474F3204
MAGIC_LINKS = 0x474F3205
POSE_FMT = "<I3f4f12f"      # 80 bytes
HELLO_FMT = "<IIff"         # 16 bytes
FB_FMT = "<IIffffff"        # 32 bytes
FB_SIZE = struct.calcsize(FB_FMT)
MESHES_HDR_FMT = "<II32s"   # 40 bytes + count x 48-byte names
LINKS_HDR_FMT = "<II"       # 8 bytes + count x 7 float32
HELLO_HFIELD, HELLO_FEEDBACK, HELLO_GRID = 1, 2, 4

HERE = os.path.dirname(os.path.abspath(__file__))
THIRDPARTY = os.path.expanduser("~/multi-robot-coordination/go2_thirdparty")
DEFAULT_MENAGERIE = os.environ.get("GO2_MENAGERIE", os.path.join(THIRDPARTY, "mujoco_menagerie/unitree_go2"))
DEFAULT_POLICY_DIR = os.environ.get("GO2_POLICY_DIR", os.path.join(THIRDPARTY, "rl_sar/policy/go2"))
DEFAULT_RL_GYM = os.environ.get("G1_RL_GYM", os.path.join(THIRDPARTY, "unitree_rl_gym"))

# ---------------------------------------------------------------------------------------------------- Go2 policies
# joint order everywhere for the Go2: menagerie / URDF order FL, FR, RL, RR x (hip, thigh, calf)
# standing pose = the policy's default joint pose (hips abducted +-0.1); starting from hips at 0 made the
# HIMLoco gait collapse on a heightfield (offline test 2026-10-30), so stand() uses exactly this
STAND_POSE = np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5])
SIT_POSE = np.array([0.05, 1.22, -2.44, -0.05, 1.22, -2.44, 0.05, 1.22, -2.44, -0.05, 1.22, -2.44])

POLICY_CONFIGS = {
    # rl_sar policy/go2/himloco/config.yaml: 45 obs x 6-step history (newest first) -> 270
    "himloco": dict(file="himloco/himloco.pt", obs=("commands", "ang_vel", "gravity", "dof_pos", "dof_vel", "actions"),
                    history=6, cmd_scale=(2.0, 2.0, 0.25), kp=40.0, kd=1.0, torque_limit=33.5,
                    default_dof=(0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5)),
    # rl_sar policy/go2/robot_lab/config.yaml: 45 obs, no history
    "robot_lab": dict(file="robot_lab/policy.pt", obs=("ang_vel", "gravity", "commands", "dof_pos", "dof_vel", "actions"),
                      history=1, cmd_scale=(1.0, 1.0, 1.0), kp=20.0, kd=0.5, torque_limit=23.5,
                      default_dof=(0.0, 0.8, -1.5) * 4,
                      # policy joint order = rl_sar base.yaml order FR, FL, RR, RL (joint_mapping identity there)
                      joint_perm=(3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8)),
}


def _torch():
    import torch  # local import so the bridge starts fast when only pose mode is used
    torch.set_num_threads(int(os.environ.get("GO2_TORCH_THREADS", "2")))  # small MLPs; more threads just spin
    return torch


class LocoPolicy:
    """rl_sar-style TorchScript locomotion policy for the Go2 (joint order FL, FR, RL, RR x hip, thigh, calf)."""
    ACTION_SCALE = np.array([0.125, 0.25, 0.25] * 4)
    ANG_VEL_SCALE, DOF_VEL_SCALE, CLIP_OBS, CLIP_ACT = 0.25, 0.05, 100.0, 100.0

    def __init__(self, policy_dir, name="himloco"):
        self.torch = _torch()
        self.name = name
        cfg = POLICY_CONFIGS[name]
        self.cfg = cfg
        self.model = self.torch.jit.load(os.path.join(policy_dir, cfg["file"]), map_location="cpu")
        self.model.eval()
        self.DEFAULT_DOF = np.array(cfg["default_dof"], float)
        self.CMD_SCALE = np.array(cfg["cmd_scale"], float)
        self.KP, self.KD, self.TORQUE_LIMIT = cfg["kp"], cfg["kd"], cfg["torque_limit"]
        self.HISTORY = cfg["history"]
        # perm[i] = menagerie index of policy joint i (menagerie order is FL, FR, RL, RR)
        self.perm = np.array(cfg.get("joint_perm", range(12)))
        self.inv_perm = np.argsort(self.perm)
        self.hist = np.zeros((self.HISTORY, 45), np.float32)
        self.actions = np.zeros(12)
        self.primed = False

    @property
    def stand_pose(self):
        return self.DEFAULT_DOF[self.inv_perm].copy()

    def reset(self):
        self.actions[:] = 0.0
        self.primed = False

    def step(self, cmd, ang_vel_body, gravity_body, q, dq, sim_time):
        q, dq = q[self.perm], dq[self.perm]   # menagerie -> policy order
        parts = {"commands": cmd * self.CMD_SCALE, "ang_vel": ang_vel_body * self.ANG_VEL_SCALE, "gravity": gravity_body,
                 "dof_pos": q - self.DEFAULT_DOF, "dof_vel": dq * self.DOF_VEL_SCALE, "actions": self.actions}
        obs = np.concatenate([parts[k] for k in self.cfg["obs"]]).astype(np.float32)
        obs = np.clip(obs, -self.CLIP_OBS, self.CLIP_OBS)
        if not self.primed:
            self.hist[:] = obs
            self.primed = True
        else:
            self.hist = np.roll(self.hist, 1, axis=0)
            self.hist[0] = obs
        with self.torch.no_grad():
            a = self.model(self.torch.from_numpy(self.hist.reshape(1, -1))).numpy()[0]
        self.actions = np.clip(a, -self.CLIP_ACT, self.CLIP_ACT)
        return (self.actions * self.ACTION_SCALE + self.DEFAULT_DOF)[self.inv_perm]  # joint targets, menagerie order


HimLocoPolicy = LocoPolicy  # backwards-compatible name


# ---------------------------------------------------------------------------------------------------- G1 policy
class G1Policy:
    """Unitree's pretrained G1 locomotion policy, exactly as deployed by unitree_rl_gym/deploy/deploy_mujoco
    (configs/g1.yaml): 47 obs = [ang_vel*0.25, gravity, cmd*(2,2,0.25), q-default, dq*0.05, last action,
    sin/cos of a 0.8 s gait phase] -> 12 leg joint targets (action*0.25 + default), PD kp/kd per joint.
    Joint order = g1_12dof.xml order: left (hip pitch, roll, yaw, knee, ankle pitch, roll), then right."""
    CMD_SCALE = np.array([2.0, 2.0, 0.25])
    ANG_VEL_SCALE, DOF_VEL_SCALE, ACTION_SCALE, PERIOD = 0.25, 0.05, 0.25, 0.8
    DEFAULT_DOF = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0])
    KP = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], float)
    KD = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], float)
    TORQUE_LIMIT = None   # the model's joint actuatorfrcrange clamps per joint (88 / 139 / 50 Nm)

    def __init__(self, policy_dir, name="unitree"):
        self.torch = _torch()
        self.name = name
        self.model = self.torch.jit.load(os.path.join(policy_dir, "motion.pt"), map_location="cpu")
        self.model.eval()
        self.actions = np.zeros(12)

    @property
    def stand_pose(self):
        return self.DEFAULT_DOF.copy()

    def reset(self):
        self.actions[:] = 0.0

    def step(self, cmd, ang_vel_body, gravity_body, q, dq, sim_time):
        phase = (sim_time % self.PERIOD) / self.PERIOD
        obs = np.concatenate([ang_vel_body * self.ANG_VEL_SCALE, gravity_body, cmd * self.CMD_SCALE,
                              q - self.DEFAULT_DOF, dq * self.DOF_VEL_SCALE, self.actions,
                              [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)]]).astype(np.float32)
        with self.torch.no_grad():
            self.actions = self.model(self.torch.from_numpy(obs).unsqueeze(0)).numpy()[0].astype(float)
        return self.actions * self.ACTION_SCALE + self.DEFAULT_DOF


# ---------------------------------------------------------------------------------------------------- robots
class RobotSpec:
    def __init__(self, **kw):
        self.__dict__.update(kw)


ROBOTS = {
    "go2": RobotSpec(
        name="go2", vehicle_name="Go2", port=12345, log_tag="go2-bridge",
        xml=os.path.join(DEFAULT_MENAGERIE, "go2.xml"), policy_dir=DEFAULT_POLICY_DIR,
        policies=tuple(POLICY_CONFIGS), default_policy=os.environ.get("GO2_POLICY", "himloco"), policy_class=LocoPolicy,
        base_body="base", imu_site="imu", n_joints=12,
        stand_z=0.30, collapse_z=0.19, topple_reset_z=0.35, sit_pose=SIT_POSE, fallback_stand_pose=STAND_POSE,
        pose_hold=True, settle_time=1.0, fixed_kp=60.0, fixed_kd=5.0, fixed_torque_limit=33.5,
        # Turn assist: HIMLoco turns in place very slowly (~2 deg/s at 0.8 rad/s command; it "skates"). A yaw
        # torque on the trunk proportional to the yaw-rate error (clipped) brings it to ~7 deg/s while the policy
        # keeps balance; 0 disables. For fast turns use pivot mode (kinematic, see set_pivot).
        turn_assist_kp=float(os.environ.get("GO2_TURN_ASSIST_KP", "30.0")), turn_assist_max=24.0,
        links=False, mesh_set=None, mesh_prefix=None),
    "g1": RobotSpec(
        name="g1", vehicle_name="G1", port=12346, log_tag="g1-bridge",
        xml=os.path.join(DEFAULT_RL_GYM, "resources/robots/g1_description/g1_12dof.xml"),
        policy_dir=os.path.join(DEFAULT_RL_GYM, "deploy/pre_train/g1"),
        policies=("unitree",), default_policy="unitree", policy_class=G1Policy,
        base_body="pelvis", imu_site=None, n_joints=12,
        stand_z=0.793, collapse_z=0.55, topple_reset_z=0.85, sit_pose=None, fallback_stand_pose=G1Policy.DEFAULT_DOF,
        # a humanoid cannot be held by joint PD alone (it falls in ~3 s): the policy balances from the first step
        pose_hold=False, settle_time=0.0, fixed_kp=G1Policy.KP, fixed_kd=G1Policy.KD, fixed_torque_limit=None,
        turn_assist_kp=0.0, turn_assist_max=0.0,   # the policy tracks yaw-rate commands itself (0.5 rad/s -> 0.47 measured)
        links=True, mesh_set="G1", mesh_prefix="SM_g1_"),
}


class RobotSim:
    """MuJoCo robot with an Unreal-fed heightfield and a policy / pose-mode controller."""
    DT = 0.002
    DECIMATION = 10          # 500 Hz physics, 50 Hz policy
    HF_ELEV = 6.0            # hfield elevation scale (m): data = (h - zmin) / HF_ELEV

    def __init__(self, spec, terrain=True, grid_half=1.5, grid_cell=0.10, policy=None):
        import mujoco
        self.mujoco = mujoco
        self.spec = spec
        self.terrain = terrain
        self.grid_cell = grid_cell
        self.grid_n = 2 * int(round(grid_half / grid_cell)) + 1
        self.grid_half = 0.5 * (self.grid_n - 1) * grid_cell
        self.nj = spec.n_joints
        robot_dir, robot_xml = os.path.split(spec.xml)
        xml = open(os.path.join(HERE, "mujoco_scene_hfield.xml")).read()
        sensors = ""
        if spec.imu_site:
            sensors = ('<sensor>\n    <framequat name="imu_quat" objtype="site" objname="%s"/>\n    <gyro name="imu_gyro" site="%s"/>\n'
                       '    <accelerometer name="imu_acc" site="%s"/>\n    <framelinvel name="base_linvel" objtype="site" objname="%s"/>\n  </sensor>'
                       % ((spec.imu_site,) * 4))
        xml = (xml.replace("ROBOT_XML", robot_xml).replace("HF_N", str(self.grid_n))
                  .replace("HF_HALF", "%.4f" % self.grid_half).replace("HF_ELEV", "%.3f" % self.HF_ELEV)
                  .replace("<!-- SENSORS -->", sensors))
        if not terrain:
            # flat-floor mode (the Unreal pawn then follows its own ground trace): plain floor at z=0, no hfield
            import re
            xml = re.sub(r'<body name="terrain_body".*?</body>', '', xml, flags=re.S)
            xml = "\n".join(l for l in xml.splitlines() if '<hfield name="terrain"' not in l)
        gen = os.path.join(robot_dir, "%s_hercules_scene.generated.xml" % spec.name)
        with open(gen, "w") as f:
            f.write(xml)
        self.model = mujoco.MjModel.from_xml_path(gen)
        self.data = mujoco.MjData(self.model)
        assert self.model.nu == self.nj and self.model.nq == 7 + self.nj, "unexpected model layout for %s" % spec.name
        self.hf_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "terrain") if terrain else -1
        self.hf_mocap = -1
        self.floor_mocap = self.model.body_mocapid[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "floor_body")]
        self.floor_geom = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.contact_mode = "flat"   # "flat": plane at the local ground height, hfield off | "relief": hfield on
        self.FLAT_RELIEF = 0.05      # cells less than this above the plane are carried by the plane (absorbs small artifacts)
        if terrain:
            self.model.hfield_data[:] = 0.0
            self.hf_mocap = self.model.body_mocapid[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "terrain_body")]
            self.data.mocap_pos[self.hf_mocap] = (0.0, 0.0, -0.07)   # below the plane until the first grid arrives
        self.imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, spec.imu_site) if spec.imu_site else -1
        policy = policy or spec.default_policy
        if policy not in spec.policies:
            raise ValueError("robot %s has policies %s, not %r" % (spec.name, spec.policies, policy))
        self.policy = spec.policy_class(spec.policy_dir, policy)
        self.stand_pose = self.policy.stand_pose if self.policy else np.array(spec.fallback_stand_pose, float)
        self.lock = threading.Lock()
        self.mode = "pose"          # "pose": PD to pose_target (policy off) | "policy": walking policy | "pivot"
        self.cmd = np.zeros(3)      # vx, vy, vyaw for the policy
        self.cmd_wanted = np.zeros(3)
        self.pose_target = self.stand_pose.copy()
        self.pose_from = self.stand_pose.copy()
        self.pose_t0, self.pose_dur = 0.0, 1.0
        self.q_target = self.stand_pose.copy()
        self.fixed_kp, self.fixed_kd = spec.fixed_kp, spec.fixed_kd
        self.turn_assist_kp = spec.turn_assist_kp
        self.turn_assist_max = spec.turn_assist_max
        self.base_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, spec.base_body)
        self.pivot_rate = 0.0
        self.last_grid = None  # (origin_x, origin_y, cell, grid)
        self.policy_steps = 0
        self.policy_enter_step = 0
        self.grid_updates = 0
        self.last_grid_stats = None
        self._init_links()
        self.reset()

    # ---- link meshes (generic Unreal rig) ----
    def _init_links(self):
        """Visual mesh geoms (group 1) -> Unreal static-mesh assets. MuJoCo re-centres every mesh at compile
        (mesh_pos / mesh_quat: v_centred = R^T (v_file - p)); the pawn renders the ORIGINAL mesh files, so the
        streamed pose is the original mesh frame: R_o = R_geom R_m^T, p_o = p_geom - R_o p_m (verified: equals
        body pose o xml geom offset to 1e-6)."""
        m, mujoco = self.model, self.mujoco
        self.link_geoms, self.link_names = [], []
        if not self.spec.links:
            return
        for g in range(m.ngeom):
            if m.geom_group[g] == 1 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                self.link_geoms.append(g)
                self.link_names.append(self.spec.mesh_prefix + mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]))
        self.link_geoms = np.array(self.link_geoms, int)
        mids = m.geom_dataid[self.link_geoms]
        self.link_RmT = np.zeros((len(mids), 3, 3))
        for i, mid in enumerate(mids):
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.mesh_quat[mid])
            self.link_RmT[i] = R.reshape(3, 3).T
        self.link_pm = m.mesh_pos[mids].copy()

    def links_relative(self):
        """(n, 7) [x y z qw qx qy qz] of every link mesh's original frame relative to the base body."""
        d = self.data
        if len(self.link_geoms) == 0:
            return np.zeros((0, 7), np.float32)
        Rg = d.geom_xmat[self.link_geoms].reshape(-1, 3, 3)
        pg = d.geom_xpos[self.link_geoms]
        Ro = Rg @ self.link_RmT
        po = pg - np.einsum("nij,nj->ni", Ro, self.link_pm)
        Rb = d.xmat[self.base_body].reshape(3, 3)
        pb = d.xpos[self.base_body]
        Rrel = np.einsum("ji,njk->nik", Rb, Ro)          # Rb^T Ro
        prel = (po - pb) @ Rb                              # Rb^T (po - pb)
        out = np.zeros((len(self.link_geoms), 7), np.float32)
        out[:, 0:3] = prel
        q = np.zeros(4)
        for i in range(len(self.link_geoms)):
            self.mujoco.mju_mat2Quat(q, Rrel[i].reshape(-1))
            out[i, 3:7] = q
        return out

    # ---- state ----
    def reset(self, xy=(0.0, 0.0), yaw=0.0):
        mujoco = self.mujoco
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = (xy[0], xy[1], self.spec.stand_z)
        self.data.qpos[3:7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        self.data.qpos[7:7 + self.nj] = self.stand_pose
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        if self.policy:
            self.policy.reset()
        self.q_target = self.stand_pose.copy()

    @property
    def time(self):
        return self.data.time

    def body_pose(self):
        """(x, y, z, qw, qx, qy, qz) in the MuJoCo world frame."""
        return tuple(float(v) for v in self.data.qpos[0:7])

    def joints(self):
        return self.data.qpos[7:7 + self.nj].copy()

    def yaw(self):
        w, x, y, z = self.data.qpos[3:7]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def gravity_body(self):
        R = np.zeros(9)
        self.mujoco.mju_quat2Mat(R, self.data.qpos[3:7])
        return R.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])

    def imu(self):
        """gyro (body), accelerometer (body incl. gravity), quat (w,x,y,z), linear velocity (world)."""
        d = self.data
        if self.imu_site >= 0:
            s = d.sensordata
            return {"quat": s[0:4].copy(), "gyro": s[4:7].copy(), "acc": s[7:10].copy(), "linvel": s[10:13].copy()}
        # from the free joint: qvel[3:6] is the body-frame angular velocity, qvel[0:3] the world linear velocity
        return {"quat": d.qpos[3:7].copy(), "gyro": d.qvel[3:6].copy(), "acc": -9.81 * self.gravity_body(), "linvel": d.qvel[0:3].copy()}

    def upright(self):
        w, x, y, z = self.data.qpos[3:7]
        return 1 - 2 * (x * x + y * y) > 0.5   # R[2][2]

    # ---- commands (thread-safe) ----
    WARMUP_STEPS = 50   # policy balances at zero command for 1 s before velocity commands are applied

    def set_velocity(self, vx, vy=0.0, vyaw=0.0):
        with self.lock:
            self.cmd_wanted = np.array((vx, vy, vyaw), float)
            if self.policy and self.mode != "policy":
                self.policy.reset()
                self.mode = "policy"
                self.policy_enter_step = self.policy_steps
            if self.policy_steps - self.policy_enter_step >= self.WARMUP_STEPS:
                self.cmd[:] = self.cmd_wanted
            else:
                self.cmd[:] = 0.0

    def set_pose(self, target, duration=1.5):
        with self.lock:
            self.mode = "pose"
            self.pose_from = self.data.qpos[7:7 + self.nj].copy()
            self.pose_target = np.asarray(target, float).copy()
            self.pose_t0, self.pose_dur = self.data.time, max(1e-3, duration)

    def stand(self, duration=1.5):
        """Go2: PD to the standing pose (policy off). G1: the policy balancing at zero command (a humanoid cannot
        be held by joint PD alone)."""
        if self.spec.pose_hold:
            self.set_pose(self.stand_pose, duration)
        else:
            self.set_velocity(0.0, 0.0, 0.0)

    def set_pivot(self, rate):
        """Kinematic turn in place (rad/s, +left): legs hold the standing pose, the trunk yaw is integrated
        directly. Not physical (the feet slide); use for fast turns where the policy's turning is too weak."""
        with self.lock:
            if self.mode != "pivot":
                self.pose_from = self.data.qpos[7:7 + self.nj].copy()
                self.pose_target = self.stand_pose.copy()
                self.pose_t0, self.pose_dur = self.data.time, 0.5
            self.mode = "pivot" if abs(rate) > 1e-6 else "pose"
            self.pivot_rate = float(rate)

    def ground_height(self, x, y):
        """Height of the Unreal-fed terrain under (x, y) from the last grid (0 without terrain)."""
        if self.last_grid is None:
            return 0.0
        ox, oy, cell, g = self.last_grid
        n = g.shape[0]
        j = int(round((x - ox) / cell)); i = int(round((y - oy) / cell))
        if 0 <= i < n and 0 <= j < n:
            return float(g[i, j])
        return 0.0

    def sit(self, duration=1.5):
        if self.spec.sit_pose is None:
            return False
        self.set_pose(self.spec.sit_pose, duration)
        return True

    # ---- stepping ----
    def step_control(self):
        """One 20 ms control step (DECIMATION physics steps)."""
        mujoco, m, d, nj = self.mujoco, self.model, self.data, self.nj
        with self.lock:
            mode, cmd = self.mode, self.cmd.copy()
            if mode in ("pose", "pivot"):
                s = min(1.0, (d.time - self.pose_t0) / self.pose_dur)
                s = s * s * (3 - 2 * s)
                self.q_target = self.pose_from + (self.pose_target - self.pose_from) * s
        if mode == "policy" and self.policy:
            if self.policy_steps - self.policy_enter_step == self.WARMUP_STEPS:
                with self.lock:
                    self.cmd[:] = self.cmd_wanted
                    cmd = self.cmd.copy()
            q, dq = d.qpos[7:7 + nj], d.qvel[6:6 + nj]
            self.q_target = self.policy.step(cmd, d.qvel[3:6].copy(), self.gravity_body(), q.copy(), dq.copy(), d.time)
            kp, kd, lim = self.policy.KP, self.policy.KD, self.policy.TORQUE_LIMIT
            self.policy_steps += 1
        else:
            kp, kd, lim = self.fixed_kp, self.fixed_kd, self.spec.fixed_torque_limit
        # turn assist torque about the trunk z axis (policy mode with a yaw command only)
        if mode == "policy" and self.turn_assist_kp > 0 and abs(cmd[2]) > 1e-3:
            d.xfrc_applied[self.base_body, 5] = float(np.clip(self.turn_assist_kp * (cmd[2] - d.qvel[5]), -self.turn_assist_max, self.turn_assist_max))
        else:
            d.xfrc_applied[self.base_body, 5] = 0.0
        for _ in range(self.DECIMATION):
            tau = kp * (self.q_target - d.qpos[7:7 + nj]) - kd * d.qvel[6:6 + nj]
            d.ctrl[:] = tau if lim is None else np.clip(tau, -lim, lim)
            if mode == "pivot":
                yaw = self.yaw() + self.pivot_rate * self.DT
                d.qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
                d.qpos[2] = float(self.data.mocap_pos[self.floor_mocap][2]) + self.spec.stand_z
                d.qvel[0:6] = 0.0
            mujoco.mj_step(m, d)

    # ---- terrain ----
    def apply_grid(self, origin_x, origin_y, cell, grid):
        """grid: (n, n) heights in m (row = y, col = x), MuJoCo frame; from the Unreal pawn."""
        if not self.terrain or self.hf_geom < 0:
            return
        n = grid.shape[0]
        if n != self.grid_n or abs(cell - self.grid_cell) > 1e-6:
            return
        if not np.all(np.isfinite(grid)) or np.abs(grid).max() > 20.0 or abs(origin_x) > 1e4 or abs(origin_y) > 1e4:
            self.bad_grids = getattr(self, "bad_grids", 0) + 1   # desynced / garbage packet: ignore
            return
        cx = origin_x + 0.5 * (n - 1) * cell
        cy = origin_y + 0.5 * (n - 1) * cell
        self.last_grid = (origin_x, origin_y, cell, grid)
        self.grid_updates += 1
        # Hybrid contact model (see mujoco_scene_hfield.xml): the plane always sits at the lowest ground of the central
        # +-0.6 m of the window and carries everything within FLAT_RELIEF of it (clean foot contacts); only cells that
        # rise more than that above the plane stay in the heightfield (ramps, steps, obstacle tops as cliffs) --
        # every other cell is pushed 5 cm below the plane so the feet never touch heightfield prisms on flat ground.
        c0 = max(0, int(round(((self.data.qpos[0] - origin_x) / cell) - 0.6 / cell))); c1 = min(n, c0 + int(round(1.2 / cell)) + 1)
        r0 = max(0, int(round(((self.data.qpos[1] - origin_y) / cell) - 0.6 / cell))); r1 = min(n, r0 + int(round(1.2 / cell)) + 1)
        core = grid[r0:r1, c0:c1] if (r1 > r0 and c1 > c0) else grid
        plane_z = float(core.min())
        self.data.mocap_pos[self.floor_mocap] = (0.0, 0.0, plane_z)
        proud = grid > plane_z + self.FLAT_RELIEF
        hf = np.where(proud, grid, plane_z - 0.05)
        zmin = float(hf.min()) - 0.02
        self.model.hfield_data[:] = np.clip((hf - zmin) / self.HF_ELEV, 0.0, 1.0).reshape(-1)
        self.data.mocap_pos[self.hf_mocap] = (cx, cy, zmin)
        self.contact_mode = "relief" if proud.any() else "flat"
        self.last_grid_stats = (float(grid.min()), float(grid.max()), float(grid.mean()), cx, cy)


Go2Sim = RobotSim  # backwards-compatible name


class MujocoBridge:
    """Runs the sim loop in a thread and talks to the Unreal pawn."""

    def __init__(self, host="127.0.0.1", port=None, pacing="wall", terrain=True, grid_half=1.5, grid_cell=0.10,
                 policy=None, robot="go2", verbose=True, connect_timeout=120.0, **_legacy):
        self.spec = ROBOTS[robot] if isinstance(robot, str) else robot
        self.host, self.port, self.pacing, self.verbose = host, int(port or self.spec.port), pacing, verbose
        self.frame_dt = float(os.environ.get('GO2_FRAME_DT', '0.04'))
        self.connect_timeout = connect_timeout
        self.sim = RobotSim(self.spec, terrain, grid_half, grid_cell, policy)
        self.sock = None
        self.rx = b""
        self.sim_time = None       # latest Unreal time
        self.sim_t0 = None         # Unreal time when the robot started
        self.ground_z = 0.0
        self.packets = 0
        self.feedback = 0
        self.running = False
        self.thread = None
        self.last_log = 0.0
        self.recoveries = 0

    def log(self, *a):
        if self.verbose:
            print("[%s]" % self.spec.log_tag, *a, flush=True)

    # ---- socket ----
    def connect(self):
        t0 = time.time()
        while True:
            try:
                s = socket.create_connection((self.host, self.port), timeout=2.0)
                break
            except OSError as e:
                if time.time() - t0 > self.connect_timeout:
                    raise RuntimeError("no %s pawn listening on %s:%d (%s) -- is the level playing?" % (self.spec.vehicle_name, self.host, self.port, e))
                time.sleep(0.5)
        s.setblocking(False)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        flags = (HELLO_HFIELD | HELLO_GRID if self.sim.terrain else 0) | HELLO_FEEDBACK
        self.sock.sendall(struct.pack(HELLO_FMT, MAGIC_HELLO, flags, self.sim.grid_half, self.sim.grid_cell))
        if self.spec.links:
            pkt = struct.pack(MESHES_HDR_FMT, MAGIC_MESHES, len(self.sim.link_names), self.spec.mesh_set.encode())
            pkt += b"".join(struct.pack("<48s", n.encode()) for n in self.sim.link_names)
            self.sock.sendall(pkt)
        self.log("connected to %s:%d (robot=%s terrain=%s pacing=%s grid n=%d cell=%.2f links=%d)" % (
            self.host, self.port, self.spec.name, self.sim.terrain, self.pacing, self.sim.grid_n, self.sim.grid_cell, len(self.sim.link_names)))

    def send_pose(self):
        x, y, z, qw, qx, qy, qz = self.sim.body_pose()
        pkt = struct.pack(POSE_FMT, MAGIC_POSE, x, y, z, qw, qx, qy, qz, *self.sim.joints()[:12])
        if self.spec.links:
            links = self.sim.links_relative()
            pkt += struct.pack(LINKS_HDR_FMT, MAGIC_LINKS, len(links)) + links.astype("<f4").tobytes()
        try:
            self.sock.sendall(pkt)
            self.packets += 1
        except BlockingIOError:
            pass

    def pump_feedback(self):
        try:
            while True:
                chunk = self.sock.recv(1 << 20)
                if not chunk:
                    raise ConnectionError("pawn closed the connection")
                self.rx += chunk
                if len(chunk) < (1 << 20):
                    break
        except BlockingIOError:
            pass
        off = 0
        while len(self.rx) - off >= FB_SIZE:
            magic, n, sim_time, ground_z, cell, ox, oy, _ = struct.unpack_from(FB_FMT, self.rx, off)
            if magic != MAGIC_FEEDBACK:
                off += 1
                continue
            need = FB_SIZE + n * n * 4
            if len(self.rx) - off < need:
                break
            if n > 0:
                grid = np.frombuffer(self.rx, dtype="<f4", count=n * n, offset=off + FB_SIZE).reshape(n, n).astype(float)
                self.sim.apply_grid(ox, oy, cell, grid)
            self.sim_time, self.ground_z = sim_time, ground_z
            if self.sim_t0 is None:
                self.sim_t0 = sim_time
            self.feedback += 1
            off += need
        if off:
            self.rx = self.rx[off:]

    # ---- loop ----
    def start(self):
        self.connect()
        self.running = True
        self.thread = threading.Thread(target=self._loop, name=self.spec.log_tag, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def _recover(self, why):
        """Re-stand (Go2: PD stand pose; G1: policy at zero command) and resume the wanted command 1.5 s later."""
        sim = self.sim
        self.recoveries += 1
        self.log("recovery %d: %s" % (self.recoveries, why))
        self._resume_cmd = sim.cmd_wanted.copy()
        sim.stand(1.0)
        self._resume_at = sim.time + 1.5
        self._stall_ref = None
        self._low_since = None

    def _collapse_guard(self):
        """Guards against the rare bad start / stumble: (1) stall: commanded to move (> 0.2 m/s) but the body has
        not moved 10 cm in 4 s; (2) collapse: body below spec.collapse_z for 1 s in policy mode (not a commanded
        sit); (3) topple: not upright for 1 s -> reset the base upright in place. All counted and logged."""
        sim = self.sim
        x, y, z = sim.body_pose()[:3]
        if sim.mode == "policy" and abs(sim.cmd[0]) > 0.2 and self._resume_at is None:
            if self._stall_ref is None or sim.time - self._stall_ref[0] > 4.0:
                if self._stall_ref is not None and math.hypot(x - self._stall_ref[1], y - self._stall_ref[2]) < 0.10:
                    self._recover("stalled (z=%.2f, no motion for 4 s) -> re-stand" % z)
                    return
                self._stall_ref = (sim.time, x, y)
        else:
            self._stall_ref = None
        low = sim.mode == "policy" and z < sim.spec.collapse_z
        if not low and sim.upright():
            self._low_since = None
            return
        if self._low_since is None:
            self._low_since = sim.time
            return
        if sim.time - self._low_since < 1.0:
            return
        if not sim.upright():
            with sim.lock:
                yaw = sim.yaw()
                sim.data.qpos[2] = sim.ground_height(sim.data.qpos[0], sim.data.qpos[1]) + sim.spec.topple_reset_z
                sim.data.qpos[3:7] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
                sim.data.qpos[7:7 + sim.nj] = sim.stand_pose
                sim.data.qvel[:] = 0.0
                if sim.policy:
                    sim.policy.reset()
            self._recover("toppled -> reset upright")
        else:
            self._recover("collapsed (z=%.2f) -> re-stand" % z)

    def _loop(self):
        sim = self.sim
        self._low_since = None
        self._stall_ref = None
        self._resume_at = None
        self._resume_cmd = np.zeros(3)
        step = sim.DT * sim.DECIMATION
        wall_t0 = time.time() - sim.time
        settle_until = sim.time + sim.spec.settle_time      # Go2: stand on fixed gains before the policy takes over
        self.send_pose()
        while self.running:
            self.pump_feedback()
            if self.pacing == "sim" and self.sim_t0 is not None:
                target = self.sim_time - self.sim_t0
            elif self.pacing == "frames":
                target = self.feedback * self.frame_dt
            else:
                target = time.time() - wall_t0
            if sim.time + step > target + 1e-6:
                time.sleep(0.002)
                # grid/feedback may still arrive while we wait; wall-clock drift guard
                if self.pacing != "sim" and target - sim.time < -0.5:
                    wall_t0 = time.time() - sim.time
                continue
            # catch up at most 0.25 s per iteration; beyond that re-anchor (never free-wheel)
            if target - sim.time > 0.25 and self.pacing != "frames":
                if self.pacing == "sim":
                    self.sim_t0 = self.sim_time - sim.time - 0.25
                else:
                    wall_t0 = time.time() - sim.time - 0.25
            if sim.time >= settle_until and sim.mode == "pose" and sim.policy and not getattr(self, "_policy_started", False) \
                    and np.allclose(sim.pose_target, sim.stand_pose):
                self._policy_started = True
                sim.set_velocity(0.0, 0.0, 0.0)
            sim.step_control()
            self.send_pose()
            self._collapse_guard()
            if self._resume_at is not None and sim.time >= self._resume_at:
                self._resume_at = None
                sim.set_velocity(*self._resume_cmd)
            if os.environ.get("GO2_DEBUG") and sim.policy_steps % 25 == 0 and sim.mode == "policy":
                x, y, z = sim.body_pose()[:3]
                mp = sim.data.mocap_pos[sim.hf_mocap] if sim.hf_mocap >= 0 else None
                self.log("dbg t=%.2f z=%.3f xy=(%.2f,%.2f) nc=%d mocap=%s q0=%s tgt0=%s" % (
                    sim.time, z, x, y, sim.data.ncon, None if mp is None else np.round(mp, 3).tolist(),
                    np.round(sim.joints()[:3], 2).tolist(), np.round(sim.q_target[:3], 2).tolist()))
            if self.verbose and time.time() - self.last_log > 5.0:
                self.last_log = time.time()
                x, y, z = sim.body_pose()[:3]
                self.log("t=%.1fs pos=(%.2f, %.2f, %.2f) yaw=%.0f mode=%s cmd=%s upright=%s pkts=%d fb=%d grids=%d%s" % (
                    sim.time, x, y, z, math.degrees(sim.yaw()), sim.mode, np.round(sim.cmd, 2).tolist(), sim.upright(),
                    self.packets, self.feedback, sim.grid_updates,
                    "" if self.sim_time is None else " ue_t=%.1f gz=%.2f" % (self.sim_time, self.ground_z)))
                if sim.last_grid_stats:
                    g = sim.last_grid_stats
                    self.log("  grid min=%.3f max=%.3f mean=%.3f centre=(%.2f, %.2f) contacts=%d mode=%s" % (
                        g[0], g[1], g[2], g[3], g[4], sim.data.ncon, sim.contact_mode))


Go2Bridge = MujocoBridge  # backwards-compatible name (robot="go2" default)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=tuple(ROBOTS), default=os.environ.get("GO2_ROBOT", "go2"))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=int(os.environ["GO2_PORT"]) if os.environ.get("GO2_PORT") else None,
                    help="pawn port (default: 12345 for go2, 12346 for g1)")
    ap.add_argument("--pacing", choices=("wall", "sim", "frames"), default="wall")
    ap.add_argument("--frame-dt", type=float, default=0.04, help="MuJoCo seconds per feedback packet with --pacing frames")
    ap.add_argument("--no-terrain", action="store_true", help="flat MuJoCo floor; the pawn follows its own ground trace")
    # 10 cm cells: at 5 cm the hfield's multi-contact artifacts under the 2.2 cm foot spheres made the gait stumble
    # or stall depending on where the feet landed on the lattice (offline sweep 2026-10-30); 10 cm is consistent.
    ap.add_argument("--grid-half", type=float, default=1.5)
    ap.add_argument("--grid-cell", type=float, default=0.10)
    ap.add_argument("--policy", default=None, help="go2: himloco (default) | robot_lab; g1: unitree")
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--vyaw", type=float, default=0.0)
    ap.add_argument("--script", default="", help="semicolon list: forward:2.0;turn:90;sit;stand;wait:3 (needs go2_commander)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this many wall seconds (0 = run forever)")
    a = ap.parse_args()
    os.environ["GO2_FRAME_DT"] = str(a.frame_dt)
    br = MujocoBridge(a.host, a.port, a.pacing, not a.no_terrain, a.grid_half, a.grid_cell, policy=a.policy, robot=a.robot).start()
    try:
        if a.script:
            from go2_commander import RobotCommander
            RobotCommander(bridge=br).run_script(a.script)
        else:
            if a.vx or a.vy or a.vyaw:
                time.sleep(1.5)
                br.sim.set_velocity(a.vx, a.vy, a.vyaw)
            t0 = time.time()
            while br.running and (a.duration <= 0 or time.time() - t0 < a.duration):
                time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        br.stop()


if __name__ == "__main__":
    main()
