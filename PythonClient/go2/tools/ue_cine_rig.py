"""Editor-side camera rig for lockstep video capture (Unreal Python; run via the MCP bridge's execute_python).

Registers a Slate pre-tick callback that stays alive after this call returns and talks to an outside orchestrator
through small JSON files in CINE_DIR:
  status.json  written by the rig every game frame (and every 0.5 s while paused):
               {"t": game seconds, "paused": bool, "robots": {"Go2": [x, y, z, yaw, stand_cm], ...}, "seq": last request}
  req.json     written by the orchestrator: {"seq": n, "cam": [x, y, z, pitch, yaw, roll] | null, "fov": deg | null,
               "view": "cam" | "ext" | null, "console": "cmd" | null, "hold": bool | null, "step": bool,
               "shot": "name" | null, "res": "1920x1080", "stop": bool}
  ack.json     {"seq": n, "shot": name | null, "t": game seconds, "ticks": world ticks run by this request}
Lockstep from the game thread: with "hold" the rig keeps the game paused (UGameplayStatics pause); a request with
"step" applies the camera, unpauses, lets exactly ONE world tick run (the Slate pre-tick runs after the world tick of
the same engine frame), pauses again and -- if "shot" is set -- takes the HighResShot of that frame; the ack is
written on the following frame, when the PNG exists. Camera moves only take effect when the game ticks (the player
camera cache is not refreshed while paused), which is why the pose is applied before the step. Pair it with
`t.OverrideFPS <fps>` (console) so every tick advances the game clock by exactly 1/fps.
Env: CINE_DIR (required), CINE_CAM (camera actor, default Husky1_camera_front_center), CINE_STOP=1 to tear down.
"""
import os
import sys
import json
import time
import types
import traceback
import unreal

DIR = os.environ["CINE_DIR"]
REQ, ACK, STATUS, LOG = (os.path.join(DIR, n) for n in ("req.json", "ack.json", "status.json", "rig.log"))
CAM_NAME = os.environ.get("CINE_CAM", "Husky1_camera_front_center")


def _log(msg):
    with open(LOG, "a") as f:
        f.write(time.strftime("%H:%M:%S ") + msg + "\n")


def _write(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


st = sys.modules.get("cine_rig_state")
if st is None:
    st = types.ModuleType("cine_rig_state")
    sys.modules["cine_rig_state"] = st
    st.handle = None


def _find(w, name, cls=unreal.Actor):
    for a in unreal.GameplayStatics.get_all_actors_of_class(w, cls):
        if a.get_name() == name:
            return a
    return None


def _teardown(restore=True):
    try:
        if getattr(st, "hold", False) and unreal.GameplayStatics.is_game_paused(st.w):
            unreal.GameplayStatics.set_game_paused(st.w, False)
        unreal.SystemLibrary.execute_console_command(st.w, "t.OverrideFPS 0")
    except Exception:
        pass
    if st.handle is not None:
        try:
            unreal.unregister_slate_pre_tick_callback(st.handle)
        except Exception:
            unreal.unregister_slate_post_tick_callback(st.handle)
        st.handle = None
    if restore:
        try:
            ext = _find(st.w, "ExternalCamera", unreal.CameraActor)
            st.pc.set_view_target_with_blend(ext, 0.0)
            cc = st.cam.get_component_by_class(unreal.CameraComponent)
            cc.set_active(False)
            cc.set_visibility(False)
        except Exception:
            _log(traceback.format_exc())
    _log("rig stopped")


if os.environ.get("CINE_STOP") == "1":
    _teardown()
    print(json.dumps({"stopped": True}))
else:
    if st.handle is not None:
        _teardown(restore=False)
    st.w = unreal.UnrealEditorSubsystem().get_game_world()
    st.pc = unreal.GameplayStatics.get_player_controller(st.w, 0)
    st.cam = _find(st.w, CAM_NAME, unreal.CameraActor)
    st.robots = {}
    for n in ("Go2", "G1"):
        a = _find(st.w, n)
        if a:
            try:
                stand = float(a.get_editor_property("StandHeightCm"))
            except Exception:
                stand = 30.0
            st.robots[n] = (a, stand)
    st.req_mtime = 0
    st.seq = -1
    st.pending = None
    st.hold = False
    st.step = None          # (seq, shot_name, t_before) while a one-tick step is in flight
    st.last_t = None
    st.last_status = 0.0
    st.res = "1920x1080"
    st.fps = 0.0

    def _status(force=False):
        t = unreal.GameplayStatics.get_time_seconds(st.w)
        now = time.time()
        if not force and t == st.last_t and now - st.last_status < 0.5:
            return
        st.last_t, st.last_status = t, now
        rob = {}
        for n, (a, stand) in st.robots.items():
            l = a.get_actor_location()
            rob[n] = [round(l.x, 2), round(l.y, 2), round(l.z, 2), round(a.get_actor_rotation().yaw, 3), stand]
        _write(STATUS, {"t": t, "paused": unreal.GameplayStatics.is_game_paused(st.w), "robots": rob, "seq": st.seq})

    def tick(dt):
        try:
            t_now = unreal.GameplayStatics.get_time_seconds(st.w)
            if st.pending is not None:
                seq, name, t0, ticks = st.pending
                st.pending = None
                _write(ACK, {"seq": seq, "shot": name, "t": t_now, "ticks": ticks})
            if st.step is not None:
                seq, name, t0 = st.step
                if t_now <= t0:
                    return          # nested Slate tick before the world ticked (editor hitch): keep waiting
                # the world ticked once since the step request was applied: freeze again, then shoot
                st.step = None
                unreal.GameplayStatics.set_game_paused(st.w, True)
                ticks = int(round((t_now - t0) * st.fps)) if st.fps else -1
                if ticks != 1:
                    _log("step seq %d: %d ticks (t %.5f -> %.5f)" % (seq, ticks, t0, t_now))
                if name:
                    unreal.SystemLibrary.execute_console_command(st.w, "HighResShot %s filename=%s" % (st.res, name))
                    st.pending = (seq, name, t0, ticks)
                else:
                    _write(ACK, {"seq": seq, "shot": None, "t": t_now, "ticks": ticks})
                _status(force=True)
                return
            if st.hold and not unreal.GameplayStatics.is_game_paused(st.w):
                unreal.GameplayStatics.set_game_paused(st.w, True)
            _status()
            try:
                m = os.stat(REQ).st_mtime_ns
            except FileNotFoundError:
                return
            if m == st.req_mtime:
                return
            st.req_mtime = m
            with open(REQ) as f:
                r = json.load(f)
            if r.get("seq") == st.seq:
                return
            st.seq = r["seq"]
            if r.get("stop"):
                _write(ACK, {"seq": st.seq, "shot": None, "stopped": True})
                _teardown()
                return
            if r.get("console"):
                unreal.SystemLibrary.execute_console_command(st.w, r["console"])
            if r.get("res"):
                st.res = r["res"]
            if r.get("fps"):
                st.fps = float(r["fps"])
                unreal.SystemLibrary.execute_console_command(st.w, "t.OverrideFPS %g" % st.fps)
            if r.get("hold") is not None:
                st.hold = bool(r["hold"])
                if not st.hold and unreal.GameplayStatics.is_game_paused(st.w):
                    unreal.GameplayStatics.set_game_paused(st.w, False)
            c = r.get("cam")
            if c:
                st.cam.set_actor_location_and_rotation(unreal.Vector(c[0], c[1], c[2]),
                                                       unreal.Rotator(roll=c[5], pitch=c[3], yaw=c[4]), False, True)
            cc = st.cam.get_component_by_class(unreal.CameraComponent)
            if r.get("fov"):
                cc.set_field_of_view(float(r["fov"]))
            if r.get("view") == "cam":
                cc.set_visibility(True)
                cc.set_active(True)
                cc.set_editor_property("aspect_ratio", 16.0 / 9.0)
                cc.set_editor_property("constrain_aspect_ratio", False)
                if st.pc.get_view_target() != st.cam:
                    st.pc.set_view_target_with_blend(st.cam, 0.0)
            elif r.get("view") == "ext":
                ext = _find(st.w, "ExternalCamera", unreal.CameraActor)
                st.pc.set_view_target_with_blend(ext, 0.0)
            if r.get("step"):
                st.step = (st.seq, r.get("shot"), t_now)
                unreal.GameplayStatics.set_game_paused(st.w, False)
            elif r.get("shot"):
                unreal.SystemLibrary.execute_console_command(st.w, "HighResShot %s filename=%s" % (st.res, r["shot"]))
                st.pending = (st.seq, r["shot"], t_now, 0)
            else:
                _write(ACK, {"seq": st.seq, "shot": None, "t": t_now, "ticks": 0})
            _status(force=True)
        except Exception:
            _log(traceback.format_exc())

    try:
        st.handle = unreal.register_slate_pre_tick_callback(tick)
        kind = "pre"
    except AttributeError:
        st.handle = unreal.register_slate_post_tick_callback(tick)
        kind = "post"
    _log("rig started (%s-tick), cam=%s robots=%s" % (kind, st.cam.get_name() if st.cam else None, list(st.robots)))
    print(json.dumps({"started": True, "tick": kind, "cam": st.cam.get_name() if st.cam else None, "robots": list(st.robots)}))
