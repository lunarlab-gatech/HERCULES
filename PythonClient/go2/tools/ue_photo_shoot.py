"""Editor-side (Unreal Python, PIE running) robot photo helper: frames a free camera around a robot pawn and takes a
1920x1080 HighResShot of it, so the Go2 / G1 can be photographed from any angle instead of only the chase view.

How: a vehicle camera actor that nothing else moves (default Husky1_camera_front_center, a PIPCamera attached to the
idle Husky) is placed by world transform and made the player's view target; the AirSim CameraDirector only repositions
its own ExternalCamera, so the view stays put. SHOT_RESTORE=1 hands the view back to the ExternalCamera.
Run through the MCP bridge (execute_python) or the editor Python console:
    import os, runpy; os.environ.update({"SHOT_ROBOT": "G1", "SHOT_AZ": "40", "SHOT_DIST": "3.6", "SHOT_H": "1.3",
                                         "SHOT_AIM": "0.7", "SHOT_FOV": "40", "SHOT_NAME": "g1_front_left_34"})
    runpy.run_path(".../tools/ue_photo_shoot.py")   # -> <Project>/Saved/Screenshots/LinuxEditor/<SHOT_NAME>.png
Group shot: SHOT_ROBOT2=<other pawn> aims at the midpoint (use SHOT_YAW for the azimuth reference, SHOT_GZ ground z cm).
Walking shots: run tools/photo_driver.py (both bridges, command files) and shoot while the robot moves.
"""
# Params (environment variables): SHOT_ROBOT pawn name; SHOT_AZ deg (0 = camera in front of the robot, + = toward its
# left); SHOT_DIST m; SHOT_H m camera height above the robot's ground; SHOT_AIM m aim height above ground; SHOT_FOV deg;
# SHOT_NAME output stem; SHOT_CAM camera actor name; SHOT_RES (1920x1080); SHOT_RESTORE=1 hands the view back.
import os, math, json, unreal
w = unreal.UnrealEditorSubsystem().get_game_world()
pc = unreal.GameplayStatics.get_player_controller(w, 0)
def find(name, cls=unreal.Actor):
    for a in unreal.GameplayStatics.get_all_actors_of_class(w, cls):
        if a.get_name() == name: return a
    return None
cam = find(os.environ.get("SHOT_CAM", "Husky1_camera_front_center"), unreal.CameraActor)
if os.environ.get("SHOT_RESTORE") == "1":
    ext = find("ExternalCamera", unreal.CameraActor)
    pc.set_view_target_with_blend(ext, 0.0)
    cc = cam.get_component_by_class(unreal.CameraComponent)
    cc.set_active(False); cc.set_visibility(False)
    print(json.dumps({"restored": True, "view": ext.get_name()}))
else:
    rob = find(os.environ["SHOT_ROBOT"])
    az = math.radians(float(os.environ.get("SHOT_AZ", "0")))
    dist = float(os.environ.get("SHOT_DIST", "2.0")) * 100.0
    h = float(os.environ.get("SHOT_H", "0.6")) * 100.0
    aim = float(os.environ.get("SHOT_AIM", "0.25")) * 100.0
    fov = float(os.environ.get("SHOT_FOV", "45"))
    rl = rob.get_actor_location(); ryaw = math.radians(rob.get_actor_rotation().yaw)
    rob2 = find(os.environ["SHOT_ROBOT2"]) if os.environ.get("SHOT_ROBOT2") else None
    if rob2:  # group shot: aim at the midpoint of the two robots
        l2 = rob2.get_actor_location()
        rl = unreal.Vector((rl.x + l2.x) / 2, (rl.y + l2.y) / 2, min(rl.z, l2.z))
    if os.environ.get("SHOT_YAW"):  # absolute UE yaw to measure the azimuth from
        ryaw = math.radians(float(os.environ["SHOT_YAW"]))
    # robot's ground = actor z minus its stand height (both pawns carry StandHeightCm)
    stand = float(rob.get_editor_property("StandHeightCm")) if hasattr(rob, "get_editor_property") else 30.0
    gz = float(os.environ["SHOT_GZ"]) if os.environ.get("SHOT_GZ") else rl.z - stand
    # camera position in the robot frame: forward = +x, left = -y (UE is left-handed)
    fx, fy = math.cos(az) * dist, -math.sin(az) * dist
    cx = rl.x + fx * math.cos(ryaw) - fy * math.sin(ryaw)
    cy = rl.y + fx * math.sin(ryaw) + fy * math.cos(ryaw)
    cz = gz + h
    tx, ty, tz = rl.x, rl.y, gz + aim
    yaw = math.degrees(math.atan2(ty - cy, tx - cx))
    pitch = math.degrees(math.atan2(tz - cz, math.hypot(tx - cx, ty - cy)))
    cam.set_actor_location_and_rotation(unreal.Vector(cx, cy, cz), unreal.Rotator(roll=0.0, pitch=pitch, yaw=yaw), False, True)
    cc = cam.get_component_by_class(unreal.CameraComponent)
    cc.set_visibility(True); cc.set_active(True)
    cc.set_field_of_view(fov)
    cc.set_editor_property("aspect_ratio", 16.0 / 9.0)
    cc.set_editor_property("constrain_aspect_ratio", False)
    if pc.get_view_target() != cam:
        pc.set_view_target_with_blend(cam, 0.0)
    name = os.environ.get("SHOT_NAME", "")
    if name:
        res = os.environ.get("SHOT_RES", "1920x1080")
        unreal.SystemLibrary.execute_console_command(w, "HighResShot %s filename=%s" % (res, name))
    print(json.dumps({"robot": [round(rl.x), round(rl.y), round(rl.z, 1), round(math.degrees(ryaw), 1)], "ground": round(gz, 1),
                      "cam": [round(cx), round(cy), round(cz), round(yaw, 1), round(pitch, 1)], "fov": fov, "name": name}))
