#!/usr/bin/env python3

import os
import time
import argparse
import numpy as np
import cv2
import setup_path  # ensure this points to your Hercules/HERCULES PythonClient
import hercules as airsim

# Aliases
ImageRequest = airsim.ImageRequest
ImageType    = airsim.ImageType


def reshape_buffer(buf, h, w, channels_expected):
    arr = np.frombuffer(buf, dtype=np.uint8)
    total = arr.size
    pix = h * w
    # try expected
    if total == pix * channels_expected:
        return arr.reshape(h, w, channels_expected)
    # fallback: mono
    if total == pix:
        return arr.reshape(h, w, 1)
    # fallback: RGB
    if total == pix * 3:
        return arr.reshape(h, w, 3)
    # fallback: RGBA
    if total == pix * 4:
        return arr.reshape(h, w, 4)
    raise ValueError(f"Unexpected buffer size {total} for {w}x{h} with ch {channels_expected}")


def get_image(obj_name, client, cam_name="front_center", delay=0.1):
    # Teleport vehicle to object pose
    pose = client.simGetObjectPose(obj_name)
    client.simSetVehiclePose(pose, True)
    time.sleep(delay)

    # Request IR, scene, segmentation
    reqs = [
        ImageRequest(cam_name, ImageType.Infrared,    False, False),
        ImageRequest(cam_name, ImageType.Scene,       False, False),
        ImageRequest(cam_name, ImageType.Segmentation,False, False)
    ]
    res = client.simGetImages(reqs)

    h, w = res[0].height, res[0].width

    # IR mono8
    ir_buf = res[0].image_data_uint8
    ir_arr = np.frombuffer(ir_buf, np.uint8)
    if ir_arr.size == h*w:
        ir = ir_arr.reshape(h, w)
    else:
        # if multi-channel, take first channel
        ir_img = reshape_buffer(ir_buf, h, w, 3)
        ir = ir_img[..., 0]

    # Scene: prefer BGR
    buf_s = res[1].image_data_uint8
    ch_s = reshape_buffer(buf_s, h, w, 3)
    if ch_s.shape[2] == 3:
        # assume BGR or RGB
        scene = ch_s
    else:
        # RGBA -> drop alpha
        scene = ch_s[..., :3]

    # Segmentation: per-pixel labels in red channel
    buf_seg = res[2].image_data_uint8
    ch_seg = reshape_buffer(buf_seg, h, w, 1)
    if ch_seg.shape[2] == 1:
        seg = ch_seg[..., 0]
    else:
        # multi-channel -> take first
        seg = ch_seg[..., 0]

    return ir, scene, seg, pose.position


def main(settings_json, out_dir, cam_name="front_center"):
    # create folders
    ir_folder    = os.path.join(out_dir, 'ir')
    scene_folder = os.path.join(out_dir, 'scene')
    seg_folder   = os.path.join(out_dir, 'segmentation')
    for d in (ir_folder, scene_folder, seg_folder):
        os.makedirs(d, exist_ok=True)

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    # List mesh instances
    all_objs = client.simListSceneObjects()
    roots = []
    for name in all_objs:
        parts = name.split('_')
        if len(parts) >= 2 and not parts[1].isdigit():
            root = f"{parts[0]}_{parts[1]}"
            if root not in roots:
                roots.append(root)
    object_list = [n for n in all_objs if any(n.startswith(r + '_') for r in roots)]

    idx = 0
    for obj in object_list:
        try:
            ir, scene, seg, pos = get_image(obj, client, cam_name)
        except Exception as e:
            print(f"Failed capture for {obj}: {e}")
            continue

        fname = str(idx).zfill(4)
        cv2.imwrite(os.path.join(ir_folder,    f"ir_{fname}.png"),  ir)
        cv2.imwrite(os.path.join(scene_folder, f"scene_{fname}.png"), scene)
        cv2.imwrite(os.path.join(seg_folder,   f"seg_{fname}.png"),   seg)

        print(f"[{idx:04d}] Captured {obj} at {pos}")
        idx += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--settings', required=True, help='Unused settings.json path')
    parser.add_argument('--out',      required=True, help='Output dir')
    parser.add_argument('--cam_name', default='front_center', help='Camera name')
    args = parser.parse_args()
    main(args.settings, args.out, args.cam_name)
