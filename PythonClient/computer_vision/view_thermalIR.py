#!/usr/bin/env python3

import time
import numpy as np
import cv2
import setup_path 
import hercules as airsim

def get_ir_frame(client, camera_name="front_center"):
    # 1) request a raw, uncompressed IR image
    req = airsim.ImageRequest(
        camera_name,
        airsim.ImageType.Infrared,
        pixels_as_float=False,  # 8-bit per channel
        compress=False          # no PNG/JPEG wrapper
    )
    res = client.simGetImages([req])[0]

    buf = res.image_data_uint8
    if not buf:
        raise RuntimeError("Empty IR buffer – check CaptureSettings→ForceUpdate.")

    # 2) turn into a 1D numpy array
    arr = np.frombuffer(buf, dtype=np.uint8)
    h, w = res.height, res.width

    # 3) detect how many channels we actually got
    total = arr.size
    pix = w * h
    if total == pix:
        # truly mono8
        img = arr.reshape(h, w)
    elif total == pix * 3:
        # got 3 channels, reshape and convert to gray
        img3 = arr.reshape(h, w, 3)
        # depending on AirSim ordering this may be BGR or RGB, but
        # since all channels are identical for IR it doesn't matter:
        img = cv2.cvtColor(img3, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError(f"Unexpected buffer size {total} for {w}x{h}")

    return img

def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()

    window = "View IR frame"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    try:
        while True:
            ir = get_ir_frame(client)
            # apply a thermal colormap to make it pretty
            vis = cv2.applyColorMap(ir, cv2.COLORMAP_INFERNO)
            cv2.imshow(window, vis)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
