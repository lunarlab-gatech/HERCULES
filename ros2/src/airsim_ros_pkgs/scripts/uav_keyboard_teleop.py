#!/usr/bin/env python3
"""Terminal keyboard flight control for an airborne AirSim ROS UAV."""

import argparse
from contextlib import contextmanager
import math
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from airsim_interfaces.msg import VelCmd
from airsim_interfaces.srv import Land
from nav_msgs.msg import Odometry


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


@contextmanager
def keyboard():
    descriptor = sys.stdin.fileno()
    original = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, original)


# Topic namespace and odometry topic of each bridge: airsim_node, or the Hero-mode
# hercules_node that hercules_hero_team.launch.py starts.
BRIDGES = {
    "airsim": ("/airsim_node/", "/odom_local"),
    "hercules": ("/hercules_node/", "/ground_truth/odom_local"),
}


class KeyboardFlight(Node):
    def __init__(self, vehicle, bridge="airsim"):
        super().__init__("uav_keyboard_teleop")
        namespace, odometry_topic = BRIDGES[bridge]
        base = namespace + vehicle
        self.last_odometry = None
        self.publisher = self.create_publisher(VelCmd, base + "/vel_cmd_body_frame", 1)
        self.subscription = self.create_subscription(
            Odometry, base + odometry_topic, self.receive_odometry, qos_profile_sensor_data
        )
        self.land_client = self.create_client(Land, base + "/land")

    def receive_odometry(self, message):
        self.last_odometry = time.monotonic()

    def ready(self):
        return (
            self.last_odometry is not None
            and time.monotonic() - self.last_odometry < 1
            and self.publisher.get_subscription_count() > 0
        )

    def send(self, forward=0.0, right=0.0, up=0.0, turn_right=0.0):
        command = VelCmd()
        # The current C++ velocity callbacks pass AirSim's NED components through.
        # Convert up to down here; body Y and positive yaw already point right.
        command.twist.linear.x = float(forward)
        command.twist.linear.y = float(right)
        command.twist.linear.z = float(-up)
        command.twist.angular.z = float(turn_right)
        self.publisher.publish(command)

    def stop(self):
        end = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < end:
            self.send()
            rclpy.spin_once(self, timeout_sec=0.02)

    def land(self):
        if not self.land_client.wait_for_service(timeout_sec=3):
            raise RuntimeError("Landing service is unavailable; the UAV was stopped.")
        request = Land.Request()
        request.wait_on_last_task = True
        print("Landing...", flush=True)
        future = self.land_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=65)
        if not future.done() or not future.result().success:
            raise RuntimeError("Landing did not report success; check the simulator.")
        print("Landed.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", default="SimpleFlight")
    parser.add_argument("--bridge", choices=sorted(BRIDGES), default="airsim",
                        help="airsim_node (default) or the Hero-mode hercules_node")
    parser.add_argument("--speed", type=positive_float, default=0.8, help="horizontal speed in m/s")
    parser.add_argument("--vertical-speed", type=positive_float, default=0.5, help="vertical speed in m/s")
    parser.add_argument("--yaw-rate", type=positive_float, default=30, help="turn rate in degrees/s")
    parser.add_argument("--key-timeout", type=positive_float, default=0.6, help="stop after this many seconds without a movement key")
    args, ros_args = parser.parse_known_args()
    if not sys.stdin.isatty():
        parser.error("run this command in an interactive terminal")

    yaw = math.radians(args.yaw_rate)
    controls = {
        "w": (args.speed, 0, 0, 0), "s": (-args.speed, 0, 0, 0),
        "a": (0, -args.speed, 0, 0), "d": (0, args.speed, 0, 0),
        "r": (0, 0, args.vertical_speed, 0), "f": (0, 0, -args.vertical_speed, 0),
        "q": (0, 0, 0, -yaw), "e": (0, 0, 0, yaw),
    }
    def interrupt_once(signum, frame):
        # ros2 run can forward a second SIGINT while the node is stopping.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt_once)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardFlight(args.vehicle, args.bridge)
    should_land = False
    connected = False
    try:
        print("Waiting for " + args.vehicle + " odometry and flight control...", flush=True)
        end = time.monotonic() + 15
        while not node.ready() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            raise RuntimeError(
                "No active UAV bridge for " + args.vehicle + ". "
                "Check the " + args.bridge + " bridge terminal for a crash and restart the bridge; "
                "also verify the vehicle name and ROS domain."
            )
        connected = True
        print(
            "Ready. Keep this terminal focused. The UAV should already be airborne.\n"
            "W/S forward/back   A/D left/right   R/F up/down   Q/E turn left/right\n"
            "Space stop/hover   G land and exit   X or Ctrl-C exit and hover\n"
            "Hold a movement key to keep moving; releasing it stops within "
            + str(args.key_timeout) + " s.", flush=True
        )
        movement = (0, 0, 0, 0)
        expires = 0
        with keyboard() as descriptor:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0)
                if select.select([descriptor], [], [], 0.05)[0]:
                    data = os.read(descriptor, 64)
                    if not data:
                        break
                    for key in data.decode(errors="ignore").lower():
                        if key in ("x", "\x03"):
                            return
                        if key == "g":
                            should_land = True
                            break
                        if key == " ":
                            movement, expires = (0, 0, 0, 0), 0
                        elif key in controls:
                            movement, expires = controls[key], time.monotonic() + args.key_timeout
                    if should_land:
                        break
                node.send(*(movement if node.ready() and time.monotonic() < expires else (0, 0, 0, 0)))
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        try:
            if should_land:
                try:
                    node.land()
                except KeyboardInterrupt:
                    print("Exiting; the submitted landing request continues in the simulator.", flush=True)
            elif connected:
                print("Flight controls stopped; leaving the UAV hovering.", flush=True)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
