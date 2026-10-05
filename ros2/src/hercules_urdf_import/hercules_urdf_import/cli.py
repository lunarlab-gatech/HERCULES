"""Command line entry point: ``hercules_urdf_import`` / ``python -m hercules_urdf_import``."""

import argparse
import sys

from .importer import DRIVES, import_robot, ImportOptions
from .meshes import MeshError
from .urdf_model import UrdfError


def _pairs(values, what, cast=str):
    out = {}
    for value in values or []:
        key, sep, item = value.partition('=')
        if not sep or not key:
            raise UrdfError('%s must look like NAME=VALUE, got "%s"' % (what, value))
        try:
            out[key] = cast(item)
        except ValueError:
            raise UrdfError('%s "%s": bad value "%s"' % (what, key, item))
    return out


def build_parser():
    parser = argparse.ArgumentParser(
        prog='hercules_urdf_import',
        description='Import a URDF robot into HERCULES: writes a settings.json '
                    'vehicle, the visual manifest and meshes for the Unreal pawn, '
                    'and the URDF robot_state_publisher should publish.')
    parser.add_argument('urdf', help='robot description (.urdf, or .xacro when the '
                                     'xacro module is installed)')
    parser.add_argument('--name', required=True,
                        help='HERCULES vehicle name (also the ROS namespace/TF prefix)')
    parser.add_argument('--drive', required=True, choices=sorted(DRIVES),
                        help='which HERCULES vehicle dynamics carry the robot')
    parser.add_argument('--sim-mode', default='hero', choices=('hero', 'native'),
                        help='hero: a vehicle for the mixed UAV/UGV Hero mode '
                             '(default); native: the single-type sim mode for '
                             'the drive (SkidVehicle, Car, Multirotor, ComputerVision)')
    parser.add_argument('-o', '--output-dir', required=True)
    parser.add_argument('--base-link',
                        help="URDF link at the pawn's origin, i.e. the HERCULES body "
                             'frame (default: base_footprint for wheeled drives, '
                             'else base_link, else the root link)')
    parser.add_argument('--sensors', help='YAML/JSON sensor file adding to or '
                                          'overriding the URDF <gazebo> sensors')
    parser.add_argument('--no-gazebo-sensors', action='store_true',
                        help='ignore <gazebo><sensor> blocks in the URDF')
    parser.add_argument('--joint', action='append', metavar='NAME=POSITION',
                        help='position (rad or m) a movable joint is frozen at; '
                             'repeatable')
    parser.add_argument('--keep-joints', action='store_true',
                        help='leave movable joints movable in the output URDF '
                             '(you then publish /<name>/joint_states yourself)')
    parser.add_argument('--package-path', action='append', metavar='PKG=DIR',
                        help='where package://PKG lives; repeatable')
    parser.add_argument('--xacro-arg', action='append', metavar='NAME=VALUE',
                        help='xacro argument; repeatable')
    parser.add_argument('--position', nargs=3, type=float, default=(0.0, 0.0, 0.0),
                        metavar=('X', 'Y', 'Z'),
                        help='spawn position in settings convention (NED meters '
                             'relative to PlayerStart)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='spawn yaw in degrees (clockwise seen from above)')
    parser.add_argument('--show-base-mesh', action='store_true',
                        help="keep drawing the stock pawn's own mesh under the URDF visuals")
    parser.add_argument('--two-sided', action='store_true',
                        help='draw back faces too (for open or inconsistently wound meshes)')
    parser.add_argument('--merge-into', metavar='SETTINGS_JSON',
                        help='add the vehicle to this settings file (written to the '
                             'output directory; the input is not modified)')
    parser.add_argument('--vehicle-type', help='override the VehicleType for the drive')
    parser.add_argument('--pawn-path', help='override the PawnPath for the drive')
    parser.add_argument('--odom-frame', default='odom_local',
                        help="the ROS wrapper's odom_frame_id parameter (default odom_local)")
    parser.add_argument('--relative-paths', action='store_true',
                        help='store paths in settings relative to the output directory')
    parser.add_argument('--keep-mesh-uris', action='store_true',
                        help='keep package:// mesh URIs in the output URDF instead of '
                             'rewriting them to the resolved file:// paths')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        options = ImportOptions(
            urdf_path=args.urdf,
            vehicle_name=args.name,
            drive=args.drive,
            output_dir=args.output_dir,
            sim_mode=args.sim_mode,
            base_link=args.base_link,
            joint_positions=_pairs(args.joint, '--joint', float),
            sensor_config=args.sensors,
            gazebo_sensors=not args.no_gazebo_sensors,
            position=args.position,
            yaw_deg=args.yaw,
            hide_base_mesh=not args.show_base_mesh,
            freeze_joints=not args.keep_joints,
            relative_paths=args.relative_paths,
            package_paths=_pairs(args.package_path, '--package-path'),
            odom_frame=args.odom_frame,
            merge_settings=args.merge_into,
            vehicle_type=args.vehicle_type,
            pawn_path=args.pawn_path,
            rewrite_mesh_uris=not args.keep_mesh_uris,
            xacro_args=_pairs(args.xacro_arg, '--xacro-arg'),
            two_sided=args.two_sided,
        )
        result = import_robot(options)
    except (UrdfError, MeshError, OSError) as exc:
        print('error: %s' % exc, file=sys.stderr)
        return 1

    for warning in result.warnings:
        print('warning: %s' % warning, file=sys.stderr)
    vehicle = result.vehicle
    print('Imported "%s" as %s (%s sim mode): %d visual(s), %d triangles, %d sensor(s).'
          % (args.name, vehicle['VehicleType'], result.settings['SimMode'],
             len(result.manifest['visuals']), result.triangles, len(result.sensors)))
    print('  settings:  %s' % result.files['settings'])
    print('  visuals:   %s' % result.files['manifest'])
    print('  ROS URDF:  %s' % result.files['urdf'])
    print('Copy or merge the settings into ~/Documents/AirSim/settings.json, then run\n'
          '  ros2 launch hercules_urdf_import robot_description.launch.py '
          'settings:=<that settings.json>')
    return 0


if __name__ == '__main__':
    sys.exit(main())
