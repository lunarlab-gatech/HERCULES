"""
Read the beam geometry of an Ouster lidar from its metadata JSON.

This is the file the Ouster SDK and ouster-ros save next to recordings (for
example ``ros2 service call /ouster/get_metadata``). Both the current layout
(``beam_intrinsics``, ``config_params``, ``lidar_data_format``) and the legacy
flat layout of older firmware are understood.

Ouster measures azimuth clockwise seen from above and fires each beam at
``encoder angle - beam_azimuth_angles[i]``, i.e. a positive beam azimuth
angle turns the beam clockwise. HERCULES sweeps its lidar clockwise too, so
the angles carry over unchanged as the lidar's ``AzimuthOffsets``; the
altitudes (positive up, top beam first) become its ``VerticalAngles``. Mount
the HERCULES lidar on the Ouster's lidar frame (``os_lidar`` in ouster-ros),
where the encoder angle is measured from.
"""

import json
import math
import re

from .urdf_model import UrdfError


def _first(data, *paths):
    for path in paths:
        node = data
        for key in path:
            if not isinstance(node, dict) or key not in node:
                node = None
                break
            node = node[key]
        if node is not None:
            return node
    return None


def _angles(values, what, path):
    if not isinstance(values, list) or not values:
        raise UrdfError('%s: %s missing or empty' % (path, what))
    try:
        out = [float(v) for v in values]
    except (TypeError, ValueError):
        raise UrdfError('%s: %s must be numbers' % (path, what))
    if not all(math.isfinite(v) for v in out):
        raise UrdfError('%s: %s has a non-finite value' % (path, what))
    return out


def read_ouster_metadata(path):
    """
    Return the lidar parameters an Ouster metadata file describes.

    Keys: ``vertical_angles`` and ``azimuth_offsets`` (degrees, one per
    channel), ``channels``, ``measurements_per_cycle``,
    ``rotations_per_second`` and ``product`` (e.g. ``OS-1-64``, or None).
    """
    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data = json.load(stream)
    except (OSError, ValueError) as exc:
        raise UrdfError('cannot read Ouster metadata %s: %s' % (path, exc))
    if not isinstance(data, dict):
        raise UrdfError('%s is not an Ouster metadata object' % path)

    altitudes = _angles(_first(data, ('beam_intrinsics', 'beam_altitude_angles'),
                               ('beam_altitude_angles',)), 'beam_altitude_angles', path)
    azimuths = _first(data, ('beam_intrinsics', 'beam_azimuth_angles'), ('beam_azimuth_angles',))
    azimuths = _angles(azimuths, 'beam_azimuth_angles', path) if azimuths is not None \
        else [0.0] * len(altitudes)
    if len(azimuths) != len(altitudes):
        raise UrdfError('%s: %d beam altitudes but %d beam azimuths'
                        % (path, len(altitudes), len(azimuths)))

    mode = _first(data, ('config_params', 'lidar_mode'), ('lidar_mode',))
    match = re.match(r'^(\d+)x(\d+)$', str(mode or ''))
    if not match:
        raise UrdfError('%s: lidar_mode "%s" is not of the form <columns>x<Hz>' % (path, mode))
    columns, rate = int(match.group(1)), int(match.group(2))
    declared = _first(data, ('lidar_data_format', 'columns_per_frame'),
                      ('data_format', 'columns_per_frame'))
    if declared is not None and int(declared) != columns:
        raise UrdfError('%s: columns_per_frame %s disagrees with lidar_mode %s'
                        % (path, declared, mode))
    pixels = _first(data, ('lidar_data_format', 'pixels_per_column'),
                    ('data_format', 'pixels_per_column'))
    if pixels is not None and int(pixels) != len(altitudes):
        raise UrdfError('%s: pixels_per_column %s but %d beam altitudes'
                        % (path, pixels, len(altitudes)))

    return {
        'vertical_angles': altitudes,
        'azimuth_offsets': azimuths,
        'channels': len(altitudes),
        'measurements_per_cycle': columns,
        'rotations_per_second': rate,
        'product': _first(data, ('sensor_info', 'prod_line'), ('prod_line',)),
    }
