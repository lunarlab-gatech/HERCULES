"""Resolve URDF resource URIs (package://, model://, file://, relative)."""

import os
from urllib.parse import unquote, urlparse
from urllib.request import url2pathname

from .urdf_model import UrdfError


class PackageResolver:
    """
    Find mesh files the way ROS tools do, without needing ROS.

    ``package://name/...`` is looked up, in order, in the explicit
    ``package_paths`` mapping, the ament index (when ROS 2 is sourced),
    ``AMENT_PREFIX_PATH`` and ``ROS_PACKAGE_PATH``, and finally the
    directories above the URDF (the usual ``<pkg>/urdf/robot.urdf`` layout).
    """

    def __init__(self, urdf_dir, package_paths=None):
        self.urdf_dir = os.path.abspath(urdf_dir)
        self.package_paths = {k: os.path.abspath(v) for k, v in (package_paths or {}).items()}
        self._cache = {}

    def package_dir(self, name):
        if name in self._cache:
            return self._cache[name]
        found = self._find_package(name)
        self._cache[name] = found
        return found

    def _find_package(self, name):
        if name in self.package_paths:
            return self.package_paths[name]
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory(name)
        except Exception:  # not sourced, not installed, or no ament at all
            pass
        for prefix in filter(None, os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)):
            candidate = os.path.join(prefix, 'share', name)
            if os.path.isdir(candidate):
                return candidate
        for root in filter(None, os.environ.get('ROS_PACKAGE_PATH', '').split(os.pathsep)):
            for candidate in (root, os.path.join(root, name)):
                if os.path.basename(candidate) == name and \
                        os.path.isfile(os.path.join(candidate, 'package.xml')):
                    return candidate
        directory = self.urdf_dir
        while True:
            for candidate in (directory, os.path.join(directory, name)):
                if os.path.basename(candidate) == name and os.path.isdir(candidate):
                    return candidate
            parent = os.path.dirname(directory)
            if parent == directory:
                return None
            directory = parent

    def _find_model(self, name):
        variables = ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'GAZEBO_MODEL_PATH')
        for variable in variables:
            for root in filter(None, os.environ.get(variable, '').split(os.pathsep)):
                candidate = os.path.join(root, name)
                if os.path.isdir(candidate):
                    return candidate
        return self.package_dir(name)

    def resolve(self, uri):
        if uri.startswith('package://') or uri.startswith('model://'):
            scheme, rest = uri.split('://', 1)
            name, _, relative = rest.partition('/')
            base = self.package_dir(name) if scheme == 'package' else self._find_model(name)
            if base is None:
                raise UrdfError('cannot find package "%s" for %s (pass --package-path '
                                '%s=/path/to/%s)' % (name, uri, name, name))
            path = os.path.join(base, relative)
        elif uri.startswith('file://'):
            # file:///C:/x on Windows, file:///home/x elsewhere
            path = url2pathname(unquote(urlparse(uri).path))
        else:
            path = os.path.join(self.urdf_dir, uri)
        path = os.path.abspath(path)
        if not os.path.isfile(path):
            raise UrdfError('mesh %s not found at %s' % (uri, path))
        return path
