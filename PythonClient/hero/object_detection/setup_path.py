# setup_path.py
# Robust path helper to import local hercules from anywhere in nested folders.

import os
import sys
import inspect
import importlib.util
import logging

def _find_upwards(start_dir: str, target_pkg: str) -> str | None:
    """
    Walk upwards from start_dir to filesystem root looking for a directory named target_pkg.
    Returns the absolute path to the package directory if found, else None.
    """
    path = os.path.abspath(start_dir)
    last = None
    while path != last:
        candidate = os.path.join(path, target_pkg)
        if os.path.isdir(candidate):
            # treat it as a valid package if it has __init__.py or a known marker (client.py)
            if (os.path.isfile(os.path.join(candidate, "__init__.py")) or
                os.path.isfile(os.path.join(candidate, "client.py"))):
                return candidate
        last = path
        path = os.path.dirname(path)
    return None

class SetupPath:
    @staticmethod
    def _this_file_dir() -> str:
        cur_filepath = os.path.abspath(inspect.getfile(inspect.currentframe()))
        return os.path.dirname(cur_filepath)

    @staticmethod
    def addHerculesModulePath():
        # 1) If already importable (pip/install), do nothing
        if importlib.util.find_spec("hercules") is not None:
            return

        # 2) Environment override: HERCULES_PATH can point to either the package
        #    directory (.../hercules) or its parent (which contains 'hercules')
        env_path = os.environ.get("HERCULES_PATH")
        if env_path:
            env_path = os.path.abspath(env_path)
            pkg_parent = (os.path.dirname(env_path)
                          if os.path.basename(env_path) == "hercules" else env_path)
            if os.path.isdir(os.path.join(pkg_parent, "hercules")):
                if pkg_parent not in sys.path:
                    sys.path.insert(0, pkg_parent)
                return

        # 3) Walk upwards from this file's directory to find the local package
        cur_dir = SetupPath._this_file_dir()
        pkg_dir = _find_upwards(cur_dir, "hercules")
        if pkg_dir:
            parent = os.path.dirname(pkg_dir)  # We add the directory *containing* the package
            if parent not in sys.path:
                sys.path.insert(0, parent)
            return

        logging.warning(
            "hercules module not found in parent directories. "
            "Falling back to installed package (pip install hercules)."
        )

SetupPath.addHerculesModulePath()
