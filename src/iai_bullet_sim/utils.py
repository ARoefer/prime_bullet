import os
import numpy    as np
import pybullet as pb

from hashlib     import md5
from pathlib     import Path


_SEARCH_PATHS = set()

def add_search_path(path):
    _SEARCH_PATHS.add(path)

if 'ROS_PACKAGE_PATH' in os.environ:
    for rpp in os.environ['ROS_PACKAGE_PATH'].split(':'):
        add_search_path(rpp)

# Datastructure representing a color
class ColorRGBA(tuple):
    def __new__(cls, r, g, b, a):
        return super(ColorRGBA, cls).__new__(cls, (r, g, b, a))

    def __add__(self, other):
        return ColorRGBA(*(np.asarray(self) + other))
    
    def __sub__(self, other):
        return ColorRGBA(*(np.asarray(self) - other))
    
    def __mul__(self, other):
        return ColorRGBA(*(np.asarray(self) * other))

    def __div__(self, other):
        return ColorRGBA(*(np.asarray(self) / other))

    def __neg__(self):
        return ColorRGBA(*(-np.asarray(self)))


def res_pkg_path(rpath):
    """Resolves a ROS package relative path to a global path.

    :param rpath: Potential ROS URI to resolve.
    :type rpath: str
    :return: Local file system path
    :rtype: str
    """
    if rpath[:10] == 'package://':
        rpath = rpath[10:]
        pkg = rpath[:rpath.find('/')]

        for rpp in _SEARCH_PATHS:
            if rpp[rpp.rfind('/') + 1:] == pkg:
                return f'{rpp[:rpp.rfind("/")]}/{rpath}'
            if os.path.isdir(f'{rpp}/{pkg}'):
                return f'{rpp}/{rpath}'
        raise Exception(f'Package "{pkg}" can not be found in search paths!')
    return rpath


def res_sdf_model_path(mpath):
    if mpath[:8] == 'model://':
        mpath = mpath[8:]
        if mpath[-4] != '.':  # It's not a model file
            mpath = f'{mpath}/model.sdf'

        for rpp in _SEARCH_PATHS:
            p = Path(f'{rpp}/models/{mpath}')
            if p.exists():
                return str(p)
        raise Exception(f'Could not resolve sdf-model path {mpath}')
    return mpath

def res_sdf_world_path(mpath):
    if mpath[:8] == 'world://':
        mpath = mpath[8:]

        for rpp in _SEARCH_PATHS:
            p = Path(f'{rpp}/worlds/{mpath}')
            if p.exists():
                return str(p)
        raise Exception(f'Could not resolve sdf-world path {mpath}')
    return mpath


def import_class(class_path):
    """Imports a class using a type string.

    :param class_path: Type string of the class.
    :type  class_path: str
    :rtype: type
    """
    components = class_path.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

_RESOLVED_FILES = {}

def abs_urdf_paths(file_path, temp_dir):
    abs_path = Path(res_pkg_path(file_path)).absolute()
    
    if hash(abs_path) in _RESOLVED_FILES:
        return _RESOLVED_FILES[hash(abs_path)]

    hex_name = md5(str(abs_path).encode('utf-8')).hexdigest()
    temp_file_name = f'{temp_dir}/{hex_name}.urdf'

    with open(file_path, 'r') as of:
        with open(temp_file_name, 'w') as tf:
            for l in of:
                idx = l.find('package://', 0)
                while idx != -1:
                    e_idx = l.find('"', idx)
                    pkg_path = l[idx:e_idx]
                    r_path = res_pkg_path(pkg_path)
                    l = l.replace(l[idx:e_idx], r_path)
                    idx = l.find('package://', idx + len(r_path))
                tf.write(l)
    
    _RESOLVED_FILES[abs_path] = temp_file_name

    return temp_file_name


def abs_sdf_paths(file_path, temp_dir):
    abs_path = Path(res_sdf_model_path(file_path)).absolute()
    
    if hash(abs_path) in _RESOLVED_FILES:
        return _RESOLVED_FILES[hash(abs_path)]

    hex_name = md5(str(abs_path).encode('utf-8')).hexdigest()
    temp_file_name = f'{temp_dir}/{hex_name}.sdf'

    with open(file_path, 'r') as of:
        with open(temp_file_name, 'w') as tf:
            for l in of:
                idx = l.find('model://', 0)
                while idx != -1:
                    e_idx = l.find('</uri>', idx)
                    pkg_path = l[idx:e_idx]
                    r_path = res_sdf_model_path(pkg_path)
                    # Recursively resolve SDF files
                    if r_path[-4:].lower() == '.sdf':
                        r_path = abs_sdf_paths(r_path, temp_dir)

                    l = l.replace(l[idx:e_idx], r_path)
                    idx = l.find('model://', idx + len(r_path))
                tf.write(l)
    
    _RESOLVED_FILES[abs_path] = temp_file_name

    return temp_file_name
