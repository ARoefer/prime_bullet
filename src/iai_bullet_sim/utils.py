import os
from collections import namedtuple

Vector3 = namedtuple('Vector3', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])
Frame  = namedtuple('Frame', ['position', 'quaternion'])

def rot3_to_quat(rot3):
	w  = sp.sqrt(1 + rot3[0,0] + rot3[1,1] + rot3[2,2]) * 0.5
	w4 = 4 * w
	x  = (rot3[2,1] - rot3[1,2]) / w4
	y  = (rot3[0,2] - rot3[2,0]) / w4
	z  = (rot3[1,0] - rot3[0,1]) / w4
	return QuatT(x,y,z,w)


def res_pkg_path(rpath):
	if rpath[:10] == 'package://':
		paths = os.environ['ROS_PACKAGE_PATH'].split(':')

		rpath = rpath[10:]
		pkg = rpath[:rpath.find('/')]

		for rpp in paths:
			if rpp[rpp.rfind('/') + 1:] == pkg:
				return '{}/{}'.format(rpp[:rpp.rfind('/')], rpath) 
			if os.path.isdir('{}/{}'.format(rpp, pkg)):
				return '{}/{}'.format(rpp, rpath)
		raise Exception('Package "{}" can not be found in ROS_PACKAGE_PATH!'.format(pkg))
	return rpath