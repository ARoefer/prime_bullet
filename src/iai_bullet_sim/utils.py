import os


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