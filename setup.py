#!/usr/bin/env python

# Try catkin install
try:
   from catkin_pkg.python_setup import generate_distutils_setup
   from distutils.core import setup

   d = generate_distutils_setup(
      packages=['iai_bullet_sim'],
      package_dir={'': 'src'}
   )

   setup(**d)

# Switch to regular pip install
except ModuleNotFoundError:
   from setuptools import setup, find_packages

   setup(
      name='IAI Bullet Sim',
      version='0.2.0',
      author='Adrian Roefer',
      author_email='aroefer@cs.uni-freiburg.de',
      packages=['iai_bullet_sim'],
      package_dir={'': 'src'},
      # scripts=['bin/script1','bin/script2'],
      url='http://pypi.python.org/pypi/iai_bullet_sim/',
      license='LICENSE',
      description='An OO wrapper for Pybullet with optional ROS1 instegration.',
      # long_description=open('README.txt').read(),
      install_requires=[
         "pybullet",
         "numpy",
         "MarkupSafe==2.0.1",  # Jinja installs the wrong markup version.
         "jinja2",
      ],
   )
