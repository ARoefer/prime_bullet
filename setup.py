#!/usr/bin/env python

# Try catkin install
try:
   from catkin_pkg.python_setup import generate_distutils_setup
   from distutils.core import setup

   d = generate_distutils_setup(
      packages=['prime_bullet'],
      package_dir={'': 'src'}
   )

   setup(**d)

# Switch to regular pip install
except ModuleNotFoundError:
   from pathlib    import Path
   from setuptools import setup, find_packages

   with open(f'{Path(__file__).parent}/requirements.txt', 'r') as f:
      pip_dependencies = f.readlines()

   setup(
      name='Prime Bullet',
      version='1.0.0',
      author='Adrian Roefer',
      author_email='aroefer@cs.uni-freiburg.de',
      packages=['prime_bullet'],
      package_dir={'': 'src'},
      # scripts=['bin/script1','bin/script2'],
      url='http://pypi.python.org/pypi/prime_bullet/',
      license='LICENSE',
      description='Object-oriented wrapper for PyBullet.',
      # long_description=open('README.txt').read(),
      install_requires=pip_dependencies
   )
