#!/usr/bin/env python
"""
This file is needed to transform mpl_graspit as a python package accesible by other modules
chekc more info
http://docs.ros.org/en/jade/api/catkin/html/user_guide/setup_dot_py.html
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mpl_graspit'],
    package_dir={'': 'src'}
)


setup(**d)