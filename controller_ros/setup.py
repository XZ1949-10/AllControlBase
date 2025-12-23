#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 Noetic setup.py (用于 catkin_make)
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['controller_ros'],
    package_dir={'': 'src'},
)

setup(**setup_args)
