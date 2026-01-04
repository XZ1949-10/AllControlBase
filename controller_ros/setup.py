#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 Noetic setup.py (用于 catkin_make)

依赖:
- universal_controller: 纯算法库，需要在 PYTHONPATH 中
  export PYTHONPATH=$PYTHONPATH:/path/to/universal_controller/..

注意:
- 使用显式包列表而非 find_packages()，因为 catkin_pkg.python_setup
  需要显式列表来正确处理 catkin 工作空间路径
- 添加新子包时，需要同时更新此列表
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        # 核心包
        'controller_ros',
        'controller_ros.adapters',
        'controller_ros.bridge',
        'controller_ros.dashboard',
        'controller_ros.io',
        'controller_ros.lifecycle',
        'controller_ros.node',
        'controller_ros.utils',
        # 可视化子包
        'controller_ros.visualizer',
        'controller_ros.visualizer.adapters',
        'controller_ros.visualizer.handlers',
        'controller_ros.visualizer.node',
        'controller_ros.visualizer.widgets',
    ],
    package_dir={'': 'src'},
    install_requires=[
        'numpy>=1.20.0',
        'scipy>=1.7.0',
        'matplotlib>=3.0.0',
    ],
)

setup(**setup_args)
