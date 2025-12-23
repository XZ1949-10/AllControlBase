#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Universal Controller 安装脚本

安装方法:
    # 可编辑安装 (推荐开发时使用)
    pip install -e .
    
    # 普通安装
    pip install .
"""

from setuptools import setup, find_packages

setup(
    name='universal-controller',
    version='3.18.0',
    author='Universal Controller Team',
    description='基于 MPC 的通用轨迹跟踪控制器，支持多平台部署',
    
    # 自动查找包
    packages=find_packages(include=['universal_controller', 'universal_controller.*']),
    
    # 依赖
    install_requires=[
        'numpy>=1.20.0',
        'scipy>=1.7.0',
        'PyYAML>=5.4.0',
    ],
    
    # Python 版本要求
    python_requires='>=3.8',
    
    # 包含数据文件
    include_package_data=True,
    zip_safe=False,
)
