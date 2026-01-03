#!/usr/bin/env python3
"""
运行所有测试

使用方法:
    python run_all_tests.py              # 运行所有测试
    python run_all_tests.py -v           # 详细输出
    python run_all_tests.py -k "state"   # 只运行包含 "state" 的测试
"""
import subprocess
import sys
import os


def main():
    # 设置 Python 路径
    project_root = os.path.dirname(os.path.abspath(__file__))
    
    # 测试目录
    test_dirs = [
        os.path.join(project_root, 'universal_controller', 'tests'),
        os.path.join(project_root, 'controller_ros', 'test'),
    ]
    
    # 构建 pytest 命令
    cmd = [sys.executable, '-m', 'pytest']
    
    # 添加测试目录
    for test_dir in test_dirs:
        if os.path.exists(test_dir):
            cmd.append(test_dir)
    
    # 添加命令行参数
    cmd.extend(sys.argv[1:])
    
    # 如果没有指定详细程度，默认使用 -v
    if '-v' not in sys.argv and '-q' not in sys.argv:
        cmd.append('-v')
    
    # 添加颜色输出
    cmd.append('--color=yes')
    
    print(f"Running: {' '.join(cmd)}")
    print("=" * 60)
    
    # 运行测试
    result = subprocess.run(cmd, cwd=project_root)
    
    return result.returncode


if __name__ == '__main__':
    sys.exit(main())
