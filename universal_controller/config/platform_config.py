"""平台配置

定义不同机器人平台的运动学特性和控制维度。

坐标系说明 (不需要建图/定位):
==================================

系统使用两个坐标系:

1. base_link (机体坐标系)
   - 原点在机器人中心，X轴朝前，Y轴朝左
   - 网络输出的轨迹在此坐标系下

2. odom (里程计坐标系)
   - 从启动位置开始累积，不需要建图
   - 控制器在此坐标系下工作

控制输出坐标系 (output_frame):
   - 差速车/阿克曼车: base_link (vx, omega) - 机体坐标系
   - 全向车/四旋翼: odom (vx, vy, omega) - 里程计坐标系

状态向量维度定义: [px, py, pz, vx, vy, vz, theta, omega]
                  [0,  1,  2,  3,  4,  5,  6,     7    ]
                  
其中 px, py, pz, vx, vy, vz 都在 odom 坐标系下。

支持的平台类型:
- ackermann: 阿克曼转向车辆 (如汽车)
- differential: 差速驱动车辆 (如两轮机器人)
- omni: 全向移动平台 (如麦克纳姆轮)
- quadrotor: 四旋翼无人机
"""
from ..core.enums import PlatformType


PLATFORM_CONFIG = {
    # 阿克曼转向车辆 (如汽车)
    "ackermann": {
        "type": PlatformType.ACKERMANN,
        "active_dims": [0, 1, 3, 6],      # 活跃状态维度: px, py, vx, theta
        "control_dims": [3, 7],            # 控制维度: vx (纵向速度), omega (角速度)
        "constraints": {
            "pz": 0,           # z 位置约束为 0 (地面车辆)
            "vy": 0,           # 横向速度约束为 0 (非完整约束)
            "vz": 0,           # z 速度约束为 0
            "curvature": True  # 启用曲率约束 (转向角限制)
        },
        "velocity_heading_coupled": True,  # 速度方向与航向耦合 (车头朝向即运动方向)
        "output_type": "differential",     # 输出类型: 差速 (v, omega)
        "output_frame": "base_link"        # 输出坐标系: 机体坐标系
    },
    
    # 差速驱动车辆 (如两轮机器人)
    "differential": {
        "type": PlatformType.DIFFERENTIAL,
        "active_dims": [0, 1, 3, 6, 7],    # 活跃状态维度: px, py, vx, theta, omega
        "control_dims": [3, 7],            # 控制维度: vx, omega
        "constraints": {
            "pz": 0,            # z 位置约束为 0
            "vy": 0,            # 横向速度约束为 0 (非完整约束)
            "vz": 0,            # z 速度约束为 0
            "curvature": False  # 无曲率约束 (可原地旋转)
        },
        "velocity_heading_coupled": True,  # 速度方向与航向耦合
        "output_type": "differential",     # 输出类型: 差速
        "output_frame": "base_link"        # 输出坐标系: 机体坐标系
    },
    
    # 全向移动平台 (如麦克纳姆轮)
    "omni": {
        "type": PlatformType.OMNI,
        "active_dims": [0, 1, 3, 4, 6, 7],  # 活跃状态维度: px, py, vx, vy, theta, omega
        "control_dims": [3, 4, 7],          # 控制维度: vx, vy, omega
        "constraints": {
            "pz": 0,  # z 位置约束为 0
            "vz": 0   # z 速度约束为 0
        },
        "velocity_heading_coupled": False,  # 速度方向与航向解耦 (可横向移动)
        "output_type": "omni",              # 输出类型: 全向
        "output_frame": "world"             # 输出坐标系: 世界坐标系
    },
    
    # 四旋翼无人机
    "quadrotor": {
        "type": PlatformType.QUADROTOR,
        "active_dims": [0, 1, 2, 3, 4, 5, 6, 7],  # 全部状态维度都活跃
        "control_dims": [3, 4, 5, 7],              # 控制维度: vx, vy, vz, omega (yaw rate)
        "constraints": {},                         # 无运动学约束 (可自由飞行)
        "attitude_interface": True,                # 使用姿态接口 (输出期望姿态角)
        "velocity_heading_coupled": False,         # 速度方向与航向解耦
        "output_type": "3d",                       # 输出类型: 三维
        "output_frame": "world"                    # 输出坐标系: 世界坐标系
    }
}
