"""
手柄控制处理器

处理手柄输入，生成控制命令，管理控制模式切换。
"""
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
import time
import logging

from ..models import JoystickState, ControlMode, VelocityData

logger = logging.getLogger(__name__)


@dataclass
class JoystickConfig:
    """手柄配置"""
    enable_button: int = 4          # LB 键
    estop_button: int = 5           # RB 键 (紧急停止)
    resume_button: int = 7          # Start 键 (恢复)
    linear_axis: int = 1            # 左摇杆 Y 轴
    angular_axis: int = 3           # 右摇杆 X 轴
    max_linear: float = 0.5         # 最大线速度 (m/s)
    max_angular: float = 1.0        # 最大角速度 (rad/s)
    deadzone: float = 0.1           # 摇杆死区
    
    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> 'JoystickConfig':
        """从字典创建配置"""
        return cls(
            enable_button=config.get('enable_button', 4),
            estop_button=config.get('estop_button', 5),
            resume_button=config.get('resume_button', 7),
            linear_axis=config.get('linear_axis', 1),
            angular_axis=config.get('angular_axis', 3),
            max_linear=config.get('max_linear', 0.5),
            max_angular=config.get('max_angular', 1.0),
            deadzone=config.get('deadzone', 0.1),
        )


class JoystickHandler:
    """
    手柄控制处理器
    
    职责:
    - 处理手柄输入
    - 管理控制模式切换 (网络/手柄)
    - 生成速度命令
    - 提供模式切换回调
    - 处理紧急停止
    """
    
    def __init__(self, config: JoystickConfig = None):
        """
        初始化手柄处理器
        
        Args:
            config: 手柄配置
        """
        self._config = config or JoystickConfig()
        
        # 状态
        self._current_mode = ControlMode.NETWORK
        self._last_joystick_state: Optional[JoystickState] = None
        self._joystick_connected = False
        self._last_update_time = 0.0
        self._last_estop_state = False  # 上一次紧急停止按键状态
        self._last_resume_state = False  # 上一次恢复按键状态
        
        # 回调
        self._on_mode_change: Optional[Callable[[ControlMode], None]] = None
        self._on_cmd_generated: Optional[Callable[[VelocityData], None]] = None
        self._on_estop: Optional[Callable[[], None]] = None
        self._on_resume: Optional[Callable[[], None]] = None
    
    def set_mode_change_callback(self, callback: Callable[[ControlMode], None]):
        """设置模式切换回调"""
        self._on_mode_change = callback
    
    def set_cmd_callback(self, callback: Callable[[VelocityData], None]):
        """设置命令生成回调"""
        self._on_cmd_generated = callback
    
    def set_estop_callback(self, callback: Callable[[], None]):
        """设置紧急停止回调"""
        self._on_estop = callback
    
    def set_resume_callback(self, callback: Callable[[], None]):
        """设置恢复回调"""
        self._on_resume = callback
    
    def update(self, joystick_state: JoystickState) -> Optional[VelocityData]:
        """
        更新手柄状态
        
        Args:
            joystick_state: 手柄状态
        
        Returns:
            如果处于手柄控制模式，返回速度命令；否则返回 None
        """
        self._last_joystick_state = joystick_state
        self._joystick_connected = joystick_state.connected
        self._last_update_time = time.time()
        
        # 检查紧急停止 (RB 键，上升沿触发)
        if joystick_state.estop_pressed and not self._last_estop_state:
            logger.warn("Emergency stop triggered by joystick (RB)")
            if self._on_estop:
                self._on_estop()
        self._last_estop_state = joystick_state.estop_pressed
        
        # 检查恢复 (Start 键，上升沿触发)
        if joystick_state.resume_pressed and not self._last_resume_state:
            logger.info("Resume triggered by joystick (Start)")
            if self._on_resume:
                self._on_resume()
        self._last_resume_state = joystick_state.resume_pressed
        
        # 检查模式切换
        new_mode = (ControlMode.JOYSTICK 
                    if joystick_state.enable_pressed 
                    else ControlMode.NETWORK)
        
        if new_mode != self._current_mode:
            self._current_mode = new_mode
            logger.info(f"Control mode changed to: {new_mode.name}")
            if self._on_mode_change:
                self._on_mode_change(new_mode)
        
        # 生成命令
        if self._current_mode == ControlMode.JOYSTICK:
            cmd = self._generate_cmd(joystick_state)
            if self._on_cmd_generated:
                self._on_cmd_generated(cmd)
            return cmd
        
        return None
    
    def _generate_cmd(self, state: JoystickState) -> VelocityData:
        """
        从手柄状态生成速度命令
        
        Args:
            state: 手柄状态
        
        Returns:
            速度命令
        """
        # 线速度: 左摇杆 Y 轴
        linear_x = state.linear_cmd * self._config.max_linear
        
        # 角速度: 右摇杆 X 轴
        angular_z = state.angular_cmd * self._config.max_angular
        
        return VelocityData(
            linear_x=linear_x,
            linear_y=0.0,
            angular_z=angular_z,
            timestamp=time.time(),
        )
    
    @property
    def current_mode(self) -> ControlMode:
        """当前控制模式"""
        return self._current_mode
    
    @property
    def is_joystick_connected(self) -> bool:
        """手柄是否连接"""
        return self._joystick_connected
    
    @property
    def last_joystick_state(self) -> Optional[JoystickState]:
        """最后的手柄状态"""
        return self._last_joystick_state
    
    @property
    def config(self) -> JoystickConfig:
        """获取配置"""
        return self._config
    
    def get_status_text(self) -> str:
        """获取状态文本"""
        if not self._joystick_connected:
            return "手柄未连接"
        
        mode_text = "手柄控制" if self._current_mode == ControlMode.JOYSTICK else "网络轨迹"
        return f"模式: {mode_text}"

    def reset(self):
        """重置手柄处理器状态"""
        self._current_mode = ControlMode.NETWORK
        self._last_joystick_state = None
        self._joystick_connected = False
        self._last_update_time = 0.0
        self._last_estop_state = False
        self._last_resume_state = False
        logger.info("JoystickHandler reset")
