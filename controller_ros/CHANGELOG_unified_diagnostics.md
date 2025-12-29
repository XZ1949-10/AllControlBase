# unified_diagnostics.py 修改日志

## 修改日期
2024-12-29

## 修改内容

### 问题
`--log-file` 参数在 `tuning` 模式下不起作用，日志文件不会被创建。

### 原因
1. `run_tuning()` 方法没有调用 `_init_log()` 初始化日志文件
2. `_run_topic_monitoring()`, `_run_chassis_tests()`, `_run_controller_diagnostics()`, `_show_tuning_results()`, `_generate_config()` 等方法使用 `print()` 而不是 `self._log()`
3. `main()` 函数中只为 `realtime` 和 `full` 模式设置默认日志文件

### 修改内容

#### 1. 修改 `run_tuning()` 方法
- 添加 `self._init_log()` 调用
- 添加 `try-finally` 块确保日志文件被正确关闭
- 将 `print()` 改为 `self._log()`

```python
def run_tuning(self):
    """运行系统调优模式"""
    self._init_ros_node('unified_diagnostics_tuning')
    self._init_log()  # 初始化日志文件
    
    try:
        self._log(f"\n{Colors.GREEN}{'='*70}")
        # ... 其他代码
    finally:
        self._close_log()  # 确保日志文件被关闭
        if self.log_file:
            print(f"\n日志已保存到: {self.log_file}")
```

#### 2. 修改辅助方法
将以下方法中的 `print()` 改为 `self._log()`:
- `_run_topic_monitoring()`
- `_run_chassis_tests()`
- `_run_controller_diagnostics()`
- `_show_tuning_results()`
- `_generate_config()`

#### 3. 修改 `main()` 函数
为所有模式设置默认日志文件：

```python
# 默认日志文件 - 所有模式都支持日志
if args.log_file is None:
    if args.mode == 'realtime':
        args.log_file = '/tmp/unified_diag_realtime.log'
    elif args.mode == 'tuning':
        args.log_file = '/tmp/unified_diag_tuning.log'
    elif args.mode == 'full':
        args.log_file = '/tmp/unified_diag_full.log'
```

## 使用示例

### 使用默认日志文件
```bash
# tuning 模式 - 日志保存到 /tmp/unified_diag_tuning.log
rosrun controller_ros unified_diagnostics.py --mode tuning \
  --duration 300 \
  --output ~/config.yaml

# realtime 模式 - 日志保存到 /tmp/unified_diag_realtime.log
rosrun controller_ros unified_diagnostics.py --mode realtime
```

### 使用自定义日志文件
```bash
# 指定日志文件路径
rosrun controller_ros unified_diagnostics.py --mode tuning \
  --duration 300 \
  --output ~/first_deployment_config.yaml \
  --log-file ~/first_deployment.log
```

## 效果
- ✅ `tuning` 模式现在会创建日志文件
- ✅ 所有诊断输出同时显示在终端和日志文件中
- ✅ 日志文件在程序结束时正确关闭
- ✅ 每个模式有独立的默认日志文件名

## 测试建议
```bash
# 测试 1: 默认日志文件
rosrun controller_ros unified_diagnostics.py --mode tuning --duration 5
ls -lh /tmp/unified_diag_tuning.log

# 测试 2: 自定义日志文件
rosrun controller_ros unified_diagnostics.py --mode tuning \
  --duration 5 \
  --log-file ~/test.log
ls -lh ~/test.log
```
