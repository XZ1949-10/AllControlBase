# 中文乱码问题修复说明

## 问题描述

在Windows系统上运行 `unified_diagnostics.py` 时，中文字符可能显示为乱码。

## 已实施的修复

### 1. 文件编码声明
文件开头已添加：
```python
# -*- coding: utf-8 -*-
```

### 2. 自动设置终端编码
脚本启动时自动检测Windows系统并设置UTF-8编码：
```python
if sys.platform == 'win32':
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    os.environ['PYTHONIOENCODING'] = 'utf-8'
```

### 3. 日志文件UTF-8编码
所有文件操作都使用UTF-8编码：
```python
open(file, 'w', encoding='utf-8')
```

### 4. 安全打印函数
添加了 `safe_print()` 函数处理编码错误。

## 如果仍然出现乱码

### 方法1: 手动设置控制台代码页（推荐）

在运行脚本前，在CMD中执行：
```cmd
chcp 65001
```

然后运行脚本：
```cmd
rosrun controller_ros unified_diagnostics.py --mode tuning
```

### 方法2: 使用PowerShell（推荐）

PowerShell对UTF-8支持更好：
```powershell
$OutputEncoding = [System.Text.Encoding]::UTF8
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
rosrun controller_ros unified_diagnostics.py --mode tuning
```

### 方法3: 创建批处理文件

创建 `run_diagnostics.bat`：
```batch
@echo off
chcp 65001 > nul
rosrun controller_ros unified_diagnostics.py %*
```

使用方法：
```cmd
run_diagnostics.bat --mode tuning --output config.yaml
```

### 方法4: 修改Windows终端设置

1. 打开"设置" → "时间和语言" → "语言"
2. 点击"管理语言设置"
3. 点击"更改系统区域设置"
4. 勾选"Beta版：使用Unicode UTF-8提供全球语言支持"
5. 重启电脑

### 方法5: 使用Git Bash或WSL

这些终端默认支持UTF-8：
```bash
rosrun controller_ros unified_diagnostics.py --mode tuning
```

## 验证编码设置

运行以下命令检查当前编码：
```cmd
chcp
```

应该显示：`活动代码页: 65001` (UTF-8)

## 日志文件

日志文件始终使用UTF-8编码保存，可以用支持UTF-8的编辑器打开：
- VS Code
- Notepad++
- Sublime Text

避免使用Windows记事本（旧版本不支持UTF-8）。

## 配置文件

生成的YAML配置文件也使用UTF-8编码，可以正常包含中文注释。
