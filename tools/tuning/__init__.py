# TurtleBot1 配置诊断与调优工具包
# 
# 使用方法:
#   python -m tools.tuning.run_diagnostics --bag <rosbag_file>
#   python -m tools.tuning.run_diagnostics --live
#
# 功能:
#   1. 分析诊断数据，识别性能瓶颈
#   2. 自动生成优化后的配置文件
#   3. 提供调优建议

__version__ = "3.3.0"
