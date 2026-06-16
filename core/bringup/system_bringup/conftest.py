# conftest.py — 本机(无 colcon)跑 pytest 时,把本包目录加入 sys.path,
# 使 `import system_bringup.consistency_check` 可解析。colcon test 下无副作用。
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
