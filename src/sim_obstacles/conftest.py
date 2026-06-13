# 让本机（无 colcon install）的 pytest 能直接 import sim_obstacles
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
