import sys
if sys.prefix == '/home/alejandro/.local/share/mamba/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alejandro/ros2_ws/install/my_first_package'
