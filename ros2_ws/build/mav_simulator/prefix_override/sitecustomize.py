import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jj/Desktop/MavLab_Study/ros2_ws/install/mav_simulator'
