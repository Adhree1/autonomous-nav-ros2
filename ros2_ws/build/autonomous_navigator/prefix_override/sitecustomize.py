import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/prohype/autonomous-nav-ros2/ros2_ws/install/autonomous_navigator'
