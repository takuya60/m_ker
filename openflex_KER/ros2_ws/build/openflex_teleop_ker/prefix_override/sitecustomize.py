import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alientek/openflex/openflex_KER/ros2_ws/install/openflex_teleop_ker'
