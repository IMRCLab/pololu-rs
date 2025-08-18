import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vincent/Ground_Robots/mocap_broadcast_ros/install/crazyflie_py'
