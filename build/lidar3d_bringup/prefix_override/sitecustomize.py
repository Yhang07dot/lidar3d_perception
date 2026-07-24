import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yaoh/lidar3d_ws/install/lidar3d_bringup'
