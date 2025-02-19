import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ecd507/gitchanges/F1Tenth-WS-ECD507/install/ros2_camera'
