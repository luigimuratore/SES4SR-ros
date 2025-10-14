import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/ses4r_ws/src/lab01_pkg/install/lab01_pkg'
