import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/magnusmeldgaard/tb3_devspace/install/tb3_remote_control'
