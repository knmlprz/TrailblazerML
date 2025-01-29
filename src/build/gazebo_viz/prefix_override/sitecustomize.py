import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/konrad/TrailblazerML/src/install/gazebo_viz'
