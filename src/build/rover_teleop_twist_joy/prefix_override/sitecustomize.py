import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/konrad/TrailblazerML/src/install/rover_teleop_twist_joy'
