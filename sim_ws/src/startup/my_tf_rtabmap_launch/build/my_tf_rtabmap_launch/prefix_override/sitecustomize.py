import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rechner1/sophies_wohnzimmer/startup/my_tf_rtabmap_launch/install'
