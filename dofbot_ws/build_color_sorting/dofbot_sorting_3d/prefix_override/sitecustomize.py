import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jloy/Desktop/robot-arm/dofbot_ws/install_color_sorting/dofbot_sorting_3d'
