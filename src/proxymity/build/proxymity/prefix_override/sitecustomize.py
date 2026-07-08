import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/heroes/Workspace/proxymity_ws/src/proxymity/install/proxymity'
