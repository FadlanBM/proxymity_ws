import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fadlan/Documents/Heroes-JAYA/proxymity_ws/install/proxymity'
