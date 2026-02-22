import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/elena/Documentos/GitHub/IR2136/install/uav_inspection'
