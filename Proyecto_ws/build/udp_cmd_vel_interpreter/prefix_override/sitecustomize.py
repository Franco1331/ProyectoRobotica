import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/franco/Proyecto_ws/install/udp_cmd_vel_interpreter'
