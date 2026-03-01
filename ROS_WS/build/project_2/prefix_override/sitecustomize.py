import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tyler/Coding_Workspace/CSCE_752/ROS_WS/install/project_2'
