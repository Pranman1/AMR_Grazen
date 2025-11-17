import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranav/turtlebot3_ws/src/grazen_project/install/grazen_project'
