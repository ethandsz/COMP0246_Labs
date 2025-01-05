import sys
if sys.prefix == '/Users/ethandsouza/miniforge3/envs/roboenv-py3.10':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/ethandsouza/Documents/GitHub/COMP0246_Labs/install/youbot_kinematics'
