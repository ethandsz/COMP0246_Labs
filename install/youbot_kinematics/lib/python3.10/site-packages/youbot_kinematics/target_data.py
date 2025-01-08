import numpy as np

target_joint_pos_1 = [2.5, 1.2, -1.8, 1.5, 1.2]
target_joint_pos_2 = [3.0, 0.8, -2.2, 1.8, 1.0]
target_joint_pos_3 = [1.8, 1.0, -1.5, 2.0, 1.5]
target_joint_pos_4 = [2.8, 1.3, -2.0, 1.2, 0.9]

# rows are target joint positions, columns are the joints
TARGET_JOINT_POSITIONS = np.array([target_joint_pos_1, target_joint_pos_2, target_joint_pos_3, target_joint_pos_4])
