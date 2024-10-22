from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import math


if __name__ == "__main__":
    env = DofbotEnv()
    env.reset()
    Reward = False

    """
    constants here
    """
    GRIPPER_DEFAULT_ANGLE = 20.0 / 180.0 * 3.1415
    GRIPPER_CLOSE_ANGLE = -20.0 / 180.0 * 3.1415

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE

    initial_jointposes = [1.57, 0.0, 1.57, 1.57, 1.57]

    # offset to grasp object
    obj_offset = [-0.023, -0.023, 0.09]
    obj_offset2 = [-0.032, 0.032, 0.13]
    obj_offset3 = [-0.025, 0.025, 0.09]

    block_pos, block_orn = env.get_block_pose()

    start_time = None

    while not Reward:

        """
        #获取物块位姿、目标位置和机械臂位姿，计算机器臂关节和夹爪角度，使得机械臂夹取绿色物块，放置到紫色区域。
        """

        """
        code here
        """

        Reward = env.reward()
