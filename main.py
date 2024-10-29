from dofbot import DofbotEnv
import numpy as np
import time
import copy
from scipy.spatial.transform import Rotation as R
import time
import math

PI = math.pi

if __name__ == "__main__":
    env = DofbotEnv()
    env.reset()
    Reward = False

    """
    constants here
    """
    GRIPPER_DEFAULT_ANGLE = 20.0 / 180.0 * PI
    GRIPPER_CLOSE_ANGLE = -20.0 / 180.0 * PI

    # define state machine
    INITIAL_STATE = 0
    GRASP_STATE = 1
    LIFT_STATE = 2
    PUT_STATE = 3
    MOVE_STATE = 4
    BACK_STATE = 5
    current_state = INITIAL_STATE

    # initial_jointposes = [1.57, 0.0, 1.57, 1.57, 1.57]
    initial_jointposes = [PI / 2, 0.0, PI / 2, PI / 2, PI / 2]

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

        # 如果当前状态为初始状态，设定抓取目标位置和姿态
        if current_state == INITIAL_STATE:
            # 深拷贝block位置并添加偏移量
            target_pos = copy.deepcopy(block_pos)
            target_pos = (
                target_pos[0] + obj_offset[0],
                target_pos[1] + obj_offset[1],
                target_pos[2] + obj_offset[2],
            )
            target_orn = copy.deepcopy(block_orn)
            target_orn = -1 * target_orn

            target_joint_state = env.dofbot_setInverseKine(
                target_pos, target_orn
            )

            # 获取当前机械臂关节状态和夹爪角度
            env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()

            # 检查当前关节状态是否接近目标关节状态
            if (
                np.sum(
                    np.isclose(
                        np.array(current_joint_state),
                        np.array(target_joint_state),
                        atol=1e-2,
                    )
                )
                == 5
            ):
                # 达到目标位置，更新状态为抓取状态
                # print("grab object!!")
                current_state = GRASP_STATE

            # 执行抓取动作，设定夹爪的闭合角度或位置
            print(target_joint_state, current_joint_state)
        elif current_state == GRASP_STATE:
            # 深拷贝block位置并添加偏移量
            target_pos = copy.deepcopy(block_pos)
            target_pos = (
                target_pos[0] + obj_offset[0],
                target_pos[1] + obj_offset[1],
                target_pos[2] + obj_offset[2],
            )
            target_orn = copy.deepcopy(block_orn)
            target_orn = -1 * target_orn

            target_joint_state = env.dofbot_setInverseKine(
                target_pos, target_orn
            )

            # 获取当前机械臂关节状态和夹爪角度
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()

            if start_time == None:
                start_time = time.time()
            current_time = time.time()
            if current_time - start_time > 2.0:
                current_state = LIFT_STATE
                start_time = None

            # 执行抓取动作，设定夹爪的闭合角度或位置
            print(target_joint_state, current_joint_state)
        elif current_state == LIFT_STATE:
            # 深拷贝block位置并添加偏移量
            target_pos = copy.deepcopy(block_pos)
            target_pos = (
                target_pos[0] + obj_offset[0],
                target_pos[1] + obj_offset[1],
                target_pos[2] + obj_offset[2],
            )
            target_orn = copy.deepcopy(block_orn)
            target_orn = -1 * target_orn

            target_joint_state = env.dofbot_setInverseKine(
                target_pos, target_orn
            )

            # 获取当前机械臂关节状态和夹爪角度
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()

            # 检查当前关节状态是否接近目标关节状态
            if (
                np.sum(
                    np.isclose(
                        np.array(current_joint_state),
                        np.array(target_joint_state),
                        atol=1e-2,
                    )
                )
                == 5
            ):
                # 达到目标位置，更新状态为抓取状态
                # print("grab object!!")
                current_state = MOVE_STATE

        elif current_state == MOVE_STATE:
            target_pos = env.get_target_pose()
            target_pos = (
                target_pos[0] + obj_offset2[0],
                target_pos[1] + obj_offset2[1],
                target_pos[2] + obj_offset2[2],
            )
            target_orn = copy.deepcopy(block_orn)
            target_orn = -1 * target_orn

            target_joint_state = env.dofbot_setInverseKine(
                target_pos, target_orn
            )

            # 获取当前机械臂关节状态和夹爪角度
            env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)
            current_joint_state, gripper_angle = env.get_dofbot_jointPoses()

            # 检查当前关节状态是否接近目标关节状态
            if (
                np.sum(
                    np.isclose(
                        np.array(current_joint_state),
                        np.array(target_joint_state),
                        atol=1e-2,
                    )
                )
                == 5
            ):
                # 达到目标位置，更新状态为抓取状态
                # print("grab object!!")
                current_state = BACK_STATE

        elif current_state == BACK_STATE:
            if start_time is None:
                start_time = time.time()
            current_time = time.time()

            target_pos = env.get_target_pose()
            target_pos = (
                target_pos[0] + obj_offset3[0],
                target_pos[1] + obj_offset3[1],
                target_pos[2] + obj_offset3[2],
            )
            target_orn = copy.deepcopy(block_orn)
            target_orn = -1 * target_orn

            target_joint_state = env.dofbot_setInverseKine(
                target_pos, target_orn
            )

            if current_time - start_time > 2.0:
                env.dofbot_control(target_joint_state, GRIPPER_DEFAULT_ANGLE)
            else:
                env.dofbot_control(target_joint_state, GRIPPER_CLOSE_ANGLE)

        print(target_joint_state, current_joint_state)

    Reward = env.reward()
