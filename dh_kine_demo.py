import roboticstoolbox as rtb
import numpy as np
import math

pi = 3.1415926          # 定义pi常数

l1 = 0.1045             # 定义第一连杆长度
l2 = 0.08285            # 定义第三连杆长度
l3 = 0.08285            # 定义第四连杆长度
l4 = 0.12842            # 定义第五连杆长度

# student version
# 用改进DH参数发表示机器人正运动学
# TODO: modify the dh param
dofbot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a = 1, alpha=11, d=111, offset=1111),
        rtb.RevoluteMDH(),
        rtb.RevoluteMDH(),
        rtb.RevoluteMDH(),
        rtb.RevoluteMDH()
    ]
)

# 输出机器人DH参数矩阵
print(dofbot)


'''
Part1 给出一下关节姿态时的机械臂正运动学解，并附上仿真结果
0.(demo) [0., pi/3, pi/4, pi/5, 0.]
1.[pi/2, pi/5, pi/5, pi/5, pi]
2.[pi/3, pi/4, -pi/3, -pi/4, pi/2]
3.[-pi/2, pi/3, -pi/3*2, pi/3, pi/3]
'''

# part1 demo
fkine_input0 = [0., pi/3, pi/4, pi/5, 0.]
fkine_result0 = dofbot.fkine(fkine_input0)
print(fkine_result0)
dofbot.plot(q=fkine_input0, block=True)

# part1-1

# part1-2

# part1-3


'''
Part1 给出一下关节姿态时的机械臂逆运动学解，并附上仿真结果
0.(demo) 
    [
        [-1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., -1., -0.1],
        [0., 0., 0., 1.]
    ]
1.
    [
        [1., 0., 0., 0.1,],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.1],
        [0., 0., 0., 1.]
    ]
2.
    [
        [cos(pi/3), 0., -sin(pi/3), 0.05,],
        [0., 1., 0., 0.03],
        [sin(pi/3), 0., cos(pi/3)., -0.1],
        [0., 0., 0., 1.]
    ]
3.
    [
        [-0.866, -0.25, -0.433, -0.03704,],
        [0.5, -0.433, -0.75, -0.06415],
        [0., -0.866, 0.5, 0.3073],
        [0., 0., 0., 1.]
    ]
'''

#part2 demo
target_pos0 = np.array([
    [-1., 0., 0., 0.1,],
    [0., 1., 0., 0.],
    [0., 0., -1., -0.1],
    [0., 0., 0., 1.]
])
ikine_result0 = dofbot.ik_LM(target_pos0)[0]
print("ikine: ", np.array(ikine_result0))
dofbot.plot(q=ikine_result0, block=True)


#part2-1


#part2-2


#part2-3