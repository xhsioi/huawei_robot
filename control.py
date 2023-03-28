import numpy as np
import sys
import math


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


# 控制机器人前往一个坐标（包含给单个机器人发送指令）
# 包含大量需要优化的未定参数
# 机器人类, 目标坐标, bot_rad是什么
def control_to_goal(Single_robot, target1, bot_rad, rob_startpoint):
    direction1 = (target1 - Single_robot.pos) / np.linalg.norm(target1 - Single_robot.pos)  # 计算机器人到目标点的方向向量
    direction_right1 = math.atan2(direction1[1][0], direction1[0][0])  # 机器人到目标点的夹角
    ori = Single_robot.toward  # 机器人当前朝向
    dis = np.linalg.norm(Single_robot.pos - target1)  # 计算机器人到目标点的距离
    err = direction_right1 - ori  # 计算机器人到目标点的夹角和机器人当前朝向的误差
    while err > math.pi:
        err -= 2.0 * math.pi
    while err < -math.pi:
        err += 2.0 * math.pi
    robot_start = np.array([[rob_startpoint[Single_robot.id - 1][0]], [rob_startpoint[Single_robot.id - 1][1]]])  # 记录机器人的起点位置（注意这里不是机器人的实时位置，起点位置表示机器人完成一个目标的那一刻的位置），如果机器人的起点到目标点距离太小，那么速度应该小点不然旋转半径太大一直转圈
    start_to_tar = np.linalg.norm(robot_start - target1)  # 计算机器人起点到目标点的距离

    # 防撞墙的路径执行

    if (target1[0][0] < 1 or target1[0][0] > 49) or (target1[1][0] < 1 or target1[1][0] > 49):  # 如果目标点工作台距离墙的距离小于1
        if dis < 0.8:  # 如果机器人当前位置到目标位置距离小于0.8米
            robot_control("forward", Single_robot.id - 1, 2)
        elif dis < 1.5:  # 如果小于2.5  速度为4
            robot_control("forward", Single_robot.id - 1, 4)
        else:  # 如果机器人距离目标点还远，就全力前进
            if abs(err) > bot_rad:  # 如果err大于一个阈值，这里设置的是pi/6
                robot_control("forward", Single_robot.id - 1, 6)  # 机器人做半径为6/pi的圆周运动，直到夹角小于阈值
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:  # 如果夹角小于阈值，机器人做一个圆弧运动。但是半径很大，所以和直线差不多
                r_sei = math.pi / 2 - err
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, 6 / r)

    elif (robot_start[0][0] < 2 or robot_start[0][0] > 48) or (
            robot_start[1][0] < 2 or robot_start[1][0] > 48):  # 如果机器人起点的位置距离墙太近，这里注意是起点位置。上面是终点位置距离墙过近
        if np.linalg.norm(robot_start - Single_robot.pos) < 1:  # 起点表示机器人在起点位置完成了一个任务，该前往其他目标点了。如果机器的位置距离起点位置小于1
            if abs(err) > math.pi / 2:  # 如果机器人朝向和目标点的方向差大于90度
                robot_control("forward", Single_robot.id - 1, -1)  # 机器人先后退，因为此时应该是机器人正朝向墙，如果前进的话会撞墙
                robot_control("rotate", Single_robot.id - 1, err * 1000000)  # 最大旋转速度旋转
            elif abs(err) > bot_rad:  # 如果差在30度到60度之间，就可以前进了，因为此时已经不面向墙了
                robot_control("forward", Single_robot.id - 1, 4)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
        else:  # 如果机器人已经远离墙了，就可以全力前进了
            if abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:
                r_sei = math.pi / 2 - err
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)

                robot_control("rotate", Single_robot.id - 1, 6 / r)
    # 非撞墙情况下
    else:
        # 6 / pi 是最大半径 （最大半径这个变量应该写成大写常数）
        # 起点到终点一半的距离小于最大半径时才能进入此条件
        # 否则 bot 转圈, 无法到达目的地
        if start_to_tar / 2 < 6 / math.pi:  # 如果机器人的起点到终点的距离太小，应该吧速度调小，不然半径太大，永远也到不了，并且一直做圆周运动

            if abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, start_to_tar / 2.5 * math.pi)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:
                r_sei = math.pi / 2 - err
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 5)
                robot_control("rotate", Single_robot.id - 1, 5 / r)
        # 起点到终点一半的距离大于最大半径时才能进入此条件
        # 正常走圆弧能够到达目的地
        else:
            # xxx
            if abs(err) > bot_rad:  # 第一段圆弧，半径为6/pi
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            # xxx
            else:  # 第二段圆弧，半径是算出来的
                r_sei = math.pi / 2 - err
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)

                robot_control("rotate", Single_robot.id - 1, 6 / r)
