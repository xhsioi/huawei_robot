import sys

from global_vars import *
from util import *


# 工作台1 到 工作台2 的价值
# 包含大量需要优化的未定参数
def comput_workstation1_value1(workstation1, workstation2):
    value1 = 3000
    if workstation2.type == 4 or workstation2.type == 5 or workstation2.type == 6:
        if workstation2.material_status != 0:  # 如果是345号工作台，且材料栏差一个就可以加工物品，则价值等于workstation1生产的物品价格加上workstation2的价格一半
            value1 = workstation1.value + workstation2.value / 2
        if workstation2.material_status == 0:
            value1 = workstation1.value + workstation2.value / 4
    elif workstation2.type == 7:
        if workstation2.material_status == 0:
            value1 = workstation1.value + workstation2.value / 6
        if workstation2.material_status // 16 == 6 or workstation2.material_status // 16 == 5 or workstation2.material_status // 16 == 3:
            value1 = workstation1.value + workstation2.value / 2
        if workstation2.material_status // 16 == 1 or workstation2.material_status // 16 == 2 or workstation2.material_status // 16 == 4:
            value1 = workstation1.value + workstation2.value / 4
    elif workstation2.type == 8 or workstation2.type == 9:
        value1 = workstation1.value
    return value1


# xxx
def nextmin_dis_comput(work_type, workstation2):
    flag = 0
    nextmin_dis = math.inf
    best_dis = 0
    value3 = 0
    best_j = 0
    if work_type == 4:
        for j in range(len(workstations)):
            # if workstations_lock[j][4]!=0:
            #     continue
            if ((workstations[j].type == 7 and workstations[j].material_status % 32 == 0) or
                    (workstations[j].type == 9)):
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * comput_workstation1_value1(workstation2, workstations[best_j])

    if work_type == 5:
        for j in range(len(workstations)):
            # if workstations_lock[j][5]!=0:
            #     continue
            if ((workstations[j].type == 7 and workstations[j].material_status // 32 % 2 == 0) or
                    (workstations[j].type == 9)):
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * comput_workstation1_value1(workstation2, workstations[best_j])

    if work_type == 6:
        for j in range(len(workstations)):
            # if workstations_lock[j][6]!=0:
            #     continue
            if ((workstations[j].type == 7 and workstations[j].material_status < 64) or
                    (workstations[j].type == 9)):
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * comput_workstation1_value1(workstation2, workstations[best_j])
    if work_type == 7:
        for j in range(len(workstations)):
            # if fid>9000-700:
            #     continue
            if workstations[j].type == 8 or workstations[j].type == 9:
                flag = 1
                next_dis2 = np.linalg.norm(workstation2.pos - workstations[j].pos)
                if next_dis2 < nextmin_dis:
                    nextmin_dis = next_dis2
                    best_dis = next_dis2
                    best_j = j
        value3 = flag * comput_workstation1_value1(workstation2, workstations[best_j])
    return best_dis, flag, value3


# xxx
def workstations_value(workstation1, workstation2, dis1, dis2):
    value1 = 3000
    # value2=0
    flag1 = 0
    # flag2=0
    nextmin_dis = 0
    nextmin_dis2 = 0
    work_type = workstation2.type
    value2 = 0
    if ((workstation2.product_status == 1) or (workstation2.proc_time < 150 and workstation2.proc_time >= 0)):
        value2 = workstation2.value
        nextmin_dis, flag1, _ = nextmin_dis_comput(work_type, workstation2)
    nextmin_dis2, flag2, value3 = nextmin_dis_comput(work_type, workstation2)

    value1 = comput_workstation1_value1(workstation1, workstation2)

    all_value = (
                    value1 * Vaule_weight1 if flag1 == 0 else value1 * Vaule_weight1 + value2 * flag1 * Vaule_weight1) + value3 * flag2 * Vaule_weight3
    all_dis = dis1 + dis2 + nextmin_dis * flag1 + nextmin_dis2 * flag2 * Vaule_weight3
    return all_dis / all_value


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


# 控制机器人前往一个坐标（包含给单个机器人发送指令）
# 包含大量需要优化的未定参数
# 机器人类, 目标坐标, bot_rad是什么
def control_to_goal(Single_robot, target1, bot_rad):
    direction1 = (target1 - Single_robot.pos) / np.linalg.norm(target1 - Single_robot.pos)  # 计算机器人到目标点的方向向量
    direction_right1 = math.atan2(direction1[1][0], direction1[0][0])  # 计算夹角
    ori = Single_robot.toward
    dis = np.linalg.norm(Single_robot.pos - target1)
    err = direction_right1 - ori
    while err > math.pi:
        err -= 2.0 * math.pi
    while err < -math.pi:
        err += 2.0 * math.pi
    robot_start = np.array([[rob_startpoint[Single_robot.id - 1][0]], [rob_startpoint[Single_robot.id - 1][1]]])
    start_to_tar = np.linalg.norm(robot_start - target1)

    # 防撞墙的路径执行
    # 阈值为 1 的情况
    if (target1[0][0] < 1 or target1[0][0] > 49) or (target1[1][0] < 1 or target1[1][0] > 49):
        if dis < 0.8:
            robot_control("forward", Single_robot.id - 1, 2)
        elif dis < 1.5:
            robot_control("forward", Single_robot.id - 1, 4)
        else:
            if abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:
                r_sei = math.pi / 2 - (err)
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, 6 / r)
    # 阈值为 2 的情况
    elif (robot_start[0][0] < 2 or robot_start[0][0] > 48) or (robot_start[1][0] < 2 or robot_start[1][0] > 48):
        if np.linalg.norm(robot_start - Single_robot.pos) < 1:
            if abs(err) > math.pi / 2:
                robot_control("forward", Single_robot.id - 1, -1)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            elif abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, 4)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
        else:
            if abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:
                r_sei = math.pi / 2 - (err)
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)

                robot_control("rotate", Single_robot.id - 1, 6 / r)
    # 非撞墙情况下
    else:
        # 6 / pi 是最大半径 （最大半径这个变量应该写成大写常数）
        # 起点到终点一半的距离小于最大半径时才能进入此条件
        # 否则 bot 转圈, 无法到达目的地
        if start_to_tar / 2 < 6 / math.pi:

            if abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, start_to_tar / 2.5 * math.pi)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:
                r_sei = math.pi / 2 - (err)
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 5)
                robot_control("rotate", Single_robot.id - 1, 5 / r)
        # 起点到终点一半的距离大于最大半径时才能进入此条件
        # 正常走圆弧能够到达目的地
        else:
            # xxx
            if abs(err) > bot_rad:
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            # xxx
            else:
                r_sei = math.pi / 2 - (err)
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)

                robot_control("rotate", Single_robot.id - 1, 6 / r)


# 查找有产品的工作台
def find_haveproduct_stations(Single_robot):  # 如果机器人没有携带产品，查找机器人可以购买产品的工作台编号

    bots_to_product = np.full((1, len(workstations)),
                              0)  # bots_to_product 1行k列矩阵。 bots_to_product[0][i]代表机器人可以到达编号为i的工作台
    if Single_robot.carried_item_type == 0 and fid < 9000 - 300:
        for j in range(len(workstations)):
            dis = np.linalg.norm(workstations[j].pos - Single_robot.pos)
            if workstations_lock[j][0] != 0:
                continue
            if ((workstations[j].product_status == 1) or (
                    workstations[j].proc_time <= dis / 7 * 50 and workstations[j].proc_time >= 0)):
                bots_to_product[0][j] = 1
    return bots_to_product


# 查找需要原料的工作台
# 有部分未优化参数
def find_needmaterial_stations(bots_to_product, Single_robot):  # 如果机器人购买了编号为i的工作台产品，查找可以收购编号i工作台产品的j号工作台
    product_to_material = np.full((len(workstations), len(workstations)),
                                  0)  # k行k列。product_to_material[i][j]代表机器人可以到i工作台购买物品，然后买到j号工作台

    for i in range(len(workstations)):  # 查找编号为i的工作台可以将产品卖到那些工作台
        if bots_to_product[0][i] == 1:
            work_type = workstations[i].type  # 编号i工作台的产品编号
            if work_type == 1:  # 如果是1号产品，则型号为4,5,9的工作台可以收购该产品
                for j in range(len(workstations)):
                    if workstations_lock[j][1] != 0:  # 如果j号工作台已经成为另一个机器人的目标，则不能成为目标
                        continue
                    if ((workstations[j].type == 5 and workstations[j].material_status % 8 == 0) or
                            (workstations[j].type == 4 and workstations[
                                j].material_status % 4 == 0) or  # 4号工作台可以收购12号产品。如果材料格没有1号产品，则可以成为目标
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 2:
                for j in range(len(workstations)):
                    if workstations_lock[j][2] != 0:
                        continue
                    if ((workstations[j].type == 4 and workstations[j].material_status < 4) or
                            (workstations[j].type == 6 and workstations[j].material_status % 8 == 0) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 3:
                for j in range(len(workstations)):
                    if workstations_lock[j][3] != 0:
                        continue
                    if ((workstations[j].type == 5 and workstations[j].material_status < 8) or
                            (workstations[j].type == 6 and workstations[j].material_status < 8) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 4:
                for j in range(len(workstations)):
                    if workstations_lock[j][4] != 0:
                        continue
                    if ((workstations[j].type == 7 and workstations[j].material_status % 32 == 0) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 5:
                for j in range(len(workstations)):
                    if workstations_lock[j][5] != 0:
                        continue
                    if ((workstations[j].type == 7 and workstations[j].material_status // 32 % 2 == 0) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1

            if work_type == 6:
                for j in range(len(workstations)):
                    if workstations_lock[j][6] != 0:
                        continue
                    if ((workstations[j].type == 7 and workstations[j].material_status < 64) or
                            (workstations[j].type == 9)):
                        product_to_material[i][j] = 1
            if work_type == 7:
                for j in range(len(workstations)):
                    if fid > 9000 - (np.linalg.norm(workstations[i].pos - Single_robot.pos) + np.linalg.norm(
                            workstations[i].pos - workstations[j].pos)) / 5 * 50:
                        continue
                    if workstations[j].type == 8 or workstations[j].type == 9:
                        product_to_material[i][j] = 1
    return product_to_material


# 查找机器人bot从当前坐标先到i工作台购买物品再卖到j工作台的最短距离
# 机器人类, xxx
def compute_shortest_paths(Single_robot, product_to_material):
    bot_path = np.full((1, 2), 1)  # 最优路径
    dis = np.full((len(workstations), len(workstations)), 1.0)
    min_dis = math.inf
    for j in range(len(workstations)):
        for k in range(len(workstations)):
            if product_to_material[j][k] == 1:
                dis1 = np.linalg.norm(workstations[j].pos - Single_robot.pos)
                dis2 = np.linalg.norm(workstations[j].pos - workstations[k].pos)
                real_dis = workstations_value(workstations[j], workstations[k], dis1,
                                              dis2)  # （最终距离=机器人到i的距离+机器人到j的距离）/i工作台到j工作台的价值
                dis[j][k] = real_dis
                if real_dis < min_dis:
                    min_dis = real_dis
                    bot_path[0][0] = j
                    bot_path[0][1] = k
    return bot_path  # 确定最优路径  机器人先到bot_path【0】 再到bot_path【1】


def update_path(Single_robot):  # 更新路径，如果一个机器人完成了运送，确定他的下一个运送路径
    bots_to_product = find_haveproduct_stations(Single_robot)
    product_to_material = find_needmaterial_stations(bots_to_product, Single_robot)
    best_path = compute_shortest_paths(Single_robot, product_to_material)
    return best_path, bots_to_product, product_to_material


def bots_coordinate_motion():  # 机器人运动
    for i in range(len(bots)):  # 分别控制每一个机器人运动      
        if rob_path_information[i][0] != -1:  # 如果第一个目标工作台未完成，则继续走
            goal = rob_path_information[i][0]
            rob_path_information[i][2] = goal
            workstations_lock[goal][0] = bots[i].id
        elif rob_path_information[i][0] == -1 and rob_path_information[i][1] != -1:  # 如果机器人已经购买了商品，则前往计划好的工作台进行销售
            goal = rob_path_information[i][1]
            rob_path_information[i][2] = goal
        else:  # 机器人完成了运送任务，进行路径更新
            goal_station, bots_to_product, product_to_material = update_path(bots[i])  # 计算新路径
            # print("帧数:", fid, "\n", file=sys.stderr)
            # print("igdoal:\n", workstations_lock, "\n", file=sys.stderr)
            # print("rob_path_information:\n", rob_path_information, "\n", file=sys.stderr)
            # print("bots_to_product:\n", bots_to_product, bots[i].id, "\n", file=sys.stderr)
            # print("product_to_material:\n", product_to_material, bots[i].id, "\n", file=sys.stderr)
            # print("goal_station:", goal_station, "\n", file=sys.stderr)

            if goal_station[0][0] == 1 and goal_station[0][1] == 1:  # 如果工作台个数太少，可能有机器人闲置，则其目标会是0号工作台
                # 如果是这样的话，清空其任务，每次循环更新其路径，直到找到新路径
                rob_path_information[i][0] = -1
                rob_path_information[i][0] = -1
                robot_control("forward", i, 6)
                robot_control("rotate", i, 1)
                continue
            goal = goal_station[0][0]  # goal为机器人当前的目标，如果是刚更新的路径，则当前目标一定是第一个工作台
            if (workstations[goal].type < 7):  # 如果机器人将type产品送到对应工作台销售，则其他机器人不能前往该工作台进行销售
                workstations_lock[goal_station[0][1]][workstations[goal].type] = bots[i].id
            rob_path_information[i][2] = goal  # rob_path_information第三个位置记录机器人当前目标
            workstations_lock[goal_station[0][0]][0] = bots[i].id  # 如果机器人正在前往goal工作台购买商品，则其他机器人不能前往该工作台购买
            rob_path_information[i][0] = goal_station[0][0]  # rob_path_information第1个位置记录最优路径的第一个工作台编号
            rob_path_information[i][1] = goal_station[0][1]  # rob_path_information第2个位置记录最优路径的第二个工作台编号
        x = workstations[goal].x  # 目标工作台的坐标
        y = workstations[goal].y
        target = np.array([[x], [y]])
        control_to_goal(bots[i], target, math.pi / 6)  # 控制机器人前往目标


# 当买和卖的目标和机器人所在工作台匹配, 且机器人在某一个工作台时 => 进行购买 出售操作
def bots_operator():
    for robot in bots:
        if ((robot.time_value_coe < 0.92 and robot.time_value_coe != 0) or  # 如果时间太久，直接去卖
                (robot.carried_item_type < 0.92 and robot.carried_item_type != 0)):
            rob_path_information[robot.id - 1][0] = -1

        if robot.at_ws_id == rob_path_information[robot.id - 1][2]:  # 当前位置是目标位置进行卖买操作
            if rob_path_information[robot.id - 1][0] == rob_path_information[robot.id - 1][2] and workstations[
                robot.at_ws_id].product_status == 1 and robot.at_ws_id != -1:
                robot_control("buy", robot.id - 1)
                rob_path_information[robot.id - 1][0] = -1
                rob_path_information[robot.id - 1][3] = robot.x
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y
                rob_path_information[robot.id - 1][4] = robot.y
                workstations_lock[rob_path_information[robot.id - 1][2]][0] = 0  # 买完之后，其他机器人可以选择本工作台
            if rob_path_information[robot.id - 1][2] == rob_path_information[robot.id - 1][1] and robot.at_ws_id != -1:
                robot_control("sell", robot.id - 1)
                rob_path_information[robot.id - 1][1] = -1
                workstations_lock[rob_path_information[robot.id - 1][2]][robot.carried_item_type] = 0
                rob_path_information[robot.id - 1][3] = robot.x
                rob_path_information[robot.id - 1][4] = robot.y
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y


# 动态更新工作台价值
# 包含大量需要优化的未定参数
def update_workstation_value():
    for i in range(len(workstations)):
        if game_map == 4:
            if workstations[i].type == 4:
                workstations[i].value = (22500 - 15400) * 3
        if workstations[i].type == 7:
            if workstations[i].material_status == 2 ** 4 + 2 ** 5:
                for j in range(len(workstations)):
                    if workstations[i].type == 6:
                        workstations[i].value = (27500 - 19200) * 2
            elif workstations[i].material_status == 2 ** 4 + 2 ** 6:
                for j in range(len(workstations)):
                    if workstations[i].type == 5:
                        workstations[i].value = (25000 - 17200) * 2
            elif workstations[i].material_status == 2 ** 5 + 2 ** 6:
                for j in range(len(workstations)):
                    if workstations[i].type == 4:
                        workstations[i].value = (22500 - 15400) * 2

            elif workstations[i].material_status == 2 ** 4:
                for j in range(len(workstations)):
                    if workstations[i].type == 5:
                        workstations[i].value = (25000 - 17200) * 1.5
                    if workstations[i].type == 4:
                        workstations[i].value = (27500 - 19200) * 1.5

            elif workstations[i].material_status == 2 ** 5:
                for j in range(len(workstations)):
                    if workstations[i].type == 6:
                        workstations[i].value = (27500 - 19200) * 1.5
                    if workstations[i].type == 4:
                        workstations[i].value = (22500 - 15400) * 1.5
            elif workstations[i].material_status == 2 ** 6:
                for j in range(len(workstations)):
                    if workstations[i].type == 5:
                        workstations[i].value = (25000 - 17200) * 1.5
                    if workstations[i].type == 4:
                        workstations[i].value = (22500 - 15400) * 1.5


if __name__ == '__main__':
    # 获取地图(100x100数组)
    game_map_array = init()
    ws_config = get_ws_config(game_map_array)
    while True:
        # 获取信息
        fid, money, workstations, bots = get_input_var()
        # 输出 fid
        sys.stdout.write('%d\n' % fid)

        # 初始化相关全局变量
        if fid == 1:
            # 初始化工作台锁
            workstations_lock = np.full((len(workstations), 8), 0)
            # 初始化机器人路径信息
            rob_path_information = np.full((len(bots), 5), -1)
            for i in range(len(bots)):
                rob_path_information[i][3] = bots[i].x
                rob_path_information[i][4] = bots[i].y

            # 初始化机器人起始位置
            rob_startpoint = np.full((len(bots), 2), 0.0)
            for i in range(len(bots)):
                rob_startpoint[i][0] = bots[i].x
                rob_startpoint[i][1] = bots[i].y

            # 判断是哪个地图
            game_map = 1
            if ws_config[0] == 43:
                game_map = 1
            elif ws_config[0] == 25:
                game_map = 2
            elif ws_config[0] == 50:
                game_map = 3
            elif ws_config[0] == 18:
                game_map = 4

            # 根据地图调节参数
            if game_map == 1:
                Vaule_weight1 = 1.2
                Vaule_weight2 = 0.9
                Vaule_weight3 = 0.1
            elif game_map == 2:
                # if len(sys.argv) < 4:
                Vaule_weight1 = 0.5
                Vaule_weight2 = 0
                Vaule_weight3 = 2
                # else:
                #     Vaule_weight1 = float(sys.argv[1])
                #     Vaule_weight2 = float(sys.argv[2])
                #     Vaule_weight3 = float(sys.argv[3])
            elif game_map == 3:
                Vaule_weight1 = 1.1
                Vaule_weight2 = 0.5
                Vaule_weight3 = 2
            elif game_map == 4:
                Vaule_weight1 = 0.5
                Vaule_weight2 = 0.0
                Vaule_weight3 = 1

        # 每一帧
        update_workstation_value()  # 更新价值
        bots_coordinate_motion()  # 移动
        bots_operator()  # 购买、销售、销毁

        # 结束
        sys.stdout.write('OK\n')
        sys.stdout.flush()

        if fid == 9000:
            exit()
