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


# 计算将workstation2的work_type产品卖到需要的工作台的最短距离
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


# 计算机器人从workstations1到workstations2的路径开销，dis1表示机器人到workstations1的距离，dis2表示workstations1到workstation2的距离
def workstations_value(workstation1, workstation2, dis1, dis2):
    value1 = 3000
    # value2=0
    flag1 = 0
    # flag2=0
    nextmin_dis = 0
    nextmin_dis2 = 0
    work_type = workstation2.type
    value2 = 0
    if ((workstation2.product_status == 1) or (workstation2.proc_time < 150 and workstation2.proc_time >= 0)):#如果workstations2有产品，或者产品快生产出来了
        value2 = workstation2.value  #那么机器从workstations1买产品卖到workstations2，又可以立马从workstations2买产品卖到其他地方
        nextmin_dis, flag1, _ = nextmin_dis_comput(work_type, workstation2)  #计算workstations2的产品可以卖到那些工作台，并计算最短的路径
    nextmin_dis2, flag2, value3 = nextmin_dis_comput(work_type, workstation2)  #如果workstations的产品是其他工作台需要的，应该尽快让workstation2产出产品

    value1 = comput_workstation1_value1(workstation1, workstation2)  

    all_value = (
                    value1 * Vaule_weight1 if flag1 == 0 else value1 * Vaule_weight1 + value2 * flag1 * Vaule_weight1) + value3 * flag2 * Vaule_weight3
    all_dis = dis1 + dis2 + nextmin_dis * flag1 + nextmin_dis2 * flag2 * Vaule_weight3  #allvalue包括了三部分。value1 value2 value3
    #value1表示workstations1的产品的一个价值
    #value2表示机器人将产品卖到workstation2时，正好买workstation2的产品，省去了还需要前往另一个工作台卖产品的距离
    #value3表示workstations2的产品是其他工作台急需的，比如7号工作台就差4了，那4就是急需的。
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
    direction_right1 = math.atan2(direction1[1][0], direction1[0][0])  # 机器人到目标点的夹角
    ori = Single_robot.toward  #机器人当前朝向
    dis = np.linalg.norm(Single_robot.pos - target1)   #计算机器人到目标点的距离
    err = direction_right1 - ori   #计算机器人到目标点的夹角和机器人当前朝向的误差
    while err > math.pi:
        err -= 2.0 * math.pi
    while err < -math.pi:
        err += 2.0 * math.pi
    robot_start = np.array([[rob_startpoint[Single_robot.id - 1][0]], [rob_startpoint[Single_robot.id - 1][1]]])  #记录机器人的起点位置（注意这里不是机器人的实时位置，起点位置表示机器人完成一个目标的那一刻的位置），如果机器人的起点到目标点距离太小，那么速度应该小点不然旋转半径太大一直转圈
    start_to_tar = np.linalg.norm(robot_start - target1)  #计算机器人起点到目标点的距离

    # 防撞墙的路径执行
   
    if (target1[0][0] < 1 or target1[0][0] > 49) or (target1[1][0] < 1 or target1[1][0] > 49):  #如果目标点工作台距离墙的距离小于1
        if dis < 0.8:    #如果机器人当前位置到目标位置距离小于0.8米
            robot_control("forward", Single_robot.id - 1, 2)   机器人速度设置为2
        elif dis < 1.5:  #如果小于2.5  速度为4
            robot_control("forward", Single_robot.id - 1, 4)   
        else:    #如果机器人距离目标点还远，就全力前进
            if abs(err) > bot_rad:   #如果err大于一个阈值，这里设置的是pi/6
                robot_control("forward", Single_robot.id - 1, 6)   #机器人做半径为6/pi的圆周运动，直到夹角小于阈值
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            else:   #如果夹角小于阈值，机器人做一个圆弧运动。但是半径很大，所以和直线差不多
                r_sei = math.pi / 2 - (err)
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, 6 / r)

    elif (robot_start[0][0] < 2 or robot_start[0][0] > 48) or (robot_start[1][0] < 2 or robot_start[1][0] > 48):  #如果机器人起点的位置距离墙太近，这里注意是起点位置。上面是终点位置距离墙过近
        if np.linalg.norm(robot_start - Single_robot.pos) < 1:  #起点表示机器人在起点位置完成了一个任务，该前往其他目标点了。如果机器的位置距离起点位置小于1
            if abs(err) > math.pi / 2:   #如果机器人朝向和目标点的方向差大于90度
                robot_control("forward", Single_robot.id - 1, -1)  #机器人先后退，因为此时应该是机器人正朝向墙，如果前进的话会撞墙
                robot_control("rotate", Single_robot.id - 1, err * 1000000)  #最大旋转速度旋转
            elif abs(err) > bot_rad:    #如果差在30度到60度之间，就可以前进了，因为此时已经不面向墙了
                robot_control("forward", Single_robot.id - 1, 4)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
        else:  #如果机器人已经远离墙了，就可以全力前进了
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
        if start_to_tar / 2 < 6 / math.pi:   #如果机器人的起点到终点的距离太小，应该吧速度调小，不然半径太大，永远也到不了，并且一直做圆周运动

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
            if abs(err) > bot_rad:#第一段圆弧，半径为6/pi
                robot_control("forward", Single_robot.id - 1, 6)
                robot_control("rotate", Single_robot.id - 1, err * 1000000)
            # xxx
            else:  #第二段圆弧，半径是算出来的
                r_sei = math.pi / 2 - (err)
                r = dis / 2 / math.cos(r_sei)
                robot_control("forward", Single_robot.id - 1, 6)

                robot_control("rotate", Single_robot.id - 1, 6 / r)


# 查找有产品的工作台
def find_haveproduct_stations(Single_robot):  # 如果机器人没有携带产品，查找机器人可以购买产品的工作台编号

    bots_to_product = np.full((1, len(workstations)),
                              0)  # bots_to_product 1行k列矩阵。 bots_to_product[0][i]代表机器人可以到达编号为i的工作台
    if Single_robot.carried_item_type == 0 and fid < 9000 - 300:  #如果机器人没有携带产品
        for j in range(len(workstations)):
            dis = np.linalg.norm(workstations[j].pos - Single_robot.pos)
            if workstations_lock[j][0] != 0:
                continue
            if ((workstations[j].product_status == 1) or (     
                    workstations[j].proc_time <= dis / 7 * 50 and workstations[j].proc_time >= 0)):  #如果j工作台有产品或者机器人到j工作台之前，j工作台可以将产品生产出来
                bots_to_product[0][j] = 1
    return bots_to_product


# 查找需要原料的工作台
# 有部分未优化参数
def find_needmaterial_stations(bots_to_product, Single_robot):  # 如果机器人购买了编号为i的工作台产品，查找可以收购编号i工作台产品的j号工作台
    product_to_material = np.full((len(workstations), len(workstations)),
                                  0)  # k行k列。product_to_material[i][j]代表机器人可以到i工作台购买物品，然后买到j号工作台

    for i in range(len(workstations)):  # 查找编号为i的工作台可以将产品卖到那些工作台
        if bots_to_product[0][i] == 1:     #如果i工作台可达，下面查找那些工作台可以收购i工作台的产品
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

def compute_shortest_paths(Single_robot, product_to_material):
    bot_path = np.full((1, 2), 1)  # 最优路径
    dis = np.full((len(workstations), len(workstations)), 1.0)  #距离矩阵
    min_dis = math.inf  
    for j in range(len(workstations)):
        for k in range(len(workstations)):
            if product_to_material[j][k] == 1: #如果可以先到j工作台买产品，在买到k工作台，那么计算路径开销
                dis1 = np.linalg.norm(workstations[j].pos - Single_robot.pos)  #机器人到j工作台的距离
                dis2 = np.linalg.norm(workstations[j].pos - workstations[k].pos)  #j工作台到k工作台的距离
                real_dis = workstations_value(workstations[j], workstations[k], dis1, 
                                              dis2)  # （最终距离=机器人到i的距离+机器人到j的距离）/i工作台到j工作台的价值
                dis[j][k] = real_dis 
                if real_dis < min_dis:   #更新开销最短的路径
                    min_dis = real_dis
                    bot_path[0][0] = j
                    bot_path[0][1] = k
    return bot_path  # 确定最优路径  机器人先到bot_path【0】 再到bot_path【1】


def update_path(Single_robot):  # 更新路径，如果一个机器人完成了运送，确定他的下一个运送路径
    bots_to_product = find_haveproduct_stations(Single_robot)    #返回Single_robot可以前往那些工作台进行买商品
    product_to_material = find_needmaterial_stations(bots_to_product, Single_robot)  #返回机器人可以从i工作台买商品，再买到j工作台
    best_path = compute_shortest_paths(Single_robot, product_to_material)  #计算所有路径的最小开销，即最优路径
    return best_path, bots_to_product, product_to_material


def bots_coordinate_motion():  # 机器人运动
    for i in range(len(bots)):  # 分别控制每一个机器人运动      
        if rob_path_information[i][0] != -1:  # 如果第一个目标工作台未完成，则继续走
            goal = rob_path_information[i][0]
            rob_path_information[i][2] = goal
            workstations_lock[goal][0] = bots[i].id
        elif rob_path_information[i][0] == -1 and rob_path_information[i][1] != -1:  # 如果机器人已经购买了商品，则前往计划好的工作台进行销售（第一个目标已经完成，前往第二个目标）
            goal = rob_path_information[i][1]  #此时目标为第二个工作台
            rob_path_information[i][2] = goal
        else:  # 机器人完成了运送任务，进行路径更新（第一个第二个目标都完成了，应该更新路径）
            goal_station, bots_to_product, product_to_material = update_path(bots[i])  # 计算新路径
            # print("帧数:", fid, "\n", file=sys.stderr)
            # print("igdoal:\n", workstations_lock, "\n", file=sys.stderr)
            # print("rob_path_information:\n", rob_path_information, "\n", file=sys.stderr)
            # print("bots_to_product:\n", bots_to_product, bots[i].id, "\n", file=sys.stderr)
            # print("product_to_material:\n", product_to_material, bots[i].id, "\n", file=sys.stderr)
            # print("goal_station:", goal_station, "\n", file=sys.stderr)

            if goal_station[0][0] == 1 and goal_station[0][1] == 1:  # 如果工作台个数太少，可能有机器人闲置，则其目标会是0号工作台
                # 如果是这样的话，清空其任务，每次循环更新其路径，控制机器人转圈，直到找到新路径
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
                robot.at_ws_id].product_status == 1 and robot.at_ws_id != -1:  #进行买商品
                robot_control("buy", robot.id - 1)
                rob_path_information[robot.id - 1][0] = -1     #买完商品，该矩阵第一个值赋值为-1，表示第一个目标已经完成
                rob_path_information[robot.id - 1][3] = robot.x   #买完商品，确定机器人此时的起点。方便判断起点和目标点的距离
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y
                rob_path_information[robot.id - 1][4] = robot.y
                workstations_lock[rob_path_information[robot.id - 1][2]][0] = 0  # 买完之后，其他机器人可以选择本工作台
            if rob_path_information[robot.id - 1][2] == rob_path_information[robot.id - 1][1] and robot.at_ws_id != -1: #进行卖操作
                robot_control("sell", robot.id - 1)
                rob_path_information[robot.id - 1][1] = -1    #卖完商品，该矩阵的第二个值赋值为-1，表示第二个目标已经完成
                workstations_lock[rob_path_information[robot.id - 1][2]][robot.carried_item_type] = 0  #卖完了商品，锁应该解开。
                rob_path_information[robot.id - 1][3] = robot.x
                rob_path_information[robot.id - 1][4] = robot.y  #更新机器人的当前位置，即前往下一个目标点的起点
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y


# 动态更新工作台价值
# 包含大量需要优化的未定参数
# 当工作7只差456产品其中一个，就将该产品升值2倍。只差456其中两个，将两个产品小额度升值1.5倍
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
        update_workstation_value()  # 更新价值，每一个种路径都有价值，此价值是动态的，需要更新
        bots_coordinate_motion()  # 控制多个机器人运动，包括目标选取、路径选择
        bots_operator()  # 控制单个机器人进行购买、销售、销毁操作，并更新一些全局变量

        # 结束
        sys.stdout.write('OK\n')
        sys.stdout.flush()

        if fid == 9000:
            exit()
