import numpy as np
import math


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
def nextmin_dis_comput(workstations, work_type, workstation2):
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
def workstations_value(workstations, Vaule_weight1, Vaule_weight2, Vaule_weight3, workstation1, workstation2, dis1, dis2):
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
        nextmin_dis, flag1, _ = nextmin_dis_comput(workstations, work_type, workstation2)  #计算workstations2的产品可以卖到那些工作台，并计算最短的路径
    nextmin_dis2, flag2, value3 = nextmin_dis_comput(workstations, work_type, workstation2)  #如果workstations的产品是其他工作台需要的，应该尽快让workstation2产出产品

    value1 = comput_workstation1_value1(workstation1, workstation2)

    all_value = (
                    value1 * Vaule_weight1 if flag1 == 0 else value1 * Vaule_weight1 + value2 * flag1 * Vaule_weight1) + value3 * flag2 * Vaule_weight3
    all_dis = dis1 + dis2 + nextmin_dis * flag1 + nextmin_dis2 * flag2 * Vaule_weight3  #allvalue包括了三部分。value1 value2 value3
    #value1表示workstations1的产品的一个价值
    #value2表示机器人将产品卖到workstation2时，正好买workstation2的产品，省去了还需要前往另一个工作台卖产品的距离
    #value3表示workstations2的产品是其他工作台急需的，比如7号工作台就差4了，那4就是急需的。
    return all_dis / all_value


# 查找有产品的工作台
def find_haveproduct_stations(fid, workstations_lock, workstations, Single_robot):  # 如果机器人没有携带产品，查找机器人可以购买产品的工作台编号

    bots_to_product = np.full((1, len(workstations)),
                              0)  # bots_to_product 1行k列矩阵。 bots_to_product[0][i]代表机器人可以到达编号为i的工作台
    if Single_robot.carried_item_type == 0 and fid < 9000 - 300:  #如果机器人没有携带产品
        for j in range(len(workstations)):
            dis = np.linalg.norm(workstations[j].pos - Single_robot.pos)
            if workstations_lock[j][0] != 0:
                continue
            if ((workstations[j].product_status == 1) or (
                    dis / 7 * 50 >= workstations[j].proc_time >= 0)):  #如果j工作台有产品或者机器人到j工作台之前，j工作台可以将产品生产出来
                bots_to_product[0][j] = 1
    return bots_to_product


# 查找需要原料的工作台
# 有部分未优化参数
def find_needmaterial_stations(fid,workstations_lock,workstations,bots_to_product, Single_robot):  # 如果机器人购买了编号为i的工作台产品，查找可以收购编号i工作台产品的j号工作台
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

def compute_shortest_paths(Vaule_weight1, Vaule_weight2, Vaule_weight3, workstations, Single_robot, product_to_material):
    bot_path = np.full((1, 2), 1)  # 最优路径
    dis = np.full((len(workstations), len(workstations)), 1.0)  #距离矩阵
    min_dis = math.inf
    for j in range(len(workstations)):
        for k in range(len(workstations)):
            if product_to_material[j][k] == 1: #如果可以先到j工作台买产品，在买到k工作台，那么计算路径开销
                dis1 = np.linalg.norm(workstations[j].pos - Single_robot.pos)  #机器人到j工作台的距离
                dis2 = np.linalg.norm(workstations[j].pos - workstations[k].pos)  #j工作台到k工作台的距离
                real_dis = workstations_value(workstations, Vaule_weight1, Vaule_weight2, Vaule_weight3, workstations[j], workstations[k], dis1,
                                              dis2)  # （最终距离=机器人到i的距离+机器人到j的距离）/i工作台到j工作台的价值
                dis[j][k] = real_dis
                if real_dis < min_dis:   #更新开销最短的路径
                    min_dis = real_dis
                    bot_path[0][0] = j
                    bot_path[0][1] = k
    return bot_path  # 确定最优路径  机器人先到bot_path【0】 再到bot_path【1】


def update_path(fid, Vaule_weight1, Vaule_weight2, Vaule_weight3, workstations_lock, workstations, Single_robot):  # 更新路径，如果一个机器人完成了运送，确定他的下一个运送路径
    bots_to_product = find_haveproduct_stations(fid, workstations_lock, workstations, Single_robot)    #返回Single_robot可以前往那些工作台进行买商品
    product_to_material = find_needmaterial_stations(fid, workstations_lock, workstations, bots_to_product, Single_robot)  #返回机器人可以从i工作台买商品，再买到j工作台
    best_path = compute_shortest_paths(Vaule_weight1, Vaule_weight2, Vaule_weight3, workstations, Single_robot, product_to_material)  #计算所有路径的最小开销，即最优路径
    return best_path, bots_to_product, product_to_material
