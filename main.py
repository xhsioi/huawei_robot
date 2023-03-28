from util import *
from update_path_information import *
from control import *


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


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


def bots_coordinate_motion():  # 机器人运动
    for i in range(len(bots)):  # 分别控制每一个机器人运动      
        if rob_path_information[i][0] != -1:  # 如果第一个目标工作台未完成，则继续走
            goal = rob_path_information[i][0]
            rob_path_information[i][2] = goal
            workstations_lock[goal][0] = bots[i].id
        elif rob_path_information[i][0] == -1 and rob_path_information[i][
            1] != -1:  # 如果机器人已经购买了商品，则前往计划好的工作台进行销售（第一个目标已经完成，前往第二个目标）
            goal = rob_path_information[i][1]  # 此时目标为第二个工作台
            rob_path_information[i][2] = goal
        else:  # 机器人完成了运送任务，进行路径更新（第一个第二个目标都完成了，应该更新路径）
            goal_station, bots_to_product, product_to_material = update_path(fid, Vaule_weight1, Vaule_weight2, Vaule_weight3, workstations_lock, workstations, bots[i])  # 计算新路径
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
            if workstations[goal].type < 7:  # 如果机器人将type产品送到对应工作台销售，则其他机器人不能前往该工作台进行销售
                workstations_lock[goal_station[0][1]][workstations[goal].type] = bots[i].id
            rob_path_information[i][2] = goal  # rob_path_information第三个位置记录机器人当前目标
            workstations_lock[goal_station[0][0]][0] = bots[i].id  # 如果机器人正在前往goal工作台购买商品，则其他机器人不能前往该工作台购买
            rob_path_information[i][0] = goal_station[0][0]  # rob_path_information第1个位置记录最优路径的第一个工作台编号
            rob_path_information[i][1] = goal_station[0][1]  # rob_path_information第2个位置记录最优路径的第二个工作台编号
        x = workstations[goal].x  # 目标工作台的坐标
        y = workstations[goal].y
        target = np.array([[x], [y]])
        control_to_goal(bots[i], target, math.pi / 6, rob_startpoint)  # 控制机器人前往目标


# 当买和卖的目标和机器人所在工作台匹配, 且机器人在某一个工作台时 => 进行购买 出售操作
def bots_operator():
    for robot in bots:
        if ((robot.time_value_coe < 0.92 and robot.time_value_coe != 0) or  # 如果时间太久，直接去卖
                (robot.carried_item_type < 0.92 and robot.carried_item_type != 0)):
            rob_path_information[robot.id - 1][0] = -1

        if robot.at_ws_id == rob_path_information[robot.id - 1][2]:  # 当前位置是目标位置进行卖买操作
            if rob_path_information[robot.id - 1][0] == rob_path_information[robot.id - 1][2] and workstations[
                robot.at_ws_id].product_status == 1 and robot.at_ws_id != -1:  # 进行买商品
                robot_control("buy", robot.id - 1)
                rob_path_information[robot.id - 1][0] = -1  # 买完商品，该矩阵第一个值赋值为-1，表示第一个目标已经完成
                rob_path_information[robot.id - 1][3] = robot.x  # 买完商品，确定机器人此时的起点。方便判断起点和目标点的距离
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y
                rob_path_information[robot.id - 1][4] = robot.y
                workstations_lock[rob_path_information[robot.id - 1][2]][0] = 0  # 买完之后，其他机器人可以选择本工作台
            if rob_path_information[robot.id - 1][2] == rob_path_information[robot.id - 1][
                1] and robot.at_ws_id != -1:  # 进行卖操作
                robot_control("sell", robot.id - 1)
                rob_path_information[robot.id - 1][1] = -1  # 卖完商品，该矩阵的第二个值赋值为-1，表示第二个目标已经完成
                workstations_lock[rob_path_information[robot.id - 1][2]][robot.carried_item_type] = 0  # 卖完了商品，锁应该解开。
                rob_path_information[robot.id - 1][3] = robot.x
                rob_path_information[robot.id - 1][4] = robot.y  # 更新机器人的当前位置，即前往下一个目标点的起点
                rob_startpoint[robot.id - 1][0] = robot.x
                rob_startpoint[robot.id - 1][1] = robot.y


if __name__ == '__main__':
    # 获取地图(100x100数组)
    game_map_array = init()
    ws_config = get_ws_config(game_map_array)
    Vaule_weight1 = 0.0
    Vaule_weight2 = 0.0
    Vaule_weight3 = 0.0

    workstations_lock = np.full((1, 1), 0)
    rob_path_information = np.full((1, 1), 0)  # 记录进货、出货的两个工作站的状态（-1：已完成操作；其他：未完成此操作）
    rob_startpoint = np.full((1, 1), 0)
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
