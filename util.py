from model import *

def init():
    # 读取地图
    game_map = []
    for i in range(100):
        line = sys.stdin.readline()
        game_map.append(list(line))

    # 直到 OK
    while input() != "OK":
        pass

    # 结束初始化
    sys.stdout.write('OK\n')
    sys.stdout.flush()

    return game_map


def get_ws_config(game_map):
    ws_config = np.full(10, 0)
    for line in game_map:
        for pos in line:
            if pos != '.' and pos != 'A' and pos != '\n':
                ws_config[0] += 1
                ws_config[int(pos)] += 1

    return ws_config


def robot_control(order, _id, value=None):  # 定义了一个名为robot_control的函数，输入指令，机器人id ，参数即可操控机器人
    if value is not None:
        sys.stdout.write("{} {} {}\n".format(order, _id, value))
    else:
        sys.stdout.write("{} {}\n".format(order, _id))


# 获取输入的变量
def get_input_var():
    # 帧数, 当前金钱
    fid, money = map(int, sys.stdin.readline().split())

    # 工作台数
    num_workstations = int(sys.stdin.readline())

    # 紧接着 K 行数据，每一行表示一个工作台，分别由如下所示的数据构成，共计 6 个数字：
    # 读取每个工作台的数据
    workstations = []
    for i in range(num_workstations):
        data = sys.stdin.readline().split()
        type = int(data[0])
        x = float(data[1])
        y = float(data[2])
        proc_time = int(data[3])
        material_status = int(data[4])
        product_status = int(data[5])
        workstation = Workstation(type, x, y, proc_time, material_status, product_status)
        workstations.append(workstation)

    # 接下来的 4 行数据，每一行表示一个机器人，分别由如下表格中所示的数据构成，每行 10 个数字。
    bots = []
    for i in range(4):
        data = sys.stdin.readline().split()
        at_ws_id = int(data[0])
        carried_item_type = int(data[1])
        time_value_coe = float(data[2])
        collision_value_coe = float(data[3])
        angular_v = float(data[4])
        linear_v_x = float(data[5])
        linear_v_y = float(data[6])
        toward = float(data[7])
        x = float(data[8])
        y = float(data[9])
        bot = Bot(
            i + 1,
            at_ws_id,
            carried_item_type,
            time_value_coe,
            collision_value_coe,
            angular_v,
            linear_v_x,
            linear_v_y,
            toward,
            x,
            y
        )
        bots.append(bot)

    while input() != "OK":
        pass

    return fid, money, workstations, bots
