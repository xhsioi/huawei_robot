# 全局变量
import numpy as np

fid = 0
Workstations = []
bots = []

Vaule_weight1 = 0.0
Vaule_weight2 = 0.0
Vaule_weight3 = 0.0

workstations_lock = np.full((1, 1), 0)
rob_path_information = np.full((1, 1), 0)
rob_startpoint = np.full((1, 1), 0)