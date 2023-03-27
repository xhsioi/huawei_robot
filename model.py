import numpy as np
class Bot:

    def __init__(self,id, at_ws_id, carried_item_type, time_value_coe, collision_value_coe, angular_v, linear_v_x, linear_v_y, toward, x, y):
        self.id=id
        self.at_ws_id = at_ws_id
        self.carried_item_type = carried_item_type
        self.time_value_coe = time_value_coe
        self.collision_value_coe = collision_value_coe
        self.angular_v = angular_v
        self.linear_v_x = linear_v_x
        self.linear_v_y = linear_v_y
        self.toward = toward
        self.x = x
        self.y = y
        
        self.pos=np.array([[x],[y]])
        
        def get_distance(self, target):
            return np.linalg.norm(target - self.pos)

        def get_direction(self, target):
            direction = target - self.pos
            return direction / np.linalg.norm(direction)

        

class Workstation:

    def __init__(self, type, x, y, proc_time, material_status, product_status):
        self.type = type
        self.x = x
        self.y = y
        self.proc_time = proc_time
        self.material_status = material_status
        self.product_status = product_status
        self.pos=np.array([[x],[y]])
        if self.type==1:
            self.value=3000
        if self.type==2:
            self.value=7600-4400
        if self.type==3:
            self.value=9200-5800
        if self.type==4:
            self.value=22500-15400
        if self.type==5:
            self.value=25000-17200
        if self.type==6:
            self.value=27500-19200
        if self.type==7:
            self.value=105000-76000
        if self.type==8:
            self.value=0
        if self.type==9:
            self.value=0
    
        
            