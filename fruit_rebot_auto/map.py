import math

dis = 100
yolo_loc = [600, 240]

class Map:
    def __init__(self, map_length, map_width) -> None:
        self.map_length = map_length
        self.map_width = map_width
        self.my_map = [[0 for i in range(map_length)] for j in range(map_width)]
        self.my_map[(self.map_width - 1)][(self.map_length - 1) // 2] = 2 #定义车的位置
    
    def add_point(self, dis , yolo_loc):
        #HFOV = 58.4度
        HFOV = 58.4 #相机的视角为58.4
        
        d_loc = [640, 480]#相机像素点尺寸
        
        cm_dis = yolo_loc[0] - (d_loc[0] // 2)

        if cm_dis > 0:
            right_angle = cm_dis / (d_loc[0]) * HFOV
            x_loc = int(dis * math.cos(math.radians(right_angle)))
            y_loc = int(dis * math.sin(math.radians(right_angle)))
            x_loc = int(x_loc // 10)
            y_loc = int(y_loc // 10)
            print('右侧', x_loc, y_loc)
            self.my_map[self.map_width - 1 - x_loc][(self.map_width - 1 // 2)  + y_loc] = 1
            self.get_map()
        elif cm_dis > 0:
            left_angle = (-cm_dis) / (d_loc[0]) * HFOV
            x_loc = int(dis[0] * math.cos(math.radians(left_angle)))
            y_loc = int(dis[0] * math.sin(math.radians(left_angle)))
            x_loc = int(x_loc // 10)
            y_loc = int(y_loc // 10)
            print('左侧', x_loc, y_loc)
            self.my_map[x_loc][self.map_width - 1 - y_loc] = 1
    
    def loc_expansion(self):
        loc_loc = []
        for i in range(self.map_length):
            for j in range(self.map_width):
                if self.my_map[j][i] == 1:
                    loc_loc.append([j, i])
        for i in loc_loc:    
            self.my_map[i[0]][i[1] + 1] = 1
            self.my_map[i[0]][i[1] - 1] = 1
            self.my_map[i[0] + 1][i[1]] = 1
            self.my_map[i[0] - 1][i[1]] = 1

            self.my_map[i[0] + 1][i[1] + 1] = 1
            self.my_map[i[0] - 1][i[1] + 1] = 1
            self.my_map[i[0] + 1][i[1] - 1] = 1
            self.my_map[i[0] - 1][i[1] - 1] = 1
            
    def get_map(self):
        for i in self.my_map:
            print(i)
        return self.my_map


if __name__ == '__main__':
    my_map = Map(40, 20)
    my_map.add_point(dis, yolo_loc)
    my_map.loc_expansion()
    my_map.get_map()