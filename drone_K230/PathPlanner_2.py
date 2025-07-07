import builtins

class PathPlanner:
    def __init__(self, hang, lie, start, target, map):
        self.hang = hang
        self.lie = lie
        self.openList = []
        self.closeList = []
        self.final_path = []
        self.f_dict = {}
        self.g_dict = {}
        self.h_dict = {}
        self.father_dict = {}
        self.start = start
        self.target = target
        self.map = map  # 保存地图
        self.empty_2d_array = map  # 假设 map 是一个二维数组，表示可通行区域
        self.flag = 0

    # 曼哈顿距离启发式函数
    def manhattan_distance(self, x, y):
        if not isinstance(x, int) or not isinstance(y, int) or x < 0 or y < 0:
            return -1
        else:
            return abs(x - self.target[0]) * 10 + abs(y - self.target[1]) * 10

    # 欧几里得距离启发式函数
    def oula_distance(self, x, y, start):
        if x < 0 or y < 0 or not isinstance(start, tuple) or x >= self.hang or y >= self.lie or self.map[x][y] == 0:
            return -1
        elif (x == start[0] - 1 and y == start[1] + 1) or \
             (x == start[0] - 1 and y == start[1] - 1) or \
             (x == start[0] + 1 and y == start[1] + 1) or \
             (x == start[0] + 1 and y == start[1] - 1):
            return 14
        else:
            return 10

    # 寻找路径
    def find_path(self):
        self.openList.append(self.start)
        while self.openList:
            current = self.openList[0]
            for node in self.openList[1:]:
                if self.f_dict[node] < self.f_dict[current]:
                    current = node

            if current == self.target:
                print("目标可到达")
                self.flag = 1
                break

            x, y = current
            self.closeList.append(current)
            self.openList.remove(current)

            # 检查八个方向
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.hang and 0 <= ny < self.lie and self.map[nx][ny] == 1 and (nx, ny) not in self.closeList:
                    g = self.g_dict.get(current, 0) + self.oula_distance(nx, ny, current)
                    h = self.manhattan_distance(nx, ny)
                    f = g + h

                    if (nx, ny) in self.openList:
                        if g < self.g_dict.get((nx, ny), float('inf')):
                            self.f_dict[(nx, ny)] = f
                            self.g_dict[(nx, ny)] = g
                            self.h_dict[(nx, ny)] = h
                            self.father_dict[(nx, ny)] = current
                    else:
                        self.openList.append((nx, ny))
                        self.f_dict[(nx, ny)] = f
                        self.g_dict[(nx, ny)] = g
                        self.h_dict[(nx, ny)] = h
                        self.father_dict[(nx, ny)] = current

        # 重建路径
        if self.flag == 1:
            path = []
            current = self.target
            while current in self.father_dict:
                path.insert(0, current)
                current = self.father_dict[current]
            path.insert(0, self.start)
            self.final_path = path
            print("路径已找到:", path)
        else:
            print("无法到达目标")

# 使用示例
if __name__ == "__main__":
    # 定义地图（1为可通行，0为不可通行）
    map = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 1, 1, 1, 1]
    ]
    hang = len(map)
    lie = len(map[0]) if hang > 0 else 0
    start = (0, 0)
    target = (4, 4)

    planner = PathPlanner(hang, lie, start, target, map)
    planner.find_path()

    print("最终路径:", planner.final_path)
