
import numpy as np
import cv2
import heapq
import time


class Astar:

    def __init__(self, matrix):
        self.mat = self.prepare_matrix(matrix)
        
    class Node:
        def __init__(self, x, y, weight=0, f=0):
            self.x = x
            self.y = y
            self.weight = weight
            self.heuristic = 0
            self.parent = None
            self.g = 0
            self.f = f

        def __repr__(self):
            return str(self.weight)

        def __lt__(self, other):
            return self.f < other.f

        def __gt__(self, other):
            return self.f > other.f

    def print(self):
        for y in self.mat:
            print(y)

    def prepare_matrix(self, mat):
        matrix_for_astar = []
        for y, line in enumerate(mat):
            tmp_line = []
            for x, weight in enumerate(line):
                if weight == 0: #or weight == 125:
                    weight = np.nan
                else:
                    weight = 1
                tmp_line.append(self.Node(x, y, weight=weight))
            matrix_for_astar.append(tmp_line)
        arr = np.array(matrix_for_astar)
            
        return arr

    def equal(self, current, end):
        return (current.x+5 >= end.x and current.x-5 <= end.x and current.y+5 >= end.y and current.y - 5 <= end.y)

    def heuristic(self, current, other):
        return abs(current.x - other.x) + abs(current.y - other.y)

    def neighbours(self, matrix, current, allow_diagonals = False):
        size = 5
        neighbours_list = []
        directions = [[-1, 0], [0, -1], [1, 0], [0, 1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
        for [y, x] in directions:
            if current.x + x - 5>= 0 and current.x + x + 5< len(matrix[0]) and current.y + y -5 >= 0 and current.y + y + 5< len(matrix) and not np.any([np.isnan(val.weight) for val in matrix[(current.y+y+5, current.y+y-5), current.x+x-5:current.x+x+5:2].flatten().tolist()]) and not np.any([np.isnan(val.weight) for val in matrix[current.y+y-4:current.y+y+4:2, (current.x+x-5, current.x+x+5)].flatten().tolist()]):
                neighbours_list.append(matrix[current.y + y][current.x + x])
        return neighbours_list

    def build(self, end):
        node_tmp = end
        path = []
        while (node_tmp):
            path.append([node_tmp.x, node_tmp.y])
            node_tmp = node_tmp.parent
        return list(reversed(path))

    def run(self, point_start, point_end, allow_diagonals=False):
        matrix = self.mat
        init_points = [point_start.copy()]
        while np.any([np.isnan(val.weight) for val in matrix[point_start[1]-7:point_start[1]+7:1, point_start[0]-7:point_start[0]+7:1].flatten().tolist()]):
            ave_x = 0
            ave_y = 0
            for val in matrix[point_start[1]-7:point_start[1]+7:1, point_start[0]-7:point_start[0]+7:1].flatten().tolist():
                if np.isnan(val.weight):
                    ave_x += val.x - point_start[0]
                    ave_y += val.y - point_start[1]
            if ave_x == 0 or ave_y == 0:
                return
            if np.abs(ave_x) > np.abs(ave_y):
                move_dir = (-int(ave_x/np.abs(ave_x)), -int(ave_y/(np.abs(ave_x))))
            else:
                move_dir = (-int(ave_x/np.abs(ave_y)), -int(ave_y/(np.abs(ave_y))))
            point_start[0] += move_dir[0]
            point_start[1] += move_dir[1]
            init_points += [point_start.copy()]

        start = self.Node(point_start[0], point_start[1])
        end = self.Node(point_end[0], point_end[1])
        #closed_list = []
        #open_list = [start]
        open_set = set()
        closed_set = set()
        open_dict = {(start.x, start.y): start.f}
        open_heap = []
        open_set.add(start)
        open_heap.append(start)
        max_iteration = (len(matrix)*len(matrix[0]))//2
        outer_iteration = 0
        while len(open_heap) > 0:
            outer_iteration+=1

            if outer_iteration > max_iteration:
                print('too many steps')
                return None

            current_node = heapq.heappop(open_heap)
            #current_node = open_list[0]

            #for node in open_set:
            #    if node.f < current_node.f:
            #        current_node = node
            #if current_node.f != 1000000000000000000:
            #if len(open_list) > 0:

            if self.equal(current_node, end):
                return init_points[:-1] + self.build(current_node)

            #for node in open_set:
            #    if self.equal(current_node, node):
            #        open_set.remove(node)
            #        break
            #open_set.remove(current_node)

            if (current_node.x, current_node.y) in closed_set:
                continue

            closed_set.add(current_node)

            for neighbour in self.neighbours(matrix, current_node, allow_diagonals):
                # if neighbour in closed_set:
                #     continue
                neighbour.heuristic = self.heuristic(neighbour, end)
                neighbour.g = current_node.g+1
                neighbour.f = neighbour.heuristic + neighbour.g
                if current_node.parent != None and (current_node.x - current_node.parent.x != neighbour.x - current_node.x or current_node.y - current_node.parent.y != neighbour.y - current_node.y):
                    neighbour.f += 50
                # if neighbour.f < current_node.f or neighbour not in open_set:
                #     neighbour.parent = current_node
                # if neighbour not in open_set:
                #     open_set.add(neighbour)
                #     heapq.heappush(open_heap, neighbour)
                pos = (neighbour.x, neighbour.y)
                add_to_open = pos not in closed_set and (pos not in open_dict or open_dict[pos] > neighbour.f)
                if add_to_open:
                    neighbour.parent = current_node
                    heapq.heappush(open_heap, neighbour)
                    open_dict[pos] = neighbour.f

        return None

def route_parser(route, angle):
    if len(route) <= 1:
        return []
    prev_dir = int(np.arctan2(-route[1][1]+route[0][1], route[1][0]-route[0][0]))
    fwd = 1
    heading = prev_dir - angle 
    r = [0.002, -heading]
    initial_position = [route[0][0], route[0][1]]
    coor = initial_position
    for i in range(1, len(route)):
        delta_x = (route[i][0]-coor[len(coor)-2])
        delta_y =  (route[i][1]-coor[len(coor)-1])
        if int(np.arctan2(-delta_y, delta_x)) == prev_dir:
            continue
        else:
            dist = np.sqrt(delta_x**2 + delta_y**2)*0.03
            if  dist < 0.2:
                continue
            r += [0.001, dist]
            prev_dir = int(np.arctan2(-delta_y, delta_x))
            next_heading = np.arctan2(-delta_y, delta_x) - angle
            r += [0.002, -(next_heading - heading)]
            heading = next_heading
            coor += [route[i][0], route[i][1]]
    if len(coor) == 2:
        coor += [(route[-1][0]), (route[-1][1])]
    if (len(r) <= 2):
        dist = np.sqrt(delta_x**2 + delta_y**2)*0.03
        r += [0.001, dist]
    #res = []
    #for i in range(2, len(coor), 2):
    #    res += [(coor[i]-coor[i-2])*0.03, (coor[i+1]-coor[i-1])*0.03]
    #r += [0.001, fwd*0.03]
    #print(res)
    #print(coor)
    return r, coor    
if __name__ == "__main__":
    file = np.genfromtxt('grid1', delimiter = ' ', dtype=np.uint8) 
    g = file
    g = cv2.erode(g, np.ones((3, 3)), iterations = 2)
    start = time.time()
    path = Astar(g)
    points = path.run([45, 140], [90, 60])
    r, p = route_parser(points, np.pi/4)
    end = time.time()
    print(r)
    print(p)
    print("time: ", end - start)
    #for x, y in points:
    #    g[y][x] = 15
    #print(points)
    #print()
    for i in range(2, len(p), 2):
        g = cv2.line(g, (p[i-2], p[i-1]), (p[i], p[i+1]), 125, 1)
    np.savetxt('gridprocs', path.mat, fmt = '%s')
    cv2.imshow('frontier', g)
    key = cv2.waitKey(0)
