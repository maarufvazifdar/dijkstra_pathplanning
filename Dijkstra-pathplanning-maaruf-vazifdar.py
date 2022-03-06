import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue
import cv2


class Node:
    def __init__(self, position, cost, parent):
        self.position = position
        self.cost = cost
        self.parent = parent


"""
Generating Map with obstacles
"""


def halfPlane(empty_map, pt1, pt2, right_side_fill=True):
    map = empty_map.copy()

    x_arranged = np.arange(400)
    y_arranged = np.arange(250)
    X, Y = np.meshgrid(x_arranged, y_arranged)

    slope = (pt1[1] - pt2[1]) / (pt1[0] - pt2[0] + 1e-7)
    # diff = pt1[1] - slope * pt1[0]
    res = Y - slope * X - (pt1[1] - slope * pt1[0])

    if right_side_fill:
        map[res > 0] = 0
    else:
        map[res <= 0] = 0

    return map


def createQuad(map):
    # Creating the convex quadraliteral
    # Points of convex quadraliteral
    p1 = [36, 65]
    p2 = [115, 40]
    p3 = [80, 70]
    p4 = [105, 150]

    # Upper triangle
    hp1 = halfPlane(map, p1, p2, True)
    hp2 = halfPlane(map, p2, p3, False)
    hp3 = halfPlane(map, p3, p1, False)

    triangle1 = hp1 + hp2 + hp3

    # Lower triangle
    hp4 = halfPlane(map, p1, p3, True)
    hp5 = halfPlane(map, p3, p4, True)
    hp6 = halfPlane(map, p4, p1, False)

    triangle2 = hp4 + hp5 + hp6

    map = cv2.bitwise_and(triangle1, triangle2)

    return map


def createCircle(map):
    # Creating circle
    center = [300, 65]
    radius = 40
    c_x = np.arange(400)
    c_y = np.arange(250)
    c_X, c_Y = np.meshgrid(c_x, c_y)
    map[(c_X - center[0])**2 + (c_Y - center[1])**2 - radius**2 <= 0] = 0
    return map


def createHexagon(map):

    h1 = [200, 150 - (70 / np.sqrt(3))]
    h2 = [200 + 35, 150 - (35 / np.sqrt(3))]
    h3 = [200 + 35, 150 + (35 / np.sqrt(3))]
    h4 = [200, 150 + (70 / np.sqrt(3))]
    h5 = [200 - 35, 150 + (35 / np.sqrt(3))]
    h6 = [200 - 35, 150 - (35 / np.sqrt(3))]

    hp1 = halfPlane(map, h1, h2, True)
    hp2 = halfPlane(map, h2, h3, False)
    hp3 = halfPlane(map, h3, h4, False)
    hp4 = halfPlane(map, h4, h5, False)
    hp5 = halfPlane(map, h5, h6, False)
    hp6 = halfPlane(map, h6, h1, True)
    map = hp1 + hp2 + hp3 + hp4 + hp5 + hp6

    return map


def createMap():
    empty_map = np.ones((250, 400), np.uint8) * 255

    quad = createQuad(empty_map)
    circle = createCircle(quad)
    hexagon = createHexagon(circle)
    map = hexagon

    return map


def check_position(position):
    if position[1] < 5 or position[1] >= 245 or position[0] < 5 or position[0] >= 395 or inflated_map[position[1]][position[0]] == 0:
        return False
    return True


def action_set(x, y):
    actions = [(x, y + 1),
               (x + 1, y),
               (x - 1, y),
               (x, y - 1),
               (x + 1, y + 1),
               (x - 1, y - 1),
               (x - 1, y + 1),
               (x + 1, y - 1)]
    return actions


def find_neighbours(node):

    actions = action_set(node.position[0], node.position[1])
    neighbours = []
    for i, position in enumerate(actions):
        if check_position(position):
            if i > 3:
                cost = 1.4
            else:
                cost = 1
            neighbours.append([position, cost])
    return neighbours


"""
Main
"""
map = createMap()
kernel = np.ones((11, 11), 'uint8')
inflated_map = cv2.erode(map, kernel, iterations=1)

plt.imshow(map)

start_x = int(input("Enter starting x: ") or 90)
start_y = int(input("Enter starting y: ") or 150)
goal_x = int(input("Enter Goal x: ") or 110)
goal_y = int(input("Enter Goal y: ") or 140)
fast = int(
    input(
        'Visualize explored nodes.\n\tEnter 0: for normal speed \n\tEnter 1: to skip some nodes for faster visualization \n\tEnter 2: to skip visualization. \n(Defaults to is 2): ')
    or '2')

start_position = [start_x, start_y]
goal_position = [goal_x, goal_y]

if check_position(start_position) is False:
    print('Invalid starting position\n')
    exit()
if check_position(goal_position) is False:
    print('Invalid Goal position\n')
    exit()

plt.xlim([0,400])
plt.ylim([0, 250])
plt.scatter(start_position[0], start_position[1], color='green')
plt.scatter(goal_position[0], goal_position[1], color='red')


total_cost = {}
pq = PriorityQueue()
closed_list = []
node_dict = {}
explored = []

closed_list.append(str(start_position))

start_node = Node(start_position, 0, None)
goal_node = Node(goal_position, 0, None)

total_cost[str(start_position)] = 0
node_dict[str(start_node.position)] = start_node
pq.put([start_node.cost, start_node.position])

while not pq.empty():
    current_node = pq.get()
    # print(current_node)
    node = node_dict[str(current_node[1])]
    if current_node[1][0] == goal_node.position[0] and current_node[1][1] == goal_node.position[1]:
        print("Goal reached!")
        goal_node.position = goal_position
        goal_node.cost = current_node[0]
        goal_node.parent = node
        node_dict[str(goal_position)] = goal_node
        break

    for i, cost in find_neighbours(node):
        if str(i) in closed_list:

            cost = cost + total_cost[str(node.position)]
            if cost < total_cost[str(i)]:
                total_cost[str(i)] = cost
                node_dict[str(i)].parent = node
        else:
            closed_list.append(str(i))
            explored.append(i)
            cost += total_cost[str(node.position)]
            total_cost[str(i)] = cost
            new_node = Node(i, cost, node_dict[str(node.position)])
            node_dict[str(i)] = new_node
            pq.put([cost, new_node.position])

goal_node = node_dict[str(goal_position)]
parent_node = goal_node.parent

# Visualizing explored nodes
print('Visualizing explored nodes')
if fast == 0:
    for i in range(0, len(closed_list), 1):
        plt.scatter(explored[i][0], explored[i][1], s=1,
                    color='black', alpha=0.15)
        plt.pause(1e-10)
elif fast == 1:
    for i in range(0, len(closed_list), 10):
        plt.scatter(explored[i][0], explored[i][1], s=1,
                    color='black', alpha=0.5)
        plt.pause(1e-10)

# Backtracking
print('Visualizing Backtracking...')
x, y = [], []
while parent_node:
    x.append(parent_node.position[0])
    y.append(parent_node.position[1])
    parent_node = parent_node.parent
    plt.plot(x, y, markersize=1, color='blue')
    plt.pause(0.1)
plt.show()
