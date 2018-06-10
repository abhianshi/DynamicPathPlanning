# Importing libraries and modules
import math
import numpy as np
import time
import threading
import AStarGUI as gui
import operator

# Importing the necessary parameters from the Grid World(GUI)

# Grid Paramters
grid1 = gui.grid
init = gui.init
goal = gui.goal
columns = gui.columns
rows = gui.rows

# g-cost between adjacent cells
cost = gui.g_cost()

# D-value used in heuristics
D = gui.get_D()

grid = [[0 for col in range(columns)] for row in range(rows)]
h_cost = [[math.inf for col in range(columns)] for row in range(rows)]


# 4-connectedness movements
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

# 4-connectedness directions
delta_name = ['^', '<', 'v', '>']


# Heuristic Cost - Manhattan Distance
def manhattan_heuristic_cost(D):
    global grid
    h_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    for i in range(rows):
        for j in range(columns):
            if(grid[i][j] != 1):
                h_cost[i][j] = float(format(D * (math.fabs(i - goal[0]) + math.fabs(j - goal[1])), "0.2f"))
    return h_cost


# Heuristic Cost - Euclidean Distance
def euclidean_heuristic_cost(D):
    global grid
    h_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    for i in range(rows):
        for j in range(columns):
            if(grid[i][j] != 1):
                h_cost[i][j] = float(format((D * (math.sqrt((i - goal[0]) ** 2 + (j - goal[1]) ** 2))), "0.2f"))
    return h_cost

# Choosing the heuristic function
if(gui.selectHeuristic() == True):
    h_cost = manhattan_heuristic_cost(D)
else:
    h_cost = euclidean_heuristic_cost(D)


def calculate_key(x,y):
    return [ min(g_cost[x][y], rhs_cost[x][y]) + h_cost[x][y], min(g_cost[x][y], rhs_cost[x][y]) ]


def initialize():
    # Initialization of costs
    global f_cost, g_cost, h_cost, action, policy, rhs_cost, U

    g_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    rhs_cost = [[math.inf for col in range(columns)] for row in range(rows)]

    U = {}

    # action list
    action = [[-1 for col in range(columns)] for row in range(rows)]

    # policy list
    policy = [[' ' for row in range(columns)] for col in range(rows)]
    policy[goal[0]][goal[1]] = "*"

    rhs_cost[init[0]][init[1]] = 0
    U[init[0],init[1]] = calculate_key(init[0],init[1])
    print(U[init[0],init[1]])
    # print(U[init[0],init[1]])
    # if((init[0],init[1]) in U):
    #     print("True")


def UpdateVertex(x,y):
    global explored_nodes
    explored_nodes += 1
    if(not (x == init[0] and y == init[1])):
        min_val = math.inf
        for i in range(len(delta)):
            x1 = x + delta[i][0]
            y1 = y + delta[i][1]

            # Check boundary conditions
            if(x1 >= 0 and x1 < rows and y1 >=0 and y1 < columns):
                min_val = min(min_val, g_cost[x1][y1] + cost)

        rhs_cost[x][y] = min_val

    if((x,y) in U):
        U.pop((x,y))

    if(g_cost[x][y] != rhs_cost[x][y]):
        U[x,y] = calculate_key(x,y)


    #print("Updated vertex ", x,y,calculate_key(x,y))





def compute_shortestPath():
    global f_cost, g_cost, h_cost, action, policy, rhs_cost, U

    sorted_U = sorted(U.items(), key=operator.itemgetter(1))
    #print(U)
    while( (len(sorted_U) > 0 and sorted_U[0][1] < calculate_key(goal[0],goal[1])) or rhs_cost[goal[0]][goal[1]] != g_cost[goal[0]][goal[1]]):
        #print("sorted_U", sorted_U, len(sorted_U))
        if(len(sorted_U) == 0 or (len(sorted_U) == 1 and sorted_U[0][0][0] == goal[0] and sorted_U[0][0][1] == goal[1])):
            if((len(sorted_U) == 1 and sorted_U[0][0][0] == goal[0] and sorted_U[0][0][1] == goal[1])):
                popped_val = sorted_U[0]
                u = popped_val[0]
                U.pop(sorted_U[0][0])
                sorted_U.remove(sorted_U[0])

            print("\nG-cost")
            print(np.array(g_cost))
            print("\n h-cost")
            print(np.array(h_cost))
            print("\n rhs-cost")
            print(np.array(rhs_cost))
            # print("\nAction")
            # print(np.array(action))
            # Compute the path and policy from Goal
            # Policy chosen - back propagate from goal
            x = goal[0]
            y = goal[1]
            temp = math.inf
            while(not x == init[0] or not y == init[1]):
                #print("Reverse Action", x,y)
                for i in range(len(delta)):
                    x1 = x + delta[i][0]
                    y1 = y + delta[i][1]
                    if(x1 >= 0 and x1 < rows and y1 >= 0 and y1 < columns and temp > g_cost[x1][y1]):
                        #print("Temp", temp, x1,y1)
                        temp = g_cost[x1][y1]
                        policy[x1][y1] = delta_name[(i + 2) % 4]
                        x = x1
                        y = y1
                        break

            print("\n Policy")
            print(np.array(policy))
            return True

        else:
            popped_val = sorted_U[0]
            u = popped_val[0]
            U.pop(sorted_U[0][0])
            sorted_U.remove(sorted_U[0])

            if(grid[u[0]][u[1]] != 1):

                if(g_cost[u[0]][u[1]] > rhs_cost[u[0]][u[1]]):
                    g_cost[u[0]][u[1]] = rhs_cost[u[0]][u[1]]

                    for i in range(len(delta)):
                        x1 = u[0] + delta[i][0]
                        y1 = u[1] + delta[i][1]

                        # Check boundary conditions
                        if(x1 >= 0 and x1 < rows and y1 >=0 and y1 < columns and grid[x1][y1] == 0):
                            UpdateVertex(x1,y1)
                            #if(action[x1][y1] == -1 and grid[x1][y1] != 1):
                            # if(action[x1][y1] == -1):
                            #     action[x1][y1] = i

                else:
                    g_cost[u[0]][u[1]] = math.inf
                    if(grid[u[0]][u[1]] == 0):
                        UpdateVertex(u[0],u[1])
                        for i in range(len(delta)):
                            x1 = u[0] + delta[i][0]
                            y1 = u[1] + delta[i][1]

                            # Check boundary conditions
                            if(x1 >= 0 and x1 < rows and y1 >=0 and y1 < columns and grid[x1][y1] == 0):
                                UpdateVertex(x1,y1)

                return False




def lifelong_planning(init):
    global policy, action

    initialize()
    count = 0
    curr = init[:]

    while(count < 200):
        found = compute_shortestPath()

        count += 1

        if(found == True):

            changed_edges = []
            curr = init[:]
            # Start travelling from the start
            travel = curr[:]
            while(not (travel[0] == goal[0] and travel[1] == goal[1])):
                # Change the grid to dynamic grid
                for i in range(len(delta)):
                    x = travel[0] + delta[i][0]
                    y = travel[1] + delta[i][1]
                    if(x >= 0 and x < rows and y >=0 and y < columns):
                        if(grid1[x][y] == 1 and grid[x][y] != 1):
                            grid[x][y] = 1
                            g_cost[x][y] = math.inf
                            rhs_cost[x][y] = math.inf
                            if([x,y] not in changed_edges):
                                changed_edges.append([x,y])

                if(policy[travel[0]][travel[1]] == '>'):
                    travel[1] += 1
                    if(grid1[travel[0]][travel[1]] == 1):
                        grid[travel[0]][travel[1]] = 1
                        curr[0] = travel[0]
                        curr[1] = travel[1] - 1
                        break

                elif(policy[travel[0]][travel[1]] == '<'):
                    travel[1] -= 1
                    if(grid1[travel[0]][travel[1]] == 1):
                        grid[travel[0]][travel[1]] = 1
                        curr[0] = travel[0]
                        curr[1] = travel[1] + 1
                        break

                elif(policy[travel[0]][travel[1]] == '^'):
                    travel[0] -= 1
                    if(grid1[travel[0]][travel[1]] == 1):
                        grid[travel[0]][travel[1]] = 1
                        curr[0] = travel[0] + 1
                        curr[1] = travel[1]
                        break

                elif(policy[travel[0]][travel[1]] == 'v'):
                    travel[0] += 1
                    if(grid1[travel[0]][travel[1]] == 1):
                        grid[travel[0]][travel[1]] = 1
                        curr[0] = travel[0] - 1
                        curr[1] = travel[1]
                        break

            # print("Current node", curr)
            # print("Grid World")
            # print(np.array(grid))
            # print("Grid World1")
            # print(np.array(grid1))
            # print("\nPolicy")
            # print(np.array(policy))
            print("Grid World")
            print(np.array(grid))
            if(travel[0] == goal[0] and travel[1] == goal[1]):
                found = True
                return policy
            else:
                for i in range(len(changed_edges)):
                    # print("Changed edges", changed_edges[i])
                    # print("\nrhs-cost",rhs_cost[changed_edges[i][0]][changed_edges[i][1]])
                    for j in range(len(delta)):
                        x = changed_edges[i][0] + delta[j][0]
                        y = changed_edges[i][1] + delta[j][1]
                        if(x >= 0 and x < rows and y >=0 and y < columns):
                            g_cost[x][y] = math.inf
                            UpdateVertex(x, y)

                    # print("\nrhs-cost",rhs_cost[changed_edges[i][0]][changed_edges[i][1]])
                    # action list
                    #action = [[-1 for col in range(columns)] for row in range(rows)]

                    # policy list
                    policy = [[' ' for row in range(columns)] for col in range(rows)]
                    policy[goal[0]][goal[1]] = "*"

                # print("\nG-cost")
                # print(np.array(g_cost))
                # print("\n h-cost")
                # print(np.array(h_cost))
                # print("\n rhs-cost")
                # print(np.array(rhs_cost))



# Making the Threads for communicating with the grid-world map
AStar_thread = threading.Thread(target = lifelong_planning)
AStar_thread.daemon = True
AStar_thread.start()

# Start Grid World GUI
gui.start()
