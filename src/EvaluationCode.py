# Importing libraries and modules
import math
import numpy as np
import time
import operator


# Grid Paramters
columns = 500
rows = 500
init = [0,0]
goal = [rows-1, columns-1]
grid1 = np.random.randint(2, size=(rows, columns))
toggle = True
for i in range(rows):
    for j in range(columns):
        if(grid1[i][j] == 1):
            if(toggle):
                grid1[i][j] = 0
                toggle = False
            else:
                toggle = True
# grid1 = [[0,0,0,1,0,0,0,0,0,0],[1,0,0,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],
# [0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0],[0,0,0,0,1,0,0,0,0,0],
# [0,0,1,0,0,1,0,0,0,0],[0,0,0,0,0,1,0,0,0,0]]

# g-cost between adjacent cells
cost = 1
grid = [[0 for col in range(columns)] for row in range(rows)]

# 4-connectedness movements
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

# 4-connectedness directions
delta_name = ['^', '<', 'v', '>']
explored_nodes = 0

# Heuristic Cost - Manhattan Distance
def manhattan_heuristic_cost():
    h_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    for i in range(rows):
        for j in range(columns):
            if(grid[i][j] != 1):
                h_cost[i][j] = float(format((math.fabs(i - goal[0]) + math.fabs(j - goal[1])), "0.2f"))
    return h_cost


def shortest_path(x1,y1,x2,y2):
    global grid
    queue = []
    visited = []

    queue.append([x1,y1,0])
    visited.append([x1,y1])

    while(len(queue) != 0):
        node = queue.pop(0)
        if(node[0] == x2 and node[1] == y2):
            return node[2]
        else:
            for i in range(len(delta)):
                x = node[0] + delta[i][0]
                y = node[1] + delta[i][1]

                if(x >= 0  and x < rows and y >= 0 and y < columns and grid[x][y] == 0 and [x,y] not in visited):
                    queue.append([x,y,node[2]+1])
                    visited.append([x,y])



def initialization_values():
    # Initialization of costs
    global f_cost, g_cost, h_cost, open, closed, action, policy

    f_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    g_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    open = []
    closed = []

    # action list
    action = [[-1 for col in range(columns)] for row in range(rows)]

    # policy list
    policy = [[' ' for row in range(columns)] for col in range(rows)]
    policy[goal[0]][goal[1]] = "*"


# A-star search method
def A_star_search(start_node):
    global f_cost, g_cost, h_cost, open, closed, action, policy

    g_cost[start_node[0]][start_node[1]] = 0
    f_cost[start_node[0]][start_node[1]] = g_cost[start_node[0]][start_node[1]] + h_cost[start_node[0]][start_node[1]]

    open.append([f_cost[start_node[0]][start_node[1]], g_cost[start_node[0]][start_node[1]], h_cost[start_node[0]][start_node[1]], start_node[0], start_node[1]])

    # Flags for expansion order
    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    # Loop until a path is found out or report failure case
    while(not found and not resign):

        # Open list is empty and the path is not foound, so returning with failure case
        if len(open) == 0:
            resign = True
            print("Fail")
            return "Failure"

        else:
            # Sort the open list and minimum f-value node is popped out and expanded
            open.sort()
            open.reverse()
            next = open.pop()

            # Coordinates of the popped out node or expanded node
            x = next[3]
            y = next[4]

            # Found the goal, return from loop
            if x == goal[0] and y == goal[1]:
                found = True
            else:

                # Add the node in closed list
                if([x,y] not in closed):

                    # Explore the popped node in all 4 directions
                    for i in range(len(delta)):

                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]

                        # Check boundary conditions
                        if(x2 >= 0 and x2 < rows and y2 >=0 and y2 < columns):

                            # If the node is not explored before
                            if(([x2,y2] not in closed) and grid[x2][y2] == 0):

                                g_cost[x2][y2] = cost + g_cost[x][y]
                                f_cost[x2][y2] = g_cost[x2][y2] + h_cost[x2][y2]

                                # Add the neighbour nodes into the open list
                                if([f_cost[x2][y2], g_cost[x2][y2], h_cost[x2][y2], x2, y2] not in open):
                                    open.append([f_cost[x2][y2], g_cost[x2][y2], h_cost[x2][y2], x2, y2])

                                # Saving action for path
                                action[x2][y2] = i
                    closed.append([x,y])

    # End of while loop
    if(found):

        # Policy chosen - back propagate from goal
        x = goal[0]
        y = goal[1]
        while(not x == start_node[0] or not y == start_node[1]):
            act = action[x][y]
            change = (act + 2) % 4
            x = x + delta[change][0]
            y = y + delta[change][1]
            policy[x][y] = delta_name[act]

        return policy




def forwardA(init):
    global explored_nodes, trajectory_cost
    explored_nodes = 0

    initialization_values()
    policy = A_star_search(init)
    explored_nodes += len(closed)

    curr = init[:]
    found = False

    while(not found and policy != "Failure"):

        # Start travelling from the start
        travel = curr[:]
        while(not (travel[0] == goal[0] and travel[1] == goal[1])):
            trajectory_cost += 1
            # Change the grid to dynamic grid
            for i in range(len(delta)):
                x = travel[0] + delta[i][0]
                y = travel[1] + delta[i][1]
                if(x >= 0 and x < rows and y >=0 and y < columns):
                    if(grid1[x][y] == 1):
                        grid[x][y] = 1

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

        if(travel[0] == goal[0] and travel[1] == goal[1]):
            found = True
            return policy
        else:
            initialization_values()
            policy = A_star_search(curr)
            explored_nodes += len(closed)

    return policy



def real_time_adaptive_astar(init):

    global closed, explored_nodes, trajectory_cost
    explored_nodes = 0

    initialization_values()
    policy = A_star_search(init)
    explored_nodes += len(closed)

    curr = init[:]
    found = False

    while(not found and policy != "Failure"):

        # Start travelling from the start
        travel = curr[:]
        while(not (travel[0] == goal[0] and travel[1] == goal[1])):
            trajectory_cost += 1
            # Change the grid to dynamic grid
            for i in range(len(delta)):
                x = travel[0] + delta[i][0]
                y = travel[1] + delta[i][1]
                if(x >= 0 and x < rows and y >=0 and y < columns):
                    if(grid1[x][y] == 1):
                        grid[x][y] = 1

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

        if(travel[0] == goal[0] and travel[1] == goal[1]):
            found = True
            return policy
        else:
            #closed.append((goal[0],goal[1]))
            for i in range(len(closed)):
                x = closed[i][0]
                y = closed[i][1]
                h_cost[x][y] = g_cost[curr[0]][curr[1]] +  h_cost[curr[0]][curr[1]] - g_cost[x][y]

            initialization_values()
            policy = A_star_search(curr)
            explored_nodes += len(closed)

    return policy


def learning_real_time_adaptive_astar(init):

    global closed, explored_nodes, trajectory_cost
    explored_nodes = 0

    initialization_values()
    policy = A_star_search(init)
    explored_nodes += len(closed)

    curr = init[:]
    found = False

    while(not found and policy != "Failure"):

        # Start travelling from the start
        travel = curr[:]
        while(not (travel[0] == goal[0] and travel[1] == goal[1])):
            trajectory_cost += 1
            # Change the grid to dynamic grid
            for i in range(len(delta)):
                x = travel[0] + delta[i][0]
                y = travel[1] + delta[i][1]
                if(x >= 0 and x < rows and y >=0 and y < columns):
                    if(grid1[x][y] == 1):
                        grid[x][y] = 1

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

        if(travel[0] == goal[0] and travel[1] == goal[1]):
            found = True
            return policy
        else:
            for i in range(len(closed)):
                x = closed[i][0]
                y = closed[i][1]

                min_value = 100000
                for j in range(len(open)):
                    x1 = open[j][3]
                    y1 = open[j][4]

                    dist = shortest_path(x,y,x1,y1)
                    if(dist != None):
                        min_value = min(dist + h_cost[x1][y1], min_value)
                h_cost[x][y] = min_value


            initialization_values()
            policy = A_star_search(curr)
            explored_nodes += len(closed)

    return policy


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

            # print("\nG-cost")
            # print(np.array(g_cost))
            # print("\n h-cost")
            # print(np.array(h_cost))
            # print("\n rhs-cost")
            # print(np.array(rhs_cost))
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

            # print("\n Policy")
            # print(np.array(policy))
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
    global policy, action, trajectory_cost

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
                trajectory_cost += 1
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
            # print("Grid World")
            # print(np.array(grid))
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



# Heuristic Cost - Manhattan Distance
def D_manhattan_heuristic_cost(a,b,c,d):
    return float(format((math.fabs(a-c) + math.fabs(b-d)), "0.2f"))


def D_calculate_key(x,y):
    return [ min(g_cost[x][y], rhs_cost[x][y]) + D_manhattan_heuristic_cost(start[0],start[1],x,y) + k_m, min(g_cost[x][y], rhs_cost[x][y]) ]


def D_initialize():
    # Initialization of costs
    global f_cost, g_cost, h_cost, action, policy, rhs_cost, U, k_m

    g_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    rhs_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    h_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    U = {}
    k_m = 0

    # policy list
    policy = [[' ' for row in range(columns)] for col in range(rows)]
    policy[goal[0]][goal[1]] = "*"

    rhs_cost[goal[0]][goal[1]] = 0
    U[goal[0],goal[1]] = D_calculate_key(goal[0],goal[1])


def D_UpdateVertex(x,y):
    global explored_nodes
    explored_nodes += 1
    if(not (x == goal[0] and y == goal[1])):
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
        U[x,y] = D_calculate_key(x,y)

    #print("in updated vertex", U, len(U))



def D_compute_shortestPath():
    global f_cost, g_cost, h_cost, action, policy, rhs_cost, U

    sorted_U = sorted(U.items(), key=operator.itemgetter(1))
    #print(U)
    while( (len(sorted_U) > 0 and sorted_U[0][1] < D_calculate_key(start[0],start[1])) or rhs_cost[start[0]][start[1]] != g_cost[start[0]][start[1]]):
        #print("sorted_U", sorted_U, len(sorted_U))
        if(len(sorted_U) == 0 or (len(sorted_U) == 1 and sorted_U[0][0][0] == start[0] and sorted_U[0][0][1] == start[1])):
            if((len(sorted_U) == 1 and sorted_U[0][0][0] == start[0] and sorted_U[0][0][1] == start[1])):
                popped_val = sorted_U[0]
                u = popped_val[0]
                U.pop(sorted_U[0][0])
                sorted_U.remove(sorted_U[0])

            # print("\nG-cost")
            # print(np.array(g_cost))
            # print("\n h-cost")
            # print(np.array(h_cost))
            # print("\n rhs-cost")
            # print(np.array(rhs_cost))
            # print("\nAction")
            # print(np.array(action))
            # Compute the path and policy from Goal
            # Policy chosen - back propagate from goal
            x = start[0]
            y = start[1]
            temp = math.inf
            while(not x == goal[0] or not y == goal[1]):
                #print("Reverse Action", x,y)
                for i in range(len(delta)):
                    x1 = x + delta[i][0]
                    y1 = y + delta[i][1]
                    if(x1 >= 0 and x1 < rows and y1 >= 0 and y1 < columns and temp > g_cost[x1][y1]):
                        #print("Temp", temp, x1,y1)
                        temp = g_cost[x1][y1]
                        policy[x][y] = delta_name[i]
                        x = x1
                        y = y1
                        break

            # print("\n Policy")
            # print(np.array(policy))
            return True

        else:
            k_old = sorted_U[0][1]
            popped_val = sorted_U[0]
            u = popped_val[0]
            U.pop(sorted_U[0][0])
            sorted_U.remove(sorted_U[0])


            if(grid[u[0]][u[1]] != 1):
                #print("k old and key", k_old, D_calculate_key(u[0],u[1]), g_cost[u[0]][u[1]] , rhs_cost[u[0]][u[1]])
                if(k_old < D_calculate_key(u[0],u[1])):
                    U[u[0],u[1]] = D_calculate_key(u[0],u[1])

                elif(g_cost[u[0]][u[1]] > rhs_cost[u[0]][u[1]]):
                    g_cost[u[0]][u[1]] = rhs_cost[u[0]][u[1]]

                    for i in range(len(delta)):
                        x1 = u[0] + delta[i][0]
                        y1 = u[1] + delta[i][1]

                        # Check boundary conditions
                        if(x1 >= 0 and x1 < rows and y1 >=0 and y1 < columns and grid[x1][y1] == 0):
                            D_UpdateVertex(x1,y1)

                else:
                    g_cost[u[0]][u[1]] = math.inf
                    if(grid[u[0]][u[1]] == 0):
                        D_UpdateVertex(u[0],u[1])
                        for i in range(len(delta)):
                            x1 = u[0] + delta[i][0]
                            y1 = u[1] + delta[i][1]

                            # Check boundary conditions
                            if(x1 >= 0 and x1 < rows and y1 >=0 and y1 < columns and grid[x1][y1] == 0):
                                D_UpdateVertex(x1,y1)

                sorted_U = sorted(U.items(), key=operator.itemgetter(1))
                # print("After updation", sorted_U, len(sorted_U))
                # print(len(sorted_U) > 0, sorted_U[0][1] , D_calculate_key(start[0],start[1]), rhs_cost[start[0]][start[1]], g_cost[start[0]][start[1]], start[0], start[1])

    x = start[0]
    y = start[1]
    temp = math.inf
    while(not x == goal[0] or not y == goal[1]):
        #print("Reverse Action", x,y)
        for i in range(len(delta)):
            x1 = x + delta[i][0]
            y1 = y + delta[i][1]
            if(x1 >= 0 and x1 < rows and y1 >= 0 and y1 < columns and temp > g_cost[x1][y1]):
                #print("Temp", temp, x1,y1)
                temp = g_cost[x1][y1]
                policy[x][y] = delta_name[i]
                x = x1
                y = y1
                break

    # print("\n Policy")
    # print(np.array(policy))
    return True



def D_star_lite(init):
    global policy, action, start, k_m, k_old, trajectory_cost

    start = init[:]
    last = start[:]
    D_initialize()
    found = D_compute_shortestPath()


    count = 0
    curr = init[:]

    while(count < 2000):
        count += 1

        if(found == True):

            changed_edges = []
            curr = start[:]
            # Start travelling from the start
            travel = start[:]
            while(not (travel[0] == goal[0] and travel[1] == goal[1])):
                trajectory_cost += 1
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
            # print("Grid World")
            # print(np.array(grid))
            if(travel[0] == goal[0] and travel[1] == goal[1]):
                found = True
                return policy
            else:
                start = curr[:]
                #g_cost[start[0]][start[1]] = math.inf
                k_m = k_m + D_manhattan_heuristic_cost(start[0], start[1], last[0], last[1])
                # print("k_m", k_m)
                last = start[:]
                for i in range(len(changed_edges)):
                    # print("Changed edges", changed_edges[i])
                    # print("\nrhs-cost",rhs_cost[changed_edges[i][0]][changed_edges[i][1]])
                    for j in range(len(delta)):
                        x = changed_edges[i][0] + delta[j][0]
                        y = changed_edges[i][1] + delta[j][1]
                        if(x >= 0 and x < rows and y >=0 and y < columns):
                            g_cost[x][y] = math.inf
                            D_UpdateVertex(x, y)

                    # print("\nrhs-cost",rhs_cost[changed_edges[i][0]][changed_edges[i][1]])
                    # action list
                    #action = [[-1 for col in range(columns)] for row in range(rows)]

                # policy list
                policy = [[' ' for row in range(columns)] for col in range(rows)]
                policy[goal[0]][goal[1]] = "*"
                #g_cost[start[0]][start[1]] = math.inf
                found = D_compute_shortestPath()


                # print("\nG-cost")
                # print(np.array(g_cost))
                # print("\n h-cost")
                # print(np.array(h_cost))
                # print("\n rhs-cost")
                # print(np.array(rhs_cost))





h_cost = manhattan_heuristic_cost()

initialization_values()

# start_time = time.time()
# policy = A_star_search(init)
# #policy = forwardA(init)
# end_time = time.time()
# explored_nodes = 0
# trajectory_cost = 0
# print("A* took time : ", round(end_time - start_time,4))
# if(policy != "Failure"):
#     # print("Grid World")
#     # print(np.array(grid))
#     # print("Grid World")
#     # print(np.array(grid1))
#     # print("\nPolicy")
#     # print(np.array(policy))
#     print("Number of explored nodes ", len(closed))
#     print("Trajectory Cost ", trajectory_cost)

explored_nodes = 0
trajectory_cost = 0
start_time = time.time()
policy = forwardA(init)
end_time = time.time()

print("Forward A* took time : ", round(end_time - start_time,4))
if(policy != "Failure"):
    # print("Grid World")
    # print(np.array(grid))
    # print("Grid World")
    # print(np.array(grid1))
    # print("\nPolicy")
    # print(np.array(policy))
    print("Number of explored nodes ", explored_nodes)
    print("Trajectory Cost ", trajectory_cost)


grid = [[0 for col in range(columns)] for row in range(rows)]
explored_nodes = 0
trajectory_cost = 0
start_time = time.time()
policy = real_time_adaptive_astar(init)
end_time = time.time()

print("Real Time Adaptive A* took time : ", round(end_time - start_time,4))
if(policy != "Failure"):
    # print("Grid World")
    # print(np.array(grid))
    # print("Grid World")
    # print(np.array(grid1))
    # print("\nPolicy")
    # print(np.array(policy))
    print("Number of explored nodes ", explored_nodes)
    print("Trajectory Cost ", trajectory_cost)




grid = [[0 for col in range(columns)] for row in range(rows)]
explored_nodes = 0
trajectory_cost = 0
#grid = grid1
start_time = time.time()
policy = lifelong_planning(init)
end_time = time.time()

print("Lifelong Planning took time : ", round(end_time - start_time,4))
if(policy != "Failure"):
    # print("Grid World")
    # print(np.array(grid))
    # print("Grid World")
    # print(np.array(grid1))
    # print("\nG-cost")
    # print(np.array(g_cost))
    # print("\n h-cost")
    # print(np.array(h_cost))
    # print("\n rhs-cost")
    # print(np.array(rhs_cost))
    # print("\nPolicy")
    # print(np.array(policy))
    print("Number of explored nodes ", explored_nodes)
    print("Trajectory Cost ", trajectory_cost)


grid = [[0 for col in range(columns)] for row in range(rows)]
#grid = grid1
explored_nodes = 0
trajectory_cost = 0
start_time = time.time()
policy = D_star_lite(init)
end_time = time.time()

print("D* Lite took time : ", round(end_time - start_time,4))
if(policy != "Failure"):
    # print("Grid World")
    # print(np.array(grid))
    # print("Grid World")
    # print(np.array(grid1))
    # print("\nG-cost")
    # print(np.array(g_cost))
    # # print("\n h-cost")
    # # print(np.array(h_cost))
    # print("\n rhs-cost")
    # print(np.array(rhs_cost))
    # print("\nPolicy")
    # print(np.array(policy))
    print("Number of explored nodes ", explored_nodes)
    print("Trajectory Cost ", trajectory_cost)



grid = [[0 for col in range(columns)] for row in range(rows)]
explored_nodes = 0
trajectory_cost = 0
start_time = time.time()
policy = learning_real_time_adaptive_astar(init)
end_time = time.time()

print("Learning Real Time Adaptive A* took time : ", round(end_time - start_time,4))
if(policy != "Failure"):
    # print("Grid World")
    # print(np.array(grid))
    # print("Grid World")
    # print(np.array(grid1))
    # print("\nPolicy")
    # print(np.array(policy))
    print("Number of explored nodes ", explored_nodes)
    print("Trajectory Cost ", trajectory_cost)
