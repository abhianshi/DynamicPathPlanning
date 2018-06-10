# Importing libraries and modules
import math
import numpy as np
import time
import threading
import AStarGUI as gui


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


# Heuristic Cost - Manhattan Distance
def manhattan_heuristic_cost(D):
    h_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    for i in range(rows):
        for j in range(columns):
            if(grid[i][j] != 1):
                h_cost[i][j] = float(format(D * (math.fabs(i - goal[0]) + math.fabs(j - goal[1])), "0.2f"))
    return h_cost


# Heuristic Cost - Euclidean Distance
def euclidean_heuristic_cost(D):
    h_cost = [[math.inf for col in range(columns)] for row in range(rows)]
    for i in range(rows):
        for j in range(columns):
            if(grid[i][j] != 1):
                h_cost[i][j] = float(format((D * (math.sqrt((i - goal[0]) ** 2 + (j - goal[1]) ** 2))), "0.2f"))
    return h_cost

# A-star search method
def A_star_search():
    global f_cost, g_cost, h_cost, open, closed, action, policy


    for i in range(len(delta)):
        x = init[0] + delta[i][0]
        y = init[1] + delta[i][1]

        # Check boundary conditions
        if(x >= 0 and x < rows and y >=0 and y < columns):
            if(grid1[x][y] == 1):
                grid[x][y] = 1
                gui.drawWall(x,y)




    # print(np.array(h_cost))

    g_cost[init[0]][init[1]] = 0
    f_cost[init[0]][init[1]] = g_cost[init[0]][init[1]] + h_cost[init[0]][init[1]]

    open.append([f_cost[init[0]][init[1]], g_cost[init[0]][init[1]], h_cost[init[0]][init[1]], init[0], init[1]])




    # Flags for expansion order
    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    # Loop until a path is found out or report failure case
    while(not found and not resign):

        # Open list is empty and the path is not foound, so returning with failure case
        if len(open) == 0:
            resign = True
            print("Fail")
            return [-1,-1]

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

                                # if(x2 == goal[0] and y2 == goal[1]):
                                #     return [x,y]

                    closed.append([x,y])



    # End of while loop
    if(found):

        # Policy chosen - back propagate from goal
        x = goal[0]
        y = goal[1]
        while(not x == init[0] or not y == init[1]):
            act = action[x][y]
            change = (act + 2) % 4
            x = x + delta[change][0]
            y = y + delta[change][1]
            policy[x][y] = delta_name[act]


        # Start travelling from the start
        travel = init[:]


        while(not (travel[0] == goal[0] and travel[1] == goal[1])):

            for i in range(len(delta)):
                x = travel[0] + delta[i][0]
                y = travel[1] + delta[i][1]
                if(x >= 0 and x < rows and y >=0 and y < columns):
                    if(grid1[x][y] == 1):
                        grid[x][y] = 1
                        gui.drawWall(x,y)

            if(policy[travel[0]][travel[1]] == '>'):
                travel[1] += 1
                if(grid1[travel[0]][travel[1]] == 1):
                    grid[travel[0]][travel[1]] = 1
                    init[0] = travel[0]
                    init[1] = travel[1] - 1
                    return [travel[0],travel[1] - 1]

            elif(policy[travel[0]][travel[1]] == '<'):
                travel[1] -= 1
                if(grid1[travel[0]][travel[1]] == 1):
                    grid[travel[0]][travel[1]] = 1
                    init[0] = travel[0]
                    init[1] = travel[1] + 1
                    return [travel[0],travel[1] + 1]

            elif(policy[travel[0]][travel[1]] == '^'):
                travel[0] -= 1
                if(grid1[travel[0]][travel[1]] == 1):
                    grid[travel[0]][travel[1]] = 1
                    init[0] = travel[0] + 1
                    init[1] = travel[1]
                    return [travel[0] + 1,travel[1]]

            elif(policy[travel[0]][travel[1]] == 'v'):
                travel[0] += 1
                if(grid1[travel[0]][travel[1]] == 1):
                    grid[travel[0]][travel[1]] = 1
                    init[0] = travel[0] - 1
                    init[1] = travel[1]
                    return [travel[0] - 1,travel[1]]

        return [goal[0], goal[1]]


lookahead = rows * columns
movements = rows * columns
cur_cell = []
cur_cell.append(init[0])
cur_cell.append(init[1])
# Choosing the heuristic function
if(gui.selectHeuristic() == True):
    h_cost = manhattan_heuristic_cost(D)
else:
    h_cost = euclidean_heuristic_cost(D)

is_loop = True

def real_time_aa():
    global f_cost, g_cost, h_cost, open, closed, action, policy, movements

    global is_loop
    while(is_loop):


        while(not (cur_cell[0] == goal[0] and cur_cell[1] == goal[1])):
            if(gui.isplay):

                initialization_values()
                cell_bar = A_star_search()
                if(cell_bar[0] == -1):
                    print("Fail")
                    return "Fail"

                print("s_bar", cell_bar[0], cell_bar[1])
                # Printing the relevant list values
                print("Grid World")
                print(np.array(grid))
                print("\n\Action")
                print(np.array(action))
                print("\n\Policy")
                print(np.array(policy))
                print("\n\nF-Values")
                print(np.array(f_cost))
                print("\n\nG-Values")
                print(np.array(g_cost))
                print("\n\nH-Values")
                print(np.array(h_cost))

                print("\n\nOpen List")
                print(np.array(open))
                print("\n\nClosed List")
                print(np.array(closed))

                for i in range(len(closed)):
                    x = closed[i][0]
                    y = closed[i][1]
                    h_cost[x][y] = g_cost[cell_bar[0]][cell_bar[1]] +  h_cost[cell_bar[0]][cell_bar[1]] - g_cost[x][y]


                print("\n\nH-Values")
                print(np.array(h_cost))


                for i in range(len(open)):
                    gui.drawText(open[i][3], open[i][4], open[i][0], open[i][1], open[i][2], "gold")
                for i in range(len(closed)):
                    gui.drawText(closed[i][0], closed[i][1], f_cost[closed[i][0]][closed[i][1]], g_cost[closed[i][0]][closed[i][1]], h_cost[closed[i][0]][closed[i][1]], "turquoise")

                while( not (cur_cell[0] == cell_bar[0] and cur_cell[1] == cell_bar[1]) and movements > 0):
                    if(gui.isplay):
                        time.sleep((10 - gui.get_sleep())/10 + 0.05)
                        gui.drawText(cur_cell[0], cur_cell[1] , f_cost[cur_cell[0]][cur_cell[1]], g_cost[cur_cell[0]][cur_cell[1]], h_cost[cur_cell[0]][cur_cell[1]], "red")

                        pol = policy[cur_cell[0]][cur_cell[1]]
                        a = 2
                        if(pol == '>'):
                            a = 3
                        elif(pol == '<'):
                            a = 1
                        elif(pol == '^'):
                            a = 0

                        x = cur_cell[0] + delta[a][0]
                        y = cur_cell[1] + delta[a][1]


                        gui.drawText(x, y, "", "", "", "red")
                        gui.drawRobot(x,y)

                    cur_cell[0] = x
                    cur_cell[1] = y
                    print("Current Cell", cur_cell[0], cur_cell[1])
                    movements -= 1

                print("Current Cell", cur_cell[0], cur_cell[1])
                init[0] = cur_cell[0]
                init[1] = cur_cell[1]

                time.sleep((10 - gui.get_sleep())/10 + 0.05)
                if(gui.isplay):
                    for i in range(len(open)):
                        gui.drawText(open[i][3], open[i][4], "", "", "", "white")
                        if(grid[open[i][3]][open[i][4]] != 1 and grid1[open[i][3]][open[i][4]] == 1):
                            gui.drawWall2(open[i][3],open[i][4])
                    for i in range(len(closed)):
                        gui.drawText(closed[i][0], closed[i][1], "", "", "", "white")
                        if(grid[closed[i][0]][closed[i][1]] != 1 and grid1[closed[i][0]][closed[i][1]] == 1):
                            gui.drawWall2(closed[i][0],closed[i][1])

                    time.sleep((10 - gui.get_sleep())/10 + 0.05)



# Making the Threads for communicating with the grid-world map
AStar_thread = threading.Thread(target = real_time_aa)
AStar_thread.daemon = True
AStar_thread.start()

# Start Grid World GUI
gui.start()
