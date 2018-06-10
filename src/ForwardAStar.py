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
grid = [[0 for col in range(columns)] for row in range(rows)]
is_loop = True


# 4-connectedness movements
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

# 4-connectedness directions
delta_name = ['^', '<', 'v', '>']



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
def search():
    global is_loop
    while(is_loop):
        if(gui.isplay):

            gui.restart()
            for i in range(rows):
                for j in range(columns):
                    if(grid[i][j] != 1 and grid1[i][j] == 1):
                        gui.drawText(i,j, "", "", "", "white")
                        gui.drawWall2(i,j)

            # Initialization of parameters
            # Initialize f_cost, g_cost and h-cost
            f_cost = [[math.inf for col in range(columns)] for row in range(rows)]
            g_cost = [[math.inf for col in range(columns)] for row in range(rows)]
            h_cost = [[math.inf for col in range(columns)] for row in range(rows)]

            # action list
            action = [[-1 for col in range(columns)] for row in range(rows)]

            # policy list
            policy = [[' ' for row in range(columns)] for col in range(rows)]
            policy[goal[0]][goal[1]] = "*"

            # g-cost between adjacent cells
            cost = gui.g_cost()

            # D-value used in heuristics
            D = gui.get_D()

            # Choosing the heuristic function
            if(gui.selectHeuristic() == True):
                h_cost = manhattan_heuristic_cost(D)
            else:
                h_cost = euclidean_heuristic_cost(D)

            g_cost[init[0]][init[1]] = 0
            f_cost[init[0]][init[1]] = g_cost[init[0]][init[1]] + h_cost[init[0]][init[1]]

            for i in range(len(delta)):
                x = init[0] + delta[i][0]
                y = init[1] + delta[i][1]
                # Check boundary conditions
                if(x >= 0 and x < rows and y >=0 and y < columns):
                    if(grid1[x][y] == 0):
                        g_cost[x][y] = cost
                        f_cost[x][y] = g_cost[x][y] + h_cost[x][y]
                    else:
                        grid[x][y] = 1
                        gui.drawWall(x,y)



            # Add the values in order - f,h,g and then coordinates for making the sorting in order f -> h -> g to break the ties if f cost is same.
            #open = [[f_cost[init[0]][init[1]], h_cost[init[0]][init[1]], g_cost[init[0]][init[1]], init[0], init[1]]]
            open = [[f_cost[init[0]][init[1]], g_cost[init[0]][init[1]], h_cost[init[0]][init[1]], init[0], init[1]]]


            # closed list
            closed = [[0 for col in range(columns)] for row in range(rows)]

            # Add start point in closed list
            closed[init[0]][init[1]] = 1

            is_obstacle = False

            # Flags for expansion order
            found = False  # flag that is set when search is complete
            resign = False # flag set if we can't find expand

            # Loop until a path is found out or report failure case
            while(not found and not resign and not is_obstacle):
                if(gui.isplay):

                    # Open list is empty and the path is not foound, so returning with failure case
                    if len(open) == 0:
                        resign = True
                        print("Fail")
                        return "Fail"

                    else:
                        # Sort the open list and minimum f-value node is popped out and expanded
                        open.sort()
                        open.reverse()
                        next = open.pop()

                        # Coordinates of the popped out node or expanded node
                        x = next[3]
                        y = next[4]

                        # GUI Interaction
                        # Expanded nodes
                        if(not (x == init[0] and y == init[1]) and not (x == goal[0] and y == goal[1])):
                            gui.drawText(x, y, f_cost[x][y], g_cost[x][y], h_cost[x][y], "turquoise")
                            time.sleep((10 - gui.get_sleep())/10 + 0.05)

                        # Found the goal, return from loop
                        if x == goal[0] and y == goal[1]:
                            found = True
                        else:

                            # # Sensing the environment
                            # for i in range(len(delta)):
                            #
                            #     x2 = x + delta[i][0]
                            #     y2 = y + delta[i][1]
                            #
                            #     # Check boundary conditions
                            #     if(x2 >= 0 and x2 < rows and y2 >=0 and y2 < columns):
                            #
                            #
                            #         # If there is a new Obstacle
                            #         if(closed[x2][y2] == 0 and grid1[x2][y2] == 1 and grid[x2][y2] == 0):
                            #             gui.drawWall(x2,y2)
                            #             is_obstacle = True
                            #         elif(closed[x2][y2] == 0 and grid1[x2][y2] == 0 and grid[x2][y2] == 0):
                            #             g_cost[x2][y2] = cost + g_cost[x][y]
                            #             f_cost[x2][y2] = g_cost[x2][y2] + h_cost[x2][y2]
                            #
                            # if(is_obstacle):
                            #     init[0] = x
                            #     init[1] = y
                            #
                            #     # Printing the relevant list values
                            #     print("Grid World")
                            #     print(np.array(grid))
                            #     print("\n\nPolicy")
                            #     print(np.array(policy))
                            #     print("\n\nF-Values")
                            #     print(np.array(f_cost))
                            #     print("\n\nG-Values")
                            #     print(np.array(g_cost))
                            #     print("\n\nH-Values")
                            #     print(np.array(h_cost))
                            #
                            #     continue


                            # Explore the popped node in all 4 directions
                            for i in range(len(delta)):

                                x2 = x + delta[i][0]
                                y2 = y + delta[i][1]

                                # Check boundary conditions
                                if(x2 >= 0 and x2 < rows and y2 >=0 and y2 < columns):


                                    # If the node is not explored before
                                    if(closed[x2][y2] == 0 and grid[x2][y2] == 0):

                                        g_cost[x2][y2] = cost + g_cost[x][y]
                                        f_cost[x2][y2] = g_cost[x2][y2] + h_cost[x2][y2]

                                        # Add the neighbour nodes into the open list
                                        #open.append([f_cost[x2][y2], h_cost[x2][y2], g_cost[x2][y2], x2, y2])
                                        open.append([f_cost[x2][y2], g_cost[x2][y2], h_cost[x2][y2], x2, y2])

                                        # Add the node in closed list
                                        closed[x2][y2] = 1

                                        # Saving action for path
                                        action[x2][y2] = i

                                        # GUI interaction
                                        # Printing text
                                        # Open Nodes
                                        if(not (init[0] == x2 and init[1] == y2)):
                                            if(not (goal[0] == x2 and goal[1] == y2)):
                                                gui.drawText(x2, y2, f_cost[x2][y2], g_cost[x2][y2], h_cost[x2][y2], "gold")
                                                time.sleep((10 - gui.get_sleep())/10 + 0.05)
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

                # Printing the relevant list values
                print("Grid World")
                print(np.array(grid))
                print("\n\nPolicy")
                print(np.array(policy))
                print("\n\nAction")
                print(np.array(action))
                print("\n\nF-Values")
                print(np.array(f_cost))
                print("\n\nG-Values")
                print(np.array(g_cost))
                print("\n\nH-Values")
                print(np.array(h_cost))


                # GUI Interaction
                # Start travelling from the start
                travel = init[:]
                gui.drawText(travel[0],travel[1], "", "", "", "white")
                gui.drawStart(init[0], init[1])

                while(not (travel[0] == goal[0] and travel[1] == goal[1])):


                    if(not (travel[0] == init[0] and travel[1] == init[1])):
                        gui.drawText(travel[0],travel[1], f_cost[travel[0]][travel[1]], g_cost[travel[0]][travel[1]], h_cost[travel[0]][travel[1]],"red")
                    if(policy[travel[0]][travel[1]] == '>'):
                        travel[1] += 1
                        if(grid1[travel[0]][travel[1]] == 1):
                            init[0] = travel[0]
                            init[1] = travel[1] - 1
                            print("Travel", travel[0],travel[1] - 1)
                            break
                    elif(policy[travel[0]][travel[1]] == '<'):
                        travel[1] -= 1
                        if(grid1[travel[0]][travel[1]] == 1):
                            init[0] = travel[0]
                            init[1] = travel[1] + 1
                            print("Travel", travel[0],travel[1] + 1)
                            break
                    elif(policy[travel[0]][travel[1]] == '^'):
                        travel[0] -= 1
                        if(grid1[travel[0]][travel[1]] == 1):
                            init[0] = travel[0] + 1
                            init[1] = travel[1]
                            print("Travel", travel[0] + 1,travel[1])
                            break
                    elif(policy[travel[0]][travel[1]] == 'v'):
                        travel[0] += 1
                        if(grid1[travel[0]][travel[1]] == 1):
                            init[0] = travel[0] - 1
                            init[1] = travel[1]
                            print("Travel", travel[0] - 1,travel[1])
                            break



                    gui.drawText(init[0],init[1], "", "", "", "white")
                    gui.drawText(travel[0],travel[1], "", "", "", "red")
                    gui.drawRobot(travel[0],travel[1])

                    time.sleep((10 - gui.get_sleep())/10 + 0.05)

                gui.drawText(travel[0],travel[1], f_cost[travel[0]][travel[1]], g_cost[travel[0]][travel[1]], h_cost[travel[0]][travel[1]],"red")
                gui.drawText(goal[0], goal[1], "", "", "", "red")
                gui.drawRobot(goal[0], goal[1])
                #is_loop = False



# Making the Threads for communicating with the grid-world map
AStar_thread = threading.Thread(target = search)
AStar_thread.daemon = True
AStar_thread.start()

# Start Grid World GUI
gui.start()
