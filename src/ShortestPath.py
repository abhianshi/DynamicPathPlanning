
grid = [[0, 0, 0, 0, 0],
[0, 0, 0, 0, 0],
[0, 0, 0, 0, 0],
[0, 0, 1, 0, 0],
[0, 0, 0, 1, 0]]

rows = len(grid)
columns = len(grid[0])

# 4-connectedness movements
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right



def shortest_path(x1,y1,x2,y2):
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
                    print("Adding node",x,y, node[2]+1)
                    queue.append([x,y,node[2]+1])
                    visited.append([x,y])

print(shortest_path(4,2,4,4))
