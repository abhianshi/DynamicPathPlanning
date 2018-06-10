# Importing libraries and modules
from tkinter import *
from PIL import ImageTk, Image
import time
from tkinter import messagebox
from tkinter.filedialog import askopenfilename


# Start of GUI
root = Tk()
root.title("A-Star Grid World")


# Grid Initialization
# Ask the user if he wants to load a pre-deined world map
result = messagebox.showinfo("Choose Location","Give path of A-Star Grid World Map")
filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
# filename = "/Users/abhianshusingla/Documents/IS_Project/A-Star/map1.txt"

# Read a grid world from pre-defined values
read_file = open(filename, "r")
read_str = str(read_file.read())
read_str = read_str.split("\n")

# Initialize start point
token = read_str[0].split(" ")
init = []

init.append(int(token[0]))
init.append(int(token[1]))

# Initialize goal point
token = read_str[1].split(" ")
goal = []
goal.append(int(token[0]))
goal.append(int(token[1]))

# Create grid
grid = []
for i in range(2, len(read_str)):
    token = read_str[i].split(" ")
    if(len(token) != 1):
        grid.append(list(map(int, token)))


# Size of Grid
rows = len(grid)
columns = len(grid[0])

# GUI Parameters
sleep_time = 9.9
bg_color = "PINK"
fg_color1 = "BLACK"
fg_color2 = "PURPLE"
fontStyle = "Chalkboard"
cell_size = 50
offset = 10
grids = {}



# Frames
main_frame = Frame(root, bg = bg_color)
main_frame.pack()

left_frame = Frame(main_frame, bg = bg_color)
left_frame.pack(side=LEFT)

right_frame = Frame(main_frame, bg = bg_color)
right_frame.pack(side=TOP)


# Left Frame Functionality
# Creating a 2D Grid
def createGrid():

    for i in range(0,rows + 1):
        x_start = offset
        y = i * cell_size + offset
        x_end = columns * cell_size + offset
        gridCanvas.create_line([(x_start,y),(x_end,y)])

    for i in range(0,columns + 1):
        y_start = offset
        x = i * cell_size + offset
        y_end = rows * cell_size + offset
        gridCanvas.create_line([(x,y_start),(x,y_end)])

    for i in range(0, rows):
        temp = []
        for j in range(0, columns):
            x = i * cell_size + offset
            y = j * cell_size + offset
            grids[(i,j)] = (x,y)
            temp.append((x,y))

    return grids


# Load all images
def loadImages():
    img_robot = Image.open("/Users/abhianshusingla/Documents/DynamicPathPlanning/images/robot1.png")
    img_robot = img_robot.resize((cell_size, cell_size))
    gridCanvas.img_robot = (ImageTk.PhotoImage(img_robot))

    img_target = Image.open("/Users/abhianshusingla/Documents/DynamicPathPlanning/images/target.png")
    img_target = img_target.resize((cell_size, cell_size))
    gridCanvas.img_target = (ImageTk.PhotoImage(img_target))

    img_start = Image.open("/Users/abhianshusingla/Documents/DynamicPathPlanning/images/start.png")
    img_start = img_start.resize((cell_size, cell_size))
    gridCanvas.img_start = (ImageTk.PhotoImage(img_start))

    img_wall2 = Image.open("/Users/abhianshusingla/Documents/DynamicPathPlanning/images/wall2.png")
    img_wall2 = img_wall2.resize((cell_size, cell_size))
    gridCanvas.img_wall2 = (ImageTk.PhotoImage(img_wall2))

    img_wall1 = Image.open("/Users/abhianshusingla/Documents/DynamicPathPlanning/images/wall1.png")
    img_wall1 = img_wall1.resize((cell_size, cell_size))
    gridCanvas.img_wall1 = (ImageTk.PhotoImage(img_wall1))


# Draws robot at cell coordinates x and y
def drawRobot(x,y):
    x1 = grids[(x,y)][0]
    y1 = grids[(x,y)][1]
    gridCanvas.create_image(y1, x1, image=gridCanvas.img_robot, anchor='nw')

# Draws house at cell coordinates x and y
def drawStart(x,y):
    x1 = grids[(x,y)][0]
    y1 = grids[(x,y)][1]
    gridCanvas.create_image(y1, x1, image=gridCanvas.img_robot, anchor='nw')

# Draws wall at cell coordinates x and y
def drawWall(x,y):
    x1 = grids[(x,y)][0]
    y1 = grids[(x,y)][1]
    gridCanvas.create_image(y1, x1, image=gridCanvas.img_wall1, anchor='nw')

# Draws wall at cell coordinates x and y
def drawWall2(x,y):
    x1 = grids[(x,y)][0]
    y1 = grids[(x,y)][1]
    gridCanvas.create_image(y1, x1, image=gridCanvas.img_wall2, anchor='nw')

# Draws flag at cell coordinates x and y
def drawTarget(x,y):
    x1 = grids[(x,y)][0]
    y1 = grids[(x,y)][1]
    gridCanvas.create_image(y1, x1, image=gridCanvas.img_target, anchor='nw')

# Writes text at cell coordinates x and y
def drawText(x,y,f,g,h,c):
    x1 = grids[(x,y)][0]
    y1 = grids[(x,y)][1]
    costs = str(str(f) + "\n" + str(g) + "\n" + str(h))
    if(not (goal[0] == x and goal[1] == y)):
        gridCanvas.create_rectangle(y1, x1, y1 + cell_size, x1 + cell_size, fill=c)
        gridCanvas.create_text(y1, x1, text = costs, anchor='nw', state = 'disabled')

# Reinitialization of code
def restart():
   for i in range(rows):
       for j in range(columns):
           x1 = grids[(i,j)][0]
           y1 = grids[(i,j)][1]

           if(grid[i][j] != 1):
               gridCanvas.create_rectangle(y1, x1, y1 + cell_size, x1 + cell_size, fill = "white")

           if(i == init[0] and j == init[1]):
               drawStart(i,j)
           if(i == goal[0] and j == goal[1]):
               drawTarget(i,j)

           # if(grid[i][j] == 1):
           #     drawWall2(i,j)




# Creation of Grid Canvas
gridCanvas = Canvas(left_frame, height = rows * cell_size + 2 * offset, width = columns * cell_size + 2 * offset)
gridCanvas.pack(fill=BOTH, expand = True)

# Creation of Grid
grids = createGrid()

# Loading all the images
loadImages()

# Draw Start, Obstacle and Goal Images
drawStart(init[0],init[1])
drawTarget(goal[0],goal[1])
for i in range(rows):
    for j in range(columns):
        if(grid[i][j] == 1):
            drawWall2(i,j)



# Right Frame Functionality
which_heuristic = True
isplay = False
cost = 1
D = 1

# Selection of Heuristic
def selectHeuristic():
    global which_heuristic
    which_heuristic = (env.get() == 1)
    return which_heuristic

# check Play Pause Button
def play():
    global isplay
    if(isplay):
        isplay = False
    else:
        isplay = True
    return isplay

# Get sleep time for speed purpose
def get_sleep():
    global sleep_time
    sleep_time = speed_bar.get()
    return sleep_time

# G- cost
def g_cost():
    global cost
    cost = float(cost_entry.get())
    return cost

# D cost
def get_D():
    global D
    D = float(D_entry.get())
    return D


# Controls
control_label = Label(right_frame, text="Controls",font=("Chalkboard", 20), fg = "RED", bg = bg_color)
control_label.pack(anchor = N)

# Heuristics
heuristic_label = Label(right_frame, text="Heuristsics", font=(fontStyle, 16), fg = fg_color1, bg = bg_color)
heuristic_label.pack(anchor = W)
env = IntVar()
env.set(1)
Radiobutton(right_frame, text="Manhattan", variable=env, value=1, command = selectHeuristic, font=(fontStyle, 16), fg = fg_color2, bg = bg_color).pack(anchor=W)
Radiobutton(right_frame, text="Euclidean", variable=env, value=2, command = selectHeuristic, font=(fontStyle, 16), fg = fg_color2, bg = bg_color).pack(anchor=W)

# Play/Pause
play_button = Button(right_frame, command = play, bg = bg_color, fg = fg_color2)
photo_button = ImageTk.PhotoImage(Image.open("/Users/abhianshusingla/Documents/DynamicPathPlanning/images/play1.png").resize((30, 30)))
play_button.config(image=photo_button,width="30",height="30")
play_button.pack()

# Speed Bar
speed_bar = Scale(right_frame, from_= 0, to= 10,length = 200, orient=HORIZONTAL, font=(fontStyle, 16), fg = fg_color2, bg = bg_color)
speed_bar.set(7)
speed_bar.pack(anchor=W)
speed_label = Label(right_frame, text="Speed", font=(fontStyle, 16), fg = fg_color2, bg = bg_color)
speed_label.pack()

# g-Cost
cost_frame = Frame(right_frame)
cost_frame.pack(anchor = W)
cost_label = Label(cost_frame, text = "G-Cost", font=(fontStyle, 16), fg = fg_color2, bg = bg_color)
cost_label.pack(side = LEFT)
cost_entry = Entry(cost_frame, width = 3, bg = bg_color, fg = fg_color2)
cost_entry.pack()
cost_entry.insert(0,1)

# D-value
D_frame = Frame(right_frame)
D_frame.pack(anchor = W)
D_label = Label(D_frame, text = "D-value", font=(fontStyle, 16), fg = fg_color2, bg = bg_color)
D_label.pack(side = LEFT)
D_entry = Entry(D_frame, width = 3, bg = bg_color, fg = fg_color2)
D_entry.pack(side = RIGHT)
D_entry.insert(3,1)

# Main Loop
def start():
    root.mainloop()
    time.sleep(0.1)
