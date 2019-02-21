## Maze Runner
# BFS
# DFS
# A star - Euclidean
# A star - Manhattan

import numpy as np
import random
import queue
import math
import copy
from tkinter import *
from statistics import mean
#from ggplot import *
#from matplotlib.pyplot import *

## Create a maze with given dimension and probability
def Create_Maze(dim,prob):
    
    maze = np.zeros((dim,dim))
    num_cells = dim * dim
    occupied_cells = int(prob * num_cells)
    #print("Non blocked cells - %d\n"%(num_cells-occupied_cells))
    while occupied_cells > 0:
        row = random.randint(0,dim-1)
        column = random.randint(0,dim-1)
        while (row is 0 and column is 0) or (row is dim-1 and column is dim-1) or (maze[row][column] == 1):
            row = random.randint(0,dim-1)
            column = random.randint(0,dim-1)
        maze[row][column] = 1;
        occupied_cells -= 1
    return maze

def GetChildNodesFromParent(maze,node,dim):
    child_nodes = []
    row = int(node / dim)
    column = node % dim
    assert (maze[row][column] == 0)
    if row > 0 and maze[row-1][column] != 1:
        child_nodes.append(((row - 1) * dim) + column)

    if column > 0 and maze[row][column-1] != 1:
        child_nodes.append((column - 1) + (row * dim))
    
    if row < dim - 1 and maze[row+1][column] != 1:
        child_nodes.append(((row + 1) * dim) + column)

    if column < dim - 1 and maze[row][column+1] != 1:
        child_nodes.append((column + 1) + (row * dim))

    return child_nodes

## DFS
def SolveMaze_DFS(maze,start,goal,dim):
    #print("\nDFS")
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_stack = list()
    Path_stack.insert(0,[start])
    Visited = set()
    Done_find = False;
    path =[]
    explored_nodes = []
    fringe_size = 1
    max_fringe_size = fringe_size
    while Path_stack and not Done_find:
        path = Path_stack.pop(0)
        fringe_size -= 1
        node = path[-1]
        explored_nodes.append(node)
        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParent(maze,node,dim):
                if node_child not in Visited and not any(node_child in fringe for fringe in Path_stack):
                    fringe_size += 1
                    path_1 = list(path)
                    path_1.append(node_child)
                    Path_stack.insert(0,path_1)
                    if node_child == goal:
                        Done_find = True
                        path = path_1
                        break
        if fringe_size > max_fringe_size:
            max_fringe_size = fringe_size
        Visited.add(node)

    #print("Explored cells - %d\n"%(len(Visited)+1),end="")
    #print("Path length - %d"%len(path))
    return path,explored_nodes,max_fringe_size

def GetKeybyValue(dictionary,value,added_list):
    for key,val in dictionary.items():
        if val == value and key not in added_list:
            return key

    assert(0)

def GetHeuristicforNode(child_x,child_y,goal_x,heuristic):
    if heuristic == "Euclidean":
        return math.sqrt(((goal_x-child_x)**2) + ((goal_x-child_y)**2))
    else:
        return (abs(goal_x-child_x)+abs(goal_x-child_y))

def GetChildNodes(maze,node,dim):
    child_nodes = []
    row = int(node / dim)
    column = node % dim
    assert (maze[row][column] == 0)
    
    if row > 0 and maze[row-1][column] != 1:
        child_nodes.append(((row - 1) * dim) + column)

    if column > 0 and maze[row][column-1] != 1:
        child_nodes.append((column - 1) + (row * dim))
    
    if row < dim - 1 and maze[row+1][column] != 1:
        child_nodes.append(((row + 1) * dim) + column)

    if column < dim - 1 and maze[row][column+1] != 1:
        child_nodes.append((column + 1) + (row * dim))

    return child_nodes

def GetChildNodesFromParentAstar(maze,node,dim,goal_x,heuristic):
    child_nodes_with_distance = []
    newlist = []
    
    child_nodes = GetChildNodes(maze,node,dim)
    eu_dict = {}
    #if eu==1:
    for child in child_nodes:
        child_x = int(child / dim)
        child_y = child % dim
        goal_1 = int(goal_x / dim)
        euclidean = GetHeuristicforNode(child_x,child_y,goal_1,heuristic)
        newlist.append(euclidean)
        eu_dict.update({child:euclidean})
            
    newlist = sorted(newlist)
    added_list = []
    for ele in newlist:
        key = GetKeybyValue(eu_dict,ele,added_list)
        child_nodes_with_distance.append([key,ele])
        added_list.append(key)
    return child_nodes_with_distance

## A star euclidean distance
def SolveMaze_Astar(maze,start,goal,dim,heuristic,Visited_start):
    #print("\nA star %s"%heuristic)
    total_eu = math.sqrt(2*((dim-1)**2))
    
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.PriorityQueue()
    Path_queue.put((total_eu,[start]))
    #Path_queue.put((priority,(0,total_eu)))
    Visited = set(Visited_start)
    Done_find = False;
    path =[]
    explored_nodes = []
    fringe_size = 1
    max_fringe_size = fringe_size
    while not Path_queue.empty() and not Done_find:
        path_full = Path_queue.get()
        fringe_size -= 1
        path = path_full[-1]
        node = path[-1]
        explored_nodes.append(node)
        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParentAstar(maze,node,dim,(dim*dim)-1,heuristic):
                if node_child[0] not in Visited:
                    fringe_size += 1
                    path_1 = list(path)
                    path_1.append(node_child[0])
                    Path_queue.put((node_child[1],path_1))
                    if node_child[0] == goal:
                        Done_find = True
                        path = path_1
                        break
        if fringe_size > max_fringe_size:
            max_fringe_size = fringe_size
        Visited.add(node)

    #print("Explored cells - %d\n"%(len(Visited)+1),end="")
    #print("Path length - %d"%len(path))
    return path,explored_nodes,max_fringe_size

def hardMaze_DFS(number_public,start,goal,dim,prob):
    public_lengths=[]
    public_maze=[]
    public_fringe_size = []
    for n in range(0,number_public):
        maze = Create_Maze(dim,prob)
        Unsolvable = False
        path,explored_nodes,max_fringe_size = SolveMaze_DFS(maze,start,goal,dim)
        if not path or path[-1] != goal:
            Unsolvable = True
            #print("\nUnsolvable maze")
        else:
            public_lengths.append(len(path))
            public_maze.append(maze)
            public_fringe_size.append(max_fringe_size)

    #print("Average fringe size")
    #print(round(mean(public_fringe_size),2))
    return public_lengths, public_maze, public_fringe_size

def hardMaze_AstarManhattan(number_public,start,goal,dim,prob):
    #print("hardMaze_AstarManhattan")
    public_explored=[]
    public_maze=[]
    public_fringe_size = []
    for n in range(0,number_public):
        maze = Create_Maze(dim,prob)
        Unsolvable = False
        path,explored_nodes,max_fringe_size = SolveMaze_Astar(maze,start,goal,dim,"Manhattan",Visited_start)
        
        if not path or path[-1] != goal:
            Unsolvable = True
            #print("\nUnsolvable maze")
        else:
            #print("explored_nodes - %d"%len(explored_nodes))
            public_explored.append(len(explored_nodes))
            public_maze.append(maze)
            public_fringe_size.append(max_fringe_size)

    #print("Average fringe size")
    #print(round(mean(public_fringe_size),2))
    return public_explored, public_maze, public_fringe_size

def genetic_algo(public_lengths,public_maze,number_fittest,type):
    new_public_maze=[]
    new_public_lengths=[]
    fittest_maze_top=[]
    fittest_maze_bottom=[]
    for n in range(0,number_fittest):
        a=public_lengths.index(max(public_lengths))
        new_public_lengths.append(public_lengths[a])
        public_lengths[a]=0
        b=public_maze[a]
        new_public_maze.append(b)
        fittest_maze_top.append(b[0:int(dim/2)][:])
        fittest_maze_bottom.append(b[int(dim/2):dim][:])

    for i in range(0,number_fittest):
        j=number_fittest-i-1
        c=np.zeros((dim,dim))
        c[0:int(dim/2)][:]=fittest_maze_top[i]
        c[int(dim/2):dim][:]=fittest_maze_bottom[j]
        #mutation
        for n in range(0,5):
            c[random.randint(1,dim-2)][random.randint(1,dim-2)]=1
        Unsolvable = False
        if type == 1:
            path,explored_nodes,max_fringe_size=SolveMaze_DFS(c,start,goal,dim)
        elif type == 2:
            path,explored_nodes,max_fringe_size=SolveMaze_DFS(c,start,goal,dim)
        elif type == 3:
            path,explored_nodes,max_fringe_size = SolveMaze_Astar(c,start,goal,dim,"Manhattan",Visited_start)
        elif type == 4:
            path,explored_nodes,max_fringe_size = SolveMaze_Astar(c,start,goal,dim,"Manhattan",Visited_start)

        if path[-1] != goal:
            Unsolvable=True
        else:
            #print("Path length - %d"%len(path))
            if type == 1:
                new_public_lengths.append(len(path))
            elif type == 2:
                new_public_lengths.append(max_fringe_size)
            elif type == 3:
                new_public_lengths.append(len(explored_nodes))
            elif type == 4:
                new_public_lengths.append(max_fringe_size)
            new_public_maze.append(c)

    return new_public_lengths, new_public_maze
def print_hardest(cc,type,n,new_public_lengths):
    if type == 1:
        print(n, "length of hardest maze path:",new_public_lengths[cc])

    elif type == 2:
        print(n, "max fringe of hardest maze path:",new_public_lengths[cc])

    elif type == 3:
        print(n, "explored nodes in hardest maze path:",new_public_lengths[cc])

    elif type == 4:
        print(n, "max fringe of hardest maze path:",new_public_lengths[cc])


def loop_genetic_algo(public_lengths, public_maze,number_fittest,number_generations,type):
    new_public_lengths, new_public_maze= genetic_algo(public_lengths, public_maze,number_fittest,type)
    for n in range(0,number_generations):
        aa=new_public_lengths.index(max(new_public_lengths))
        dd.append(new_public_lengths[aa])
        print_hardest(aa,type,n,new_public_lengths)
        #print(n," explored nodes in hardest maze path:",new_public_lengths[aa])
        new_public_lengths, new_public_maze= genetic_algo(new_public_lengths, new_public_maze,number_fittest,type)

    cc=new_public_lengths.index(max(new_public_lengths))
    dd.append(new_public_lengths[cc])
    print_hardest(cc,type,number_generations,new_public_lengths)
    hardest_maze=new_public_maze[cc]
    return hardest_maze

dim = 50
prob = 0.3
start = 0
goal = (dim*dim)-1
solved_cnt = 0
max_len = 0
Visited_start = set()
number_public=100
number_fittest=10
number_generations=200
dd=[]
#for i in range(0,1000):
    #print("Iteration - %d"%(i+1))

Unsolvable = False
type = 4

if type == 1:
    ## DFS with maximal shortest path
    public_lengths, public_maze, public_fringe_size=hardMaze_DFS(number_public,start,goal,dim,prob)
    hardest_maze=loop_genetic_algo(public_lengths, public_maze,number_fittest,number_generations,type)
    path,explored_nodes,max_fringe_size=SolveMaze_DFS(hardest_maze,start,goal,dim)

elif type == 2:
    ## DFS with maximum fringe size
    public_lengths, public_maze, public_fringe_size=hardMaze_DFS(number_public,start,goal,dim,prob)
    hardest_maze=loop_genetic_algo(public_fringe_size, public_maze,number_fittest,number_generations,type)
    path,explored_nodes,max_fringe_size=SolveMaze_DFS(hardest_maze,start,goal,dim)


elif type == 3:
    ## A star manhattan with number of nodes explored
    public_explored, public_maze, public_fringe_size=hardMaze_AstarManhattan(number_public,start,goal,dim,prob)
    hardest_maze=loop_genetic_algo(public_explored, public_maze,number_fittest,number_generations,type)
    path,explored_nodes,max_fringe_size = SolveMaze_Astar(hardest_maze,start,goal,dim,"Manhattan",Visited_start)

elif type == 4:
    ## A star manhattan with maximum fringe size
    public_explored, public_maze, public_fringe_size=hardMaze_AstarManhattan(number_public,start,goal,dim,prob)
    hardest_maze=loop_genetic_algo(public_fringe_size, public_maze,number_fittest,number_generations,type)
    path,explored_nodes,max_fringe_size = SolveMaze_Astar(hardest_maze,start,goal,dim,"Manhattan",Visited_start)

else:
    print("not a valid search choice for hard maze generation")


print()
print(dd)

#print("Maximum fringe size - hard maze")
#print(max_fringe_size)
#print("Number of solved mazes for dim=%d, prob=%f is %d"%(dim,prob,solved_cnt))
#print("Max length is %d"%max_len)
if not Unsolvable and 1:
    master = Tk()
    w = Canvas(master, width=1000, height=1000)
    w.pack()
    size = int(600 / dim)
    w.create_rectangle(size, size, dim*size, dim*size)
    id = []
    for height in range(1,dim+1):
        for width in range(0,dim):
            id.append(w.create_rectangle((width*size)+size, size*height, (width*size)+(size*2), (size*height)+size,fill='#fff'))

    for row in range(0,dim):
        for column in range(0,dim):
            if hardest_maze[row][column] == 1:
                w.itemconfigure((row*dim+column)+2, fill='black')

            if (row*dim+column) in path:
                w.itemconfigure((row*dim+column)+2, fill='red')

            if (row*dim+column) in explored_nodes and (row*dim+column) not in path:
                w.itemconfigure((row*dim+column)+2, fill='green')
    mainloop()
