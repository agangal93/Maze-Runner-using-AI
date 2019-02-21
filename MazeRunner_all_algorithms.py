## Group - 
# Akshay Gangal, Akshay Sovani, Kshitij Minhas
# Maze Runner
# BFS
# DFS
# A star - Euclidean
# A star - Manhattan

import sys
import numpy as np
import random
import queue
import math
import copy
from tkinter import *
from time import perf_counter as pc
from statistics import mean

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

    # Print the maze
    #for row in range(0,dim):
    #    for column in range(0,dim):
    #        print("%d "%maze[row][column] ,end="")
    #    print()

    return maze

## Get the non-blocked neighbours for the current node
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

## BFS
def SolveMaze_BFS(maze,start,goal,dim):
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.Queue()
    Path_queue.put([start])
    Visited = set()
    Done_find = False;
    path =[]
    explored_nodes = []
    # Pick the first entry from the fringe
    # Get its non-blocked children.
    # Append the children to the previous path
    # If goal is found or queue is empty, exit the loop and return the path
    while not Path_queue.empty() and not Done_find:
        path = Path_queue.get()
        node = path[-1]
        explored_nodes.append(node)
        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParent(maze,node,dim):
                if node_child not in Visited:
                    path_1 = list(path)
                    path_1.append(node_child)
                    Path_queue.put(path_1)
                    if node_child == goal:
                        Done_find = True
                        path = path_1
                        break
        # Add entry to visited set
        Visited.add(node)

    return path,explored_nodes

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
    # Pop the first entry from the fringe
    # Get its non-blocked children.
    # Append the children to the previous path
    # If goal is found or queue is empty, exit the loop and return the path
    while Path_stack and not Done_find:
        path = Path_stack.pop(0)
        node = path[-1]
        explored_nodes.append(node)
        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParent(maze,node,dim):
                if node_child not in Visited and not any(node_child in fringe for fringe in Path_stack):
                    path_1 = list(path)
                    path_1.append(node_child)
                    Path_stack.insert(0,path_1)
                    if node_child == goal:
                        Done_find = True
                        path = path_1
                        break
        # Add entry to visited set
        Visited.add(node)

    return path,explored_nodes

# Get key for the dictionary from value
def GetKeybyValue(dictionary,value,added_list):
    for key,val in dictionary.items():
        if val == value and key not in added_list:
            return key

    assert(0)

# Calculate the heusristic for the node based on input type
def GetHeuristicforNode(child_x,child_y,goal_x,heuristic):
    if heuristic == "Euclidean":
        return math.sqrt(((goal_x-child_x)**2) + ((goal_x-child_y)**2))
    else:
        return (abs(goal_x-child_x)+abs(goal_x-child_y))

# Get child nodes from parent node
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
    # Prioritize the child node entries based on the heuristic distances
    for child in child_nodes:
        child_x = int(child / dim)
        child_y = child % dim
        goal_1 = int(goal_x / dim)
        euclidean = GetHeuristicforNode(child_x,child_y,goal_1,heuristic)
        newlist.append(euclidean)
        eu_dict.update({child:euclidean})
            
    newlist = sorted(newlist)
    added_list = []
    # Generate the set of {child,distance} pairs in the increasing order of distance
    for ele in newlist:
        key = GetKeybyValue(eu_dict,ele,added_list)
        child_nodes_with_distance.append([key,ele])
        added_list.append(key)
    return child_nodes_with_distance

## A star with input heuristic
def SolveMaze_Astar(maze,start,goal,dim,heuristic,Visited_start):
    total_eu = math.sqrt(2*((dim-1)**2))
    
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.PriorityQueue()
    Path_queue.put((total_eu,[start]))
    #Path_queue.put((priority,(0,total_eu)))
    Visited = set(Visited_start)
    Done_find = False;
    path =[]
    explored_nodes = []
    # Pick the first entry from the priority queue
    # Get its non-blocked children.
    # Add the children to the priority queue based on the heuristic distance
    # If goal is found or queue is empty, exit the loop and return the path
    while not Path_queue.empty() and not Done_find:
        path_full = Path_queue.get()
        path = path_full[-1]
        node = path[-1]
        explored_nodes.append(node)
        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodesFromParentAstar(maze,node,dim,(dim*dim)-1,heuristic):
                if node_child[0] not in Visited:
                    path_1 = list(path)
                    path_1.append(node_child[0])
                    Path_queue.put((node_child[1],path_1))
                    if node_child[0] == goal:
                        Done_find = True
                        path = path_1
                        break
        Visited.add(node)

    return path,explored_nodes

## Recursive A star at each node to get the heuristic for that node
def GetChildNodeforParentWithThining(maze,thin_maze,node,dim,goal_x,heuristic,Visited_start):
    child_nodes_with_distance = []
    newlist =[]
    eu_dict = {}
    child_nodes = GetChildNodes(maze,node,dim)
    # Solve A star for all child nodes on the thin maze considering them as start node.
    # Arrange them based on increasing value of path length
    for child in child_nodes:
        Visited_child = set(Visited_start)
        if child not in Visited_child:
            path,explored_nodes = SolveMaze_Astar(thin_maze,child,goal_x,dim,heuristic,Visited_child)
            if path[-1] != goal_x:
                eu_dict.update({child:dim*dim}) # No path found. Set max distance
            else:
                newlist.append(len(path))
                eu_dict.update({child:len(path)})

    newlist = sorted(newlist)
    added_list = []
    for ele in newlist:
        key = GetKeybyValue(eu_dict,ele,added_list)
        child_nodes_with_distance.append([key,ele])
        added_list.append(key)
    return child_nodes_with_distance

def Solve_ThiningAstar(maze,start,goal,dim,prob):
    print("Thining A star")
    maze_org = copy.deepcopy(maze)
    assert(prob > 0.02)
    blocked_nodes = int(dim * dim * prob)
    new_blocked_nodes = int(dim * dim * (prob*0.5)) # reduce number of blockages by 50% of current value(q)
    blockage_to_remove = blocked_nodes - new_blocked_nodes

    # Simplified maze
    while blockage_to_remove > 0:
        row = random.randint(0,dim-1)
        column = random.randint(0,dim-1)
        while maze[row][column] == 0:
            row = random.randint(0,dim-1)
            column = random.randint(0,dim-1)
        maze[row][column] = 0
        blockage_to_remove -= 1

    ## Print the new maze
    #print("Simplified maze")
    #for row in range(0,dim):
    #    for column in range(0,dim):
    #        print("%d "%maze[row][column] ,end="")
    #    print()

    total_eu = math.sqrt(2*((dim-1)**2))
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.PriorityQueue()
    Path_queue.put((total_eu,[start]))
    Visited = set()
    Done_find = False;
    path =[]
    explored_nodes = []
    # Pick the first entry from the priority queue
    # Get its non-blocked children by running A* on the thinned maze.
    # Add the children to the priority queue based on the heuristic distance
    # If goal is found or queue is empty, exit the loop and return the path
    while not Path_queue.empty() and not Done_find:
        path_full = Path_queue.get()
        path = path_full[-1]
        node = path[-1]
        explored_nodes.append(node)
        if node == goal:
            break
        elif node not in Visited:
            for node_child in GetChildNodeforParentWithThining(maze_org,maze,node,dim,(dim*dim)-1,"Manhattan",Visited):
                if node_child[0] not in Visited:
                    path_1 = list(path)
                    path_1.append(node_child[0])
                    Path_queue.put((node_child[1],path_1))
                    if node_child[0] == goal:
                        Done_find = True
                        path = path_1
                        break
        Visited.add(node)

    return path,maze_org,explored_nodes


dim = 100
prob = 0.2
start = 0
goal = (dim*dim)-1
solved_cnt = 0
max_len = 0
Visited_start = set()
maze = Create_Maze(dim,prob)

Unsolvable = False
## BFS
path_1,explored_nodes_1 = SolveMaze_BFS(maze,start,goal,dim)
print("BFS length = %d"%len(path_1))
## DFS
Visited_start = set()
path_2,explored_nodes_2 = SolveMaze_DFS(maze,start,goal,dim)
print("DFS length = %d"%len(path_2))
## A star euclidean
Visited_start = set()
path_3,explored_nodes_3 = SolveMaze_Astar(maze,start,goal,dim,"Euclidean",Visited_start)
print("A* euclidean length = %d"%len(path_3))
## A star manhattan
Visited_start = set()
path_4,explored_nodes_4 = SolveMaze_Astar(maze,start,goal,dim,"Manhattan",Visited_start)
print("A* Manhattan length = %d"%len(path_4))

if not path_1 or path_1[-1] != goal:
    Unsolvable = True
    print("\nUnsolvable maze")
print()

def toplevel1(maze,dim,path_2,explored_nodes_2):
    top1 = Toplevel()
    top1.title("DFS")
    ## DFS
    w2 = Canvas(top1, width=1000, height=1000)
    w2.pack()
    size = int(600 / dim)
    w2.create_rectangle(size, size, dim*size, dim*size)
    id = []
    for height in range(1,dim+1):
        for width in range(0,dim):
            id.append(w2.create_rectangle((width*size)+size, size*height, (width*size)+(size*2), (size*height)+size,fill='#fff'))

    for row in range(0,dim):
        for column in range(0,dim):
            if maze[row][column] == 1:
                w2.itemconfigure((row*dim+column)+2, fill='black')

            if (row*dim+column) in path_2:
                w2.itemconfigure((row*dim+column)+2, fill='red')

            if (row*dim+column) in explored_nodes_2 and (row*dim+column) not in path_2:
                w2.itemconfigure((row*dim+column)+2, fill='green')

def toplevel2(maze,dim,path_3,explored_nodes_3):
    top2 = Toplevel()
    top2.title("A* Euclidean")
    ## A* euclidean
    w3 = Canvas(top2, width=1000, height=1000)
    w3.pack()
    size = int(600 / dim)
    w3.create_rectangle(size, size, dim*size, dim*size)
    id = []
    for height in range(1,dim+1):
        for width in range(0,dim):
            id.append(w3.create_rectangle((width*size)+size, size*height, (width*size)+(size*2), (size*height)+size,fill='#fff'))

    for row in range(0,dim):
        for column in range(0,dim):
            if maze[row][column] == 1:
                w3.itemconfigure((row*dim+column)+2, fill='black')

            if (row*dim+column) in path_3:
                w3.itemconfigure((row*dim+column)+2, fill='red')

            if (row*dim+column) in explored_nodes_3 and (row*dim+column) not in path_3:
                w3.itemconfigure((row*dim+column)+2, fill='green')

def toplevel3(maze,dim,path_4,explored_nodes_4):
    top3 = Toplevel()
    top3.title("A* Manhattan")
    ## A* manhattan
    w4 = Canvas(top3, width=1000, height=1000)
    w4.pack()
    size = int(600 / dim)
    w4.create_rectangle(size, size, dim*size, dim*size)
    id = []
    for height in range(1,dim+1):
        for width in range(0,dim):
            id.append(w4.create_rectangle((width*size)+size, size*height, (width*size)+(size*2), (size*height)+size,fill='#fff'))

    for row in range(0,dim):
        for column in range(0,dim):
            if maze[row][column] == 1:
                w4.itemconfigure((row*dim+column)+2, fill='black')

            if (row*dim+column) in path_4:
                w4.itemconfigure((row*dim+column)+2, fill='red')

            if (row*dim+column) in explored_nodes_4 and (row*dim+column) not in path_4:
                w4.itemconfigure((row*dim+column)+2, fill='green')

if not Unsolvable and 1:

    # Using Tkinter Canvas to display the mazes
    root = Tk()
    root.title("BFS")
    # Create a new canvas
    w1 = Canvas(root, width=1000, height=1000)
    w1.pack()
    size = int(600 / dim)
    # Create the border of the maze
    w1.create_rectangle(size, size, dim*size, dim*size)
    id = []
    # Create all the inner rectanges (dim * dim)
    for height in range(1,dim+1):
        for width in range(0,dim):
            id.append(w1.create_rectangle((width*size)+size, size*height, (width*size)+(size*2), (size*height)+size,fill='#fff'))

    # Display the maze with different colors for path, blockages and explored nodes
    for row in range(0,dim):
        for column in range(0,dim):
            if maze[row][column] == 1:
                w1.itemconfigure((row*dim+column)+2, fill='black')

            if (row*dim+column) in path_1:
                w1.itemconfigure((row*dim+column)+2, fill='red')

            if (row*dim+column) in explored_nodes_1 and (row*dim+column) not in path_1:
                w1.itemconfigure((row*dim+column)+2, fill='green')

    toplevel1(maze,dim,path_2,explored_nodes_2)
    toplevel2(maze,dim,path_3,explored_nodes_3)
    toplevel3(maze,dim,path_4,explored_nodes_4)
    root.mainloop()
