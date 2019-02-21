## Maze Runner - Maze on fire
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
import matplotlib.pyplot as plt

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
    
    if row > 0 and maze[row-1][column] != 1 and maze[row-1][column] != 2:
        child_nodes.append(((row - 1) * dim) + column)

    if column > 0 and maze[row][column-1] != 1 and maze[row][column-1] != 2:
        child_nodes.append((column - 1) + (row * dim))
    
    if row < dim - 1 and maze[row+1][column] != 1 and maze[row+1][column] != 2:
        child_nodes.append(((row + 1) * dim) + column)

    if column < dim - 1 and maze[row][column+1] != 1 and maze[row][column+1] != 2:
        child_nodes.append((column + 1) + (row * dim))

    return child_nodes

def GetNeighbours(node,dim):
    child_nodes = []
    row = int(node / dim)
    column = node % dim    
    if row > 0:
        child_nodes.append(((row - 1) * dim) + column)
    if column > 0:
        child_nodes.append((column - 1) + (row * dim))    
    if row < dim - 1:
        child_nodes.append(((row + 1) * dim) + column)
    if column < dim - 1:
        child_nodes.append((column + 1) + (row * dim))
    return child_nodes

def IsOnFire(maze,node,dim):
    row = int(node / dim)
    column = node % dim
    return (maze[row][column] == 2) 

def GetBurningNeighbours(maze,node,dim):
    nodes_fire = []
    child_nodes = GetNeighbours(node,dim)
    for child in child_nodes:
        if IsOnFire(maze,child,dim):
            nodes_fire.append(child)

    return nodes_fire

def SetNodesOnFire(maze,fire_prob,dim):
    Fire_spread = []
    for row in range(0,dim):
        for col in range(0,dim):
            if fire_prob[row][col] != 0 and fire_prob[row][col] != 1:
                p = fire_prob[row][col]
                val = random.randint(0,100)
                if val <= p * 100:
                    fire_prob[row][col] = 1
                    maze[row][col] = 2
                    Fire_spread.append(row*dim+col)

    return Fire_spread

def UpdateFireperStep(maze,fire_prob,dim):
    for row in range(0,dim):
        for col in range(0,dim):
            if maze[row][col] != 2:
                neighbour_fire = GetBurningNeighbours(maze,row*dim+col,dim)
                if neighbour_fire:
                    k = len(neighbour_fire)
                    p = 1 - (pow(0.5,k))
                    fire_prob[row][col] = p


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

## A star euclidean distance
def SolveMaze_AstarWithFire(maze,start,goal,dim,heuristic,Visited_start,fire_prob):
    total_eu = math.sqrt(2*((dim-1)**2))
    
    assert (maze[0][0] == 0) and (maze[dim-1][dim-1] == 0)
    Path_queue = queue.PriorityQueue()
    Path_queue.put((total_eu,[start]))
    Visited = set(Visited_start)
    Done_find = False;
    path =[]
    explored_nodes = []
    Fire_spread = []
    # Pick the first entry from the priority queue
    # Get its non-blocked children.
    # Add the children to the priority queue based on the heuristic distance
    # If goal is found or queue is empty, exit the loop and return the path
    while not Path_queue.empty() and not Done_find:
        path_full = Path_queue.get()
        UpdateFireperStep(maze,fire_prob,dim)
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
        for fire_node in SetNodesOnFire(maze,fire_prob,dim):
            Fire_spread.append(fire_node)

    return path,explored_nodes,Fire_spread

dim = 100
prob = 0.3
start = 0
goal = (dim*dim)-1
solved_cnt = 0
max_len = 0
type = 4
Visited_start = set()
maze = Create_Maze(dim,prob)
fire_prob = np.zeros((dim,dim))
fire_prob[0][dim-1] = 1 # Start the fire from the top right corner of the maze
maze[0][dim-1] = 2
Unsolvable = False
## Regular A star manhattan
path,explored_nodes,Fire_spread = SolveMaze_AstarWithFire(maze,start,goal,dim,"Manhattan",Visited_start,fire_prob)
print("Length of path A* = %d"%len(path))

print("Fire spread")
print(Fire_spread)

if not path or path[-1] != goal:
    Unsolvable = True
    print("\nUnsolvable maze")

print()

#if not Unsolvable and 0:
if 1:
    # Using Tkinter Canvas to display the mazes
    root = Tk()
    root.title("A star manhattan")
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

            if (row*dim+column) in path:
                w1.itemconfigure((row*dim+column)+2, fill='red')

            if (row*dim+column) in explored_nodes and (row*dim+column) not in path:
                w1.itemconfigure((row*dim+column)+2, fill='green')

            if (row*dim+column) in Fire_spread and (row*dim+column) not in path and (row*dim+column) not in explored_nodes:
                w1.itemconfigure((row*dim+column)+2, fill='yellow')

    root.mainloop()
