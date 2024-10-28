import numpy as np


def BFS(matrix, start, end):
    """
    BFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}

    dim = len(matrix) # get the dimension of np array
    
    queue = [(start, None)] # using list as queue
    
    while queue: # iterate through each node
        current, parent = queue.pop(0) # dequeue 
        visited.update({current: parent}) # update current node in visited 

        # if end meet
        if current == end: 
            """
            if meet the end node:
                run through visited dict to get start <- goal path
                then reverse to get a start -> goal path
            """
            while current is not None:
                path.append(current)
                current = visited[current]
                
            path.reverse()
            return visited, path
        
        # else visit adjacent node (right -> left)
        for i in range(dim - 1, -1, -1):
            if (matrix[current][i] > 0) and (i not in visited):
                queue.append((i, current)) # enqueue this node if visit-able

    return visited, path

def DFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    path=[]
    visited={}

    dim = len(matrix)
    stack = [(start, None)]

    while stack:
        current, parent = stack.pop()
        visited.update({current: parent})

        if current == end:
            while current is not None: 
                path.append(current)
                current = visited[current]
            path.reverse()
            return visited, path
        
        for i in range(0, dim): # using stack -> reverse push order -> right child node on top of stack
            if matrix[current][i] > 0 and i not in visited:
                stack.append((i, current))

    return visited, path


def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}

    dim = len(matrix)
    pri_queue = [(0, start, None)] # using list as prioritized queue

    while pri_queue:
        pri_queue.sort() # sort by first element of tuple a.k.a travel cost
        cost, current, parent = pri_queue.pop(0)
        visited.update({current: parent})

        if current == end:
            while current is not None:
                path.append(current)
                current = visited[current]
            path.reverse()
            return visited, path
        
        for i in range(dim - 1, -1, -1):
            if matrix[current][i] > 0 and i not in visited:
                pri_queue.append((matrix[current][i] + cost, i, current))

    return visited, path


def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 
    path=[]
    visited={}

    dim = len(matrix)

    # using list as prioritized queue
    pri_queue = [(0, start, None)] # init value (h, node, parent_of_node)

    while pri_queue:
        pri_queue.sort(key = lambda tup: tup[0]) # sorted by first element of tuple a.k.a heuristic
        h, current, parent = pri_queue.pop(0)
        visited.update({current: parent})

        if current == end:
            while current is not None:
                path.append(current)
                current = visited[current]
            path.reverse()
            return visited, path
        
        for i in range(dim - 1, -1, -1):
            if matrix[current][i] > 0 and i not in visited:
                pri_queue.append((matrix[current][i], i, current))

    return visited, path


def calEuclidDistance(pos: dict, curr_node: int, goal_node: int) -> float:
    """
    Support function for Astar algorithm
    ---------------------------
    return euclid distanct of 2 node's position in pos dictionary
    """
 
    x1, x2 = pos[curr_node]
    y1, y2 = pos[goal_node]

    # borrow square root method from numpy
    return np.sqrt((x1 - y1) ** 2 + (x2 - y2) ** 2)


def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: eclid distance based positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO: 

    path=[]
    visited={}

    dim = len(matrix)
    pri_queue = [(0, 0.0, start, None)] # init value (gn, hn, node, parent_of_node)
    
    while pri_queue:
        pri_queue.sort(key = lambda tup: tup[0] + tup[1])
        gn, hn, current, parent = pri_queue.pop(0)
        visited.update({current: parent})

        if current == end:
            while current is not None:
                path.append(current)
                current = visited[current]
            path.reverse()
            return visited, path
        
        for i in range(dim - 1, -1, -1):
            if matrix[current][i] > 0 and i not in visited:
                pri_queue.append((gn + matrix[current][i], calEuclidDistance(pos, current, end), i, current))

    return visited, path