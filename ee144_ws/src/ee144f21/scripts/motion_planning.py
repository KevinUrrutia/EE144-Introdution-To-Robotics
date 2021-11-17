from math import sqrt

def neighbors(current):
    #define the list of 4 neighbors
    neighbors = [[-1, 0], [0, 1], [1, 0], [0, -1]]
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ]

def heuristic_distance(candidate, goal):
    #return the euclidean distance between the cnadiate and the goal
    return sqrt(pow((goal[0] - candidate[0]), 2) + pow((goal[1] - candidate[1]), 2))

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 

    #creates an open list with the cost first and the point second
    open_list = [[0, start]]
    
    #create an empty closed list that is initialized as empty
    closed_list = []

    #create a dictionary for the past costs
    past_cost = {}
    past_cost[start] = 0

    #create a dictionary of parents
    parent = {}

    #initialize a path to return
    path = []

    #if the goal is in a obstacle return an error
    if goal in obstacles:
        print("ERROR: GOAL IS WITHIN AN OBSTACLE")
        return
    
    #while the open_list is not empty
    while open_list:
        #remove the current cheapest node from the open list and move it to closed 
        print(open_list)
        current = open_list.pop(0)
        closed_list.append(current[1])
        
        #if the current is within the goal the return, we dont want to do anything
        if current[1] == goal:
            print("REACHED GOAL")
            break

        for nbr in neighbors(current[1]):
            if nbr not in closed_list and nbr not in obstacles: 
                tentative_past_cost = past_cost[current[1]] + 1
                
                #if the neighbor has not been previously visited we need to make a addition to the cost
                if(nbr not in open_list):
                    past_cost[nbr] = tentative_past_cost

                if tentative_past_cost <= past_cost[nbr]:
                    #update past cost with the lower value
                    past_cost[nbr] = tentative_past_cost
                    #set the parent of the previous nbr
                    parent[nbr] = current[1]
                    #calculate the estimated total cost based on past cost and the heuristic
                    est_total_cost = past_cost[nbr] + heuristic_distance(nbr, goal)
                    #append to open list and sort based on the total cost
                    open_list.append([est_total_cost, nbr])
                    open_list.sort()
                #print(open_list)       
    #print(parent)
    #start from the goal to get path
    dict_index = goal 
    while(parent.get(dict_index) is not start):
        #append to front of path
        path.insert(0, parent[dict_index])
        dict_index = parent[dict_index]
      
    path.append(goal)
    return path

if __name__ == '__main__':
   start = (0, 0) 
   goal = (-5, -2)
   obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
   path = get_path_from_A_star(start, goal, obstacles)
   print(path)
