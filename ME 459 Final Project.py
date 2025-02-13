# -*- coding: utf-8 -*-
"""
Created on Wed Feb  9 04:58:11 2022

@author: alexa
"""
import numpy as np
import math as m
import matplotlib.pyplot as plt 
import time

class Node():
    def __init__(self, x, y, z, cost, index):
        self.x = x
        self.y = y
        self.z = z
        self.cost = cost
        self.index = index

class Turtle():
    def __init__(self, x, y, z, step_size):
        self.position  = [x,y,z]
        self.move_list = [[step_size,0,0], #move right
                          [-step_size,0,0], #move left 
                          [0,step_size,0], #move up
                          [0,-step_size,0],#move down
                          [-step_size,-step_size,0], #move southwest
                          [step_size,-step_size,0],#move southeast
                          [step_size,step_size,0],#move northeast
                          [-step_size,step_size,0],#move northwest
                          [step_size,0,step_size], #move right a plane above
                          [-step_size,0,step_size], #move left a plane above 
                          [0,step_size,step_size], #move up a plane above
                          [0,-step_size,step_size],#move down a plane above
                          [-step_size,-step_size,step_size], #move southwest a plane above
                          [step_size,-step_size,step_size],#move southeast a plane above
                          [step_size,step_size,step_size],#move northeast a plane above
                          [-step_size,step_size,step_size],#move northwest a plane above
                          [step_size,0,-step_size], #move right a plane below
                          [-step_size,0,-step_size], #move left a plane below 
                          [0,step_size,-step_size], #move up a plane below
                          [0,-step_size,-step_size],#move down a plane below
                          [-step_size,-step_size,-step_size], #move southwest a plane below
                          [step_size,-step_size,-step_size],#move southeast a plane below
                          [step_size,step_size,-step_size],#move northeast a plane below
                          [-step_size,step_size,-step_size],#move northwest a plane below
                          ]
        
        self.visited_history = {}
        self.not_visited = {} 
        self.obstacle_location = {}
        
   
            
class ConfigSpace():
    
    def __init__(self, x_bounds, y_bounds, z_bounds, spacing):
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds
        self.spacing = spacing
        
    def set_obstacles(self, obstacle_list):
        self.obstacles = obstacle_list
            
    def set_graph_coords(self):
        """graph coordinates and define the search space"""
        self.x_coords = np.arange(self.x_bounds[0], self.x_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.y_coords = np.arange(self.y_bounds[0], self.y_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.z_coords = np.arange(self.z_bounds[0], self.z_bounds[1]+self.spacing,
                                  self.spacing)
        
        self.generate_search_space()
        
    def get_x_y_z_coords(self):
        return self.x_coords, self.y_coords, self.z_coords
    
    def generate_search_space(self):
        """generate our search space"""
        self.search_space = np.zeros((len(self.x_coords),len(self.y_coords),len(self.y_coords))) 
    
     
    def place_obstacles(self, obst_list):
        """places obstacles in grid by inserting a 1""" 
        for obstacle in obst_list:
            obs_x = obstacle[0]
            obs_y = obstacle[1]
            obs_z = obstacle[2]
            self.search_space[obs_x, obs_y, obs_z]= 1
    
    def calc_index(self,position):
        """calculate index """
        index = ((position[1] - self.y_bounds[0]) / \
            self.spacing * (self.x_bounds[1] - self.x_bounds[0] + self.spacing)/ \
                self.spacing + (position[0] - self.x_bounds[0]) / self.spacing) \
                + position[2]*(self.x_bounds[1] - self.x_bounds[0] + self.spacing)* \
                (self.y_bounds[1] - self.y_bounds[0] + self.spacing)
                
        return index              
    
def check_within_obstacle(obstacle_list, current_position, obstacle_radius):
    """check if I am within collision of obstacle return True if it is
    false if I'm not"""
    for obstacle in obstacle_list:
        distance = compute_distance(current_position, obstacle)
        
        if distance<=obstacle_radius:
            return True
        else:    
            return False

def check_if_obstacle_is_present(obstacle_list, node_in_question):
    """check to see if an obstacle is in the way"""
    if node_in_question in obstacle_list:
        return True

def check_obstacle_exists(obstacle_list):
    """sanity check to see if obstacle exists"""
    for obst in obstacle_list:
        if configSpace.search_space[obst[0],obst[1],obst[2]] == 1:
            print("yes", configSpace.search_space[obst[0],obst[1]],obst[2])

   
def compute_distance(current_pos, another_pos):
    """compute distance"""
    dist = m.sqrt((another_pos[0] - current_pos[0])**2+(another_pos[1]- current_pos[1])**2+(another_pos[2]- current_pos[2])**2)
    
    return dist
    #return dist(current_pos, another_pos)

def check_out_bounds( current_position, x_bounds, y_bounds, z_bounds, bot_radius):
        """check out of bounds of configuration space"""
        
        if current_position[0] < x_bounds[0]+bot_radius or current_position[0] > x_bounds[1]-bot_radius:
            return True
        
        if current_position[1] < y_bounds[0]+bot_radius or current_position[1] > y_bounds[1]-bot_radius:
            return True
        
        if current_position[2] < z_bounds[0]+bot_radius or current_position[2] > y_bounds[1]-bot_radius:
            return True
        
        return False
    

def Astar(x_span, y_span, z_span, spacing, start_position, goal_point, obstacle_list, obstacle_radius,start_time, bot_radius):
    
    
    
    #%% ##### BUILD WORLD
    configSpace = ConfigSpace(x_span, y_span, z_span, spacing)
    configSpace.set_graph_coords()
    
    x_bounds, y_bounds, z_bounds = configSpace.get_x_y_z_coords()
    configSpace.set_obstacles(obstacle_list)
   
 


    turtle = Turtle(start_position[0],start_position[1],start_position[2],spacing)

    current_node = Node(turtle.position[0], turtle.position[1], turtle.position[2], 0, -1)
    current_index = configSpace.calc_index(turtle.position)
    turtle.not_visited[current_index] = current_node
    new_node_list = []
    node_cost_transaction_list= []
    while len(turtle.not_visited) != 0:
    
        current_node_index = min(turtle.not_visited, key=lambda x:turtle.not_visited[x].cost)
#        current_node_index = min(turtle.not_visited)
#        current_node_index = min(turtle.not_visited,  key=lambda x:turtle.not_visited[x].cost)
        current_node = turtle.not_visited[current_node_index]
#        print(current_node.x,current_node.y,current_node.cost,current_node.index)
        turtle.position = [current_node.x, current_node.y, current_node.z]
        turtle.visited_history[current_node_index] = current_node
        del turtle.not_visited[current_node_index]
        
#        if [current_node.x, current_node.y]  == goal_point:
#            #Have method to return path
#            print("I've arrived!", current_node.x, current_node.y)
#            break
        
#        print("turtle position is", turtle.position)
    
        for move in turtle.move_list:
            new_position = [turtle.position[0] + move[0], 
                            turtle.position[1] + move[1],
                            turtle.position[2] + move[2]]
            new_index = configSpace.calc_index(new_position)
            
            heuristic = compute_distance(new_position, goal_point)
            
#            weight = 1
#            weight = 2
#            weight = 5
#            weight = 10
            weight = 1000

            
            greedy_cost = compute_distance(new_position, [current_node.x, current_node.y, current_node.z]) + current_node.cost + weight*heuristic

            new_node = Node(new_position[0], new_position[1], new_position[2], greedy_cost, current_node_index)
           
            new_node_list.append([new_node.x,new_node.y,new_node.z,new_node.cost,new_node.index])
            if new_index in turtle.visited_history:
                continue
                
            if check_out_bounds(new_position, x_span, y_span, z_span, bot_radius) == True:
                
                continue
            
            if check_if_obstacle_is_present(obstacle_list, new_position) == True:
#                print('obstacle',new_index)
                continue
            
            if check_within_obstacle(obstacle_list, new_position, obstacle_radius) == True:
                continue
            if new_index not in turtle.not_visited:
                turtle.not_visited[new_index] = new_node
                continue
            if new_node.cost < turtle.not_visited[new_index].cost:
                node_cost_transaction_list.append([])
                turtle.not_visited[new_index].cost = new_node.cost
                turtle.not_visited[new_index].index = new_node.index
                continue    
    path_x = []
    path_y = []
    path_z = []
    path_xyz=[]
    
    
    goal_node = Node(goal_point[0],goal_point[1],goal_point[2],0,0)
    path_index = configSpace.calc_index([goal_node.x,goal_node.y,goal_node.z])
    path_x.append(turtle.visited_history[path_index].x)
    path_y.append(turtle.visited_history[path_index].y)
    path_z.append(turtle.visited_history[path_index].z)
    
#    print (path_index)
#    print(turtle.visited_history[394].index)
    while turtle.visited_history[path_index].index != -1:
        path_index = turtle.visited_history[path_index].index        
        path_x.append(turtle.visited_history[path_index].x)
        path_y.append(turtle.visited_history[path_index].y)
        path_z.append(turtle.visited_history[path_index].z)
        path_xyz.append([turtle.visited_history[path_index].x,turtle.visited_history[path_index].y,turtle.visited_history[path_index].z])
    first_point = goal_point
    final_cost = 0
    for point in path_xyz:
        part_of_the_cost = compute_distance(first_point, point)
        final_cost = final_cost + part_of_the_cost
        first_point = point

    time2converge = time.time() - start_time

    
    
    #plotting#
    fig = plt.figure(1)
    ax = fig.add_subplot(111,projection='3d')
    for obst in obstacle_list:
        ax.scatter(obst[0], obst[1], obst[2], zdir='z', s=3, c='red', depthshade=True)
    ax.plot3D(path_x,path_y,path_z)
    ax.set_xlabel('X Distance, m')
    ax.set_ylabel('Y Distance, m')
    ax.set_zlabel('Altitude, m')

    fig = plt.figure(2)
    ax = fig.add_subplot(111,projection='3d')
#    for obst in obstacle_list:
#        ax.scatter(obst[1], obst[0], obst[2], zdir='z', s=3, c='red', depthshade=True)
    ax.plot3D(path_y,path_x,path_z)
    ax.set_xlabel('Y Distance, m')
    ax.set_ylabel('X Distance, m')
    ax.set_zlabel('Altitude, m')
       

    return path_x, path_y, path_z, path_xyz, final_cost, time2converge, weight
if __name__=='__main__':
    
#    #########
#    voxmap = create_voxmap(data, 10)
#    print(voxmap.shape)
#    ########
    #set up parameters
    step = 1
    layermovelist = [[0,0],#no move
                 [step,0], #move right
                 [-step,0], #move left 
                 [0,step], #move up
                 [0,-step],#move down
                 [-step,-step], #move southwest
                 [step,-step],#move southeast
                 [step,step],#move northeast
                 [-step,step]]#move northwest
    buildings = []
    ######## layer 1 #######
    buildingXYcenter = [3,1]
    buildingheight = 12
    for level in range(0,buildingheight):
        for obs in layermovelist:
            new_obs = [buildingXYcenter[0] + obs[0], 
                            buildingXYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building1XYcenter = [3,4]
    building1height = 8
    for level in range(0,building1height):
        for obs in layermovelist:
            new_obs = [building1XYcenter[0] + obs[0], 
                            building1XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building2XYcenter = [3,7]
    building2height = 12
    for level in range(0,building2height):
        for obs in layermovelist:
            new_obs = [building2XYcenter[0] + obs[0], 
                            building2XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building3XYcenter = [3,10]
    building3height = 15
    for level in range(0,building3height):
        for obs in layermovelist:
            new_obs = [building3XYcenter[0] + obs[0], 
                            building3XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building4XYcenter = [3,13]
    building4height = 17
    for level in range(0,building4height):
        for obs in layermovelist:
            new_obs = [building4XYcenter[0] + obs[0], 
                            building4XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [3,16]
    building5height = 17
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [3,19]
    building5height = 20
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    ######## layer 2 #######
    buildingXYcenter = [30,1]
    buildingheight = 12
    for level in range(0,buildingheight):
        for obs in layermovelist:
            new_obs = [buildingXYcenter[0] + obs[0], 
                            buildingXYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building1XYcenter = [30,4]
    building1height = 8
    for level in range(0,building1height):
        for obs in layermovelist:
            new_obs = [building1XYcenter[0] + obs[0], 
                            building1XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building2XYcenter = [30,7]
    building2height = 12
    for level in range(0,building2height):
        for obs in layermovelist:
            new_obs = [building2XYcenter[0] + obs[0], 
                            building2XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building3XYcenter = [30,10]
    building3height = 15
    for level in range(0,building3height):
        for obs in layermovelist:
            new_obs = [building3XYcenter[0] + obs[0], 
                            building3XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building4XYcenter = [30,13]
    building4height = 17
    for level in range(0,building4height):
        for obs in layermovelist:
            new_obs = [building4XYcenter[0] + obs[0], 
                            building4XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [30,16]
    building5height = 17
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [30,19]
    building5height = 20
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)        
    ######layer 3######
    buildingXYcenter = [58,1]
    buildingheight = 20
    for level in range(0,buildingheight):
        for obs in layermovelist:
            new_obs = [buildingXYcenter[0] + obs[0], 
                            buildingXYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building1XYcenter = [58,4]
    building1height = 20
    for level in range(0,building1height):
        for obs in layermovelist:
            new_obs = [building1XYcenter[0] + obs[0], 
                            building1XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building2XYcenter = [58,7]
    building2height = 20
    for level in range(0,building2height):
        for obs in layermovelist:
            new_obs = [building2XYcenter[0] + obs[0], 
                            building2XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building3XYcenter = [58,10]
    building3height = 20
    for level in range(0,building3height):
        for obs in layermovelist:
            new_obs = [building3XYcenter[0] + obs[0], 
                            building3XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building4XYcenter = [58,13]
    building4height = 20
    for level in range(4,building4height):
        for obs in layermovelist:
            new_obs = [building4XYcenter[0] + obs[0], 
                            building4XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [58,16]
    building5height = 20
    for level in range(4,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [58,19]
    building5height = 20
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    ####### layer 4 #######
    buildingXYcenter = [85,1]
    buildingheight = 20
    for level in range(4,buildingheight):
        for obs in layermovelist:
            new_obs = [buildingXYcenter[0] + obs[0], 
                            buildingXYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building1XYcenter = [85,4]
    building1height = 20
    for level in range(4,building1height):
        for obs in layermovelist:
            new_obs = [building1XYcenter[0] + obs[0], 
                            building1XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building2XYcenter = [85,7]
    building2height = 20
    for level in range(0,building2height):
        for obs in layermovelist:
            new_obs = [building2XYcenter[0] + obs[0], 
                            building2XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building3XYcenter = [85,10]
    building3height = 20
    for level in range(0,building3height):
        for obs in layermovelist:
            new_obs = [building3XYcenter[0] + obs[0], 
                            building3XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building4XYcenter = [85,13]
    building4height = 20
    for level in range(0,building4height):
        for obs in layermovelist:
            new_obs = [building4XYcenter[0] + obs[0], 
                            building4XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [85,16]
    building5height = 20
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    building5XYcenter = [85,19]
    building5height = 20
    for level in range(0,building5height):
        for obs in layermovelist:
            new_obs = [building5XYcenter[0] + obs[0], 
                            building5XYcenter[1] + obs[1],
                            level]
            buildings.append(new_obs)
    start_time = time.time()
#    x_span = [0,15] #layer1
#    layer = 1
#    x_span = [0,40] #layer2
#    layer = 2
#    x_span = [0,65] #layer3
#    layer = 3
    x_span = [0,91] #layer4
    layer = 4
    y_span = [0,20]
    z_span = [0,20]
    spacing = 1
    start_position = [0,0,0]
#    goal_point = [10,5,2] #layer1
#    goal_point = [35,5,2] #layer2
#    goal_point = [62,5,2] #layer3
    goal_point = [90,15,2]
    bot_radius = 1.0
    obstacle_radius = 1
    obstacle_list = buildings
  
    path_x, path_y, path_z, path_xyz, final_cost, time2converge, weight = Astar(x_span, y_span, z_span, spacing, start_position, goal_point, obstacle_list, obstacle_radius, start_time, bot_radius)
    
    print("using a weight of:", weight)
    print("and %s layer of complexity to the c-space" % (layer))
    print("The time to converge is: %s seconds" % (time2converge))
    print('The cost is: ', final_cost )
    print('the path is:', path_xyz)
