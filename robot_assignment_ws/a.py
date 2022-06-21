#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import random
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import matplotlib.pyplot as plt
import copy

graph = {}
vertices = []
vertices_no = 0
sample_points = []
converted_path = []
yaw = 0
occugrid = np.array([0,1])
initial_x = 0
initial_y = 0
grid = []

def callback(msg):
    global occugrid
    occugrid = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

def calculateError(x,y,x_goal,y_goal):
    x_error = (x_goal - x)**2
    y_error = (y_goal - y)**2
    error = (x_error+y_error)**(1/2)
    return error

def calculateU(error):
    k1 = 0.2
    k2 = 0.01
    k3 = 0.1
    sumX = 0
    try:
        for i in range(len(error)-20,len(error)):
            sumX = sumX + error[i]
    except:
        for i in range(len(error)):
            sumX = sumX + error[i]

    sums = sumX
    sums = np.multiply(sums,k2)
    e = np.multiply(error[len(error)-1],k1) + sums
    try:
        e = e+(k3*(np.array(error[len(error)-1]) - np.array(error[len(error)-2])))
    except:
        e = e+(k3*(np.array(error[len(error)-1]) - np.array(error[len(error)-1])))
    return e

def execute_path(path):
    error = []
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    pub_position = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    base_coordinates = get_coordinates("mobile_base","")

    x_cord = base_coordinates.pose.position.x
    y_cord = base_coordinates.pose.position.y
    x_goal = converted_path[1][0]
    y_goal = converted_path[1][1]
    distance = (x_goal - y_goal)**2
    var_twist = Twist()
    rate = rospy.Rate(20) # 20hz
    i = 2
    while not rospy.is_shutdown():
        get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        x_cord = base_coordinates.pose.position.x
        y_cord = base_coordinates.pose.position.y
        target_angle = np.arctan2(y_goal-y_cord, x_goal-x_cord)
        var_twist.angular.z = 1*(target_angle-yaw)
        if(round(target_angle,1) - round(yaw,1) == 0):
            x_cord = base_coordinates.pose.position.x
            var_twist.angular.z = 0
            pub_position.publish(var_twist)
            y_cord = base_coordinates.pose.position.y
            while(not(abs(x_cord-x_goal)<0.5 and (abs(y_cord-y_goal)<0.5))):
                base_coordinates = get_coordinates('mobile_base',"")
                x_cord = base_coordinates.pose.position.x
                y_cord = base_coordinates.pose.position.y
                if(abs(x_cord-x_goal)<0.5 and (abs(y_cord-y_goal)<0.5)):
                    if(i==len(converted_path)):
                        print("Done")
                        exit()
                    else:
                        x_goal = converted_path[i][0]
                        y_goal = converted_path[i][1]
                    i = i + 1
                    var_twist.linear.x = 0
                    var_twist.angular.z = 0
                    pub_position.publish(var_twist)
                    target_angle = np.arctan2(y_goal-y_cord, x_goal-x_cord)
                    while(not(round(target_angle,1) - round(yaw,1) == 0)):
                        target_angle = np.arctan2(y_goal-y_cord, x_goal-x_cord)
                        var_twist.angular.z = 0.8*(target_angle-yaw)
                        pub_position.publish(var_twist)
                    del error[:]
                var_twist.angular.z = 0
                error.append(calculateError(x_cord,y_cord,x_goal,y_goal))
                U = calculateU(error)
                var_twist.linear.x = 1.1*U
                pub_position.publish(var_twist)
                rate.sleep()
        else:
            pub_position.publish(var_twist)
            rate.sleep()


def print_path(path):
    global converted_path
    try:
        for i in range(len(path)):
            sample_points[path[i]][0] = round((((sample_points[path[i]][0])/float(20))-100),2)
            sample_points[path[i]][1] = round((((sample_points[path[i]][1])/float(20))-100),2)
            converted_path.append(sample_points[path[i]])
            print(sample_points[path[i]])
    except:
        print("No path found")
        exit()

def add_vertex(v):
    global graph
    global vertices_no
    if v in graph:
        print("Vertex ", v, " already exists.")
    else:
        vertices_no = vertices_no + 1
        graph[v] = []

def add_edge(v1, v2):
    global graph
    if v1 not in graph:
        print("Vertex ", v1, " does not exist.")
    elif v2 not in graph:
        print("Vertex ", v2, " does not exist.")
    else:
        graph[v1].append(v2)

def check_collision(x,y):
    global occugrid
    global grid
    x[1] = 4000 - x[1]
    y[1] = 4000 - y[1]
    if(x[0]>y[0]):
        a = -1
    else:
        a = 1
        
    if(y[0]-x[0]==0):
        if(x[1]>y[1]):
            a = -1
        else:
            a = 1
        for i in range(int(x[1]),int(y[1]),a):
            if(not grid[i][int(x[0])]==grid[2000][2000]):
                return True
        return False
    
    if(y[1]-x[1]==0):
        if(x[0]>y[0]):
            a = -1
        else:
            a = 1
        for i in range(int(x[0]),int(y[0]),a):
            if(not grid[int(x[1])][i]==grid[2000][2000]):
                return True
        return False

    gradient = ((y[1]-x[1])/(y[0]-x[0]))
    intercept = y[1]-(gradient*y[0])
    output = 0
    prevY = copy.deepcopy(x[1])
    for i in range(int(x[0]),int(y[0])+a,a):
        output = (math.floor(i*gradient)) + intercept
        if(prevY>output):
            h = -1
        else:
            h = 1
        for j in range(int(prevY),int(output),h):
            if(not grid[j][i] == grid[2000][2000]):
                return True
        prevY = copy.deepcopy(output)
        if(not grid[int(output)][i]==grid[2000][2000]):
            return True
    return False
     
def bfs(start, end):
    global graph
    queue = [(start,[start])]
    visited = set()

    while queue:
        vertex, path = queue.pop(0)
        visited.add(vertex)
        for node in graph[vertex]:
            if node == end:
                return path + [end]
            else:
                if node not in visited:
                    visited.add(node)
                    queue.append((node, path + [node]))

def find_path(start_x,start_y,goal_x,goal_y,grid):
    num_sampling_points = 10000
    square_radius = 60
    global sample_points
    sample_points.append([start_x,start_y])
    add_vertex(0)
    j = 1
    for i in range(1,num_sampling_points):
        rand_x = random.randint(80*20, 120*20)
        rand_y = random.randint(80*20, 120*20)
        if(grid[rand_x][rand_y] == grid[2000][2000]):
            sample_points.append([rand_x,rand_y])
            add_vertex(j)
            j = j + 1

    sample_points.append([goal_x,goal_y])
    add_vertex(j)
    f = 0
    nearest_neighbors = []
    for i in range(len(sample_points)):
        for j in range(i+1,len(sample_points)):
            if(abs(sample_points[i][0]-sample_points[j][0])<square_radius and abs(sample_points[i][1]-sample_points[j][1])<square_radius):
                collision = check_collision(sample_points[i],sample_points[j])
                sample_points[i][1] = 4000 - sample_points[i][1]
                sample_points[j][1] = 4000 - sample_points[j][1]
                if(not collision):
                    add_edge(i,j)
                    f = f + 1
    return bfs(0,j)

def main_script():
    global grid
    goal_x = input()
    goal_y = input()
    goal_x = (goal_x+100)*20
    goal_y = (goal_y+100)*20

    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, callback)
    global get_coordinates
    get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    rospy.sleep(2)
    base_coordinates = get_coordinates("mobile_base","")
    start_x = round(((base_coordinates.pose.position.x)+100)*20)
    start_y = round(((base_coordinates.pose.position.y)+100)*20)

    grid = read_map('main_map_enhanced.png')
    path = find_path(start_x,start_y,goal_x,goal_y,grid)
    print_path(path)
    execute_path(path)


def read_map(pgmf):
    with open(pgmf,'rb') as map_pgm:
        grid_map = plt.imread(map_pgm)

    return grid_map
        
if __name__ == '__main__':
    try:
        main_script()
    except rospy.ROSInterruptException:
        pass
