# You may need to import some classes of the controller module. Ex:
import typing

#  from controller import Robot, Motor, DistanceSensor

from controller import Robot
import pickle

# import numpy as it may be used in future labs

import numpy as np

import math


goal = 6
#######################################################

# Creates Robot

#######################################################

robot = Robot()

#######################################################

# Sets the time step of the current world

#######################################################

timestep = int(robot.getBasicTimeStep())

#######################################################

# Gets Robots Distance Sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/distancesensor

#######################################################

frontDistanceSensor = robot.getDevice('front distance sensor')

leftDistanceSensor = robot.getDevice('left distance sensor')

rightDistanceSensor = robot.getDevice('right distance sensor')

rearDistanceSensor = robot.getDevice('rear distance sensor')

frontDistanceSensor.enable(timestep)

leftDistanceSensor.enable(timestep)

rightDistanceSensor.enable(timestep)

rearDistanceSensor.enable(timestep)

#######################################################

# Gets Robots Lidar Distance Sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/lidar

#######################################################

lidar = robot.getDevice('lidar')

lidar.enable(timestep)

lidar_horizontal_res = lidar.getHorizontalResolution()

lidar_num_layers = lidar.getNumberOfLayers()

lidar_min_dist = lidar.getMinRange()

lidar_max_dist = lidar.getMaxRange()

print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res,
      "\nLidar Image Number of Layers: ", lidar_num_layers)

print("Lidar Range: [", lidar_min_dist, " ,", lidar_max_dist, '] in meters')

#######################################################

# Gets Robots Camera

# Documentation:

#  https://cyberbotics.com/doc/reference/camera

#######################################################

camera_front = robot.getDevice('cameraFront')

camera_front.enable(timestep)

camera_front.recognitionEnable(timestep)

camera_right = robot.getDevice('cameraRight')

camera_right.enable(timestep)

camera_right.recognitionEnable(timestep)

camera_rear = robot.getDevice('cameraRear')

camera_rear.enable(timestep)

camera_rear.recognitionEnable(timestep)

camera_left = robot.getDevice('cameraLeft')

camera_left.enable(timestep)

camera_left.recognitionEnable(timestep)

#######################################################

# Gets Robots Motors

# Documentation:

#  https://cyberbotics.com/doc/reference/motor

#######################################################

leftMotor = robot.getDevice('left wheel motor')

rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))

rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0)

rightMotor.setVelocity(0)

#######################################################

# Gets Robot's the position sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/positionsensor

#######################################################

leftposition_sensor = robot.getDevice('left wheel sensor')

rightposition_sensor = robot.getDevice('right wheel sensor')

leftposition_sensor.enable(timestep)

rightposition_sensor.enable(timestep)

#######################################################

# Gets Robot's IMU sensors

# Documentation:

#  https://cyberbotics.com/doc/reference/inertialunit

#######################################################

imu = robot.getDevice('inertial unit')

imu.enable(timestep)


def set_velocity(l, r):
    leftMotor.setVelocity(l / 0.8)
    rightMotor.setVelocity(r / 0.8)

dir_90 = -1
dir_time = -1
def turn_90_degrees():
    global dir_time
    global dir_90
    print("tunring the robot to 90 degrees")
    d = 2.28
    while(robot.step(timestep)!=-1):
        if(dir_90!=-1):
            if(dir_90=='a'):
                if(dir_time>=0):
                    set_velocity(1,-1)
                    dir_time = dir_time - 0.032
                    continue
                else:
                    print("completed")
                    dir_90=-1
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
            else:
                if(dir_time>=0):
                    set_velocity(1,-1)
                    dir_time = dir_time - 0.032
                    continue
                else:
                    print("completed")
                    dir_90=-1
                    leftMotor.setVelocity(0)
                    rightMotor.setVelocity(0)
                    break
        p = imu.getRollPitchYaw()[2]
        if (p < 0):
            p = p + 6.28
        diff = p - 1.57
        if(diff<0):
            omega = (2)/d
            dir_time = abs(diff)/omega
            dir_90 = 'a'
        else:
            omega = (2) / d
            dir_time = abs(diff) / omega
            dir_90 = 'c'


# Main loop:
# perform simulation steps until Webots is stopping the controller


def update_pose(x,y,angle,distance):
    import math
    if(angle<0):
        angle = angle+6.28
    x = x+distance*math.cos(angle)
    y = y+distance*math.sin(angle)
    return [x,y]


def rotate_90(dir):
    omega = 2 / (2.28)
    time_r = (1.57) / omega
    if(dir=='a'):
        while(robot.step(timestep)!=-1):
            if(time_r>0):
                set_velocity(-1,1)
                time_r = time_r-0.032
            else:
                set_velocity(0,0)
                break
    if (dir == 'c'):
        while (robot.step(timestep) != -1):
            if (time_r > 0):
                set_velocity(1, -1)
                time_r = time_r-0.032
            else:
                set_velocity(0, 0)
                break
def getCell(x,y):

    if (x <= -10 and x >= -20 and y >= 10 and y <= 20):
        return 1
    elif (x <= 0 and x >= -10 and y >= 10 and y <= 20):
        return 2
    elif (x >= 0 and x <= 10 and y >= 10 and y <= 20):
        return 3
    elif (x >= 10 and x <= 20 and y >= 10 and y <= 20):
        return 4
    elif (x <= -10 and x >= -20 and y >= 0 and y <= 10):
        return 5
    elif (x <= 0 and x >= -10 and y >= 0 and y <= 10):
        return 6
    elif (x >= 0 and x <= 10 and y >= 0 and y <= 10):
        return 7
    elif (x >= 10 and x <= 20 and y >= 0 and y <= 10):
        return 8
    elif (x <= -10 and x >= -20 and y <= 0 and y >= -10):
        return 9
    elif (x <= 0 and x >= -10 and y <= 0 and y >= -10):
        return 10
    elif (x >= 0 and x <= 10 and y <= 0 and y >= -10):
        return 11
    elif (x >= 10 and x <= 20 and y <= 0 and y >= -10):
        return 12
    elif (x <= -10 and x >= -20 and y <= -10 and y >= -20):
        return 13
    elif (x <= 0 and x >= -10 and y <= -10 and y >= -20):
        return 14
    elif (x >= 0 and x <= 10 and y <= -10 and y >= -20):
        return 15
    elif (x >= 10 and x <= 20 and y <= -10 and y >= -20):
        return 16


def move_one_cell(dist):
    global X
    global Y
    time = 2
    while(robot.step(timestep)!=-1):
        front_dist = frontDistanceSensor.getValue()
        if(front_dist*39.37<3):
            time = -1
        if(time>0):
            set_velocity(5,5)
            dif_dist = leftposition_sensor.getValue() - dist
            dist = leftposition_sensor.getValue()
            if X is not None:
                X,Y = update_pose(X,Y,imu.getRollPitchYaw()[2],dif_dist*0.8)
                print('-----Robot Pose-------')
                print(f'X = {X}')
                print(f'Y = {Y}')
                print(f'Theta = {getTheta(imu.getRollPitchYaw()[2])}')
                print(f'cell = {getCell(X,Y)}')
                print('-----------------------')
            time = time - 0.032
        else:
            break

def getTheta(angle):
    if(angle<0):
        angle = angle+6.28
    import math
    return math.degrees(angle)

def move(start, dest):
    diff = start - dest
    rotation = None
    if(diff == 0):
        return
    if(abs(diff)==1):
        if(diff==-1):
            rotation = 'c'
        else:
            rotation = 'a'
    if(abs(diff)==4):
        if (diff==-4):
            rotation = 'cc'
    if rotation is not None:
        for d in rotation:
            rotate_90(d)
    leftDistance = None
    while (robot.step(timestep) != -1):
        leftDistance = leftposition_sensor.getValue()
        break
    move_one_cell(dist = leftDistance)
    time = 1
    while(robot.step(timestep)!=-1):
        if(time>0):
            set_velocity(0,0)
            time = time - 0.032
        else:
            break
    if(rotation is not None):
        for d in rotation:
            if(d=='c'):
                rotate_90('a')
            if(d=='a'):
                rotate_90('c')
    time = 0.5
    while(robot.step(timestep)!=-1):
        if(time>0):
            set_velocity(0,0)
            time = time - 0.032
        else:
            break


def printPath(path):
    res = ""
    i=0
    for i in range(len(path)-1):
        res = res + f"Cell {path[i]} -> "

    res = res + f"Cell {path[i]} (Goal)"
    print(res)
def get_neighbours(position):
    while(robot.step(timestep)!=-1):
        front_dist = frontDistanceSensor.getValue()

        right_dist = rightDistanceSensor.getValue()

        left_dist = leftDistanceSensor.getValue()

        rear_dist = rearDistanceSensor.getValue()

        res = [left_dist*39.37>10,front_dist*39.37>10,  right_dist*39.37>10, rear_dist*39.37>10]

        result = []
        mapState[position]=""
        if(res[0]==True):
            result.append(position - 1)
            mapState[position] += "O"
        else:
            mapState[position] +="W"
        if (res[1] == True):
            result.append(position - 4)
            mapState[position] += "O"
        else:
            mapState[position] += "W"
        if (res[2] == True):
            result.append(position + 1)
            mapState[position] += "O"
        else:
            mapState[position] += "W"
        if (res[3] == True):
            result.append(position + 4)
            mapState[position] += "O"
        else:
            mapState[position] += "W"
        return result

map = {}
mapState={}

def perform_dfs_recursive(position, visited):
    visited.append(position)
    neighbours = get_neighbours(position)
    map[position] = neighbours
    for n in neighbours:
        if n not in visited:
            move(position,n)
            visited.append(n)
            perform_dfs_recursive(n,visited)
            move(n,position)

def normaLizePoints(diction,mapState1):
    value = min(diction.keys())
    factor = 1 - value
    res= {}
    for k in diction.keys():
        res[k+factor]=[]
        for j in diction[k]:
            res[factor+k].append(factor+j)
    res1={}
    for k in mapState1:
        res1[factor+k]=mapState[k]
    return (res,res1,factor)

wavefrontDict = {}
import queue
def applyWaveFrontDict(waveFrontDict, dest):
    q = queue.Queue()
    q.put(dest)

    while not q.empty():
        t = q.get()
        for i in map1[t]:
            print(i,t)
            if(waveFrontDict[i]==0):
                wavefrontDict[i] = wavefrontDict[t]+1
                q.put(i)
    return waveFrontDict
def waveFrontPlanner(start, dest):
    for i in range(1,17):
        wavefrontDict[i] = 0
    wavefrontDict[dest] = 2
    return applyWaveFrontDict(wavefrontDict,dest)

def getPathToTraverse(waveFrontD,source,dest,path):
    if(waveFrontD[source]==0):
        print("there was no path from source to dest")
    path.append(source)
    if(source==dest):
        return
    neighbours = map1[source]
    m = 99999
    for n in neighbours:
        if(waveFrontD[n]<m):
            m = waveFrontD[n]
            nodeToTraverse = n
    # path.append(nodeToTraverse)
    getPathToTraverse(waveFrontD,nodeToTraverse,dest,path)


def getCoorinates(cell):
    X = -15 + ((cell-1)%4)*10
    Y = 15 - ((cell-1)//4)*10
    return [X,Y]


def navigateToGoal(X,Y,path):
    for i in range(len(path)-1):
        move(path[i],path[i+1])

X=None
Y =None

while robot.step(timestep) != -1:

    # Read the sensors:

    # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances

    full_range_image = lidar.getRangeImage()

    # print size of Range Image

    print('#################################################################')

    print("Lidar's Full Range Image Size: ", len(full_range_image))

    # Compare Distance Sensors to Lidar Ranges

    front_dist = frontDistanceSensor.getValue()

    right_dist = rightDistanceSensor.getValue()

    rear_dist = rearDistanceSensor.getValue()

    left_dist = leftDistanceSensor.getValue()

    print("Distance Sensor vs Lidar")

    print("\tFront:\t", front_dist, "\t|", full_range_image[0])

    print("\tRight:\t", right_dist, "\t|", full_range_image[90])

    print("\tRear:\t", rear_dist, "\t|", full_range_image[180])

    print("\tLeft:\t", left_dist, "\t|", full_range_image[270])

    turn_90_degrees()
    perform_dfs_recursive(1,[])
    break

map1 = normaLizePoints(map,mapState)[0]
mapState = normaLizePoints(map,mapState)[1]
startingLocation = normaLizePoints(map,mapState)[2] + 1

for key in sorted(map1.keys()):
    print(key, map1[key])

for key in sorted(mapState.keys()):
    print(key, mapState[key])

wavefrontDict = waveFrontPlanner(startingLocation,goal)

path = []
getPathToTraverse(wavefrontDict,startingLocation,goal,path)


print('Path to be Navigated\n')
# print('-->'.join(map(str,path)))
printPath(path)
X,Y = getCoorinates(startingLocation)

navigateToGoal(X,Y,path)

# Enter here exit cleanup code