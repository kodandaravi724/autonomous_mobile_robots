# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math


def saturation_function(velocity):
    if ((velocity / 0.8) > 6.28):
        return 4.28
    if ((velocity / 0.8) < -6.28):
        return -5.28
    return velocity / 0.8


def followObject(distance, orientation):
    global motion_completed
    targetDistance = 5
    targetO = 42
    dif = orientation - targetO
    et = distance - 5
    kp = 1
    if (abs(et) <= 0.1):
        motion_completed = 1
        print("motion completed")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
    else:
        et = distance - 5
        v = saturation_function(kp * et)
        if (dif > 0):
            leftMotor.setVelocity(6.28)
            rightMotor.setVelocity(v)
        else:
            leftMotor.setVelocity(v)
            rightMotor.setVelocity(6.28)


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
motion_completed = 0

print("Lidar is enabled. \nLidar has a Horizontal Resolution of: ", lidar_horizontal_res,
      "\nLidar Image Number of Layers: ", lidar_num_layers)
print("Lidar Range: [", lidar_min_dist, " ,", lidar_max_dist, '] in meters')

#######################################################
# Gets Robots Camera
# Documentation:
#  https://cyberbotics.com/doc/reference/camera
#######################################################
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

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

# Main loop:
# perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1:
    if (motion_completed == 1):
        break
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

    # camera object recognition
    obj_in_view = len(camera.getRecognitionObjects())
    print("Objects in View: ", obj_in_view)

    if (obj_in_view > 0):
        pos_image = camera.getRecognitionObjects()[0].getPositionOnImage()[0]
        pos_view = camera.getRecognitionObjects()[0].getPosition()[0]
        followObject(pos_view * 39.3701, pos_image)
        print("\tPosition in Image:\t", pos_image)  # in pixels relative to image
        print("\tPosition in View:\t", pos_view * 39.3701)  # in meters relative to camera
    else:
        leftMotor.setVelocity(2)
        rightMotor.setVelocity(-2)
        continue

# Enter here exit cleanup code.
