# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np


def saturation_function(velocity):
    if ((velocity / 0.8) > 6.28):
        return 6.28
    if ((velocity / 0.8) < -6.28):
        return -6.28
    return velocity / 0.8


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
kpf = 0.5
kps = 1.0
imu.enable(timestep)

st = robot.getTime()
desired_distance = 5
m = 999
# Main loop:
# perform simulation steps until Webots is stopping the controller

while robot.step(timestep) != -1:
    if (robot.getTime() - st > 30):
        break
    # Read the sensors:
    # Getting full Range Image from Lidar returns a list of 1800 distances = 5 layers X 360 distances
    full_range_image = lidar.getRangeImage()
    # print size of Range Image
    print('#################################################################')
    print("Lidar's Full Range Image Size: ", len(full_range_image))
    # Compare Distance Sensors to Lidar Ranges
    front_dist = frontDistanceSensor.getValue() * 39.37
    right_dist = rightDistanceSensor.getValue() * 39.37
    rear_dist = rearDistanceSensor.getValue() * 39.37
    left_dist = leftDistanceSensor.getValue() * 39.37

    print("Distance Sensor vs Lidar")
    print("\t", front_dist, "\t|", full_range_image[180] * 39.37)
    print("\t", right_dist, "\t|", full_range_image[270] * 39.3)
    print("\t", rear_dist, "\t|", full_range_image[359] * 39.37)
    print("\t", left_dist, "\t|", full_range_image[90] * 39.37)

    et = front_dist - desired_distance
    if (abs(et) < m):
        m = abs(et)
    v = saturation_function(kpf * et)
    if (left_dist < 3):
        et = left_dist - 3
        leftMotor.setVelocity(v)
        rightMotor.setVelocity(v - abs(et * kps))
    elif (right_dist < 3):
        et = right_dist - 3
        leftMotor.setVelocity(v - abs(et * kps))
        rightMotor.setVelocity(v)
    else:
        leftMotor.setVelocity(v)
        rightMotor.setVelocity(v)
    print(f"distance from wall: {full_range_image[180] * 39.37-1.4}")
    if (abs(et) < 0.01):
        print("simulation completed")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break
# Enter here exit cleanup code.
