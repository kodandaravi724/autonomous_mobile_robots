# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np

kp = 1
time = 0
rotate_45 = 0


def saturation_function(velocity):
    if (velocity == -1):
        return -1 / 0.8
    if (velocity == 1):
        return 1 / 0.8
    if ((velocity / 0.8) > 6.28):
        return 6.28
    if ((velocity / 0.8) < 1):
        return 1 / 0.8
    return velocity / 0.8


def setSpeed(vl, vr):
    print(saturation_function(vl), saturation_function(vr))
    leftMotor.setVelocity(saturation_function(vl))
    rightMotor.setVelocity(saturation_function(vr))


def wall_follow(left_distance, right_distance, front_distance, target, follow_wall):
    global rotate_45
    global time
    v = 5
    if (follow_wall == 'l'):
        if (front_distance < 2):
            rotate_45 = 1
            time = 1.79 / 2
            pass
        error = left_distance - target
        print(error, "err")
        if (error < 0):
            setSpeed(v, v - abs(error) * kp)
            # setSpeed(v - abs(error) * kp,v)
        else:
            print("TooFar")
            # setSpeed(v,v-abs(error)*kp)
            setSpeed(v - abs(error) * kp, v)
    else:
        if (front_distance < 2):
            rotate_45 = 1
            time = 1.79 / 2
            pass
        error = right_distance - target
        print(error, "err")
        if (error < 0):
            setSpeed(v - abs(error) * kp,v)
            # setSpeed(v - abs(error) * kp,v)
        else:
            print("TooFar")
            # setSpeed(v,v-abs(error)*kp)
            setSpeed(v,v - abs(error) * kp)

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
kps = [0.8]
imu.enable(timestep)
fl = 0
intial = 0
cn = 0
follow_wall = 'l'
for kp in kps:
    st = robot.getTime()
    # Main loop:
    # perform simulation steps until Webots is stopping the controller

    while robot.step(timestep) != -1:
        if (rotate_45 == 1):
            if (time > 0):
                if(follow_wall=='l'):
                    setSpeed(1, -1)
                else:
                    setSpeed(-1, 1)
                time = time - 0.032
                print(time, 'time')
                continue
            else:
                rotate_45 = 0
                time = 0
        else:

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
            print("\t", front_dist, "\t|", full_range_image[180] * 39.37, "front")
            print("\t", right_dist, "\t|", full_range_image[270] * 39.37, "right")
            print("\t", rear_dist, "\t|", full_range_image[359] * 39.37, "rear")
            print("\t", left_dist, "\t|", full_range_image[90] * 39.37, "left")

            wall_follow(left_dist, right_dist, front_dist, 0.8, follow_wall)

# Enter here exit cleanup code.
