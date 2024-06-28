# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
# import numpy as it may be used in future labs
import numpy as np
import math
kp=1
time = 0
rotate_45 = 0


def saturation_function1(velocity):
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
    print(saturation_function1(vl),vl, saturation_function1(vr),vr)
    leftMotor.setVelocity(saturation_function1(vl))
    rightMotor.setVelocity(saturation_function1(vr))


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

def saturation_function(velocity):
    if ((velocity / 0.8) > 6.28):
        return 4.28
    if ((velocity / 0.8) < -6.28):
        return -5.28
    return velocity / 0.8


def followObject(distance, orientation):
    global motion_completed
    print("ok")
    targetDistance = 5
    targetO = 42
    dif = orientation - targetO
    et = distance - 5
    print(dif, "diff", et)
    kp = 1
    if (abs(et) <= 0.1):
        motion_completed = 1
        print("motion completed")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
    else:
        et = distance - 5
        v = saturation_function(kp * et)
        print(v, orientation)
        if (dif > 0):
            leftMotor.setVelocity(6.28)
            rightMotor.setVelocity(v)
        else:
            print("here")
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
mtw=1
#######################################################
# Gets Robot's IMU sensors
# Documentation:
#  https://cyberbotics.com/doc/reference/inertialunit
#######################################################
imu = robot.getDevice('inertial unit')
imu.enable(timestep)
follow_wall = 'l'
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
        mtw=1
        pos_image = camera.getRecognitionObjects()[0].getPositionOnImage()[0]
        pos_view = camera.getRecognitionObjects()[0].getPosition()[0]
        a = camera.getRecognitionObjects()[0]
        for o in a.getOrientation():
            print(o)
        followObject(pos_view * 39.3701, pos_image)
        print("\tPosition in Image:\t", pos_image)  # in pixels relative to image
        print("\tPosition in View:\t", pos_view * 39.3701)  # in meters relative to camera
    else:
        if(mtw==1):
            print("here")
            rotate_45=1
            time = 1.79/4
            mtw=0
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

            wall_follow(left_dist, right_dist, front_dist, 1, follow_wall)
        continue

# Enter here exit cleanup code.
