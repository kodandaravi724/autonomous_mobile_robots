# import necessary modules
import math
from controller import Robot, Motor
from typing import List


# Define function to control the robot's movement
def robotControl(A: List[List[float]]):
    startTime = robot.getTime()
    for v in A:
        # check if left and right motor velocities exceed maximum limit
        if abs(v[0]) > leftMotor.getMaxVelocity():
            print(f"Error: Left motor velocity exceeds maximum limit. Max angular velocity of left motor is {leftMotor.getMaxVelocity()} radians/sec. Hence Skipping this vector!")
            continue
        if abs(v[1]) > rightMotor.getMaxVelocity():
            print(f"Error: Right motor velocity exceeds maximum limit. Max angular velocity of right motor is {rightMotor.getMaxVelocity()} radians/sec. Hence Skipping this vector!")
            continue
        elapsedTime = robot.getTime()
        maxRunTime = elapsedTime + v[2]
        st = robot.getTime()
        # run the simulation until maxRunTime is reached
        while robot.step(1) != -1:
            elapsedTime = robot.getTime()
            if elapsedTime < maxRunTime:
                leftMotor.setVelocity(v[0])
                rightMotor.setVelocity(v[1])
            else:
                print(f"Elapsed time for vector [{v[0]},{v[1]},{v[2]}] is {elapsedTime-st} sec")
                break
    print(f"Total elapsed time in simulation: {elapsedTime-startTime} seconds")
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

    print("simulation completed")

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()


    leftMotor:Motor = robot.getDevice('left wheel motor')
    rightMotor:Motor = robot.getDevice('right wheel motor')

    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))

    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

    # Main loop:
    # perform simulation steps until Webots is stopping the controller

    A = [[4, 0, 7], [6.2, -6, 3], [8, -6, 5], [1, -6, 4], [4, -6, 6],[4,5,6],[2,4,3],[5,-5,4],[4,1,5],[6,-1,4]]

    robotControl(A)

