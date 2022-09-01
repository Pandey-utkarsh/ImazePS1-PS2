"""iMaze ps 2 remote sol 1.0 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
import numpy as np
import cv2
import math
import time
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

prox_sense = []
for i in range(8):
    prox_sense.append(robot.getDevice('ps'+str(i)))
    prox_sense[i].enable(timestep)

max_speed = 6.27

camera = robot.getDevice('camera')
camera.enable(1)


def moveForward(speed):
    if (speed > 100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)


def turnright(speed, rng, sharp):
    if (speed > 100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(rng):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed*sharp)


def turnleft(speed, rng, sharp):
    if (speed > 100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(rng):
        left_motor.setVelocity(speed*sharp)
        right_motor.setVelocity(speed)


lower_colors = np.array(
    [[50, 150, 0], [110, 150, 0], [130, 123, 0], [160, 150, 0]])
upper_colors = np.array([[70, 255, 255], [130, 255, 255], [
                        160, 255, 255], [179, 255, 255]])


i = 0
t1 = time.time()
t2 = t1
flag = False
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    fl = math.floor(prox_sense[7].getValue())
    fr = math.floor(prox_sense[0].getValue())
    f = fl+fr
    lf = math.floor(prox_sense[6].getValue())
    rf = math.floor(prox_sense[1].getValue())
    l = math.floor(prox_sense[5].getValue())
    r = math.floor(prox_sense[2].getValue())

    # contour detection
    camera.saveImage('x.jpg', 100)
    img = cv2.imread('x.jpg')
    img = cv2.resize(img, (0, 0), fx=0.75, fy=0.75)
    img = cv2.medianBlur(img, 3)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_hsv = cv2.inRange(hsv, lower_colors[i], upper_colors[i])
    masked_img = cv2.bitwise_and(img, img, mask=mask_hsv)
    contours, gbg = cv2.findContours(
        mask_hsv, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if (len(contours) >= 1):
        contours = sorted(
            contours, key=lambda x: cv2.contourArea(x), reverse=True)
        if (cv2.contourArea(contours[0]) > 100):
            M = cv2.moments(contours[0])
            if (M['m00'] != 0):
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                if (lf > 150):
                    turnright(100, 10, -1)
                elif (rf > 150):
                    turnleft(100, 10, -1)
                elif (cx < 180):
                    turnleft(100, 5, 0.75)
                elif (cx > 210):
                    turnright(100, 5, 0.75)
                else:
                    moveForward(100)

            if (f > 200 and cv2.contourArea(contours[0]) > 100000):
                i += 1
    elif i == 0:
        if (time.time()-t2 >= 12 and flag == 0):
            t2 = time.time()
            flag = 1
        elif (time.time()-t2 >= 3 and flag == 1):
            t2 = time.time()
            flag = 0
        if (flag):
            moveForward(100)
        else:
            turnleft(100, 10, -1)
    else:
        if (f < 150 and l > 80):
            moveForward(100)

        if (f < 150 and l <= 80 and lf < 80):
            turnleft(100, 10, 0.3)
        else:
            turnright(100, 10, -1)
        continue
    # cv2.imshow("masked image",masked_img)
    # cv2.imshow("Blue Mask",mask_blue)
    # cv2.imshow("HSV Mask",mask_hsv)
    # cv2.imshow("HSV",hsv)
    # print(l,lf,f,rf,r)
    # k = cv2.waitKey(1) #& 0xFF
    # if k == 27:
    #    break

    if (i == 4):
        break
    pass
moveForward(0)
cv2.destroyAllWindows()
