# Team ALU Remote code for Problem Statement I.

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

max_speed = 6.27

prox_sensor = []
for i in range(8):
    name = 'ps' + str(i)
    prox_sensor.append(robot.getDevice(name))
    prox_sensor[i].enable(timestep)
    
def moveForward(speed):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(10):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)

def turnright(speed,rng):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(rng):
        left_motor.setVelocity(speed)
        right_motor.setVelocity(-speed)
def turnleft(speed,rng):
    if(speed>100):
        speed = 100
    speed = (max_speed/100.0)*speed
    for _ in range(rng):
        left_motor.setVelocity(-speed)
        right_motor.setVelocity(speed)
# Main loop:

# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    f=math.floor(prox_sensor[0].getValue()+prox_sensor[7].getValue())
    l=math.floor(prox_sensor[5].getValue())
    r=math.floor(prox_sensor[2].getValue())
    lf=math.floor(prox_sensor[6].getValue())
    rf=math.floor(prox_sensor[1].getValue())

    print(l,lf,f,rf,r)
    if(f<=150):
         moveForward(100)
         if(lf>80 and r<l):
                turnright(100,5)
         elif(rf>80 and l<r):
                 turnleft(100,5)        
    else:     
         if(l>r or (lf>rf and r-l<10)):
                 turnright(100,10)
         elif(r>l or (rf>lf and l-r<10)):
                 turnleft(100,10)
                       
    if(f>150 and (l+r>220 and (l>80 and r>80))):
         break


    pass
moveForward(0)

# Enter here exit cleanup code.
