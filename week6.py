# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math
import sys
import random # need in order to perform the random wander
# import matplotlib as mpl    #used for image plotting;

class Robot():
    
    # def __init__(self) -> None:
    def __init__(self):
                
        # Setup Motors
        res, self.leftMotor = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
        res, self.rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

        # Setup Sonars
        res, self.frontLeftSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_blocking)
        res, self.Front_50_RightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor7',sim.simx_opmode_blocking)
        res, self.RightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8',sim.simx_opmode_blocking)
        res, self.backRightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor9',sim.simx_opmode_blocking)

         # Start Sonars
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.frontLeftSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.Front_50_RightSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.RightSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.backRightSonar,sim.simx_opmode_streaming)

        time.sleep(2)


        #Starting Sensors, front and back

    def getDistanceReading(self, objectHandle):
        # Get reading from sensor
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,objectHandle,sim.simx_opmode_buffer)

        if detectionState == 1:
            # return magnitude of detectedPoint
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            # resturn another value that we know cannon be true and handle it (use a large number so that if you do 'distance < reading' it will work)
            return 9999

    def move(self, velocity):
        # velocity < 0 = reverse
        # velocity > 0 = forward
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, velocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, velocity, sim.simx_opmode_blocking)

    def turn(self, turnVelocity):
        # turnVelocity < 0 = trun left
        # turnVelocity > 0 = turn right
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, turnVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, turnVelocity, sim.simx_opmode_blocking)
        
    def curveCorner(self, leftMotorVelocity, rightMotorVelocity):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, sim.simx_opmode_blocking)

    def stop(self):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, sim.simx_opmode_blocking)

    def slowdown(self):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, 0.1, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, 0.1, sim.simx_opmode_blocking)
        
print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')

    robot = Robot()
    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    for i in range(10):
        print(robot.getDistanceReading(robot.frontLeftSonar))
        print(robot.getDistanceReading(robot.Front_50_RightSonar))
        print(robot.getDistanceReading(robot.RightSonar))
        print(robot.getDistanceReading(robot.backRightSonar))
        
        time.sleep(0.2)

    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    # Function used to return a random integer within the range of 0 and 100 for the random wander
    randomNumber = random.randint(0, 100)    

    #------------------------------------------#

    # PID Controller value
    # Kp = 0.05 # Proportional Gain
    # Ki = 0.0 # Integral Gain
    # Kd = 0.02 # Derivative Gain

    Kp = 2 # Proportional Gain
    Ki = 0.05 # Integral Gain
    Kd = 5 # Derivative Gain
    Dt = 0.2 # Set Time stable

    #------------------------------------------#

    # Variables
    cp = 0
    current_error = 0
    previous_error = 0
    sum_error = 0
    previous_error_derivative = 0
    current_error_derivative = 0

    #------------------------------------------#

    # Distance variables
    d_R_front = robot.getDistanceReading(robot.RightSonar)
    d_R_back = robot.getDistanceReading(robot.backRightSonar)

    #calculate distance
    dist_robot_and_wall = d_R_front + d_R_back / 2 # distance between robot and wall
    angle = d_R_front - d_R_back # angel of the wall
    dist_to_wall = math.cos(angle) == (d_R_front / d_R_back) # desired distance between robot and wall
    
    #------------------------------------------#

    # Loop execution (nested while statement)
    while True:
        # the robot will move untill it detects an object
        robot.move(1)

        #------------------------------------------#
        
        # Setting Sonar ( detectionState(R) )
        res,detectionStateR,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,robot.backRightSonar,sim.simx_opmode_buffer)

        # calculate
        current_error = dist_to_wall - dist_robot_and_wall
        sum_error = sum_error + (current_error * Dt)
        current_error_derivative = (current_error - previous_error) / Dt
        previous_error = current_error

        #------------------------------------------#

        # PID Process
        cp = (Kp * current_error) + (Ki * sum_error) + (Kd * current_error_derivative)


        # Process Coding
        cp = max(min(0.0, cp), 1.0) # limiting the output of values 0 - 1
        cp = abs(cp)            # used to make any output value a positive value
        print("error {} cp {} distance {}".format(current_error, cp, dist_robot_and_wall))
        
        #------------------------------------------#

        # if...elif ..else statements that allows us to check for multiple expressions
        if robot.getDistanceReading(robot.frontLeftSonar) <= 1: #threshold value
            robot.curveCorner(-cp, cp) # velocity adjectment
            print("Wall detected in (L)front cp")  # the robot will turn if a wall is detected

        elif robot.getDistanceReading(robot.Front_50_RightSonar) <= dist_to_wall:
            robot.curveCorner(-cp, cp)  # velocity adjustment
            print("Wall detected in Right_front_50_degrees ") # the robot will trun if a wall is detected;

        elif robot.getDistanceReading(robot.backRightSonar) == dist_to_wall and robot.getDistanceReading(robot.RightSonar) == dist_to_wall and detectionStateR == 1.0: # threshold value
            robot.move(1)   #velecity adjustment
            print("--------- [B-F] Perfectly Forward fix velocity ---------")   # the robot will turn to adjust direction

        elif robot.getDistanceReading(robot.backRightSonar) <= dist_to_wall and detectionStateR == 1.0:
            robot.curveCorner(cp, angle)  # velocity adjustment
            print("fix position on the Right (1)") # the robot will turn if detected on the right within the threshold value
       
        elif robot.getDistanceReading(robot.backRightSonar) >= dist_to_wall and detectionStateR == 1.0:
            robot.curveCorner(angle, cp)  # velocity adjustment
            print("fix position on the Left (angle,cp)") # the robot will turn to adjust direction

        elif robot.getDistanceReading(robot.backRightSonar) >= dist_to_wall:
            robot.curveCorner(cp, cp)  # velecity adjustment
            print("Maintain position on the Left (cp, cp)") # the robot will turn to adjust direction 

        elif robot.getDistanceReading(robot.Front_50_RightSonar) <= dist_to_wall and detectionStateR == 1.0:
            robot.curveCorner(cp, angle)
            print("Change direction to the Right")

        elif robot.getDistanceReading(robot.Front_50_RightSonar) >= dist_to_wall and detectionStateR == 1.0:
            robot.curveCorner(angle, cp)
            print("fix position to the Left #1")    

        elif robot.getDistanceReading(robot.Front_50_RightSonar) >= dist_to_wall :
            robot.curveCorner(cp, cp)
            print("fix position to the Left #2")

        elif robot.getDistanceReading(robot.RightSonar) == dist_to_wall and robot.getDistanceReading(robot.backRightSonar) == dist_to_wall and detectionStateR == 1.0:
            robot.move(1)
            print("--------- [F-&-D] Perfectly forward to fix velocity ---------")   

        elif robot.getDistanceReading(robot.RightSonar) <= dist_to_wall and detectionStateR == 1.0:
            robot.curveCorner(cp, angle)
            print("Change direction to the Right")  

        elif robot.getDistanceReading(robot.RightSonar) >= dist_to_wall and detectionStateR == 1.0:
                robot.curveCorner(angle, cp)
                print("fix position to the Left")

        elif robot.getDistanceReading(robot.RightSonar) >= dist_to_wall :
                robot.curveCorner(cp, cp)
                print("fix position to the Left")      

    


        # elif robot.getDistanceReading(robot.RightSonar) <= 0.40:
        #     # robot.stop()
        #     # robot.curveCorner(0.25, 0.35)             
        #     robot.curveCorner(0.85, 1)             
        #     # robot.move(0.5)           
        #     print("0.40 Change direction to the Left") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.RightSonar) <= 0.45:
        #     # robot.stop()
        #     # robot.curveCorner(0.25, 0.35)             
        #     robot.curveCorner(2, 0.15)             
        #     # robot.move(0.5)           
        #     print("0.45 Change direction to the Left") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.RightSonar) <= 0.50:
        #     # robot.stop()
        #     # robot.curveCorner(0.40, 0.30)    
        #     robot.curveCorner(0.85, 1)    
        #     # robot.move(0.5)                    
        #     print("0.50 Change direction to the Left") # the robot will turn to adjust direction;
            
        # elif robot.getDistanceReading(robot.RightSonar) <= 0.60:
        #     # robot.stop()
        #     # robot.curveCorner(0.35, 0.25)                        
        #     robot.curveCorner(2, 0.25)                        
        #     print("0.60 Change direction to the Left") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.RightSonar) <= 2:
        #     robot.stop()
        #     robot.curveCorner(3, 0.1)                        
        #     print("Big TurnRight") # the robot will turn to adjust direction;

        # elif robot.getDistanceReading(robot.backRightSonar) <= 2:
        #     robot.stop()
        #     robot.curveCorner(3, 0.1)
        #     print("Searching the wall")
        #     # print(robot.getDistanceReading(robot.backRightonar))

        else: 
            robot.curveCorner(randomNumber - 1, randomNumber + 1) # The robot will take a random turn untill it detects an object;
        robot.stop()

else:
    print ('Failed connecting to remote API server')
    print ('Program ended')
sys.exit('Could not connect')