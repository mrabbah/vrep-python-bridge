
#Import Libraries:
import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl   #used for image plotting

#Pre-Allocation

PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print 'Connected to remote API server'
    
else:
    print 'Connection not successful'
    sys.exit('Could not connect')


#retrieve motor  handles
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)

#retrieve target handles
errorCode,targethandler=vrep.simxGetObjectHandle(clientID,'Target',vrep.simx_opmode_oneshot_wait)
errorCode,targetPosition = vrep.simxGetObjectPosition(clientID, targethandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
print targetPosition

errorCode,myrobothandler=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)
errorCode,position = vrep.simxGetObjectPosition(clientID, myrobothandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
print position

deltaX = targetPosition[0] - position[0]
deltaY = targetPosition[1] - position[1]
distance = math.sqrt(math.pow(deltaX,2) + math.pow(deltaY,2))
print distance


v0 = 2

t = time.time()


while (time.time()-t)<60:
    print "------------------------"
    
    errorCode,position = vrep.simxGetObjectPosition(clientID, myrobothandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
    #print "--------------------"
    print "nouvelle position robot : ", position

    deltaX = targetPosition[0] - position[0]
    deltaY = targetPosition[1] - position[1]
    distance = math.sqrt(math.pow(deltaX,2) + math.pow(deltaY,2))
    print "distance = ", distance
    if(distance < 0.1) :
        break

    vLeft=v0
    vRight=v0
    
    for i in range(0,15+1) :
        vLeft=vLeft + braitenbergL[i]*sensor_val[i]
        vRight=vRight + braitenbergR[i]*sensor_val[i]
    
            
    
    print "V_l =",vLeft
    print "V_r =",vRight

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vLeft, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vRight, vrep.simx_opmode_streaming)

    


    time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)

#Post ALlocation
errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
  
# Stop simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
# Now close the connection to V-REP:
vrep.simxFinish(clientID)


