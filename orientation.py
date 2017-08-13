
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


errorCode,myrobothandler=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)
errorCode,eulerAngles = vrep.simxGetObjectOrientation(clientID, myrobothandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
print eulerAngles
  

t = time.time()


while (time.time()-t)<1:
    print "------------------------"
    #Loop Execution
    #sensor_val=np.array([])  
    
    vLeft=0
    vRight=1
    
            
    
    print "V_l =",vLeft
    print "V_r =",vRight

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vLeft, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vRight, vrep.simx_opmode_streaming)
    
    errorCode,eulerAngles = vrep.simxGetObjectOrientation(clientID, myrobothandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
    print eulerAngles

    time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)

#Post ALlocation
errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
  
# Stop simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
# Now close the connection to V-REP:
vrep.simxFinish(clientID)


