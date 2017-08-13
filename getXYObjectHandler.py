
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

errorCode,myrobothandler=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)
errorCode,position = vrep.simxGetObjectPosition(clientID, myrobothandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
print position

errorCode,targethandler=vrep.simxGetObjectHandle(clientID,'Target',vrep.simx_opmode_oneshot_wait)
errorCode,targetPosition = vrep.simxGetObjectPosition(clientID, targethandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
print targetPosition

deltaX = targetPosition[0] - position[0]
deltaY = targetPosition[1] - position[1]
distance = math.sqrt(math.pow(deltaX,2) + math.pow(deltaY,2))

# Stop simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
# Now close the connection to V-REP:
vrep.simxFinish(clientID)

print distance


