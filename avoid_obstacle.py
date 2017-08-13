
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


sensor_h=[None] * 16 #empty list for handles
sensor_val=[None] * 16 #empty array for sensor measurements

#orientation of all the sensors: 
sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 

braitenbergL=[-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR=[-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
v0 = 2
maxDistanceDetection=0.5
minDistanceDetection=0.2

#for loop to retrieve sensor arrays and initiate sensors
for x in range(0,15+1):
        #print x
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor'+str(x+1),vrep.simx_opmode_oneshot_wait)
        sensor_h[x] = sensor_handle #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming)                
        #print detectedPoint
        #distance_val = 1-((detectedPoint[2]-minDistanceDetection)/(maxDistanceDetection-minDistanceDetection))  
        sensor_val[x]= 0 
        #sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values
print sensor_h   
print sensor_val    

t = time.time()


while (time.time()-t)<60:
    print "------------------------"
    #Loop Execution
    #sensor_val=np.array([])  
    for x in range(0,15+1):
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x],vrep.simx_opmode_buffer)      
        #errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x-1],vrep.simx_opmode_buffer) 
        #print detectedPoint
        #print sensor_h[x]
        if detectionState == True :
            print "detected distance = ",detectedPoint[2] 
            if(detectedPoint[2] > maxDistanceDetection or detectedPoint[2] < minDistanceDetection) :
                sensor_val[x]= 0
            else :
                #print "Distance entre capteur ",x," et l'objet : ", detectedObjectHandle, " = " , detectedPoint[2]
                distance_val = 1-((detectedPoint[2]-minDistanceDetection)/(maxDistanceDetection-minDistanceDetection))  
                print "Distance entre capteur ",x," et l'objet : ", detectedObjectHandle, " = " , distance_val
                print distance_val
                sensor_val[x]= distance_val 
        else :
            print "Le capteur ",x," n a pas detecte d obstacle"
            sensor_val[x]= 0

         #get list of values
    
    vLeft=v0
    vRight=v0
    
    for i in range(0,15+1) :
        vLeft=vLeft + braitenbergL[i]*sensor_val[i]
        vRight=vRight + braitenbergR[i]*sensor_val[i]
    
            
    
    print "V_l =",vLeft
    print "V_r =",vRight

    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,vLeft, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,vRight, vrep.simx_opmode_streaming)

    errorCode,position = vrep.simxGetObjectPosition(clientID, myrobothandler, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
    #print "--------------------"
    print "nouvelle position robot : ", position

    deltaX = targetPosition[0] - position[0]
    deltaY = targetPosition[1] - position[1]
    distance = math.sqrt(math.pow(deltaX,2) + math.pow(deltaY,2))
    print "distance = ", distance


    time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)

#Post ALlocation
errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0, vrep.simx_opmode_streaming)
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0, vrep.simx_opmode_streaming)
  
# Stop simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
# Now close the connection to V-REP:
vrep.simxFinish(clientID)


