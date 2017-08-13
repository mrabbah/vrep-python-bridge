#Import Libraries:
import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math
import matplotlib as mpl

epsilon = 0.05

def getAngle(clientID, oh, toh) :
	errorCode,op = vrep.simxGetObjectPosition(clientID, oh, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
	print "OPX = " , op[0] , " OPY = " , op[1]
	errorCode,tp = vrep.simxGetObjectPosition(clientID, toh, vrep.sim_handle_parent, vrep.simx_opmode_oneshot_wait)
	print "TPX = " , tp[0] , " TPY = " , tp[1]
	tangentGamma = op[1] - tp[1] / op[0] - tp[0]
	gamma = math.atan(tangentGamma)
	print 'Atan = ', gamma
	return gamma



def tourner(clientID, oh, toh, lmh, rmh):
	errorCode,o_angles = vrep.simxGetObjectOrientation(clientID, oh, toh, vrep.simx_opmode_oneshot_wait)
	#errorCode,t_angles = vrep.simxGetObjectOrientation(clientID, toh, -1, vrep.simx_opmode_oneshot_wait)
	o_gamma = o_angles[2]
	t_gamma = getAngle(clientID, oh, toh)
	delta = math.fabs(o_gamma - t_gamma)
	print 'delta = ', delta
	while delta > epsilon :
		errorCode=vrep.simxSetJointTargetVelocity(clientID,lmh,2, vrep.simx_opmode_streaming)
		errorCode=vrep.simxSetJointTargetVelocity(clientID,rmh,-2, vrep.simx_opmode_streaming)
		errorCode,o_angles = vrep.simxGetObjectOrientation(clientID, oh, -1, vrep.simx_opmode_oneshot_wait)
		o_gamma = o_angles[2]
		delta = math.fabs(o_gamma - t_gamma)
		print 'o_gamma = ', o_gamma, ' t_gamma = ', t_gamma, ' delta = ', delta

#J arrete la liaison TCP/IP avec tous les serveurs
vrep.simxFinish(-1)

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print 'Connection au serveur V-REP'
    
else:
    print 'Echec de la connetion'
    sys.exit('Impossible de se connecter')


errorCode,targetHandler=vrep.simxGetObjectHandle(clientID,'Target',vrep.simx_opmode_oneshot_wait)
orCode,robotHandler=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)
#retrieve motor  handles
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
tourner(clientID, robotHandler, targetHandler, left_motor_handle, right_motor_handle);

t = time.time()

while (time.time()-t)<5:
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,2, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,2, vrep.simx_opmode_streaming)

# Arreter la simulation:
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
# Fermeture de la connection avec V-REP:
vrep.simxFinish(clientID)

