import sim
import numpy as np
import time

def connect(port):
	sim.simxFinish(-1)
	clientID=sim.simxStart('127.0.0.1' ,port,True,True,2000,5)
	if clientID ==0:print("conectado al puerto", port)
	else: print("no se pudo establecer conexion")
	return clientID


#Primer paso conectamos
clientID = connect(19999)
#obtenemos los manejadores
returnCode,handle=sim.simxGetObjectHandle(clientID,'Dummy',sim.simx_opmode_blocking)
brazo = handle

#Le doy nombres a los joint en coppelia para poder controlarlos
ret,cintura=sim.simxGetObjectHandle(clientID,'cintura',sim.simx_opmode_blocking)
ret,torso=sim.simxGetObjectHandle(clientID,'torso',sim.simx_opmode_blocking)
ret,hombro=sim.simxGetObjectHandle(clientID,'hombro',sim.simx_opmode_blocking)
ret,codo=sim.simxGetObjectHandle(clientID,'codo',sim.simx_opmode_blocking)
ret,muneca=sim.simxGetObjectHandle(clientID,'muneca',sim.simx_opmode_blocking)


#codigo para mover por grados cada parte del brazo


q1 = 0*np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)

q2 = 15*np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, torso,q2,sim.simx_opmode_oneshot)

q3 = 45*np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, hombro,q3,sim.simx_opmode_oneshot)

q4 = 45*np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, codo,q4,sim.simx_opmode_oneshot)

q5 = 0*np.pi/180
returnCode = sim.simxSetJointTargetPosition(clientID, muneca,q5,sim.simx_opmode_oneshot)


