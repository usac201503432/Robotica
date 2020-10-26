import sim
import numpy as np
import time
import cv2


def connect(port):
	sim.simxFinish(-1)
	clientID=sim.simxStart('127.0.0.1' ,port,True,True,2000,5)
	if clientID ==0:print("conectado al puerto", port)
	else: print("no se pudo establecer conexion")
	return clientID


#Primer paso conectamos
clientID = connect(19999)
#retCode, resolution, image = sim.simxGetVisionSensorImage(clientID,sensorHandle,0,sim.simx_opmode_oneshot_wait)

#manejadores de la camara
_, camhandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
_, resolution, image = sim.simxGetVisionSensorImage(clientID,camhandle,0,sim.simx_opmode_streaming)
time.sleep(1)

_, resolution, image= sim.simxGetVisionSensorImage(clientID, camhandle, 0,sim.simx_opmode_buffer)
img = np.array(image,dtype = np.uint8)
img.resize([resolution[0],resolution[1],3])
img = np.rot90(img,2)
img = np.fliplr(img)
img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#color rojo
rojoBajo1 =np.array([0,100,20], np.uint8)
rojoAlto1 =np.array([8,255,255], np.uint8)
rojoBajo2 =np.array([175,100,20],np.uint8)
rojoAlto2 =np.array([179,255,255],np.uint8)
maskRojo1 = cv2.inRange(hsv, rojoBajo1, rojoAlto1)
maskRojo2 = cv2.inRange(hsv,rojoBajo2, rojoAlto2)
maskRed = cv2.add(maskRojo1,maskRojo2)

#color verde
verdeBajo = np.array([45,50,50])
verdeAlto = np.array([80,255,255])
maskGreen = cv2.inRange(hsv,verdeBajo,verdeAlto)

#color azul
azulBajo = np.array([100,65,75])
azulAlto = np.array([130,255,255])
maskBlue = cv2.inRange(hsv, azulBajo, azulAlto)

#buscando el centro de los objetos
M1 = cv2.moments(maskRed)
M2 = cv2.moments(maskGreen)
M3 = cv2.moments(maskBlue)
area1 = M1['m00']
area2 = M2['m00']
area3 = M3['m00']

if(area1 > 100):
	x = int(M1['m10']/M1['m00'])
	y = int(M1['m01']/M1['m00'])
	cv2.rectangle(img,(x,y),(x+2,y+2),(255,0,0),2)

if(area2 > 100):
	x = int(M2['m10']/M2['m00'])
	y = int(M2['m01']/M2['m00'])
	cv2.rectangle(img, (x,y), (x+2,y+2),(0,0,255),2)

if(area3 > 100):
	x = int(M3['m10']/M3['m00'])
	y = int(M3['m01']/M3['m00'])

#Se muestra la ventana con el objeto detectado
cv2.imshow('Imagen',img)
#cv2.imshow('Mascara Rojo',maskRed)
#cv2.imshow('Mascara Verde',maskGreen)
#cv2.imshow('Mascara Azul',maskBlue)
#presionar letra q para cerrar
#if cv2.waitKey(1) & 0xFF == ord('q'):
	#break

#Le doy nombres a los joint en coppelia para poder controlarlos
ret,cintura=sim.simxGetObjectHandle(clientID,'cintura',sim.simx_opmode_blocking)
ret,torso=sim.simxGetObjectHandle(clientID,'torso',sim.simx_opmode_blocking)
ret,hombro=sim.simxGetObjectHandle(clientID,'hombro',sim.simx_opmode_blocking)
ret,codo=sim.simxGetObjectHandle(clientID,'codo',sim.simx_opmode_blocking)
#ret,muneca=sim.simxGetObjectHandle(clientID,'muneca',sim.simx_opmode_blocking)



#obtenemos las coordenadas
while(1):
	if(area2>100):

		q1 = np.arctan((y)/(x)) -np.pi/2
		returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		h1 = np.sqrt(x**2+200**2)
		
		q2 = np.arccos((310**2+h1**2-453**2)/(2*453*310))
		returnCode = sim.simxSetJointTargetPosition(clientID, torso,q2,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q3 = np.arccos((453**2+310**2-h1**2)/(2*453*310))
		returnCode = sim.simxSetJointTargetPosition(clientID, hombro,q3,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q4 = -np.pi/6
		returnCode = sim.simxSetJointTargetPosition(clientID, codo,q4,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		time.sleep(1)
		#Coloca el cubo en su lugar
		q1 = np.pi/2
		returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)
	
		#Espera un momento y vuelve a la posicion inicial
		time.sleep(1)

		q1 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q2 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, torso,q2,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q3 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, hombro,q3,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q4 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, codo,q4,sim.simx_opmode_oneshot)
		time.sleep(0.4)
		
	#Estas lineas es para que la camara quede en el bucle infinito y vuelva a buscar mas cubos

	time.sleep(0.9)
	_, camhandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
	_, resolution, image = sim.simxGetVisionSensorImage(clientID,camhandle,0,sim.simx_opmode_streaming)
	time.sleep(1)

	_, resolution, image= sim.simxGetVisionSensorImage(clientID, camhandle, 0,sim.simx_opmode_buffer)
	img = np.array(image,dtype = np.uint8)
	img.resize([resolution[0],resolution[1],3])
	img = np.rot90(img,2)
	img = np.fliplr(img)
	img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	#color rojo
	rojoBajo1 =np.array([0,100,20], np.uint8)
	rojoAlto1 =np.array([8,255,255], np.uint8)
	rojoBajo2 =np.array([175,100,20],np.uint8)
	rojoAlto2 =np.array([179,255,255],np.uint8)
	maskRojo1 = cv2.inRange(hsv, rojoBajo1, rojoAlto1)
	maskRojo2 = cv2.inRange(hsv,rojoBajo2, rojoAlto2)
	maskRed = cv2.add(maskRojo1,maskRojo2)

	#color verde
	verdeBajo = np.array([45,50,50])
	verdeAlto = np.array([80,255,255])
	maskGreen = cv2.inRange(hsv,verdeBajo,verdeAlto)

	#color azul
	azulBajo = np.array([100,65,75])
	azulAlto = np.array([130,255,255])
	maskBlue = cv2.inRange(hsv, azulBajo, azulAlto)

	#buscando el centro de los objetos
	M1 = cv2.moments(maskRed)
	M2 = cv2.moments(maskGreen)
	M3 = cv2.moments(maskBlue)
	area1 = M1['m00']
	area2 = M2['m00']
	area3 = M3['m00']

	if(area1 > 100):
		x = int(M1['m10']/M1['m00'])
		y = int(M1['m01']/M1['m00'])
		cv2.rectangle(img,(x,y),(x+2,y+2),(255,0,0),2)

	if(area2 > 100):
		x = int(M2['m10']/M2['m00'])
		y = int(M2['m01']/M2['m00'])
		cv2.rectangle(img, (x,y), (x+2,y+2),(0,0,255),2)

	if(area3 > 100):
		x = int(M3['m10']/M3['m00'])
		y = int(M3['m01']/M3['m00'])
		
	if(area3>100):

		q1 = np.arctan(y/x) +np.pi/3
		returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		h1 = np.sqrt(x**2+200**2)

		q2 = np.arccos((310**2+h1**2-453**2)/(2*453*310))
		returnCode = sim.simxSetJointTargetPosition(clientID, torso,q2,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q3 = np.arccos((453**2+310**2-h1**2)/(2*453*310))
		returnCode = sim.simxSetJointTargetPosition(clientID, hombro,q3,sim.simx_opmode_oneshot)
		time.sleep(0.4)
		
		q4 = -np.pi/6
		returnCode = sim.simxSetJointTargetPosition(clientID, codo,q4,sim.simx_opmode_oneshot)
		time.sleep(0.4)
		
		#Tiempo de espera antes de que se vuelva  a mover a la posicion inicial

		time.sleep(1)

		q1 = -np.pi/2
		returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)
		time.sleep(1)

		#Vuelve a la posicion inicial
		q1 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, cintura,q1,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q2 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, torso,q2,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q3 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, hombro,q3,sim.simx_opmode_oneshot)
		time.sleep(0.4)

		q4 = 0
		returnCode = sim.simxSetJointTargetPosition(clientID, codo,q4,sim.simx_opmode_oneshot)
		time.sleep(0.4)
	
		
		
		