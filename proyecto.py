import sim
import numpy as np
from numpy import pi
import math
import sympy as sp
import time
import cv2
import matplotlib.pyplot as plt

def connect(port):
# Establece la conexión a VREP
# port debe coincidir con el puerto de conexión en VREP
# retorna el número de cliente o -1 si no puede establecer conexión
    sim.simxFinish(-1) 
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: 
        print("Conexion exitosa al puerto", port)
    else: 
        print("Error en la conexion")
    return clientID

def MoverBrazoaInicial(angulos,joints):
    theta = angulos
    joint = joints
    retCode = sim.simxSetJointTargetPosition(clientID, joint[1], theta[1]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[0], theta[0]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[3], theta[3]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[2], theta[2]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[4], 0*3.1415/180, sim.simx_opmode_oneshot)

def MoverBrazoaObjetivo(angulos,joints):
    theta = angulos
    joint = joints
    retCode = sim.simxSetJointTargetPosition(clientID, joint[0], theta[0]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[2], theta[2]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[3], theta[3]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[1], theta[1]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(4)#0,1,3,2

def MoverBrazoaContenedor(angulos,joints):
    theta = angulos
    joint = joints
    retCode = sim.simxSetJointTargetPosition(clientID, joint[1], theta[1]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(4)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[4], 180*3.1415/180, sim.simx_opmode_oneshot)
    #time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[0], theta[0]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(4)#
    retCode = sim.simxSetJointTargetPosition(clientID, joint[2], theta[2]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(2)
    retCode = sim.simxSetJointTargetPosition(clientID, joint[3], theta[3]*3.1415/180, sim.simx_opmode_oneshot)
    time.sleep(3)#0,1,3,2

def setEffector(val):

    returnCode=sim.simxSetFloatSignal(clientID,'RG2_open',val,sim.simx_opmode_oneshot)
    # acciona el efector final
    # val es Int con valor 0 para posición original, > 0 para abrir la garra, < 0 para cerrar la garra
    return returnCode

def getHandlers():
    #Handles de los cubos
    retCode,cubo=sim.simxGetObjectHandle(clientID,'Cuboid',sim.simx_opmode_blocking)
    retCode,cubo0=sim.simxGetObjectHandle(clientID,'Cuboid0',sim.simx_opmode_blocking)
    retCode,cubo1=sim.simxGetObjectHandle(clientID,'Cuboid1',sim.simx_opmode_blocking)
    #Handles de las esferas
    retCode,esfera=sim.simxGetObjectHandle(clientID,'Sphere',sim.simx_opmode_blocking)
    retCode,esfera0=sim.simxGetObjectHandle(clientID,'Sphere0',sim.simx_opmode_blocking)
    retCode,esfera1=sim.simxGetObjectHandle(clientID,'Sphere1',sim.simx_opmode_blocking)
    #Handles de los triangulos

    #Handles de los joints
    retCode,joint0=sim.simxGetObjectHandle(clientID,'Revolute_joint0',sim.simx_opmode_blocking)
    retCode,joint1=sim.simxGetObjectHandle(clientID,'Revolute_joint1',sim.simx_opmode_blocking)
    retCode,joint2=sim.simxGetObjectHandle(clientID,'Revolute_joint2',sim.simx_opmode_blocking)
    retCode,joint3=sim.simxGetObjectHandle(clientID,'Revolute_joint3',sim.simx_opmode_blocking)
    retCode,joint4=sim.simxGetObjectHandle(clientID,'Revolute_joint4',sim.simx_opmode_blocking)
    #Handle de los sensores
    retCode,sensorHandle=sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking)
    retCode,sensorHandle1=sim.simxGetObjectHandle(clientID,'Vision_sensor0',sim.simx_opmode_blocking)
    retCode,sensor=sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking)
    cubos = [cubo,cubo0,cubo1]
    esferas = [esfera,esfera0,esfera1]
    #sensor_vision = [sensorHandle,sensorHandle1]
    #sensor_proximidad = sensor
    joint = [joint0, joint1, joint2, joint3,joint4]
    return joint, cubos, esferas,sensorHandle,sensorHandle1,sensor

def MetodoG(pos):
    xc = pos[0]
    yc = pos[1]
    zc = pos[2]

    L = 0.2508
    M = 0.1244
    N = 0.3815

    R = np.sqrt((xc**2) + (yc**2))
    s = R - N
    #mayor a 0.5 y menor a 0.75
    z = zc - 0.1649
    Q = np.sqrt((s**2) + (z**2))

    f = math.atan2(z, s)
    g = math.acos(((L**2) + (Q**2) - (M**2))/(2*L*Q))

    a = np.rad2deg(f + g)
    b = np.rad2deg(math.acos(((M**2) + (L**2) - (Q**2))/(2*L*M)))
    c = 180 - (-b - a + 360)
    d = np.rad2deg(math.atan2(yc, xc))

    theta1 = d
    theta2 = a
    theta3 = 180 - b
    theta4 = c
    theta = [theta1, theta2, theta3, theta4]
    return theta

def get_contour_centers(contours: np.ndarray) -> np.ndarray:
    """
    Calculate the centers of the contours
    :param contours: Contours detected with find_contours
    :return: object centers as numpy array
    """

    if len(contours) == 0:
        return np.array([])

    #((x, y), radius) = cv2.minEnclosingCircle(c)
    centers = np.zeros((len(contours), 2), dtype=np.int16)
    for i, c in enumerate(contours):
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        centers[i] = center
    return centers 

def obtener_forma(image,resolution,rango):
    forma = ''
    #returnCode,resolution,image=sim.simxGetVisionSensorImage(clientID,sensorHandle1,0,sim.simx_opmode_oneshot_wait)

    imagen = np.array(image,dtype=np.uint8)
    imagen.resize([resolution[1],resolution[0],3])
    img_BGR = cv2.cvtColor(imagen,cv2.COLOR_RGB2BGR)

    img_GRAY = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2GRAY)

    #plt.hist(img_GRAY.ravel(),256,[0,256])
    #plt.show()

    ret,threshold = cv2.threshold(img_GRAY,rango,255,cv2.THRESH_BINARY)
    #plt.imshow(threshold)
    #plt.show()

    # using a findContours() function
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    longitud = len(contours)
    if longitud == 0:
        forma = 'no hay forma'
    else:
        contorno = contours[0]
        approx = cv2.approxPolyDP(contorno, 0.01 * cv2.arcLength(contorno, True), True)
        cv2.drawContours(imagen, [contorno], 0, (0, 0, 255), 5)

        M = cv2.moments(contorno)
        if M['m00'] != 0.0:
            x = int(M['m10']/M['m00'])
            y = int(M['m01']/M['m00'])

        if len(approx) == 3:
                forma = 'triangulo'
                cv2.putText(imagen, 'Triangulo', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        elif len(approx) == 4:
                forma = 'cuadrado'
                cv2.putText(imagen, 'Quadrilateral', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        else:
                forma = 'circulo'
                cv2.putText(imagen, 'circulo', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        #cv2.imshow('shapes', imagen)
    
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
    return forma

def obtener_coordenadas(rango):
    returnCode,resolution,image=sim.simxGetVisionSensorImage(clientID,sensorHandle,0,sim.simx_opmode_oneshot_wait)
    img = np.array(image,dtype=np.uint8)
    img.resize([resolution[1],resolution[0],3])
    img_BGR = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    img_GRAY = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2GRAY)

    #plt.hist(img_GRAY.ravel(),256,[0,256])
    #plt.show()

    ret,thresh = cv2.threshold(img_GRAY,rango,255,cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    centers = get_contour_centers(contours)

    x = centers[1][0]
    y = centers[1][1]
    #plt.imshow(thresh)
    #plt.plot(x,y,'o')
    #plt.show()

    #xc = (x*2/1000)
    xc = 0.015
    yc = 0.5 + (y*2/1000)
    zc = 0.2275
    position = [xc,yc,zc]

    return position

def obtener_forma_color():

    color = ['rojo','azul','verde']
    color_forma = ""
    
    light_blue = np.array([110,50,50])
    dark_blue = np.array([130,255,255])

    light_red = np.array([0,50,50])
    dark_red = np.array([10,255,255])

    lower_green = np.array([36, 0, 0])
    upper_green = np.array([70, 255,255])

    rangos_color = [25,10,50] #rojo, azul, verde
    colores = [[light_red,dark_red],[light_blue,dark_blue],[lower_green,upper_green]]
    returnCode,resolution,image=sim.simxGetVisionSensorImage(clientID,sensorHandle1,0,sim.simx_opmode_oneshot_wait)

    img = np.array(image,dtype=np.uint8)
    img.resize([resolution[1],resolution[0],3])
    img_BGR = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

    hsv = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)

    for i in range(3):
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, colores[i][0], colores[i][1])

        # Bitwise-AND mask and original image
        output = cv2.bitwise_and(img,img, mask= mask)
        forma = obtener_forma(output,resolution,rangos_color[i])
        if forma == 'circulo':
            color_forma = color[i]
            break
        if forma == 'cuadrado':
            color_forma = color[i]
            break
        if forma == 'triangulo':
            color_forma = color[i]
            break
        
        
    #cv2.imshow("Color Detected", np.hstack((img,output)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    return forma,color_forma

clientID = connect(19997)
# Iniciamos la simulación
returnCode=sim.simxStartSimulation(clientID,sim.simx_opmode_blocking)

tiempo = 1
# Obtenemos lo Handlers de los joints y los objetos
joint, cubos, esferas, sensorHandle, sensorHandle1, sensor = getHandlers()

# definimos las coordenadas de destino en la posición de reposo del manipulador final
pos = [0.3299, 0, 0.3438]

# Colocamos el manipulador en su posición inicial
setEffector(0)
theta = MetodoG(pos)
MoverBrazoaInicial(theta,joint)
time.sleep(2)

while(1):
    returnCode,detected,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=sim.simxReadProximitySensor(clientID,sensor,sim.simx_opmode_streaming)
    if (detected==1):
        print('objeto detectado')
        forma,color = obtener_forma_color()

        if forma == "cuadrado" and color == "verde":
            position = obtener_coordenadas(150)
            print(position)
            #calculamos los angulos para la posicion XYZ y lo movemos la pose deseada
            theta = MetodoG(position) 
            MoverBrazoaObjetivo(theta,joint)
            time.sleep(tiempo)
            setEffector(-0.02)
            time.sleep(tiempo)
            #calculamos los angulos y lo movemos al contenedor correspondiente
            position = [-0.5,0.0150,0.3220]
            theta = MetodoG(position)
            MoverBrazoaContenedor(theta,joint)
            time.sleep(tiempo)
            setEffector(0)
            returnCode=sim.simxSetObjectParent(clientID,cubos[0],-1,False,sim.simx_opmode_oneshot)
            time.sleep(0.2)
            returnCode=sim.simxSetObjectParent(clientID,cubos[0],-1,False,sim.simx_opmode_oneshot)
            #time.sleep(2)
            #calculamos los angulos y movemos el brazo a la posicio inicial
            theta = MetodoG(pos)
            MoverBrazoaInicial(theta,joint)
            time.sleep(2)
            #returnCode=sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)

        if forma == "cuadrado" and color == "rojo":
            position = obtener_coordenadas(140)
            print(position)
            #calculamos los angulos para la posicion XYZ y lo movemos la pose deseada
            theta = MetodoG(position) 
            MoverBrazoaObjetivo(theta,joint)
            time.sleep(tiempo)
            setEffector(-0.02)
            time.sleep(tiempo)
            #calculamos los angulos y lo movemos al contenedor correspondiente
            position = [-0.493,-0.1220,0.3220]
            theta = MetodoG(position)
            MoverBrazoaContenedor(theta,joint)
            time.sleep(tiempo)
            setEffector(0)
            returnCode=sim.simxSetObjectParent(clientID,cubos[1],-1,False,sim.simx_opmode_oneshot)
            time.sleep(0.2)
            returnCode=sim.simxSetObjectParent(clientID,cubos[1],-1,False,sim.simx_opmode_oneshot)
            #calculamos los angulos y movemos el brazo a la posicio inicial
            theta = MetodoG(pos)
            MoverBrazoaInicial(theta,joint)
            time.sleep(2)

        if forma == "cuadrado" and color == "azul":
            position = obtener_coordenadas(140)
            print(position)
            #calculamos los angulos para la posicion XYZ y lo movemos la pose deseada
            theta = MetodoG(position) 
            MoverBrazoaObjetivo(theta,joint)
            time.sleep(tiempo)
            setEffector(-0.02)
            time.sleep(tiempo)
            #calculamos los angulos y lo movemos al contenedor correspondiente
            position = [-0.493,0.13,0.3220]
            theta = MetodoG(position)
            MoverBrazoaContenedor(theta,joint)
            time.sleep(tiempo)
            setEffector(0)
            returnCode=sim.simxSetObjectParent(clientID,cubos[2],-1,False,sim.simx_opmode_oneshot)
            time.sleep(0.2)
            returnCode=sim.simxSetObjectParent(clientID,cubos[2],-1,False,sim.simx_opmode_oneshot)
            #calculamos los angulos y movemos el brazo a la posicio inicial
            theta = MetodoG(pos)
            MoverBrazoaInicial(theta,joint)
            time.sleep(2)

        if forma == "circulo" and color == "verde":
            position = obtener_coordenadas(140)
            print(position)
            #calculamos los angulos para la posicion XYZ y lo movemos la pose deseada
            theta = MetodoG(position) 
            MoverBrazoaObjetivo(theta,joint)
            time.sleep(tiempo)
            setEffector(-0.02)
            time.sleep(tiempo)
            #calculamos los angulos y lo movemos al contenedor correspondiente
            position = [0,-0.4380,0.3220]
            theta = MetodoG(position)
            MoverBrazoaContenedor(theta,joint)
            time.sleep(tiempo)
            setEffector(0)
            returnCode=sim.simxSetObjectParent(clientID,esferas[0],-1,False,sim.simx_opmode_oneshot)
            time.sleep(0.2)
            returnCode=sim.simxSetObjectParent(clientID,esferas[0],-1,False,sim.simx_opmode_oneshot)
            #calculamos los angulos y movemos el brazo a la posicio inicial
            theta = MetodoG(pos)
            MoverBrazoaInicial(theta,joint)
            time.sleep(2)
            #returnCode=sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot)

        if forma == "circulo" and color == "rojo":
            position = obtener_coordenadas(140)
            print(position)
            #calculamos los angulos para la posicion XYZ y lo movemos la pose deseada
            theta = MetodoG(position) 
            MoverBrazoaObjetivo(theta,joint)
            time.sleep(tiempo)
            setEffector(-0.02)
            time.sleep(tiempo)
            #calculamos los angulos y lo movemos al contenedor correspondiente
            position = [0.1270,-0.4380,0.3220]
            theta = MetodoG(position)
            MoverBrazoaContenedor(theta,joint)
            time.sleep(tiempo)
            setEffector(0)
            returnCode=sim.simxSetObjectParent(clientID,esferas[1],-1,False,sim.simx_opmode_oneshot)
            time.sleep(0.2)
            returnCode=sim.simxSetObjectParent(clientID,esferas[1],-1,False,sim.simx_opmode_oneshot)
            #calculamos los angulos y movemos el brazo a la posicio inicial
            theta = MetodoG(pos)
            MoverBrazoaInicial(theta,joint)
            time.sleep(2)

        if forma == "circulo" and color == "azul":
            position = obtener_coordenadas(140)
            print(position)
            #calculamos los angulos para la posicion XYZ y lo movemos la pose deseada
            theta = MetodoG(position) 
            MoverBrazoaObjetivo(theta,joint)
            time.sleep(tiempo)
            setEffector(-0.02)
            time.sleep(tiempo)
            #calculamos los angulos y lo movemos al contenedor correspondiente
            position = [-0.1310,-0.4380,0.3220]
            theta = MetodoG(position)
            MoverBrazoaContenedor(theta,joint)
            time.sleep(tiempo)
            setEffector(0)
            returnCode=sim.simxSetObjectParent(clientID,esferas[2],-1,False,sim.simx_opmode_oneshot)
            time.sleep(0.2)
            returnCode=sim.simxSetObjectParent(clientID,esferas[2],-1,False,sim.simx_opmode_oneshot)
            #calculamos los angulos y movemos el brazo a la posicio inicial
            theta = MetodoG(pos)
            MoverBrazoaInicial(theta,joint)
            time.sleep(2)


    else:
        print('objeto no detectado')
    time.sleep(3)

""" # Colocamos el manipulador en las coordenadas de la caja objetivo
position = [0.02,0.6370,0.3130]
theta = MetodoG(position)
MoverBrazoaObjetivo(theta,joint)

# Cerramos la garra
setEffector(-0.025)
time.sleep(2)
retCode = sim.simxSetJointTargetPosition(clientID, joint[0], 180*3.1415/180, sim.simx_opmode_oneshot)
time.sleep(5)

setEffector(0)
returnCode=sim.simxSetObjectParent(clientID,effector,-1,False,sim.simx_opmode_oneshot)
time.sleep(2)
returnCode=sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot) """
