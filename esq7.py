#! /usr/bin/env python

from cmath import rect
from glob import glob
from operator import xor
import re
from tempfile import tempdir
from turtle import st
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty
import numpy as np

class turtleWatcher:
    def __init__(self,turtle_name):
        position_topic = "/" + turtle_name + "/pose"
        self.pose_subscriber = rospy.Subscriber(position_topic, Pose, self.poseCallback)
        self.x = 0
        self.y = 0
        self.theta = 0

        self.xAnt = 0
        self.yAnt = 0

        self.vel = 0
        self.tAnt = time.time()
    def poseCallback(self, msg):
        self.xAnt = self.x
        self.yAnt = self.y
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        t = time.time()
        dt = t - self.tAnt
        self.vel = np.sqrt((self.x-self.xAnt)**2 + (self.y - self.yAnt)**2) / 0.016
        self.tAnt = t
        

    def calcRecta(self):
        vert = False
        try:
            m = (self.yAnt - self.y) / (self.xAnt - self.x)
            b = self.y - m * self.x
        except:
            #print("vertical")
            vert = True
            m = 0
            b = x
        return [m,b,vert]
    def calcDistancia(self,x2,y2):
        return np.sqrt((self.x-x2)**2 + (self.y - y2)**2)

x = 0
y = 0
z = 0
theta = 0

xAnt = 0
yAnt = 0

radioTortuga = 1
radioCerca = 2

seguida = False
circulo = False
cuadrado = False

tortugas = []

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    global xAnt
    global yAnt
    xAnt = x
    yAnt = y
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta
    

def calcRecta():
    vert = False
    try:
        m = (yAnt - y) / (xAnt - x)
        b = y - m * x
    except:
        m = 0
        b = x
        vert = True
    
    return [m,b,vert]
def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        
        if getSign(theta) != getSign(desired_angle_goal):
            if getSign(theta) < 0:
                theta = np.pi*2 + theta
            else:
                desired_angle_goal = np.pi*2 + desired_angle_goal
            
        dtheta = desired_angle_goal-theta
        angular_speed = ka * (dtheta)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
       ##print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
def inRange(x,x2,tolerance):
    return x2 - tolerance < x and x2 + tolerance > x   # cambie and por or

def oppositeAngle(angle):
    newAngle = angle + np.pi
    if(abs(newAngle) > np.pi):
        newAngle = -abs(newAngle)/newAngle * abs(abs(newAngle) - np.pi)

    return newAngle
def getSign(x):
    try:
        return abs(x) / x
    except:
        return 0

class recta:
    def __init__(self,x1,y1,x2,y2):
        self.vert = False
        self.inicio = [x1,y1]
        self.fin = [x2,y2]
        if x1 < x2:
            self.dir = 1
        elif x2 < x1:
            self.dir = -1
        elif y1 < y2:
            self.dir = 1
        else:
            self.dir = -1
        try:
            self.m = (y2 - y1) / (x2 - x1)
            self.b = y1 - self.m * x1
        except:
            self.m = 0
            self.b = x2
            self.vert = True
    def calcPoint(self,x1,y1,d):
        if not self.vert:
            newX = x1 + self.dir * (d * np.cos(np.arctan(self.m)))
            newY = self.m * newX + self.b
        else:
            newX = self.b
            newY = y1 + d
        return newX, newY
    def calcDist(self,x,y):
        if self.vert:
            return abs(self.m * y - x + self.b)/np.sqrt(self.m**2+(-1)**2)
        else:
            return abs(self.m * x - y + self.b)/np.sqrt(self.m**2+(-1)**2)

    def calcDistGoal(self,x,y):
        return np.sqrt((y - self.fin[1])**2 + (x - self.fin[0])**2)
      
        
def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    
    esquivando = False
    goalUpdate = False
    iter = 0
    pasoEsquivo = 0
    coordenadasPaso2 = [0,0]
    coordenadasPaso3 = [0,0]
    linear_speed = 0
    objTemp = []

    carriles = []
    carriles.append(recta(x,y,xgoal,ygoal))
    if carriles[0].vert:
        newRecta = [0,y,False]
        newRecta2 = [0,ygoal,False]
    elif carriles[0].m == 0:
        newRecta = [0,x,True]
        newRecta2 = [0,xgoal,True]
    else:
        newM = -1/carriles[0].m
        newRecta = [newM,y-newM*x,False]
        newRecta2 = [newM,ygoal-newM*xgoal,False]
    distanciaCarriles = radioTortuga * 2
    if not newRecta[2]:
        xT = x + np.cos(np.arctan(newRecta[0])) * distanciaCarriles
        yT = newRecta[0] * xT + newRecta[1]
        xT2 = xgoal + np.cos(np.arctan(newRecta2[0])) * distanciaCarriles
        yT2 = newRecta2[0] * xT2 + newRecta2[1]
    else:
        xT = x
        yT = y + distanciaCarriles
        xT2 = xgoal
        yT2 =  ygoal + distanciaCarriles
    carriles.append(recta(xT,yT,xT2,yT2))
    if not newRecta[2]:
        xT = x - np.cos(np.arctan(newRecta[0])) * distanciaCarriles
        yT = newRecta[0] * xT + newRecta[1]
        xT2 = xgoal - np.cos(np.arctan(newRecta2[0])) * distanciaCarriles
        yT2 = newRecta2[0] * xT2 + newRecta2[1]
    else:
        xT = x
        yT = y - distanciaCarriles
        xT2 = xgoal
        yT2 =  ygoal - distanciaCarriles
    carriles.append(recta(xT,yT,xT2,yT2))
    carrilAct = 0
    distGoalT = 0.5
    borderLimit = 0.5
    if xgoal < borderLimit:
        xgoal = borderLimit
    elif xgoal > 11-borderLimit:
        xgoal = 11-borderLimit
    if ygoal < borderLimit:
        ygoal = borderLimit
    elif ygoal > 11-borderLimit:
        ygoal = 11-borderLimit
    goal = [xgoal, ygoal]
    while(True):
        
        iter +=1
        dGoalFinal = abs(math.sqrt(((goal[0]-x)**2)+((goal[1]-y)**2)))
        if dGoalFinal < distGoalT:
            xgoal = goal[0]
            ygoal = goal[1]
        else:
            xgoal,ygoal = carriles[carrilAct].calcPoint(x,y,distGoalT)
        orientate(xgoal,ygoal)
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        
        #print("Data",[xgoal,ygoal],[x,y])
        #print("Original",theta,desired_angle_goal)
        if getSign(theta) != getSign(desired_angle_goal):
            if abs(abs(theta)-abs(desired_angle_goal)) > np.pi:
                if getSign(theta) < 0:
                    theta = np.pi*2 + theta
                else:
                    desired_angle_goal = np.pi*2 + desired_angle_goal
                #print("mod",theta,desired_angle_goal)
        dtheta = desired_angle_goal-theta
        
        #dtheta = desired_angle_goal-theta
        if abs(dtheta) > np.pi:
            if inRange(abs(dtheta),2 * np.pi,0.1):
                dtheta = (abs(dtheta) - 2 * np.pi)  
            else:
                dtheta = (abs(dtheta) - np.pi)  
                
            #print('dtheta new',dtheta)
        angular_speed = ka * (dtheta)
        if(abs(dtheta) > 0.5):
            kv = 1.5
        else:
            kv = 3
        if seguida:
            kv = kv * 2
        if esquivando:
            kv = kv * 7
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = kv * distance

        distancias = []
        indexCercanas = []
        cont = 0
        for turt in tortugas:
            d = turt.calcDistancia(x,y)
            if d < radioCerca:
                distancias.append(d)
                indexCercanas.append(cont)
            cont += 1
        miRecta = calcRecta()
        for i in range(len(distancias)):
            rectas = tortugas[indexCercanas[i]].calcRecta()
            #Checar si pendientes similares
            ##print("newTortuga")
            ##print("yo",[x,y,theta])
            ##print("ella",[tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y,tortugas[indexCercanas[i]].theta])
            ##print("mirecta",miRecta)
            ##print("surecta",rectas)
            ##print("mioposite",oppositeAngle(theta))
            anguloTemp = np.arctan2((tortugas[indexCercanas[i]].y - y),(tortugas[indexCercanas[i]].x - x))
            anguloTemp -=theta
            distObje = np.sqrt((tortugas[indexCercanas[i]].x - goal[0]) ** 2 + (tortugas[indexCercanas[i]].y - goal[1]) ** 2)
            if distObje > radioTortuga and goalUpdate:
                xgoal = goal[0]
                ygoal = goal[1]
                goalUpdate = False
            
            if tortugas[indexCercanas[i]].vel < 0.05 and iter > 1000 :
                ##print('estancada')
                
                
                #print('Estancada',distObje,distancias[i])
                if abs(distObje) <= radioTortuga:
                    
                    if not goalUpdate:
                        #print('Cambiar goal')
                        #Obtener el punto en mi recta que este a dos radio tortuga de mi objetivo
                        if miRecta[2]:
                            newYGoal = ygoal - radioTortuga * np.sin(theta) * 3/2
                            newXGoal = miRecta[0] * newYGoal + miRecta[1]
                        else:
                            newXGoal = xgoal - radioTortuga * np.cos(theta) * 3/2
                            newYGoal = miRecta[0] * newXGoal + miRecta[1]
                        
                        ignorarCaso = True
                        #print("yo",[x,y,theta])
                        #print('newGoal\n\n\n\n\n\n\n\n\n\n\n\n\n\n',[newXGoal,newYGoal])
                        xgoal = newXGoal
                        ygoal = newYGoal
                        otraEnPunto = True
                        iter = 0
                        goalUpdate = True
                
                else:
                    distanciaMiCarril = carriles[carrilAct].calcDist(tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y)
                    distGoalYo = carriles[carrilAct].calcDistGoal(x,y)
                    distGoalOtra = carriles[carrilAct].calcDistGoal(tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y)
                    if distanciaMiCarril < radioTortuga and distGoalYo > distGoalOtra:
                        idxCercano = 0
                        menor = radioTortuga * 3
                        for carr in range(3):
                            if carr != carrilAct:
                                distanciaOtroCarril = carriles[carr].calcDist(tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y)
                                if distanciaOtroCarril > radioTortuga and distanciaOtroCarril < menor:
                                    idxCercano = carr
                                    menor = distanciaOtroCarril
                        print("Lane change:",distanciaMiCarril,"act",carrilAct,"new",idxCercano,"yo",[x,y,distGoalYo],"ella",[tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y,distGoalOtra])
                        carrilAct = idxCercano
                    elif carrilAct != 0:
                        distanciaCentral = carriles[0].calcDist(tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y)
                        distGoalYo = carriles[0].calcDistGoal(x,y)
                        distGoalOtra = carriles[0].calcDistGoal(tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y)
                        if distGoalOtra - distGoalYo > radioTortuga/2 and distanciaCentral < radioTortuga:
                            carrilAct = 0
                            print("Regreso",distGoalOtra,distGoalYo,distGoalOtra - distGoalYo)
                    print("lol")

                    
                    
                   
            elif not esquivando and linear_speed > 0.05:
                #Regresar a objetivo si ya la otra no esta
                tempAngleCheck = inRange(oppositeAngle(theta),tortugas[indexCercanas[i]].theta,0.2) or inRange(theta,tortugas[indexCercanas[i]].theta,0.2) or inRange(oppositeAngle(theta),oppositeAngle(tortugas[indexCercanas[i]].theta),0.2)
                if inRange(abs(miRecta[0]),abs(rectas[0]),0.2) and miRecta[2] == rectas[2] and inRange(anguloTemp,0,0.2): 
                    #print("lol",abs(abs(miRecta[1]) - abs(rectas[1])))
                    if abs(abs(miRecta[1]) - abs(rectas[1])) < radioTortuga * 2:
                        
                        #print("Paralelas")
                        ##print(getSign(miRecta[0]))
                        ##print(getSign(rectas[0]))
                        if getSign(miRecta[0]) == getSign(rectas[0]) and inRange(theta,tortugas[indexCercanas[i]].theta,0.2):
                            #print("Misma dir")
                            if abs(anguloTemp) < 0.01:
                                anguloTemp = 0
                            ##print(anguloTemp)
                            if anguloTemp + np.pi/2 > 0  and linear_speed > tortugas[indexCercanas[i]].vel:
                                #print('ve;',tortugas[indexCercanas[i]].vel)
                                linear_speed = tortugas[indexCercanas[i]].vel
                            elif anguloTemp + np.pi/2 < 0  and linear_speed < tortugas[indexCercanas[i]].vel:
                                linear_speed = tortugas[indexCercanas[i]].vel
                                #print("2")
                        else:
                            #print("Frente a frente")
                            '''''
                            if not esquivando:
                                esquivando = True
                                if abs(anguloTemp + np.pi/2) <= np.pi/2:
                                    #Derecha
                                    xgoal = x + np.cos(theta) * radioCerca /2 + np.sin(theta) * radioTortuga * 2
                                    ygoal = y + np.sin(theta) * radioCerca /2 + np.cos(theta) * radioTortuga * 2
                                    
                                else:
                                    xgoal = x + np.cos(theta) * radioCerca /2 - np.sin(theta) * radioTortuga * 2
                                    ygoal = y + np.sin(theta) * radioCerca /2 - np.cos(theta) * radioTortuga * 2
                                if not miRecta[2]:
                                    newX = x + 3/2 * radioCerca * np.cos(theta) + radioTortuga * np.sin(theta)
                                    newY = miRecta[0] * newX + miRecta[1]
                                else:
                                    newY = x + 3/2 * radioCerca * np.cos(theta) + radioTortuga * np.sin(theta)
                                    newX = miRecta[0] * newY + miRecta[1]
                                coordenadasPaso2 = [xgoal + 1 * radioCerca /4 * np.cos(theta), ygoal + 1 * radioCerca /4 * np.sin(theta)]
                                coordenadasPaso3 = [newX,newY]
                                #print("coor2",coordenadasPaso2)
                                #print("coor3",coordenadasPaso3)
                            '''
                            if not esquivando:
                                esquivando = True
                                if miRecta[2]:
                                    newRecta = [0,y,False]
                                elif miRecta[0] == 0:
                                    newRecta = [0,x,True]
                                else:
                                    newM = -1/miRecta[0]
                                    newRecta = [newM,y-newM*x,False]
                                if not newRecta[2]:
                                    xgoal = radioTortuga * 4/2 * np.cos(np.arctan(newRecta[0])) + x
                                    ygoal = newRecta[0] * xgoal + newRecta[1]
                                
                                    if xgoal > 10.5 or ygoal > 10.5 or ygoal < 0.5:
                                        #time.sleep(1.0)
                                        #print(newRecta)
                                        xgoal = x - radioTortuga * 4/2 * np.cos(np.arctan(newRecta[0]))
                                        ygoal = newRecta[0] * xgoal + newRecta[1]
                                    if xgoal > 10.5 or xgoal < 0.5 or ygoal > 10.5 or ygoal < 0.5:
                                        xgoal = abs(xgoal)
                                        ygoal = abs(ygoal)
                                    #print('actual',x,y,'new',xgoal,ygoal)
                                    pasoEsquivo = 3
                                else:
                                    #print('lol?')
                                    xgoal = radioTortuga * 4/2 * np.cos(np.arctan(newRecta[0])) + x
                                    ygoal = y

                                    if xgoal > 10.5 or xgoal < 0.5 or ygoal > 10.5 or ygoal < 0.5:
                                        xgoal = x - radioTortuga * 4/2 * np.cos(np.arctan(newRecta[0]))
                                        ygoal = y
                else:
                    #y = m1 x + b1
                    #y = m2 x + b2
                    #m1 x + b1 = m2 x + b2
                    #(b2 - b1) / (m1 - m2) = x
                    #y = m1 x + b1
                    try:
                        if miRecta[2]:
                            xChoque = miRecta[1]
                            yChoque = rectas[0] * xChoque + rectas[1]
                        elif rectas[2]:
                            xChoque = rectas[1]
                            yChoque = miRecta[0] * xChoque + miRecta[1]
                        else:
                            xChoque = (miRecta[1]-rectas[1])/(rectas[0] - miRecta[0])
                            yChoque = miRecta[0] * xChoque + miRecta[1]
                    except:
                        if inRange(abs(theta),np.pi/2,0.1): #Estoy vertical
                            xChoque = miRecta[1]
                            yChoque = rectas[0] * xChoque + rectas[1]
                        elif  inRange(abs(tortugas[indexCercanas[i]].theta),np.pi/2,0.1): #Esta en vertical
                            xChoque = rectas[1]
                            yChoque = miRecta[0] * xChoque + miRecta[1]
                    #print("No paralelas")
                    
                    #print("newTortuga",i)
                    #print("yo",[x,y,theta])
                    #print("ella",[tortugas[indexCercanas[i]].x,tortugas[indexCercanas[i]].y,tortugas[indexCercanas[i]].theta])
                    #print("mirecta",miRecta)
                    #print("surecta",rectas)
                    #print("Choque",xChoque,yChoque)
                    #print("mioposite",oppositeAngle(theta))
        
                    if np.arctan2(yChoque - y,xChoque - x) - theta + np.pi < 0:
                        #print("Atras de mi")
                        linear_speed = linear_speed * 3 /2
                    elif np.arctan2(yChoque - tortugas[indexCercanas[i]].y,xChoque - tortugas[indexCercanas[i]].x) - tortugas[indexCercanas[i]].theta + np.pi < 0:
                        #print("Atras de ella")
                        linear_speed = linear_speed / 2
                    else:
                        
                        distPuntoOtra = np.sqrt((xChoque - tortugas[indexCercanas[i]].x)**2 +(yChoque - tortugas[indexCercanas[i]].y)**2)
                        distPuntoMio = np.sqrt((xChoque - x)**2 +(yChoque - y)**2)
                        if distPuntoMio < radioCerca:
                            #print("LLego antes")
                            if tortugas[indexCercanas[i]].vel != 0:
                                tOtra = distPuntoOtra / tortugas[indexCercanas[i]].vel
                            else:
                                tOtra = 100000
                            tMia = distPuntoMio / linear_speed
                            '''
                            if tOtra <= tMia:
                                tChoque = tOtra
                                otraPos = 
                            else:
                                tChoque = tMia
                            '''
                            if tOtra <= tMia:
                                #print("La otra llega antes",distancias[i])
                                if distancias[i] < radioTortuga*1.5:
                                    linear_speed = -linear_speed 
                                else:
                                    linear_speed = 0
                            elif tMia < tOtra:
                                
                                
                                if distancias[i] < radioTortuga*1.5:
                                    otraAtras = False
                                    for p in range(len(distancias)):
                                        if p != i:
                                            angleOther = np.arctan2((tortugas[indexCercanas[i]].y - y),(tortugas[indexCercanas[i]].x - x)) -theta + np.pi/2
                                            otraAtras = otraAtras or angleOther < 0
                                    if otraAtras:
                                        linear_speed = 0
                                    else:
                                        linear_speed = -linear_speed /2
                                else:
                                    linear_speed = 0
                                #linear_speed =+ radioCerca / distancias[indexCercanas[i]] * 10
                

            '''
            
            
            contansteente calcular vel todas tortugas ok
            calcular quienes estan en mi radio ok
            calcular su trayectoria ok
            calcular mi trayectoria ok
            if las pendientes muy similar y la dist entre pendientes es menor a radio tortuga ok
                if vamos en misma dir ok
                    if la otra tortuga esta frente y nuestra vel es menor o = ok
                        nada ok
                    else if esta frente y mi vel es mayor  ok
                        igualar su vel ok
                    else if esta atras y su vel es mayor ok
                        igualar su vel ok
                else
                    ver si esta a nuestra izq o der cos
                    if der 
                        agregar vel ang der ok
                    if izq
                        agregar vel ang izq ok

            calcular punto donde se cruzan trayec ok
            if punto choque atras de mi acelero poco ok
            esle punto chque atras de ellos desacelero poco ok
            else 
                usanod vel calcular el tiempo de la otra tortuga al punto ok
                calcular cuanto tardo yo en llegar a ese pnto ok
                calcular mi pos en ese momento
                if su tiempo menor al mio ok
                    paro ok
                if mi timpo menor al suyo ok
                    acelero inv proporcional a la dist entre tortugas ok
                
             

            punto delante de mi reviso 
            0 -pi/2  calcular su vel
            '''

        if xgoal < borderLimit:
            xgoal = borderLimit
        elif xgoal > 11-borderLimit:
            xgoal = 11-borderLimit
        if xgoal < borderLimit:
            ygoal = borderLimit
        elif xgoal > 11-borderLimit:
            ygoal = 11-borderLimit
        if x < 0.25:
            if abs(theta) > np.pi/2:
                linear_speed = 0
        elif x > 10.75:
            if abs(theta) < np.pi/2:
                linear_speed = 0
        if y < 0.25:
            if theta < 0:
                linear_speed = 0
        elif y > 10.75:
            if theta > 0:
                linear_speed = 0
        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        
        ##print ('x=', x, 'y=', y)
        pastTheta = desired_angle_goal
        if seguida:
            if (distance < 0.1):
                break
        else:
            if not esquivando:
                if (distance < 0.01):
                    break
            else:
                if (distance < 0.2):
                    #print("arrived")
                    
                    if pasoEsquivo == 0:
                        #moverse en y nada mas
                        xgoal = coordenadasPaso2[0]
                        ygoal = coordenadasPaso2[1]
                        pasoEsquivo += 1
                    elif pasoEsquivo == 1:
                        xgoal = coordenadasPaso3[0]
                        ygoal = coordenadasPaso3[1]
                        
                        pasoEsquivo += 1
                    else:
                        xgoal = goal[0]
                        ygoal = goal[1]
                        pasoEsquivo = 0  
                        esquivando = False
                    orientate(xgoal,ygoal)
                    '''
                    
                    '''
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
                    
def stop():
    print("Esquivator finito")

if __name__ == '__main__':
    try:
        start_time = time.time()
        rospy.init_node('turtlesim_motion_pose', anonymous = True)
        rospy.on_shutdown(stop)
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        tortugas.append(turtleWatcher('turtle2'))
        tortugas.append(turtleWatcher('turtle3'))
        
        time.sleep(2)     
        delayTime = 1.0
        initPos = [5.5,1.0]
        pos = [[10.0,3.0],[1.0,7.0],[9.0,10.0]]

        for i in range(len(pos)):
            #print(pos[i][0],"\t",pos[i][1])
            orientate(pos[i][0],pos[i][1])
            time.sleep(1.0)
            go_to_goal(pos[i][0],pos[i][1])
            time.sleep(1.0)	


    except rospy.ROSInterruptException:        
	    pass
