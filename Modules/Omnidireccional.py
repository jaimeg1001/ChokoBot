import numpy as np
from time import sleep
import sys
import os
from gpiozero import Servo
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from Motor import Motor

class Omnidireccional:
    def __init__(self,posInX:float,velInX:float,posInY:float,velInY:float,orIn:float,dorIn:float,radius:float,distH:float,distW:float):
        self.location = np.array([posInX,posInY,orIn])
        self.vel = np.array([velInX,velInY,dorIn])
        self.R = radius
        self.H = distH
        self.W = distW
        self.servos = [Servo(12),Servo(5),Servo(4)]
        self.openGripper()
        self.wheelFR = Motor(PWM=6,dir2=19,dir1=26,setpoint=0.0,posIn=0.0,velIn=0.0)
        self.wheelFL = Motor(PWM=17,dir2=27,dir1=22,setpoint=0.0,posIn=0.0,velIn=0.0)
        self.wheelBR = Motor(PWM=23,dir1=24,dir2=25,setpoint=0.0,posIn=0.0,velIn=0.0)
        self.wheelBL = Motor(PWM=16,dir1=20,dir2=21,setpoint=0.0,posIn=0.0,velIn=0.0)
        self.wheels = [self.wheelFR,self.wheelFL,self.wheelBR,self.wheelBL] 

    def closeGripper(self): #-unidades acopladas
        for servo in self.servos:
            servo.value = -1
        sleep(0.5)
        self.stopGripper()

    def openGripper(self):#Liberar acoplamiento
        for servo in self.servos:
            servo.value = 0.5
        sleep(0.5)
        self.stopGripper()

    def stopGripper(self):#Liberar acoplamiento
        for servo in self.servos:
            servo.detach()

    def getLocation(self):
        return self.location

    def getOrientation(self):
        return self.location[2]

    def getRotM(self,rot:bool):
        aux = np.array([[1,0,0],[0,1,0],[0,0,1]])
        if rot:
            aux = np.array([[1,0,0],[0,np.cos(np.pi), np.sin(np.pi)], [0,-1*np.sin(np.pi), np.cos(np.pi)]])
        return np.matmul(np.array([[np.cos(self.location[2]), np.sin(self.location[2]), 0], [-1*np.sin(self.location[2]), np.cos(self.location[2]), 0], [0, 0, 1]]),aux)

    def getkinInv(self):
        return (1/self.R) * np.array([[1, -1, -(self.W+self.H)],[1, 1, (self.W+self.H)],[1, 1, -(self.W+self.H)],[1, -1, (self.W+self.H)]])

    def updateWheelSpeed(self,wheelSpeedFR:float,wheelSpeedFL:float,wheelSpeedBR:float,wheelSpeedBL:float):
        self.wheelFR.speed = wheelSpeedFR
        self.wheelFL.speed = wheelSpeedFL
        self.wheelBR.speed = wheelSpeedBR
        self.wheelBL.speed = wheelSpeedBL
    
    def updateWheelSetPoint(self,wheelSetPFR:float,wheelSetPFL:float,wheelSetPBR:float,wheelSetPBL:float):
        self.wheelFR.setSetpoint(wheelSetPFR) 
        self.wheelFL.setSetpoint(wheelSetPFL)
        self.wheelBR.setSetpoint(wheelSetPBR)
        self.wheelBL.setSetpoint(wheelSetPBL)

    def updateLocation(self,posVector):
        self.location = posVector
    
    def controlWheelSpeed(self):
        for wheel in self.wheels:
            wheel.controlVel()

    def getWheelSpeed(self,rot:bool):
        aux1 = np.matmul(self.getRotM(rot),self.vel)
        return np.matmul(self.getkinInv(),aux1) * (30 / np.pi)
    
    def limitWheelSpeed(self,wheelsSpeed):
        for i in range(len(wheelsSpeed)):
            if wheelsSpeed[i] > 200:
                wheelsSpeed[i] = 200
            elif wheelsSpeed[i] < -200:
                wheelsSpeed[i] = -200
        return wheelsSpeed

    def updateRobotSpeed(self,velX:float,velY:float,velO:float):
        self.vel = np.array([velX,0.0,0.0])
        #print('Velocidades XYZ robot: ',self.vel)
        
        wheelsSpeedX = self.getWheelSpeed(rot=False)
        wheelsSpeedX = self.limitWheelSpeed(wheelsSpeedX)
        self.vel = np.array([0.0,velY,0.0])
        wheelsSpeedY = self.getWheelSpeed(rot=False)
        wheelsSpeedY = self.limitWheelSpeed(wheelsSpeedY)
        self.vel = np.array([0.0,0.0,velO])
        wheelsSpeedO = self.getWheelSpeed(rot=False)
        wheelsSpeedO = self.limitWheelSpeed(wheelsSpeedO)
        wheelsSpeed = [wheelsSpeedX[0]+wheelsSpeedY[0]+wheelsSpeedO[0],wheelsSpeedX[1]+wheelsSpeedY[1]+wheelsSpeedO[1],wheelsSpeedX[2]+wheelsSpeedY[2]+wheelsSpeedO[2],wheelsSpeedX[3]+wheelsSpeedY[3]+wheelsSpeedO[3]]
        wheelsSpeed = self.limitWheelSpeed(wheelsSpeed)
        """
        self.vel = np.array([velX,velY,velO])
        wheelsSpeed = self.getWheelSpeed(rot=True)
        print('Velocidades ruedas: ',wheelsSpeed)
        #print(wheelsSpeed)
        """
        print(wheelsSpeed)
        self.updateWheelSetPoint(wheelSetPFR=wheelsSpeed[1],wheelSetPFL=wheelsSpeed[0],wheelSetPBR=wheelsSpeed[3],wheelSetPBL=wheelsSpeed[2])
    
     
    
if __name__ == '__main__':
    from time import sleep
    omni1 = Omnidireccional(posInX=0.0,velInX=0.0,posInY=0.0,velInY=-10.0,orIn=0.0,dorIn=0.0,radius=0.03*1000,distW=0.118101*1000,distH=0.125*1000)
    print(omni1.getkinInv())
    print(omni1.getRotM(rot=False))
    print(omni1.getWheelSpeed(rot=False))
    sleep(5)
    print('Inicio')
    while True:
        omni1.closeGripper()
        sleep(5)
        omni1.openGripper()
        sleep(5)
