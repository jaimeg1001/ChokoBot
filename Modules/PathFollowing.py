import numpy as np
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
cwd = os.path.abspath(os.getcwd())

from Trajectory import Trajectoria
from GPIController import GPIController
from Omnidireccional import Omnidireccional

class Trajectory2D:
    def __init__(self,xPath:str,yPath:str,orPath:str,step:float,orIn=0.0) -> None:
        self.X = Trajectoria(csvPath=xPath,step=step)
        self.Y = Trajectoria(csvPath=yPath,step=step)
        self.O = Trajectoria(csvPath=orPath,step=step)
        self.O.q = self.O.q * orIn
    def invertTrajectory(self):
        self.X.invertTrajectory()
        self.Y.invertTrajectory()
        self.O.invertTrajectory()

class ControlOmnidireccional:
    def __init__(self,kx:float,ky:float,ko:float,trajectory:Trajectory2D,robot:Omnidireccional,f_stop:bool) -> None:
        self.gpiX = GPIController(k=kx)
        self.gpiY = GPIController(k=ky)
        self.gpiO = GPIController(k=ko)
        self.robot = robot
        self.UR=[]
        self.lider=[0,0,0]
        self.f_lider=False
        self.URpos=[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
        self.traj = trajectory
        self.f_stop=False
        self.f_acoplamiento=False
        #self.f_servo=False

    def alto(self):
        self.robot.updateRobotSpeed(0.0,0.0,0.0)
        #if self.f_servo:
            #self.robot.closeGripper()
        #else:
            #self.robot.openGripper()
        time.sleep(0.01)
    
    def startController(self,trajectory):
        self.traj=trajectory
        i = 0
        tAnt = time.time_ns() / 1_000_000
        while True:
            tAct = time.time_ns() / 1_000_000
            tSample = tAct - tAnt
            if tSample > 100:
                #print(tSample/1000)
                tAnt = tAct
                #if self.f_servo:
                    #self.robot.closeGripper()
                #else:
                  #  self.robot.openGripper()
                    
                if self.f_stop:
                    self.robot.updateRobotSpeed(0.0,0.0,0.0)
                else: 
                    self.gpiX.gpiControl(qd=self.traj.X.q[i],dqd=self.traj.X.dq[i],q=1*self.robot.location[0],sampleTime=tSample/1000)
                    self.gpiY.gpiControl(qd=1*self.traj.Y.q[i],dqd=1*self.traj.Y.dq[i],q=1*self.robot.location[1],sampleTime=tSample/1000)
                    self.gpiO.gpiControl(qd=self.traj.O.q[i],dqd=1*self.traj.O.dq[i],q=1*self.robot.location[2],sampleTime=tSample/1000)
                    print('Xd: ',self.traj.X.q[i],', Xactual: ',self.robot.location[0],'Yd: ',self.traj.Y.q[i],', Yactual: ',self.robot.location[1],'Od: ',self.traj.O.q[i],', Oactual: ',self.robot.location[2])
                    #print('dXd: ',self.traj.X.dq[i],', dXControl: ',self.gpiX.u,'dYd: ',self.traj.Y.dq[i],', dYControl: ',self.gpiY.u,'dOd: ',self.traj.O.dq[i],', dOControl: ',self.gpiO.u)
                    #print(self.gpiX.intU,self.gpiY.intU,self.gpiO.intU)
                    self.robot.updateRobotSpeed(self.gpiX.u,self.gpiY.u,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,0.0,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,self.gpiY.u,0.0)
                    #robot.updatePosition(controlSignal=controlador.u,sampleTime=tSample/1000)
                    i = i + 1
                    if i==50:
                        self.robot.updateRobotSpeed(0.0,0.0,0.0)
                    if i > (len(self.traj.X.q)-1):
                        self.robot.updateRobotSpeed(0.0,0.0,0.0)
                        return True
                    
    def startCouplingLider(self,trajectory):
        self.traj=trajectory
        i = 0
        tAnt = time.time_ns() / 1_000_000
        while True:
            tAct = time.time_ns() / 1_000_000
            tSample = tAct - tAnt
            if tSample > 100:
                #print(tSample/1000)
                tAnt = tAct
                
                #self.robot.openGripper()
                    
                if self.f_stop:
                    self.robot.updateRobotSpeed(0.0,0.0,0.0)
                else: 
                    self.gpiX.gpiControl(qd=self.traj.X.q[i],dqd=self.traj.X.dq[i],q=1*self.robot.location[0],sampleTime=tSample/1000)
                    self.gpiY.gpiControl(qd=1*self.traj.Y.q[i],dqd=1*self.traj.Y.dq[i],q=1*self.robot.location[1],sampleTime=tSample/1000)
                    self.gpiO.gpiControl(qd=self.traj.O.q[i],dqd=1*self.traj.O.dq[i],q=1*self.robot.location[2],sampleTime=tSample/1000)
                    print('Xd: ',self.traj.X.q[i],', Xactual: ',self.robot.location[0],'Yd: ',self.traj.Y.q[i],', Yactual: ',self.robot.location[1],'Od: ',self.traj.O.q[i],', Oactual: ',self.robot.location[2])
                    #print('dXd: ',self.traj.X.dq[i],', dXControl: ',self.gpiX.u,'dYd: ',self.traj.Y.dq[i],', dYControl: ',self.gpiY.u,'dOd: ',self.traj.O.dq[i],', dOControl: ',self.gpiO.u)
                    #print(self.gpiX.intU,self.gpiY.intU,self.gpiO.intU)
                    self.robot.updateRobotSpeed(self.gpiX.u,self.gpiY.u,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,0.0,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,self.gpiY.u,0.0)
                    #robot.updatePosition(controlSignal=controlador.u,sampleTime=tSample/1000)
                    if self.f_acoplamiento:
                        #self.f_servo = True
                        #self.robot.closeGripper()
                        return True
                    
    def startCouplingFollower(self,lider):
        self.lider=self.UR[lider-1]
        tAnt = time.time_ns() / 1_000_000
        tin = tAnt
        while True:
            tAct = time.time_ns() / 1_000_000
            tSample = tAct - tAnt
            if tSample > 100:
                #print(tSample/1000)
                tAnt = tAct
                
                #if self.f_servo:
                    #self.robot.closeGripper()
                #else:
                  #  self.robot.openGripper()

                if self.f_stop:
                    self.robot.updateRobotSpeed(0.0,0.0,0.0)
                else: 
                    self.gpiX.gpiControl(qd=self.lider[0],dqd=0.0,q=1*self.robot.location[0],sampleTime=tSample/1000)
                    if (tAnt - tin)//1000 > 10: 
                        self.gpiY.gpiControl(qd=1*self.lider[1]-336,dqd=0.0,q=1*self.robot.location[1],sampleTime=tSample/1000)
                    else:
                        self.gpiY.gpiControl(qd=1*self.lider[1]-500,dqd=0.0,q=1*self.robot.location[1],sampleTime=tSample/1000)
                    self.gpiO.gpiControl(qd=0.0,dqd=0.0,q=1*self.robot.location[2],sampleTime=tSample/1000)
                    print('Xd: ',self.lider[0],', Xactual: ',self.robot.location[0],'Yd: ',self.lider[1],', Yactual: ',self.robot.location[1],'Od: ',self.lider[2],', Oactual: ',self.robot.location[2])
                    self.robot.updateRobotSpeed(self.gpiX.u,self.gpiY.u,self.gpiO.u)
                    if self.f_acoplamiento:
                        return True

    def startControllerFollower(self,lider):
        tAnt = time.time_ns() / 1_000_000
        while True:
            self.lider=self.UR[lider-1]
            tAct = time.time_ns() / 1_000_000
            tSample = tAct - tAnt
            if tSample > 100:
                #print(tSample/1000)
                tAnt = tAct

                #if self.f_servo:
                    #self.robot.closeGripper()
                #else:
                  #  self.robot.openGripper()

                if self.f_stop:
                    self.robot.updateRobotSpeed(0.0,0.0,0.0)
                else: 
                    self.gpiX.gpiControl(qd=self.lider[0],dqd=0.0,q=1*self.robot.location[0],sampleTime=tSample/1000)
                    self.gpiY.gpiControl(qd=1*self.lider[1]-500,dqd=0.0,q=1*self.robot.location[1],sampleTime=tSample/1000)
                    self.gpiO.gpiControl(qd=0.0,dqd=0.0,q=1*self.robot.location[2],sampleTime=tSample/1000)
                    print('Xd: ',self.lider[0],', Xactual: ',self.robot.location[0],'Yd: ',self.lider[1],', Yactual: ',self.robot.location[1],'Od: ',self.lider[2],', Oactual: ',self.robot.location[2])
                    self.robot.updateRobotSpeed(self.gpiX.u,self.gpiY.u,self.gpiO.u)
                    if self.f_lider:
                        return True

    def startDecouplingLider(self):
        xi=1*self.robot.location[0]
        yi=1*self.robot.location[0]
        tAnt = time.time_ns() / 1_000_000
        tin = tAnt
        while True:
            tAct = time.time_ns() / 1_000_000
            tSample = tAct - tAnt
            if tSample > 100:
                #print(tSample/1000)
                tAnt = tAct
                #self.robot.openGripper()
                if self.f_stop:
                    self.robot.updateRobotSpeed(0.0,0.0,0.0)
                else: 
                    self.gpiX.gpiControl(qd=xi,dqd=0.0,q=1*self.robot.location[0],sampleTime=tSample/1000)
                    self.gpiY.gpiControl(qd=yi+500,dqd=0.0,q=1*self.robot.location[1],sampleTime=tSample/1000)
                    self.gpiO.gpiControl(qd=0.0,dqd=0.0,q=1*self.robot.location[2],sampleTime=tSample/1000)
                    #print('Xd: ',self.traj.X.q[i],', Xactual: ',self.robot.location[0],'Yd: ',self.traj.Y.q[i],', Yactual: ',self.robot.location[1],'Od: ',self.traj.O.q[i],', Oactual: ',self.robot.location[2])
                    #print('dXd: ',self.traj.X.dq[i],', dXControl: ',self.gpiX.u,'dYd: ',self.traj.Y.dq[i],', dYControl: ',self.gpiY.u,'dOd: ',self.traj.O.dq[i],', dOControl: ',self.gpiO.u)
                    #print(self.gpiX.intU,self.gpiY.intU,self.gpiO.intU)
                    self.robot.updateRobotSpeed(self.gpiX.u,self.gpiY.u,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,0.0,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,self.gpiY.u,0.0)
                    #robot.updatePosition(controlSignal=controlador.u,sampleTime=tSample/1000)
                    if (tAnt - tin)//1000 > 10:
                        return True

    def startDecouplingFollower(self):
        xi=1*self.robot.location[0]
        yi=1*self.robot.location[0]
        tAnt = time.time_ns() / 1_000_000
        tin = tAnt
        while True:
            tAct = time.time_ns() / 1_000_000
            tSample = tAct - tAnt
            if tSample > 100:
                #print(tSample/1000)
                tAnt = tAct
                #self.robot.openGripper()
                if self.f_stop:
                    self.robot.updateRobotSpeed(0.0,0.0,0.0)
                else: 
                    self.gpiX.gpiControl(qd=xi,dqd=0.0,q=1*self.robot.location[0],sampleTime=tSample/1000)
                    self.gpiY.gpiControl(qd=yi-500,dqd=0.0,q=1*self.robot.location[1],sampleTime=tSample/1000)
                    self.gpiO.gpiControl(qd=0.0,dqd=0.0,q=1*self.robot.location[2],sampleTime=tSample/1000)
                    #print('Xd: ',self.traj.X.q[i],', Xactual: ',self.robot.location[0],'Yd: ',self.traj.Y.q[i],', Yactual: ',self.robot.location[1],'Od: ',self.traj.O.q[i],', Oactual: ',self.robot.location[2])
                    #print('dXd: ',self.traj.X.dq[i],', dXControl: ',self.gpiX.u,'dYd: ',self.traj.Y.dq[i],', dYControl: ',self.gpiY.u,'dOd: ',self.traj.O.dq[i],', dOControl: ',self.gpiO.u)
                    #print(self.gpiX.intU,self.gpiY.intU,self.gpiO.intU)
                    self.robot.updateRobotSpeed(self.gpiX.u,self.gpiY.u,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,0.0,self.gpiO.u)
                    #self.robot.updateRobotSpeed(0.0,self.gpiY.u,0.0)
                    #robot.updatePosition(controlSignal=controlador.u,sampleTime=tSample/1000)
                    if (tAnt - tin)//1000 > 10:
                        return True

    def invertTrajectory(self):
        self.traj.invertTrajectory()


if __name__ == '__main__':
    xCSVPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','x.csv')
    yCSVPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','y.csv')
    oCSVPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','or.csv')
    exampleTraj = Trajectory2D(xPath=xCSVPath,yPath=yCSVPath,orPath=oCSVPath,step=0.1,orIn=(np.pi)/2)
    print('X: ',exampleTraj.X.q)
    print('Y: ',exampleTraj.Y.q)
    print('O: ',exampleTraj.O.q)