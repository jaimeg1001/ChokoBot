import numpy as np
import matplotlib.pyplot as plt
import time

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
cwd = os.path.abspath(os.getcwd())

from Trajectory import Trajectoria

class GPIController:
    def __init__(self,k:float):
        self.k = k
        self.u = 0.0
    """
    def getCoefficients(self):
        coeffs = np.convolve( np.array([1, self.p]), np.array([1, 2*self.z*self.wn, self.wn**2]) )
        return [coeffs[3],coeffs[2],coeffs[1]]
    """
    def gpiControl(self,qd:float,dqd:float,q:float,sampleTime:float):
        self.u = dqd - self.k * (q - qd)

class TestRobot:
    def __init__(self,q0:float,dq0:float,d2q0:float):
        self.q = q0
        self.dq = dq0
        self.d2q = d2q0 
    
    def updatePosition(self,controlSignal,sampleTime):
        self.dq = self.d2q + controlSignal*sampleTime
        self.q = self.q + self.dq*sampleTime

if __name__ == '__main__':
    controlador = GPIController(wn=1.0, z=2.0, p=3.0)
    csvPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','x.csv')
    traj = Trajectoria(csvPath,step=0.1)
    robot = TestRobot(q0=5.0,dq0=0.0,d2q0=0.0)
    posRobot = [robot.q]
    fig, ax = plt.subplots()
    ax.plot(traj.t, traj.q)
    ax.set(xlabel='time (s)', ylabel='distance (mm)', title='Test trajectory')
    ax.grid()
    i = 0
    tAnt = time.time_ns() / 1_000_000
    while True:
        tAct = time.time_ns() / 1_000_000
        tSample = tAct - tAnt
        if tSample > 10:
            #print(tSample/1000)
            tAnt = tAct
            ax.plot(traj.t[i], robot.q,"*r")
            controlador.gpiControl(qd=traj.q[i],dqd=traj.dq[i],d2qd=traj.d2q[i],q=robot.q,sampleTime=tSample/1000)
            robot.updatePosition(controlSignal=controlador.u,sampleTime=tSample/1000)
            i += 1
            if i > (len(traj.t)-1):
                break
    
    plt.show()
