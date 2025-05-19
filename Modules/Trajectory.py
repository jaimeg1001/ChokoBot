import numpy as np
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
cwd = os.path.abspath(os.getcwd())


class Trajectoria:
    def __init__(self,csvPath:str,step:float) -> None:
        self.path = csvPath
        self.step = step
        self.q,self.dq,self.d2q = self.getBezier()
        self.t = self.getTimeArray()

    def readCSVFile(self):
        csvData = np.loadtxt(self.path, delimiter=",", dtype=float)
        return csvData
    
    def getBezier(self):
        dataCSV = self.readCSVFile()
        return dataCSV[0],dataCSV[1],dataCSV[2]
    
    def getTimeArray(self):
        sampleLen = len(self.q)
        tf = (sampleLen - 1) * self.step
        ti = 0
        return np.arange(ti,tf+self.step,self.step)
    def invertTrajectory(self):
        self.q = self.q[::-1]
        self.dq = self.dq[::-1]
        self.d2q = self.d2q[::-1]
if __name__ == '__main__':
    csvPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','x.csv')
    exampleTraj = Trajectoria(csvPath,0.1)
    print('Px: ',exampleTraj.q,)
    print('Vx: ',exampleTraj.dq)
    print('Ax: ',exampleTraj.d2q)
    print('Size Px: ',len(exampleTraj.q),', Size Vx: ',len(exampleTraj.dq),', Size Px: ',len(exampleTraj.d2q),'Size t: ',len(exampleTraj.t),', Tf =', exampleTraj.t[-1])