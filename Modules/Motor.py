from gpiozero import LED, PWMLED
import simple_pid 

class Motor:
    def __init__(self,PWM,dir1,dir2,setpoint,posIn,velIn):
        self.lPWM = PWMLED(PWM,frequency=5000)
        self.dir1 = LED(dir1)
        self.dir2 = LED(dir2)
        self.count = posIn
        self.speed = velIn
        self.setpoint = setpoint
        self.controlador = simple_pid.PID(Kp=5,Ki=4,Kd=0,setpoint=self.setpoint,sample_time=None,output_limits=(-254,254))

    def moveMotor(self,dir:int,valPWM:float):
            self.lPWM.value = valPWM
            if dir == 1:            
                self.dir1.on()
                self.dir2.off()
            else:
                self.dir1.off()
                self.dir2.on()

    def stopMotor(self):
            self.lPWM.off()
            self.dir1.off()
            self.dir2.off()

    def setSetpoint(self,newSetPoint):
        self.setpoint = newSetPoint
        self.controlador.setpoint = self.setpoint

    def controlVel(self):
            controlOut = self.controlador(self.speed)
            PWMval = abs(controlOut) / 255
            #print("PWM: ", PWMval)
            if controlOut < 0:
                self.moveMotor(0,PWMval)
            else:
                self.moveMotor(1,PWMval)
                
                