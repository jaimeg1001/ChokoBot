# Librerias generales
import struct
import threading as th
from numpy import pi
from time import sleep
import json
angulo = 0.0
# Modulos del sistema operativo y creados por usuario
import sys
import os

# Configuración para lectura de modulos y obtención de path's
cwd = os.path.abspath(os.getcwd())

sys.path.append('../')
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append('Chokobot')

# Configuración del puerto Serial
import serial
ser = serial.Serial('/dev/ttyS0',
                    baudrate=115200,
					parity=serial.PARITY_NONE,
					stopbits=serial.STOPBITS_ONE)

def resetESP32():
    ser.write(b'X')
    sleep(5)
    ser.reset_output_buffer()
    ser.reset_input_buffer()
    sleep(0.1)


# Carga de las trayectorias
from Modules.PathFollowing import Trajectory2D
#xCSVPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','x33.csv')
#yCSVPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','y33.csv')
#oCSVPath = os.path.join(cwd,'Chokobot','BezierPoints','ExamplePoints','or33.csv')
xCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\x3.csv')
yCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\y3.csv')
oCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\or3.csv')

trajCarga = Trajectory2D(xPath=xCSVPath,yPath=yCSVPath,orPath=oCSVPath,step=0.1,orIn=0)

xCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\x1.csv')
yCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\y1.csv')
oCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\or1.csv')

trajDescarga = Trajectory2D(xPath=xCSVPath,yPath=yCSVPath,orPath=oCSVPath,step=0.1,orIn=0)

xCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\x4.csv')
yCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\y4.csv')
oCSVPath = os.path.join(cwd,'Chokobot','BezierPoints',r'FinalPoints\or4.csv')

trajRest = Trajectory2D(xPath=xCSVPath,yPath=yCSVPath,orPath=oCSVPath,step=0.1,orIn=0)

    
#Inicializacion de canal MQTT
RobotID=2
from Modules.mqttRobot import MqttPublisher,SensorData
# Inicialización del robot omnidireccional
from Modules.Omnidireccional import Omnidireccional
robotOmni = Omnidireccional(posInX=0.0,velInX=0.0,posInY=0.0,velInY=0.0,orIn=0.0,dorIn=0.0,radius=30,distW=115,distH=130)
sensor = SensorData(id=RobotID)
    # Set callback functions for tasks

# Controlador GPI
from Modules.PathFollowing import ControlOmnidireccional
controlP = ControlOmnidireccional(kx=0.4,ky=0.3,ko=5,trajectory=trajRest,robot=robotOmni,f_stop=sensor.f_Stop)


def goCarga():
    sensor.f_EO=1
    sensor.f_work=True
    sensor.f_debug=True
    
def goDescarga():
    sensor.f_EO=2
    sensor.f_work=True
    sensor.f_debug=True
        
def goHome():
    sensor.f_EO=3
    sensor.f_work=True
    sensor.f_debug=True
    
def emergencyStop():
    controlP.f_stop=True
    data={"UR":sensor.id,"Task":"Paro de emergencia","Estatus":"Detenido"}
    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
    
def abrirAcopl():
    sensor.f_servo=False
    data={"UR":sensor.id,"Task":"Abrir Griper"}
    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)

def cerrarAcopl():
    sensor.f_servo=True
    data={"UR":sensor.id,"Task":"Cerrar Gripper"}
    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)

def emergencyStopFree():
    controlP.f_stop=False
    data={"UR":sensor.id,"Task":"Paro de emergencia","Estatus":"Reanudado"}
    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)

def loadCharge():
    sensor.f_carga=True
    data={"UR":sensor.id,"Task":"Carga colocada"}
    sensor.carga_colocada="Si"
    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)

def loadDischarge():
    sensor.f_carga=False
    data={"UR":sensor.id,"Task":"Carga recogida"}
    sensor.carga_colocada="No"
    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
    
def liderInitAcopl():
    sensor.f_work=True
    controlP.f_acoplamiento=False
    sensor.f_seguidor=False
    sensor.f_EO=4
    
def followerInitAcopl():
    sensor.f_work=True
    controlP.f_acoplamiento=False
    sensor.f_seguidor=True
    sensor.f_EO=4
    
def liderConfrmAcop():
    sensor.f_work=True
    controlP.f_acoplamiento=True
    sensor.f_EO=0
    sensor.f_servo=True

def followerConfrmAcop():
    sensor.f_work=True
    controlP.f_acoplamiento=True
    sensor.f_EO=5
    
def liberarAcoplamiento():
    sensor.f_work=True
    sensor.f_servo=False
    controlP.f_lider=True
    sensor.f_EO=0
    
    
# Lectura de ESP32 y control de motores

def sensorLecture():
    # Aquí va el codigo para leer los sensores
    #ser.reset_input_buffer()
    global angulo
    while True:
        received_data = ser.read(48)#read serial port
        [N1,N2,N3,N4,V1,V2,V3,V4,V,C,P,angulo] = struct.unpack('iiiiffffffff',received_data)
        robotOmni.updateWheelSpeed(wheelSpeedBL=V3,wheelSpeedBR=V4,wheelSpeedFL=V2,wheelSpeedFR=V1)
        robotOmni.controlWheelSpeed()


# Hilo para el nodo de ROS2
from Modules.socketOptitrack import socketOptiTrack
from time import sleep
client = socketOptiTrack()
import math
def optitrackLecture():
    global angulo,V,C
    while True:
        try:
            #print(client.body_pos1[0],client.body_pos1[1],math.radians(angulo))
            robotOmni.updateLocation([client.body_pos2[0],client.body_pos2[1],math.radians(angulo)])
            #controlP.UR=[client.body_pos1,client.body_pos2,client.body_pos3]
        #controlP.URpos[0]=[client.body_pos1[0],client.body_pos1[1],math.radians(angulo)]
        #controlP.URpos[1]=[client.body_pos2[0],client.body_pos2[1],0.0]
        #controlP.URpos[2]=[client.body_pos3[0],client.body_pos3[1],0.0]
        #robotOmni.updateLocation([0.0,0.0,math.radians(angulo)])
        #robotOmni.updateLocation([client.body_pos1[0],0.0,math.radians(angulo)])
        #robotOmni.updateLocation([0.0,0.0,0.0])
            #sensor.posicionDir=angulo
            #sensor.posicionX=client.body_pos1[0]
            #sensor.posicionY=client.body_pos1[1]
            #sensor.porcentaje_bateria=(12600*V/100)
            #sensor.consumo_corriente=C        
            sleep(0.01)
        except:
            print('Connection failed')
        #print('Robot Pos: ', client.body_pos1)

if __name__ == '__main__':
    #Reseteo del buffer de lectura
    # Codigo para lectura del nodo de ROS que contiene las información de la posición del sistema de camaras Optitrak
    ser.reset_input_buffer()
    # Hilo para lectura de ESP32
    dataESP = th.Thread(target=sensorLecture,daemon=True)
    dataESP.start()
    # Hilo para lectura del sistema Optitrack
    dataOptitrack = th.Thread(target=optitrackLecture,daemon=True)
    # Inicialización de los hilos
    dataOptitrack.start()
    
    # Programa principa
    sleep(1)
    mqtt_publisher = MqttPublisher(client_id="UR"+str(sensor.id),broker_address="192.168.174.174",topic="UR",answer_topic="Resp",subscribe_topic="Task",id=sensor.id,get_new_values_callback=sensor.get_json_data)
    mqtt_publisher.set_task_callback(1,goCarga)
    mqtt_publisher.set_task_callback(2,goDescarga)
    mqtt_publisher.set_task_callback(3,goHome)
    mqtt_publisher.set_task_callback(4,abrirAcopl)
    mqtt_publisher.set_task_callback(5,cerrarAcopl)
    mqtt_publisher.set_task_callback(6,emergencyStop)
    mqtt_publisher.set_task_callback(7,emergencyStopFree)
    mqtt_publisher.set_task_callback(8,loadCharge)
    mqtt_publisher.set_task_callback(9,loadDischarge)
    
    mqtt_publisher.set_task_callback(11,liderInitAcopl)
    mqtt_publisher.set_task_callback(12,followerInitAcopl)
    mqtt_publisher.set_task_callback(13,liderConfrmAcop)
    mqtt_publisher.set_task_callback(14,followerConfrmAcop)
    mqtt_publisher.set_task_callback(15,liberarAcoplamiento)
    sleep(1)
    mqtt_publisher.run()
    sensor.f_EO=0
    controlP.f_servo=False
    sensor.f_debug=False
    sensor.f_work=True
    while True:    
        if sensor.f_EO==0: #Espera
            sensor.f_debug=False
            controlP.alto()
            
        elif sensor.f_EO==1: #Recoleccion
            if sensor.f_work:
                sensor.f_work=False
                data={"UR":sensor.id,"Task":"Recoger objeto","Estatus":"Aceptada"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.estado_operativo="En recolección"
                sensor.f_work=controlP.startController(trajectory=trajCarga)
                data={"UR":sensor.id,"Task":"Recoger objeto","Estatus":"Finalizda"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.estado_operativo="En espera"
                if sensor.f_debug:
                    sensor.f_EO=0
                else:
                    sensor.f_EO=2
            else:
                data={"UR":sensor.id,"Task":"Recoger objeto","Estatus":"Rechazada"}
                #  print("UR busy")
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
        
        elif sensor.f_EO==2: #Entrega
            if sensor.f_work:
                sensor.f_work=False
                #while(sensor.f_carga==True):
                #    data={"UR":sensor.id,"SOL":"Favor de confirmar Carga"}
                #    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                #    sleep(1)
                data={"UR":sensor.id,"Task":"Entregar objeto","Estatus":"Aceptada"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.estado_operativo="En entrega"
                sensor.f_work=controlP.startController(trajectory=trajDescarga)
                data={"UR":sensor.id,"Task":"Entregar objeto","Estatus":"Finalizda"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.estado_operativo="En espera"
                if sensor.f_debug:
                    sensor.f_EO=0
                else:
                    sensor.f_EO=3
            else:
                data={"UR":sensor.id,"Task":"Entregar objeto","Estatus":"Rechazada"}
            #  print("UR busy")
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
        elif sensor.f_EO==3: #Casa
            if sensor.f_work:
                sensor.f_work=False
                #while(sensor.f_carga==True):
                #    data={"UR":sensor.id,"SOL":"Confirmar Descarga"}
                #    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                #    sleep(1)
                data={"UR":sensor.id,"Task":"Zona de descanso","Estatus":"Aceptada"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.estado_operativo="Hacia zona de descanso"
                sensor.f_work=controlP.startController(trajectory=trajRest)
                data={"UR":sensor.id,"Task":"Zona de descanso","Estatus":"Finalizda"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.estado_operativo="En espera"
                sensor.f_EO=0
                    
            else:
                data={"UR":sensor.id,"Task":"Zona de descanso","Estatus":"Rechazada"}
                print("UR busy")
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                
        elif sensor.f_EO==4:
            if sensor.f_work:
                sensor.f_work=False
                if sensor.f_seguidor:                  
                    data={"UR":sensor.id,"Task":"Iniciar proceso de acoplamiento","Estatus":"Aceptada","Modo":"Seguidor"}
                    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                    sensor.f_work=controlP.startCouplingFollower()
                    data={"UR":sensor.id,"Task":"Iniciar proceso de acoplamiento","Estatus":"Finalizada","Modo":"Seguidor"}
                    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                else:              
                    data={"UR":sensor.id,"Task":"Iniciar proceso de acoplamiento","Estatus":"Aceptada","Modo":"Lider"}
                    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                    sensor.f_work=controlP.startCouplingLider()
                    data={"UR":sensor.id,"Task":"Iniciar proceso de acoplamiento","Estatus":"Finalizada","Modo":"Lider"}
                    mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                    robotOmni.closeGripper()
                    
        elif sensor.f_EO == 5:
            if sensor.f_work:
                sensor.f_work=False
                data={"UR":sensor.id,"Task":"Iniciar controlador de acoplamiento","Estatus":"Aceptada","Modo":"Seguidor"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
                sensor.f_work=controlP.startControllerFollower(lider=1)
                data={"UR":sensor.id,"Task":"Iniciar controlador de acoplamiento","Estatus":"Finalizada","Modo":"Seguidor"}
                mqtt_publisher.publish_data(mqtt_publisher.topic_resp,data)
     
     
"""
    #resetESP32()
        #else:
            #print(angulo)    
            #
    
    while True:
        velFR = float(input("Ingrese la velocidad en RPM deseada: "))
        velFL = float(input("Ingrese la velocidad en RPM deseada: "))
        velBR = float(input("Ingrese la velocidad en RPM deseada: "))
        velBL = float(input("Ingrese la velocidad en RPM deseada: "))
        robotOmni.updateWheelSetPoint(wheelSetPFR=velFR,wheelSetPFL=velFL,wheelSetPBR=velBR,wheelSetPBL=velBL)
    """