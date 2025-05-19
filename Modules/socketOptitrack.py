#!/usr/bin/env python3
import logging
import time
from NatNetClient import NatNetClient
import math

logger = logging.getLogger('autokin')

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class socketOptiTrack():
    """
    Para enviar comandos de actuación a motores del robot
    y leer datos de posición enviados por las cámaras.
    """
    def __init__(self):
        logger.info("Iniciando instancia externa")
        self.body_recv_semaphore = False
        self.body_pos1 = None
        self.body_pos2 = None
        self.body_pos3 = None
        self.connect_cam()
        
    def connect_cam(self):
        self.cam_client_connected = False
        self.cam_client = NatNetClient()
        self.cam_client.newFrameListener = self.recv_frame
        self.cam_client.rigidBodyListener = self.recv_body
        try:
            self.cam_client.run()
        except OSError:
            logger.info("Falló conexión de NatNet")
        else:
            logger.info("Sockets de NatNet conectados")

    def test_cam_connection(self):
        self.cam_client_connected = False
        time.sleep(0.1) # Esperar un frame para registrar estado de conexión
        return self.cam_client_connected

    def recv_frame(self, frameNumber, 
                   markerSetCount,unlabeledMarkersCount,
                   rigidBodyCount,skeletonCount,labeledMarkerCount,
                   timecode,timecodeSub,timestamp,
                   isRecording,trackedModelsChanged):
        if not self.cam_client_connected:
            logger.info(f"NatNet recibiendo datos: {rigidBodyCount} cuerpos rìgidos")
            self.cam_client_connected = True # Brincar if y correr sólo esta línea?

    def recv_body(self,id, position, rotation):
        # called once per rigid body per frame
        body_pos = None
        body_pos = [i * 1000 for i in position]
        xr,yr,zr,wr=rotation
        xb, yb, zb = euler_from_quaternion(xr,yr,zr,wr)
        if id==1: 
            self.body_pos1=[-1*body_pos[0],1*body_pos[2],yb]
            #print(1,body_pos)
            #print(self.body_pos1,id)
        elif id==2:
            self.body_pos2=[-1*body_pos[0],1*body_pos[2],yb]
            #print(self.body_pos2,id)
        elif id==3:
            self.body_pos3=[-1*body_pos[0],1*body_pos[2],yb]
            #print(self.body_pos3,id)
        #print(body_pos,id)
        
    def status(self):
        cam_status = self.test_cam_connection()

        if not cam_status:
            logger.info("NatNet desconectado, reintentando")
            self.connect_cam()
            cam_status = self.test_cam_connection()

        return (cam_status)


if __name__ == "__main__":
    logging.basicConfig(level=logging.NOTSET)
    client = socketOptiTrack()
    input()
    logger.info("Node finished")

