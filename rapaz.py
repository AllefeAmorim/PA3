from is_wire.core import Channel, Message, Subscription
from google.protobuf.wrappers_pb2 import FloatValue
from is_msgs.common_pb2 import Pose
import socket
import numpy as np
import time

import socket

class StreamChannel(Channel):
    def __init__(self, uri="amqp://guest:guest@10.10.2.211:30000", exchange="is"):
        super().__init__(uri=uri, exchange=exchange)

    def consume_last(self, return_dropped=False):
        dropped = 0
        try:
            msg = super().consume(timeout=0.1)
        except socket.timeout:
            return False

        while True:
            try:
                # will raise an exceptin when no message remained
                msg = super().consume(timeout=0.0)
                dropped += 1
            except socket.timeout:
                return (msg, dropped) if return_dropped else msg

class Posinha:
    x = 0.0
    y = 0.0
    z = 0.0
    yaw = 0.0

# Cria canal e subscription
channel5 = StreamChannel("amqp://guest:guest@10.10.2.211:30000")
channel9 = StreamChannel("amqp://guest:guest@10.10.2.211:30000")
subscription5 = Subscription(channel5)
subscription9 = Subscription(channel9)
subscription5.subscribe(f"reconstruction.5.ArUco")
subscription9.subscribe(f"reconstruction.9.ArUco")

def getPose(channel):
    message = channel.consume_last()
    if type(message) != bool:
        posemsg = message.unpack(Pose)
        pose = Posinha()
        pose.x = posemsg.position.x
        pose.y = posemsg.position.y
        pose.z = posemsg.position.z
        pose.yaw = np.rad2deg(posemsg.orientation.yaw)
        return pose
    return False

while True:
    pose5 = getPose(channel5)  
    pose9 = getPose(channel9)    
    if type(pose5) != bool and type(pose9) != bool:
        print(f'pose5 - x = {pose5.x} m  y = {pose5.y} m  z = {pose5.z} m yaw = {pose5.yaw} deg')
        print(f'pose9 - x = {pose9.x} m  y = {pose9.y} m  z = {pose9.z} m yaw = {pose9.yaw} deg')
        # #print(f"Pose recebida: {pose}")
        # #print(type(pose))
        x = pose9.x
        y = pose9.y
        x_ref = pose5.x
        y_ref = pose5.y
        # # MOL
        ex = x_ref - x#(-1)#x
        ey = y_ref - y#(2)#y
        alpha = np.rad2deg(np.arctan2(ey,ex))
        beta = pose9.yaw
        erro = np.sqrt(ey*ey + ex*ex)
        theta = alpha - beta
        print(f"Graus:{alpha}")
        print(f"Erro:{erro}\n")
        print(f"Vai Scarlett:{(beta)}")
        if erro > 0.4:
            print(f'VAI(5,{theta})')
        # yaw = pose.orientation.yaw
        # print(f"Posição Atual\nX: {x}\nY: {y}\nZ: {z}\nOrientação: {yaw}")
        # #print(f"Posição Indicada\nX: {x_ref}\nY: {y_ref}")

    # Intervalo entre as leituras
    time.sleep(0.1)



# http://10.10.2.211:30300/ArUco.1.FrameTransformations