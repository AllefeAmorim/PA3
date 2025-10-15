from is_wire.core import Channel, Message, Subscription
from google.protobuf.wrappers_pb2 import FloatValue
from is_msgs.common_pb2 import Pose
import socket
import numpy as np
import time

# Cria canal e subscription
channel = Channel("amqp://guest:guest@10.10.2.211:30000")
subscription = Subscription(channel)

# Envia requisição para iniciar detecção por 5 minutos
request = Message(content=FloatValue(value=60.0), reply_to=subscription)
channel.publish(request, topic="Tiffany.StartDetections")



# Espera resposta de confirmação
try:
    reply = channel.consume(timeout=5.0)
    print('RPC Status:', reply.status)
except socket.timeout:
    print('No reply :(')

# Loop contínuo para pegar a pose
x_ref = 1.5
y_ref = 2.0

while True:
    # Requisição de pose
    request = Message(reply_to=subscription)
    channel.publish(request, topic="Tiffany.GetPose")

    try:
        reply = channel.consume(timeout=2.0)
        pose: Pose = reply.unpack(Pose)
        #print(f"Pose recebida: {pose}")
        #print(type(pose))
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        ex = x_ref - x#(-1)#x
        ey = y_ref - y#(2)#y
        alpha = np.rad2deg(np.arctan2(ey,ex))
        print(f"Graus:{alpha}\n")
        yaw = pose.orientation.yaw
        print(f"Posição Atual\nX: {x}\nY: {y}\nZ: {z}\nOrientação: {yaw}")
        #print(f"Posição Indicada\nX: {x_ref}\nY: {y_ref}")
    except socket.timeout:
        print("Sem resposta de pose...")

    # Intervalo entre as leituras
    time.sleep(1.0)
