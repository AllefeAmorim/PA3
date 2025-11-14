from is_wire.core import Channel, Message, Subscription
from google.protobuf.wrappers_pb2 import FloatValue
from google.protobuf.struct_pb2 import Struct
from google.protobuf import json_format
from is_msgs.common_pb2 import Pose
import socket
import numpy as np
import time
import cv2


# Cria canal e subscription
channel = Channel("amqp://guest:guest@10.10.2.211:30000")
subscription = Subscription(channel)

# Envia requisição para iniciar detecção por 5 minutos
request = Message(content=FloatValue(value=60.0), reply_to=subscription)
channel.publish(request, topic="Tiffany.StartDetections2")

def publica_vel(canal,modulo,angulo):
    img_msg = Message()
    img_msg.body = (str(modulo)+","+str(angulo)).encode("utf-8")
    canal.publish(img_msg, topic='tiffany.vel')
    print("tiffany.vel",modulo,angulo)


# # Detector ArUco com refinamento
# parameters = cv2.aruco.DetectorParameters()
# parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# arucoDetector = cv2.aruco.ArucoDetector(dictionary, parameters)

# # Dados de cameras e inscricao
# camera_ids = [1, 2, 3, 4]
# calib = {}
# P = {}
# for i in camera_ids:
#     data = np.load(f"/calib_rt{i}.npz")
#     calib[i] = {key: data[key] for key in ['K', 'dist', 'nK', 'roi', 'rt']}
#     P[i] = calib[i]['nK'] @ calib[i]['rt']

# for cam_id in camera_ids:
#     subscription.subscribe(f"UndistortedCamera.{cam_id}.Frame")
#     # subscription.subscribe(f"Tiffany.{cam_id}.Detection")

# def detectAruco(frame, arucoIds = [0]):
#     arucoSet = {}
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     markerCorners, markerIds, _ = arucoDetector.detectMarkers(gray)

#     if markerIds is not None:
#         for corners, marker_id in zip(markerCorners, markerIds.flatten()):
#             if marker_id in arucoIds:
#                 corners = corners[0]
#                 center_x = np.mean(corners[:,0])
#                 center_y = np.mean(corners[:,1])
#                 arucoSet[marker_id] = np.array([center_x,center_y])
#     return arucoSet

# Espera resposta de confirmação
try:
    reply = channel.consume(timeout=5.0)
    print('RPC Status:', reply.status)
except socket.timeout:
    print('No reply :(')

# Loop contínuo para pegar a pose
x_ref = 0#1.5
y_ref = 0#2.0

publica_vel(channel,-10,0)
time.sleep(2.0)
publica_vel(channel,-10,0)
time.sleep(2.0)
print('vou nessa hein!!!')
time.sleep(10.0)

xa = 0
ya = 0
za = 0
count = 0
pwmx = 0
pwmy = 0
kp = 1
ki = 0
Ix = 0
Iy = 0
tempoa = time.time()
while True:
    # Requisição de pose
    msg = Struct()
    msg.fields['timestamp'].bool_value = True
    request = Message(reply_to=subscription)
    request.pack(msg)
    channel.publish(request, topic="Tiffany.GetPose2")

    if count == 5:
        publica_vel(channel,0,0)

    try:
        reply = channel.consume(timeout=2.0)
        data = reply.unpack(Struct)
        data = json_format.MessageToDict(data)
        pose = json_format.ParseDict(data.get("pose", {}),Pose())
        #tempo = data.get("timestamp")
        #deltat = time.time() - tempo # em segundos
        tempo = time.time()
        deltat = tempo - tempoa
        # #print(f"Pose recebida: {pose}")
        # #print(type(pose))
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        # MOL
        ex = x_ref - x#(-1)#x
        ey = y_ref - y#(2)#y
        Ix += ex*ki*deltat
        Iy += ey*ki*deltat
        pwmx = ex*kp + Ix
        pwmy = ey*kp + Iy
        alpha = np.rad2deg(np.arctan2(ey,ex))   # angulo ponto desejado-centro do robo
        print(f"Graus:{alpha}\n")
        beta = pose.orientation.yaw              # angulo dela no lab
        theta = alpha - beta
        erro = np.sqrt(ey*ey + ex*ex)
        print(f"Posição Atual\nX: {x}\nY: {y}\nZ: {z}\nOrientação: {beta}")
        print(f"Graus:{alpha}")
        print(f"Erro:{erro}\n")
        if erro > 0.3:
            alphaf = np.rad2deg(np.arctan2(pwmy,pwmx))   # angulo ponto desejado-centro do robo
            while alphaf > 180:
                alphaf -= 360
            while alphaf < -180:
                alphaf += 360
            publica_vel(channel,5,alphaf+90)
            print(f'VAI(5,{alphaf})')
        else:
            publica_vel(channel,0,0)
        #print(f"Posição Indicada\nX: {x_ref}\nY: {y_ref}")
        xa = x+0
        ya = y+0
        za = z+0

    except socket.timeout:
        print("Sem resposta de pose...")
    if xa == x and ya == y and za == z:
        count += 1
    else:
        count = 0
    tempoa = tempo
    # Intervalo entre as leituras
    time.sleep(0.1)
