from is_wire.core import Channel, Message, Subscription
from google.protobuf.wrappers_pb2 import FloatValue
from is_msgs.common_pb2 import Pose
import socket
import numpy as np
import time
import cv2

def detectAruco(frame, arucoIds = [0]):
    arucoSet = {}
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    markerCorners, markerIds, _ = arucoDetector.detectMarkers(gray)

    if markerIds is not None:
        for corners, marker_id in zip(markerCorners, markerIds.flatten()):
            if marker_id in arucoIds:
                corners = corners[0]
                center_x = np.mean(corners[:,0])
                center_y = np.mean(corners[:,1])
                arucoSet[marker_id] = np.array([center_x,center_y])
    return arucoSet

def rvecToR(rvec):
    R, _ = cv2.Rodrigues(rvec)
    return R

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])   # roll
        y = np.arctan2(-R[2,0], sy)      # pitch
        z = np.arctan2(R[1,0], R[0,0])   # yaw
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])  # row, pitch, yaw

# Cria canal e subscription
channel = Channel("amqp://guest:guest@10.10.2.211:30000")
subscription = Subscription(channel)

# Detector ArUco com refinamento
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoDetector = cv2.aruco.ArucoDetector(dictionary, parameters)
marker_length = 

# Dados de cameras e inscricao
camera_ids = [1, 2, 3, 4]
calib = {}
P = {}
for i in camera_ids:
    data = np.load(f"/calib_rt{i}.npz")
    calib[i] = {key: data[key] for key in ['K', 'dist', 'nK', 'roi', 'rt']}
    calib[i]['r'] = calib[i]['rt'][:,:3]
    calib[i]['t'] = np.array(calib[i]['rt'][:,3], dtype=float).reshape(3,1)
    P[i] = calib[i]['nK'] @ calib[i]['rt']

for cam_id in camera_ids:
    subscription.subscribe(f"UndistortedCamera.{cam_id}.Frame")
    # subscription.subscribe(f"Tiffany.{cam_id}.Detection")


# Espera resposta de confirmação
try:
    reply = channel.consume(timeout=5.0)
    print('RPC Status:', reply.status)
except socket.timeout:
    print('No reply :(')

# Loop contínuo para pegar a pose
x_ref = 0.0
y_ref = 0.0

while True:
    # Requisição de pose
    request = Message(reply_to=subscription)
    channel.publish(request, topic="Tiffany.GetPose")

    try:
        reply = channel.consume(timeout=2.0)
        pose: Pose = reply.unpack(Pose)
        # #print(f"Pose recebida: {pose}")
        # #print(type(pose))
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        # MOL
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
