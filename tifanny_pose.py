from is_wire.core import Channel, Message, Subscription
from google.protobuf.duration_pb2 import Duration
from is_msgs.common_pb2 import Pose
import socket
import time

# Create a channel and a subscription for replies
channel = Channel("amqp://guest:guest@10.10.2.211:30000")
subscription = Subscription(channel)

# Start detection threads for 5 minutes
request = Message(content=Duration(seconds=3600), reply_to=subscription)
channel.publish(request, topic="Tiffany.StartDetections2")
channel.publish(request, topic="Tiffany.Keypoints.4.StartStream")

# Wait for reply with timeout
try:
    reply = channel.consume(timeout=25.0)
    print('RPC Status: ', reply.status)
except socket.timeout:
    print('No reply :(')

# Wait a little to let detections run
time.sleep(5.0)

# Request the latest pose
request = Message(reply_to=subscription)


while True:
    channel.publish(request, topic="Tiffany.GetPose2")
    try:
        reply = channel.consume(timeout=5.0)
        print('RPC Status: ', reply.status)
        pose = reply.unpack(Pose)
        print('Pose:', pose)
    except socket.timeout:
        print('No reply :(')