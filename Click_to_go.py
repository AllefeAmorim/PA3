from is_wire.core import Channel, Message, Subscription
from google.protobuf.wrappers_pb2 import FloatValue
from is_msgs.common_pb2 import Pose
import socket
import time

channel =Channel("amqp://guest:guest@10.10.2.211:30000")
subscription=Subscription(channel)

request = Message(content=FloatValue(value=60.0), reply_to=subscription)
channel.publish(request, topic="Tiffany.StartDetections")


try:
    reply = channel.consume(timeout=5.0)
    print('RPC Status: ', reply.status)
except socket.timeout:
    print('No reply :(')

# Wait a little to let detections run
time.sleep(5.0)
