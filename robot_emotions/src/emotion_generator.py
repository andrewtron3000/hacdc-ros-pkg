#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_emotions')
import rospy

# Import the EmotionalState message from robot_emotions/msg
from robot_emotions.msg import EmotionalState

# Import the SensorPacket message from irobot_create_2_1/msg
from irobot_create_2_1.msg import SensorPacket

# Define the callback that will be called when a new
# SensorPacket is received.  This callback is responsible
# for also *publishing* a new robot emotional state 
# message, using some function of the sensorPacket data.
def callback(publisher, sensorpacket):
    # The robot is scared of higher voltages!
    scared_value = float(sensorpacket.voltage) / 65535.0
    # Form the new emotion message.
    msg = EmotionalState(happiness = 1.0,
                         sadness = 0.0,
                         angriness = 0.0,
                         scaredness = scared_value,
                         tenderness = 0.0,
                         excitedness = 1.0)
    # Publish the emotion state message.
    publisher.publish(msg)

# Define the listener function that installs the 
# "callback" function and then just spins forever, 
# not doing anything else except calling the callback
# on each new received SensorPacket message.
def listener(publisher):
    # Initialize the node with the name "emotion_node"
    rospy.init_node('emotion_node', anonymous=True)
    # Subscribe to the sensorPacket topic, calling
    # the "callback" function with each new packet.
    # lambda is a python builtin in function --
    # look at the python documentation to learn more
    rospy.Subscriber("sensorPacket", 
                     SensorPacket, 
                     lambda sensorpacket: callback(publisher, sensorpacket))
    rospy.spin()

# This is called first.
if __name__ == '__main__':
    # Create a publisher object that publishes to the
    # "robot_emotions" topic, and it publishes a message
    # of type EmotionalState
    publisher = rospy.Publisher('robot_emotions', EmotionalState)
    # Call the listener and never return
    listener(publisher)
