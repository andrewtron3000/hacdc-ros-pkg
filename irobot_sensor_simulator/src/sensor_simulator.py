#! /usr/bin/env python
#*********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011 andrewtron3000
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/
#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_sensor_simulator')
import rospy
import dynamic_reconfigure.server
from irobot_sensor_simulator.cfg import SensorConfig
from irobot_create_2_1.msg import SensorPacket

import threading

data_lock = threading.Lock()
sensor_data = None

def getSimulatedData():
    global data_lock
    data_lock.acquire()
    data = sensor_data
    data_lock.release()
    return data

def setSimulatedData(data):
    global data_lock, sensor_data
    data_lock.acquire()
    sensor_data = data
    data_lock.release()

def generateSensorMessage(simulated_data):
    msg = SensorPacket(wheeldropCaster=simulated_data['wheeldropCaster'],wheeldropLeft=simulated_data['wheeldropLeft'],wheeldropRight=simulated_data['wheeldropRight'],bumpLeft=simulated_data['bumpLeft'],bumpRight=simulated_data['bumpRight'],wall=simulated_data['wall'],cliffLeft=simulated_data['cliffLeft'],cliffFronLeft=simulated_data['cliffFronLeft'],cliffFrontRight=simulated_data['cliffFrontRight'],cliffRight=simulated_data['cliffRight'],virtualWall=simulated_data['virtualWall'],infraredByte=simulated_data['infraredByte'],advance=simulated_data['advance'],play=simulated_data['play'],distance=simulated_data['distance'],angle=simulated_data['angle'],chargingState=simulated_data['chargingState'],voltage=simulated_data['voltage'],current=simulated_data['current'],batteryTemperature=simulated_data['batteryTemperature'],batteryCharge=simulated_data['batteryCharge'],batteryCapacity=simulated_data['batteryCapacity'],wallSignal=simulated_data['wallSignal'],cliffLeftSignal=simulated_data['cliffLeftSignal'],cliffFrontLeftSignal=simulated_data['cliffFrontLeftSignal'],cliffFrontRightSignal=simulated_data['cliffFrontRightSignal'],cliffRightSignal=simulated_data['cliffRightSignal'],homeBase=simulated_data['homeBase'],internalCharger=simulated_data['internalCharger'],songNumber=simulated_data['songNumber'],songPlaying=simulated_data['songPlaying'])
    return msg

def main():
    rospy.init_node("irobot_sensor_simulator")
    publisher = rospy.Publisher('sensorPacket', SensorPacket)
    dynamic_reconfigure.server.Server(SensorConfig, reconfigure)
    while not rospy.is_shutdown():
        data = getSimulatedData()
        if data != None:
            publisher.publish(generateSensorMessage(data))
        rospy.sleep(0.1)

def reconfigure(config, level):
    setSimulatedData(config)
    return config # Returns the updated configuration.

if __name__ == '__main__':
    main()
