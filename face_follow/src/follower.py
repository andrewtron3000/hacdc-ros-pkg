#!/usr/bin/env python
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
import roslib; roslib.load_manifest('face_follow')
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from pid_control import PID
from pid_control.msg import PIDDiagnostics
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

pid_controller = PID.PIDController('face_follow', -0.2, 0.2, -0.2, 0.2)
twist_publisher = rospy.Publisher("/cmd_vel", Twist)

def listener():
    rospy.init_node('listener', anonymous=True)
    while True:
       try:
          data = rospy.wait_for_message("face_coords", PointStamped, timeout=0.5)
          control = pid_controller.update(352.0/2.0 - data.point.x, data.point.x)
          twist_publisher.publish(Twist(angular = Vector3(x = 0.0, y = 0.0, z = control)))
       except:
          twist_publisher.publish(Twist(angular = Vector3(x = 0.0, y = 0.0, z = 0.0)))
       rospy.sleep(1.0)

if __name__ == '__main__':
    listener()
