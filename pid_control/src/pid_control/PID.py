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

#
# PID controller adapted from "PID without a PhD" written by
# Tim Wescott
#
import roslib; roslib.load_manifest('pid_control')
import rospy
from pid_control.msg import PIDDiagnostics

class PIDController:
    def __init__(self, namespace, commandMin, commandMax, iMin, iMax):
        self._namespace = namespace 
        self._commandMin= commandMin
        self._commandMax= commandMax
        self._iMin = iMin 
        self._iMax= iMax
        self._iState = 0.0
        self._dState = None
        self._Publisher = rospy.Publisher(self._namespace + '/diagnostics', PIDDiagnostics)
    def update(self, error, position):
        # get the gains from the parameter server
        try: 
            gains = rospy.get_param(self._namespace)          
            p, i, d = gains['p'], gains['i'], gains['d']

            # proportional term
            pTerm = p * error

            # integral term
            self._iState += error
            if self._iState > self._iMax:
                self._iState = self._iMax
            elif self._iState < self._iMin:
                self._iState = self._iMin
            iTerm = i * self._iState

            # derivitive term
            if self._dState == None:
                self._dState = position
            dTerm = d * (self._dState - position)
            self._dState = position

            # compute command
            command = pTerm + iTerm + dTerm
            if command > self._commandMax:
                command = self._commandMax
            elif command < self._commandMin:
                command = self._commandMin

            # publish diagnostics
            self._Publisher.publish(PIDDiagnostics(error = error, command = command))
        except:
            rospy.logwarn('PID gains not set')
            command = 0.0

        return command
