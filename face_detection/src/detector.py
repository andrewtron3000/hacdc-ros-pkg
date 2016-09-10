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
import roslib; roslib.load_manifest('face_detection')
import rospy
import sys
import cv
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

#
#  Instantiate a new opencv to ROS bridge adaptor
#
cv_bridge = CvBridge()

#
# Define the callback that will be called when a new image is received.
#
def callback(publisher, coord_publisher, cascade, imagemsg):
    #
    #  Convert the ROS imagemsg to an opencv image.
    #
    image = cv_bridge.imgmsg_to_cv(imagemsg, 'mono8')

    #
    #  Blur the image.
    #
    cv.Smooth(image, image, cv.CV_GAUSSIAN)

    #
    #  Allocate some storage for the haar detect operation.
    #
    storage = cv.CreateMemStorage(0)

    #
    #  Call the face detector function.
    #
    faces = cv.HaarDetectObjects(image, cascade, storage, 1.2, 2, 
                                 cv.CV_HAAR_DO_CANNY_PRUNING, (100,100))
 
    #
    #  If faces are detected, compute the centroid of all the faces
    #  combined.
    #
    face_centroid_x = 0.0
    face_centroid_y = 0.0
    if len(faces) > 0:
        #
        #  For each face, draw a rectangle around it in the image,
        #  and also add the position of the face to the centroid
        #  of all faces combined.
        #
        for (i, n) in faces:
            x = int(i[0])
            y = int(i[1])
            width = int(i[2])
            height = int(i[3])
            cv.Rectangle(image, 
                         (x, y),
                         (x + width, y + height),
                         cv.CV_RGB(0,255,0), 3, 8, 0)
            face_centroid_x += float(x) + (float(width) / 2.0)
            face_centroid_y += float(y) + (float(height) / 2.0)
        #
        #  Finish computing the face_centroid by dividing by the
        #  number of faces found above.
        #
        face_centroid_x /= float(len(faces))
        face_centroid_y /= float(len(faces))
        #
        #  Lastly, if faces were detected, publish a PointStamped 
        #  message that contains the centroid values.
        #
        pt = Point(x = face_centroid_x, y = face_centroid_y, z = 0.0)
        pt_stamped = PointStamped(point = pt)
        coord_publisher.publish(pt_stamped)

    #
    #  Convert the opencv image back to a ROS image using the 
    #  cv_bridge.
    #
    newmsg = cv_bridge.cv_to_imgmsg(image, 'mono8')

    #
    #  Republish the image.  Note this image has boxes around 
    #  faces if faces were found.
    #
    publisher.publish(newmsg)

def listener(publisher, coord_publisher):
    rospy.init_node('face_detector', anonymous=True)
    #
    #  Load the haar cascade.  Note we get the 
    #  filename from the "classifier" parameter
    #  that is configured in the launch script.
    #
    cascadeFileName = rospy.get_param("~classifier")
    cascade = cv.Load(cascadeFileName)
    rospy.Subscriber("/stereo/left/image_rect", 
                     Image, 
                     lambda image: callback(publisher, coord_publisher, cascade, image))
    rospy.spin()

# This is called first.
if __name__ == '__main__':
    publisher = rospy.Publisher('face_view', Image)
    coord_publisher = rospy.Publisher('face_coords', PointStamped)
    listener(publisher, coord_publisher)
