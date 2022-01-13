#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from yolo_ros.msg import ShipInfo
from sbg_driver.msg import SbgEkfQuat, SbgEkfNav
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion

import numpy as np

def talker():
    quat_pub = rospy.Publisher('Quat', SbgEkfQuat)
    nav_pub = rospy.Publisher('Nav', SbgEkfNav)
    rospy.init_node('Centrale_inertielle', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    num_message = 0
    while not rospy.is_shutdown():
        # msg = ShipInfo()
        # msg.altitude = np.random.random()
        # msg.latitude = 42.99247 + np.random.random()*0.1
        # msg.longitude = 5.97839 + np.random.random()*0.1
        # msg.roll = 150.8 + np.random.random()*10
        # msg.tilt = -1.4 + np.random.random()
        # msg.yaw = num_message
        # num_message = (num_message + 1)
        # rospy.loginfo(msg)
        # pub.publish(msg)
        # rate.sleep()
        # Equivalent to [20,1.5,0.8] rotation
        # rot = R.from_euler(
        #     'zyx',
        #     [15 + np.random.random()/10, 1 + np.random.random()/10, 0.2 + np.random.random()/10]
        # )
        rot = R.from_euler(
            'zyx',
            [15, -1, 0]
        )
        x,y,z,w = rot.as_quat()
        
        msg_quat = SbgEkfQuat(
            quaternion = Quaternion(
                x=x, y=y,z=z,w=w
                )
        )

        msg_nav = SbgEkfNav(
            latitude = 42.52961333333333 + np.random.random()/100,
            longitude = 5.6977166666666665 + np.random.random()/100,
            altitude = np.random.random() - 0.5
        )
        
        
        rospy.loginfo(msg_quat)
        rospy.loginfo(msg_nav)
        
        quat_pub.publish(msg_quat)
        nav_pub.publish(msg_nav)
        
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
