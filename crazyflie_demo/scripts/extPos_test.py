#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped, Point

def onNewTransform(point):
    global msg
    global pub
    msg.header.frame_id = "/vicon/crazyflie/crazyflie"
    msg.header.stamp = rospy.Time.now()
    msg.header.seq += 1
    msg.point.x = point.x
    msg.point.y = point.y
    msg.point.z = point.z
    pub.publish(msg)
    
def onNewTransformBroad(point):
    global msg2
    global pub2
    msg2.header.frame_id = "/vicon/crazyflie/crazyflie"
    msg2.header.stamp = rospy.Time.now()
    msg2.header.seq += 1
    msg2.point.x = point.x
    msg2.point.y = point.y
    msg2.point.z = point.z
    pub2.publish(msg2)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vicon', anonymous=True)
    topic = rospy.get_param("~topic", "/vicon/cf/cf")

    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg2 = PointStamped()
    msg2.header.seq = 0
    msg2.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    pub2 = rospy.Publisher("external_position_broad", PointStamped, queue_size=1)
    rospy.Subscriber("/fakePos", Point, onNewTransform)
    rospy.Subscriber("/fakePosBroad", Point, onNewTransformBroad)

    rospy.spin()
