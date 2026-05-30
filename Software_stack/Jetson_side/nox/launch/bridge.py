#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3Stamped

def cb_cmd(msg):
    vs = Vector3Stamped()
    vs.header.stamp = rospy.Time.now()
    # map linear.x -> vector.x, angular.z -> vector.z
    vs.vector.x = msg.linear.x
    vs.vector.y = msg.linear.y
    vs.vector.z = msg.angular.z
    pub.publish(vs)

rospy.init_node('cmdvel_to_speed')
pub = rospy.Publisher('/speed', Vector3Stamped, queue_size=1)
sub = rospy.Subscriber('/cmd_vel', Twist, cb_cmd)
rospy.spin()
