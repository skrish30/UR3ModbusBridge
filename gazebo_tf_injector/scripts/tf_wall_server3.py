#!/usr/bin/env python  

import tf2_ros
import tf2_geometry_msgs
import message_filters 

from util import *

import rospy
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Vector3Stamped, Quaternion, PointStamped
# from ais_effector.srv import get_wall_position, get_wall_positionResponse
from sensor_msgs.msg import Range, LaserScan
from std_srvs.srv import Trigger, TriggerResponse


class WallServer(object):
    def __init__(self):
        
        #tf2
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()

        
        self.laser1_Sub = message_filters.Subscriber('/ur3/laser1/scan', LaserScan)
        self.laser2_Sub = message_filters.Subscriber('/ur3/laser2/scan', LaserScan)
        self.laser3_Sub = message_filters.Subscriber('/ur3/laser3/scan', LaserScan)

        self.ts = message_filters.TimeSynchronizer([self.laser1_Sub, self.laser2_Sub, self.laser3_Sub], 2)
        self.ts.registerCallback(self.laser_cb)

        # self.service = rospy.Service('get_wall_tf', get_wall_position , self.handle_wall_position)


    def laser_cb(self, range1, range2, range3):

        v = Vector3Stamped()
        v.vector = Vector3(1,1,1) 
        wall_points = []

        try:
            trans = self.tfBuffer.lookup_transform("world", "ee_link" , rospy.Time() , rospy.Duration(2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/world"
        t.child_frame_id = "ee_link_up"
        t.transform.translation = trans.transform.translation
        t.transform.rotation = Quaternion(0, 0.7070727, 0, 0.7071408 )
        self.broadcaster.sendTransform(t)


        for r,name in zip([range1.ranges[1], range2.ranges[1], range3.ranges[1]], ["ur3_laser1_link", "ur3_laser2_link", "ur3_laser3_link"]):
            v = PointStamped()
            v.header.stamp = rospy.Time()
            v.header.frame_id = name
            v.point.x = r
            v.point.y = 0
            v.point.z = 0
            out = self.tfBuffer.transform(v, "ee_link_up", rospy.Duration(5))

            wall_points.append(np.array([out.point.x,out.point.y,out.point.z]))

        n = get_normal(wall_points[0],wall_points[1],wall_points[2])
        rospy.loginfo(n)
        forward_v = np.array([1,0,0])
        theta = angle_between(forward_v, -n *100000)
        rospy.loginfo(theta)

        # rospy.loginfo(wall_points)
        rospy.loginfo("\n")


def main():
    rospy.init_node("tf_wall_server")
    rate = rospy.Rate(10)

    w = WallServer()

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()