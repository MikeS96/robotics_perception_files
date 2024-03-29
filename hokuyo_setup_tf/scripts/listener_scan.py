#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection

from sklearn.cluster import DBSCAN


class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.ranges)
    rospy.loginfo("/n")
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
