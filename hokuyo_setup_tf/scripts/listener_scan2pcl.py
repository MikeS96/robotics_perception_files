#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
from rospy.numpy_msg import numpy_msg
from sensor_msgs import point_cloud2 as pc2c
import numpy as np
import matplotlib as plt

from sklearn.cluster import DBSCAN


class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPCL", pc2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.laserCallback)

    def laserCallback(self, data):
        cloud_out = self.laserProj.projectLaser(data)
        Xg = pc2c.read_points(cloud_out, skip_nans=True, field_names=("x", "y", "z"))
        cloud_points = np.empty((cloud_out.width, 2))
        a = 0
        for p in Xg:
            #rospy.loginfo(p)
            #rospy.loginfo(p[0])
            cloud_points[a, 0] = p[0]
            cloud_points[a, 1] = p[1]
            #cloud_points[a, 2] = p[2]
            #rospy.loginfo(cloud_points[a,:])
            a=a+1

        #for d in cloud_out.data:
        #    print(d<<4)

        #rospy.loginfo(cloud_points)

        # Cluster PCL:
        #scaler = StandardScaler()
        #scaler.fit(cloud_points)
        dbscan= DBSCAN()
        clusters = dbscan.fit_predict(cloud_points)
	#print("Cluster Membership:\n{}".format(clusters))

	

        self.pcPub.publish(cloud_out)
        rospy.loginfo("Start Clusters")
        rospy.loginfo(clusters)
        rospy.loginfo("Start Clusters")

if __name__ == '__main__':
    rospy.init_node('laser2PC', anonymous=True)
    l2pc = Laser2PC()
    rospy.spin()
