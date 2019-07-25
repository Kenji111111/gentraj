#!/usr/bin/env python2
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math

from mg_msgs.msg import PVAYStampedTrajectory
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Twist, Vector3, Point, Quaternion

import numpy as np

def callback(data):
    print('recieved callback')
    points = data.trajectory
    posx = [v.pos.x for v in points]
    posy = [v.pos.y for v in points]
    posz = [v.pos.z for v in points]

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(posx, posy, posz)
    plt.show()


if __name__ == '__main__':
    rospy.init_node('traj_listener', anonymous=True)
    sub = rospy.Subscriber("/trajectory", PVAYStampedTrajectory, callback)

    rospy.sleep(1)
    pub = rospy.Publisher('/ENU/local_odom', Odometry, queue_size=5)
    position = Point(0,0,0)
    orientation = Quaternion(0,0,math.sqrt(0.5),math.sqrt(0.5))
    covariance = np.zeros(36)
    pose = Pose(position, orientation)
    posec = PoseWithCovariance(pose, covariance)
    linear = Vector3(0,0,0)
    angular = Vector3(0,0,0)
    twist = Twist(linear, angular)
    twistc = TwistWithCovariance(twist, covariance)
    stamp = rospy.Time.now()
    header = Header(0, stamp, '/traj_listener')
    odom = Odometry(header, '0', posec, twistc)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(odom)
        r.sleep()

    rospy.spin()