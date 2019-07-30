#!/usr/bin/env python2
import rospy
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import tf
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from mg_msgs.msg import PVAYStampedTrajectory, PVAYStamped
from geometry_msgs.msg import Pose, Twist, Accel, Vector3, Point, Quaternion

published = False

def gen_traj(dist, accel, max_vel, frequency):

    # NOTE: in the future, cap velocity to min(sqrt(acc * dist), max_vel)

    accel_time = max_vel/accel
    accel_dist = 0.5*accel*accel_time*accel_time
    cruise_dist = dist - 2 * accel_dist
    cruise_time = cruise_dist/max_vel
    total_time = accel_time*2 + cruise_time
    period = 1.0/frequency

    accel_points = math.ceil(accel_time * frequency)
    cruise_points = math.ceil(cruise_time * frequency)

    points = []

    for i in range(int(accel_points)):
        curr_time = i * period
        curr_vel = curr_time * accel
        curr_pos = 0.5 * accel * curr_time * curr_time

        v = Vector3(curr_pos, curr_vel, accel)
        points.append(v)

    travelled = 0.5 * (accel * math.pow(accel_points * period, 2))
    for i in range(int(cruise_points)):
        curr_time = i * period
        curr_pos = max_vel * curr_time + travelled

        v = Vector3(curr_pos, max_vel, 0)
        points.append(v)

    travelled += max_vel * cruise_points * period
    for i in range(int(accel_points) + 1):
        curr_time = i * period
        curr_vel = max_vel - curr_time * accel
        curr_pos = travelled + max_vel * curr_time - 0.5 * accel * curr_time * curr_time

        v = Vector3(curr_pos, curr_vel, -accel)
        points.append(v)

    pos = [v.x for v in points]
    vel = [v.y for v in points]
    acc = [v.z for v in points]

    # plt.plot(pos)
    # plt.plot(vel)
    # plt.plot(acc)
    # plt.show()

    return points

def callback(data):
    global published
    if published:
        return
    print('Generating trajectory...')
    # generate trajectory
    max_height = rospy.get_param('~max_height')
    distance = rospy.get_param('~forward_dist')
    velocity = rospy.get_param('~velocity')
    acceleration = rospy.get_param('~acceleration')
    frequency = rospy.get_param('~frequency')
    # publish to /trajectory

    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    origin = data.pose.pose.position

    up = gen_traj(max_height, acceleration, velocity, frequency)
    forward = gen_traj(distance, acceleration, velocity, frequency)

    points = []

    seq = 0
    
    for pt in up:
        point = Point(origin.x, origin.y, origin.z + pt.x)
        
        linear_v = Vector3(0, 0, pt.y)
        angular_v = Vector3(0, 0, 0)
        vel = Twist(linear_v, angular_v)

        linear_a = Vector3(0, 0, pt.z)
        angular_a = Vector3(0, 0, 0)
        acc = Accel(linear_a, angular_a)

        stamp = rospy.Time.now()
        header = Header(seq, stamp, '/gentraj')
        seq += 1
        point = PVAYStamped(header, point, yaw, vel, acc)

        points.append(point)

    for pt in forward:
        point = Point(origin.x + math.cos(yaw) * pt.x, origin.y + math.sin(yaw) * pt.x, origin.z + max_height)
        
        linear_v = Vector3(pt.y, 0, 0)
        angular_v = Vector3(0, 0, 0)
        vel = Twist(linear_v, angular_v)

        linear_a = Vector3(pt.z, 0, 0)
        angular_a = Vector3(0, 0, 0)
        acc = Accel(linear_a, angular_a)

        stamp = rospy.Time.now()
        header = Header(seq, stamp, '/gentraj')
        seq += 1
        point = PVAYStamped(header, point, yaw, vel, acc)
        
        points.append(point)

    for pt in up[::-1]:
        point = Point(origin.x + math.cos(yaw) * distance, origin.y + math.sin(yaw) * distance, origin.z + pt.x)
        
        linear_v = Vector3(0, 0, -pt.y)
        angular_v = Vector3(0, 0, 0)
        vel = Twist(linear_v, angular_v)

        linear_a = Vector3(0, 0, -pt.z)
        angular_a = Vector3(0, 0, 0)
        acc = Accel(linear_a, angular_a)

        stamp = rospy.Time.now()
        header = Header(seq, stamp, '/gentraj')
        seq += 1
        point = PVAYStamped(header, point, yaw, vel, acc)

        points.append(point)

    # posx = [v.pos.x for v in points]
    # posy = [v.pos.y for v in points]
    # posz = [v.pos.z for v in points]

    # fig = plt.figure()
    # ax = Axes3D(fig)
    # ax.scatter(posx, posy, posz)
    # plt.show()
    
    # publish to /trajectory
    pub = rospy.Publisher('/trajectory', PVAYStampedTrajectory, queue_size=5)
    traj = PVAYStampedTrajectory(points)
    rospy.sleep(7)
    pub.publish(traj)
    print('published!')
    published = True

def publishENULocalOdom():
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

if __name__ == '__main__':

    rospy.init_node('trajectory', anonymous=True)

    poseTopic = rospy.get_param('~quadPoseTopic')
    sub = rospy.Subscriber(poseTopic, Odometry, callback)
    rospy.spin()