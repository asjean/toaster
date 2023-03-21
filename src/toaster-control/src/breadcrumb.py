#!/usr/bin/env python3

import rospy
import time
import serial
import scipy
from scipy.optimize import least_squares
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped.msg

try:
    ser = serial.Serial()
    ser.port = '/dev/breadcrumb0'
    ser.baudrate = 115200
except Exception as e:
    print(e)
    pass


ser.open()

def equations( guess ):
    x,y,z,r = guess
    return((x - x1)**2 + (y - y1)**2 + (z - z1)**2 - (dist1 - r )**2,
        (x - x2)**2 + (y - y2)**2 + (z - z2)**2 - (dist2 - r )**2,
        (x - x3)**2 + (y - y3)**2 + (z - z3)**2 - (dist3 - r )**2)

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
	rospy.init_node('BreadcrumbListener', anonymous=True)
	rospy.Subscriber('breadcrumb/data', String, callback)
	rospy.spin()
def estimate_pose 

def estimate_point(coords, dists):
    """
    Estimates the (x,y) coordinates of an unknown point given distances from
    multiple fixed points.

    :param coords: list of (x,y) coordinates of fixed points
    :type coords: list of tuples
    :param dists: list of distances from fixed points to unknown point
    :type dists: list of floats
    :return: estimated point and its covariance
    :rtype: geometry_msgs.msg.PoseWithCovariance
    """

    def residual(point, coords, dists):
        # Calculates the residual (difference between estimated and actual distances)
        # for the least squares optimization
        residuals = []
        for i in range(len(coords)):
            d = np.sqrt((point[0]-coords[i][0])**2 + (point[1]-coords[i][1])**2)
            residuals.append(d - dists[i])
        return residuals

    # Set up the initial estimate for the unknown point as the average of the fixed points
    initial_estimate = np.mean(coords, axis=0)

    # Perform least squares optimization to estimate the unknown point
    result = least_squares(residual, initial_estimate, args=(coords, dists))

    # Calculate covariance matrix
    jac = result.jac
    cov = np.linalg.inv(np.dot(jac.T, jac))

    # Format result as PoseWithCovariance message
    pose = PoseWithCovariance()
    pose.pose.position.x = result.x[0]
    pose.pose.position.y = result.x[1]
    pose.pose.position.z = 0.0  # Assuming 2D space
    pose.pose.orientation.w = 1.0 # No orientation information
    pose.covariance[0] = cov[0][0]
    pose.covariance[1] = cov[0][1]
    pose.covariance[6] = cov[1][0]
    pose.covariance[7] = cov[1][1]

    return pose

if __name__ == '__main__':
    rospy.init_node('point_estimation')

    # Load fixed point coordinates and distances from ROS parameter server
    coords = rospy.get_param('fixed_point_coords')
    dists = rospy.get_param('distances')

    # Continuously update coordinates and distances from parameter server and estimate point
    while not rospy.is_shutdown():
        coords = rospy.get_param('fixed_point_coords')
        dists = rospy.get_param('distances')
        estimated_pose = estimate_point(coords, dists)
        rospy.loginfo(f"Estimated pose: {estimated_pose}")
        rospy.sleep(1.0) # sleep for 1 second before checking parameters again
def breadcrumb():
    pub = rospy.Publisher('breadcrumb/pose', String, queue_size=10)
    rospy.init_node('breadcrumb', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        try:
            datastr = ser.readline()
        except Exception as e:
            print(e)
            pass
        except KeyboardInterrupt:
            ser.close()
        ser.close()
        rospy.loginfo(datastr)
        pub.publish(datastr)
        rate.sleep()

def shutdown_hook():
    print("Breadcrumb Shutdown")
rospy.on_shutdown(shutdown_hook)
if __name__ == '__main__':
    try:
        breadcrumb()
    except rospy.ROSInterruptException:
        rospy.loginfo("Breadcrumb exited with exception.")
