#!/usr/bin/env python3
import signal
import time
import board
import busio
import adafruit_bno08x
from adafruit_bno08x import (BNO_REPORT_ACCELEROMETER,BNO_REPORT_GYROSCOPE,BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_MAGNETOMETER)
from adafruit_bno08x.i2c import BNO08X_I2C
import rospy
from sensor_msgs.msg import MagneticField, Imu
from std_msgs.msg import Float64
def imu_node():
    dataPublisher = rospy.Publisher('imu/data', Imu, queue_size=10)
    #magneticPublisher = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rospy.init_node('imu')
    rate = rospy.Rate(100)
    rospy.loginfo('Imu Initialized')
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c,address=0x4a)
    bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(BNO_REPORT_GYROSCOPE)
    #bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    time.sleep(3.0)
    while not rospy.is_shutdown():
        data_msg = Imu()
        data_msg.header.stamp = rospy.Time.now()
        data_msg.header.frame_id = 'imu_link'
        ax,ay,az = bno.acceleration
        data_msg.linear_acceleration.x = ax
        data_msg.linear_acceleration.y = ay
        data_msg.linear_acceleration.z = az
        gx,gy,gz = bno.gyro
        data_msg.angular_velocity.x = gx
        data_msg.angular_velocity.y = gy
        data_msg.angular_velocity.z = gz
        ow,ox,oy,oz = bno.quaternion
        data_msg.orientation.w = ow
        data_msg.orientation.x = ox
        data_msg.orientation.y = oy
        data_msg.orientation.z = oz
        data_msg.orientation_covariance[0] = 0.01 #tune these covariances to create accurate sensor fused data
        data_msg.linear_acceleration_covariance[0] = 0.01
        data_msg.angular_velocity_covariance[0] = 0.01
        dataPublisher.publish(data_msg)
        #mag_msg = MagneticField()
        #mx,my,mz = bno.magnetic
        #mag_msg.header.stamp = rospy.Time.now()
        #mag_msg.magnetic_field.x = mx
        #mag_msg.magnetic_field.y = my
        #mag_msg.magnetic_field.z = mz
        #mag_msg.magnetic_field_covariance[0] = -1
        #magneticPublisher.publish
        rate.sleep()

    rospy.loginfo("Imu Shutdown")

def shutdown_hook():
    print("Imu Node Shutdown")
rospy.on_shutdown(shutdown_hook)
if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Imu node exited with exception.")

