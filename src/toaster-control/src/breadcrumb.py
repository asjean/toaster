#!/usr/bin/env python3

import rospy
import time
import serial
from std_msgs.msg import String

try:
    ser = serial.Serial()
    ser.port = '/dev/ttyACM0'
    ser.baudrate = 115200
except Exception as e:
    print(e)
    pass
print(ser)

ser.open()

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():
	rospy.init_node('BreadcrumbListener', anonymous=True)
	rospy.Subscriber('breadcrumb/data', String, callback)
	rospy.spin()

def breadcrumb():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('breadcrumb', anonymous=True)
    rate = rospy.Rate(10) # 10hz
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
