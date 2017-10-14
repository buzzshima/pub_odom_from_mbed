#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from time import sleep
import serial


rospy.init_node('talker')
pub = rospy.Publisher('odom',Float64,queue_size=10)
rate = rospy.Rate(10)
ser = serial.Serial('/dev/sensors/keisoku',9600)
tamo = []
sleep(1)
while not rospy.is_shutdown():
    tamo = map(float,ser.readline().split())
    #print tamo
    hello_str = Float64()
    hello_str.data = tamo[2]
    pub.publish(hello_str)
    rate.sleep