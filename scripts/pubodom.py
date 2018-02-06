#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
#from std_msgs.msg import Float64
from time import sleep
#import serial
import tf
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point,pose #Quaternion以下を追加
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3,TransformStamped

#初期化 宣言
rospy.init_node('mea_odom_publisher') # mea_odom_publisherというノードを初期化
odom_pub = rospy.Publisher("mea_odom", Odometry, queue_size=50)  #mes_odomというトピック名 odmetry 型のメッセージ
odom_broadcaster = tf.TransformBroadcaster() #tf TransformBroadcasterというクラスがありその型のodom_broadcasterをつくたったついでにコンストラクタをしている



current_time = rospy.Time.now()
rate = rospy.Rate(100)#周期を決めている
odom_data = Odometry()
mbed_odom = []
mbed_odom = [0,0,0,0,0,0]
#ser = serial.Serial('/dev/sensors/keisoku',9600)
sleep(1)
#mbed_odom = map(float,ser.readline().split())
while not rospy.is_shutdown():
    
    current_time = rospy.Time.now()
    mbed_odom = map(float,ser.readline().split()) #読み取り部分
    print mbed_odom

    if len(mbed_odom) == 6:
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, mbed_odom[2])#thは角度を入れる
        odom_broadcaster.sendTransform((mbed_odom[0],mbed_odom[1],0.0), odom_quat, current_time,"base_link2","mea_odom")
    
        #odom
        odom_data.header.frame_id = "mea_odom"
        odom_data.child_frame_id = "base_link2"
        odom_data.header.stamp = current_time
        
        odom_data.pose.pose.position = Point(mbed_odom[0],mbed_odom[1],0.)
        odom_data.pose.pose.orientation = Quaternion(*odom_quat)
        #odom.pose.pose = Pose(Point(mbed_odom[0], mbed_odom[1], 0.), Quaternion(*odom_quat)) #Quaternion型にキャスト

           
        odom_data.pose.covariance = [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
       
        
        odom_data.twist.twist.linear.x = mbed_odom[3]
        odom_data.twist.twist.linear.y = mbed_odom[4]
        odom_data.twist.twist.angular.z = mbed_odom[5]

        #odom.twist.twist = Twist(Vector3(mbed_odom[3], mbed_odom[4], 0), Vector3(0, 0, mbed_odom[5]))
        odom_pub.publish(odom_data)# caution object name 
        #last_time = current_time iいる？
    rate.sleep()

    


  
