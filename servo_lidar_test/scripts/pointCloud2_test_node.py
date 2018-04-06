#!/usr/bin/env python
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
from servo_lidar_test.msg import controller

x = 0
y = 0
z = 0

def callbackForCordinates(msg):

    global x
    global y
    global z

    x = msg.x
    y = msg.y
    z = msg.z




if __name__ == '__main__':

    define_publish_rate = 7000

    number_of_data_poins = 10000 # Number of data points to be published


    rospy.init_node('pointCloud2_test')

    rate = rospy.Rate(define_publish_rate)  # The while loop rate

    #--------------------------------------------------
    # Subscriber Setup
    #---------------------------------------------------

    rospy.Subscriber('controller', controller, callbackForCordinates) # Subscribes to Adrik's lidar



    #--------------------------------------------------
    # Publisher Setup
    #--------------------------------------------------

    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2, queue_size=10)



    rospy.loginfo("Initializing sample pcl2 publisher node...")
    
    #give time to roscore to make the connections
    rospy.sleep(1.)
    
    
    
    #header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'laser'
    
    
    
    
    
    #publish    
    rospy.loginfo("happily publishing sample pointcloud.. !")
    count = 0
    cloud_points = []

    while not rospy.is_shutdown():
        #count += 1



        #x1 = math.sin(count)
        #x2 = math.sin(count + math.pi)

        
        #cloud_points = [[x1, 1.0, 1.0],[x2, 2.0, 0.0]]
        
        if(x != 0 or y!=0 or z!=0):

            cloud_points.append([x, y, z])

        
        if (len(cloud_points) > number_of_data_poins):

            del(cloud_points[0])





        #create pcl from points
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)

        pcl_pub.publish(scaled_polygon_pcl)

        rate.sleep()

    rospy.spin()


        

