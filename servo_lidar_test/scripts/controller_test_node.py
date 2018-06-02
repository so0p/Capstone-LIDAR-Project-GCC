#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from servo_lidar_test.msg import Num
from servo_lidar_test.msg import pointCloud
from servo_lidar_test.msg import servo
from servo_lidar_test.srv import controller_client

myRanges= []
angleOfIncrement = 0
startAngle = 0


servoAngle = 0
servoOffset = 0.3316 # Offseting servo
start_scan = 1



# def callbackOne(msg):
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
#     rospy.loginfo('x: {}, y: {}'. format(x, y))

def get_lidar_data(msg):
     
  
    global myRanges    # Needs to be global to be used in the main
    global angleOfIncrement
    global startAngle
    global rangeTime

    myRanges = msg.ranges[:]
    angleOfIncrement = msg.angle_increment
    startAngle = msg.angle_min
    rangeTime = msg.header

    #rospy.loginfo(myRanges)
     

def get_servo_data(msg):

    global servoAngle
    global servoTime
    servoTime = msg.header
    servoAngle = msg.servoAngle + servoOffset
    print(servoTime)

     

def handle_start(req):

    start_scan = req.start 
    print  (req.start)
    return (req.start)


def main():

    rospy.init_node('controller_test')
    s = rospy.Service('controller_service_name', controller_client, handle_start) ####Vaneh
    
    controller_publish_rate = 10 # Node Publish Rate


    print("Starting the controller node.....")

    count = 0 
    #coordinates = []
    x = []
    y = []
    z = []
    desiredRangeMax = 6
    desiredRangeMin = 0.1

    ######t = input('hi, should I start Scanning?') #####input from user node
   
    #--------------------------------------------------
    # Publisher Setup
    #--------------------------------------------------
    pub = rospy.Publisher('pointCloud', pointCloud, queue_size=10)
    #pub2 = rospy.Publisher('custom_chatter', Num, queue_size=10)
    

    
    #--------------------------------------------------
    # Subscriber Setup
    #--------------------------------------------------
    #rospy.Subscriber("/odom", Odometry, callbackOne) For Odometry
    rospy.Subscriber("/scan", LaserScan, get_lidar_data)
    rospy.Subscriber("servo_cmd", servo, get_servo_data)
    
    

    
    rospy.sleep(1.)
    rate = rospy.Rate(controller_publish_rate)  # The while loop rate
    print("controller node started!")
    
    msg = pointCloud()
    #msg.pointCloudRanges = []

    #header 
    msg.header.stamp = rospy.Time.now()
    

    #--------------------------------------------------
    # Loop
    #--------------------------------------------------
    
    while not rospy.is_shutdown():
        
        rayAngle = startAngle
        index = 0
        for r in myRanges:

            rayAngle = startAngle + index * angleOfIncrement + math.pi / 2.0
            incrementedServoAngle = servoAngle + index * 0.000117 

            index = index + 1
            
            
            if(r > desiredRangeMax or r < desiredRangeMin):

                r = 0 


            #------------To publish single coordinates of x,y,z------------
            # y.append(r * math.cos(servoAngle) * math.sin(rayAngle))
            # z.append (r * math.sin(servoAngle) * math.sin(rayAngle))
            # x.append(r * math.cos(rayAngle))

            y.append(r * math.cos(incrementedServoAngle) * math.sin(rayAngle))
            z.append (r * math.sin(incrementedServoAngle) * math.sin(rayAngle))
            x.append(r * math.cos(rayAngle))

        msg.x = x[:]
        msg.y = y[:]
        msg.z = z[:]
            

        pub.publish(msg)
        del x[:]
        del y[:]
        del z[:]


      
    
        rate.sleep()

         
        
    rospy.spin()

if __name__ == '__main__':
           
        main()