#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from servo_lidar_test.msg import Num
from servo_lidar_test.msg import controller
from servo_lidar_test.msg import servo


myRanges= []
angleOfIncrement = 0
startAngle = 0


servoAngle = 0


# def callbackOne(msg):
#     x = msg.pose.pose.position.x
#     y = msg.pose.pose.position.y
#     rospy.loginfo('x: {}, y: {}'. format(x, y))

def callbackTwo(msg):
     
  
    global myRanges    # Needs to be global to be used in the main
    global angleOfIncrement
    global startAngle

    myRanges = msg.ranges[:]
   
    angleOfIncrement = msg.angle_increment
    startAngle = msg.angle_min

    #rospy.loginfo(myRanges)
     

def callbackThree(msg):

    global servoAngle
    servoAngle = msg.servoAngle
     # Offseting servo




def main():

    rospy.init_node('controller_test')
    
    define_publish_rate = 6000 # Node Publish Rate


    print("Starting the controller node.....")

    count = 0 
    coordinates = []

    ######t = input('hi, should I start Scanning?') #####input from user node
   
    #--------------------------------------------------
    # Publisher Setup
    #--------------------------------------------------
    pub = rospy.Publisher('controller', controller, queue_size=10)
    #pub2 = rospy.Publisher('custom_chatter', Num, queue_size=10)
    

    
    #--------------------------------------------------
    # Subscriber Setup
    #--------------------------------------------------
    #rospy.Subscriber("/odom", Odometry, callbackOne) For Odometry
    rospy.Subscriber("/scan", LaserScan, callbackTwo)
    rospy.Subscriber("servo_cmd", servo, callbackThree)
    
    

    
    rospy.sleep(1.)
    rate = rospy.Rate(define_publish_rate)  # The while loop rate
    print("controller node started!")
    
    msg = controller()
    #msg.pointCloudRanges = []

    #header 
    msg.header.stamp = rospy.Time.now()
    
    # msg2 = Num()
    # msg2.num = 4
    # msg2.name = "A"

    #--------------------------------------------------
    # Loop
    #--------------------------------------------------
    
    while not rospy.is_shutdown():

        
        
        rayAngle = startAngle
        index = 0
        for r in myRanges:

            rayAngle = startAngle + index * angleOfIncrement + math.pi / 2.0
            index = index + 1
             
            #r = 1 
            
            #------------To publish single coordinates of x,y,z------------
            msg.y = r * math.cos(rayAngle) * math.cos(servoAngle + (33.75*math.pi/180))
            msg.x = r * math.sin(rayAngle) * math.cos(servoAngle + (33.75*math.pi/180))
            msg.z = r * math.sin(servoAngle + (33.75*math.pi/180))


             


            ## -------- To publish an array of x,y,z------------------------
            # y = r * math.cos(rayAngle) * math.cos(servoAngle + (33.75*math.pi/180))
            # x = r * math.sin(rayAngle) * math.cos(servoAngle + (33.75*math.pi/180))
            # z = - r * math.sin(servoAngle)

            # coordinates.append([x,y,z])
            # msg.pointCloudCoordinates = coordinates
            pub.publish(msg)
            #rospy.loginfo(rayAngle)

      
        
        
        
        
    
        rate.sleep()

         
        
    rospy.spin()

if __name__ == '__main__':
           
        main()