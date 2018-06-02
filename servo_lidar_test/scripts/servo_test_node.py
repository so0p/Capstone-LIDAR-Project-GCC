#!/usr/bin/env python
import os 
from lib_robotis import * 
import os.path 

import rospy
from std_msgs.msg import String
from servo_lidar_test.msg import Num
from servo_lidar_test.msg import servo
 
 #-----------------------------------------------------------------
 #Need this part to be able to run the servo
dyn = USB2Dynamixel_Device('/dev/ttyUSB0')
servo_offset= math.radians(-24)
p = Robotis_Servo( dyn, 1, series = "MX" )

#-------------------------------------------------------------------
#--------------------Our main is defined here----------------------- 
def main():


#-------------------------------------------------------------------
#---------------------------Variables-------------------------------
  define_rate = 30 
  #amount that servo reads is not the same as the amount the servo moves
  #to make sure servo moves to our desired ranges, we have added this offset
  upperBound= 2500 #2500 in bits
  lowerBound= 1200 #1200 in bits
  boundOffset= 1500 #amount that servo reads is not the same as the amount the servo moves so 
  #to make sure servo moves to our desired ranges, we have added this offset
  home=0 #in radians
  servo_speed = 1 # radians/sec

#--------------------------------------------------------------------
#--------------------------Publisher node----------------------------
  servo_pub = rospy.Publisher('servo_cmd', servo, queue_size=10)
  #(topic name, message file(.srv) name, and queue size)

#---------------------------------------------------------------------
#If you want to recieve what position the servo should go to create the rest of this part 
#rospy.Subscriber("controller", controller, callbackThree) #what position to go


#--------------------------------------------------------------------
#--------------------------Node Name---------------------------------
  rospy.init_node('servo_test')


#--------------------------------------------------------------------
#----------------------Setting the rate---------------------------
  rate = rospy.Rate(define_rate)


#------------------------Code starts running here--------------------
  msg3 = servo() #accessing the variable names in the servo message file 
    
  count = 0

  p.move_angle(home + servo_offset) #offset because of the shaft 
  print(math.degrees(p.read_angle())) 
  p.set_angvel(servo_speed) #speed of the servo motor

  while not rospy.is_shutdown(): 
      

   while count < upperBound: #Going from encoder position 0 to 2500
     p.move_to_encoder(boundOffset) #Making sure servo moves to the max encoder position 2500
     msg3.servoAngle = p.read_angle() #assigns the angle read to the variable servoAngle
     servo_pub.publish(msg3) #publishing the message file
     count = p.read_encoder()#Count changes as servo is going to 4000 
     #rospy.loginfo(math.degrees(p.read_angle()))


   while count > lowerBound: #Going back down from encoder position 2500
      msg3.servoAngle = p.read_angle()
      p.move_to_encoder(boundOffset) #making sure servo goes down by addign the offset
      count = p.read_encoder() 
      servo_pub.publish(msg3)

      #rospy.loginfo(math.degrees(p.read_angle()))

         # for i in range(4,2040):
         #     p.move_to_encoder(i)
         #     msg3.servoAngle = p.read_angle()
         #     pub.publish(msg3)



        
         # t.secs=10
         #pub2.publish(msg3)
         #rospy.loginfo(msg3)
   rate.sleep() 
   
  rospy.spin()
  
if __name__ == '__main__':
           
        main()
        if(rospy.is_shutdown()):
          print("Home------------------------------------------------------------------")
        