import os #1 
from lib_robotis import * #2
import os.path 

import rospy
from std_msgs.msg import String
from servo_lidar_test.msg import Num
from servo_lidar_test.msg import servo
from servo_lidar_test.msg import controller

 
dyn = USB2Dynamixel_Device('/dev/ttyUSB0')
p = Robotis_Servo( dyn, 1, series = "MX" )
p.set_angvel(0.1)

def callbackOne(msg):
	global position
	position= pos 



def main():

	step_pub = rospy.Publisher('encoder_position', pointCloudInfo, queue_size=10)
	#publishing the encoder position
	rospy.Subscriber("pointCloudInfo", position, callbackOne) 
	#subscribing to pointclouds position

	msg4= pointCloudInfo()

	num=0

	while not rospy.is_shutdown():
	













	num= p.read_encoder()
	#publish encoder position
	servo_pub.publish(msg4)
	#publishing the encoder to pointcloudinfo()
	rospy.loginfo(num)


		


	