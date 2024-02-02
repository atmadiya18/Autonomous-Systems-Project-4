#!/usr/bin/env python3
import rospy
import cv2
from PIL import Image
from cv_bridge import CvBridge
import numpy as np;
import time
import argparse
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

#Create bridge object for interfacing between ROS and OpenCV
bridge = CvBridge()

#Initializing Global Variables for the center of the masked object and time
center = [0,0]
starttime = time.time()

class image_processor:

	def __init__(self):
		#Initialize Subscriber to read Image Data
		self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.scan_callback)
	
	
	def scan_callback(self,data):
		#Convert ROS Message Image, into one that OpenCV can use!
		self.cv_image = bridge.imgmsg_to_cv2(data, "bgr8")	#Image is BGR, not RGB
		
		#Display source image
		#cv2.imshow('source2', self.cv_image)
		
		#Wait 3ms for stability	
		cv2.waitKey(3)
		
		#Begin Image scaling
		scale_percent = 50		#scale %
		width = int(self.cv_image.shape[1]*scale_percent/100)
		height = int(self.cv_image.shape[0]*scale_percent/100)
		dim = (width,height)		#desired scaled dimension
		self.resize = cv2.resize(self.cv_image,dim,interpolation = cv2.INTER_AREA)	#use OpenCV to resize image to desired scaled dimension
		#Display Resized Image
		#cv2.imshow('resize', self.resize)
		
		#Call color position object to retrieve center of ball
		self.color_position() 

		
	def color_position(self):
		#Stating that center is a global variable
		global center
		
		#Upper and lower limits for our red color in HSV space
		lower = np.array([0, 168, 20])
		upper = np.array([179,255,255])
		
		#Converting image to HSV
		hsv_image = cv2.cvtColor(self.resize, cv2.COLOR_BGR2HSV)
		
		#Masking part of image that meets upper and lower limits in HSV
		mask = cv2.inRange(hsv_image, lower, upper)
		
		#Display Masked image (tracked ball)
		cv2.imshow('mask', mask)
		
		#Analyze the countours of the tracked object (if there is something tracked)
		contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		#Only execute if contours are detected
		if len(contours) > 0:
			#con = max(contours, key=cv2.contourArea)
			
			#Determine Coordinate of Center and save them
			M=cv2.moments(contours[0])
			center = (int(M['m10']/M['m00']) , int(M['m01']/M['m00']))
			#print(center)
			#return center		



def track_red_ball():
	#Create Publisher object for velocity
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	#Declaring the node and registering with name
	rospy.init_node('track_red_ball', anonymous=True)
	#Define execution rate object (20Hz)
	rate = rospy.Rate(20)
	
	#Create object of class that begins to filter camera feed and output center coordinates
	ip = image_processor()
	
	while not rospy.is_shutdown():
		
		#Gives the system 3 seconds to properly initialize before starting following
		if time.time() - starttime > 3:
			#Initialize Velocity Objext
			vel_msg = Twist()
			#Print out the Center for monitoring purposes
			print(center)			#center limits 959,539
			
			#Initialize Velocities to 0
			vel_msg.angular.z = 0
			vel_msg.linear.x = 0
			vel_msg.linear.y = 0
			
			#Only edit if ball is detected
			if center[0]!=0 and center[1]!=0:			
				#Based on the center's X position, control angular speed
				if center[0] > 0 and center[0] < 250:
					vel_msg.angular.z = 0.75
				elif center[0] >= 250 and center[0] < 460:
					vel_msg.angular.z = 0.25
				elif center[0] > 500 and center[0] < 750:
					vel_msg.angular.z = -0.25
				elif center[0] >= 750 and center[0] < 960:
					vel_msg.angular.z = -0.75
				else:
					vel_msg.angular.z = 0
				
				#Based on the center's Y position, control linear speed
				if center[1] > 0 and center[1] < 370:
					vel_msg.linear.x = 0.2
				elif center[1] > 400 and center[1] < 540:
					vel_msg.linear.x = -0.2
				else:
					vel_msg.linear.x = 0
			
			#publish speed to motors
			pub.publish(vel_msg)
			
		#Sleep long enough to maintain refresh rate
		rate.sleep()
		
	
if __name__ == '__main__':
	try:	
		#Run Node
		track_red_ball()
	except rospy.ROSInterruptException:
		pass
