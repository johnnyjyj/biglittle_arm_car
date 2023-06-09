#!/usr/bin/env python

from __future__ import division
import rospy
import message_filters
import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Vector3
from wheeltec_arm_pick.msg import four_arm_position as PositionMsg
from std_msgs.msg import String as StringMsg
from dynamic_reconfigure.server import Server
from wheeltec_arm_pick.cfg import Params_colorConfig
np.seterr(all='raise')  
displayImage=False
global count 
trackstart=0
m=0
def nothing(s):
    pass

Switch = '0:Yellow\n1:Blue\n2:Green\n3.user-defined'

plt.close('all')
class visualTracker:
	def __init__(self):
		self.bridge = CvBridge() 
		self.targetUpper = np.array(rospy.get_param('~target/upper'))
		self.targetLower = np.array(rospy.get_param('~target/lower'))
		# self.lower_red = np.array(rospy.get_param('~target/red_lower'))
		# self.upper_red = np.array(rospy.get_param('~target/red_upper'))
		self.lower_green = np.array(rospy.get_param('~target/green_lower'))
		self.upper_green = np.array(rospy.get_param('~target/green_upper'))
		self.lower_blue = np.array(rospy.get_param('~target/blue_lower'))
		self.upper_blue = np.array(rospy.get_param('~target/blue_upper'))
		self.lower_yellow = np.array(rospy.get_param('~target/yellow_lower'))
		self.upper_yellow = np.array(rospy.get_param('~target/yellow_upper'))
		# self.lower_black = np.array(rospy.get_param('~target/black_lower'))
		# self.upper_black = np.array(rospy.get_param('~target/black_upper'))

		self.pictureHeight= rospy.get_param('~pictureDimensions/pictureHeight')
		self.pictureWidth = rospy.get_param('~pictureDimensions/pictureWidth')
		vertAngle =rospy.get_param('~pictureDimensions/verticalAngle')
		horizontalAngle =  rospy.get_param('~pictureDimensions/horizontalAngle')
		# precompute tangens since thats all we need anyways:
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)	
		self.lastPoCsition =None
		# one callback that deals with depth and rgb at the same time
		im_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
		dep_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
		self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
		self.timeSynchronizer.registerCallback(self.trackObject)
		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=3)
		self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
		#self.infoPublisher = rospy.Publisher('/object_size=3)
		self.color_obj = Server(Params_colorConfig,self.colorreconfigure)
		rospy.logwarn(self.targetUpper)     
	def trackObject(self, image_data, depth_data):
		global m
		global mask1
		global trackstart
		trackstart=1
		if(image_data.encoding != 'rgb8'):
			raise ValueError('image is not rgb8 as expected')
		#convert both images to numpy arrays
		frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
		depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#"32FC1")	
		if(np.shape(frame)[0:2] != (self.pictureHeight, self.pictureWidth)):
			raise ValueError('image does not have the right shape. shape(frame): {}, shape parameters:{}'.format(np.shape(frame)[0:2], (self.pictureHeight, self.pictureWidth)))
		# blure a little and convert to HSV color space
		#blurred = cv2.GaussianBlur(frame, (11,11), 0)
		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)	
		# select all the pixels that are in the range specified by the target
	        if m == 0:
	        	org_mask = cv2.inRange(hsv, self.upper_yellow, self.lower_yellow)
	        elif m == 1:
	        	org_mask = cv2.inRange(hsv, self.upper_blue, self.lower_blue)
	        elif m == 2:
				org_mask = cv2.inRange(hsv, self.upper_green, self.lower_green)
	        # elif m == 3:
	        #    org_mask = cv2.inRange(hsv, self.upper_black, self.lower_black)
	        elif m == 3:
	 			org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)	
	        else:
				org_mask = cv2.inRange(hsv, self.upper_blue, self.lower_blue)
		#clean that up a little, the iterations are pretty much arbitrary
		mask = cv2.erode(org_mask, None, iterations=5)
		mask1 = cv2.dilate(mask,None, iterations=3)
		# find contours of the object
		contours = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		newPos = None #if no contour at all was found the last position will again be set to none
		# lets you display the image for debuging. Not in realtime though

		# if displayImage:
		# 	backConverted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
		# 	#cv2.imshow('frame', backConverted)
		# 	#cv2.waitKey(0)
		# 	#print(backConverted)			
		# 	plt.figure()
		# 	plt.subplot(2,2,1)
		# 	plt.imshow(frame)
		# 	plt.xticks([]),plt.yticks([])
		# 	plt.subplot(2,2,2)
		# 	plt.imshow(org_mask, cmap='gray', interpolation = 'bicubic')			
		# 	plt.xticks([]),plt.yticks([])
		# 	plt.subplot(2,2,3)			
		# 	plt.imshow(mask, cmap='gray', interpolation = 'bicubic')
		# 	plt.xticks([]),plt.yticks([])
		# 	plt.show()
		# 	rospy.sleep(0.2)
			
		# go threw all the contours. starting with the bigest one
		for contour in sorted(contours, key=cv2.contourArea, reverse=True):
			area = cv2.contourArea(contour)
			if area<200:
				return 

			# get position of object for this contour
		 	pos = self.analyseContour(contour, depthFrame)
		
			# if it's the first one we found it will be the fall back for the next scan if we don't find a plausible one
			if newPos is None:
				newPos = pos
			# check if the position is plausible
			#if self.checkPosPlausible(pos):
			self.lastPosition = pos
			self.publishPosition(pos)
			count=0
			return
		
		self.lastPosition = newPos #we didn't find a plossible last position, so we just save the biggest contour
		#count=count+1
		#if count > 20:
		#    velocity = Twist()
		#    velocity.linear = Vector3(0,0,0)
		#    velocity.angular = Vector3(0,0,0)
		#    self.cmdVelPublisher.publish(velocity)
		# and publish warnings
		#rospy.logwarn('no position found')
		#self.infoPublisher.publish(StringMsg('visual:nothing found'))

	def publishPosition(self, pos):
		# calculate the angles from the raw position
		angleX = self.calculateAngleX(pos)
		angleY = self.calculateAngleY(pos)
		# publish the position (angleX, angleY, distance)
		posMsg = PositionMsg(angleX, angleY, pos[1])
		self.positionPublisher.publish(posMsg)

	def checkPosPlausible(self, pos):
		'''Checks if a position is plausible. i.e. close enough to the last one.'''

		# for the first scan we cant tell
		if self.lastPosition is None:
			return False

		# unpack positions
		((centerX, centerY), dist)=pos	
		((PcenterX, PcenterY), Pdist)=self.lastPosition
		
		if np.isnan(dist):
			return False

		# distance changed to much
		if abs(dist-Pdist)>0.5:
			return False

		# location changed to much (5 is arbitrary)
		if abs(centerX-PcenterX)>(self.pictureWidth /5):
			return False

		if abs(centerY-PcenterY)>(self.pictureHeight/5):
			return False
		
		return True
			
		
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''

		centerX = pos[0][0]
		displacement = 2*centerX/self.pictureWidth-1
		angle = -1*np.arctan(displacement*self.tanHorizontal)
		return angle

	def calculateAngleY(self, pos):
		centerY = pos[0][1]
		displacement = 2*centerY/self.pictureHeight-1
		angle = -1*np.arctan(displacement*self.tanVertical)
		return angle
	
	def analyseContour(self, contour, depthFrame):
		'''Calculates the centers coordinates and distance for a given contour

		Args:
			contour (opencv contour): contour of the object
			depthFrame (numpy array): the depth image
		
		Returns:
			centerX, centerY (doubles): center coordinates
			averageDistance : distance of the object
		'''
		# get a rectangle that completely contains the object
		centerRaw, size, rotation = cv2.minAreaRect(contour)

		# get the center of that rounded to ints (so we can index the image)
		center = np.round(centerRaw).astype(int)

		# find out how far we can go in x/y direction without leaving the object (min of the extension of the bounding rectangle/2 (here 3 for safety)) 
		minSize = int(min(size)/3)

		# get all the depth points within this area (that is within the object)
		depthObject = depthFrame[(center[1]-minSize):(center[1]+minSize), (center[0]-minSize):(center[0]+minSize)]

		# get the average of all valid points (average to have a more reliable distance measure)
		depthArray = depthObject[~np.isnan(depthObject)]
		averageDistance = np.mean(depthArray)
		if(averageDistance>400 or averageDistance<3000):
			pass
		else:
			averageDistance=400

		if len(depthArray) == 0:
			rospy.logwarn('empty depth array. all depth values are nan')

		return (centerRaw, averageDistance)
	
	def colorreconfigure(self, config, level):
		HSV_H_MIN =config.HSV_H_MIN
		HSV_S_MIN =config.HSV_S_MIN
		HSV_V_MIN =config.HSV_V_MIN
		HSV_H_MAX =config.HSV_H_MAX
		HSV_S_MAX =config.HSV_S_MAX
		HSV_V_MAX =config.HSV_V_MAX
		self.targetUpper=np.array([HSV_H_MIN,HSV_S_MIN,HSV_V_MIN])
		self.targetLower=np.array([HSV_H_MAX,HSV_S_MAX,HSV_V_MAX])
		return  config


if __name__ == '__main__':
	rospy.init_node('visual_tracker')
	tracker=visualTracker()
	rospy.logwarn('visualTracker init done')
	temp=0
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
		if temp==0:
			cv2.createTrackbar(Switch,'Adjust_hsv',0,3,nothing)
			temp=1
		m=cv2.getTrackbarPos(Switch,'Adjust_hsv')
		if trackstart==1:
			cv2.imshow("Adjust_hsv", mask1)
			cv2.waitKey(3)
		rate.sleep()


