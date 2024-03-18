#!/usr/bin/env python3
# based on Dr. Saeidi's flip_image.py node
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# set up flag
img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")

# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True
	
def detect_ball(image): 

	# convert the image that's passed into the detect_ball function to HSV space
	hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	
	# create rectangle that's the same size as the image we pass
	rectangle = np.zeros((720, 1280, 1), dtype="uint8")
	# use OpenCV function "rectangle" to overlay region of interest
	cv2.rectangle(rectangle, (200,200), (850,500) , 255, -1)
	
	# create the hsv upper and lowers bounds for color filtering
	lower_yellow_hsv = np.array([22,4,10])
	upper_yellow_hsv = np.array([60,255,255])
	# apply the mask
	yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
	
	# apply overlay
	overlay_hsv = cv2.bitwise_and(rectangle, yellow_mask)
	
	# return maskes and overlayed image
	return overlay_hsv

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('detect_ball', anonymous = True)
	# define a subscriber to ream images
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	# define a publisher to publish images
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1)
	
	# set the loop frequency
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			
			# apply the function to color filter the image
			mono_ball = detect_ball(rgb_img)
			
			# convert it to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(mono_ball, encoding="mono8")
			# publish the image
			img_pub.publish(img_msg)
		# pause until the next iteration			
		rate.sleep()

