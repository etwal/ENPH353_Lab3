#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


## Initialize the ROS node.
rospy.init_node('image_subscriber_and_velocity_publisher')

print("************ Linebot Initialized")

## @brief Callback function for processing incoming images.
## @param image_msg The incoming image message.
def image_callback(image_msg):
    print('*************processing image')
    
    ## Initialize the CvBridge for converting ROS images to OpenCV images.
    bridge = CvBridge()
    try:
        ## Convert the image message to an OpenCV image.
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Error processing image: {str(e)}")

    ## Process the image and identify the road direction.
    road_direction = process_image(cv_image)
    print("*** DIRECTION: " + road_direction)

    ## Create a Twist message to send velocity commands.
    move = Twist()

    move.linear.x = 2  # Move forward

    ## Adjust velocity direction commands based on road_direction.
    if road_direction == 'straight':
        
        move.angular.z = 0.0  # Maintain straight orientation
    elif road_direction == 'left':
        
        move.angular.z = 10  # Turn right to align with the left road
    elif road_direction == 'right':
        
        move.angular.z = -10  # Turn left to align with the right road

    ## Publish the velocity command.
    pub.publish(move)

## Subscribe to the image topic from the robot's camera.
img = rospy.Subscriber('/rrbot/camera1/image_raw', Image, image_callback)
print("********* Subscribed")


## Create a publisher for velocity commands.
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


## @brief Function for processing the input image and determining road direction.
## @param cv_image The input image in OpenCV format.
## @return A string indicating the detected road direction ('straight', 'left', 'right').
def process_image(cv_image):
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper HSV thresholds for the road color
    lower_color = np.array([25, 25, 25], dtype=np.uint8)
    upper_color = np.array([200, 200, 200], dtype=np.uint8)

    # Create a binary mask based on the color thresholds
    mask = cv2.inRange(hsv_frame, lower_color, upper_color)

    # Apply the mask to the original frame
    masked_frame = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    gray_masked_frame = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Gray", gray_masked_frame)
    cv2.waitKey(3)

    contours, _ = cv2.findContours(gray_masked_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = None
    road_center = None

    for contour in contours:
        if largest_contour is None or cv2.contourArea(contour) > cv2.contourArea(largest_contour):
            largest_contour = contour

    if largest_contour is not None:
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            road_center = (cx, cy)

        road_circle_img = cv2.circle(cv_image, road_center, 20, (0, 0, 255), -1)
        cv2.imshow("road center", road_circle_img)
        cv2.waitKey(3)
        # Determine road direction based on the road center
        image_width = cv_image.shape[1]
        if road_center[0] < image_width / 3:
            road_direction = 'left'
        elif road_center[0] > 2 * image_width / 3:
            road_direction = 'right'
        else:
            road_direction = 'straight'

        ## Return the detected road direction.
        return road_direction

rospy.spin()