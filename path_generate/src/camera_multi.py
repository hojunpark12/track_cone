#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Initialize the camera window as a black image
camera_window = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

def callback_right_image(right):
    global camera_window
    bridge = CvBridge()
    cv_image_right = bridge.imgmsg_to_cv2(right, desired_encoding='rgb8')
    
    # Crop the image to match the height of the camera window
    cv_image_right = cv_image_right[:CAMERA_HEIGHT, :]
    
    # Check if the image height matches the camera height
    if cv_image_right.shape[0] == CAMERA_HEIGHT:
        # Combine the right image with the camera window
        camera_window[:, CAMERA_WIDTH//2:, :] = cv_image_right
        
    # If the image height does not match the camera height, fill the missing area with white color
    else:
        # Crop the image to match the width of the camera window
        cv_image_right = cv_image_right[:, :CAMERA_WIDTH//2, :]
        # Create a new image with white color for the missing area
        white_image = np.ones((CAMERA_HEIGHT - cv_image_right.shape[0], cv_image_right.shape[1], 3), dtype=np.uint8) * 255
        # Combine the right image and the white image with the camera window
        camera_window[:, CAMERA_WIDTH//2:, :] = np.vstack((cv_image_right, white_image))
    
    # Publish the combined image
    image_message = bridge.cv2_to_imgmsg(camera_window, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()
    cam_pub.publish(image_message)

def callback_left_image(left):
    global camera_window
    bridge = CvBridge()
    cv_image_left = bridge.imgmsg_to_cv2(left, desired_encoding='rgb8')
    
    # Crop the image to match the height of the camera window
    cv_image_left = cv_image_left[:CAMERA_HEIGHT, :]
    
    # Check if the image height matches the camera height
    if cv_image_left.shape[0] == CAMERA_HEIGHT:
        # Combine the left image with the camera window
        camera_window[:, :CAMERA_WIDTH//2, :] = cv_image_left
        
    # If the image height does not match the camera height, fill the missing area with white color
    else:
        # Crop the image to match the width of the camera window
        cv_image_left = cv_image_left[:, :CAMERA_WIDTH//2, :]
        # Create a new image with white color for the missing area
        white_image = np.ones((CAMERA_HEIGHT - cv_image_left.shape[0], cv_image_left.shape[1], 3), dtype=np.uint8) * 255
        # Combine the left image and the white image with the camera window
        camera_window[:, :CAMERA_WIDTH//2, :] = np.vstack((cv_image_left, white_image))
    
    # Publish the combined image
    image_message = bridge.cv2_to_imgmsg(camera_window, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()
    cam_pub.publish(image_message)

def callback_right(msg):

    mtx = np.array([[473.076838646638,	0,	327.496302291813 ], # 758.264986 0.000000 411.309665      637.672100, 0.000000, 309.566959
    [ 0,	475.928903243506,	153.712466224663   ], #  0.000000, 640.079980, 271.465664
    [  0.        ,   0.        ,   1.        ]])
    dist = np.array([[-0.358929840004944,	0.116610411060671,0,0,0 ]])  #    -0.362970, 0.114217, 0.000856, -0.000690, 0.000000

       
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img = cv_image
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(640,360),0,(640,360))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi

    image_message = bridge.cv2_to_imgmsg(dst, encoding="rgb8")
    
    image_message.header.stamp = rospy.Time.now()
    callback_right_image(image_message)
    print("right blue" ,newcameramtx)
    #print(roi)
    cam_pub1.publish(image_message)


def callback_left(msg):

    ret = np.array(1.1727846969115212)
    mtx = np.array([[476.158198067448,	0,	317.354087253852 ], # 656.463286, 0.000000, 319.359193
    [ 0.,	481.276240660148,	191.739300729386 ], #   0.000000, 657.446167, 221.879265
    [  0.        ,   0.        ,   1.        ]])
    dist = np.array([[-0.351038115699235, 0.0988182582447672,	0,	0,	0]]) #   -0.391683, 0.139876, -0.006383, 0.008809, 0.000000

    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img = cv_image
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(640,360),0,(640,360))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi

    image_message = bridge.cv2_to_imgmsg(dst, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()
    callback_left_image(image_message)
    print("left yellow ", newcameramtx)
    #print(roi)
    cam_pub2.publish(image_message)
    
if __name__ == '__main__':

    rospy.init_node('usb_cam_cal2')
    
    rospy.Subscriber("/camera_right/camera2/usb_cam/image_raw",Image,callback_right)   #/camera1/usb_cam/image_raw  /camera_right/camera1/usb_cam/image_raw
    rospy.Subscriber("/camera_left/camera1/usb_cam/image_raw",Image,callback_left)
    #rospy.Subscriber("/usb_cam/image1",Image,callback_right_image)
    #rospy.Subscriber("/usb_cam/image2",Image,callback_left_image)
    cam_pub2 = rospy.Publisher("/left_image",Image,queue_size =1)
    cam_pub1 = rospy.Publisher("/right_image",Image,queue_size =1)
    cam_pub = rospy.Publisher("/combined_image",Image,queue_size =1)
    rospy.spin()