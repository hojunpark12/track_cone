#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def callback(msg):
   
    mtx = np.array([[637.672100, 0.000000, 309.566959],
    [  0.000000, 640.079980, 271.465664 ],
    [  0.        ,   0.        ,   1.        ]])
    dist = np.array([[-0.362970, 0.114217, 0.000856, -0.000690, 0.000000 ]])
       
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img = cv_image
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(640,360),0,(640,360))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi

    image_message = bridge.cv2_to_imgmsg(dst, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()

    print(newcameramtx)
    print(roi)
    cam_pub1.publish(image_message)


def callback2(msg):
   
    ret = np.array(1.1727846969115212)
    mtx = np.array([[656.463286, 0.000000, 319.359193],
    [  0.000000, 657.446167, 221.879265],
    [  0.        ,   0.        ,   1.        ]])
    dist = np.array([[-0.391683, 0.139876, -0.006383, 0.008809, 0.000000]])
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    img = cv_image
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(640,360),0,(640,360))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi

    image_message = bridge.cv2_to_imgmsg(dst, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()

    print(newcameramtx)
    print(roi)
    cam_pub2.publish(image_message)
    
if __name__ == '__main__':

    rospy.init_node('usb_cam_cal2')
    
    rospy.Subscriber("/camera_right/camera1/usb_cam/image_raw",Image,callback)
    rospy.Subscriber("/camera_left/camera2/usb_cam/image_raw",Image,callback2)

    cam_pub2 = rospy.Publisher("/usb_cam/image2",Image,queue_size =1)
    cam_pub1 = rospy.Publisher("/usb_cam/image1",Image,queue_size =1)

    rospy.spin()