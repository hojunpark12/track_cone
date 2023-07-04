#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import glob

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(msg):
    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #print(1)
    img = cv_image
    # cv2.imshow("distort", img)
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    x,y,w,h = roi
    #print(roi)
    #dst = dst[y:y+h, x:x+w]
    
    # cv2.imshow('undistort',dst)


    image_message = bridge.cv2_to_imgmsg(dst, encoding="rgb8")
    image_message.header.stamp = rospy.Time.now()

    cam_pub.publish(image_message)




if __name__ == '__main__':

    rospy.init_node('usb_cam_cal1')
    cam_pub = rospy.Publisher("/usb_cam/image1",Image,queue_size =1)
    rospy.Subscriber("/camera1/usb_cam/image_raw",Image,callback)
    # rospy.Subscriber("/usb_cam/camera_info",callback2)

    ret = np.array(1.1727846969115212)
    mtx = np.array([[627.487510, 0.000000, 322.106978],
    [   0.,         647.18212891, 242.52794634],
    [  0.        ,   0.        ,   1.        ]])
    dist = np.array([[0.006631, -0.049452, -0.003513, -0.002012, 0.000000]])
    #dist = np.array([[-0.1,  0, 0,  0,  0]])
    #newcameramtx= np.array([[307.00064087,   0.        , 329.04994561],
    #[  0.        , 397.51873779, 197.66134326],
    #[  0.        ,   0.        ,   1.        ]])) 
#newcameramtx = np.array([[866.45654297,0,332.08775071],[0,940.07519531,85.50009563],[0,0,1]])
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(640,360),0,(640,360))
    print(newcameramtx)
    print(roi)
    rospy.spin()
