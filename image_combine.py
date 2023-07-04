import numpy as np
import cv2
import glob
import rospy
import message_filters

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Define the dimensions of the camera window
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Initialize the camera window as a black image
camera_window = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

def callback_right(right):
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

def callback_left(left):
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



if __name__ == '__main__':
    
    rospy.init_node('image_combiner')
    cam_pub = rospy.Publisher("/combined_image",Image,queue_size =1)
    rospy.Subscriber("/usb_cam/image1",Image,callback_right)
    rospy.Subscriber("/usb_cam/image2",Image,callback_left)
    rospy.spin()
