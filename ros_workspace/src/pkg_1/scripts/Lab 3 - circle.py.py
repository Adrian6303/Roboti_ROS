#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
ROS_NODE_NAME = "image_processing_node"
def img_process(img):
    rospy.loginfo("image width: %s height: %s" % (img.width, img.height))
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    center_coordinates = (img.width // 2, img.height // 2)  # center of the image
    radius = min(img.width, img.height) // 10               # radius as 1/10th of the smaller dimension
    color = (0, 255, 0)                                     # green color in BGR
    thickness = 2                                           # thickness of the circle outline
    cv2.circle(frame, center_coordinates, radius, color, thickness)
    cv2.imshow("Frame", frame)
    cv2.waitKey(1) #this forces the window opened by OpenCV to remain open
def cleanup():
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")
if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
try:
    rospy.spin()
except KeyboardInterrupt:
    pass