#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image

ROS_NODE_NAME = "image_processing_node"
def img_process(img):
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    color = (0, 255, 0)                                     # green color in BGR
    detect_largest_blob(frame)

    #cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    #cv2.circle(cv2_img, center_coordinates, radius, color, thickness)
    #cv2.imshow("Frame", cv2_img)
    #cv2.waitKey(1) #this forces the window opened by OpenCV to remain open

def detect_largest_blob(image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        lower_color = np.array([0, 100, 100])
        upper_color = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_color, upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)

            # Draw the bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # Display tihe result
        color = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        cv2.imshow('Detected Blob', color)
        cv2.waitKey(1)

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

