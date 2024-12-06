#!/usr/bin/env python3 
import rospy
import cv2
import math
import numpy as np
import time
from threading import Thread, Lock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from puppy_control.msg import Pose, Gait, Velocity
from puppy_control.srv import SetRunActionName

ROS_NODE_NAME = "move_on_detect_node"

lock = Lock()
move_th = None

max_contour = None
contour_center = None
radius = 0

surface = 0

pose_pub = None
gait_pub = None
vel_pub = None

shut = False
pose_msg = Pose(roll=math.radians(0), pitch=math.radians(0), yaw=0, height=-10, x_shift=0, stance_x=0, stance_y=0, run_time=500)
gait_msg = Gait(overlap_time=0.3, swing_time=0.5, clearance_time=0.0, z_clearance=3)
vel_msg = Velocity(x=0, y=0, yaw_rate=math.radians(0))

def detect_largest_blob(image):
        global largest_contour
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
            if w<50 or h<50:
                x,y,w,h,largest_contour=0,0,0,0,None
        else:
            x,y,w,h,largest_contour=0,0,0,0,None
        # Display tihe result
        color = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        #cv2.imshow('Detected Blob', color)
        #cv2.waitKey(1)
        return largest_contour,x,y,w,h 

def img_process(img):
    global pose_pub, pose_msg
    global contour_center, radius, max_contour
    global lock, shut, surface
    frame = np.ndarray(shape=(img.height, img.width, 3), dtype=np.uint8, buffer=img.data)
    cv2_img = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # find all contours
    with lock:
        if not shut:
            #max_contour = get max contour and store in this variable
            max_contour, x, y,w,h = detect_largest_blob(cv2_img)
            if max_contour is not None:
               # print(max_contour)
               contour_center = (x+w //2,y+h //2)
            surface = x * y
                # calculate the center of the contour and estimate the size of the contour               
                # draw the bounding circle or box around the contour
        cv2.imshow("Frame", cv2_img)
        cv2.waitKey(1)

def cleanup():
    global shut, lock
    with lock:
        shut = True
    rospy.loginfo("Shutting down...")
    cv2.destroyWindow("Frame")
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()

def move():
    global pose_pub, vel_pub
    global contour_center, radius
    global max_contour
    global lock, shut
    while True:
        time.sleep(0.2)
        with lock:
            if shut:
                break
            if max_contour is not None:
                # if there is a contour, decide how to move and change pitch
                vel_msg.x = 10
                vel_pub.publish(vel_msg)

            elif surface < 10:
                vel_msg.x = 0
                vel_pub.publish(vel_msg)
            elif surface > 100:
                vel_msg.x = 0
                vel_pub.publish(vel_msg)
                # if no contour is detected, do something else

if __name__ == "__main__":
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)
    
    pose_pub = rospy.Publisher("/puppy_control/pose", Pose, queue_size=1)
    gait_pub = rospy.Publisher("/puppy_control/gait", Gait, queue_size=1)
    vel_pub = rospy.Publisher("/puppy_control/velocity", Velocity, queue_size=1)
    
    gait_pub.publish(gait_msg)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, img_process)
    rospy.ServiceProxy("/puppy_control/go_home", Empty)()
    # create a daemon that will run the "move" function in the background
    # the move function should contain all the logic for moving the robot towards the detected object and for tracking it
    move_th = Thread(target=move, daemon=True)
    move_th.start()
    rospy.spin()
