# #! /usr/bin/env python

import pyautogui
import cv2
import numpy as np
import time

import json
import sys,re
import socket

import rospy
import tf
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped

import math,time


ip_port=('127.0.0.1',8082)
BUFSIZE=1024
s=socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
s.connect_ex(ip_port)


class UAV:

    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('/firefly/ground_truth/position',
                                                PointStamped, self.update_pose)

        self.x = 0
        self.y = 0
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.x = data.point.x
        self.y = data.point.y

def Show_Image(AnImage):
    cv2.imshow("View", AnImage)
    cv2.waitKey(10) 

def get_image():
    img = pyautogui.screenshot(region=[1404, 162, 480, 300])
    img = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
    return img

# Image Process
def Calcuate_Center(CV_Image):
    hsv = cv2.cvtColor(CV_Image, cv2.COLOR_BGR2HSV)
    lower= np.array([ 0, 0, 0])
    upper = np.array([180, 255, 46])
    mask = cv2.inRange(hsv, lower, upper)
    # Show_Image(mask)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] 
    if len(cnts) > 0: 
        c = max(cnts, key = cv2.contourArea) 
        ((x, y), radius) = cv2.minEnclosingCircle(c) 
        # center=(int(x),int(y))
        center=[int(x),int(y)]
        cv2.circle(CV_Image, (int(x),int(y)), int(radius), (0,0,255), 2)
        Show_Image(CV_Image)
        # print CV_Image.shape
        # print center

        return center

def Send_Center(center):
    send_msg=center
    json_send_data=json.dumps(send_msg)
    s.send(json_send_data.encode('utf-8'))
    
    
    

def Rev_Control():
    json_rev_msg = s.recv(BUFSIZE)
    global rev_msg
    p1=re.compile(r'\[(.*?)\]',re.S)
    rev_msg=re.findall(p1,json_rev_msg)[-1].split(',')
    return rev_msg
    # print rev_msg

# def pose_callback(point):
#     # rospy.loginfo('I heard %f',point.point.x)
#     # rospy.loginfo(" >> Published waypoint: x: {}, y: {}".format(point.point.x,point.point.y))
#     x_des=point.point.x+2
#     y_des=point.point.y+2
# 	# x_des=point.point.x + float(rev_msg[0])*math.sin(270.0-float(rev_msg[1]))
# 	# y_des=point.point.y + float(rev_msg[0])*math.cos(270.0-float(rev_msg[1]))
#     z_des=float(2)
#     yaw_des=float(180.0+float(rev_msg[1]))
#     print yaw_des
#     # publish_waypoint(point.point.x,point.point.y,z_des,yaw_des)
#     publish_waypoint(point.point.x,point.point.y,z_des,yaw_des)
# 	# publish_waypoint(x_des,y_des,z_des,yaw_des)
# 	# rospy.loginfo(" >> Published to UAV: x: {}, y: {}, z: {}, yaw: {}".format(x_des,y_des,z_des,yaw_des))

def publish_waypoint(x,y,z,yaw):
	"""
	Publish a waypoint to 
	"""
	command_publisher = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size = 1)
	print "000000000"
	# create trajectory msg
	traj = MultiDOFJointTrajectory()
	traj.header.stamp = rospy.Time.now()
	traj.header.frame_id = 'frame'
	traj.joint_names.append('base_link')


	# create start point for trajectory
	transforms = Transform()
	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(1))
	traj.points.append(point)

	# create end point for trajectory
	# transforms = Transform()
	transforms.translation.x = x
	transforms.translation.y = y
	transforms.translation.z = z 

	quat = tf.transformations.quaternion_from_euler(0, 0, yaw*np.pi/180.0, axes = 'rzyx')
    # quat = tf.transformations.quaternion_from_euler(yaw*np.pi/180.0, 0, 0, axes = 'rzyx')

	transforms.rotation.x = quat[0]
	# transforms.rotation.x = 0
	transforms.rotation.y = quat[3]
	transforms.rotation.z = quat[2]
	transforms.rotation.w = quat[1]

	# print("quat[0]:",transforms.rotation.x)
	# print("quat[1]:",transforms.rotation.y)
	# print("quat[2]:",transforms.rotation.z)
	# print("quat[3]:",transforms.rotation.w)

	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
	traj.points.append(point)
	# rospy.sleep(1)
	command_publisher.publish(traj)    



def main():
    rospy.init_node('uav_image', anonymous=True)
    U = UAV()
    yaw_des=180
    temp_rev=0
    while True:
        image = get_image()
        # print "size:",image.shape
        # time.sleep(1)
        # Show_Image(image)
        center=Calcuate_Center(image)
        print center
        Send_Center(center)
        rev_msg = Rev_Control()     
        # yaw_des = float(180.0+float(rev_msg[1]))
        if rev_msg==temp_rev:
            yaw_des=temp_yaw
        else:
            yaw_des = float(yaw_des+float(rev_msg[1]))
        print yaw_des
        publish_waypoint(U.x,U.y,2,yaw_des)
        print 333
        temp_rev=rev_msg
        temp_yaw=yaw_des
        time.sleep(2.5)
        print "v1:",rev_msg[0]
        if abs(float(rev_msg[0])) < 0.05:
            print "Don't move"
            publish_waypoint(U.x,U.y,2,yaw_des)
            
        else:
            x_des=U.x-float(rev_msg[0])*math.cos(yaw_des)
            y_des=U.y+float(rev_msg[0])*math.sin(yaw_des)
            publish_waypoint(x_des,y_des,2,yaw_des)
            print "x_des:",x_des
            print "y_des:",y_des

    # s.close()
    rospy.spin()
        
        

if __name__ == '__main__':
    main()
    
   








































# import socket
# import rospy
# import sys,re
# import tf
# import numpy as np
# import json

# from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
# from geometry_msgs.msg import Twist 
# from geometry_msgs.msg import Transform
# from geometry_msgs.msg import PointStamped

# import math,time

# import cv2
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# bridge = CvBridge()

# ip_port=('127.0.0.1',8081)
# BUFSIZE=1024
# s=socket.socket(socket.AF_INET,socket.SOCK_STREAM) 
# s.connect_ex(ip_port)



# def Show_Image(AnImage):
#     cv2.imshow("View", AnImage)
#     cv2.waitKey(10) 

# # class UAV:

# #     def __init__(self):
# #         # Creates a node with name 'turtlebot_controller' and make sure it is a
# #         # unique node (using anonymous=True).

# #         self.pose_subscriber = rospy.Subscriber('/firefly/vi_sensor/left/image_raw',
# #                                                 Image, self.update_image)

# #         self.image = Image()
# #         self.rate = rospy.Rate(5)

# #     def update_image(self, data):
# #         # print type(data)
# #         self.image.data = data
# #         # cv_image = bridge.imgmsg_to_cv2(self.image.data, 'bgr8')
# #         # Show_Image(cv_image)

# #     def process(self):
# #         while True:
# #             cv_image = bridge.imgmsg_to_cv2(self.image.data, 'bgr8')
# #             Show_Image(cv_image)

# # Image Process
# def Calcuate_Center(CV_Image):
#     hsv = cv2.cvtColor(CV_Image, cv2.COLOR_BGR2HSV)
#     lower= np.array([ 0, 0, 0])
#     upper = np.array([180, 255, 46])
#     mask = cv2.inRange(hsv, lower, upper)
#     # Show_Image(mask)
#     cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] 
#     if len(cnts) > 0: 
#         c = max(cnts, key = cv2.contourArea) 
#         ((x, y), radius) = cv2.minEnclosingCircle(c) 
#         # center=(int(x),int(y))
#         center=[int(x),int(y)]
#         cv2.circle(CV_Image, (int(x),int(y)), int(radius), (0,0,255), 2)
#         Show_Image(CV_Image)
#         # print CV_Image.shape
#         # print center

#         return center

#         #  Send Center
#         # send_center(int(center[0]), int(center[1]))

       
# def callback(data):
#     try:
#         global bridge
#         # print data
#         cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
#         Center=Calcuate_Center(cv_image)
#         send_msg=Center
#         print Center
#         # json_send_data=json.dumps(send_msg)
#         # s.send(json_send_data.encode('utf-8'))
#         # json_rev_msg = s.recv(BUFSIZE)
#         # global rev_msg
#         # p1=re.compile(r'\[(.*?)\]',re.S)
#         # rev_msg=re.findall(p1,json_rev_msg)[-1].split(',')
#         # print rev_msg

#     except CvBridgeError as e:
#         print(e)

# def subscriber():
#     rospy.Subscriber('/firefly/vi_sensor/left/image_raw', Image, callback,queue_size=10)



# def main():
#     rospy.init_node('turtlebot_controller', anonymous=True)
#     while True:
#         subscriber()

# if __name__ == '__main__':
#     main()