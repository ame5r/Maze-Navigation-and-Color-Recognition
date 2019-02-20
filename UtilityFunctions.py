#!/usr/bin/env python


from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion
from image_geometry import PinholeCameraModel
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from geometry_msgs.msg import Twist
import math 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
roll = pitch = curr_yaw = axes = 0
yawFirstRads = "start"
from sensor_msgs.msg import LaserScan
forwardMov = True
from sys import exit
RobotCanMove = "noinit"
CurrX = "noinit"
ObjFound = "noinit"
FirstX = "noinit"
CurrY = 0

FirstY = 0

ranges = 0 

	
	
def ObsChecker(msg): 
	global RobotCanMove, ranges	
	ranges = msg.ranges
	
	scan_filter = []
        for i in range(360):
            if i <= 15 or i > 335:
                if msg.ranges[i] >= 0.01:
                    scan_filter.append(msg.ranges[i])

        if len(scan_filter)>0 and min(scan_filter) > 0.19:
            RobotCanMove = True
        else:
             RobotCanMove = False
             

		

def forwardFunc(distance,checkObs):
	global CurrX,CurrY, FirstX, FirstY
	global RobotCanMove, ranges
	backword = 1
	CurrX = FirstX = RobotCanMove = "noinit"
	if checkObs: 
            sub_helper_laser = rospy.Subscriber('/scan', LaserScan, ObsChecker)
        else:
            RobotCanMove = True
	sub_helper_pose = rospy.Subscriber('/odom', Odometry, getPosFunc)


	publisher_handler = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(10)
	move_parametes = Twist()
	

    	current_distance = 0
	if (distance < 0):
            distance = -1 * distance
            backword = -1
	
	while RobotCanMove == "noinit" or FirstX == "noinit" or CurrX == "noinit" :
		pass
	
	
	if RobotCanMove or not checkObs:
            
		while (not rospy.is_shutdown()) and current_distance < distance and (RobotCanMove or not checkObs): # we will move till we pass 0.5 m
			
			move_parametes.linear.x = backword * 0.05  
			move_parametes.angular.z = 0	 # no angle
			publisher_handler.publish(move_parametes)
			r.sleep()			
			# distance between 2 points, by fomula			
			current_distance = math.sqrt(math.pow((FirstX - CurrX),2) + math.pow( (FirstY - CurrY), 2) )
		

		move_parametes.linear.x = 0
		#Force the robot to stop
		publisher_handler.publish(move_parametes)

	else:
		print ("There's an obstacle, robot can't move")

def getRot (msg):
	global roll, pitch, curr_yaw, yawFirstRads 
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	if yawFirstRads == "start":
		roll, pitch, yawFirstRads = euler_from_quaternion (orientation_list)		
	else:
		roll, pitch, curr_yaw = euler_from_quaternion (orientation_list)
    	

	
def rotate(NumOfDegrees):
	global roll, pitch, curr_yaw, yawFirstRads 
	yawFirstRads = curr_yaw = "start"
	sub_handler = rospy.Subscriber('/odom', Odometry, getRot)
	while yawFirstRads == "start" or curr_yaw == "start" :
		pass
        yawFirstDigrees = 180 * yawFirstRads / math.pi

	if (yawFirstDigrees < 0):
		yawFirstDigrees = (yawFirstDigrees+360)

	DistDeg = yawFirstDigrees - NumOfDegrees 
	while DistDeg< 0:
		DistDeg = DistDeg + 360
	while DistDeg > 360:
		DistDeg = DistDeg - 360
	
	if (DistDeg>180):
		DistDeg = DistDeg - 360
	destination_radians = math.pi * DistDeg / 180
	
        
	turn_around_parametes = Twist()  
	publisher_handler = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	r = rospy.Rate(10)

	
	sign = 1

	while not rospy.is_shutdown():			
		turn_around_parametes.angular.z = (0.3 * sign)	
		publisher_handler.publish(turn_around_parametes) 
		r.sleep()
		if abs(destination_radians - curr_yaw) <0.04:
			turn_around_parametes.angular.z = 0 # we stop the robot
			publisher_handler.publish(turn_around_parametes)  	
			break
	
        
 

def distanceCalculation(degrees) :
        
	laserInfo = rospy.wait_for_message('/scan',LaserScan)

        
	distance = np.inf
	if not laserInfo.ranges[degrees] == np.inf :
		return laserInfo.ranges[degrees]
	      
    	return distance
    
                    
def DistToColoredFunc(color,needToTurn) :
        global ObjFound 
        camInfo = rospy.wait_for_message('/usb_cam/camera_info',CameraInfo)
        IMAGE = rospy.wait_for_message('/usb_cam/image_raw',Image)
        camera = PinholeCameraModel()
        camera.fromCameraInfo(camInfo)
	bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(IMAGE,'bgr8')
        
    
        
        gau_blur = cv2.GaussianBlur(cv_image, (3,3), 0)
		######################################################################
        if (color == 'blue'): 
            lower = np.array([135,115,80], dtype="uint8")
            upper = np.array([255,160,120 ], dtype="uint8")
        elif (color == 'red'):
            lower = np.array([0,25,110], dtype="uint8")
            upper = np.array([50,100,255], dtype="uint8")
        elif (color == 'green'):
            lower = np.array([75,120,80], dtype="uint8")
            upper = np.array([110,255,110], dtype="uint8")
        else:
            exit()
        mask_image = cv2.inRange(gau_blur, lower, upper)

        kernelOpen = np.ones((5,5))
        kernelClose = np.ones((20,20))
        maskOpen = cv2.morphologyEx (mask_image ,cv2.MORPH_OPEN,kernelOpen)
        final_mask = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kernelClose)	        
	
        
        
        ##############################################################
        img2, contours, h = cv2.findContours(final_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        contour_area = False
        for i in range (len(contours)) :
            if (cv2.contourArea(contours[i]) > 40000 ):   
                contour_area = True
     

	if(len(contours)!=0 and contour_area):
                ObjFound = True
                cnt = contours[0]
                for i in range (len(contours)) :
                    if cv2.contourArea(contours[i]) > cv2.contourArea(cnt): # we take the biggest one
                        cnt = contours[i]
		
        	M = cv2.moments(cnt)
        	x_center_obj = int(M['m10']/M['m00'])		
        	y_center_obj = 240 
		center_obj = (x_center_obj,y_center_obj)
		image_center = (320, 240) 
		ray_obj = camera.projectPixelTo3dRay(center_obj)
		ray_img = camera.projectPixelTo3dRay(image_center)
		 
		# angle between two vectors - by formula. We'll get the cos of the angle so we'll do arccos
	
		radians = math.acos ( np.dot (ray_obj, ray_img) / ( np.linalg.norm(ray_obj) * np.linalg.norm(ray_img) ) ) 
		degrees = int(math.degrees(radians))
		#if not needToTurn:
                if abs(x_center_obj-image_center[0])>100:
                    degrees = degrees + 5
                    degrees = degrees%360
                elif abs(x_center_obj-image_center[0])>40 :
                    degrees = degrees + 2
                    degrees = degrees%360
                
                if x_center_obj>= image_center[0]:
                    degrees = 360-degrees
                if degrees == 360:
                    degrees = 0
                
               
                if needToTurn:
                    laserInfo = rospy.wait_for_message('/scan',LaserScan)
                    
                    rotate(-degrees) #turn the robot
                    degrees = 0
                    
                    
                    if laserInfo.ranges[0] > 0.55:
                        laserInfo = rospy.wait_for_message('/scan',LaserScan)
                        #rospy.sleep(10)
                
		
		distance = distanceCalculation(degrees)
		if distance == 0:
                    return 0
                return distance
	else:
                ObjFound = False
                return None

def getPosFunc(msg):
	global CurrX,CurrY, FirstX, FirstY
	position = msg.pose.pose.position
	if FirstX == "noinit":
		FirstX = position.x   
		FirstY = position.y 	
	else:
		CurrX = position.x
		CurrY = position.y

def main():
	rospy.init_node('main', anonymous=True)


if __name__ == '__main__':
	main()

  	
