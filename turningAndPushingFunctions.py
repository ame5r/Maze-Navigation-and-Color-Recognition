import rospy
import actionlib
from UtilityFunctions import DistToColoredFunc,rotate,rotate,forwardFunc 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan

def moveToOneSide(cntr,left):
    forwardFunc(0.4,True)
    rospy.sleep(1.)
    rotate(left*(-150+cntr))
    forwardFunc(0.45,False)
    rospy.sleep(1.)
    forwardFunc(-0.20,False)
    rotate(left*90)



def RightCheckerFunc(counter):
    rotate(70-counter)
    rospy.sleep(1.)
    laserInfo = rospy.wait_for_message('/scan',LaserScan)
    theRightSide = True
    
    for i in range(17):
        if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
            theRightSide  = False 
    for i in range(343, 359):
        if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
            theRightSide  = False  
    return theRightSide

def LeftCheckerFunc(counter):
        rotate(-120+2*counter) 
        rospy.sleep(1.)
        laserInfo = rospy.wait_for_message('/scan',LaserScan)
        theLeftSide = True
        for i in range(17):
            if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
                left_side  = False 
        for i in range(343, 359):
            if laserInfo.ranges[i] < 0.3 and not laserInfo.ranges[i]==0:
                theLeftSide  = False  
        return theLeftSide





def SideCheckerFunc(counter):  

    theRightSide = RightCheckerFunc(counter)
    if theRightSide:
         print("right is ok")
         moveToOneSide(counter,1) # 1 means move right 
         
    else:
            theLeftSide = LeftCheckerFunc(counter)
            print("left is ok")
            moveToOneSide(counter,-1)# -1 means move left
		

