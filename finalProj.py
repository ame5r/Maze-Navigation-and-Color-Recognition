#!/usr/bin/env python


import rospy
import actionlib
from UtilityFunctions import DistToColoredFunc,rotate,rotate,forwardFunc 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from turningAndPushingFunctions import SideCheckerFunc

def startTheMoveBase(xgoal,ygoal):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = xgoal
    goal.target_pose.pose.position.y = ygoal
    goal.target_pose.pose.orientation.w = 1
    
    client.send_goal(goal)
    dist = DistToColoredFunc("blue",False)
    
    
    while dist is None or dist > 0.55:
        dist = DistToColoredFunc("blue",False)
   
    client.cancel_goal()     
    
    dist = DistToColoredFunc("blue",True)  ## getting the exact distance from the object (while centralizing)
    factor=1
    sgn = 1
    while dist==None or dist==0: 
        rotate(2*factor*sgn)
        sign = sgn * -1
        factor=2*factor
        dist = DistToColoredFunc("blue",True)
    
    rospy.sleep(1.)
  
    
    
    forwardFunc(dist-0.22,True) # getting closer to the obj so we are 0.3 at most
    rospy.sleep(1.)
    SideCheckerFunc(0)
    
   
    # After moving the obstacle we resume the navigation to the end
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        
        # The Goal Point
        xgoal = 6.58269893045
        ygoal = 6.42210343874
        
        rospy.init_node('startTheMoveBase_py')
        result = startTheMoveBase(xgoal,ygoal)
        
        
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
