
Course: Topics In Robotics 191 (https://www.cs.bgu.ac.il/~toari191/Main)
        Instructors: Prof. Ronen Brafman and Mr. Shai Givati


Project: Maze Navigation and Color Recognition
         Students: Ameer abu Ganeem and Dor Tabakuli


Robot Model: TurtleBot3 Burger
[Robot](AdditionalFiles/robot.png)

Map:
[Map](AdditionalFiles/map.png)  
Goal:  
the goal of this mission is to navigate the turtlebot from the start of the maze to the end, while THERE IS A COLORED OBSTACLE IN FRONT OF THE EXIT, which the robot need to push to get out.  

- Our Algorithm (Overview):  
1- Navigate to the exit point.  
2- When recognize the colored object:  
    2.1- Cancel the navigation and stop  
    2.2- Rotate until you are in front of the center of the object (approximatly).  
    2.3- Move forward to get Closer to object.  
    2.4- Check if the right side is wide enough to push the object from. if it is:  
        2.4.1- Move to the right side.  
        2.4.2- Rotate back to the object.  
        2.4.3- Start pushing the object.  
        2.4.4- Go backward.  
        2.4.5- Renavigate to the exit (cancelled in step 2.1).  
        2.4.6- Finish.  
    2.5-Else,Check if the left side is wide enough to push the object from:  
        2.5.1- Move to the left side.  
        2.5.2- Rotate back to the object.  
        2.5.3- Start pushing the object.  
        2.5.4- Go backward.  
        2.5.5- Renavigate to the exit (cancelled in step 2.1)  
        2.5.6- Finish.  


- Algorithm Explanation :  
Step 1: Here, we use the "move_base" planner, which we give an (x,y) coordinations for the exit, and the robot start navigating to there while avoiding  
        the obstacles in its way.(more about "ROS move_base" package, in the references).  
  
Step 2: The very first moment when the robot recognize the color and the distance is at most 0.55m.  
  
Step 2.1: Cancel the planner's navigation, at this exact moment, the robot will stop immidiatly.  
  
Step 2.2: Here we need to centralize the robot, so we can get better distances and angles measurements.The robot measures the distance from the colored object,when it stopped  
          and check the diffirence between it and the center of the current image (which is 320 in this case), and if the difference is bigger than a specific number, the robot  
          rotates for a specific angle (e.g. if the difference is bigger than 100, rotate 5 degrees).  
  
Step 2.3: Getting closer by 0.22m for enhancing measurments (so the robot now is at distance ~0.33m).  
 
Step 2.4 \ 2.5: Rotate by 60 degrees to the right\left, and check (using the laser ranges) the sides of the robot, that means, check if the laser beams in the degrees [0,17] (right side of                  the robot),and degrees [343,359] (left side of the robot) are indicating for an object with distance, at least, 0.3m so the robot can move there and rotating without                        colliding.  
  
Step 2.4.1 \ 2.5.1: Move the right\left side for 0.4m.  
  
Step 2.4.2 \2.5.2: Rotate toward the colored object to face it, by 60 degrees.  
  
Step 2.4.3 \ 2.5.3: Move forward toward the colored object, so the robot is pushing the object now, by 0.5m.  
  
Step 2.4.4 \ 2.5.4: Go backward for another 0.4m, to be ready for exiting.  
  
Step 2.4.5 \ 2.5.5: Renavigate to the exit, by resending the exit coordination to the "move_base" planner, so it can continue navigating from the point where the robot stopped  
                    while the obstacle (the colored object) has been removed from in front of the exit.  
  
Step 2.4.6 \ 2.5.6: The execution will considered done when the planner returns "SUCCESS", that indicates that the goal has been reached and the robot arrived to the distination.  
  
  
Note: We used some constants in rotations and movements, because the design of the maze was already, approximatly, known. You are free to use the tools installed on your robot (e.g. camera,        laser, thearmal camera, etc...) to develop a global solution.  
  
- Problems that may affect the performance of this algorithm:  
    1- Weak internet signal: so the data from the laser beams or the camera may arrive to the master with a delay.  
    2- Robot speed: if the robot is too fast, the master may not be able to calculate immidiatly.  
    3- Low battery: the robot computer may still on, but the motor may not work, and the data that the robot sends may not be accurate.  
    4- Wrong robot parameters (e.g. velosity, rotation speed, etc...): some parameters may not fit to the nature of the given maze.  
  Note: one or more of the problems mentioned above, may make the behaviour of the robot unexpected.  
  
  
 References:  
    Robot Operating System (ROS): http://www.ros.org/   
    ROBOTIS TurtleBot3 Manual: http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/  
    "move_base" Package: http://wiki.ros.org/move_base  