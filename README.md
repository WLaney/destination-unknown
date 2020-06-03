# Destination Unknown

*This project was originaly submited for ROS Assigment 3 in Cornell [MAE 6710: Human Robot Interaction](http://hriclass.com/). Only the readme has been changed from that submmission.*

I implemented a portion of "[Destination unknown: Walking side-by-side without knowing the goal](https://www.researchgate.net/publication/262391011_Destination_unknown_Walking_side-by-side_without_knowing_the_goal)" that covers the situation where both agents know the goal. This implementation was done in the ROS turtle sim enviroment. 

The situation where both agents know the goal is a prerequisite to implementing the unknown goal situation. I was intending to implement all of Destination Unknown, but reduced the scope of the project once I started working. My code is structured so that I could expand upon it and implement the remainder of Destination Unknown.

The know goal situation was the topic of an earlier paper by the same authors titled "[How Do People Walk Side-By-Side? – Using A Computational Model Of Human Behavior For A Social Robot](http://www.dylanglas.com/research/pdf/morales-hri2012.pdf) ''. There are differences in the utility function and tuning parameters between the two papers, and I used almost the exact tuning parameters used in How Do People Walk Side-By-Side. I chose these parameters as a starting place over the ones in Destination Unknown because some of the parameters in Destination Unknown struck me as strange. Most concerningly is the obstacle function that raises a number to the negative one thousand power.

There is one major diffrence between my implementation and the method put foward in the papers: I modeled my robot as a holonomic point robot, and not a differential drive robot. This led to me getting rid of the angular velocity utility function. This utility function had a low weight, so I do not think omitting it changed the robot behavior significantly.

There are thirty one parameters in this model, and many are coupled. The two I change from what was put forward in How Do People Walk Side-By-Side was the overall weighting on the relative angle utility, and the nominal speed in the velocity utility. I lowered the relative angle utility weight because this utility was causing the robot to get stuck in states where it believed it should not move. I changed the nominal speed to match the speed of the leader.

With this very light tuning the follower shows poor behavior in the simulator. I believe this is because the leader violates a lot of the papers expected behaviors. Specifically the controls of telop turtle lead it to have rapid acceleration, and not always move directly towards the goal. The follower was not capable of doing that, and also did not expect the leader to do that in its movement prediction.

## Installation

### Pre-reqs

Install ROS Kinetic

Install TurtleSim

### Set-Up
Navigate to ~/catkin_ws/src/

Create a package: `catkin_create_pkg laney_hw3 std_msgs rospy roscpp`

replace `laney_hw3/src/` with the src directory in the repository

move `launch/` from the repository into `laney_hw3/`

Make files executable: `chmod +x laney_hw3/src/side_follower.py`

Navigate to `~/catkin_ws/`

Run `catkin_make`

### Run
Run `source devel/setup.bash`

roslaunch `laney_hw3 laney_hw3.launch`
