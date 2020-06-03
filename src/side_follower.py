#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
import tf_conversions as tf_con
from random import randint 
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose

class pose:
    """Hold pose information about the tutrle

    There is probably something in ROS for this but I didn't find it
    """
    def __init__(self, x, y, theta=0):
        self.x = x
        self.y = y
        self.theta = theta
 
class Side_Follower:
    """Have a turtle move with a user controlled turtle using the methode
    laid out in "How Do People Walk Side-By-Side?" by Y. Morales and S Satake
"""
    def __init__(self):
        """Set up what we need to follow"""

        # get spawn service so we can create a turtle
        rospy.wait_for_service('spawn')
        try:
            self.turtle_spawn = rospy.ServiceProxy('spawn', Spawn)
        except rospy.ServiceException as ex:
            rospy.signal_shutdown('Could not spawn!')

        # spwan our new turtle friend in the middle
        self.turtle_name = rospy.get_param('turtle', 'follower')
        self.turtle_spawn(5, 5, 0,
                          self.turtle_name)

        # spawn a turtle to mark the goal point
        # the goals are defined as static tf2 transformts in the launch file
        self.turtle_goal1 = rospy.get_param('turtle', 'goal1')
        self.turtle_spawn(9, 9, 0,
                          self.turtle_goal1)

        # get the telporter
        telport_name = self.turtle_name + '/teleport_absolute'
        rospy.wait_for_service(telport_name)
        try:
            self.turtle_telport = rospy.ServiceProxy(telport_name, \
                                                TeleportAbsolute)
        except rospy.ServiceException as ex:
            rospy.signal_shutdown('Cant beam you up scotty!')

        # get the leader pose
        # I use this for the leader velcotiy
        # I should get that velocity by just rembering where the leader
        # was and calculating it
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.leader_pose = Pose()

        # set up the tf2 listener 
        self.tfBuffer = tf2_ros.Buffer()
        tf_listen = tf2_ros.TransformListener(self.tfBuffer)

        # set up timing
        self.rate_value = 10#hz
        self.delta_t = 1.0/self.rate_value
        self.rate = rospy.Rate(self.rate_value)
        self.rate.sleep() # let things connect

    def follow(self):
        """Stay with the controlled turtle to the goal"""

        # we start stoped
        follow_vel = 0

        while not rospy.is_shutdown():

            # need leader velcotity for the leader acceration util
            lead_vel = self.leader_pose.linear_velocity
            
            # get the turtle and obstical transform
            # I know that I'm getting global so I could just
            # suscribe to the turtles pose, but I am
            # using tf2 because my first attempt with this code I tried
            # to do everything in follower corridantes, and still think
            # that's a better way for a real robot
            try:
                world2lead_trans = self.tfBuffer.lookup_transform(
                    'world', 'turtle1', rospy.Time())

                world2follow_trans = self.tfBuffer.lookup_transform(
                    'world', self.turtle_name, rospy.Time())

                world2goal1_trans = self.tfBuffer.lookup_transform(
                    'world', 'goal1', rospy.Time())
                
            # this execption and response is from the tutorial
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue

            # determine if we're in the follower or collaboration state
            # spoiler: I didn't write follower code or more than one
            # obstical so we are alwasy in the collaboration state
            self.determin_collaboration()

            # get all the nessary poses
            lead_pose_global = world2lead_trans.transform.translation
            follow_pose_global = world2follow_trans.transform.translation
            goal1_pose_global = world2goal1_trans.transform.translation

            max_util = -1e20
            # this determins the locations the robot will exam moving to
            grid_position = [i * 0.1 for i in range(-3,4)] 
            # am I doing this in the worst way possibel? probably
            for lead_x_off in grid_position:
                for lead_y_off in grid_position:
                    for follow_x_off in grid_position:
                        for follow_y_off in grid_position:
                            # canidate leader pose
                            new_lead = pose(\
                                lead_pose_global.x + lead_x_off,\
                                lead_pose_global.y + lead_y_off)

                            # canidate follower pose
                            new_follow = pose(\
                                follow_pose_global.x + follow_x_off, \
                                follow_pose_global.y + follow_y_off)

                            # get the predicted lead utility
                            util_lead = self.utility_collaboration(\
                                lead_pose_global, follow_pose_global,\
                                new_lead, new_follow, lead_vel,\
                                goal1_pose_global)

                            # get the predicted follower utility
                            util_follow = self.utility_collaboration(\
                                follow_pose_global, lead_pose_global,\
                                new_follow, new_lead, follow_vel,\
                                goal1_pose_global)

                            # we find the best utilty for both the leader
                            # and follower
                            util = util_lead + util_follow
                            if util > max_util:
                                max_util = util
                                best_move = new_follow

            follow_vel = self.move_follower(best_move, follow_pose_global)
            self.rate.sleep()

    def move_follower(self, destination, current_position):
        # controlers are hard
        # I spend a while trying to get the turlte diff drive work
        # well, but I just ran out of time and that's not really the
        # important problem here
        self.turtle_telport(destination.x, destination.y, 0)
        # this velocity is sort of a lie because we teloprted
        vel = self.calc_velocity(current_position, destination)
        return vel

    def determin_collaboration(self):
        # determin in we are in collaboration state or follower state
        self.collaboration = True


    def utility_collaboration(self, pose_a, pose_b, new_pose_a, new_pose_b, a_velocity, goal):
        #define utility function scalers
        #         [k_Eo, k_Es, k_Rd, k_Ra, k_Rv, k_Ma, k_Mv]
        #scalers = [0.11, 0.20, 0.25, 0.32, 0.01, 0.01, 0.05]
        scalers = [0.10, 0.20, 0.25, 0.15, 0.01, 0.01, 0.05]

        ou = self.obstical_util(pose_a)
        es = self.subgoal_util(pose_a, new_pose_a, goal)
        rdu = self.relative_distance_util(new_pose_a, new_pose_b)
        rau = self.relative_angle_util(pose_a, new_pose_a, new_pose_b)
        rvu = self.relative_velocity_util( \
            pose_a, new_pose_a, pose_b, new_pose_b)
        au = self.acceration_util(pose_a, new_pose_a, a_velocity)
        vu = self.velocity_util(pose_a, new_pose_a)
        # I'm treating the follower like a point source so angular velocity
        # does not make a ton of sense
        # It's also weighted low in the paper so I thinking ignoring it
        # should be okay
        #av = self.angular_velocity_util(pose_a, new_pose_a)

        utilities = [ou, es, rdu, rau, rvu, au, vu]
        total_utility = sum(np.array(scalers) * np.array(utilities))
        return total_utility

    # utility functions
    def obstical_util(self, new_pose):
        """
        Get the utility of being near obsticals

        new_pose_global: Pose of agent at t+1 in global frame
        """
        # to make life a little easier the only obsticals will
        # be the sides of the sim.
        # the walls are around y=0, y=11, x=0, x=11
        wall_far = 11
        distances_to_wall = 4 * [0]
        distances_to_wall[0] = new_pose.y
        distances_to_wall[1] = new_pose.x
        distances_to_wall[2] = wall_far - new_pose.y
        distances_to_wall[3] = wall_far - new_pose.x
        # it dosent mater what wall we're near
        min_dist = min(distances_to_wall)
        #print("obs dist " + str(min_dist))
        cost = self.obstacel_field(min_dist)
        return cost

    def subgoal_util(self, pose, new_pose, goal):
        """
        Get the utility of moving towards the goal

        new_pose: pose at t+1 in global frame
        goal: goal position in global frame
        """
        if self.collaboration:
            a = 0.75
            b = 1.0
            c = 0
        else:
            a = 0.3
            b = 1.5
            c = 0
        
        # we need the angle between the vecotr of our new pose to current
        # pose and goal to new pose
        new2old = np.array([pose.x-new_pose.x, pose.y-new_pose.y])
        goal2new = np.array([new_pose.x - goal.x, new_pose.y - goal.y])
        # compute the cost
        angle = self.vector_angle(new2old, goal2new)
        #print("sub angle " + str(angle))
        cost = self.power_function(angle, a, b, c)
        return cost

    def velocity_util(self, pose, new_pose):
        """ Get the utility of moving at the prefered walking speed

        pose: pose in the global frame
        new_pose: pose at t+1 in the global frame
        """
        if self.collaboration:
            a = 0.9 
            b = 1.6
            c = 2.0 # paper has walking speed at 1.1, but sim is diffrent
        else:
            a = 0.9 
            b = 1.6
            c = 1.10

        velocity = self.calc_velocity(pose, new_pose)
        #print("velcotiy " + str(velocity))
        cost = self.power_function(velocity, a, b, c)
        return cost
        
    def angular_velocity_util(self, pose, new_pose):
        """ Get the utility of moving straight

        pose: pose in the global frame
        new_poser: pose at t+1 in the global frame
        """
        if self.collaboration:
            a = 0.7
            b = 4.4
            c = 0
        else:
            a = 0.7
            b = 4.4
            c = 0

        angular_velocity = np.arctan2(new_pose.y - pose.y, new_pose.x - pose.x) / self.delta_t
        #print('ang vel ' + str(angular_velocity))
        cost = self.power_function(angular_velocity, a, b, c)
        return cost
        
    def acceration_util(self, pose, new_pose, old_vel):
        """ Get the utlity of minimizing changes in speed

        pose: current pose in global corrodiantes
        new_pose: pose at t+1 in global corrodinates
        old_vel: current linear velocity
        """
        if self.collaboration:
            a = 1.5
            b = 1.0
            c = 0
        else:
            a = 1.5
            b = 1.0
            c = 0

        velocity = self.calc_velocity(pose, new_pose)
        acceration = (velocity - old_vel)/self.delta_t
        #print("accel " + str(acceration))
        cost = self.power_function(np.abs(acceration), a, b, c)
        return cost

    def relative_distance_util(self, new_pose_a, new_pose_b):
        """ Utility of keeping a confertable distance between the two agents

        new_pose_a: pose at t+1 of one agent
        new_pose_b: pose at t+1 of the other agent
        """
        if self.collaboration:
            a = 0.45
            b = 2.0
            c = 0.75
        else:
            a = 0.45
            b = 2.0
            c = 0.75
        
        distance = np.sqrt((new_pose_a.x - new_pose_b.x)**2 + \
                        (new_pose_a.y - new_pose_b.y)**2)
        #print("rel dist " + str(distance))
        cost = self.power_function(distance, a, b, c)
        return cost

    def relative_angle_util(self, pose_a, new_pose_a, new_pose_b):
        """ Utility of moving in the same direction as the partner

        pose_a: current pose in global frame
        new_pose_a: pose at t+1 in the global frame
        new_pose_b: pose of other agent at t+1 in the global frame
        """
        if self.collaboration:
            a = 0.08
            b = 3.0
            c = np.pi / 2 
        else:
            a = 0.08
            b = 3.0
            c = np.pi / 2 

        new2old = np.array(\
            [pose_a.x - new_pose_a.x, pose_a.y - new_pose_a.y])

        b2a = np.array(\
            [new_pose_a.x - new_pose_b.x, new_pose_a.y - new_pose_b.y])

        angle = self.vector_angle(new2old, b2a)
        cost = self.power_function(angle, a, b, c)
        #print("rel ang " + str(angle) + ' cost' + str(cost))
        return cost

    def relative_velocity_util(self, pose_a, new_pose_a, pose_b, new_pose_b):
        """ Utility of partners moving at the same speed

        pose_a: pose at time t+1 in agent_a frame
        pose_b: pose at time t+1 in agent_b frame
        """
        if self.collaboration:
            a = 0.2
            b = 1.2
            c = 0
        else:
            a = 0.2
            b = 1.2
            c = 0

        velocity_a = self.calc_velocity(pose_a, new_pose_a)
        velocity_b = self.calc_velocity(pose_b, new_pose_b)
        #print("rel vel: " + str(velocity_a - velocity_b))
        cost = self.power_function(abs(velocity_a-velocity_b), a, b, c)
        return cost

    # cost functions
    def obstacel_field(self, x):
        """Obstical field from paper
        x is distance to obstical
        """
        # values from 2012 paper, same for collab and leader-follower
        # the 2014 paper has a=400 and b=500, but that is cazy because
        # b is in an exponent
        a = 20
        b = 0.2
        # this makes up for me not actualy knowing where the walls
        # are exactly in the simulator
        if x < 0.3:
            x = 0.3
        value = (x / a) ** (-2*b)
        return -1 * np.abs(value)

    def power_function(self, x, a, b, c):
        """Power function from paper
        This is used for all utlity functions except the obstical
        and open space
        x is the given value
        a, b, c are tunable parameters
        """
        # 2014 paper power function
        # value = |((x-c)/a)^2b|
        # 2012 paper power function
        # value = 1/(1+|((x-c)/a)^2b|) - 1

        # we going play some tricks to get around numpy problems
        # numpy wont let you rase a negative number to a float power
        # because it can not figure predict the data type
        # to get around this we are going to do the abs first
        fraction = (x-c) / a
        value = -1 * np.power(np.abs(fraction), 2*b) 
        
        # we're using the 2012 function
        value = (1/(1 - value)) - 1

        return value

    # math helper functions
    def calc_velocity(self, pose, new_pose):
        """Get the velcoty need to move between poses"""
        distance = np.sqrt((new_pose.x - pose.x)**2 + \
                (new_pose.y - pose.y)**2)
        velocity = distance / self.delta_t
        return velocity
   
    def vector_angle(self, vector1, vector2):
        """Get the angle between two vecotrs"""

        # need to handel 0
        if vector1[0] == 0 and vector1[1] == 0:
            vector1_unit = np.array([0,0]) 
        else:
            vector1_unit = vector1 / np.linalg.norm(vector1)

        if vector2[0] == 0 and vector2[1] == 0:
            vector2_unit = np.array([0,0]) 
        else:
            vector2_unit = vector2 / np.linalg.norm(vector2)

        dot_prod = np.dot(vector1_unit, vector2_unit)
        # clip to deal with 1 and -1 cases
        angle = np.arccos(np.clip(dot_prod, -1.0, 1.0))
        return angle

    def quant_struct2angle(self, quaternion_struct):
        # unpack the tf2 angles and change it
        quant = [quaternion_struct.x,
                 quaternion_struct.y,
                 quaternion_struct.z,
                 quaternion_struct.w]
        euler = tf_con.transformations.euler_from_quaternion(quant)
        return(euler[2])

    # call backs
    def callback(self, data):
        """ callback for leader pose"""
        self.leader_pose = data
   

if __name__ == '__main__':
    rospy.init_node('leader-follower-basic')
    follower = Side_Follower()
    follower.follow()
    rospy.spin()
