#!/usr/bin/env python

import math
import gym
import rospy
import time
import numpy as np
import tf
import time
from gym import utils, spaces
from geometry_msgs.msg import Twist, Vector3Stamped, Pose
from nav_msgs.msg import Odometry
#from hector_uav_msgs.msg import Altimeter
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty as EmptyTopicMsg
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection

#register the training environment in the gym as an available one
reg = register(
    id='QuadcopterLiveShow-v0',
    entry_point='myquadcopter_env:QuadCopterEnv'
    )


class QuadCopterEnv(gym.Env):

    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment
        
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', EmptyTopicMsg, queue_size=0)
        
        # gets training parameters from param server
        self.speed_value = rospy.get_param("/speed_value")
        self.rotation_speed_value = rospy.get_param("/rotation_value")
        self.desired_pose = Pose()
        self.desired_pose.position.z = rospy.get_param("/desired_pose/z")
        self.desired_pose.position.x = rospy.get_param("/desired_pose/x")
        self.desired_pose.position.y = rospy.get_param("/desired_pose/y")
        self.running_step = rospy.get_param("/running_step")
        self.max_incl = rospy.get_param("/max_incl")
        self.max_altitude = rospy.get_param("/max_altitude")
        self.max_distance = rospy.get_param("/max_distance")
        
        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        high = np.array([
         1])
        self.action_space = spaces.Discrete(3) #noop, up down, rot cw, rot ccw
        self.observation_space = spaces.Box(-high, high)
        self.reward_range = (-np.inf, np.inf)

        self.seed()

    # A function to initialize the random generator
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    def render(self, mode=None):
        None
    # Resets the state of the environment and returns an initial observation.
    def reset(self):
        
        # 1st: resets the simulation to initial values
        self.gazebo.resetSim()
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0


        # 2nd: Unpauses simulation
        self.gazebo.unpauseSim()
        self.vel_pub.publish(vel_cmd)

        # 3rd: resets the robot to initial conditions
        self.check_topic_publishers_connection()
        self.init_desired_pose()
        self.takeoff_sequence()
        rotation = [0,0,0]
        # 4th: takes an observation of the initial condition of the robot
        data_pose, self.data_imu = self.take_observation()
        # observation = [self.roundTo(data_pose.position.x,100)-self.roundTo(self.desired_pose.position.x,100), 
                #  self.roundTo(data_pose.position.y,100)-self.roundTo(self.desired_pose.position.y,100), 
                #  self.roundTo(data_pose.position.z,100)-self.roundTo(self.desired_pose.position.z,100), 

                 #self.roundTo(self.desired_pose.position.x,100), 
                 #self.roundTo(self.desired_pose.position.y,100), 
                 #self.roundTo(self.desired_pose.position.z,100),

                #  self.roundTo(rotation[0], 100),
                #  self.roundTo(rotation[1], 100),
                #  self.roundTo(rotation[2], 100)
                #  ]
        #observation = [data_pose.position.z - self.desired_pose.position.z, data_pose.position.z, self.data_imu.orientation.z]
        observation = [self.data_imu.orientation.z]
        # 5th: pauses simulation
        self.gazebo.pauseSim()

        with open("/root/catkin_ws/src/position.csv", "a") as myfile:
            myfile.write("reset\n")
        with open("/root/catkin_ws/src/distance.csv", "a") as myfile:
            myfile.write("reset\n")
        self.simulationStartTime = rospy.get_time()
        self.realStarttime = time.time();
        self.simulationStep = 0
        return observation

    def step(self, action):
        self.simulationStep += 1
        print("Action->" + str(action))

        # Given the action selected by the learning algorithm,
        # we perform the corresponding movement of the robot
        
        # 1st, we decide which velocity command corresponds
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0

        if action == 0: # noop
            pass
        elif action == 1: # ccw
            vel_cmd.angular.z = self.rotation_speed_value
        elif action == 2: # cw
            vel_cmd.angular.z = -self.rotation_speed_value

        # Then we send the command to the robot and let it go
        # for running_step seconds
        if (rospy.get_time() - self.simulationStartTime) > 0:
            rate = (time.time() - self.realStarttime) / (rospy.get_time() - self.simulationStartTime)
        else:
            rate = 0.1
        self.gazebo.unpauseSim()
        
        self.vel_pub.publish(vel_cmd)
        time.sleep(self.running_step * rate)
        data_pose, self.data_imu = self.take_observation()
        self.reset_cmd_vel_commands()
        
        self.gazebo.pauseSim()

        # finally we get an evaluation based on what happened in the sim
        reward,done, rotation_distance = self.process_data(data_pose, self.data_imu)

        state = [rotation_distance]
        #print "after step state"
        print (state)
        print (reward)
        with open("/root/catkin_ws/src/position.csv", "a") as myfile:
            myfile.write("rot:{}\n".format(state[0] ))
        return state, reward, done, {}

    def roundTo(self,data,to):
        return int(data * to)

    def take_observation (self):
        data_pose = None
        while data_pose is None:
            try:
#                data_pose = rospy.wait_for_message('/drone/gt_pose', Pose, timeout=5)
                data_pose = rospy.wait_for_message('/ground_truth/state', Odometry, timeout=5)
                data_pose = data_pose.pose.pose
            except Exception as e:
                None
                #rospy.loginfo("Current drone pose not ready yet, retrying for getting robot pose {0}".format(e))

        data_imu = None
        while data_imu is None:
            try:
                data_imu = rospy.wait_for_message('/ardrone/imu', Imu, timeout=5)
            except:
                None
                #rospy.loginfo("Current drone imu not ready yet, retrying for getting robot imu")
        
        return data_pose, data_imu

    def calculate_dist_between_two_Points(self,p_init,p_end):
        a = np.array((p_init.x ,p_init.y, p_init.z))
        b = np.array((p_end.x ,p_end.y, p_end.z))
        
        dist = np.linalg.norm(a-b)
        
        return dist


    def init_desired_pose(self):
        current_init_pose, self.data_imu = self.take_observation()
        self.best_dist = self.calculate_dist_between_two_Points(current_init_pose.position, self.desired_pose.position)

    def check_topic_publishers_connection(self):
        
        rate = rospy.Rate(1) # 10hz
        while(self.takeoff_pub.get_num_connections() == 0):
            rospy.loginfo("No susbribers to Takeoff yet so we wait and try again")
            rate.sleep();
        rospy.loginfo("Takeoff Publisher Connected")

        while(self.vel_pub.get_num_connections() == 0):
            rospy.loginfo("No susbribers to Cmd_vel yet so we wait and try again")
            rate.sleep();
        rospy.loginfo("Cmd_vel Publisher Connected")
        

    def reset_cmd_vel_commands(self):
        # We send an empty null Twist
        vel_cmd = Twist()
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)


    def takeoff_sequence(self, seconds_taking_off=0.1):
        rospy.loginfo("Takeoff sequence starting")
        # Before taking off be sure that cmd_vel value there is is null to avoid drifts
        self.reset_cmd_vel_commands()
        rospy.loginfo("Takeoff reset cmd_vel")
        
        takeoff_msg = EmptyTopicMsg()
        rospy.loginfo( "Taking-Off Start")
        self.takeoff_pub.publish(takeoff_msg)
        time.sleep(seconds_taking_off)
        rospy.loginfo( "Taking-Off sequence completed")
        

    def improved_distance_reward(self, current_pose, current_orientation):
        # current_dist = self.calculate_dist_between_two_Points(current_pose.position, self.desired_pose.position)
        current_dist = current_pose.position.z - self.desired_pose.position.z
        x = self.desired_pose.position.x - current_pose.position.x
        y = self.desired_pose.position.y - current_pose.position.y
        desired_angle = math.atan2(x, y)
        current_rotation_distance = current_orientation.z 
        rospy.loginfo( "rotation distance: " + str(current_rotation_distance))
        #rospy.loginfo("Calculated Distance = "+str(current_dist))
        
        self.speed_value = abs(current_dist) / 5
        self.rotation_speed_value = abs(current_rotation_distance) / 5

        # Reward:
        reward = (100 - 100 * abs(current_rotation_distance)) ** 2
        
        return reward, current_dist, current_rotation_distance
        
    def process_data(self, data_position, data_imu):

        done = False
        
        euler = tf.transformations.euler_from_quaternion([data_imu.orientation.x,data_imu.orientation.y,data_imu.orientation.z,data_imu.orientation.z])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        #print "Yaw->" + str(yaw)

        pitch_bad = not(-self.max_incl < pitch < self.max_incl)
        roll_bad = not(-self.max_incl < roll < self.max_incl)
        altitude_bad = data_position.position.z > self.max_altitude  or data_position.position.z < 0.05

        if altitude_bad or pitch_bad:
            rospy.loginfo ("(Drone flight status is wrong) >>> ("+str(altitude_bad)+","+str(pitch_bad)+","+str(roll_bad)+")")
            rospy.loginfo ("(Drone flight status is wrong) >>> ("+str(data_position.position.z)+","+str(pitch)+","+str(roll)+")")
            done = True
            reward = -2000000
            rotation_distance = 0
        else:
            reward, distance, rotation_distance = self.improved_distance_reward(data_position, data_imu.orientation)
            with open("/root/catkin_ws/src/distance.csv", "a") as myfile:
                myfile.write("{}, {}\n".format(distance, rotation_distance))

            if (distance > self.max_distance):
                done = True

        return reward, done, rotation_distance