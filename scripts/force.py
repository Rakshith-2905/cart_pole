#!/usr/bin/env python

import roslib
roslib.load_manifest('cart_pole')
import rospy,os,sys
from std_srvs.srv import Empty as EmptySrv
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty as EmptyMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench
import tf.transformations as tft
import math


class MoveCart():
    def __init__(self):
        try:
            # Initialize Node
            rospy.init_node("cart_pole", anonymous=True)

            # Create Publisher for publishing Velocity message
            self.vel_pub = rospy.Publisher("/mybot/cmd_vel", Twist, queue_size=100)

            # Create a service for publishing force messages
            self.apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            # Create a service to pause physics
            self.g_pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptySrv)
            # Create a service to unpause physics
            self.g_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptySrv)
            # Create a service for teleporting robot
            self.g_set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            # Create a joint state publisher
            self.joint_states_pub = rospy.Publisher('/joint_states', JointState)
            self.body_name = "mybot::footprint" # Name_of_robot::link
            self.wrench = Wrench() # create a object of the class Wrench()
            self.force_prev = None
            self.r = rospy.Rate(50) # 50hz
            self.x = 0
            self.x_dot = 0
            self.o = 0
            self.o_dot = 0
            self.previous_o = 0
            self.string = None

        except rospy.ROSInterruptException:
            pass

    def odometry(self,msg):
        # print('X: ',msg.pose.pose.position.x)
        # print('X_dot: ',msg.twist.twist.linear.x)
        self.x = msg.pose.pose.position.x
        self.x_dot =  msg.twist.twist.linear.x
        return self.x,self.x_dot

    def joint_states_callback(self,msg):

        self.o = msg.position
        self.o = self.o[0]
        # print self.o[0]
        return self.o#,self.x_dot

    def joint_states_publisher(self):
        msg = JointState()
        msg.name = ['rod_to_chassis']
        msg.position = [0]
        msg.velocity = []
        msg.effort = []
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'rod_to_chassis'
        self.joint_states_pub.publish(msg)

    def reset(self):

        rospy.wait_for_service("/gazebo/pause_physics")
        # rospy.loginfo("Pausing physics")

        try:
          self.g_pause()
        except Exception, e:
           rospy.logerr('Error on calling service: %s',str(e))

        pose = Pose()

        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 0

        state = ModelState()

        state.model_name = "mybot"
        state.pose = pose

        # rospy.loginfo("Moving robot")

        try:
          ret = self.g_set_state(state)
          print ret.status_message
        except Exception, e:
           rospy.logerr('Error on calling service: %s',str(e))

        # rospy.loginfo("Unpausing physics")
        try:
          self.g_unpause()
        except Exception, e:
           rospy.logerr('Error on calling service: %s',str(e))

        self.joint_states_publisher()


    def apply_force(self,force,reset=None):
        if force > 0:force = 10
        if force < 0:force = -10

        self.wrench.force.x = force
        self.wrench.force.y = 0
        self.wrench.force.z = 0

        try:
            resp1 = self.apply_body_wrench(self.body_name, "", None, self.wrench, rospy.Time.from_sec(0), rospy.Duration.from_sec(0.02))
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
        rospy.Subscriber('/mybot/odom',Odometry,self.odometry,queue_size=1)
        rospy.Subscriber('/mybot/joint_states', JointState, self.joint_states_callback,queue_size=1)

        self.r.sleep()

        if(abs(self.x > 2.4)):

            self.string = 'Done'
            self.reset()
            self.x = 0
            pass
        else:self.string = None
        self.o_dot = (self.o - self.previous_o)/0.02
        self.previous_o = self.o
        return self.x,self.x_dot,self.o,self.o_dot,self.string
if __name__ == "__main__":
    a = MoveCart()
    count = 0
    while not rospy.is_shutdown():
        f = a.apply_force(10)
        print(f)
        count += 1
    # rospy.spin()
