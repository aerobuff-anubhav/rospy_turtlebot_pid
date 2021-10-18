#! /usr/bin/env python

import geometry_msgs.msg
import nav_msgs.msg
import numpy
import rospy
from tf.transformations import *

class Controller:
    def __init__(self):
        self.sim = rospy.get_param("~sim")
        self.mocap = rospy.get_param("~mocap")
        self.Kp_angular = rospy.get_param("~Kp_angular")
        self.Kp_linear = rospy.get_param("~Kp_linear")
        self.vel_x_max = rospy.get_param("~vel_x_max")
        self.yaw_rate_max = rospy.get_param("~yaw_rate_max")
        self.safety_distance = rospy.get_param("~safety_distance")

        self.publisher_cmd_vel = rospy.Publisher("pub_info", geometry_msgs.msg.Twist, queue_size=10)
        if self.sim:
            self.agent_pose = nav_msgs.msg.Odometry()
            self.pose_subscriber = rospy.Subscriber("agent_pose", nav_msgs.msg.Odometry, self.agent_pose_callback)
        else:
            self.agent_pose = geometry_msgs.msg.PoseStamped()
            self.pose_subscriber = rospy.Subscriber("agent_pose", geometry_msgs.msg.PoseStamped, self.agent_pose_callback)
        self.target_pose = geometry_msgs.msg.PoseStamped()
        self.pose_subscriber = rospy.Subscriber("target_pose", geometry_msgs.msg.PoseStamped, self.target_pose_callback)

        self.msg_pub = geometry_msgs.msg.Twist()
        self.publisher_cmd_vel.publish(self.msg_pub)
        self.loop_rate = rospy.Rate(10)

    def agent_pose_callback(self, msg):
        self.agent_pose = msg

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def command_velocity(self):
        target_x = self.target_pose.pose.position.x
        target_y = self.target_pose.pose.position.y
        target_z = self.target_pose.pose.position.z
        target_q0 = self.target_pose.pose.orientation.x
        target_q1 = self.target_pose.pose.orientation.y
        target_q2 = self.target_pose.pose.orientation.z
        target_q3 = self.target_pose.pose.orientation.w

        if self.sim:
            agent_x = self.agent_pose.pose.pose.position.x
            agent_y = self.agent_pose.pose.pose.position.y
            agent_z = self.agent_pose.pose.pose.position.z
            agent_q0 = self.agent_pose.pose.pose.orientation.x
            agent_q1 = self.agent_pose.pose.pose.orientation.y
            agent_q2 = self.agent_pose.pose.pose.orientation.z
            agent_q3 = self.agent_pose.pose.pose.orientation.w
        else:
            agent_x = self.agent_pose.pose.position.x
            agent_y = self.agent_pose.pose.position.y
            agent_z = self.agent_pose.pose.position.z
            agent_q0 = self.agent_pose.pose.orientation.x
            agent_q1 = self.agent_pose.pose.orientation.y
            agent_q2 = self.agent_pose.pose.orientation.z
            agent_q3 = self.agent_pose.pose.orientation.w

        target_q = numpy.array([target_q0, target_q1, target_q2, target_q3])
        target_e = euler_from_quaternion(target_q)

        agent_q = numpy.array([agent_q0, agent_q1, agent_q2, agent_q3])
        agent_e = euler_from_quaternion(agent_q)

        target_linear_position = numpy.array([target_x, target_y, target_z])
        agent_linear_position = numpy.array([agent_x, agent_y, agent_z])

        relative_yaw = math.atan2((target_y - agent_y),(target_x - agent_x))
        angular_error = relative_yaw - agent_e[2]
        distance_euclidean = numpy.linalg.norm(target_linear_position - agent_linear_position)

        if angular_error > math.pi:
            angular_error = angular_error - 2*math.pi
        if angular_error < -math.pi:
            angular_error = angular_error + 2*math.pi

        cmd_vel_angular = self.Kp_angular*angular_error
        cmd_vel_linear = self.Kp_linear*distance_euclidean

        if cmd_vel_linear > self.vel_x_max:
            cmd_vel_linear = self.vel_x_max

        if cmd_vel_angular > self.yaw_rate_max:
            cmd_vel_angular = self.yaw_rate_max

        if cmd_vel_angular < -self.yaw_rate_max:
            cmd_vel_angular = -self.yaw_rate_max

        if distance_euclidean < self.safety_distance:
            cmd_vel_angular = 0
            cmd_vel_linear = 0

        self.msg_pub.linear.x = cmd_vel_linear
        self.msg_pub.angular.z = cmd_vel_angular

        self.publisher_cmd_vel.publish(self.msg_pub)

    def spin(self):
        self.command_velocity()
        self.loop_rate.sleep()
