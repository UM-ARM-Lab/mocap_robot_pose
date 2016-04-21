#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import tf
from geometry_msgs.msg import *

class RightArmPosePublisher(object):

    def __init__(self, torso_frame, wrist_frame, pose_topic, rate):
        self.torso_frame = torso_frame
        self.wrist_frame = wrist_frame
        self.rate = rate
        self.pose_pub = rospy.Publisher(pose_topic, PoseStamped)
        self.tf_client = tf.TransformListener()
        rospy.loginfo("Waiting for TF cache to fill...")
        rospy.sleep(rospy.Duration(2.0))
        rospy.loginfo("Loaded TF client")
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish_arm_pose()
            rate.sleep()

    def publish_arm_pose(self):
        try:
            # Look up the transform from the torso frame to the wrist frame
            position, orientation = self.tf_client.lookupTransform(self.torso_frame, wrist_frame, rospy.Time(0))
            # Pack into a PoseStamped
            msg = PoseStamped()
            msg.header.frame_id = self.torso_frame
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            msg.pose.orientation.x = orientation[0]
            msg.pose.orientation.y = orientation[1]
            msg.pose.orientation.z = orientation[2]
            msg.pose.orientation.w = orientation[3]
            self.pose_pub.publish(msg)
        except:
            rospy.logwarn("TF lookup failed")

if __name__ == "__main__":
    rospy.init_node("right_arm_pose_pub")
    rospy.loginfo("Starting the right arm pose publisher...")
    #Get the parameters from the server
    torso_frame = rospy.get_param("~torso_frame", "/torso_lift_link")
    wrist_frame = rospy.get_param("~wrist_frame", "/mocap_right_wrist")
    pose_topic = rospy.get_param("~right_arm_pose_topic", "/r_arm_pose_controller/pose")
    rate = rospy.get_param("~rate", 50.0)
    RightArmPosePublisher(torso_frame, wrist_frame, pose_topic, rate)
