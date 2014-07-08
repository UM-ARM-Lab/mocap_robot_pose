#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <lightweight_vicon_bridge/MocapState.h>

typedef Eigen::Affine3d Pose;

std::unique_ptr<tf::TransformListener> g_transform_listener;

ros::Publisher g_left_arm_pose_pub;
ros::Publisher g_right_arm_pose_pub;
std::string g_robot_object_name;
std::string g_robot_segment_name;
std::string g_left_arm_object;
std::string g_right_arm_object;
std::string g_left_arm_segment;
std::string g_right_arm_segment;

void PublishLeftArmPose(tf::StampedTransform& torso_transform, geometry_msgs::Transform& mocap_base_pose, geometry_msgs::Transform& mocap_left_arm_pose)
{
    // Make the Eigen transform for the torso
    Eigen::Translation3d torso_to_base_translation(torso_transform.getOrigin().x(), torso_transform.getOrigin().y(), torso_transform.getOrigin().z());
    Eigen::Quaterniond torso_to_base_rotation(torso_transform.getRotation().w(), torso_transform.getRotation().x(), torso_transform.getRotation().y(), torso_transform.getRotation().z());
    Pose torso_to_base = torso_to_base_translation * torso_to_base_rotation;
    // Make the Eigen transforms for the base
    Eigen::Translation3d world_to_base_translation(mocap_base_pose.translation.x, mocap_base_pose.translation.y, mocap_base_pose.translation.z);
    Eigen::Quaterniond world_to_base_rotation(mocap_base_pose.rotation.w, mocap_base_pose.rotation.x, mocap_base_pose.rotation.y, mocap_base_pose.rotation.z);
    Pose world_to_base = world_to_base_translation * world_to_base_rotation;
    Pose world_to_base_inv = world_to_base.inverse();
    // Make the Eigen transform for the hand
    Eigen::Translation3d world_to_left_hand_translation(mocap_left_arm_pose.translation.x, mocap_left_arm_pose.translation.y, mocap_left_arm_pose.translation.z);
    Eigen::Quaterniond world_to_left_hand_rotation(mocap_left_arm_pose.rotation.w, mocap_left_arm_pose.rotation.x, mocap_left_arm_pose.rotation.y, mocap_left_arm_pose.rotation.z);
    Pose world_to_left_hand = world_to_left_hand_translation * world_to_left_hand_rotation;
    // Make the fixed hand->wrist offset transform
    Eigen::Translation3d left_hand_to_left_wrist_translation(-0.12, 0.0, -0.048);
    Eigen::Quaterniond left_hand_to_left_wrist_rotation(1.0, 0.0, 0.0, 0.0);
    Pose left_hand_to_left_wrist = left_hand_to_left_wrist_translation * left_hand_to_left_wrist_rotation;
    // Compose the transforms together
    Pose torso_to_left_wrist = torso_to_base * world_to_base_inv * world_to_left_hand * left_hand_to_left_wrist;
    // Extract from Eigen and publish
    Eigen::Quaterniond torso_to_left_wrist_rotation(torso_to_left_wrist.rotation());
    geometry_msgs::PoseStamped left_arm_pose;
    left_arm_pose.header.frame_id = "/torso_lift_link";
    left_arm_pose.pose.position.x = torso_to_left_wrist.translation().x();
    left_arm_pose.pose.position.y = torso_to_left_wrist.translation().y();
    left_arm_pose.pose.position.z = torso_to_left_wrist.translation().z();
    left_arm_pose.pose.orientation.x = torso_to_left_wrist_rotation.x();
    left_arm_pose.pose.orientation.y = torso_to_left_wrist_rotation.y();
    left_arm_pose.pose.orientation.z = torso_to_left_wrist_rotation.z();
    left_arm_pose.pose.orientation.w = torso_to_left_wrist_rotation.w();
    g_left_arm_pose_pub.publish(left_arm_pose);
}

void PublishRightArmPose(tf::StampedTransform& torso_transform, geometry_msgs::Transform& mocap_base_pose, geometry_msgs::Transform& mocap_right_arm_pose)
{
    // Make the Eigen transform for the torso
    Eigen::Translation3d torso_to_base_translation(torso_transform.getOrigin().x(), torso_transform.getOrigin().y(), torso_transform.getOrigin().z());
    Eigen::Quaterniond torso_to_base_rotation(torso_transform.getRotation().w(), torso_transform.getRotation().x(), torso_transform.getRotation().y(), torso_transform.getRotation().z());
    Pose torso_to_base = torso_to_base_translation * torso_to_base_rotation;
    // Make the Eigen transforms for the base
    Eigen::Translation3d world_to_base_translation(mocap_base_pose.translation.x, mocap_base_pose.translation.y, mocap_base_pose.translation.z);
    Eigen::Quaterniond world_to_base_rotation(mocap_base_pose.rotation.w, mocap_base_pose.rotation.x, mocap_base_pose.rotation.y, mocap_base_pose.rotation.z);
    Pose world_to_base = world_to_base_translation * world_to_base_rotation;
    Pose world_to_base_inv = world_to_base.inverse();
    // Make the Eigen transform for the hand
    Eigen::Translation3d world_to_right_hand_translation(mocap_right_arm_pose.translation.x, mocap_right_arm_pose.translation.y, mocap_right_arm_pose.translation.z);
    Eigen::Quaterniond world_to_right_hand_rotation(mocap_right_arm_pose.rotation.w, mocap_right_arm_pose.rotation.x, mocap_right_arm_pose.rotation.y, mocap_right_arm_pose.rotation.z);
    Pose world_to_right_hand = world_to_right_hand_translation * world_to_right_hand_rotation;
    // Make the fixed hand->wrist offset transform
    Eigen::Translation3d right_hand_to_right_wrist_translation(-0.12, 0.0, -0.048);
    Eigen::Quaterniond right_hand_to_right_wrist_rotation(1.0, 0.0, 0.0, 0.0);
    Pose right_hand_to_right_wrist = right_hand_to_right_wrist_translation * right_hand_to_right_wrist_rotation;
    // Now that we have all the transforms, compose the transforms together
    Pose torso_to_right_wrist = torso_to_base * world_to_base_inv * world_to_right_hand * right_hand_to_right_wrist;
    // Extract from Eigen and publish
    Eigen::Quaterniond torso_to_right_wrist_rotation(torso_to_right_wrist.rotation());
    geometry_msgs::PoseStamped right_arm_pose;
    right_arm_pose.header.frame_id = "/torso_lift_link";
    right_arm_pose.pose.position.x = torso_to_right_wrist.translation().x();
    right_arm_pose.pose.position.y = torso_to_right_wrist.translation().y();
    right_arm_pose.pose.position.z = torso_to_right_wrist.translation().z();
    right_arm_pose.pose.orientation.x = torso_to_right_wrist_rotation.x();
    right_arm_pose.pose.orientation.y = torso_to_right_wrist_rotation.y();
    right_arm_pose.pose.orientation.z = torso_to_right_wrist_rotation.z();
    right_arm_pose.pose.orientation.w = torso_to_right_wrist_rotation.w();
    g_right_arm_pose_pub.publish(right_arm_pose);
}

void MocapMsgCB(lightweight_vicon_bridge::MocapState msg)
{
    bool got_base = false;
    bool got_left_arm = false;
    bool got_right_arm = false;
    geometry_msgs::Transform base_transform;
    geometry_msgs::Transform left_arm_transform;
    geometry_msgs::Transform right_arm_transform;
    for (size_t idx_o = 0; idx_o < msg.tracked_objects.size(); idx_o++)
    {
        lightweight_vicon_bridge::MocapObject current_object = msg.tracked_objects[idx_o];
        if (current_object.name == g_robot_object_name)
        {
            for (size_t idx_s = 0; idx_s < current_object.segments.size(); idx_s++)
            {
                lightweight_vicon_bridge::MocapSegment current_segment = current_object.segments[idx_s];
                if (current_segment.name == g_robot_segment_name)
                {
                    if (!current_segment.occluded)
                    {
                        base_transform = current_segment.transform;
                        got_base = true;
                    }
                    else
                    {
                        ROS_WARN("Tracking robot base, but base is occluded");
                    }
                }
            }
        }
        else if (current_object.name == g_left_arm_object)
        {
            for (size_t idx_s = 0; idx_s < current_object.segments.size(); idx_s++)
            {
                lightweight_vicon_bridge::MocapSegment current_segment = current_object.segments[idx_s];
                if (current_segment.name == g_left_arm_segment)
                {
                    if (!current_segment.occluded)
                    {
                        left_arm_transform = current_segment.transform;
                        got_left_arm = true;
                    }
                    else
                    {
                        ROS_WARN("Tracking left arm, but left arm is occluded");
                    }
                }
            }
        }
        else if (current_object.name == g_right_arm_object)
        {
            for (size_t idx_s = 0; idx_s < current_object.segments.size(); idx_s++)
            {
                lightweight_vicon_bridge::MocapSegment current_segment = current_object.segments[idx_s];
                if (current_segment.name == g_right_arm_segment)
                {
                    if (!current_segment.occluded)
                    {
                        right_arm_transform = current_segment.transform;
                        got_right_arm = true;
                    }
                    else
                    {
                        ROS_WARN("Tracking right arm, but right arm is occluded");
                    }
                }
            }
        }
    }
    // Query TF to get the transform from /torso_lift_link to /base_footprint
    try
    {
        tf::StampedTransform torso_transform;
        g_transform_listener->lookupTransform("/torso_lift_link", "/base_footprint", ros::Time(0), torso_transform);
        // Now that we have a transform, go to work
        if (got_base)
        {
            if (got_left_arm)
            {
                PublishLeftArmPose(torso_transform, base_transform, left_arm_transform);
            }
            else
            {
                ROS_WARN("Unable to publish left arm pose");
            }
            if (got_right_arm)
            {
                PublishRightArmPose(torso_transform, base_transform, right_arm_transform);
            }
            else
            {
                ROS_WARN("Unable to publish right arm pose");
            }
        }
        else
        {
            ROS_WARN("No base pose - unable to publish either left or right arm pose");
        }
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Unable to lookup transform from /torso_lift_link to /base_footprint");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap_robot_arm_pub");
    ROS_INFO("Starting mocap robot arm pub...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string tracker_topic;
    std::string left_arm_pose_topic;
    std::string right_arm_pose_topic;
    nhp.param(std::string("tracker_topic"), tracker_topic, std::string("mocap_tracking"));
    nhp.param(std::string("left_arm_pose_topic"), left_arm_pose_topic, std::string("/l_arm_pose_controller/pose"));
    nhp.param(std::string("right_arm_pose_topic"), right_arm_pose_topic, std::string("/r_arm_pose_controller/pose"));
    nhp.param(std::string("robot_object_name"), g_robot_object_name, std::string("ArchieBase"));
    nhp.param(std::string("robot_segment_name"), g_robot_segment_name, std::string("ArchieBase"));
    nhp.param(std::string("left_arm_object"), g_left_arm_object, std::string("ArchieLeftHand"));
    nhp.param(std::string("right_arm_object"), g_right_arm_object, std::string("ArchieRightHand"));
    nhp.param(std::string("left_arm_segment"), g_left_arm_segment, std::string("ArchieLeftHand"));
    nhp.param(std::string("right_arm_segment"), g_right_arm_segment, std::string("ArchieRightHand"));
    // Create the pose publishers
    g_left_arm_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(left_arm_pose_topic, 1, false);
    g_right_arm_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(right_arm_pose_topic, 1, false);
    // Create the TF listener
    g_transform_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener(nh, ros::Duration(20.0)));
    ROS_INFO("Waiting for a few seconds to fill the TF cache...");
    ros::Duration(2.5).sleep();
    // Create the mocap data subscriber
    ros::Subscriber mocap_sub = nh.subscribe(tracker_topic, 1, MocapMsgCB);
    // Start streaming data
    ROS_INFO("Streaming data...");
    while (ros::ok())
    {
        // Handle ROS stuff
        ros::spinOnce();
    }
    return 0;
}

