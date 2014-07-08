#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <lightweight_vicon_bridge/MocapState.h>

// These values are tuned by hand as a "best match" to the real robot, since it's hard to measure directly
#define LEFT_FOREARM_X_OFFSET -0.235
#define LEFT_FOREARM_Y_OFFSET 0.0
#define LEFT_FOREARM_Z_OFFSET 0.02
#define RIGHT_FOREARM_X_OFFSET -0.230
#define RIGHT_FOREARM_Y_OFFSET 0.0
#define RIGHT_FOREARM_Z_OFFSET 0.02

typedef Eigen::Affine3d Pose;

std::unique_ptr<tf::TransformListener> g_transform_listener;
std::unique_ptr<tf::TransformBroadcaster> g_transform_broadcaster;

ros::Publisher g_left_arm_pose_pub;
ros::Publisher g_right_arm_pose_pub;
std::string g_robot_object_name;
std::string g_robot_segment_name;
std::string g_left_arm_object;
std::string g_right_arm_object;
std::string g_left_arm_segment;
std::string g_right_arm_segment;

void PublishLeftArmPose(tf::StampedTransform& torso_transform, tf::StampedTransform& forearm_to_wrist_transform,  geometry_msgs::Transform& mocap_base_pose, geometry_msgs::Transform& mocap_left_arm_pose)
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
    // Make the Eigen transform for the mocap forearm
    Eigen::Translation3d world_to_left_mocap_forearm_translation(mocap_left_arm_pose.translation.x, mocap_left_arm_pose.translation.y, mocap_left_arm_pose.translation.z);
    Eigen::Quaterniond world_to_left_mocap_forearm_rotation(mocap_left_arm_pose.rotation.w, mocap_left_arm_pose.rotation.x, mocap_left_arm_pose.rotation.y, mocap_left_arm_pose.rotation.z);
    Pose world_to_left_mocap_forearm = world_to_left_mocap_forearm_translation * world_to_left_mocap_forearm_rotation;
    // Make the fixed mocap left forearm->real left forearm transform
    Eigen::Translation3d mocap_left_forearm_to_left_forearm_translation(LEFT_FOREARM_X_OFFSET, LEFT_FOREARM_Y_OFFSET, LEFT_FOREARM_Z_OFFSET);
    Eigen::Quaterniond mocap_left_forearm_to_left_forearm_rotation(1.0, 0.0, 0.0, 0.0);
    Pose mocap_left_forearm_to_left_forearm = mocap_left_forearm_to_left_forearm_translation * mocap_left_forearm_to_left_forearm_rotation;
    // Make the Eigen transform for the left forearm->left wrist
    Eigen::Translation3d left_forearm_to_left_wrist_translation(forearm_to_wrist_transform.getOrigin().x(), forearm_to_wrist_transform.getOrigin().y(), forearm_to_wrist_transform.getOrigin().z());
    Eigen::Quaterniond left_forearm_to_left_wrist_rotation(forearm_to_wrist_transform.getRotation().w(), forearm_to_wrist_transform.getRotation().x(), forearm_to_wrist_transform.getRotation().y(), forearm_to_wrist_transform.getRotation().z());
    Pose left_forearm_to_left_wrist = left_forearm_to_left_wrist_translation * left_forearm_to_left_wrist_rotation;
    // Compose the transforms together
    Pose torso_to_left_wrist = torso_to_base * world_to_base_inv * world_to_left_mocap_forearm * mocap_left_forearm_to_left_forearm * left_forearm_to_left_wrist;
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
    // Publish as a TF frame
    /* Convert to a TF transform and publish */
    tf::Transform left_arm_tf;
    left_arm_tf.setOrigin(tf::Vector3(left_arm_pose.pose.position.x, left_arm_pose.pose.position.y, left_arm_pose.pose.position.z));
    left_arm_tf.setRotation(tf::Quaternion(left_arm_pose.pose.orientation.x, left_arm_pose.pose.orientation.y, left_arm_pose.pose.orientation.z, left_arm_pose.pose.orientation.w));
    /* Publish TF */
    tf::StampedTransform left_arm_tf_msg(left_arm_tf, torso_transform.stamp_, std::string("/torso_lift_link"), std::string("/mocap_left_wrist"));
    g_transform_broadcaster->sendTransform(left_arm_tf_msg);
}

void PublishRightArmPose(tf::StampedTransform& torso_transform, tf::StampedTransform& forearm_to_wrist_transform, geometry_msgs::Transform& mocap_base_pose, geometry_msgs::Transform& mocap_right_arm_pose)
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
    // Make the Eigen transform for the mocap forearm
    Eigen::Translation3d world_to_right_mocap_forearm_translation(mocap_right_arm_pose.translation.x, mocap_right_arm_pose.translation.y, mocap_right_arm_pose.translation.z);
    Eigen::Quaterniond world_to_right_mocap_forearm_rotation(mocap_right_arm_pose.rotation.w, mocap_right_arm_pose.rotation.x, mocap_right_arm_pose.rotation.y, mocap_right_arm_pose.rotation.z);
    Pose world_to_right_mocap_forearm = world_to_right_mocap_forearm_translation * world_to_right_mocap_forearm_rotation;
    // Make the fixed mocap right forearm->real right forearm transform
    Eigen::Translation3d mocap_right_forearm_to_right_forearm_translation(RIGHT_FOREARM_X_OFFSET, RIGHT_FOREARM_Y_OFFSET, RIGHT_FOREARM_Z_OFFSET);
    Eigen::Quaterniond mocap_right_forearm_to_right_forearm_rotation(1.0, 0.0, 0.0, 0.0);
    Pose mocap_right_forearm_to_right_forearm = mocap_right_forearm_to_right_forearm_translation * mocap_right_forearm_to_right_forearm_rotation;
    // Make the Eigen transform for the right forearm->right wrist
    Eigen::Translation3d right_forearm_to_right_wrist_translation(forearm_to_wrist_transform.getOrigin().x(), forearm_to_wrist_transform.getOrigin().y(), forearm_to_wrist_transform.getOrigin().z());
    Eigen::Quaterniond right_forearm_to_right_wrist_rotation(forearm_to_wrist_transform.getRotation().w(), forearm_to_wrist_transform.getRotation().x(), forearm_to_wrist_transform.getRotation().y(), forearm_to_wrist_transform.getRotation().z());
    Pose right_forearm_to_right_wrist = right_forearm_to_right_wrist_translation * right_forearm_to_right_wrist_rotation;
    // Compose the transforms together
    Pose torso_to_right_wrist = torso_to_base * world_to_base_inv * world_to_right_mocap_forearm * mocap_right_forearm_to_right_forearm * right_forearm_to_right_wrist;
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
    // Publish as a TF frame
    /* Convert to a TF transform and publish */
    tf::Transform right_arm_tf;
    right_arm_tf.setOrigin(tf::Vector3(right_arm_pose.pose.position.x, right_arm_pose.pose.position.y, right_arm_pose.pose.position.z));
    right_arm_tf.setRotation(tf::Quaternion(right_arm_pose.pose.orientation.x, right_arm_pose.pose.orientation.y, right_arm_pose.pose.orientation.z, right_arm_pose.pose.orientation.w));
    /* Publish TF */
    tf::StampedTransform right_arm_tf_msg(right_arm_tf, torso_transform.stamp_, std::string("/torso_lift_link"), std::string("/mocap_right_wrist"));
    g_transform_broadcaster->sendTransform(right_arm_tf_msg);
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
    tf::StampedTransform torso_transform;
    bool got_torso = false;
    try
    {
        g_transform_listener->lookupTransform("/torso_lift_link", "/base_footprint", ros::Time(0), torso_transform);
        got_torso = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Unable to lookup transform from /torso_lift_link to /base_footprint");
    }
    // Query TF to get the transform from /l_forearm_link to /l_wrist_roll_link
    tf::StampedTransform left_forearm_to_wrist_transform;
    bool got_left_forearm = false;
    try
    {
        g_transform_listener->lookupTransform("/l_forearm_link", "/l_wrist_roll_link", ros::Time(0), left_forearm_to_wrist_transform);
        got_left_forearm = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Unable to lookup transform from /l_forearm_link to /l_wrist_roll_link");
    }
    // Query TF to get the transform from /r_forearm_link to /r_wrist_roll_link
    tf::StampedTransform right_forearm_to_wrist_transform;
    bool got_right_forearm = false;
    try
    {
        g_transform_listener->lookupTransform("/r_forearm_link", "/r_wrist_roll_link", ros::Time(0), right_forearm_to_wrist_transform);
        got_right_forearm = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Unable to lookup transform from /r_forearm_link to /r_wrist_roll_link");
    }
    // Publish what we can, depending on which transforms we've received
    if (got_base)
    {
        if (got_left_arm && got_left_forearm)
        {
            PublishLeftArmPose(torso_transform, left_forearm_to_wrist_transform, base_transform, left_arm_transform);
        }
        else
        {
            ROS_WARN("Unable to publish left arm pose");
        }
        if (got_right_arm && got_right_forearm)
        {
            PublishRightArmPose(torso_transform, right_forearm_to_wrist_transform, base_transform, right_arm_transform);
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
    nhp.param(std::string("left_arm_object"), g_left_arm_object, std::string("ArchieLeftForearm"));
    nhp.param(std::string("right_arm_object"), g_right_arm_object, std::string("ArchieRightForearm"));
    nhp.param(std::string("left_arm_segment"), g_left_arm_segment, std::string("ArchieLeftForearm"));
    nhp.param(std::string("right_arm_segment"), g_right_arm_segment, std::string("ArchieRightForearm"));
    // Create the pose publishers
    g_left_arm_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(left_arm_pose_topic, 1, false);
    g_right_arm_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(right_arm_pose_topic, 1, false);
    // Create the TF broadcaster
    g_transform_broadcaster = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
    // Create the TF listener
    g_transform_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener(nh, ros::Duration(20.0)));
    ROS_INFO("Waiting for a few seconds to fill the TF cache...");
    ros::Duration(2.5).sleep();
    // Create the mocap data subscriber
    ros::Subscriber mocap_sub = nh.subscribe(tracker_topic, 1, MocapMsgCB);
    // Start streaming data
    ROS_INFO("Streaming data...");
    ros::Rate spin_rate(200.0);
    while (ros::ok())
    {
        // Handle ROS stuff
        ros::spinOnce();
        // Sleep briefly
        spin_rate.sleep();
    }
    return 0;
}

