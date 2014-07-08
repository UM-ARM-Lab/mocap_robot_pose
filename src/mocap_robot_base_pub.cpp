#include <ros/ros.h>
#include <iostream>
#include <string>
#include <lightweight_vicon_bridge/MocapState.h>
#include <tf/transform_broadcaster.h>

std::unique_ptr<tf::TransformBroadcaster> g_tf_broadcaster;
std::string g_robot_object_name;
std::string g_robot_segment_name;

void MocapMsgCB(lightweight_vicon_bridge::MocapState msg)
{
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
                        /* Convert to a TF transform and publish */
                        tf::Transform current_tf;
                        current_tf.setOrigin(tf::Vector3(current_segment.transform.translation.x, current_segment.transform.translation.y, current_segment.transform.translation.z));
                        current_tf.setRotation(tf::Quaternion(current_segment.transform.rotation.x, current_segment.transform.rotation.y, current_segment.transform.rotation.z, current_segment.transform.rotation.w));
                        /* Publish TF */
                        tf::StampedTransform current_tf_msg(current_tf, msg.header.stamp, std::string("/odom_combined"), std::string("/base_footprint"));
                        g_tf_broadcaster->sendTransform(current_tf_msg);
                        return;
                    }
                    else
                    {
                        ROS_WARN("Receiving tracking for robot, but robot is occluded");
                    }
                }
            }
        }
    }
    ROS_WARN("Receiving mocap tracking messages, but robot is not tracked");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap_robot_base_pub");
    ROS_INFO("Starting mocap robot base pub...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string tracker_topic;
    nhp.param(std::string("tracker_topic"), tracker_topic, std::string("mocap_tracking"));
    nhp.param(std::string("robot_object_name"), g_robot_object_name, std::string("ArchieBase"));
    nhp.param(std::string("robot_segment_name"), g_robot_segment_name, std::string("ArchieBase"));
    // Create the TF broadcaster
    g_tf_broadcaster = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());
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

