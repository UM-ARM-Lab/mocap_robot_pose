#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <atomic>
#include <Eigen/Geometry>

class MocapRobotPoseMonitor
{
protected:

    ros::NodeHandle nh_;
    ros::Subscriber robot_odom_sub_;
    ros::Subscriber mocap_pose_sub_;
    ros::Publisher odom_2d_pub_;
    ros::Publisher odom_3d_pub_;
    bool offset_computed_;
    bool odom_received_;
    Eigen::Affine3d last_odom_pose_;
    Eigen::Affine3d pose_offset_;

public:

    MocapRobotPoseMonitor(ros::NodeHandle& nh, std::string& robot_odometry_in_topic, std::string& mocap_pose_in_topic, std::string& odom_2d_out_topic, std::string& odom_3d_out_topic) : nh_(nh)
    {
        offset_computed_ = false;
        odom_received_ = false;
        robot_odom_sub_ = nh_.subscribe(robot_odometry_in_topic, 1, &MocapRobotPoseMonitor::RobotOdometryCB, this);
        mocap_pose_sub_ = nh_.subscribe(mocap_pose_in_topic, 1, &MocapRobotPoseMonitor::MocapPoseCB, this);
        odom_2d_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_2d_out_topic, 1, true);
        odom_3d_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_3d_out_topic, 1, true);
    }

    ~MocapRobotPoseMonitor() {}

    void Loop()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }

    void RobotOdometryCB(nav_msgs::Odometry robot_odom_msg)
    {
        /* Save the odometry message */
        /* Convert the odometry pose to Eigen */
        Eigen::Translation3d translation(robot_odom_msg.pose.pose.position.x, robot_odom_msg.pose.pose.position.y, robot_odom_msg.pose.pose.position.z);
        Eigen::Quaterniond rotation(robot_odom_msg.pose.pose.orientation.w, robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y, robot_odom_msg.pose.pose.orientation.z);
        Eigen::Affine3d odom_transform = translation * rotation;
        /* Save */
        last_odom_pose_ = odom_transform;
        odom_received_ = true;
        /* If we have a stored odometry->mocap pose offset, we combine
           with the pose reported by the robot's odometry system. */
        if (offset_computed_)
        {
            /* Apply the offset */
            Eigen::Affine3d odom_with_offset = pose_offset_ * odom_transform;
            Eigen::Quaterniond odom_with_offset_rotation(odom_with_offset.rotation());
            /* Convert the mocap pose into a nav_msgs::Odometry message. */
            nav_msgs::Odometry robot_odom;
            robot_odom.header.frame_id = std::string("");
            robot_odom.header.stamp = robot_odom_msg.header.stamp;
            /* Set the pose */
            robot_odom.pose.pose.position.x = odom_with_offset.translation().x();
            robot_odom.pose.pose.position.y = odom_with_offset.translation().y();
            robot_odom.pose.pose.position.z = odom_with_offset.translation().z();
            robot_odom.pose.pose.orientation.x = odom_with_offset_rotation.x();
            robot_odom.pose.pose.orientation.y = odom_with_offset_rotation.y();
            robot_odom.pose.pose.orientation.z = odom_with_offset_rotation.z();
            robot_odom.pose.pose.orientation.w = odom_with_offset_rotation.w();
            /* Set the covariance matrix */
            robot_odom.pose.covariance = robot_odom_msg.pose.covariance;
            /* Publish */
            odom_2d_pub_.publish(robot_odom);
        }
        /* If we don't have a stored odometry->mocap pose offset, we
           just publish the received odometry pose with a warning. */
        else
        {
            ROS_WARN("Publishing received odometry pose with no offset, since offset has not been computed yet");
            odom_2d_pub_.publish(robot_odom_msg);
        }
    }

    void MocapPoseCB(geometry_msgs::TransformStamped mocap_pose_msg)
    {
        /* If we have odometry data, compute the odometry->mocap pose offset. */
        if (odom_received_)
        {
            /* Convert the mocap pose to Eigen */
            Eigen::Translation3d mocap_translation(mocap_pose_msg.transform.translation.x, mocap_pose_msg.transform.translation.y, mocap_pose_msg.transform.translation.z);
            Eigen::Quaterniond mocap_rotation(mocap_pose_msg.transform.rotation.w, mocap_pose_msg.transform.rotation.x, mocap_pose_msg.transform.rotation.y, mocap_pose_msg.transform.rotation.z);
            Eigen::Affine3d mocap_transform = mocap_translation * mocap_rotation;
            /* Compute the offset */
            Eigen::Affine3d odom_transform = last_odom_pose_;
            pose_offset_ = mocap_transform * odom_transform.inverse();
            if (!offset_computed_)
            {
                ROS_INFO("Computed first pose offset from mocap data");
            }
            offset_computed_ = true;
        }
        else
        {
            ROS_WARN("Mocap data received before odometry data, offset will be computed when odometry data is available");
        }
        /* Convert the mocap pose into a nav_msgs::Odometry message. */
        nav_msgs::Odometry mocap_odom;
        mocap_odom.header.frame_id = std::string("");
        mocap_odom.header.stamp = mocap_pose_msg.header.stamp;
        /* Set the pose */
        mocap_odom.pose.pose.position.x = mocap_pose_msg.transform.translation.x;
        mocap_odom.pose.pose.position.y = mocap_pose_msg.transform.translation.y;
        mocap_odom.pose.pose.position.z = mocap_pose_msg.transform.translation.z;
        mocap_odom.pose.pose.orientation.x = mocap_pose_msg.transform.rotation.x;
        mocap_odom.pose.pose.orientation.y = mocap_pose_msg.transform.rotation.y;
        mocap_odom.pose.pose.orientation.z = mocap_pose_msg.transform.rotation.z;
        mocap_odom.pose.pose.orientation.w = mocap_pose_msg.transform.rotation.w;
        /* Set the covariance matrix */
        /* Row 1 */
        mocap_odom.pose.covariance[0] = 1e-12;
        mocap_odom.pose.covariance[1] = 0.0;
        mocap_odom.pose.covariance[2] = 0.0;
        mocap_odom.pose.covariance[3] = 0.0;
        mocap_odom.pose.covariance[4] = 0.0;
        mocap_odom.pose.covariance[5] = 0.0;
        /* Row 2 */
        mocap_odom.pose.covariance[6] = 0.0;
        mocap_odom.pose.covariance[7] = 1e-12;
        mocap_odom.pose.covariance[8] = 0.0;
        mocap_odom.pose.covariance[9] = 0.0;
        mocap_odom.pose.covariance[10] = 0.0;
        mocap_odom.pose.covariance[11] = 0.0;
        /* Row 3 */
        mocap_odom.pose.covariance[12] = 0.0;
        mocap_odom.pose.covariance[13] = 0.0;
        mocap_odom.pose.covariance[14] = 1e-12;
        mocap_odom.pose.covariance[15] = 0.0;
        mocap_odom.pose.covariance[16] = 0.0;
        mocap_odom.pose.covariance[17] = 0.0;
        /* Row 4 */
        mocap_odom.pose.covariance[18] = 0.0;
        mocap_odom.pose.covariance[19] = 0.0;
        mocap_odom.pose.covariance[20] = 0.0;
        mocap_odom.pose.covariance[21] = 1e-12;
        mocap_odom.pose.covariance[22] = 0.0;
        mocap_odom.pose.covariance[23] = 0.0;
        /* Row 5 */
        mocap_odom.pose.covariance[24] = 0.0;
        mocap_odom.pose.covariance[25] = 0.0;
        mocap_odom.pose.covariance[26] = 0.0;
        mocap_odom.pose.covariance[27] = 0.0;
        mocap_odom.pose.covariance[28] = 1e-12;
        mocap_odom.pose.covariance[29] = 0.0;
        /* Row 6 */
        mocap_odom.pose.covariance[30] = 0.0;
        mocap_odom.pose.covariance[31] = 0.0;
        mocap_odom.pose.covariance[32] = 0.0;
        mocap_odom.pose.covariance[33] = 0.0;
        mocap_odom.pose.covariance[34] = 0.0;
        mocap_odom.pose.covariance[35] = 1e-12;
        /* Publish */
        odom_3d_pub_.publish(mocap_odom);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mocap_robot_pose_node");
    ROS_INFO("Starting mocap robot pose monitor...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("`");
    std::string robot_odometry_in_topic;
    std::string mocap_pose_in_topic;
    std::string odom_2d_out_topic;
    std::string odom_3d_out_topic;
    nhp.param(std::string("odometry_topic"), robot_odometry_in_topic, std::string("odometry"));
    nhp.param(std::string("mocap_topic"), mocap_pose_in_topic, std::string("mocap"));
    nhp.param(std::string("odom_2d_topic"), odom_2d_out_topic, std::string("odom_2d"));
    nhp.param(std::string("odom_3d_topic"), odom_3d_out_topic, std::string("odom_3d"));
    MocapRobotPoseMonitor monitor(nh, robot_odometry_in_topic, mocap_pose_in_topic, odom_2d_out_topic, odom_3d_out_topic);
    ROS_INFO("...startup complete");
    monitor.Loop();
    return 0;
}
