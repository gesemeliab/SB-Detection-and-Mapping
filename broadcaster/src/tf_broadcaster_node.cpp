/**
 * @file tf_broadcaster_node.cpp
 * @brief ROS node for broadcasting transform frames from rover odometry to camera
 * 
 * This node subscribes to ZED camera odometry and publishes TF transforms
 * for the coordinate frame hierarchy: world -> rover -> camera
 * 
 * @author Gesem Gudiño
 * @date April 2020
 * @license GPLv3
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <cmath>

/**
 * @class TFBroadcaster
 * @brief Manages coordinate frame transformations for the robotic system
 */
class TFBroadcaster {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    explicit TFBroadcaster(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), nh_private_(nh_private) {
        
        // Load parameters
        loadParameters();
        
        // Initialize subscriber
        odom_sub_ = nh_.subscribe(odom_topic_, 10, 
                                 &TFBroadcaster::odometryCallback, this);
        
        ROS_INFO("TF Broadcaster initialized");
        ROS_INFO("Rover frame: %s", rover_frame_.c_str());
        ROS_INFO("Camera frame: %s", camera_frame_.c_str());
        ROS_INFO("Listening to odometry on: %s", odom_topic_.c_str());
    }

private:
    // ROS interfaces
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // Configuration parameters
    std::string world_frame_;
    std::string rover_frame_;
    std::string camera_frame_;
    std::string odom_topic_;
    
    double rover_height_;
    double camera_offset_x_;
    double camera_offset_y_;
    double camera_offset_z_;
    double orientation_offset_;
    
    /**
     * @brief Load parameters from ROS parameter server
     */
    void loadParameters() {
        // Frame IDs
        nh_private_.param<std::string>("world_frame", world_frame_, "world");
        nh_private_.param<std::string>("rover_frame", rover_frame_, "rover");
        nh_private_.param<std::string>("camera_frame", camera_frame_, "camera");
        
        // Topic names
        nh_private_.param<std::string>("odom_topic", odom_topic_, 
                                      "/zedm/zed_node/odom");
        
        // Transform parameters
        nh_private_.param("rover_height", rover_height_, 0.13);
        nh_private_.param("camera_offset_x", camera_offset_x_, 0.0);
        nh_private_.param("camera_offset_y", camera_offset_y_, -0.21);
        nh_private_.param("camera_offset_z", camera_offset_z_, 0.26);
        nh_private_.param("orientation_offset", orientation_offset_, M_PI / 2.0);
        
        ROS_INFO("Loaded parameters:");
        ROS_INFO("  Rover height: %.3f m", rover_height_);
        ROS_INFO("  Camera offset: (%.3f, %.3f, %.3f) m", 
                 camera_offset_x_, camera_offset_y_, camera_offset_z_);
    }
    
    /**
     * @brief Callback for odometry messages
     * @param msg Odometry message containing robot pose
     */
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Get current timestamp
        ros::Time current_time = ros::Time::now();
        
        // Broadcast world -> rover transform
        broadcastWorldToRoverTransform(msg, current_time);
        
        // Broadcast rover -> camera transform
        broadcastRoverToCameraTransform(current_time);
    }
    
    /**
     * @brief Broadcast transform from world to rover frame
     * @param odom_msg Odometry message with rover pose
     * @param timestamp Current timestamp
     */
    void broadcastWorldToRoverTransform(const nav_msgs::Odometry::ConstPtr& odom_msg,
                                       const ros::Time& timestamp) {
        tf::Transform transform;
        
        // Set translation (add height offset for rover base)
        transform.setOrigin(tf::Vector3(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            rover_height_
        ));
        
        // Set rotation (apply orientation offset for coordinate system alignment)
        tf::Quaternion q;
        q.setRPY(0, 0, odom_msg->pose.pose.orientation.z + orientation_offset_);
        transform.setRotation(q);
        
        // Broadcast transform
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, timestamp, world_frame_, rover_frame_)
        );
    }
    
    /**
     * @brief Broadcast transform from rover to camera frame
     * @param timestamp Current timestamp
     */
    void broadcastRoverToCameraTransform(const ros::Time& timestamp) {
        tf::Transform transform;
        
        // Set translation (camera mounting position relative to rover base)
        transform.setOrigin(tf::Vector3(
            camera_offset_x_,
            camera_offset_y_,
            camera_offset_z_
        ));
        
        // Set rotation (identity - camera aligned with rover frame)
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        
        // Broadcast transform
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, timestamp, rover_frame_, camera_frame_)
        );
    }
};

/**
 * @brief Main entry point
 */
int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "tf_broadcaster_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    try {
        // Create broadcaster instance
        TFBroadcaster broadcaster(nh, nh_private);
        
        // Spin
        ROS_INFO("TF Broadcaster node running...");
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_FATAL("Fatal error in TF broadcaster: %s", e.what());
        return 1;
    }
    
    return 0;
}
