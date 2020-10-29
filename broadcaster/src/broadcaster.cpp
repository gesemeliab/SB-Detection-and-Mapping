#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

std::string rover_name;



void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Transform transform_2;
  transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.13) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->pose.pose.orientation.z+3.1416/2);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", rover_name));

  transform_2.setOrigin( tf::Vector3(0.0, -0.21, 0.26) );
  tf::Quaternion q_2;
  q_2.setRPY(0, 0, 0);
  transform_2.setRotation(q_2);
  br.sendTransform(tf::StampedTransform(transform_2, ros::Time::now(), rover_name, "camera"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need rover name as argument"); return -1;};
  rover_name = argv[1];
  //rover_name = "rover";

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/zedm/zed_node/Odometry", 10, &poseCallback);

  ros::spin();
  return 0;
};
