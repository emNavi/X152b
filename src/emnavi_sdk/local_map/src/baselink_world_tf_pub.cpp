#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf_conversions/tf_eigen.h>
#include <Eigen/StdVector>

void pose_tfCallback(const nav_msgs::OdometryConstPtr &odom){
    Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
    tf::Quaternion tf_cam2body_q;
    tf::quaternionEigenToTF(cam2body_q,tf_cam2body_q);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  
  transform.setOrigin( tf::Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z) );
  transform.setRotation(tf_cam2body_q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));
  ROS_INFO("sendTransform");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/vins_fusion/odometry", 10, &pose_tfCallback);
  ros::spin();
  return 0;
};