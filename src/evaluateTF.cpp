#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ContactState.h"
#include "gazebo_msgs/ModelStates.h"
#include <sstream>

Eigen::Affine3d calcu_relative_trans(geometry_msgs::Pose& p1, geometry_msgs::Pose& p2)
{
    Eigen::Translation3d t1(p1.position.x, p1.position.y, p1.position.z);
    Eigen::Quaterniond q1(p1.orientation.w,p1.orientation.x, p1.orientation.y,  p1.orientation.z);
    Eigen::Affine3d aff1 = t1*q1.toRotationMatrix();
    Eigen::Translation3d t2(p2.position.x, p2.position.y, p2.position.z);
    Eigen::Quaterniond q2(p2.orientation.w,p2.orientation.x, p2.orientation.y,  p2.orientation.z);
    Eigen::Affine3d aff2 = t2*q2.toRotationMatrix();
    return aff1.inverse()*aff2;
}

void model_states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    geometry_msgs::Pose stereo_camera_pose = msg->pose[3];
    geometry_msgs::Pose lidar_pose = msg->pose[1];
    geometry_msgs::Pose target_pose = msg->pose[2];
    Eigen::Affine3d trans_target_to_stereo_camera = calcu_relative_trans(target_pose, stereo_camera_pose);
    std::cout<< stereo_camera_pose.position <<std::endl;
    std::cout<< target_pose.position <<std::endl;
    std::cout<< trans_target_to_stereo_camera.matrix() <<std::endl;
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "tf_listener_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/gazebo/model_states", 10, model_states_callback);
  ros::spin();

  return 0;
}
