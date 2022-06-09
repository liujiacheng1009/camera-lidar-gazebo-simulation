#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Subscriber lidar_sub;
ros::Subscriber  left_cam_sub, right_cam_sub;
std::mutex m_img_buf;

static bool lidar_flag = false;
static bool left_camera_flag = false;
static bool right_camera_flag = false;

void check_for_exit()
{
    if(lidar_flag&& left_camera_flag && right_camera_flag)
    {
        ros::shutdown();
        std::cout<< "lidar pcd and stereo images have saved !!!"<<std::endl;
    }
    
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr&  cloud_msg)
{   
    m_img_buf.lock();
    std::cout<<"got lidar 0..."<<std::endl;
    if(!lidar_flag)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        pcl::io::savePCDFileASCII("/home/jc/Documents/catkin_ws2/src/camera-lidar-gazebo-simulization/test_pcd.pcd", cloud);
        lidar_flag = true;
    }
    m_img_buf.unlock();
    check_for_exit();
}

void left_camera_callback(const sensor_msgs::ImageConstPtr& msg)
{
    m_img_buf.lock();
    std::cout<<"got image 0..."<<std::endl;
    if(!left_camera_flag)
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imwrite("/home/jc/Documents/catkin_ws2/src/camera-lidar-gazebo-simulization/left_image.png", img);
        left_camera_flag = true;
    }
    m_img_buf.unlock();
    check_for_exit();
}

void right_camera_callback(const sensor_msgs::ImageConstPtr& msg)
{
    m_img_buf.lock();
    std::cout<<"got image 1..."<<std::endl;
    if(!right_camera_flag)
    {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imwrite("/home/jc/Documents/catkin_ws2/src/camera-lidar-gazebo-simulization/right_image.png", img);
        right_camera_flag = true;
    }
    m_img_buf.unlock();
    check_for_exit();
}


int main(int argc,  char **argv)
{
    ros::init(argc, argv, "sim_data_collector_node");

    ros::NodeHandle nh("~");

    static const std::string lidar_topic = "/velodyne_points";
    static const std::string left_camera_topic = "/stereo_camera/left/image_rect_color";
    static const std::string right_camera_topic = "/stereo_camera/right/image_rect_color";

    lidar_sub = nh.subscribe(lidar_topic, 1, lidar_callback);
    left_cam_sub = nh.subscribe(left_camera_topic, 10, left_camera_callback);
    right_cam_sub = nh.subscribe(right_camera_topic, 10, right_camera_callback);


    ros::spin();
    return 0;
}
