cmake_minimum_required(VERSION 3.0.2)
project(camera_lidar_gazebo_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    OpenCV
    image_transport
    roscpp
    cv_bridge
    std_msgs  
    sensor_msgs
    pcl_conversions     
    tf     
)


find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package( Eigen3 REQUIRED )
include_directories( EIGEN3_INCLUDE_DIR )

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sim_data_collector_node src/collectData.cpp)

target_link_libraries(sim_data_collector_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(tf_listener_node src/evaluateTF.cpp)

target_link_libraries(tf_listener_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
