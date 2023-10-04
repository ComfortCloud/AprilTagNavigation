#include <iostream>
#include <thread>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include <time.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"


// AprilTag定位信息订阅：回调函数
void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{

    for (std::vector<apriltag_ros::AprilTagDetection>::const_iterator it = msg->detections.begin(); it != msg->detections.end(); ++it)
    {
        tf::Quaternion q;
        tf::quaternionMsgToTF(it->pose.pose.pose.orientation, q);
        double roll, pitch, yaw;                      // 定义存储r\p\y的容器
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw); // 进行转换

        std::cout << "Tag ID:" << it->id[0] << " " << it->size[0] << " data: " << it->pose.pose.pose.position.x << " " << it->pose.pose.pose.position.y
                  << " " << it->pose.pose.pose.position.z << " " << it->pose.pose.pose.orientation.x << " "
                  << it->pose.pose.pose.orientation.y << " " << it->pose.pose.pose.orientation.z << " " << it->pose.pose.pose.orientation.w << std::endl;
        std::cout << "r p y: " << roll << " " << pitch << " " << yaw << std::endl;
    }
}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");
    
	//创建节点语柄
	ros::NodeHandle n;

	// 创建一个Subscriber
	ros::Subscriber sub=n.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 5, aprilTagCallback);

	// 循环等待回调函数
	ros::spin();

    return 0;
}
