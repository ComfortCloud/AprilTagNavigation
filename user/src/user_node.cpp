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

/*
 * 为实现当前ROS节点多线程运行，且各线程之间可以方便地共享数据，故使用类中创建多线程
 */
class moveControl
{

private:
    ros::NodeHandle n;
    ros::NodeHandle n_robot;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Publisher pub1;
    ros::Publisher pub2;
    geometry_msgs::Twist vel_msg;

public:
    /*
     * 变量定义
     */
    // 计数：用于确认当前定位步骤所用的aprilTag编号
    int count = 0;

    // Tag相对于cam的位姿记录
    Eigen::Vector3d tag2CamPosition;
    Eigen::Vector4d tag2CamPose;
    Eigen::Matrix4d tag2CamT;

    // 当前tag相对机器人位姿记录
    Eigen::Matrix4d tag2RobotT;
    Eigen::Matrix3d tag2RobotR;

    // 相机标定数据记录
    Eigen::Vector3d cTag2CamPosition;
    Eigen::Vector3d cTag2CamPose;
    Eigen::Matrix4d cTag2CamT;
    Eigen::Matrix4d cam2RobotT;
    Eigen::Matrix4d calibrationT;

    // 机器人odom位姿定义
    // 本次导航机器人目标位姿在base坐标系下的表示
    Eigen::Vector3d targetRobot2DPos;
    double targetDis;

    // 本次导航机器人起始位姿在{0}坐标系下的表示
    Eigen::Matrix4d baseRobotPos;
    Eigen::Vector3d baseRobot2DPos;

    // 各状态量定义
    int aprilTagDetectFlag;
    int NaviCtrlFlag;
    int odomUpdateFlag = 0;
    int moveLinearFlag = 0;
    int waitingFlag = 1;

    /*
     * 函数定义
     */
    moveControl() {}

    //void keyBoardListener();
    void startRunning();
    void aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    void tracerCallback(const nav_msgs::Odometry::ConstPtr &msg);
};


/* ROS初始化：spin()线程 */
void moveControl::startRunning()
{

    /* 计算相机在机器人坐标系下 标定的 齐次变换矩阵 */
    // 预先标定aprilTag到机器人距离为1m(x方向)
    calibrationT << 1, 0, 0, 0.7945,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
   
    
    cTag2CamPosition << -0.071291, -0.186241, 1.66887;
    cTag2CamPose << 3.086, -0.189099, -1.5276; //rpy

    // 计算标签在相机坐标系下的齐次变换矩阵
    cTag2CamT = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d cTag2CamR;
    //Eigen::Quaterniond cTag2CamQ(cTag2CamPose);
    //cTag2CamR = cTag2CamQ.toRotationMatrix();

    cTag2CamR = Eigen::AngleAxisd(cTag2CamPose(2), Eigen::Vector3d::UnitZ()) * 
                Eigen::AngleAxisd(cTag2CamPose(1), Eigen::Vector3d::UnitY()) * 
                Eigen::AngleAxisd(cTag2CamPose(0), Eigen::Vector3d::UnitX());
    
    cTag2CamT.block(0, 0, 3, 3) = cTag2CamR;
    cTag2CamT.block(0, 3, 3, 1) = cTag2CamPosition;

    // 计算相机在机器人坐标系下的位姿
    cam2RobotT = calibrationT * cTag2CamT.inverse();
    std::cout << cam2RobotT << std::endl;

    // 给定机器人base初始值：与{0}坐标系重合
    baseRobotPos = Eigen::Matrix4d::Identity();

    /* 初始化ROS订阅者和发布者 */
    n.setParam("NaviCtrlFlag",1);
    n.setParam("aprilTagDetectFlag",1);
    n.setParam("odomUpdateFlag",0);
    n.setParam("moveLinearFlag",0);
    sub1 = n.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 5, &moveControl::aprilTagCallback, this);
    sub2 = n.subscribe<nav_msgs::Odometry>("/odom", 5, &moveControl::tracerCallback, this);
    pub1 = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   
    /* 开始监听 */
    ros::spin();
}

/* AprilTag定位信息订阅：回调函数 */
void moveControl::aprilTagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg)
{
    bool result0 = n.getParam("NaviCtrlFlag", NaviCtrlFlag);
    bool result1 = n.getParam("aprilTagDetectFlag", aprilTagDetectFlag);
    if (NaviCtrlFlag == 1)
    {
        if (aprilTagDetectFlag)
        {
            /* 转存数据 */
            if (msg->detections[0].id[0] == count)
            {
                for (std::vector<apriltag_ros::AprilTagDetection>::const_iterator it = msg->detections.begin(); it != msg->detections.end(); ++it)
                {
                    std::cout << "INFO: Received " << it->id[0] << std::endl;
                    tag2CamPosition(0) = it->pose.pose.pose.position.x;
                    tag2CamPosition(1) = it->pose.pose.pose.position.y;
                    tag2CamPosition(2) = it->pose.pose.pose.position.z;
                    tag2CamPose(0) = it->pose.pose.pose.orientation.x;
                    tag2CamPose(1) = it->pose.pose.pose.orientation.y;
                    tag2CamPose(2) = it->pose.pose.pose.orientation.z;
                    tag2CamPose(3) = it->pose.pose.pose.orientation.w;
                    break;
                }

                /* 计算aprilTag在机器人坐标系下的齐次变换矩阵 */
                tag2CamT = Eigen::Matrix4d::Identity();
                Eigen::Quaterniond tag2CamQ(tag2CamPose);
                Eigen::Matrix3d tag2CamR;
                tag2CamR = tag2CamQ.toRotationMatrix();
                tag2CamT.block(0, 0, 3, 3) = tag2CamR;
                tag2CamT.block(0, 3, 3, 1) = tag2CamPosition;
                tag2RobotT = cam2RobotT * tag2CamT;

                // std::cout << "tag2Robot: " << tag2RobotT << std::endl;

                /* 确定机器人相对于当前需要到达AprilTag标记位置所要行进的方向 */
                Eigen::Matrix3d tag2RobotR = tag2RobotT.block(0, 0, 3, 3);
                Eigen::Vector3d euler_angles = tag2RobotR.eulerAngles(2, 1, 0);
                // std::cout << "yaw(z) pitch(y) roll(x) = " << euler_angles.transpose() << std::endl;
                double ytemp = tag2RobotT(1, 3);

                /* 机器人旋转调整位姿，直到tag在robot坐标系下y坐标为0 */
                // std::cout << "ouler_angles: " << euler_angles(0) << std::endl;
                targetDis = sqrt(pow(tag2RobotT(0, 3), 2) + pow(tag2RobotT(1, 3), 2));
                std::cout << "ytemp: " << ytemp << std::endl;
                std::cout << "targetDis: " << targetDis << std::endl;
                std::cout << "angle z: " << euler_angles(0) << std::endl;
                if (abs(ytemp) / targetDis < 0.0001)
                {
                    n.setParam("aprilTagDetectFlag", 0);
                    /* 确定机器人相对于当前需要到达AprilTag标记位置所要行进的距离 */
                    n.setParam("moveLinearFlag", 1);
                    vel_msg.angular.z = 0.0;
                    vel_msg.linear.x = 0.08;
                    pub1.publish(vel_msg);
                }
                else
                {
                    if (ytemp > 0)
                    {
                        vel_msg.angular.z = 0.04;
                        vel_msg.linear.x = 0.0;
                    }
                    else
                    {
                        vel_msg.angular.z = -0.04;
                        vel_msg.linear.x = 0.0;
                    }
                    pub1.publish(vel_msg);
                }
            }
        }
    }
}

/* 机器人里程计信息订阅：回调函数 */
void moveControl::tracerCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
    bool result0 = n.getParam("NaviCtrlFlag", NaviCtrlFlag);
    bool result2 = n.getParam("moveLinearFlag", moveLinearFlag);
    if (NaviCtrlFlag == 1)
    {
        if (moveLinearFlag)
        {
            // 整理odom数据为机器人当前里程计相对{0}坐标系的齐次变换矩阵
            Eigen::Matrix4d nowRobotPos;
            Eigen::Matrix4d robotPosT;
            Eigen::Matrix3d robotPosR;
            robotPosT = Eigen::Matrix4d::Identity();
            Eigen::Vector4d robot3DPose;
            robotPosT(0, 3) = odom->pose.pose.position.x;
            robotPosT(1, 3) = odom->pose.pose.position.y;
            robotPosT(2, 3) = odom->pose.pose.position.z;
            robot3DPose(0) = odom->pose.pose.orientation.x;
            robot3DPose(1) = odom->pose.pose.orientation.y;
            robot3DPose(2) = odom->pose.pose.orientation.z;
            robot3DPose(3) = odom->pose.pose.orientation.w;

            /* 计算aprilTag在机器人坐标系下的齐次变换矩阵 */
            Eigen::Quaterniond robot3DQ(robot3DPose);
            robotPosR = robot3DQ.toRotationMatrix();
            robotPosT.block(0, 0, 3, 3) = robotPosR;
            nowRobotPos = baseRobotPos.inverse() * robotPosT;

            // 计算已经移动的距离
            Eigen::Vector3d nowRobot2DPos;
            Eigen::Matrix3d nowRobotR = nowRobotPos.block(0, 0, 3, 3);
            Eigen::Vector3d euler_angles = nowRobotR.eulerAngles(2, 1, 0);
            nowRobot2DPos(0) = nowRobotPos(0, 3);
            nowRobot2DPos(1) = nowRobotPos(1, 3);
            nowRobot2DPos(2) = euler_angles(0);

            // 判断是否运动到指定地点
            double diff, distance;
            // diff = sqrt(pow(targetRobotPos(0) - nowRobot2DPos(0), 2) + pow(targetRobotPos(1) - nowRobot2DPos(1), 2));
            distance = sqrt(pow(nowRobot2DPos(0), 2) + pow(nowRobot2DPos(1), 2));

            // std::cout << "robot odom: " << robotPosT << std::endl;
            // std::cout << "distance: " << distance << std::endl;

            pub1.publish(vel_msg);

            // 若到达tag点，机器人停止运动
            if (distance - targetDis > 0)
            {
                std::cout << "Robot arrives at position " << count << std::endl;

                // 发布机器人停止消息
                vel_msg.linear.x = 0.0;
                vel_msg.angular.z = 0.0;
                pub1.publish(vel_msg);

                // 允许更新base坐标系
                n.setParam("odomUpdateFlag",1);
                bool result3 = n.getParam("odomUpdateFlag", odomUpdateFlag);
                // 判断是否需要更新base坐标系，当机器人运动到目标标签处更新
                if (odomUpdateFlag)
                {
                    std::cout << "INFO: Update base robot pose." << std::endl;
                    baseRobotPos = robotPosT;
                    n.setParam("moveLinearFlag",0);
                    n.setParam("odomUpdateFlag",0);
                }
            }
        }
    }
}

/* 主函数 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_node");
    moveControl obj;
    // std::thread t1(&moveControl::keyBoardListener, &obj);
    obj.startRunning();
    // t1.join();

    ROS_INFO("Quit robot navigation.");
    return 0;
}




// /* 键盘监听线程 */
// void moveControl::keyBoardListener()
// {
//     int ch;
//     while (1)
//     {
//         ch = getchar();
//         if (waitingFlag)
//         {
//             if (ch == 65)
//             {
//                 if (count == 36)
//                 {
//                     std::cout << "This is the last tag." << std::endl;
//                     waitingFlag = 0;
//                     break;
//                 }
//                 count = count + 12;
//                 aprilTagDetectFlag = 1;
//                 std::cout << "Now detect tag: " << count << std::endl;
//             }
//             else
//             {
//                 std::cout << "Wrong input!" << std::endl;
//             }
//         }
//         if (ch == 66)
//         {
//             std::cout << "Break out." << std::endl;
//             break;
//         }
//     }
// }
