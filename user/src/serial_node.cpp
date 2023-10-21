#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <time.h>
#include <ros/callback_queue.h>
#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>

using namespace std;

class serialControl
{

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    geometry_msgs::Twist vel_msg;

public:
    /*
     * 变量定义
     */
    // 各状态量定义
    int count;

    /*
     * 函数定义
     */
    serialControl() {}

    int serialListener();
};

int serialControl::serialListener()
{
    // 创建一个serial对象
    serial::Serial sp;
    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    // 设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    // 设置串口通信的波特率
    sp.setBaudrate(115200);
    // 串口设置timeout
    sp.setTimeout(to);

    try
    {
        // 打开串口
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    // 判断串口是否打开成功
    if (sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x = 0.0;

    count = 0;
    ros::Rate loop_rate(500);
    uint8_t buffer[1024];
    while (ros::ok())
    {
        // 获取缓冲区内的字节数
        size_t num = sp.available();
        if (num != 0)
        {
            memset(buffer, 0, sizeof(buffer));
            // 读出数据
            num = sp.read(buffer, num);

            // 1：移动到下一个点
            if (!strcmp((char*)buffer, "1"))
            {
                std::cout << "收到命令：移动到下一个点" << std::endl;
                n.setParam("NaviCtrlFlag", 1);
                n.setParam("aprilTagDetectFlag", 1);
                bool result = n.getParam("targetCount", count);
                strcpy((char*)buffer, std::to_string(count).c_str()); // reply cilent with "ok"
            }
            // 2：暂停
            else if (!strcmp((char*)buffer, "2"))
            {
                std::cout << "收到命令：立即停止" << std::endl;
                n.setParam("NaviCtrlFlag", 0);
                n.setParam("moveLinearFlag", 0);
                sleep(0.05);
                for (int i = 0; i < 10; i++)
                {
                    vel_msg.linear.x = 0.0;
                    vel_msg.angular.z = 0.2 - (i + 1) * 0.02;
                    pub.publish(vel_msg);
                    sleep(0.05);
                }
                bool result = n.getParam("targetCount", count);
                strcpy((char*)buffer, std::to_string(count).c_str());
            }
            // 3：继续
            else if (!strcmp((char*)buffer, "3"))
            {
                std::cout << "收到命令：继续移动" << std::endl;
                n.setParam("NaviCtrlFlag", 1);
                n.setParam("aprilTagDetectFlag", 1);
                bool result = n.getParam("targetCount", count);
                strcpy((char*)buffer, std::to_string(count).c_str()); // reply cilent with "ok"
            }
            // 4：查询
            else if (!strcmp((char*)buffer, "4"))
            {
                std::cout << "收到命令：查询位置" << std::endl;
                bool result = n.getParam("targetCount", count);
                strcpy((char*)buffer, std::to_string(count).c_str()); // reply cilent with "ok"
            }

            // 把数据发送回去
            sp.write(buffer, num);
        }
        loop_rate.sleep();
    }
        
    // 关闭串口
    sp.close();

    return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_port");
    serialControl s;
    int flag = s.serialListener();

    return 0;
}
