#include <stdio.h>
#include <iostream>
#include <cstring>
#include <string>
#include <vector>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <errno.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <cmath>
#include <iomanip>
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

class socketControl
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
    socketControl() {}

    int socketListener();
};


int socketControl::socketListener()
{
    // first step ->create socket for server
    int listenfd;
    listenfd = socket(AF_INET, SOCK_STREAM, 0); // in socket code,it must be AF_INET(protocol)
    if (listenfd == -1)
    {
        printf("socket create fail\n");
        return -1;
    }

    // second step bind ->server's ip&port for communication to socket created in fist step
    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    // serveraddr.sin_addr.s_addr=atoi(argv[1]);// specify ip address
    serveraddr.sin_port = htons(8888); // specify port
    // printf("%s %s\n",argv[1],argv[2]);
    if (bind(listenfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) != 0)
    {
        printf("bind failed \n");
        return -1;
    }

    // Third step ->Set socket to listening mode
    /*
    The listen function changes the active connection socket interface into the connected socket interface,
    so that a process can accept the requests of other processes and become a server process.
    In TCP server programming, the listen function changes the process into a server and specifies that the corresponding socket becomes a passive connection.
    */
    if (listen(listenfd, 5) != 0)
    {
        printf("Listen failed\n");
        close(listenfd);
        return -1;
    }

    // 4th step -> receive client's request
    int clintfd; // socket for client
    int socklen = sizeof(struct sockaddr_in);
    struct sockaddr_in client_addr;
    clintfd = accept(listenfd, (struct sockaddr *)&client_addr, (socklen_t *)&socklen);
    if (clintfd == -1)
        printf("connect failed\n");
    else
        printf("client %s has connnected\n", inet_ntoa(client_addr.sin_addr));

    // 5th step ->connect with client,receive data and reply OK
    char buffer[1024];
    int command;

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vel_msg.angular.z = 0.0;
    vel_msg.linear.x = 0.0;

    count = 0;


    while (ros::ok())
    {
        int iret;
        memset(buffer, 0, sizeof(buffer));
        iret = recv(clintfd, buffer, sizeof(buffer), 0);
        if (iret <= 0)
        {
            printf("iret=%d\n", iret);
            break;
        }

        printf("receive :%s\n", buffer);
        const char *buffer1 = buffer;

        // 1：移动到下一个点
        if (!strcmp(buffer1, "1"))
        {
            std::cout << "收到命令：移动到下一个点" << std::endl;
            n.setParam("NaviCtrlFlag", 1);
            n.setParam("aprilTagDetectFlag", 1);
            bool result = n.getParam("targetCount", count);
            strcpy(buffer, std::to_string(count).c_str()); // reply cilent with "ok"
        }
        // 2：暂停
        else if(!strcmp(buffer1, "2"))
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
                sleep(0.02);
            }
            bool result = n.getParam("targetCount", count);
            strcpy(buffer, std::to_string(count).c_str());
        }
        // 3：继续
        else if(!strcmp(buffer1, "3"))
        {
            std::cout << "收到命令：继续移动" << std::endl;
            n.setParam("NaviCtrlFlag", 1);
            n.setParam("aprilTagDetectFlag", 1);
            bool result = n.getParam("targetCount", count);
            strcpy(buffer, std::to_string(count).c_str()); // reply cilent with "ok"
        }
        // 4：查询
        else if(!strcmp(buffer1, "4"))
        {
            std::cout << "收到命令：查询位置" << std::endl;
            bool result = n.getParam("targetCount", count);
            strcpy(buffer, std::to_string(count).c_str()); // reply cilent with "ok"
        }

        if ((iret = send(clintfd, buffer, strlen(buffer), 0)) <= 0)
        {
            perror("send");
            break;
        }
    }
    // 6th close socket
    close(listenfd);
    close(clintfd);
    return 0;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "socket_node");
    socketControl s;
    int flag = s.socketListener();

    return 0;
}
