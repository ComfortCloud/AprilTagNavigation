#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <iostream>
#include <ros/ros.h>
using namespace std;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "client_node");
    // first step create socket
    int sockfd;
    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        return -1;
    }

    // second step: send request to server for connection
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(49200);                                 // server's port
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");                            // server's ip
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0) // send request to server for connection
    {
        perror("connect");
        close(sockfd);
        return -1;
    }

    char buffer[1024];

    // 3th step connect with server
    while (1)
    {
        int iret;
        memset(buffer, 0, sizeof(buffer));
        int choice;
        printf("if you want to continue to chat please input 1 or input else\n");
        scanf("%d", &choice);
        getchar();
        if (choice != 1)
            break;
        else
        {
            printf("please input what you want to say\n");
            cin.getline(buffer, 1024);
        }

        if ((iret = send(sockfd, buffer, strlen(buffer), 0)) <= 0) // send data to server
        {
            perror("send");
            break;
        }
        printf("send: %s\n", buffer);

        memset(buffer, 0, sizeof(buffer));
        if ((iret = recv(sockfd, buffer, sizeof(buffer), 0)) <= 0) // receive server's reply
        {
            printf("iret=%d\n", iret);
            break;
        }
        printf("receive: %s\n", buffer);
    }
    close(sockfd);

    return 0;
}
