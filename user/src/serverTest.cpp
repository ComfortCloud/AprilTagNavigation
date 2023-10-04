#include <stdio.h>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <sys/fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <errno.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ros/ros.h>
using namespace std;
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "server_node");
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
    serveraddr.sin_port = htons(atoi(argv[1])); // specify port
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
    while (1)
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

        strcpy(buffer, "ok"); // reply cilent with "ok"
        if ((iret = send(clintfd, buffer, strlen(buffer), 0)) <= 0)
        {
            perror("send");
            break;
        }
        printf("send :%s\n", buffer);
    }
    // 6th close socket
    close(listenfd);
    close(clintfd);

    return 0;
}
