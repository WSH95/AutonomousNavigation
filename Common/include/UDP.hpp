//
// Created by wsh on 2021/9/7.
//

#ifndef FUNCTIONAL_UDP_HPP
#define FUNCTIONAL_UDP_HPP

#include <unistd.h>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <cmath>

constexpr int UDPBUFFSIZE = 1024;

class SocketTools
{
public:
    SocketTools() = default;

    ~SocketTools() = default;

    static void setSockAddr(struct sockaddr_in &sockaddr, const std::string &ipaddr, int port)
    {
        memset(&sockaddr, 0, sizeof(sockaddr)); //数据初始化--清零
        sockaddr.sin_family = AF_INET; //设置为IP通信

        if (ipaddr.empty())
        {
            sockaddr.sin_addr.s_addr = INADDR_ANY;//服务器IP地址--允许连接到所有本地地址上
        }
        else
        {
            sockaddr.sin_addr.s_addr = inet_addr((char *) ipaddr.c_str());//服务器IP地址
        }

        sockaddr.sin_port = htons(port); //服务器端口号
    }

    static bool setSockFd(int &fd, const std::string &protocol)
    {
        if (protocol == "tcp")
        {
            fd = socket(AF_INET, SOCK_STREAM, 0);
        }
        else if (protocol == "udp")
        {
            fd = socket(AF_INET, SOCK_DGRAM, 0);
        }
        return (fd >= 0);
    }

    static float ConvertByte2Float(unsigned char *pByte)
    {
        float floatVariable;
        unsigned char i;
        void *pf;
        pf = &floatVariable;

        for (i = 0; i < 4; i++)
        {
            *((unsigned char *) pf + i) = *(pByte + i);
        }

        return floatVariable;
    }

    static float ConvertByte2Int(unsigned char *pByte)
    {
        int intVariable;
        unsigned char i;
        void *pf;
        pf = &intVariable;

        for (i = 0; i < 4; i++)
        {
            *((unsigned char *) pf + i) = *(pByte + i);
        }

        return intVariable;
    }
};


class Udp : public SocketTools
{
public:
    Udp() : SocketTools() {}

    ~Udp() = default;

    //period == 0: 阻塞 无超时
    bool init(const std::string &targetIP, int targetPort, int localPort, float period)
    {
        if (fd != 0)
        {
            printf("The network has been initialized.\n");
            return true;
        }

        /*创建服务器端套接字--IPv4协议，面向无连接通信，UDP协议*/
        if (!setSockFd(fd, "udp"))
        {
            perror("Socket error!\n");
            return false;
        }

        if (!targetIP.empty() && targetPort)
            setSockAddr(targetAddr, targetIP, targetPort);
        else
            tmpAddr = &targetAddr;

        // 设置超时 0 means block
        if (period > 1e-4)
        {
            struct timeval tv;
            tv.tv_sec = (long int) period;
            tv.tv_usec = (long int) (1e6 * std::fmod(period, 1.f));
            if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof(struct timeval)) < 0)
            {
                perror("Timeout setting error!\n");
                return false;
            }
        }

        int on = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) < 0)
        {
            perror("Reuse addr error!\n");
            return false;
        }

        if (localPort)
        {
            struct sockaddr_in localAddr;
            setSockAddr(localAddr, "", localPort);
            /*将套接字绑定到服务器的网络地址上*/
            if (bind(fd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0)
            {
                perror("Bind error\n");
                return false;
            }
        }

        return true;
    }

    int Receive(unsigned char *pRcvBuf)
    {
        int len;

        if (fd == 0)
        {
            perror("Client fd has been released before, receive failed!\n");
            return -1;
        }

        if ((len = recvfrom(fd, pRcvBuf, UDPBUFFSIZE, 0, (struct sockaddr *) tmpAddr, (socklen_t *) &sin_size)) < 0)
        {
//        perror("Recvfrom error!\n");
            return -1;
        }

        return len;
    }

    int Send(unsigned char *pSndBuf, int bufLen)
    {
        int len;

        if (fd == 0)
        {
            perror("Client fd has been released before, send failed!\n");
            return -1;
        }

        if ((len = sendto(fd, pSndBuf, bufLen, 0, (struct sockaddr *) &targetAddr, sizeof(targetAddr))) < 0)
        {
//        perror("Sendto error!\n");
            return -1;
        }

        return len;
    }

    int Send(const std::string &targetIP, int targetPort, unsigned char *pSndBuf, int bufLen)
    {
        int len;
        struct sockaddr_in targetAddr_;

        if (fd == 0)
        {
            perror("Server fd has been released before, send failed!\n");
            return -1;
        }

        setSockAddr(targetAddr_, targetIP, targetPort);

        if ((len = sendto(fd, pSndBuf, bufLen, 0, (struct sockaddr *) &targetAddr_, sizeof(targetAddr_))) < 0)
        {
//        perror("Sendto error!\n");
            return -1;
        }

        return len;
    }

    int SendBack(unsigned char *pSndBuf, int bufLen)
    {
        int len;

        if (fd == 0)
        {
            perror("Client fd has been released before, send failed!\n");
            return -1;
        }

        if ((len = sendto(fd, pSndBuf, bufLen, 0, (struct sockaddr *) &tmpAddr, sizeof(tmpAddr))) < 0)
        {
//        perror("Sendto error!\n");
            return -1;
        }

        return len;
    }

    void Release()
    {
        if (fd != 0)
        {
            close(fd);
            fd = 0;
        }
    }

private:
    int fd = 0;

    int sin_size = sizeof(struct sockaddr_in);

    struct sockaddr_in targetAddr;

    struct sockaddr_in *tmpAddr;
};

#endif //FUNCTIONAL_UDP_HPP
