//
// Created by wsh on 12/1/20.
//

#ifndef DOGMANAGER7_SOCKETTOOL_H
#define DOGMANAGER7_SOCKETTOOL_H

#define BUFFSIZE 1024

#include <unistd.h>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <cmath>
#include "LockCircleQueue.hpp"


class SocketTool
{
public:
    SocketTool() = default;

    ~SocketTool() = default;

    static void setSockAddr(struct sockaddr_in &sockaddr, const std::string &ipaddr, int port);

    static bool setSockFd(int &fd, const std::string &protocol);

protected:
    int sin_size = sizeof(struct sockaddr_in);

};

class UdpServer : public SocketTool
{
public:
    UdpServer() : SocketTool(){}

    ~UdpServer() = default;

    //period == 0: 阻塞 无超时
    bool init(int port, float period);

    int Receive(unsigned char *pRcvBuf);

    int Send(unsigned char *pSndBuf, int bufLen);

    int Send(const std::string &targetIP, int targetPort, unsigned char *pSndBuf, int bufLen);

    void Release();

private:
    int _serverFD = 0;
    int _serverPort;
    struct sockaddr_in _serverAddr;
    struct sockaddr_in _clientAddr;
    float _period;
    LockCircleQueue<struct sockaddr_in, 1> lock_clientAddr;
};

class UdpClient : public SocketTool
{
public:
    UdpClient() : SocketTool(){}

    ~UdpClient() = default;

    //period == 0: 阻塞 无超时
    bool init(const std::string &serverIP, int serverPort, float period);

    int Receive(unsigned char *pRcvBuf);

    int Send(unsigned char *pSndBuf, int bufLen);

    void Release();

private:
    int _clientFD = 0;
    std::string _serverIP;
    int _serverPort;
    struct sockaddr_in _serverAddr;
    float _period;

};


class TcpServer : public SocketTool
{
public:
    TcpServer() : SocketTool(){}

    ~TcpServer() = default;

    bool init(int port);

    bool waitConnect();

    int Receive(char *pRcvBuf);

    int Send(char *pSndBuf, int bufLen);

    void Release();

private:
    int _serverPort;
    int _serverFD = 0;
    int _clientFD = 0;
    struct sockaddr_in _serverAddr;
    struct sockaddr_in _clientAddr;
    bool _flag_connect = false;
};

class TcpClient : public SocketTool
{
public:
    TcpClient() : SocketTool(){}

    ~TcpClient() = default;

    bool init(const std::string &serverIP, int serverPort);

    bool Connect();

    int Receive(char *pRcvBuf);

    int Send(char *pSndBuf, int bufLen);

    void Release();

private:
    int _clientFD = 0;
    std::string _serverIP;
    int _serverPort;
    struct sockaddr_in _serverAddr;
    bool _flag_connect = false;
};

#endif //DOGMANAGER7_SOCKETTOOL_H
