//
// Created by wsh on 12/1/20.
//

#include "SocketTool.h"

void SocketTool::setSockAddr(struct sockaddr_in &sockaddr, const std::string &ipaddr, int port)
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

bool SocketTool::setSockFd(int &fd, const std::string &protocol)
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

bool UdpServer::init(int port, float period)
{
    _serverPort = port;
    _period = period;
    if (_serverFD != 0)
    {
        printf("The server has been initialized.\n");
        return true;
    }

    setSockAddr(_serverAddr, "", _serverPort);

    /*创建服务器端套接字--IPv4协议，面向无连接通信，UDP协议*/
    if (!setSockFd(_serverFD, "udp"))
    {
        perror("Socket error!\n");
        return false;
    }

    // 设置超时
    if (_period != 0)
    {
        struct timeval tv;
        tv.tv_sec = (long int) _period;
        tv.tv_usec = (long int) (1e6 * std::fmod(_period, 1.f));
        if (setsockopt(_serverFD, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof(struct timeval)) < 0)
        {
            perror("Timeout setting error!\n");
            return false;
        }
    }

    int on = 1;
    if (setsockopt(_serverFD, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) < 0)
    {
        perror("Reuse addr error!\n");
        return false;
    }

    /*将套接字绑定到服务器的网络地址上*/
    if (bind(_serverFD, (struct sockaddr *) &_serverAddr, sizeof(_serverAddr)) < 0)
    {
        perror("Bind error\n");
        return false;
    }

    return true;
}

int UdpServer::Receive(unsigned char *pRcvBuf)
{
    int len;

    if (_serverFD == 0)
    {
        perror("Server fd has been released before, receive failed!\n");
        return -1;
    }

    if ((len = recvfrom(_serverFD, pRcvBuf, BUFFSIZE, 0, (struct sockaddr *) &_clientAddr, (socklen_t *) &sin_size)) <
        0)
    {
//        perror("Recvfrom error!\n");
        return -1;
    }

    lock_clientAddr.push(_clientAddr);

    return len;
}

int UdpServer::Send(unsigned char *pSndBuf, int bufLen)
{
    int len;
    struct sockaddr_in targetAddr;
    lock_clientAddr.pop_wait(&targetAddr);

    if (_serverFD == 0)
    {
        perror("Server fd has been released before, send failed!\n");
        return -1;
    }

    if ((len = sendto(_serverFD, pSndBuf, bufLen, 0, (struct sockaddr *) &targetAddr, sizeof(targetAddr))) < 0)
    {
//        perror("Sendto error!\n");
        return -1;
    }

    return len;
}

int UdpServer::Send(const std::string &targetIP, int targetPort, unsigned char *pSndBuf, int bufLen)
{
    int len;
    struct sockaddr_in targetAddr;

    setSockAddr(targetAddr, targetIP, targetPort);

    if (_serverFD == 0)
    {
        perror("Server fd has been released before, send failed!\n");
        return -1;
    }

    if ((len = sendto(_serverFD, pSndBuf, bufLen, 0, (struct sockaddr *) &targetAddr, sizeof(targetAddr))) < 0)
    {
//        perror("Sendto error!\n");
        return -1;
    }

    return len;
}

void UdpServer::Release()
{
    if (_serverFD != 0)
    {
        close(_serverFD);
        _serverFD = 0;
    }
}

bool UdpClient::init(const std::string &serverIP, int serverPort, float period)
{
    _serverIP = serverIP;
    _serverPort = serverPort;
    _period = period;
    if (_clientFD != 0)
    {
        printf("The client has been initialized.\n");
        return true;
    }

    setSockAddr(_serverAddr, _serverIP, _serverPort);

    /*创建客户端套接字--IPv4协议，面向无连接通信，UDP协议*/
    if (!setSockFd(_clientFD, "udp"))
    {
        perror("Socket error!\n");
        return false;
    }

    // 设置超时
    if (_period != 0)
    {
        struct timeval tv;
        tv.tv_sec = (long int) _period;
        tv.tv_usec = (long int) (1e6 * std::fmod(_period, 1.f));
        if (setsockopt(_clientFD, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof(struct timeval)) < 0)
        {
            perror("Timeout setting error!\n");
            return false;
        }
    }

    return true;
}

int UdpClient::Receive(unsigned char *pRcvBuf)
{
    int len;

    if (_clientFD == 0)
    {
        perror("Client fd has been released before, receive failed!\n");
        return -1;
    }

    if ((len = recvfrom(_clientFD, pRcvBuf, BUFFSIZE, 0, (struct sockaddr *) &_serverAddr, (socklen_t *) &sin_size)) <
        0)
    {
//        perror("Recvfrom error!\n");
        return -1;
    }

    return len;
}

int UdpClient::Send(unsigned char *pSndBuf, int bufLen)
{
    int len;

    if (_clientFD == 0)
    {
        perror("Client fd has been released before, send failed!\n");
        return -1;
    }

    if ((len = sendto(_clientFD, pSndBuf, bufLen, 0, (struct sockaddr *) &_serverAddr, sizeof(_serverAddr))) < 0)
    {
//        perror("Sendto error!\n");
        return -1;
    }

    return len;
}

void UdpClient::Release()
{
    if (_clientFD != 0)
    {
        close(_clientFD);
        _clientFD = 0;
    }
}

bool TcpServer::init(int port)
{
    _serverPort = port;

    if (_serverFD != 0)
    {
        printf("The server has been initialized.\n");
        return true;
    }

    setSockAddr(_serverAddr, "", _serverPort);

    /*创建服务器端套接字--IPv4协议，面向无连接通信，UDP协议*/
    if (!setSockFd(_serverFD, "tcp"))
    {
        perror("Socket error!\n");
        return false;
    }

    int on = 1;
    if (setsockopt(_serverFD, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int)) < 0)
    {
        perror("Reuse addr error!\n");
        return false;
    }

    /*将套接字绑定到服务器的网络地址上*/
    if (bind(_serverFD, (struct sockaddr *) &_serverAddr, sizeof(_serverAddr)) < 0)
    {
        perror("Bind error\n");
        return false;
    }

    /*监听连接请求--监听队列长度为2*/
    if (listen(_serverFD, 2) < 0)
    {
        perror("Listen error\n");
        return false;
    }

    return true;
}

bool TcpServer::waitConnect()
{
    if ((_clientFD = accept(_serverFD, (struct sockaddr *) &_clientAddr, (socklen_t *) &sin_size)) < 0)
    {
        perror("Accept error\n");
        return false;
    }
    printf("Accept client %s\n", inet_ntoa(_clientAddr.sin_addr));
    send(_clientFD, "Welcome to the Dog Manager server!\n", 35, 0);

    _flag_connect = true;
    return true;
}

int TcpServer::Receive(char *pRcvBuf)
{
    int len;

    if (_clientFD == 0)
    {
        perror("Not connected yet or client fd has been released before, receive failed!\n");
        return -2;
    }

    len = recv(_clientFD, pRcvBuf, BUFFSIZE, 0);

    if (len < 0)
    {
        perror("Receive error!\n");
        return -1;
    }

    if (len == 0)
    {
        perror("Connect interrupt! The client may be shutdown!\n");
        return 0;
    }

    return len;
}

int TcpServer::Send(char *pSndBuf, int bufLen)
{
    int len;

    if (_clientFD == 0)
    {
        perror("Not connected yet or client fd has been released before, send failed!\n");
        return -1;
    }

    if ((len = send(_clientFD, pSndBuf, bufLen, 0)) < 0)
    {
        perror("Send error!\n");
        return -1;
    }

    return len;
}

void TcpServer::Release()
{
    if (_clientFD != 0)
    {
        close(_clientFD);
        _clientFD = 0;
    }

    if (_serverFD != 0)
    {
        close(_serverFD);
        _serverFD = 0;
    }
}

bool TcpClient::init(const std::string &serverIP, int serverPort)
{
    _serverIP = serverIP;
    _serverPort = serverPort;

    if (_clientFD != 0)
    {
        printf("The client has been initialized.\n");
        return true;
    }

    setSockAddr(_serverAddr, _serverIP, _serverPort);

    /*创建客户端套接字--IPv4协议，面向无连接通信，UDP协议*/
    if (!setSockFd(_clientFD, "tcp"))
    {
        perror("Socket error!\n");
        return false;
    }

    return true;
}

bool TcpClient::Connect()
{
    if (connect(_clientFD, (struct sockaddr *) &_serverAddr, sizeof(_serverAddr)) < 0)
    {
        perror("Connect error\n");
        return false;
    }

    printf("Connected to server\n");
    char pRcvBuf[BUFFSIZE];
    int len = recv(_clientFD, pRcvBuf, BUFFSIZE, 0);//接收服务器端信息
    pRcvBuf[len] = '\0';
    printf("%s", pRcvBuf); //打印服务器端信息

    _flag_connect = true;
    return true;
}

int TcpClient::Receive(char *pRcvBuf)
{
    int len;

    if (_clientFD == 0)
    {
        perror("Client fd has been released before, receive failed!\n");
        return -2;
    }

    len = recv(_clientFD, pRcvBuf, BUFFSIZE, 0);

    if (len < 0)
    {
        perror("Receive error!\n");
        return -1;
    }

    if (len == 0)
    {
        perror("Connect interrupt! The server may be shutdown!\n");
        return 0;
    }

    return len;
}

int TcpClient::Send(char *pSndBuf, int bufLen)
{
    int len;

    if (_clientFD == 0)
    {
        perror("Client fd has been released before, send failed!\n");
        return -1;
    }

    if ((len = send(_clientFD, pSndBuf, bufLen, 0)) < 0)
    {
        perror("Send error!\n");
        return -1;
    }

    return len;
}

void TcpClient::Release()
{
    if (_clientFD != 0)
    {
        close(_clientFD);
        _clientFD = 0;
    }
}