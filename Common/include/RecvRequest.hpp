//
// Created by wsh on 2021/9/11.
//

#ifndef FUNCTIONAL_RECVREQUEST_HPP
#define FUNCTIONAL_RECVREQUEST_HPP

#include "UDP.hpp"
#include <thread>
#include "LockCircleQueue.hpp"

constexpr int RecvRequest_LOCAL_PORT = 6116;

class RecvRequest
{
public:
    RecvRequest()
    {
        _udp.init("", 0, RecvRequest_LOCAL_PORT, 0);
        isSending = false;
        _recvThread = std::thread(&RecvRequest::receiveRequest, this);
    }

    RecvRequest(const RecvRequest &) = delete;

    RecvRequest &operator=(const RecvRequest &) = delete;

    RecvRequest(RecvRequest &&) = default;

    RecvRequest &operator=(RecvRequest &&) = default;

    ~RecvRequest() = default;

    int SendBack(unsigned char *pSndBuf, int bufLen)
    {
        if (isSending)
        {
            int ret = _udp.Send(pSndBuf, bufLen);

            return ret;
        }
        else
            return -1;
    }

    bool sendingImg()
    {
        return isSending;
    }

    void parsePickedPointsPtr(LockCircleQueue<std::vector<std::vector<float>>, 1> *p)
    {
        pickedPointsList = p;
    }

private:
    void receiveRequest()
    {
        while (true)
        {
            uint8_t recvBuf[1024];
            for (int i = 0; i < 1024; i++)
            {
                recvBuf[i] = 0;
            }
            _udp.Receive(recvBuf);
            std::string message = std::string(recvBuf, recvBuf + 13);
            if (message == "Request Image")
            {
                isSending = true;
                uint8_t tmp[11];
                tmp[0] = 0x11;
                strncpy((char *) tmp + 1, "I am Ready", 10);
                _udp.Send(tmp, 11);
                std::cout << "Receive the image request." << std::endl;
            }
            else if (message == "Stop sending!")
            {
                isSending = false;
                uint8_t tmp[8];
                tmp[0] = 0x12;
                strncpy((char *) tmp + 1, "Stopped", 7);
                _udp.Send(tmp, 8);
            }
            else if (message == "Picked Points")
            {
                int num_points = SocketTools::ConvertByte2Int(recvBuf + 13);
//                std::cout << "*********************num points: " << num_points << std::endl;
                if (num_points > 0)
                {
                    std::vector<std::vector<float>> currentPointsList;
                    for (int i = 0; i < num_points; i++)
                    {
                        std::vector<float> tmpPoint;
                        tmpPoint.push_back(SocketTools::ConvertByte2Float(recvBuf + 17 + 8 * i));
                        tmpPoint.push_back(SocketTools::ConvertByte2Float(recvBuf + 21 + 8 * i));
                        currentPointsList.push_back(tmpPoint);
                    }
                    /// test
                    std::cout << "picked points: " << std::endl;
                    for (auto &&point : currentPointsList)
                    {
                        std::cout << "( " << point[0] << ", " << point[1] << " )" << std::endl;
                    }

                    pickedPointsList->push(currentPointsList);
                }
            }
            usleep(100000);
        }
    }

    volatile bool isSending;
    Udp _udp;
    std::thread _recvThread;
    LockCircleQueue<std::vector<std::vector<float>>, 1> *pickedPointsList = nullptr;
};

#endif //FUNCTIONAL_RECVREQUEST_HPP
