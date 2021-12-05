//
// Created by wsh on 2021/9/7.
//

#ifndef RECVPLANNINGUDP_HPP
#define RECVPLANNINGUDP_HPP

#include "UDP.hpp"
#include "Loop.hpp"
#include "LockCircleQueue.hpp"
#include <tuple>

constexpr int PLANNING_SERVER_PORT = 6112;
constexpr int PLANNING_COMMAND_LENGTH = 18;

class RecvPlanningUdp
{
public:
    RecvPlanningUdp()
    {
        udp.init("", 0, PLANNING_SERVER_PORT, 0);
        loop_receiving = LoopFunction("RecvPlanning", 0.0025, [this]{ receive(); });
        clock_gettime(CLOCK_MONOTONIC, &startTime);
        startTime.tv_sec = 0;
        waitFirstCmd = true;
        loop_receiving.start();
    }

    ~RecvPlanningUdp() = default;

    void readCmd(float &vel_x, float &vel_y, float &turn_rate)
    {
        clock_gettime(CLOCK_MONOTONIC, &endTime);
        if ((endTime.tv_sec - startTime.tv_sec) > 1)
            waitFirstCmd = true;
        commandQueue.pop_wait(&command);
        while (waitFirstCmd && (!std::get<0>(command)))
        {
            usleep(1000);
            commandQueue.pop_anyway(&command);
        }

        std::tie(std::ignore, vel_x, vel_y, turn_rate) = command;
        waitFirstCmd = false;
        clock_gettime(CLOCK_MONOTONIC, &startTime);
    }

    void testRead()
    {
        float tmp_x, tmp_y, tmp_w;
        while (true)
        {
            readCmd(tmp_x, tmp_y, tmp_w);
        }
    }

private:
    float ConvertByte2Float(unsigned char *pByte)
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

    void receive()
    {
        int len = udp.Receive(recvBuf);
        if (len == PLANNING_COMMAND_LENGTH)
        {
            auto head = std::string(recvBuf, recvBuf + 4);
            if (head == "Auto")
            {
                bool isFirst = recvBuf[5];
                float v_x = ConvertByte2Float((unsigned char *)(recvBuf + 6));
                float v_y = ConvertByte2Float((unsigned char *)(recvBuf + 10));
                float v_w = ConvertByte2Float((unsigned char *)(recvBuf + 14));
                commandQueue.push(std::make_tuple(isFirst, v_x, v_y, v_w));
            }
            else
            {
                std::cout << "[Auto] remote receive error!" << std::endl;
            }
        }
    }

    Udp udp;
    LoopFunction loop_receiving;
    timespec startTime;
    timespec endTime;
    uint8_t recvBuf[PLANNING_COMMAND_LENGTH];
    LockCircleQueue<std::tuple<bool, float, float, float>, 1> commandQueue;
    std::tuple<bool, float, float, float> command;
    bool waitFirstCmd;
};

class TestRecvPlanning
{
public:
    TestRecvPlanning()
    {
        recvAll_thread = LoopFunction("TestRecvPlanning", 0.005, [this]{ receive(); });
    }
    ~TestRecvPlanning() = default;

    void start()
    {
        recvAll_thread.start();
    }

    void receive()
    {
        float tmp_x, tmp_y, tmp_w;
        recv_planning_cmd.readCmd(tmp_x, tmp_y, tmp_w);
        // std::cout << "vel_x: " << tmp_x << ' ' << "vel_y: " << tmp_y << ' ' << "vel_w: " << tmp_w << std::endl;
        std::cout << tmp_x << '\t' << tmp_y << '\t' << tmp_w << std::endl;
    }

private:
    RecvPlanningUdp recv_planning_cmd;
    LoopFunction recvAll_thread;
};

#endif //RECVPLANNINGUDP_HPP
