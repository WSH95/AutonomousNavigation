//
// Created by wsh on 2021/12/17.
//

#ifndef AUTONOMOUS_NAVIGATION_READEULERZ_HWT101_HPP
#define AUTONOMOUS_NAVIGATION_READEULERZ_HWT101_HPP

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <stdlib.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <time.h>
#include <iostream>

#define MODBUS_DEV_NAME "/dev/ttyUSB0"    ///< 串口设备
constexpr int HW101_DEVICE_ID = 0x50;
constexpr int BAUD = 921600;

class ReadEulerZ_HWT101
{
public:
    ReadEulerZ_HWT101() = default;

    ~ReadEulerZ_HWT101()
    {
        close();
    }

    void setup()
    {
        init();
        waitConnect();
        setZero();
    }

    bool init()
    {
        // 以串口的方式创建libmobus实例,并设置参数
        rtuMaster = modbus_new_rtu(MODBUS_DEV_NAME, BAUD, 'N', 8, 1);
        if (rtuMaster == nullptr)
        {
            fprintf(stderr, "[HWT101] Unable to allocate libmodbus contex\n");
            return false;
        }
        // 使用RS485时需考虑设置串口模式、RTS引脚等
        modbus_rtu_set_serial_mode(rtuMaster, MODBUS_RTU_RS485);    //设置串口模式

        modbus_set_debug(rtuMaster, 0);      //设置1可看到调试信息
        modbus_set_slave(rtuMaster, HW101_DEVICE_ID);      //设置slave ID

        return true;
    }

    bool waitConnect()
    {
        while (modbus_connect(rtuMaster) == -1)
        {
            fprintf(stderr, "[HWT101] No Device Connected:%s\n", modbus_strerror(errno));
            std::cout << "[HWT101] Waitting for Device" << std::endl;
            usleep(100000);
        }
        std::cout << "[HWT101] Device connect successfully!" << std::endl;
        return true;
    }

    void close()
    {
        modbus_close(rtuMaster);
        modbus_free(rtuMaster);
    }

    int getNumSlave()
    {
        return modbus_get_slave(rtuMaster);
    }

    float readZ_rad()
    {
        uint16_t tmp_read_buffer[3];
        int len = modbus_read_registers(rtuMaster, 0x003d, 3, tmp_read_buffer);
        if (len == 3)
        {
            float rad_z = (int16_t) tmp_read_buffer[2] / 32768.0 * 3.1415926;
            std::cout << "angle: " << rad_z << " rad" << std::endl;
            return rad_z;
        }
        else
        {
            printf("[HWT101] read error: %s\n", modbus_strerror(errno));
            return -12345;
        }
    }

    bool resetSensor()
    {
        int len = modbus_write_register(rtuMaster, 0x0000, 0x0001);
        if (len == 1)
        {
            std::cout << "[HWT101] reset successfully" << std::endl;
            return true;
        }
        else
        {
            std::cout << "[HWT101] reset false" << std::endl;
            return false;
        }
    }

    bool setZero()
    {
        int len = modbus_write_register(rtuMaster, 0x0076, 0x0000);
        if (len == 1)
        {
            std::cout << "[HWT101] set zero successfully" << std::endl;
            return true;
        }
        else
        {
            std::cout << "[HWT101] set zero false" << std::endl;
            return false;
        }
    }

    bool unlock()
    {
        int len = modbus_write_register(rtuMaster, 0x0069, 0xB588);
        if (len == 1)
        {
            std::cout << "[HWT101] unlock successfully" << std::endl;
            return true;
        }
        else
        {
            std::cout << "[HWT101] unlock false" << std::endl;
            return false;
        }
    }

    bool setBaudRate(int rate)
    {
        int len;
        switch (rate)
        {
            case 4800:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0001);
                break;
            case 9600:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0002);
                break;
            case 19200:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0003);
                break;
            case 38400:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0004);
                break;
            case 57600:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0005);
                break;
            case 115200:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0006);
                break;
            case 230400:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0007);
                break;
            case 460800:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0008);
                break;
            case 921600:
                len = modbus_write_register(rtuMaster, 0x0004, 0x0009);
                break;
            default:
                std::cout << "invalid input.(required: 4800 9600 19200 38400 57600 115200 230400 460800 921600)"
                          << std::endl;
                return false;
        }

        if (len == 1)
        {
            std::cout << "[HWT101] baud rate set successfully (" << rate << ")" << std::endl;
            return true;
        }
        else
        {
            std::cout << "[HWT101] baud rate set false" << std::endl;
            return false;
        }
    }

private:
    modbus_t *rtuMaster = nullptr;
};

#endif //AUTONOMOUS_NAVIGATION_READEULERZ_HWT101_HPP
