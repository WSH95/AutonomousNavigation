//
// Created by wsh on 2021/9/4.
//

#ifndef FUNCTION_LOOP_HPP
#define FUNCTION_LOOP_HPP

#include <iostream>
#include <string>
#include <thread>
#include <sys/timerfd.h>
#include <cmath>
#include <boost/function.hpp>
#include <boost/bind.hpp>

class Loop
{
public:
    Loop() = default;

    Loop(std::string name, float period) : _name(name), _period(period)
    {}

    Loop(const Loop &) = delete;

    Loop &operator=(const Loop &) = delete;

    Loop(Loop &&) = default;

    Loop &operator=(Loop &&) = default;

    ~Loop() = default;

    void start(int priority = 0)
    {
        if (_running)
        {
            std::cout << "[PeriodicTask-" << _name << "] The task has already been running!" << std::endl;
            return;
        }
        _running = true;
        _thread = std::thread(&Loop::loopFunc, this);

        if (priority != 0)
        {
            setScheduling(_thread, SCHED_FIFO, priority);
        }
        std::cout << "[PeriodicTask-" << _name << "] Start running!" << std::endl;
    }

    void stop()
    {
        if (!_running)
        {
            std::cout << "[PeriodicTask-" << _name << "] The task has already been stopped!" << std::endl;
            return;
        }
        _running = false;
        std::cout << "[PeriodicTask-" << _name << "] Stopping!" << std::endl;
        _thread.join();
    }

    virtual void loopCallback() = 0;

    static void setScheduling(std::thread &th, int policy, int priority)
    {
        sched_param sch_params{};
        sch_params.sched_priority = priority;
        if (pthread_setschedparam(th.native_handle(), policy, &sch_params))
        {
            std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
        }
        else
        {
            std::cout << "Success to set Thread scheduling!" << std::endl;
        }
    }

    void printStatus()
    {
        printf("\n-----------------------------------------------------------\n");
        printf("|%-20s|%-6s|%-6s|%-6s|%-6s|%-6s\n", "name", "rT", "rT_max", "rP", "rP_max", "rP_des");
        printf("-----------------------------------------------------------\n");
        printf("|%-20s|%6.5f|%6.5f|%6.5f|%6.5f|%6.5f\n", _name.c_str(), _lastRuntime, _maxRuntime, _lastPeriodTime,
               _maxPeriod, _period);
        printf("-----------------------------------------------------------\n");
    }

private:
    void loopFunc()
    {
        auto timerFd = timerfd_create(CLOCK_MONOTONIC, 0);

        int seconds = (int) _period;
        int nanoseconds = (int) (1e9 * std::fmod(_period, 1.f));

        timespec startTime;
        timespec endTime;
        clock_gettime(CLOCK_MONOTONIC, &startTime);

        itimerspec timerSpec;
        timerSpec.it_interval.tv_sec = seconds;
        timerSpec.it_value.tv_sec = seconds;
        timerSpec.it_value.tv_nsec = nanoseconds;
        timerSpec.it_interval.tv_nsec = nanoseconds;

        timerfd_settime(timerFd, 0, &timerSpec, nullptr);

        unsigned long long missed = 0;

        printf("[PeriodicTask] Start %s (%d s, %d ns)\n", _name.c_str(), seconds,
               nanoseconds);
        while (_running)
        {
            clock_gettime(CLOCK_MONOTONIC, &endTime);
            _lastPeriodTime = (float) ((endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
                                       (endTime.tv_sec - startTime.tv_sec) * 1e3);
            clock_gettime(CLOCK_MONOTONIC, &startTime);

            // run the loop callback function.
            loopCallback();

            clock_gettime(CLOCK_MONOTONIC, &endTime);
            _lastRuntime = (float) ((endTime.tv_nsec - startTime.tv_nsec) / 1.e6 +
                                    (endTime.tv_sec - startTime.tv_sec) * 1e3);

            // may delay for keeping the period.
            int m = read(timerFd, &missed, sizeof(missed));
            //printf("overtime: %d\n", m);
            (void) m;

            _maxPeriod = std::max(_maxPeriod, _lastPeriodTime);
            _maxRuntime = std::max(_maxRuntime, _lastRuntime);
        }
        std::cout << "[PeriodicTask-" << _name << "] Stopped!" << std::endl;
    }

    std::string _name;
    float _period;
    volatile bool _running = false;
    std::thread _thread;

    // ms
    float _lastRuntime = 0;
    float _maxRuntime = 0;
    float _lastPeriodTime = 0;
    float _maxPeriod = 0;
};

class LoopFunction : public Loop
{
public:
    LoopFunction() = default;

    LoopFunction(std::string name, float period, const boost::function<void()> &callback) : Loop(name, period),
                                                                                            _callback(callback)
    {}

    LoopFunction(const LoopFunction &) = delete;

    LoopFunction &operator=(const LoopFunction &) = delete;

    LoopFunction(LoopFunction &&) = default;

    LoopFunction &operator=(LoopFunction &&) = default;

    ~LoopFunction() = default;

    void loopCallback() override
    {
        _callback();
    }

private:
    boost::function<void()> _callback;
};

#endif //FUNCTION_LOOP_HPP
