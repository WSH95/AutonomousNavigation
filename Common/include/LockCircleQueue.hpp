//
// Created by wsh on 8/22/21.
//

#ifndef LOCKCIRCLEQUEUE_HPP
#define LOCKCIRCLEQUEUE_HPP

#include <cstdio>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "assert.h"

template<typename T, std::size_t _Size = 1>
class LockCircleQueue
{
public:
    LockCircleQueue()
    {
        buffer_size = _Size;
        buffer.resize(buffer_size);
        front = -1;
        rear = -1;
        _flag_update = false;
    };

    ~LockCircleQueue() = default;

    void push(const T &newElem)
    {
        std::lock_guard<std::mutex> lk(_mut);

        // If the queue is empty, then fill it with T().
        if (front == -1)
        {
            front = 0;
            rear = buffer_size - 1;
        }

        // Update the index and fresh the new rear item with newElem.
        front = (front + 1) % buffer_size;
        rear = (rear + 1) % buffer_size;
        buffer[rear] = newElem;

        _flag_update = true;

        // Wake up the waiting thread.
        cv.notify_one();
    }

    void push(T &&newElem)
    {
        std::lock_guard<std::mutex> lk(_mut);

        // If the queue is empty, then fill it with T().
        if (front == -1)
        {
            front = 0;
            rear = buffer_size - 1;
        }

        // Update the index and fresh the new rear item with newElem.
        front = (front + 1) % buffer_size;
        rear = (rear + 1) % buffer_size;
        buffer[rear] = std::forward<decltype(newElem)>(newElem);

        _flag_update = true;

        // Wake up the waiting thread.
        cv.notify_one();
    }

    void push(T *newElem, int len)
    {
        // Promise the len is smaller than buffer_size.
        assert(len <= buffer_size);

        std::lock_guard<std::mutex> lk(_mut);

        // If the queue is empty, then fill it with T().
        if (front == -1)
        {
            front = 0;
            rear = buffer_size - 1;
        }

        for (int i = 0; i < len; ++i)
        {
            // Update the index and fresh the new rear item with newElem.
            front = (front + 1) % buffer_size;
            rear = (rear + 1) % buffer_size;
            buffer[rear] = *(newElem + i);
        }

        _flag_update = true;

        // Wake up the waiting thread.
        cv.notify_one();
    }

    void pop_wait(T *receiver)
    {
        std::unique_lock<std::mutex> lk(_mut);

        // If the queue is empty, wait for data.
        cv.wait(lk, [this]
        { return (front != -1); });

        *receiver = buffer[rear];

        _flag_update = false;
    }

    void pop_wait(T &receiver)
    {
        std::unique_lock<std::mutex> lk(_mut);

        // If the queue is empty, wait for data.
        cv.wait(lk, [this]
        { return (front != -1); });

        receiver = buffer[rear];

        _flag_update = false;
    }

    void pop_wait(T *receiver, int len)
    {
        // Promise the len is smaller than buffer_size.
        assert(len <= buffer_size);

        std::unique_lock<std::mutex> lk(_mut);

        // If the queue is empty, wait for data.
        cv.wait(lk, [this]
        { return (front != -1); });

        for (int i = 0; i < len; ++i)
        {
            *(receiver + i) = buffer[(rear - (len - 1) + i + buffer_size) % buffer_size];
        }

        _flag_update = false;
    }

    void pop_anyway(T *receiver)
    {
        std::lock_guard<std::mutex> lk(_mut);

        // If the queue is empty, extract any on of the buffer, otherwise extract the rear one.
        if (front == -1)
        {
            *receiver = buffer[0];
        }
        else
        {
            *receiver = buffer[rear];
        }

        _flag_update = false;
    }

    void pop_anyway(T *receiver, int len)
    {
        // Promise the len is smaller than buffer_size.
        assert(len <= buffer_size);

        std::lock_guard<std::mutex> lk(_mut);

        // If the queue is empty, extract any on of the buffer, otherwise extract from the rear in sequence.
        if (front == -1)
        {
            for (int i = 0; i < len; ++i)
            {
                *(receiver + i) = buffer[0];
            }
        }
        else
        {
            for (int i = 0; i < len; ++i)
            {
                *(receiver + i) = buffer[(rear - (len - 1) + i + buffer_size) % buffer_size];
            }
        }
        _flag_update = false;
    }

    void pop_uptodate(T *receiver)
    {
        std::unique_lock<std::mutex> lk(_mut);

        // If the queue is empty, wait for data.
        cv.wait(lk, [this]
        { return (_flag_update); });

        *receiver = buffer[rear];

        _flag_update = false;
    }

    void pop_uptodate(T &receiver)
    {
        std::unique_lock<std::mutex> lk(_mut);

        // If the queue is empty, wait for data.
        cv.wait(lk, [this]
        { return (_flag_update); });

        receiver = buffer[rear];

        _flag_update = false;
    }

    bool isUPdate()
    {
        std::lock_guard<std::mutex> lk(_mut);
        return _flag_update;
    }

    bool empty()
    {
        std::lock_guard<std::mutex> lk(_mut);
        if (front == -1)
            return true;
        else
            return false;
    }

    void clear()
    {
        std::lock_guard<std::mutex> lk(_mut);
        front = -1;
        rear = -1;
        for (int i = 0; i < buffer_size; i++)
        {
            buffer[i] = T();
        }
        _flag_update = false;
    }

private:
    long front, rear;
    std::vector<T> buffer;
    size_t buffer_size;
    std::mutex _mut;
    std::condition_variable cv;
    volatile bool _flag_update;
};

#endif //LOCKCIRCLEQUEUE_HPP
