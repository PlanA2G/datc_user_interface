/**
 * @file concurrent_queue.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief  A class for temporarily storing data to be processed simultaneously
 * @details This class uses a queue, an STL container, to process communication data
 * accumulated in real time in the form of FIFO (First in, First Out).
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef CONCURRENT_QUEUE_HPP
#define CONCURRENT_QUEUE_HPP

#include <queue>

using namespace std;

namespace tcp_communication {

/**
 * @brief Stores data by implementation of queue.
 *
 * @details This class uses a queue, an STL container, to process communication data
 * accumulated in real time in the form of FIFO (First in, First Out).
 */
template<typename Data>
class ConcurrentQueue {
public:
    /**
     * @brief Construct a new concurrent queue object
     */
    ConcurrentQueue() {}

    /**
     * @brief Move Constructor of the Concurrent Queue object
     *
     * @param[in] rhs ConcurrentQueue type object for a copy
     */
    ConcurrentQueue(const ConcurrentQueue &&rhs) {}

public:
    /**
     * @brief A function that adds data to the end of the queue
     *
     * @param[in] data Data to push into queue
     */
    void push(Data const &data) {
        queue_.push(data);
    }

    /**
     * @brief A function that returns true if the queue is empty.
     *
     * @return true If the queue is empty
     * @return false If the queue is not empty
     */
    bool empty() const {
        return queue_.empty();
    }

    /**
     * @brief A function that tries to pop data from queue if not empty
     *
     * @param[out] value Data to be tried to be popped off the queue
     *
     * @return true If the data is successfully popped off from the queue
     * @return false If an queue is empty
     */
    bool tryPop(Data &value) {
        if (queue_.empty()) {
            return false;
        }

        value = queue_.front();
        queue_.pop();
        return true;
    }

    bool clear() {
        std::queue<Data> empty_queue;
        queue_ = empty_queue;
        if (queue_.empty()) {
            return true;
        } else {
            return false;
        }
    }

private:
    queue<Data> queue_; /**< Queue implemented as containers adaptors */
};
} // namespace tcp_comm
#endif
