/**
 * Copyright(c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "nonstd/optional.hpp"

/**
 * @brief a threadsafe queue.
 *
 * This is a blocking threadsafe queue class: pushing blocks forever until there
 * is room in the queue for a new item; popping blocks until an item is
 * available or the queue is "shut down" by another thread, causing all threads
 * waiting for a pop() to receive a nonstd::nullopt.
 *
 * This queue also requires that items be movable.
 */
template <typename T>
class ThreadsafeQueue {
    std::queue<T> queue_{};
    std::mutex mutex_{};
    std::condition_variable empty_condition_{};
    std::condition_variable full_condition_{};
    bool shutdown_{false};
    size_t capacity_{};

   public:
    /**
     * Instantiates a ThreadsafeQueue with the given capacity.
     */
    ThreadsafeQueue(size_t capacity) : capacity_(capacity) {}

    /**
     * Adds an item to the queue, blocking until there is room for the item.
     *
     * @throws std::logic_error if the queue has been shut down.
     */
    void push(T&& t) {
        std::unique_lock<std::mutex> lock(mutex_);

        if (shutdown_) {
            throw std::logic_error("queue is shutdown");
        }

        // Block until there is room for the item.
        while (queue_.size() >= capacity_) {
            full_condition_.wait(lock);
        }
        queue_.push(std::move(t));

        // Wake up a thread waiting to pop.
        empty_condition_.notify_one();
    }

    /**
     * Returns the oldest item in the queue,
     * or std::nullopt if there are no more items and the queue is shut down.
     *
     * This method blocks the calling thread indefinitely,
     * until either an item is available or shutdown() is called from another
     * thread, after which std::nullopt is returned.
     *
     * @throws std::logic_error if it would return a nonstd::nullopt
     * but the queue is not shut down (a contradiction.)
     */
    nonstd::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);

        // Wait for the queue to have an item in it, or to be shutdown.
        while (queue_.empty() && !shutdown_) {
            empty_condition_.wait(lock);
        }

        if (queue_.empty()) {
            if (!shutdown_) {
                throw std::logic_error("queue is empty but not shut down");
            }
            return nonstd::nullopt;
        }

        auto front = std::move(queue_.front());
        queue_.pop();

        // Wake up a thread waiting to push.
        full_condition_.notify_one();
        return front;
    }

    /**
     * Shuts down the queue, waking up any threads that have called "pop".
     */
    void shutdown() {
        std::unique_lock<std::mutex> lock(mutex_);
        shutdown_ = true;
        empty_condition_.notify_all();
    }
};
