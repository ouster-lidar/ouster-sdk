/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

/// A threadpool capable of running arbitrary functions
template <class T>
class Threadpool {
    struct QueueItem {
        std::function<T()> task;
        std::promise<T> promise;
    };

    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable cv_;

    std::deque<std::unique_ptr<QueueItem>> tasks_;
    bool terminate_ = false;

   public:
    /// Create a new threadpool
    Threadpool() {
        // create threads
        int n_threads = std::thread::hardware_concurrency();
        if (n_threads == 0) {
            n_threads = 4;
        }
        for (int i = 0; i < n_threads; i++) {
            threads_.emplace_back(std::bind(&Threadpool::thread_fn, this));
        }
    }

    /// Destruct the threadpool
    ~Threadpool() {
        {
            std::unique_lock<std::mutex> lk(mutex_);
            terminate_ = true;
        }
        for (size_t i = 0; i < threads_.size(); i++) {
            cv_.notify_one();
        }
        for (auto& thread : threads_) {
            thread.join();
        }
    }

    /// Enqueue an item to be processed
    std::future<T> enqueue(std::function<T()> task) {
        auto item = std::make_unique<QueueItem>();
        item->task = task;
        auto future = item->promise.get_future();
        mutex_.lock();
        tasks_.push_back(std::move(item));
        mutex_.unlock();
        cv_.notify_one();
        return future;
    }

   private:
    void thread_fn() {
        while (true) {
            std::unique_lock<std::mutex> lk(mutex_);
            if (terminate_) {
                break;
            }
            cv_.wait(lk, [this]() { return !tasks_.empty() || terminate_; });
            if (tasks_.empty()) {
                lk.unlock();
                continue;
            }
            auto task = std::move(tasks_.front());
            tasks_.pop_front();
            lk.unlock();

            execute_task(*task);
        }
    }

    inline void execute_task(QueueItem& task);
};

template <class T>
void Threadpool<T>::execute_task(Threadpool<T>::QueueItem& task) {
    task.promise.set_value(task.task());
}

template <>
inline void Threadpool<void>::execute_task(Threadpool<void>::QueueItem& task) {
    try {
        task.task();
        task.promise.set_value();
    } catch (...) {
        task.promise.set_exception(std::current_exception());
    }
}
