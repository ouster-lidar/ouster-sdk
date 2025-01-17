/**
 * Copyright(c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/impl/threadsafe_queue.h"

#include <gtest/gtest.h>

TEST(ThreadsafeQueueTest, ThreadsafeQueueTest) {
    ThreadsafeQueue<int> queue(3);
    queue.push(1);
    queue.push(2);
    queue.push(3);
    queue.shutdown();
    EXPECT_EQ(queue.pop(), 1);
    EXPECT_EQ(queue.pop(), 2);
    EXPECT_EQ(queue.pop(), 3);
    EXPECT_EQ(queue.pop(), nonstd::nullopt);
}

TEST(ThreadsafeQueueTest, ThreadsafeQueueTest2) {
    ThreadsafeQueue<int> queue(3);
    queue.push(1);
    queue.shutdown();

    EXPECT_THROW(
        {
            try {
                queue.push(2);
            } catch (std::logic_error& e) {
                EXPECT_STREQ("queue is shutdown", e.what());
                throw;
            }
        },
        std::logic_error);
    EXPECT_EQ(queue.pop(), 1);
    EXPECT_EQ(queue.pop(), nonstd::nullopt);
}
