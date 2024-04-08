/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Wrapper around sensor::client to provide buffering
 *
 * *Not* a public API. Currently part of the Python bindings implementation.
 *
 * Maintains a single-producer / single-consumer circular buffer that can be
 * populated by a thread without holding the GIL to deal the relatively small
 * default OS buffer size and high sensor UDP data rate. Must be thread-safe to
 * allow reading data without holding the GIL while other references to the
 * client exist.
 */

#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "ouster/client.h"
#include "ouster/impl/ring_buffer.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {
namespace impl {

struct Event {
    int source;
    client_state state;

    bool operator==(const Event& other) const {
        return source == other.source && state == other.state;
    }
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

namespace std {

template <>
struct hash<ouster::sensor::impl::Event> {
    std::size_t operator()(const ouster::sensor::impl::Event& e) const {
        auto h = std::hash<int>{};
        return h(e.source) ^ h(static_cast<int>(e.state));
    }
};

}  // namespace std

namespace ouster {
namespace sensor {
namespace impl {

using EventSet = std::unordered_set<Event>;

/**
 * Thread safe event queue.
 *
 * Safe to use with many-to-many producers/consumers, although some
 * considerations apply.
 *
 * Two main ways of usage are: one queue per multiple consumers, using
 * next(events) to have consumers wait on specific events, or using
 * multiple queues, one per consumer, as implemented in publisher/subscriber.
 */
class EventQueue {
    mutable std::mutex m;
    mutable std::condition_variable cv;
    std::deque<Event> q;

    template <typename Predicate>
    Event _next(Predicate&& p) {
        Event e;
        {
            std::unique_lock<std::mutex> lock{m};
            cv.wait(lock, p);

            e = q.front();
            q.pop_front();
        }
        cv.notify_all();
        return e;
    }

    template <typename Predicate>
    Event _next_timeout(float sec, Predicate&& p) {
        Event e;
        {
            std::unique_lock<std::mutex> lock{m};
            using fsec = std::chrono::duration<float>;
            bool timeout = !cv.wait_for(lock, fsec{sec}, p);

            if (timeout) return {-1, client_state::TIMEOUT};

            e = q.front();
            q.pop_front();
        }
        cv.notify_all();
        return e;
    }

   public:
    /**
     * Push an event to the back of the queue.
     *
     * Notifies all threads waiting on the queue.
     *
     * @param[in] e event
     */
    void push(Event e) {
        {
            std::lock_guard<std::mutex> lock{m};
            q.push_back(e);
        }
        cv.notify_all();
    }

    /**
     * Push a [first,last) range of events to the back of the queue.
     *
     * Notifies all threads waiting on the queue.
     *
     * @param[in] first iterator to the first event
     * @param[in] last past-the-end iterator
     */
    template <typename EventIterT>
    void push(EventIterT first, EventIterT last) {
        {
            std::lock_guard<std::mutex> lock{m};
            q.insert(q.end(), first, last);
        }
        cv.notify_all();
    }

    /**
     * Push an event to the front of the queue.
     *
     * Notifies all threads waiting on the queue.
     *
     * @param[in] e event
     */
    void push_priority(Event e) {
        {
            std::lock_guard<std::mutex> lock{m};
            q.push_front(e);
        }
        cv.notify_all();
    }

    /**
     * Pop an event from the front of the queue.
     * If the queue is empty, blocks until an event is pushed.
     *
     * Notifies all other threads waiting on the queue.
     *
     * @return event
     */
    Event pop() {
        return _next([this] { return !q.empty(); });
    }

    /**
     * Pop an event from the front of the queue.
     * If the queue is empty, blocks until an event is pushed or timeout_sec
     * seconds have passed
     *
     * Notifies all other threads waiting on the queue.
     *
     * @param[in] timeout_sec timeout time in seconds
     * @return event or Event{-1, client_state::TIMEOUT} in case of timeout
     */
    Event pop(float timeout_sec) {
        return _next_timeout(timeout_sec, [this] { return !q.empty(); });
    }

    /**
     * Pop an event of the set of events from the front of the queue.
     * Blocks until a suitable event is at the front of the queue.
     *
     * Notifies all other threads waiting on the queue.
     *
     * @param[in] events subset of events to wait for
     * @return event
     */
    Event next(const EventSet& events) {
        return _next([this, &events] {
            return !q.empty() && events.count(q.front()) == 1;
        });
    }

    /**
     * Pop an event of the set of events from the front of the queue.
     * Blocks until a suitable event is at the front of the queue, or
     * timeout_sec seconds have passed.
     *
     * Notifies all other threads waiting on the queue.
     *
     * @param[in] timeout_sec timeout time in seconds
     * @param[in] events subset of events to wait for
     * @return event or Event{-1, client_state::TIMEOUT} in case of timeout
     */
    Event next(float timeout_sec, const EventSet& events) {
        return _next_timeout(timeout_sec, [this, &events] {
            return !q.empty() && events.count(q.front()) == 1;
        });
    }

    /**
     * Flush the queue, immediately returning all elements.
     *
     * @return queue with all remaining events
     */
    // TODO: [[nodiscard]] once we move to cpp17 -- Tim T.
    std::deque<Event> flush() {
        std::lock_guard<std::mutex> lock{m};
        std::deque<Event> out;
        out.swap(q);
        return out;
    }
};

class Publisher {
   protected:
    EventSet events_;
    std::shared_ptr<EventQueue> q_;

   public:
    /**
     * Construct publisher accepting a corresponding set of events
     *
     * @param[in] events set of events to accept
     */
    Publisher(EventSet events)
        : events_(std::move(events)), q_(std::make_shared<EventQueue>()) {}

    /**
     * Construct empty publisher with events to be set later
     */
    Publisher() : Publisher(EventSet{}) {}

    /**
     * Sets publisher to accept events of type `e`
     *
     * @param[in] e Event type to accept
     */
    void set_accept(Event e) { events_.insert(e); }

    /**
     * Checks whether publisher accepts event type.
     *
     * @param[in] e Event type
     * @return true if publisher accepts events of type e
     */
    bool accepts(Event e) const { return events_.count(e); }

    /**
     * Publish event to the publisher queue.
     *
     * @param[in] e event
     * @param[in] to_front if true, publishes event to the front of the queue
     */
    void publish(Event e, bool to_front = false) {
        if (accepts(e)) {
            if (to_front) {
                q_->push_priority(e);
            } else {
                q_->push(e);
            }
        }
    }

    /**
     * Retrieve internal event queue.
     *
     * Mainly used for constructing subscribers.
     *
     * @return shared pointer to EventQueue
     */
    std::shared_ptr<EventQueue> queue() { return q_; }
};

class Subscriber {
   protected:
    std::shared_ptr<EventQueue> q_;
    std::shared_ptr<RingBufferMap<Event, Packet>> rb_;

    static bool _has_packet(Event e) {
        return e.state & (client_state::LIDAR_DATA | client_state::IMU_DATA);
    }

   public:
    Subscriber(std::shared_ptr<EventQueue> q,
               std::shared_ptr<RingBufferMap<Event, Packet>> rb)
        : q_(q), rb_(rb) {}

    Subscriber(Subscriber&& other) : q_(), rb_() {
        std::swap(q_, other.q_);
        std::swap(rb_, other.rb_);
    }

    /**
     * Pop the next event from the queue.
     *
     * Blocks thread until event is available.
     *
     * @return event
     */
    Event pop() { return q_->pop(); }

    /**
     * Pop the next event from the queue.
     *
     * Blocks thread until event is available, or timeout is reached.
     *
     * @param[in] timeout_sec timeout in seconds
     * @return event or {-1, client_state::TIMEOUT}
     */
    Event pop(float timeout_sec) { return q_->pop(timeout_sec); }

    /**
     * Retrieve the packet to the corresponding event.
     *
     * Packet is guaranteed to stay valid until advance() is called.
     * Will throw if the event does not correspond to any packets.
     *
     * @param[in] e event
     * @return packet corresponding to the event
     */
    Packet& packet(Event e) { return rb_->front(e); }
    const Packet& packet(Event e) const { return rb_->front(e); }

    /**
     * Advance the ring buffer read index for the corresponding event.
     */
    void advance(Event e) {
        if (_has_packet(e)) rb_->pop(e);
    }

    /**
     * Flush the queue, releasing all corresponding packets from the ring buffer
     */
    void flush() {
        auto events = q_->flush();
        for (const auto& e : events) {
            if (e.state == client_state::EXIT) {
                // return exit event back to the queue for later processing
                q_->push_priority(e);
            } else {
                advance(e);
            }
        }
    }
};

class Producer {
   protected:
    std::vector<std::shared_ptr<Publisher>> pubs_;
    std::vector<std::shared_ptr<client>> clients_;
    std::shared_ptr<RingBufferMap<Event, Packet>> rb_;

    std::mutex mtx_;
    std::atomic<bool> stop_;

    bool _verify() const;

   public:
    Producer()
        : rb_(std::make_shared<RingBufferMap<Event, Packet>>()), stop_(false) {}

    // TODO: move out to client_state extensions of some sort
    /* Extra bit flag compatible with client_state to signal buffer overflow. */
    static constexpr int CLIENT_OVERFLOW = 0x10;

    /**
     * Add client and allocate buffers for it.
     *
     * @param[in] cli shared_ptr with initialized client
     * @param[in] lidar_buf_size size of the lidar buffer, in packets
     * @param[in] lidar_packet_size size of the lidar packet, in bytes
     * @param[in] imu_buf_size size of the imu buffer, in packets
     * @param[in] imu_packet_size size of the imu packet, in bytes
     * @return id of the client used in produced events e.g.
     *         Event{id, client_state}
     */
    int add_client(std::shared_ptr<client> cli, size_t lidar_buf_size,
                   size_t lidar_packet_size, size_t imu_buf_size,
                   size_t imu_packet_size);

    /**
     * Add client and allocate buffers for it.
     *
     * Calculates the buffer sizes for the client based on hz rate and provided
     * seconds_to_buffer parameter.
     *
     * @param[in] cli shared_ptr with initialized client
     * @param[in] info sensor_info corresponding to the client
     * @param[in] seconds_to_buffer amount of seconds worth of buffer allocation
     * @return id of the client used in produced events e.g.
     *         Event{id, client_state}
     */
    int add_client(std::shared_ptr<client> cli, const sensor_info& info,
                   float seconds_to_buffer);

    /**
     * Subscribe to a preassembled publisher.
     *
     * @param[in] pub shared_ptr containing preassembled publisher
     * @return shared_ptr with subscriber corresponding to the publisher
     */
    std::shared_ptr<Subscriber> subscribe(std::shared_ptr<Publisher> pub);

    /**
     * Subscribe to a specific set of events.
     *
     * @param[in] events set of events to subscribe to
     * @return shared_ptr with subscriber waiting on the events
     */
    std::shared_ptr<Subscriber> subscribe(EventSet events);

    /**
     * Write data from the network into the circular buffer, reporting events to
     * publishers.
     *
     * Internally verifies that at least some publishers are subscribed to all
     * of the events that could be reported by the producer, otherwise returns
     * early.
     *
     * Will return when either shutdown() is called by one of the threads, or
     * when CLIENT_ERROR or EXIT are reported by clients.
     */
    void run();

    /**
     * Signal the producer to exit and reports EXIT event to all listening
     * subscribers, then waits for producer thread to exit before returning.
     *
     * Additionally, clears all internal publishers and buffers.
     */
    void shutdown();

    /**
     * Reports total amount of currently stored packets in internal buffers.
     *
     * NOTE: this is not a great metric since it does not report specific
     * buffers, but the total amount instead.
     */
    size_t size() const { return rb_->size(); }

    /**
     * Reports total allocated capacity of packets stored in internal buffers.
     *
     * NOTE: this is not a great metric since it does not report specific
     * buffers, but the total amount instead.
     */
    size_t capacity() const { return rb_->capacity(); }
};

class UDPPacketSource : protected Producer, protected Subscriber {
    void _accept_client_events(int id);

   public:
    UDPPacketSource();

    /**
     * Add client and allocate buffers for it.
     *
     * @param[in] cli shared_ptr with initialized client
     * @param[in] lidar_buf_size size of the lidar buffer, in packets
     * @param[in] lidar_packet_size size of the lidar packet, in bytes
     * @param[in] imu_buf_size size of the imu buffer, in packets
     * @param[in] imu_packet_size size of the imu packet, in bytes
     * @return id of the client used in produced events e.g.
     *         Event{id, client_state}
     */
    void add_client(std::shared_ptr<client> cli, size_t lidar_buf_size,
                    size_t lidar_packet_size, size_t imu_buf_size,
                    size_t imu_packet_size);

    /**
     * Add client and allocate buffers for it.
     *
     * Calculates the buffer sizes for the client based on hz rate and provided
     * seconds_to_buffer parameter.
     *
     * @param[in] cli shared_ptr with initialized client
     * @param[in] info sensor_info corresponding to the client
     * @param[in] seconds_to_buffer amount of seconds worth of buffer allocation
     * @return id of the client used in produced events e.g.
     *         Event{id, client_state}
     */
    void add_client(std::shared_ptr<client> cli, const sensor_info& info,
                    float seconds_to_buffer);

    using Producer::capacity;
    using Producer::shutdown;
    using Producer::size;
    void produce() { Producer::run(); }

    using Subscriber::advance;
    using Subscriber::flush;
    using Subscriber::packet;
    using Subscriber::pop;
};

class BufferedUDPSource : protected Producer, protected Subscriber {
    BufferedUDPSource();

   public:
    /**
     * Listen for sensor data using client
     *
     * @param[in] client externally created client
     * @param[in] lidar_buf_size size of the lidar buffer, in packets
     * @param[in] lidar_packet_size size of the lidar packet, in bytes
     * @param[in] imu_buf_size size of the imu buffer, in packets
     * @param[in] imu_packet_size size of the imu packet, in bytes
     */
    BufferedUDPSource(std::shared_ptr<client> client, size_t lidar_buf_size,
                      size_t lidar_packet_size, size_t imu_buf_size,
                      size_t imu_packet_size);

    /**
     * Listen for sensor data using client
     *
     * Calculates the buffer sizes for the client based on hz rate and provided
     * seconds_to_buffer parameter.
     *
     * @param[in] client externally created client
     * @param[in] info sensor_info corresponding to the client
     * @param[in] seconds_to_buffer amount of seconds worth of buffer allocation
     */
    BufferedUDPSource(std::shared_ptr<client> client, const sensor_info& info,
                      float seconds_to_buffer);

    using Producer::capacity;
    using Producer::shutdown;
    using Producer::size;
    void produce() { Producer::run(); }

    using Subscriber::flush;

    /**
     * Pop the next client_state from the queue.
     *
     * Blocks thread until client_state is available.
     *
     * @return client_state
     */
    client_state pop() { return Subscriber::pop().state; }

    /**
     * Pop the next client_state from the queue.
     *
     * Blocks thread until client_state is available, or timeout is reached.
     *
     * @param[in] timeout_sec timeout in seconds
     * @return client_state or client_state::TIMEOUT
     */
    client_state pop(float timeout_sec) {
        return Subscriber::pop(timeout_sec).state;
    }

    /**
     * Retrieve the packet to the corresponding client_state.
     *
     * Packet is guaranteed to stay valid until advance() is called.
     * Will throw if the event does not correspond to any packets.
     *
     * @param[in] st client_state
     * @return packet corresponding to st
     */
    Packet& packet(client_state st) { return Subscriber::packet({0, st}); }
    const Packet& packet(client_state st) const {
        return Subscriber::packet({0, st});
    }

    /**
     * Advances read in internal buffers
     *
     * @param[in] st client_state to advance. Does nothing if st is not one of
     *            LIDAR_DATA or IMU_DATA
     */
    void advance(client_state st) { Subscriber::advance({0, st}); }

    /**
     * Read next available packet in the buffer.
     *
     * If client_state returns LIDAR_DATA, submitted lidar packet will be
     * populated, similarly if client_state returns IMU_DATA, submitted
     * imu packet will be populated instead.
     *
     * Blocks if the queue is empty for up to `timeout_sec` (zero means wait
     * forever). Should only be called by the consumer thread. If reading from
     * the network was blocked because the buffer was full, the the
     * CLIENT_OVERFLOW flag will be set on the next returned status.
     *
     * @param[in] lidarp lidar packet to read into
     * @param[in] imup imu packet to read into
     * @param[in] timeout_sec maximum time to wait for data.
     * @return client status, see sensor::poll_client().
     */
    client_state consume(LidarPacket& lidarp, ImuPacket& imup,
                         float timeout_sec);
};

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
