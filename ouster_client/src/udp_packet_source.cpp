/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/udp_packet_source.h"

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "logging.h"
#include "ouster/client.h"
#include "ouster/impl/client_poller.h"
#include "ouster/types.h"

namespace ouster {
namespace sensor {
namespace impl {

std::string to_string(client_state st) {
    switch (static_cast<int>(st)) {
        case client_state::TIMEOUT:
            return "TIMEOUT";
        case client_state::CLIENT_ERROR:
            return "CLIENT_ERROR";
        case client_state::LIDAR_DATA:
            return "LIDAR_DATA";
        case client_state::IMU_DATA:
            return "IMU_DATA";
        case client_state::EXIT:
            return "EXIT";
        case Producer::CLIENT_OVERFLOW:
            return "OVERFLOW";
        default:
            return "UNKNOWN_EVENT";
    }
}

std::string to_string(Event e) {
    return std::string("{") + std::to_string(e.source) + ", " +
           to_string(e.state) + "}";
}

int Producer::add_client(std::shared_ptr<client> cli, size_t lidar_buf_size,
                         size_t lidar_packet_size, size_t imu_buf_size,
                         size_t imu_packet_size) {
    std::unique_lock<std::mutex> lock{mtx_, std::defer_lock};
    if (!lock.try_lock())
        throw std::runtime_error("add_client called on a running producer");

    if (!cli) throw std::runtime_error("add_client called with nullptr");

    int id = clients_.size();
    clients_.push_back(cli);
    rb_->allocate({id, client_state::LIDAR_DATA}, lidar_buf_size,
                  Packet(lidar_packet_size));
    rb_->allocate({id, client_state::IMU_DATA}, imu_buf_size,
                  Packet(imu_packet_size));
    return id;
}

int Producer::add_client(std::shared_ptr<client> cli, const sensor_info& info,
                         float seconds_to_buffer) {
    const data_format& df = info.format;
    uint32_t packets_per_frame = df.columns_per_frame / df.columns_per_packet;
    float lidar_hz = static_cast<float>(packets_per_frame) * df.fps;
    float imu_hz = 100.f;
    const packet_format& pf = get_format(info);
    return add_client(cli, static_cast<size_t>(lidar_hz * seconds_to_buffer),
                      pf.lidar_packet_size,
                      static_cast<size_t>(imu_hz * seconds_to_buffer),
                      pf.imu_packet_size);
}

std::shared_ptr<Subscriber> Producer::subscribe(
    std::shared_ptr<Publisher> pub) {
    std::unique_lock<std::mutex> lock{mtx_, std::defer_lock};
    if (!lock.try_lock())
        throw std::runtime_error("subscribe called on a running producer");

    pubs_.push_back(pub);
    return std::make_shared<Subscriber>(pub->queue(), rb_);
}

std::shared_ptr<Subscriber> Producer::subscribe(EventSet events) {
    auto pub = std::make_shared<Publisher>(events);
    return subscribe(pub);
}

bool Producer::_verify() const {
    if (clients_.size() == 0) {
        logger().error("Producer started with no clients");
        return false;
    }

    if (pubs_.size() == 0) {
        logger().error("Producer started with no publishers");
        return false;
    }

    bool out = true;

    Event last_chk;

    auto n_pubs_accept = [this, &last_chk](Event e) {
        last_chk = e;
        auto l = [e](int total, auto& pub) {
            return total + static_cast<int>(pub->accepts(e));
        };
        return std::accumulate(pubs_.begin(), pubs_.end(), 0, l);
    };

    if (n_pubs_accept({-1, client_state::CLIENT_ERROR}) == 0) {
        logger().error("Producer: none of the publishers accept {}",
                       to_string(last_chk));
        out = false;
    }

    if (n_pubs_accept({-1, client_state::EXIT}) == 0) {
        logger().error("Producer: none of the publishers accept {}",
                       to_string(last_chk));
        out = false;
    }

    for (int i = 0, end = clients_.size(); i < end; ++i) {
        if (n_pubs_accept({i, client_state::LIDAR_DATA}) != 1) {
            logger().error(
                "Producer: {} publishers accept {}, needs to be exactly one",
                n_pubs_accept(last_chk), to_string(last_chk));
            out = false;
        }

        if (n_pubs_accept({i, client_state::IMU_DATA}) != 1) {
            logger().error(
                "Producer: {} publishers accept {}, needs to be exactly one",
                n_pubs_accept(last_chk), to_string(last_chk));
            out = false;
        }

        if (n_pubs_accept({i, client_state(Producer::CLIENT_OVERFLOW)}) == 0) {
            logger().error("Producer: no publishers accept {}",
                           to_string(last_chk));
        }
    }
    return out;
}

static bool read_packet(const client& cli, Packet& packet, client_state st) {
    switch (st) {
        case client_state::LIDAR_DATA:
            return read_lidar_packet(cli, packet.as<LidarPacket>());
        case client_state::IMU_DATA:
            return read_imu_packet(cli, packet.as<ImuPacket>());
        default:
            return false;
    }
}

static client_state operator&(client_state a, client_state b) {
    int a_i = static_cast<int>(a);
    int b_i = static_cast<int>(b);
    return static_cast<client_state>(a_i & b_i);
}

void Producer::run() {
    // check publisher/client consistency
    if (!_verify()) return;

    std::vector<bool> overflows(clients_.size(), false);

    // this could be a private virtual instead
    auto handle_event = [this, &overflows](Event e) {
        const client_state overflow = client_state(Producer::CLIENT_OVERFLOW);
        switch (e.state) {
            case 0:
                break;
            case client_state::CLIENT_ERROR:
            case client_state::EXIT:
                for (auto& pub : pubs_) pub->publish(e);
                break;
            case client_state::LIDAR_DATA:
            case client_state::IMU_DATA:
                if (rb_->full(e)) {
                    if (!overflows[e.source]) {
                        overflows[e.source] = true;
                        for (auto& pub : pubs_) {
                            // publish with priority
                            pub->publish({e.source, overflow}, true);
                        }
                    }
                } else if (read_packet(*clients_[e.source], rb_->back(e),
                                       e.state)) {
                    rb_->push(e);
                    for (auto& pub : pubs_) pub->publish(e);
                    overflows[e.source] = false;
                }
                break;
            default:
                break;
        }
    };

    std::lock_guard<std::mutex> lock{mtx_};

    std::shared_ptr<client_poller> poller = make_poller();
    while (!stop_) {
        reset_poll(*poller);

        for (auto& cli : clients_) set_poll(*poller, *cli);

        int res = poll(*poller);

        if (res == 0) {  // TIMEOUT
            continue;
        } else if (res < 0) {  // CLIENT_ERROR / EXIT
            client_state st = get_error(*poller);
            handle_event({-1, st & client_state::CLIENT_ERROR});
            handle_event({-1, st & client_state::EXIT});
            break;
        } else {
            for (int i = 0, end = clients_.size(); i < end; ++i) {
                client_state st = get_poll(*poller, *clients_[i]);
                handle_event({i, st & client_state::LIDAR_DATA});
                handle_event({i, st & client_state::IMU_DATA});
            }
        }
    }
}

/*
 * Producer will release mtx_ only when it exits the loop.
 */
void Producer::shutdown() {
    stop_ = true;
    for (auto& pub : pubs_) pub->publish({-1, client_state::EXIT});

    std::lock_guard<std::mutex> lock{mtx_};
    // close UDP sockets when any producer has exited
    clients_.clear();
    pubs_.clear();
    rb_.reset(new RingBufferMap<Event, Packet>());
    stop_ = false;
}

UDPPacketSource::UDPPacketSource()
    : Producer(),
      Subscriber(std::move(*Producer::subscribe(
          {{-1, client_state::CLIENT_ERROR}, {-1, client_state::EXIT}}))) {}

void UDPPacketSource::_accept_client_events(int id) {
    pubs_[0]->set_accept({id, client_state::LIDAR_DATA});
    pubs_[0]->set_accept({id, client_state::IMU_DATA});
    pubs_[0]->set_accept({id, client_state(Producer::CLIENT_OVERFLOW)});
}

void UDPPacketSource::add_client(std::shared_ptr<client> cli,
                                 size_t lidar_buf_size,
                                 size_t lidar_packet_size, size_t imu_buf_size,
                                 size_t imu_packet_size) {
    _accept_client_events(Producer::add_client(
        cli, lidar_buf_size, lidar_packet_size, imu_buf_size, imu_packet_size));
}
void UDPPacketSource::add_client(std::shared_ptr<client> cli,
                                 const sensor_info& info,
                                 float seconds_to_buffer) {
    _accept_client_events(Producer::add_client(cli, info, seconds_to_buffer));
}

BufferedUDPSource::BufferedUDPSource()
    : Producer(),
      Subscriber(std::move(*Producer::subscribe(
          {{-1, client_state::CLIENT_ERROR},
           {-1, client_state::EXIT},
           {0, client_state::LIDAR_DATA},
           {0, client_state::IMU_DATA},
           {0, client_state(Producer::CLIENT_OVERFLOW)}}))) {}

BufferedUDPSource::BufferedUDPSource(std::shared_ptr<client> client,
                                     size_t lidar_buf_size,
                                     size_t lidar_packet_size,
                                     size_t imu_buf_size,
                                     size_t imu_packet_size)
    : BufferedUDPSource() {
    Producer::add_client(client, lidar_buf_size, lidar_packet_size,
                         imu_buf_size, imu_packet_size);
}

BufferedUDPSource::BufferedUDPSource(std::shared_ptr<client> client,
                                     const sensor_info& info,
                                     float seconds_to_buffer)
    : BufferedUDPSource() {
    Producer::add_client(client, info, seconds_to_buffer);
}

client_state BufferedUDPSource::consume(LidarPacket& lidarp, ImuPacket& imup,
                                        float timeout_sec) {
    Event e = Subscriber::pop(timeout_sec);
    client_state st = e.state;

    // return early without advancing queue
    if (!Subscriber::_has_packet(e)) return st;

    Packet& p = Subscriber::packet(e);

    auto write_packet = [&p](auto& packet) {
        auto sz = std::min<size_t>(packet.buf.size(), p.buf.size());
        std::memcpy(packet.buf.data(), p.buf.data(), sz);
        packet.host_timestamp = p.host_timestamp;
    };

    if (st & client_state::LIDAR_DATA) {
        write_packet(lidarp);
    } else if (st & client_state::IMU_DATA) {
        write_packet(imup);
    }

    Subscriber::advance(e);
    return st;
}

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
