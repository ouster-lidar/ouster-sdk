/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <numeric>
#include <thread>
#include <unordered_set>

#include "ouster/pcap.h"
#include "ouster/types.h"
#include "ouster/udp_packet_source.h"
#include "util.h"

// clang-format off
#include "ouster/impl/netcompat.h"
// clang-format on

using namespace ouster::sensor;
using namespace ouster::sensor::impl;

TEST(UdpQueueTest, lidar_packet_capacity_test) {
    auto lp = LidarPacket(2048);
    EXPECT_GE(lp.buf.capacity(), lp.buf.size() + 1);

    // this is a platform sanity check more than anything
    uint8_t* ptr = lp.buf.data() + lp.buf.size();
    EXPECT_NO_THROW(*ptr = 0xca);
    EXPECT_EQ(*(lp.buf.data() + lp.buf.size()), 0xca);
}

struct non_default_constructible {
    void* ptr;
    non_default_constructible() = delete;
    non_default_constructible(void* p) : ptr(p) {}
};

TEST(UdpQueueTest, ring_buffer_test) {
    // should compile
    RingBuffer<non_default_constructible>{10, {nullptr}};
    // will not compile
    // RingBuffer<non_default_constructible>{10};

    RingBuffer<int> rb{10, 0};

    EXPECT_EQ(rb.capacity(), 10);
    EXPECT_EQ(rb.size(), 0);
    EXPECT_EQ(rb.empty(), true);
    EXPECT_EQ(rb.full(), false);

    EXPECT_THROW(rb.pop(), std::underflow_error);

    std::vector<int> writes{};
    std::vector<int> reads{};

    // write
    for (int i = 0; !rb.full(); ++i) {
        writes.push_back(i);
        rb.back() = i;
        EXPECT_NO_THROW(rb.push());
        EXPECT_EQ(rb.size(), writes.size());
    }
    EXPECT_EQ(rb.size(), rb.capacity());
    EXPECT_THROW(rb.push(), std::overflow_error);

    // read
    while (!rb.empty()) {
        reads.push_back(rb.front());
        EXPECT_NO_THROW(rb.pop());
        EXPECT_EQ(rb.size(), writes.size() - reads.size());
    }
    EXPECT_EQ(rb.size(), 0);
    EXPECT_THROW(rb.pop(), std::underflow_error);

    EXPECT_EQ(writes, reads);
}

TEST(UdpQueueTest, event_queue_tests) {
    auto eq = std::make_shared<EventQueue>();

    auto produce = [](std::shared_ptr<EventQueue> eq, int consumers, int runs) {
        std::vector<Event> events;
        while (runs > 0) {
            events.clear();

            for (int c = 0; c < consumers; ++c) {
                events.push_back({c, LIDAR_DATA});
                events.push_back({c, IMU_DATA});
            }

            eq->push(events.begin(), events.end());
            std::this_thread::sleep_for(std::chrono::microseconds(100));

            --runs;
        }

        for (int i = 0; i < consumers; ++i) eq->push({-1, client_state::EXIT});
    };

    auto consume = [](std::shared_ptr<EventQueue> eq, EventSet subscriptions,
                      int* lidar_counts, int* imu_counts) {
        while (true) {
            auto e = eq->next(subscriptions);
            if (e.state == client_state::LIDAR_DATA) (*lidar_counts)++;
            if (e.state == client_state::IMU_DATA) (*imu_counts)++;
            if (e.state == client_state::EXIT) break;
        }
    };

    int n_clients = 10;
    int runs = 100;
    std::thread prod(produce, eq, n_clients, runs);

    std::vector<int> lidar_counts(n_clients);
    std::vector<int> imu_counts(n_clients);
    std::vector<std::thread> consumers;
    for (int i = 0; i < n_clients; ++i) {
        EventSet subs = {{-1, client_state::EXIT},
                         {i, client_state::LIDAR_DATA},
                         {i, client_state::IMU_DATA}};
        consumers.emplace_back(consume, eq, subs, &lidar_counts[i],
                               &imu_counts[i]);
    }
    prod.join();
    for (auto& c : consumers) {
        c.join();
    }

    for (auto& c : lidar_counts) {
        EXPECT_EQ(c, runs);
    }
    for (auto& c : imu_counts) {
        EXPECT_EQ(c, runs);
    }
}

using str_pair = std::pair<std::string, std::string>;
class UdpQueuePcapTest : public ::testing::TestWithParam<str_pair> {};

// clang-format off
INSTANTIATE_TEST_CASE_P(
    UdpQueuePcapTests,
    UdpQueuePcapTest,
    ::testing::Values(
        str_pair{"OS-0-128-U1_v2.3.0_1024x10.pcap",
                 "OS-0-128-U1_v2.3.0_1024x10.json"},
        str_pair{"OS-0-32-U1_v2.2.0_1024x10.pcap",
                 "OS-0-32-U1_v2.2.0_1024x10.json"},
        str_pair{"OS-1-128_767798045_1024x10_20230712_120049.pcap",
                 "OS-1-128_767798045_1024x10_20230712_120049.json"},
        str_pair{"OS-1-128_v2.3.0_1024x10_lb_n3.pcap",
                 "OS-1-128_v2.3.0_1024x10.json"},
        str_pair{"OS-2-128-U1_v2.3.0_1024x10.pcap",
                 "OS-2-128-U1_v2.3.0_1024x10.json"})
);
// clang-format on

using namespace ouster::sensor_utils;

template <typename Duration,
          typename Clock = std::chrono::high_resolution_clock>
struct delayer {
   private:
    Duration start_;

   public:
    delayer() : start_(Duration::zero()) {}

    static Duration now() {
        return std::chrono::duration_cast<Duration>(
            Clock::now().time_since_epoch());
    }

    Duration now_from_start() const { return now() - start_; }

    void start() { start_ = now(); }

    void reset() { start_ = Duration::zero(); }

    void delay(Duration next) const {
        std::this_thread::sleep_for(next - now_from_start());
    }
};

class PcapReplay {
    SOCKET sockfd_;
    sockaddr_in dest_;

    PcapReader pcap_;
    std::chrono::microseconds pcap_start_ts_;

    uint16_t lidar_port_;
    uint16_t imu_port_;
    uint16_t pcap_lidar_port_;
    uint16_t pcap_imu_port_;

    void _send(const uint8_t* data, size_t len, uint16_t port) const {
        auto dest = dest_;
        dest.sin_port = htons(port);
        sendto(sockfd_, (const char*)data, len, 0, (const sockaddr*)&dest,
               sizeof(dest));
    }

    uint16_t _reroute_port(int port) const {
        if (port == pcap_lidar_port_)
            return lidar_port_;
        else if (port == pcap_imu_port_)
            return imu_port_;
        return 0;
    }

   public:
    PcapReplay(std::string pcap_filename, uint16_t pcap_lidar_port,
               uint16_t pcap_imu_port, uint16_t lidar_port, uint16_t imu_port)
        : sockfd_(socket(AF_INET, SOCK_DGRAM, 0)),
          pcap_(pcap_filename),
          lidar_port_(lidar_port),
          imu_port_(imu_port),
          pcap_lidar_port_(pcap_lidar_port),
          pcap_imu_port_(pcap_imu_port) {
        if (!impl::socket_valid(sockfd_)) {
            throw std::runtime_error("PcapReplay: failed to bind socket");
        }

        memset(&dest_, 0, sizeof(dest_));
        dest_.sin_family = AF_INET;
        dest_.sin_addr.s_addr = inet_addr("127.0.0.1");

        pcap_.next_packet();
        pcap_start_ts_ = pcap_.current_info().timestamp;
    }

    ~PcapReplay() {}

    std::chrono::microseconds next_delay() const {
        return pcap_.current_info().timestamp - pcap_start_ts_;
    }

    int send(delayer<std::chrono::microseconds> delayer) {
        _send(pcap_.current_data(), pcap_.current_length(),
              _reroute_port(pcap_.current_info().dst_port));
        delayer.delay(next_delay());
        return pcap_.next_packet();
    }

    void reset() {
        pcap_.reset();
        pcap_.next_packet();
    }
};

void replay(bool* stop, std::vector<std::shared_ptr<PcapReplay>> replays,
            int loop = 1) {
    std::vector<int> loops(replays.size(), 0);
    std::vector<int> alive(replays.size());
    std::iota(alive.begin(), alive.end(), 0);

    auto all_loops_complete = [&loops, loop] {
        return std::all_of(loops.begin(), loops.end(),
                           [loop](int l) { return l >= loop; });
    };

    auto next_replay_id = [&replays, &alive] {
        auto it = std::min_element(
            alive.begin(), alive.end(), [&replays](int id1, int id2) {
                return replays[id1]->next_delay() < replays[id2]->next_delay();
            });
        return *it;
    };

    delayer<std::chrono::microseconds> delayer;

    delayer.start();

    std::vector<int> counts(replays.size(), 0);

    while (!(*stop)) {
        if (loop && all_loops_complete()) break;

        int id = next_replay_id();

        if (!replays[id]->send(delayer)) {
            ++loops[id];
            replays[id]->reset();
            if (loop && loops[id] == loop) {
                alive.erase(std::remove(alive.begin(), alive.end(), id));
            }
        }
    }
}

TEST_P(UdpQueuePcapTest, single_client_test) {
    // TODO: reenable once we figure out determinism
    GTEST_SKIP();

    auto data_dir = getenvs("DATA_DIR");
    const auto test_params = GetParam();
    auto info = metadata_from_json(data_dir + "/" + std::get<1>(test_params));
    auto pf = packet_format(info);

    std::vector<LidarPacket> lidar_packets;
    std::vector<ImuPacket> imu_packets;
    {  // collect packets
        PcapReader pcap(data_dir + "/" + std::get<0>(test_params));
        while (pcap.next_packet()) {
            if (pcap.current_info().dst_port == info.udp_port_lidar) {
                LidarPacket p(pcap.current_length());
                std::memcpy(p.buf.data(), pcap.current_data(), p.buf.size());
                lidar_packets.push_back(std::move(p));
            }
            if (pcap.current_info().dst_port == info.udp_port_imu) {
                ImuPacket p(pcap.current_length());
                std::memcpy(p.buf.data(), pcap.current_data(), p.buf.size());
                imu_packets.push_back(std::move(p));
            }
        }
    }

    size_t lidar_buf_size = 640;
    size_t imu_buf_size = 100;

    int lidar_port = 50001;
    int imu_port = 50002;

    BufferedUDPSource queue(init_client("127.0.0.1", lidar_port, imu_port),
                            lidar_buf_size, pf.lidar_packet_size, imu_buf_size,
                            pf.imu_packet_size);

    bool stop = false;
    std::vector<std::shared_ptr<PcapReplay>> replays;
    replays.push_back(std::make_shared<PcapReplay>(
        data_dir + "/" + std::get<0>(test_params), info.udp_port_lidar,
        info.udp_port_imu, lidar_port, imu_port));

    std::thread producer(&BufferedUDPSource::produce, &queue);
    std::thread sensor_emu(&replay, &stop, replays, 1);

    size_t lidar_packets_recv = 0;
    size_t imu_packets_recv = 0;
    while (true) {
        auto st = queue.pop(1.0);

        if (st == client_state::TIMEOUT) break;
        if (st == client_state::EXIT) break;
        if (static_cast<int>(st) == Producer::CLIENT_OVERFLOW) break;

        if (st == client_state::LIDAR_DATA) {
            Packet& p = queue.packet(st);
            EXPECT_EQ(p.buf, lidar_packets[lidar_packets_recv].buf);
            ++lidar_packets_recv;
        }
        if (st == client_state::IMU_DATA) {
            Packet& p = queue.packet(st);
            EXPECT_EQ(p.buf, imu_packets[imu_packets_recv].buf);
            ++imu_packets_recv;
        }

        queue.advance(st);

        if (lidar_packets_recv == lidar_packets.size() &&
            imu_packets_recv == imu_packets.size())
            break;
    }
    queue.shutdown();
    producer.join();
    sensor_emu.join();

    EXPECT_EQ(lidar_packets_recv, lidar_packets.size());
    EXPECT_EQ(imu_packets_recv, imu_packets.size());
}

TEST(UdpQueueTest, multi_client_test) {
    // TODO: reenable once we figure out determinism
    GTEST_SKIP();

    auto data_dir = getenvs("DATA_DIR");

    std::vector<str_pair> inputs = {
        str_pair{"OS-0-128-U1_v2.3.0_1024x10.pcap",
                 "OS-0-128-U1_v2.3.0_1024x10.json"},
        str_pair{"OS-0-32-U1_v2.2.0_1024x10.pcap",
                 "OS-0-32-U1_v2.2.0_1024x10.json"},
        str_pair{"OS-1-128_767798045_1024x10_20230712_120049.pcap",
                 "OS-1-128_767798045_1024x10_20230712_120049.json"},
        str_pair{"OS-1-128_v2.3.0_1024x10_lb_n3.pcap",
                 "OS-1-128_v2.3.0_1024x10.json"},
        str_pair{"OS-2-128-U1_v2.3.0_1024x10.pcap",
                 "OS-2-128-U1_v2.3.0_1024x10.json"}};

    // set up reference
    RingBufferMap<Event, Packet> orig_packets;
    for (int i = 0, end = inputs.size(); i < end; ++i) {
        auto&& p = inputs[i];

        PcapReader pcap(data_dir + "/" + p.first);
        auto info = metadata_from_json(data_dir + "/" + p.second);
        auto&& pf = get_format(info);

        int n_lidar = 0, n_imu = 0;
        while (pcap.next_packet()) {
            if (pcap.current_info().dst_port == info.udp_port_lidar) ++n_lidar;
            if (pcap.current_info().dst_port == info.udp_port_imu) ++n_imu;
        }
        Event e_lidar{i, client_state::LIDAR_DATA};
        Event e_imu{i, client_state::IMU_DATA};
        orig_packets.allocate(e_lidar, n_lidar + 1,
                              Packet(pf.lidar_packet_size));
        orig_packets.allocate(e_imu, n_imu + 1, Packet(pf.imu_packet_size));
        pcap.reset();

        while (pcap.next_packet()) {
            if (pcap.current_info().dst_port == info.udp_port_lidar) {
                std::memcpy(orig_packets.back(e_lidar).buf.data(),
                            pcap.current_data(),
                            orig_packets.back(e_lidar).buf.size());
                orig_packets.push(e_lidar);
            }
            if (pcap.current_info().dst_port == info.udp_port_imu) {
                std::memcpy(orig_packets.back(e_imu).buf.data(),
                            pcap.current_data(),
                            orig_packets.back(e_imu).buf.size());
                orig_packets.push(e_imu);
            }
        }
    }

    int port = 50000;

    std::vector<std::shared_ptr<PcapReplay>> replays;
    bool replay_stop = false;
    Producer producer;
    std::vector<std::shared_ptr<Subscriber>> subs;
    for (auto&& p : inputs) {
        int lidar_port = port++;
        int imu_port = port++;
        auto info = metadata_from_json(data_dir + "/" + p.second);
        replays.push_back(std::make_shared<PcapReplay>(
            data_dir + "/" + p.first, info.udp_port_lidar, info.udp_port_imu,
            lidar_port, imu_port));

        auto id = producer.add_client(
            init_client("localhost", lidar_port, imu_port), info, 1.f);

        subs.push_back(producer.subscribe(
            EventSet{{-1, client_state::EXIT},
                     {-1, client_state::CLIENT_ERROR},
                     {id, client_state::LIDAR_DATA},
                     {id, client_state::IMU_DATA},
                     {id, client_state(Producer::CLIENT_OVERFLOW)}}));
    }

    auto consumer = [&orig_packets](std::shared_ptr<Subscriber> sub) {
        while (true) {
            auto e = sub->pop(1.0);
            auto st = e.state;

            if (st == client_state::TIMEOUT) break;
            if (st == client_state::EXIT) break;
            if (st == client_state::CLIENT_ERROR) break;
            if (static_cast<int>(st) == Producer::CLIENT_OVERFLOW) break;

            if (st == client_state::LIDAR_DATA ||
                st == client_state::IMU_DATA) {
                EXPECT_EQ(sub->packet(e).buf, orig_packets.front(e).buf);
                orig_packets.pop(e);
                sub->advance(e);
            }
        }
    };

    std::thread sensor_emu(&replay, &replay_stop, replays, 1);
    std::thread producer_thread(&Producer::run, &producer);
    std::vector<std::thread> consumers;
    for (auto&& sub : subs) consumers.emplace_back(consumer, sub);

    sensor_emu.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    producer.shutdown();
    producer_thread.join();
    for (auto&& c : consumers) c.join();

    for (int i = 0, end = inputs.size(); i < end; ++i) {
        EXPECT_EQ(orig_packets.size({i, client_state::LIDAR_DATA}), 0);
        EXPECT_EQ(orig_packets.size({i, client_state::IMU_DATA}), 0);
    }
}
