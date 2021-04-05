/**
 * @file
 * @brief ouster_pyclient python module
 *
 * Note: the type annotations in `sensor.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

using fsec = std::chrono::duration<float>;
namespace py = pybind11;

using ouster::sensor::data_format;
using ouster::sensor::packet_format;
using ouster::sensor::sensor_info;
using namespace ouster;

/*
 * Extra bit flag compatible with client_state used to signal buffer overflow.
 */
constexpr int CLIENT_OVERFLOW = 0x10;

/*
 * Check that buffer is a 1-d byte array of size > bound and return an internal
 * pointer to the data for writing. Check is strictly greater to account for the
 * extra byte required to determine if a datagram is bigger than expected.
 */
inline uint8_t* getptr(size_t bound, py::buffer& buf) {
    auto info = buf.request();
    if (info.format != py::format_descriptor<uint8_t>::format() ||
        info.ndim != 1 || info.size < bound) {
        throw std::invalid_argument(
            "Incompatible argument: expected a bytearray of size >= " +
            std::to_string(bound));
    }
    return (uint8_t*)info.ptr;
}

// 32K max packet size, shoould be big enough for 128
constexpr size_t packet_size = 32768;

/*
 * Wrapper around the lower level C++ API. Must be thread-safe to allow reading
 * data without holding the GIL while other references to the client exist.
 * Provides a large single-producer / single-consumer circular buffer that can
 * be populated by a thread without holding the GIL to deal the relatively small
 * default OS buffer size and high sensor UDP data rate.
 *
 * Locks are not held during the actual buffer reads and writes: thread safety
 * w.r.t the producer relies on the invariants that only the consumer modifies
 * the read index and only the producer modifies the write index, always while
 * holding cv_mtx_ to make sure that cv notifications are not lost between
 * checking the empty/full condition and entering the waiting state.
 */
struct PyClient {
    // native client handle
    std::mutex cli_mtx_;
    std::shared_ptr<ouster::sensor::client> cli_;

    // protect read/write_ind_ and stop_
    std::mutex cv_mtx_;
    std::condition_variable cv_;
    size_t read_ind_{0}, write_ind_{0};

    // flag for other threads to signal producer to shut down
    bool stop_{false};

    // internal packet buffer
    size_t capacity_{0};
    using entry = std::pair<sensor::client_state, std::unique_ptr<uint8_t[]>>;
    std::vector<entry> bufs_;

    /*
     * Initialize the internal circular buffer.
     *
     * NOTE: capacity_ is the capacity of the internal buffer vector, not the
     * max number of buffered packets (which is one less).
     */
    explicit PyClient(size_t buf_size) : capacity_{buf_size + 1} {
        std::generate_n(std::back_inserter(bufs_), capacity_, [&] {
            return std::make_pair(
                sensor::CLIENT_ERROR,
                std::unique_ptr<uint8_t[]>{new uint8_t[packet_size]});
        });
    }

    PyClient(const std::string& hostname, int lidar_port, int imu_port,
             size_t buf_size)
        : PyClient(buf_size) {
        cli_ = sensor::init_client(hostname, lidar_port, imu_port);
        if (!cli_) throw std::runtime_error("Failed to initialize client");
    }

    PyClient(const std::string& hostname, const std::string& udp_dest_host,
             sensor::lidar_mode mode, sensor::timestamp_mode ts_mode,
             int lidar_port, int imu_port, int timeout_sec, size_t buf_size)
        : PyClient(buf_size) {
        cli_ = sensor::init_client(hostname, udp_dest_host, mode, ts_mode,
                                   lidar_port, imu_port);
        if (!cli_) throw std::runtime_error("Failed to initialize client");
    }

    /*
     * Fetch metadata using the native client handle.
     */
    std::string get_metadata(int timeout_sec) {
        std::lock_guard<std::mutex> cli_lock{cli_mtx_};
        if (!cli_)
            throw std::runtime_error("Client has already been shut down");
        return sensor::get_metadata(*cli_, timeout_sec);
    }

    /*
     * Signal the producer to exit. Subsequent calls to consume() will return
     * CLIENT_EXIT instead of blocking. Multiple calls to shutdown() are not an
     * error.
     *
     * Invariant: nothing can access cli_ when stop_ is true. Producer will
     * release _cli_mtx_ only when it exits the loop.
     */
    void shutdown() {
        {
            std::unique_lock<std::mutex> lock{cv_mtx_};
            if (stop_) return;
            stop_ = true;
        }
        cv_.notify_all();

        // close UDP sockets when any producer has exited
        std::lock_guard<std::mutex> cli_lock{cli_mtx_};
        cli_.reset();
    }

    /*
     * Advance the read index to drop data. Drop all internally buffered data
     * when n_packets = 0.
     *
     * Can only be called by the consumer (also while holding the GIL) to
     * maintain the invariant that only the reader modifies the read index.
     */
    void flush(size_t n_packets) {
        {
            std::unique_lock<std::mutex> lock{cv_mtx_};
            auto sz = (capacity_ + write_ind_ - read_ind_) % capacity_;
            auto n = (n_packets == 0) ? sz : std::min(sz, n_packets);
            read_ind_ = (capacity_ + read_ind_ + n) % capacity_;
        }
        cv_.notify_one();
    }

    /*
     * Get current buffer size in no. packets.
     */
    size_t size() {
        std::unique_lock<std::mutex> lock{cv_mtx_};
        return (capacity_ + write_ind_ - read_ind_) % capacity_;
    }

    /*
     * Get the maximum buffer size in no. packets.
     *
     * Note: one less than the size of the internal bufs_ vector
     */
    size_t capacity() { return (capacity_ - 1); }

    /*
     * Read next available buffer in the queue and advance the read index. Block
     * if the queue is empty for up to `timeout_sec` (zero means wait forever).
     *
     * This must be called while holding the GIL to ensure that only a single
     * consumer runs at a time.
     */
    sensor::client_state consume(py::buffer buf, float timeout_sec) {
        // wait for producer to wake us up if the queue is empty
        {
            std::unique_lock<std::mutex> lock{cv_mtx_};
            bool timeout = !cv_.wait_for(lock, fsec{timeout_sec}, [this] {
                return stop_ || write_ind_ != read_ind_;
            });
            if (timeout)
                return sensor::TIMEOUT;
            else if (stop_)
                return sensor::EXIT;
        }

        // read data into buffer
        auto info = buf.request();
        auto sz = std::min<size_t>(info.size, packet_size);
        auto& e = bufs_[read_ind_];
        std::memcpy(info.ptr, e.second.get(), sz);

        // advance read ind and unblock producer, if necessary
        {
            std::unique_lock<std::mutex> lock{cv_mtx_};
            read_ind_ = (read_ind_ + 1) % capacity_;
        }
        cv_.notify_one();
        return e.first;
    }

    /*
     * Release the GIL and write data from the network into the circular buffer
     * until shutdown() is signaled by the reader.
     *
     * If reading from the network was blocked because the buffer was full, set
     * the CLIENT_OVERFLOW flag on the status of the following packet.
     *
     * Hold the client mutex to protect client state and prevent multiple
     * producers from running in parallel
     */
    sensor::client_state produce(const packet_format& pf) {
        py::gil_scoped_release release;
        std::lock_guard<std::mutex> cli_lock{cli_mtx_};

        auto exit_mask =
            sensor::client_state(sensor::CLIENT_ERROR | sensor::EXIT);
        auto st = sensor::client_state(0);

        while (!(st & exit_mask)) {
            // Wait for consumer to wake us up if the queue is full
            bool overflow = false;
            {
                std::unique_lock<std::mutex> lock{cv_mtx_};
                while (!stop_ && (write_ind_ + 1) % capacity_ == read_ind_) {
                    overflow = true;
                    cv_.wait(lock);
                }
                if (stop_) return sensor::EXIT;
            }

            // Write data and status to circular buffer. EXIT and ERROR status
            // are just passed through with stale data.
            st = sensor::poll_client(*cli_);
            if (st == sensor::TIMEOUT) continue;

            auto& e = bufs_[write_ind_];
            if (st & sensor::LIDAR_DATA) {
                if (!sensor::read_lidar_packet(*cli_, e.second.get(), pf))
                    st = sensor::client_state(st | sensor::CLIENT_ERROR);
            } else if (st & sensor::IMU_DATA) {
                if (!sensor::read_imu_packet(*cli_, e.second.get(), pf))
                    st = sensor::client_state(st | sensor::CLIENT_ERROR);
            }
            if (overflow) st = sensor::client_state(st | CLIENT_OVERFLOW);
            e.first = st;

            // Advance write ind and wake up consumer, if blocked
            {
                std::unique_lock<std::mutex> lock{cv_mtx_};
                write_ind_ = (write_ind_ + 1) % capacity_;
            }
            cv_.notify_one();
        }

        return st;
    }
};

PYBIND11_MODULE(_sensor, m) {
    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    m.doc() = R"(Sensor client bindings generated by pybind11.

This module is generated directly from the C++ code and not meant to be used
directly.
)";

    // clang-format off

    // Packet Format
    py::class_<packet_format>(m, "PacketFormat")
        .def(py::init([](const sensor_info& info) { return sensor::get_format(info); }))
        .def_readonly("lidar_packet_size", &packet_format::lidar_packet_size)
        .def_readonly("imu_packet_size", &packet_format::imu_packet_size)
        .def_readonly("columns_per_packet", &packet_format::columns_per_packet)
        .def_readonly("pixels_per_column", &packet_format::pixels_per_column)
        .def_readonly("encoder_ticks_per_rev", &packet_format::encoder_ticks_per_rev)

        // Measurement block accessors
        .def("col_timestamp", [](packet_format& pf, int col, py::buffer buf) {
            return pf.col_timestamp(pf.nth_col(col, getptr(pf.lidar_packet_size, buf)));
        })
        .def("col_encoder", [](packet_format& pf, int col, py::buffer buf) {
            return pf.col_encoder(pf.nth_col(col, getptr(pf.lidar_packet_size, buf)));
        })
        .def("col_measurement_id", [](packet_format& pf, int col, py::buffer buf) {
            return pf.col_measurement_id(pf.nth_col(col, getptr(pf.lidar_packet_size, buf)));
        })
        .def("col_frame_id", [](packet_format& pf, int col, py::buffer buf) {
            return pf.col_frame_id(pf.nth_col(col, getptr(pf.lidar_packet_size, buf)));
        })
        .def("col_status", [](packet_format& pf, int col, py::buffer buf) {
            return pf.col_status(pf.nth_col(col, getptr(pf.lidar_packet_size, buf)));
        })

        // IMU packet accessors
        .def("imu_sys_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_sys_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_accel_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_accel_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_gyro_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_gyro_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_x", [](packet_format& pf, py::buffer buf) { return pf.imu_av_x(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_y", [](packet_format& pf, py::buffer buf) { return pf.imu_av_y(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_z", [](packet_format& pf, py::buffer buf) { return pf.imu_av_z(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_x", [](packet_format& pf, py::buffer buf) { return pf.imu_la_x(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_y", [](packet_format& pf, py::buffer buf) { return pf.imu_la_y(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_z", [](packet_format& pf, py::buffer buf) { return pf.imu_la_z(getptr(pf.imu_packet_size, buf)); });

    // Data Format
    py::class_<data_format>(m, "DataFormat")
        .def(py::init<>())
        .def_readwrite("pixels_per_column", &data_format::pixels_per_column)
        .def_readwrite("columns_per_packet", &data_format::columns_per_packet)
        .def_readwrite("columns_per_frame", &data_format::columns_per_frame)
        .def_readwrite("pixel_shift_by_row", &data_format::pixel_shift_by_row);

    // Sensor Info
    py::class_<sensor_info>(m, "SensorInfo", R"(
        Sensor metadata required to interpret UDP data streams.
        )")
        .def(py::init<>(), R"(
        Args:
            raw (str): json string to parse
        )")
        .def(py::init([](const std::string& s) { return sensor::parse_metadata(s); }))
        .def_readwrite("hostname", &sensor_info::name)
        .def_readwrite("sn", &sensor_info::sn)
        .def_readwrite("fw_rev", &sensor_info::fw_rev)
        .def_readwrite("mode", &sensor_info::mode)
        .def_readwrite("prod_line", &sensor_info::prod_line)
        .def_readwrite("format", &sensor_info::format)
        .def_readwrite("beam_azimuth_angles", &sensor_info::beam_azimuth_angles)
        .def_readwrite("beam_altitude_angles", &sensor_info::beam_altitude_angles)
        .def_readwrite("imu_to_sensor_transform", &sensor_info::imu_to_sensor_transform)
        .def_readwrite("lidar_to_sensor_transform", &sensor_info::lidar_to_sensor_transform)
        .def_readwrite("lidar_origin_to_beam_origin_mm", &sensor_info::lidar_origin_to_beam_origin_mm)
        .def_readwrite("extrinsic", &sensor_info::extrinsic)
        .def_static("from_default", &sensor::default_sensor_info, R"(
        Create gen-1 OS-1-64 metadata populated with design values.
        )")
        .def("__eq__", [](const sensor_info& i, const sensor_info& j) { return i == j; })
        .def("__str__", [](const sensor_info& i) { return to_string(i); })
        .def("__repr__", [](const sensor_info& info) {
            return "<ouster.client.SensorInfo " + info.prod_line + " " +
                info.sn + " " + info.fw_rev + " " + to_string(info.mode) + ">";
        });


    // Lidar Mode
    auto lidar_mode = py::enum_<sensor::lidar_mode>(m, "LidarMode")
        .value("MODE_UNSPEC", sensor::lidar_mode::MODE_UNSPEC)
        .value("MODE_512x10", sensor::lidar_mode::MODE_512x10)
        .value("MODE_512x20", sensor::lidar_mode::MODE_512x20)
        .value("MODE_1024x10", sensor::lidar_mode::MODE_1024x10)
        .value("MODE_1024x20", sensor::lidar_mode::MODE_1024x20)
        .value("MODE_2048x10", sensor::lidar_mode::MODE_2048x10)
        .def_property_readonly("cols", [](const sensor::lidar_mode& self) {
            return sensor::n_cols_of_lidar_mode(self); })
        .def_property_readonly("frequency", [](const sensor::lidar_mode& self) {
            return sensor::frequency_of_lidar_mode(self); })
        .def_static("from_string", &sensor::lidar_mode_of_string);
    // workaround for https://github.com/pybind/pybind11/issues/2537
    lidar_mode.attr("__str__") = py::cpp_function([](const sensor::lidar_mode& u) { return to_string(u); },
                                                  py::name("__str__"),
                                                  py::is_method(lidar_mode));

    // Timestamp Mode
    auto timestamp_mode = py::enum_<sensor::timestamp_mode>(m, "TimestampMode")
        .value("TIME_FROM_UNSPEC", sensor::timestamp_mode::TIME_FROM_UNSPEC)
        .value("TIME_FROM_INTERNAL_OSC", sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC)
        .value("TIME_FROM_SYNC_PULSE_IN", sensor::timestamp_mode::TIME_FROM_SYNC_PULSE_IN)
        .value("TIME_FROM_PTP_1588", sensor::timestamp_mode::TIME_FROM_PTP_1588)
        .def_static("from_string", &sensor::timestamp_mode_of_string);
    // workaround for https://github.com/pybind/pybind11/issues/2537
    timestamp_mode.attr("__str__") = py::cpp_function([](const sensor::timestamp_mode& u) { return to_string(u); },
                                                      py::name("__str__"),
                                                      py::is_method(timestamp_mode));

    // Version Info
    py::class_<util::version>(m, "Version")
        .def(py::init<>())
        .def("__eq__", [](const util::version& u, const util::version& v) { return u == v; })
        .def("__lt__", [](const util::version& u, const util::version& v) { return u < v; })
        .def("__le__", [](const util::version& u, const util::version& v) { return u <= v; })
        .def("__str__", [](const util::version& u) { return to_string(u); })
        .def_readwrite("major", &util::version::major)
        .def_readwrite("minor", &util::version::minor)
        .def_readwrite("patch", &util::version::patch)
        .def_static("from_string", &util::version_of_string);

    m.attr("invalid_version") = util::invalid_version;

    m.attr("min_version") = sensor::min_version;

    // clang-format on

    // Client Handle
    py::enum_<sensor::client_state>(m, "ClientState", py::arithmetic())
        .value("TIMEOUT", sensor::client_state::TIMEOUT)
        .value("ERROR", sensor::client_state::CLIENT_ERROR)
        .value("LIDAR_DATA", sensor::client_state::LIDAR_DATA)
        .value("IMU_DATA", sensor::client_state::IMU_DATA)
        .value("EXIT", sensor::client_state::EXIT)
        // TODO: revisit including in C++ API
        .value("OVERFLOW", sensor::client_state(CLIENT_OVERFLOW));

    py::class_<PyClient>(m, "Client")
        .def(py::init<std::string, int, int, size_t>(),
             py::arg("hostname") = "", py::arg("lidar_port") = 7502,
             py::arg("imu_port") = 7503, py::arg("capacity") = 128)
        .def(
            py::init<std::string, std::string, sensor::lidar_mode,
                     sensor::timestamp_mode, int, int, int, size_t>(),
            py::arg("hostname"), py::arg("udp_dest_host"),
            py::arg("mode") = sensor::lidar_mode::MODE_1024x10,
            py::arg("ts_mode") = sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC,
            py::arg("lidar_port") = 0, py::arg("imu_port") = 0,
            py::arg("timeout_sec") = 30, py::arg("capacity") = 128)
        .def("get_metadata", &PyClient::get_metadata,
             py::arg("timeout_sec") = 30)
        .def("shutdown", &PyClient::shutdown)
        .def("consume", &PyClient::consume)
        .def("produce", &PyClient::produce)
        .def("flush", &PyClient::flush, py::arg("n_packets") = 0)
        .def_property_readonly("capacity", &PyClient::capacity)
        .def_property_readonly("size", &PyClient::size);

    // Scans
    py::class_<LidarScan::BlockHeader>(m, "BlockHeader")
        .def(py::init([](uint64_t ts, uint32_t encoder, uint32_t status) {
            return new LidarScan::BlockHeader{LidarScan::ts_t{ts}, encoder,
                                              status};
        }))
        .def_property_readonly("timestamp",
                               [](const LidarScan::BlockHeader& self) {
                                   return self.timestamp.count();
                               })
        .def_readonly("encoder", &LidarScan::BlockHeader::encoder)
        .def_readonly("status", &LidarScan::BlockHeader::status);

    py::class_<LidarScan>(m, "LidarScan")
        .def(py::init<size_t, size_t>())
        .def_readonly("w", &LidarScan::w)
        .def_readonly("h", &LidarScan::h)
        .def_readwrite("frame_id", &LidarScan::frame_id)
        .def_readwrite("headers", &LidarScan::headers)
        .def_property_readonly(
            "data",
            [](LidarScan& self) -> LidarScan::data_t& { return self.data; },
            py::return_value_policy::reference_internal);

    m.def("destagger", [](const ouster::img_t<LidarScan::raw_t>& field,
                          const sensor_info& sensor) {
        return ouster::destagger<LidarScan::raw_t>(
            field, sensor.format.pixel_shift_by_row);
    });

    py::class_<ScanBatcher>(m, "ScanBatcher")
        .def(py::init<int, packet_format>())
        .def("__call__", [](ScanBatcher& self, py::buffer& buf, LidarScan& ls) {
            uint8_t* ptr = getptr(self.pf.lidar_packet_size, buf);
            return self(ptr, ls);
        });

    // XYZ Projection
    py::class_<XYZLut>(m, "XYZLut")
        .def(py::init(
            [](const sensor_info& sensor) { return make_xyz_lut(sensor); }))
        .def("__call__", [](const XYZLut& self, const LidarScan& scan) {
            return cartesian(scan, self);
        });

    m.attr("__version__") = VERSION_INFO;
}
