/**
 * @file
 * @brief ouster_pyclient_pcap python module
 */
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

#include "ouster/os_pcap.h"
// Disabled until buffers are replaced
//#include "ouster/ouster_pybuffer.h"
#include <pcap/pcap.h>

#include <sstream>

namespace py = pybind11;

void handler(int) { std::quick_exit(EXIT_SUCCESS); }

/// @TODO The pybind11 buffer_info struct is actually pretty heavy-weight – it
/// includes some string fields, vectors, etc. Might be worth it to avoid
/// calling this twice.
/*
 * Check that buffer is a 1-d byte array of size > bound and return an internal
 * pointer to the data for writing. Check is strictly greater to account for the
 * extra byte required to determine if a datagram is bigger than expected.
 */
inline uint8_t* getptr(py::buffer& buf) {
    auto info = buf.request();
    if (info.format != py::format_descriptor<uint8_t>::format()) {
        throw std::invalid_argument(
            "Incompatible argument: expected a bytearray");
    }
    return (uint8_t*)info.ptr;
}
/// @TODO The pybind11 buffer_info struct is actually pretty heavy-weight – it
/// includes some string fields, vectors, etc. Might be worth it to avoid
/// calling this twice.
/*
 * Return the size of the python buffer
 */
inline size_t getptrsize(py::buffer& buf) {
    auto info = buf.request();

    return (size_t)info.size;
}

// hack: Wrap replay to allow terminating via SIGINT
int replay_pcap(const std::string& file, const std::string& src_ip,
                const std::string& dest_ip, double rate) {
    py::gil_scoped_release release;
    auto py_handler = std::signal(SIGINT, handler);
    auto res = ouster::sensor_utils::replay(file, src_ip, dest_ip, rate);
    std::signal(SIGINT, py_handler);

    return res;
}

// Record functionality removed for a short amount of time
// until we switch it over to support libtins

PYBIND11_MODULE(_pcap, m) {
    m.doc() = "ouster._pcap"; // optional module docstring
    
    py::class_<ouster::sensor_utils::stream_info,
               std::shared_ptr<ouster::sensor_utils::stream_info>>(m, "stream_info")
        .def(py::init<>())
        .def("__repr__", [](const ouster::sensor_utils::stream_info& data) {
                             std::stringstream result;
                             result << data;
                             return result.str();})
        .def_readonly("packets_processed", &ouster::sensor_utils::stream_info::packets_processed)
        .def_readonly("packets_reassembled", &ouster::sensor_utils::stream_info::packets_reassembled)
        .def_readonly("port_to_packet_sizes", &ouster::sensor_utils::stream_info::port_to_packet_sizes)
        .def_readonly("port_to_packet_count", &ouster::sensor_utils::stream_info::port_to_packet_count)
        .def_readonly("packet_size_to_port", &ouster::sensor_utils::stream_info::packet_size_to_port);

    py::class_<ouster::sensor_utils::port_couple>(m, "port_couple")
        .def(py::init<>())
        .def("__repr__", [](const ouster::sensor_utils::port_couple& data) {
                             std::stringstream result;
                             result << data;
                             return result.str();})
        .def_readwrite("lidar_port", &ouster::sensor_utils::port_couple::lidar_port)
        .def_readwrite("imu_port", &ouster::sensor_utils::port_couple::imu_port);
    
    py::class_<ouster::sensor_utils::playback_handle,
               std::shared_ptr<ouster::sensor_utils::playback_handle>>(m, "playback_handle")
        .def(py::init<>());
    
    m.def("replay_pcap", &replay_pcap);
    m.def("replay_get_pcap_info", &ouster::sensor_utils::replay_get_pcap_info, py::return_value_policy::reference);
    m.def("guess_ports",
          [](const ouster::sensor_utils::stream_info& stream_data) -> ouster::sensor_utils::port_couple {
              py::gil_scoped_release release;
              return ouster::sensor_utils::guess_ports(stream_data);
          });
    m.def("replay_initalize",
          (std::shared_ptr<ouster::sensor_utils::playback_handle>(*)(
              const std::string&, const std::string&, const std::string&,
              const ouster::sensor_utils::port_couple&, int, int)) &
              ouster::sensor_utils::replay_initalize);
    m.def("replay_uninitialize",
          [](std::shared_ptr<ouster::sensor_utils::playback_handle> handle) {
              py::gil_scoped_release release;
              ouster::sensor_utils::replay_uninitialize(*handle);
          });
    m.def("replay_next_lidar_packet",
          [](std::shared_ptr<ouster::sensor_utils::playback_handle> handle) -> bool {
              py::gil_scoped_release release;
              return ouster::sensor_utils::replay_next_lidar_packet(*handle);
          });
    m.def("replay_next_imu_packet",
          [](std::shared_ptr<ouster::sensor_utils::playback_handle> handle) -> bool {
              py::gil_scoped_release release;
              return ouster::sensor_utils::replay_next_imu_packet(*handle);
          });
    m.def("get_next_lidar_data",
          [](std::shared_ptr<ouster::sensor_utils::playback_handle> handle,
                  py::buffer buf) -> bool { py::gil_scoped_release release; return
              ouster::sensor_utils::get_next_lidar_data(*handle, getptr(buf), getptrsize(buf));
          });
    m.def("get_next_imu_data",
          [](std::shared_ptr<ouster::sensor_utils::playback_handle> handle,
                  py::buffer buf) -> bool { py::gil_scoped_release release; return
              ouster::sensor_utils::get_next_imu_data(*handle, getptr(buf), getptrsize(buf));
          });
}
