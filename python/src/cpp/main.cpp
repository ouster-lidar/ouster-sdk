
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void init_client(py::module&, py::module&);
void init_pcap(py::module&, py::module&);
void init_osf(py::module&, py::module&);
void init_viz(py::module&, py::module&);

PYBIND11_MODULE(_bindings, m) {
    m.doc() = R"(
    SDK bindings generated by pybind11.

    This module is generated directly from the C++ code and not meant to be used
    directly.
    )";

    auto pcap = m.def_submodule("pcap");
    init_pcap(pcap, m);
    auto client = m.def_submodule("client");
    init_client(client, m);
    auto osf = m.def_submodule("osf");
    init_osf(osf, m);
    auto viz = m.def_submodule("viz");
    init_viz(viz, m);
}
