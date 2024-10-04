#pragma once

// Common make opaques for each submodule. We need to include this in each.
PYBIND11_MAKE_OPAQUE(std::vector<uint8_t>);
PYBIND11_MAKE_OPAQUE(std::map<uint64_t, uint64_t>);
