#include "ouster/chanfield.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ouster {
namespace sdk {
namespace core {

#if defined(VOID)
#define OUSTER_REMOVED_VOID
#pragma push_macro("VOID")
#undef VOID
#endif
std::string to_string(ChanFieldType field_type) {
    switch (field_type) {
        case ChanFieldType::VOID:
            return "VOID";
        case ChanFieldType::UINT8:
            return "UINT8";
        case ChanFieldType::UINT16:
            return "UINT16";
        case ChanFieldType::UINT32:
            return "UINT32";
        case ChanFieldType::UINT64:
            return "UINT64";
        case ChanFieldType::INT8:
            return "INT8";
        case ChanFieldType::INT16:
            return "INT16";
        case ChanFieldType::INT32:
            return "INT32";
        case ChanFieldType::INT64:
            return "INT64";
        case ChanFieldType::FLOAT32:
            return "FLOAT32";
        case ChanFieldType::FLOAT64:
            return "FLOAT64";
        case ChanFieldType::CHAR:
            return "CHAR";
        default:
            return "UNKNOWN";
    }
}
#if defined(OUSTER_REMOVED_VOID)
#pragma pop_macro("VOID")
#undef OUSTER_REMOVED_VOID
#endif

size_t field_type_size(ChanFieldType field_type) {
    switch (field_type) {
        case ChanFieldType::INT8:
        case ChanFieldType::UINT8:
            return 1;
        case ChanFieldType::INT16:
        case ChanFieldType::UINT16:
            return 2;
        case ChanFieldType::INT32:
        case ChanFieldType::UINT32:
        case ChanFieldType::FLOAT32:
            return 4;
        case ChanFieldType::INT64:
        case ChanFieldType::UINT64:
        case ChanFieldType::FLOAT64:
            return 8;
        default:
            return 0;
    }
}

uint64_t field_type_mask(ChanFieldType field_type) {
    switch (field_type_size(field_type)) {
        case 1:
            return 0xff;
        case 2:
            return 0xffff;
        case 4:
            return 0xffffffff;
        case 8:
            return 0xffffffffffffffff;
        default:
            throw std::runtime_error(
                "field_type_mask error: wrong ChanFieldType");
    }
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
