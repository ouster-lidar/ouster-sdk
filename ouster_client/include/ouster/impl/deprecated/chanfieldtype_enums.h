#pragma once

namespace ouster {
namespace sdk {
namespace core {

// clang-format on
#if defined(VOID)
#define OUSTER_REMOVED_VOID
#pragma push_macro("VOID")
#undef VOID
#endif

/**
 * Types of channel fields.
 */
/// @cond DOXYGEN_SHOULD_SKIP_THIS
OUSTER_DEPRECATED_CONSTEXP(VOID, ChanFieldType::VOID,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(UINT8, ChanFieldType::UINT8,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(UINT16, ChanFieldType::UINT16,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(UINT32, ChanFieldType::UINT32,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(UINT64, ChanFieldType::UINT64,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(INT8, ChanFieldType::INT8,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(INT16, ChanFieldType::INT16,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(INT32, ChanFieldType::INT32,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(INT64, ChanFieldType::INT64,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(FLOAT32, ChanFieldType::FLOAT32,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(FLOAT64, ChanFieldType::FLOAT64,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(CHAR, ChanFieldType::CHAR,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(ZONE_STATE, ChanFieldType::ZONE_STATE,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
OUSTER_DEPRECATED_CONSTEXP(UNREGISTERED, ChanFieldType::UNREGISTERED,
                           OUSTER_DEPRECATED_LAST_SUPPORTED_0_16);
/// @endcond
#if defined(OUSTER_REMOVED_VOID)
#pragma pop_macro("VOID")
#undef OUSTER_REMOVED_VOID
#endif

}  // namespace core
}  // namespace sdk
}  // namespace ouster