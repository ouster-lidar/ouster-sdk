#pragma once

// clang-format off
#define OUSTER_DEPRECATED_MSG(new_symbol, deprecation_version)             \
    [[deprecated(                                                          \
        "Use " #new_symbol                                                 \
        " instead. "                                                       \
        "The last supported version for this will be " deprecation_version \
        ". ")]]

#define OUSTER_DEPRECATED_TYPE(old_symbol, new_symbol, deprecation_version) \
    /**                                                                     \
     * @deprecated This symbol is deprecated, please                        \
     * check compiler warnings for replacement                              \
     */                                                                     \
    OUSTER_DEPRECATED_MSG(new_symbol, deprecation_version)                  \
    typedef new_symbol old_symbol;  // NOLINT(modernize-use-using)

#if defined(__GNUC__) || defined(__clang__)
#define OUSTER_DIAGNOSTIC_PUSH _Pragma("GCC diagnostic push")
#define OUSTER_DIAGNOSTIC_POP _Pragma("GCC diagnostic pop")
#define OUSTER_DIAGNOSTIC_IGNORE_UNUSED \
    _Pragma("GCC diagnostic ignored \"-Wunused-variable\"")
#define OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#define OUSTER_IGNORE_ATTRIBUTE \
    _Pragma("GCC diagnostic ignored \"-Wattributes\"")
#elif defined(_MSC_VER)
#define OUSTER_DIAGNOSTIC_PUSH __pragma(warning(push))
#define OUSTER_DIAGNOSTIC_POP __pragma(warning(pop))
#define OUSTER_DIAGNOSTIC_IGNORE_UNUSED \
    __pragma(warning(disable : 4101))  // 4101: unreferenced local variable
#define OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED \
    __pragma(warning(disable : 4996))  // 4996: item was declared deprecated
#define OUSTER_IGNORE_ATTRIBUTE
#else
#define OUSTER_DIAGNOSTIC_PUSH
#define OUSTER_DIAGNOSTIC_POP
#define OUSTER_DIAGNOSTIC_IGNORE_UNUSED
#define OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED
#define OUSTER_IGNORE_ATTRIBUTE
#endif

#define OUSTER_DEPRECATED_VALUE(old_symbol, new_symbol, deprecation_version) \
    OUSTER_DIAGNOSTIC_PUSH                                                   \
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED                                          \
    OUSTER_DEPRECATED_MSG(new_symbol, deprecation_version)                   \
    auto old_symbol = new_symbol;                                            \
    OUSTER_DIAGNOSTIC_POP

#define OUSTER_DEPRECATED_CONSTEXP(old_symbol, new_symbol, deprecation_version) \
    OUSTER_DIAGNOSTIC_PUSH                                                      \
    OUSTER_DIAGNOSTIC_IGNORE_UNUSED                                             \
    OUSTER_DEPRECATED_MSG(new_symbol, deprecation_version)                      \
    constexpr auto old_symbol = new_symbol;                                     \
    OUSTER_DIAGNOSTIC_POP

#define OUSTER_DEPRECATED_ENUM_ENTRY(old_symbol, new_symbol, deprecation_version)   \
    OUSTER_DEPRECATED_MSG(new_symbol, deprecation_version)                          \
    old_symbol = new_symbol

#define OUSTER_DEPRECATED_ENUM_CLASS_ENTRY(old_symbol, new_symbol, deprecation_version)   \
    old_symbol OUSTER_DEPRECATED_MSG(new_symbol, deprecation_version) = new_symbol

#define OUSTER_DEPRECATED_LAST_SUPPORTED_0_16 "0.16"
// clang-format on
