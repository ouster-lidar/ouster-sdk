find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_PCAP QUIET libpcap)
  if(PC_PCAP_FOUND)
    set(PCAP_VERSION_STRING ${PC_PCAP_VERSION})
  endif()
endif()

find_path(PCAP_INCLUDE_DIR
  NAMES pcap.h
  HINTS ${PC_PCAP_INCLUDE_DIRS})

find_library(PCAP_LIBRARY
  NAMES pcap pcap_static wpcap
  HINTS ${PC_PCAP_LIBRARY_DIRS})

if(NOT TARGET libpcap::libpcap)
  add_library(libpcap::libpcap UNKNOWN IMPORTED)
  set_target_properties(libpcap::libpcap PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${PCAP_INCLUDE_DIR}"
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${PCAP_LIBRARY}")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pcap
  REQUIRED_VARS PCAP_LIBRARY PCAP_INCLUDE_DIR
  VERSION_VAR PCAP_VERSION_STRING)

mark_as_advanced(
  PCAP_INCLUDE_DIR
  PCAP_LIBRARY)
