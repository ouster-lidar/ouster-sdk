find_path(PCAP_ROOT_DIR
  NAMES include/pcap.h
)

find_path(PCAP_INCLUDE_DIR
  NAMES pcap.h
  HINTS ${PCAP_ROOT_DIR}/include
)

set(HINT_DIR ${PCAP_ROOT_DIR}/lib)

if(WIN32)
  find_library(PCAP_LIBRARY
    NAMES wpcap
    HINTS ${HINT_DIR})
else() 
  find_library(PCAP_LIBRARY
    NAMES pcap
    HINTS ${HINT_DIR})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Pcap DEFAULT_MSG
  PCAP_LIBRARY
  PCAP_INCLUDE_DIR
)

mark_as_advanced(
  PCAP_ROOT_DIR
  PCAP_INCLUDE_DIR
  PCAP_LIBRARY
)
