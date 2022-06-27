# Try to find a cmake config compatible with vcpkg and fall back to defining
# targets using pkgconfig.

include(FindPackageHandleStandardArgs)

if(CONAN_EXPORTED)
  find_package(pcapplusplus REQUIRED)

  get_target_property(_LIBS pcapplusplus::pcapplusplus INTERFACE_LINK_LIBRARIES)
  LIST(REMOVE_ITEM _LIBS CONAN_LIB::pcapplusplus_Pcap++)
  LIST(REMOVE_ITEM _LIBS libpcap::libpcap)
  set_property(TARGET pcapplusplus::pcapplusplus PROPERTY INTERFACE_LINK_LIBRARIES ${_LIBS})

  set(PCAPPLUSPLUS_LIBS pcapplusplus::pcapplusplus)
elseif(NOT Pcapplusplus_FOUND AND NOT pcapplusplus_FOUND)
  if(WIN32)
    SET(SUFFIX .lib)
    SET(PREFIX "")
  else()
    SET(SUFFIX .a)
    SET(PREFIX "lib")
  endif()
  if(DEFINED VCPKG_INSTALLED_DIR)
    message("VCPKG Found, searching for Pcapplusplus")
    string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_lower )
    if(cmake_lower STREQUAL "debug")
      set(POST_FIX "debug/lib")
    else()
      set(POST_FIX "lib")
    endif()
    set(PCAPPLUSPLUS_LIB_SEARCH_DIR ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/${POST_FIX})
    set(PCAPPLUSPLUS_INCLUDE_SEARCH_DIR ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET})
  else()
    message("VCPKG NOT Found, using pkgconfig to search for Pcapplusplus")
    find_package(PkgConfig QUIET REQUIRED)
    pkg_check_modules(PC_Pcapplusplus REQUIRED PcapPlusPlus)
    set(PCAPPLUSPLUS_LIB_SEARCH_DIR "${PC_Pcapplusplus_LIBRARY_DIRS}")
    set(PCAPPLUSPLUS_INCLUDE_SEARCH_DIR "${PC_Pcapplusplus_INCLUDE_DIRS}")
  endif()
  set(PCAPPLUSPLUS_COMMON_STATIC_LIBRARY "${PCAPPLUSPLUS_LIB_SEARCH_DIR}/${PREFIX}Common++${SUFFIX}")
  message("${PCAPPLUSPLUS_COMMON_STATIC_LIBRARY}")
  message("${PCAPPLUSPLUS_PACKET_STATIC_LIBRARY}")
  set(PCAPPLUSPLUS_PACKET_STATIC_LIBRARY "${PCAPPLUSPLUS_LIB_SEARCH_DIR}/${PREFIX}Packet++${SUFFIX}")
  find_path(PCAPPLUSPLUS_INCLUDE_DIR 
    NAMES Packet.h
    PATHS ${PCAPPLUSPLUS_INCLUDE_SEARCH_DIR})

  if(NOT TARGET Pcapplusplus::Packet)
    message("-- Found Pcapplusplus Common: ${PCAPPLUSPLUS_COMMON_STATIC_LIBRARY}")
    add_library(Pcapplusplus::Packet INTERFACE IMPORTED)
    set_target_properties(Pcapplusplus::Packet PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${PCAPPLUSPLUS_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${PCAPPLUSPLUS_PACKET_STATIC_LIBRARY}"
    )
  endif ()
  if(NOT TARGET Pcapplusplus::Common)
    message("-- Found Pcapplusplus Packet: ${PCAPPLUSPLUS_PACKET_STATIC_LIBRARY}")
    add_library(Pcapplusplus::Common INTERFACE IMPORTED)
    set_target_properties(Pcapplusplus::Common PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${PCAPPLUSPLUS_INCLUDE_DIRS}"
      INTERFACE_LINK_LIBRARIES "${PCAPPLUSPLUS_COMMON_STATIC_LIBRARY}"
    )
  endif ()
  set(PCAPPLUSPLUS_LIBS Pcapplusplus::Packet Pcapplusplus::Common)
endif()

