find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

set(_DOC_CPP_FEATURES_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(_DOC_CPP_SNIPPET_LIBS Eigen3::Eigen GTest::gtest)
set(_DOC_CPP_OPTIONAL_LIBS
    OusterSDK::ouster_core
    OusterSDK::ouster_mapping
    OusterSDK::ouster_osf
    OusterSDK::ouster_pcap
    OusterSDK::ouster_sensor
    OusterSDK::ouster_viz
)

foreach(_doc_lib IN LISTS _DOC_CPP_OPTIONAL_LIBS)
    if(TARGET ${_doc_lib})
        list(APPEND _DOC_CPP_SNIPPET_LIBS ${_doc_lib})
    else()
        message(STATUS "Skipping ${_doc_lib} - target not found")
    endif()
endforeach()

if(NOT CMAKE_TESTING_ENABLED)
    enable_testing()
endif()

function(_doc_cpp_add_example snippet_dir example_source)
    get_filename_component(example_name "${example_source}" NAME_WE)
    set(target_name "doc_${example_name}")

    message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
    message(STATUS "Building example: ${example_name}")
    message(STATUS "Linking libraries: ${_DOC_CPP_SNIPPET_LIBS}")


    set(example_sources "${example_source}")
    set(example_main "${snippet_dir}/${example_name}_main.cpp")
    if(EXISTS "${example_main}")
        list(APPEND example_sources "${example_main}")
    endif()

    add_executable(${target_name} ${example_sources})
    target_link_libraries(${target_name} PRIVATE ${_DOC_CPP_SNIPPET_LIBS})
    target_include_directories(${target_name} PRIVATE
        "${CMAKE_SOURCE_DIR}/ouster_core/include"
        "${CMAKE_SOURCE_DIR}/ouster_pcap/include"
        "${CMAKE_SOURCE_DIR}/ouster_sensor/include"
        "${CMAKE_SOURCE_DIR}/ouster_osf/include"
        "${CMAKE_SOURCE_DIR}/tests"
        "${_DOC_CPP_FEATURES_ROOT}/cpp_test_utils"
    )
    target_compile_definitions(${target_name} PRIVATE
        "OUSTER_SDK_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\""
    )
    if(TARGET OusterSDK::ouster_osf)
        target_compile_definitions(${target_name} PRIVATE OUSTER_OSF)
        message(STATUS "Adding OUSTER_OSF definition to ${target_name}")
    endif()
    if(TARGET OusterSDK::ouster_pcap)
        target_compile_definitions(${target_name} PRIVATE OUSTER_PCAP)
        message(STATUS "Adding OUSTER_PCAP definition to ${target_name}")
    endif()
    if(EXISTS "${CMAKE_SOURCE_DIR}/ouster_mapping/include")
        target_include_directories(
            ${target_name} PRIVATE "${CMAKE_SOURCE_DIR}/ouster_mapping/include"
        )
    endif()
    add_test(NAME ${target_name} COMMAND ${target_name})

    set(example_test "${snippet_dir}/${example_name}_test.cpp")
    if(EXISTS "${example_test}")
        set(test_target "${target_name}_test")
        add_executable(${test_target} ${example_source} "${example_test}")
        target_link_libraries(${test_target} PRIVATE ${_DOC_CPP_SNIPPET_LIBS} GTest::gtest_main)
        target_include_directories(${test_target} PRIVATE
            "${CMAKE_SOURCE_DIR}/ouster_core/include"
            "${CMAKE_SOURCE_DIR}/ouster_pcap/include"
            "${CMAKE_SOURCE_DIR}/ouster_sensor/include"
            "${CMAKE_SOURCE_DIR}/ouster_osf/include"
            "${CMAKE_SOURCE_DIR}/tests"
            "${_DOC_CPP_FEATURES_ROOT}/cpp_test_utils"
        )
        target_compile_definitions(${test_target} PRIVATE 
            "OUSTER_SDK_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\""
        )
        if(TARGET OusterSDK::ouster_osf)
            target_compile_definitions(${test_target} PRIVATE OUSTER_OSF)
        endif()
        if(TARGET OusterSDK::ouster_pcap)
            target_compile_definitions(${test_target} PRIVATE OUSTER_PCAP)
        endif()
        if(EXISTS "${CMAKE_SOURCE_DIR}/ouster_mapping/include")
            target_include_directories(
                ${test_target} PRIVATE "${CMAKE_SOURCE_DIR}/ouster_mapping/include"
            )
        endif()
        add_test(NAME ${test_target} COMMAND ${test_target})
        set_tests_properties(${test_target}
            PROPERTIES ENVIRONMENT "CTEST_FULL_OUTPUT=1")
        add_dependencies(${target_name} ${test_target})
        set_property(GLOBAL APPEND PROPERTY DOC_CPP_SNIPPET_TARGETS ${test_target})
    endif()

    set_property(GLOBAL APPEND PROPERTY DOC_CPP_SNIPPET_TARGETS ${target_name})
endfunction()

file(
    GLOB
    _DOC_CPP_FEATURE_DIRS
    RELATIVE "${_DOC_CPP_FEATURES_ROOT}"
    "${_DOC_CPP_FEATURES_ROOT}/*"
)

foreach(feature_dir IN LISTS _DOC_CPP_FEATURE_DIRS)
    if(NOT IS_DIRECTORY "${_DOC_CPP_FEATURES_ROOT}/${feature_dir}")
        continue()
    endif()
    set(snippet_dir "${_DOC_CPP_FEATURES_ROOT}/${feature_dir}/_snippets/cpp")
    if(NOT IS_DIRECTORY "${snippet_dir}")
        continue()
    endif()

    file(GLOB example_files "${snippet_dir}/*_test.cpp")
    foreach(example_file IN LISTS example_files)
        _doc_cpp_add_example("${snippet_dir}" "${example_file}")
    endforeach()
    
endforeach()

get_property(_doc_cpp_targets GLOBAL PROPERTY DOC_CPP_SNIPPET_TARGETS)
if(_doc_cpp_targets)
    list(REMOVE_DUPLICATES _doc_cpp_targets)
    add_custom_target(doc_cpp_examples DEPENDS ${_doc_cpp_targets})
endif()
