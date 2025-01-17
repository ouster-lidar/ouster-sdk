/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

/**
 * Turn off formatting to better organise the ifdefs with indents,
 * otherwise the code is hard to read.
 */
// clang-format off

/**
 * Defines for the check_exports.py script to be able to analyze
 * missing attributes.
 */
#if defined OUSTER_CHECK_EXPORTS
    #define OUSTER_API_FUNCTION __attribute__((annotate("OUSTER_API_FUNCTION")))
    #define OUSTER_API_CLASS __attribute__((annotate("OUSTER_API_CLASS")))
    #define OUSTER_API_IGNORE __attribute__((annotate("OUSTER_API_IGNORE")))
    #define OUSTER_API_DEFINES

/**
 * We are not running the check_exports.py script.
 */
#else

    /**
     * Tag to use for items that the check_exports.py script stubbornly insists needs
     * an attribute. For some reason clang returns public visibility for some protected
     * members of classes.
     */
    #define OUSTER_API_IGNORE

    /**
     * Currently compiling for windows.
     */
    #if defined _WIN32

        /**
         * Currently compiling the DLL for exports.
         */
        #ifdef BUILD_SHARED_LIBS_EXPORT
            #define OUSTER_API_FUNCTION __declspec( dllexport )
            #define OUSTER_API_CLASS
            #define OUSTER_API_DEFINES

        /**
         * Currently compiling the code requiring the ouster
         * shared library.
         */
        #elif defined(BUILD_SHARED_LIBS_IMPORT)
            #define OUSTER_API_FUNCTION __declspec( dllimport )
            #define OUSTER_API_CLASS
            #define OUSTER_API_DEFINES
        #endif

    /**
     * Currently compiling for linux or macos.
     */
    #else

        /**
         * Only add for linux and macos, and not emscripten.
         */
        #ifndef __EMSCRIPTEN__
            #define _OUSTER_API_UNIX_ATTR __attribute__((visibility("default")))
            #define OUSTER_API_FUNCTION _OUSTER_API_UNIX_ATTR
            #define OUSTER_API_CLASS _OUSTER_API_UNIX_ATTR
            #define OUSTER_API_DEFINES
        #endif
    #endif
#endif

/**
 * Check to make sure we defined all of the defines, if not
 * define them as empty. Emscripten usage also falls into this
 * ifndef.
 */
#ifndef OUSTER_API_DEFINES
    #define OUSTER_API_FUNCTION
    #define OUSTER_API_CLASS
    #define OUSTER_API_DEFINES
#endif
// clang-format on
