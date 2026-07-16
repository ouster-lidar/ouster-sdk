/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstdlib>
#include <fstream>

#ifndef _WIN32
#include <sys/stat.h>
#endif

#include <curl/curl.h>
#include <curl/easy.h>

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

// Set CAINFO/CAPATH together, explicitly clearing whichever one is null.
//
// This is important: libcurl seeds CAINFO/CAPATH with the paths compiled in at
// build time. If we set only one of them, libcurl still tries to load the other
// stale default and (with OpenSSL 3.x, which loads the file and directory
// independently) fails the whole request with CURLE_SSL_CACERT_BADFILE. Passing
// nullptr overwrites the compiled-in default with "unset" so it is not loaded.
inline void set_ca_options(CURL* handle, const char* ca_info, const char* ca_path) {
    curl_easy_setopt(handle, CURLOPT_CAINFO, ca_info);
    curl_easy_setopt(handle, CURLOPT_CAPATH, ca_path);
}

// Point a libcurl easy handle at a valid CA certificate bundle for TLS
// verification.
//
// Statically-linked builds (e.g. the CI-produced wheels) bake in a default CA
// path from the build machine that frequently does not exist on the user's
// system, causing HTTPS requests to fail with CURLE_SSL_CACERT_BADFILE. Unlike
// the curl command-line tool, libcurl does not consult CURL_CA_BUNDLE on its
// own, and once curl has a compiled-in CAINFO it never falls back to OpenSSL's
// SSL_CERT_FILE/SSL_CERT_DIR handling.
//
// Resolution order (first match wins) so this works by default with no setup:
//   1. CURL_CA_BUNDLE / SSL_CERT_FILE / SSL_CERT_DIR environment variables.
//      This is also how the Python layer injects certifi's bundle.
//   2. On Windows, the OS certificate store (via CURLSSLOPT_NATIVE_CA).
//   3. Well-known system CA bundle files/directories on Linux and macOS.
inline void configure_ca_bundle(CURL* handle) {
    const char* ca_info = std::getenv("CURL_CA_BUNDLE");
    if (ca_info == nullptr || ca_info[0] == '\0') {
        ca_info = std::getenv("SSL_CERT_FILE");
    }
    const char* ca_path = std::getenv("SSL_CERT_DIR");
    // Only treat CAINFO as usable if the file actually exists and is readable;
    // otherwise libcurl would fail with CURLE_SSL_CACERT_BADFILE. A bad value
    // instead falls through to the platform/system fallbacks below.
    const bool have_ca_info =
        (ca_info != nullptr && ca_info[0] != '\0' && std::ifstream(ca_info).good());
    const bool have_ca_path = (ca_path != nullptr && ca_path[0] != '\0');
    if (have_ca_info || have_ca_path) {
        set_ca_options(handle, have_ca_info ? ca_info : nullptr, have_ca_path ? ca_path : nullptr);
        return;
    }

#if defined(_WIN32)
#if defined(CURLSSLOPT_NATIVE_CA)
    // Use the Windows system certificate store (supported with the OpenSSL
    // backend since curl 7.71).
    curl_easy_setopt(handle, CURLOPT_SSL_OPTIONS,
                     static_cast<long>(CURLSSLOPT_NATIVE_CA));  // NOLINT(google-runtime-int)
#endif
    return;
#else
    static const char* const ca_files[] = {
        "/etc/ssl/certs/ca-certificates.crt",    // Debian, Ubuntu, Alpine
        "/etc/pki/tls/certs/ca-bundle.crt",      // Fedora, RHEL, CentOS, AlmaLinux
        "/etc/ssl/ca-bundle.pem",                // openSUSE
        "/etc/ssl/cert.pem",                     // macOS (LibreSSL), Alpine
        "/opt/homebrew/etc/openssl@3/cert.pem",  // macOS Homebrew (Apple Silicon)
        "/usr/local/etc/openssl@3/cert.pem",     // macOS Homebrew (Intel)
    };
    for (const char* ca_file : ca_files) {
        if (std::ifstream(ca_file).good()) {
            set_ca_options(handle, ca_file, nullptr);
            return;
        }
    }
    static const char* const ca_dirs[] = {
        "/etc/ssl/certs",      // Debian, Ubuntu
        "/etc/pki/tls/certs",  // Fedora, RHEL
    };
    struct stat st {};
    for (const char* ca_dir : ca_dirs) {
        if (stat(ca_dir, &st) == 0 && S_ISDIR(st.st_mode)) {
            set_ca_options(handle, nullptr, ca_dir);
            return;
        }
    }
#endif
}

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
