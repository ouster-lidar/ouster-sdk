/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

// FW's build treats using zero as a null ptr constant as an error.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <Eigen/Core>
#include <Eigen/Dense>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
