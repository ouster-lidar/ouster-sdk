#pragma once

#ifdef __CUDACC__
#define OSDK_FN __device__ __host__
#define OSDK_FN_HOST __host__
#define OSDK_FN_DEVICE __device__
#define RESTRICT __restrict__
#else
#define OSDK_FN
#define OSDK_FN_HOST
#define OSDK_FN_DEVICE
#define RESTRICT
#endif
