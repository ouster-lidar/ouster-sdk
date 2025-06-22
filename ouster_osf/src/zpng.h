/** \file
    \brief Zpng - Experimental Lossless Image Compressor
    \copyright Copyright (c) 2018 Christopher A. Taylor.  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of Zpng nor the names of its contributors may be
      used to endorse or promote products derived from this software without
      specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CAT_ZPNG_H
#define CAT_ZPNG_H

/** \page Zpng - Experimental Lossless Image Compressor

    It's much faster than PNG and compresses better for photographic images.
    This compressor often takes less than 6% of the time of a PNG compressor
    and produces a file that is 66% of the size.
    It was written in about 600 lines of C code thanks to the Zstd library.

    This library is similar to PNG in that the image is first filtered,
    and then submitted to a data compressor.
    The filtering step is a bit simpler and faster but somehow more effective
    than the one used in PNG.
    The data compressor used is Zstd, which makes it significantly faster
    than PNG to compress and decompress.

    Filtering:

    (1) Reversible color channel transformation.
    (2) Split each color channel into a separate color plane.
    (3) Subtract each color value from the one to its left.
*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Buffer returned by library
struct ZPNG_Buffer
{
    // Pointer to data
    unsigned char* Data;

    // Size of buffer in bytes
    unsigned Bytes;
};

// Image data returned by the library
struct ZPNG_ImageData
{
    // Pixel data
    ZPNG_Buffer Buffer;

    // Number of bytes for each color channel (1-2)
    unsigned BytesPerChannel;

    // Number of channels for each pixel (1-4)
    unsigned Channels;

    // Width in pixels of image
    unsigned WidthPixels;

    // Height in pixels of image
    unsigned HeightPixels;

    // Width of pixel row in bytes
    unsigned StrideBytes;
};


//------------------------------------------------------------------------------
// API

/**
    ZPNG_Compress()

    Compress image into a buffer.

    The returned buffer should be passed to ZPNG_Free().

    On success returns a valid data pointer.
    On failure returns a null pointer.
*/
ZPNG_Buffer ZPNG_Compress(
    const ZPNG_ImageData* imageData
);

/*
    ZPNG_Decompress()

    Decompress image from a buffer.

    The returned ZPNG_Buffer should be passed to ZPNG_Free().

    On success returns a valid data pointer.
    On failure returns a null pointer.
*/
ZPNG_ImageData ZPNG_Decompress(
    ZPNG_Buffer buffer
);

/*
    ZPNG_Free()

    Free buffer when done to avoid leaks.

    This will also set the buffer data pointer to null.
*/
void ZPNG_Free(
    ZPNG_Buffer* buffer
);


#ifdef __cplusplus
}
#endif


#endif // CAT_ZPNG_H
