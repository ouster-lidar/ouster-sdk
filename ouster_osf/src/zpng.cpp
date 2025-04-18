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

#include "zpng.h"

#include <zstd.h>

#include <stdlib.h> // calloc


//------------------------------------------------------------------------------
// Constants

// Higher compression levels do not gain much but hurt speed
static const int kCompressionLevel = 1;

// This enabled some specialized versions for RGB and RGBA
#define ENABLE_RGB_COLOR_FILTER

// Header definitions
#define ZPNG_HEADER_MAGIC 0xFBF8
#define ZPNG_HEADER_OVERHEAD_BYTES 8

// File format header
#pragma pack(1)
struct ZPNG_Header
{
    uint16_t Magic;
    uint16_t Width;
    uint16_t Height;
    uint8_t Channels;
    uint8_t BytesPerChannel;
};
#pragma pack()


//------------------------------------------------------------------------------
// Image Processing

// Interleaving is a 1% compression win, and a 0.3% performance win: Not used.
// Splitting the data into blocks of 4 at a time actually reduces compression.

template<int kChannels>
static void PackAndFilter(
    const ZPNG_ImageData* imageData,
    uint8_t* output
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    for (unsigned y = 0; y < height; ++y)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            // For each channel:
            for (unsigned i = 0; i < kChannels; ++i)
            {
                uint8_t a = input[i];
                uint8_t d = a - prev[i];
                output[i] = d;
                prev[i] = a;
            }

            input += kChannels;
            output += kChannels;
        }
    }
}

template<int kChannels>
static void UnpackAndUnfilter(
    const uint8_t* input,
    ZPNG_ImageData* imageData
)
{
    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    for (unsigned y = 0; y < height; ++y)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            // For each channel:
            for (unsigned i = 0; i < kChannels; ++i)
            {
                uint8_t d = input[i];
                uint8_t a = d + prev[i];
                output[i] = a;
                prev[i] = a;
            }

            input += kChannels;
            output += kChannels;
        }
    }
}


#ifdef ENABLE_RGB_COLOR_FILTER

template<>
void PackAndFilter<3>(
    const ZPNG_ImageData* imageData,
    uint8_t* output
    )
{
    static const unsigned kChannels = 3;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    uint8_t* output_y = output;
    uint8_t* output_u = output + planeBytes;
    uint8_t* output_v = output + planeBytes * 2;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t r = input[0];
            uint8_t g = input[1];
            uint8_t b = input[2];

            r -= prev[0];
            g -= prev[1];
            b -= prev[2];

            prev[0] = input[0];
            prev[1] = input[1];
            prev[2] = input[2];

            // GB-RG filter from BCIF
            uint8_t y = b;
            uint8_t u = g - b;
            uint8_t v = g - r;

            *output_y++ = y;
            *output_u++ = u;
            *output_v++ = v;

            input += kChannels;
        }
    }
}

template<>
void UnpackAndUnfilter<3>(
    const uint8_t* input,
    ZPNG_ImageData* imageData
    )
{
    static const unsigned kChannels = 3;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    const uint8_t* input_y = input;
    const uint8_t* input_u = input + planeBytes;
    const uint8_t* input_v = input + planeBytes * 2;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t y = *input_y++;
            uint8_t u = *input_u++;
            uint8_t v = *input_v++;

            // GB-RG filter from BCIF
            const uint8_t B = y;
            const uint8_t G = u + B;
            uint8_t r = G - v;
            uint8_t g = G;
            uint8_t b = B;

            r += prev[0];
            g += prev[1];
            b += prev[2];

            output[0] = r;
            output[1] = g;
            output[2] = b;

            prev[0] = r;
            prev[1] = g;
            prev[2] = b;

            output += kChannels;
        }
    }
}


// Version for RGBA (with alpha):

template<>
void PackAndFilter<4>(
    const ZPNG_ImageData* imageData,
    uint8_t* output
    )
{
    static const unsigned kChannels = 4;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    const uint8_t* input = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    uint8_t* output_y = output;
    uint8_t* output_u = output + planeBytes;
    uint8_t* output_v = output + planeBytes * 2;
    uint8_t* output_a = output + planeBytes * 3;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t r = input[0];
            uint8_t g = input[1];
            uint8_t b = input[2];
            uint8_t a = input[3];

            r -= prev[0];
            g -= prev[1];
            b -= prev[2];
            a -= prev[3];

            prev[0] = input[0];
            prev[1] = input[1];
            prev[2] = input[2];
            prev[3] = input[3];

            // GB-RG filter from BCIF
            uint8_t y = b;
            uint8_t u = g - b;
            uint8_t v = g - r;

            *output_y++ = y;
            *output_u++ = u;
            *output_v++ = v;
            *output_a++ = a;

            input += kChannels;
        }
    }
}

template<>
void UnpackAndUnfilter<4>(
    const uint8_t* input,
    ZPNG_ImageData* imageData
    )
{
    static const unsigned kChannels = 4;

    const unsigned height = imageData->HeightPixels;
    const unsigned width = imageData->WidthPixels;

    uint8_t* output = imageData->Buffer.Data;

    // Color plane split
    const unsigned planeBytes = width * height;
    const uint8_t* input_y = input;
    const uint8_t* input_u = input + planeBytes;
    const uint8_t* input_v = input + planeBytes * 2;
    const uint8_t* input_a = input + planeBytes * 3;

    for (unsigned row = 0; row < height; ++row)
    {
        uint8_t prev[kChannels] = { 0 };

        for (unsigned x = 0; x < width; ++x)
        {
            uint8_t y = *input_y++;
            uint8_t u = *input_u++;
            uint8_t v = *input_v++;
            uint8_t a = *input_a++;

            // GB-RG filter from BCIF
            const uint8_t B = y;
            const uint8_t G = u + B;
            uint8_t r = G - v;
            uint8_t g = G;
            uint8_t b = B;

            r += prev[0];
            g += prev[1];
            b += prev[2];
            a += prev[3];

            output[0] = r;
            output[1] = g;
            output[2] = b;
            output[3] = a;

            prev[0] = r;
            prev[1] = g;
            prev[2] = b;
            prev[3] = a;

            output += kChannels;
        }
    }
}

#endif


#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// API

ZPNG_Buffer ZPNG_Compress(
    const ZPNG_ImageData* imageData
)
{
    uint8_t* packing = nullptr;
    uint8_t* output = nullptr;

    ZPNG_Buffer bufferOutput;
    bufferOutput.Data = nullptr;
    bufferOutput.Bytes = 0;

    const unsigned pixelCount = imageData->WidthPixels * imageData->HeightPixels;
    const unsigned pixelBytes = imageData->BytesPerChannel * imageData->Channels;
    const unsigned byteCount = pixelBytes * pixelCount;

    // FIXME: One day add support for other formats
    if (pixelBytes > 8) {
        return bufferOutput;
    }

    // Space for packing
    packing = (uint8_t*)calloc(1, byteCount);

    if (!packing) {
ReturnResult:
        if (bufferOutput.Data != output) {
            free(output);
        }
        free(packing);
        return bufferOutput;
    }

    const unsigned maxOutputBytes = (unsigned)ZSTD_compressBound(byteCount);

    // Space for output
    output = (uint8_t*)calloc(1, ZPNG_HEADER_OVERHEAD_BYTES + maxOutputBytes);

    if (!output) {
        goto ReturnResult;
    }

    // Pass 1: Pack and filter data.

    switch (pixelBytes)
    {
    case 1:
        PackAndFilter<1>(imageData, packing);
        break;
    case 2:
        PackAndFilter<2>(imageData, packing);
        break;
    case 3:
        PackAndFilter<3>(imageData, packing);
        break;
    case 4:
        PackAndFilter<4>(imageData, packing);
        break;
    case 5:
        PackAndFilter<5>(imageData, packing);
        break;
    case 6:
        PackAndFilter<6>(imageData, packing);
        break;
    case 7:
        PackAndFilter<7>(imageData, packing);
        break;
    case 8:
        PackAndFilter<8>(imageData, packing);
        break;
    }

    // Pass 2: Compress the packed/filtered data.

    const size_t result = ZSTD_compress(
        output + ZPNG_HEADER_OVERHEAD_BYTES,
        maxOutputBytes,
        packing,
        byteCount,
        kCompressionLevel);

    if (ZSTD_isError(result)) {
        goto ReturnResult;
    }

    // Write header

    ZPNG_Header* header = (ZPNG_Header*)output;
    header->Magic = ZPNG_HEADER_MAGIC;
    header->Width = (uint16_t)imageData->WidthPixels;
    header->Height = (uint16_t)imageData->HeightPixels;
    header->Channels = (uint8_t)imageData->Channels;
    header->BytesPerChannel = (uint8_t)imageData->BytesPerChannel;

    bufferOutput.Data = output;
    bufferOutput.Bytes = ZPNG_HEADER_OVERHEAD_BYTES + (unsigned)result;

    goto ReturnResult;
}

ZPNG_ImageData ZPNG_Decompress(
    ZPNG_Buffer buffer
)
{
    uint8_t* packing = nullptr;
    uint8_t* output = nullptr;

    ZPNG_ImageData imageData;
    imageData.Buffer.Data = nullptr;
    imageData.Buffer.Bytes = 0;
    imageData.BytesPerChannel = 0;
    imageData.Channels = 0;
    imageData.HeightPixels = 0;
    imageData.StrideBytes = 0;
    imageData.WidthPixels = 0;

    if (!buffer.Data || buffer.Bytes < ZPNG_HEADER_OVERHEAD_BYTES) {
ReturnResult:
        if (imageData.Buffer.Data != output) {
            free(output);
        }
        free(packing);
        return imageData;
    }

    const ZPNG_Header* header = (const ZPNG_Header*)buffer.Data;
    if (header->Magic != ZPNG_HEADER_MAGIC) {
        goto ReturnResult;
    }

    imageData.WidthPixels = header->Width;
    imageData.HeightPixels = header->Height;
    imageData.Channels = header->Channels;
    imageData.BytesPerChannel = header->BytesPerChannel;
    imageData.StrideBytes = imageData.WidthPixels * imageData.Channels;

    const unsigned pixelCount = imageData.WidthPixels * imageData.HeightPixels;
    const unsigned pixelBytes = imageData.BytesPerChannel * imageData.Channels;
    const unsigned byteCount = pixelBytes * pixelCount;

    // Space for packing
    packing = (uint8_t*)calloc(1, byteCount);

    if (!packing) {
        goto ReturnResult;
    }

    // Stage 1: Decompress back to packing buffer

    const size_t result = ZSTD_decompress(
        packing,
        byteCount,
        buffer.Data + ZPNG_HEADER_OVERHEAD_BYTES,
        buffer.Bytes - ZPNG_HEADER_OVERHEAD_BYTES);

    if (ZSTD_isError(result)) {
        goto ReturnResult;
    }

    // Stage 2: Unpack/Unfilter

    // Space for output
    output = (uint8_t*)calloc(1, byteCount);

    if (!output) {
        goto ReturnResult;
    }

    imageData.Buffer.Data = output;
    imageData.Buffer.Bytes = byteCount;

    switch (pixelBytes)
    {
    case 1:
        UnpackAndUnfilter<1>(packing, &imageData);
        break;
    case 2:
        UnpackAndUnfilter<2>(packing, &imageData);
        break;
    case 3:
        UnpackAndUnfilter<3>(packing, &imageData);
        break;
    case 4:
        UnpackAndUnfilter<4>(packing, &imageData);
        break;
    case 5:
        UnpackAndUnfilter<5>(packing, &imageData);
        break;
    case 6:
        UnpackAndUnfilter<6>(packing, &imageData);
        break;
    case 7:
        UnpackAndUnfilter<7>(packing, &imageData);
        break;
    case 8:
        UnpackAndUnfilter<8>(packing, &imageData);
        break;
    }

    goto ReturnResult;
}

void ZPNG_Free(
    ZPNG_Buffer* buffer
)
{
    if (buffer && buffer->Data)
    {
        free(buffer->Data);
        buffer->Data = nullptr;
        buffer->Bytes = 0;
    }
}


#ifdef __cplusplus
}
#endif
