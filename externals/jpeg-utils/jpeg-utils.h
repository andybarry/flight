#ifndef _jpeg_utils_ijg_h_
#define _jpeg_utils_ijg_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @defgroup JpegUtils JPEG Utilities
 * @brief Convenience functions for JPEG compression / decompression.
 */

/**
 * Decompress JPEG data into an 8-bit RGB buffer.
 */
int
jpeg_decompress_8u_rgb (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride);

/**
 * Decompress JPEG data into an 8-bit grayscale buffer.  If the original JPEG image
 * contained color data, the color values are discarded and only the Y channel
 * is decompressed.  This is faster than decompressing to RGB and converting to
 * grayscale.
 */
int
jpeg_decompress_8u_gray (const uint8_t * src, int src_size,
        uint8_t * dest, int width, int height, int stride);

/**
 * @dest: output buffer
 * @destsize:  input and output parameter.  On input, stores the buffer size
 * available in dest.  On output, stores the amount of buffer used for the compressed
 * data.
 * @quality: compression quality.  0 - 100.
 *
 * JPEG compress 8-bit grayscale data.  
 */
int
jpeg_compress_8u_gray (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality);

/**
 * @dest: output buffer
 * @destsize:  input and output parameter.  On input, stores the buffer size
 * available in dest.  On output, stores the amount of buffer used for the compressed
 * data.
 * @quality: compression quality.  0 - 100.
 *
 * JPEG compress 8-bit grayscale data.
 */
int
jpeg_compress_8u_rgb (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality);
int
jpeg_compress_8u_bgra (const uint8_t * src, int width, int height, int stride,
        uint8_t * dest, int * destsize, int quality);

#ifdef __cplusplus
}
#endif


#endif
