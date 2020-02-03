/*!< Copyright(c) 2019 Intel Corporation
 * SPDX - License - Identifier: BSD - 2 - Clause - Patent */

#ifndef EbPictureOperators_SSE2_h
#define EbPictureOperators_SSE2_h

#include <emmintrin.h>
#include "EbDefinitions.h"

#ifdef __cplusplus
extern "C" {
#endif

/******/
extern void zero_out_coeff4x4_sse(int16_t *coeff_buffer, uint32_t coeff_stride,
                                  uint32_t coeff_origin_index, uint32_t area_width,
                                  uint32_t area_height);

extern void zero_out_coeff8x8_sse2(int16_t *coeff_buffer, uint32_t coeff_stride,
                                   uint32_t coeff_origin_index, uint32_t area_width,
                                   uint32_t area_height);

extern void zero_out_coeff16x16_sse2(int16_t *coeff_buffer, uint32_t coeff_stride,
                                     uint32_t coeff_origin_index, uint32_t area_width,
                                     uint32_t area_height);

extern void zero_out_coeff32x32_sse2(int16_t *coeff_buffer, uint32_t coeff_stride,
                                     uint32_t coeff_origin_index, uint32_t area_width,
                                     uint32_t area_height);

extern void residual_kernel16bit_sse2_intrin(uint16_t *input, uint32_t input_stride, uint16_t *pred,
                                             uint32_t pred_stride, int16_t *residual,
                                             uint32_t residual_stride, uint32_t area_width,
                                             uint32_t area_height);

static INLINE int32_t hadd32_sse2_intrin(const __m128i src) {
    const __m128i dst0 = _mm_add_epi32(src, _mm_srli_si128(src, 8));
    const __m128i dst1 = _mm_add_epi32(dst0, _mm_srli_si128(dst0, 4));

    return _mm_cvtsi128_si32(dst1);
}

uint64_t spatial_full_distortion_kernel4x_n_sse2_intrin(uint8_t *input, uint32_t input_offset,
                                                        uint32_t input_stride, uint8_t *recon,
                                                        uint32_t recon_offset,
                                                        uint32_t recon_stride, uint32_t area_width,
                                                        uint32_t area_height);

uint64_t spatial_full_distortion_kernel8x_n_sse2_intrin(uint8_t *input, uint32_t input_offset,
                                                        uint32_t input_stride, uint8_t *recon,
                                                        uint32_t recon_offset,
                                                        uint32_t recon_stride, uint32_t area_width,
                                                        uint32_t area_height);

uint64_t spatial_full_distortion_kernel16x_n_sse2_intrin(uint8_t *input, uint32_t input_offset,
                                                         uint32_t input_stride, uint8_t *recon,
                                                         uint32_t recon_offset,
                                                         uint32_t recon_stride, uint32_t area_width,
                                                         uint32_t area_height);

uint64_t spatial_full_distortion_kernel32x_n_sse2_intrin(uint8_t *input, uint32_t input_offset,
                                                         uint32_t input_stride, uint8_t *recon,
                                                         uint32_t recon_offset,
                                                         uint32_t recon_stride, uint32_t area_width,
                                                         uint32_t area_height);

uint64_t spatial_full_distortion_kernel64x_n_sse2_intrin(uint8_t *input, uint32_t input_offset,
                                                         uint32_t input_stride, uint8_t *recon,
                                                         uint32_t recon_offset,
                                                         uint32_t recon_stride, uint32_t area_width,
                                                         uint32_t area_height);

uint64_t spatial_full_distortion_kernel128x_n_sse2_intrin(
    uint8_t *input, uint32_t input_offset, uint32_t input_stride, uint8_t *recon,
    uint32_t recon_offset, uint32_t recon_stride, uint32_t area_width, uint32_t area_height);

#ifdef __cplusplus
}
#endif
#endif // EbPictureOperators_SSE2_h
