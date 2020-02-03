/*!< Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent. */

#include "EbDefinitions.h"
#include "aom_dsp_rtcd.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef uint32_t (*HighVarianceFn)(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                   int32_t ref_stride, uint32_t *sse, int32_t *sum);

uint32_t aom_highbd_calc4x4var_sse2(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                    int32_t ref_stride, uint32_t *sse, int32_t *sum);

uint32_t aom_highbd_calc8x8var_sse2(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                    int32_t ref_stride, uint32_t *sse, int32_t *sum);

uint32_t aom_highbd_calc16x16var_sse2(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                      int32_t ref_stride, uint32_t *sse, int32_t *sum);

#ifdef __cplusplus
}
#endif // __cplusplus
static void highbd_8_variance_sse2(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                   int32_t ref_stride, int32_t w, int32_t h, uint32_t *sse,
                                   int32_t *sum, HighVarianceFn var_fn, int32_t block_size) {
    int32_t i, j;

    *sse = 0;
    *sum = 0;

    for (i = 0; i < h; i += block_size) {
        for (j = 0; j < w; j += block_size) {
            uint32_t sse0;
            int32_t  sum0;
            var_fn(src + src_stride * i + j,
                   src_stride,
                   ref + ref_stride * i + j,
                   ref_stride,
                   &sse0,
                   &sum0);
            *sse += sse0;
            *sum += sum0;
        }
    }
}

static void highbd_10_variance_sse2(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                    int32_t ref_stride, int32_t w, int32_t h, uint32_t *sse,
                                    int32_t *sum, HighVarianceFn var_fn, int32_t block_size) {
    int32_t  i, j;
    uint64_t sse_long = 0;
    int32_t  sum_long = 0;

    for (i = 0; i < h; i += block_size) {
        for (j = 0; j < w; j += block_size) {
            uint32_t sse0;
            int32_t  sum0;
            var_fn(src + src_stride * i + j,
                   src_stride,
                   ref + ref_stride * i + j,
                   ref_stride,
                   &sse0,
                   &sum0);
            sse_long += sse0;
            sum_long += sum0;
        }
    }
    *sum = ROUND_POWER_OF_TWO(sum_long, 2);
    *sse = (uint32_t)ROUND_POWER_OF_TWO(sse_long, 4);
}

static void highbd_12_variance_sse2(const uint16_t *src, int32_t src_stride, const uint16_t *ref,
                                    int32_t ref_stride, int32_t w, int32_t h, uint32_t *sse,
                                    int32_t *sum, HighVarianceFn var_fn, int32_t block_size) {
    int32_t  i, j;
    uint64_t sse_long = 0;
    int32_t  sum_long = 0;

    for (i = 0; i < h; i += block_size) {
        for (j = 0; j < w; j += block_size) {
            uint32_t sse0;
            int32_t  sum0;
            var_fn(src + src_stride * i + j,
                   src_stride,
                   ref + ref_stride * i + j,
                   ref_stride,
                   &sse0,
                   &sum0);
            sse_long += sse0;
            sum_long += sum0;
        }
    }
    *sum = ROUND_POWER_OF_TWO(sum_long, 4);
    *sse = (uint32_t)ROUND_POWER_OF_TWO(sse_long, 8);
}

#define HIGH_GET_VAR(S)                                                                 \
    void eb_aom_highbd_get##S##x##S##var_sse2(const uint8_t *src8,                      \
                                              int32_t        src_stride,                \
                                              const uint8_t *ref8,                      \
                                              int32_t        ref_stride,                \
                                              uint32_t *     sse,                       \
                                              int32_t *      sum) {                           \
        uint16_t *src = CONVERT_TO_SHORTPTR(src8);                                      \
        uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);                                      \
        aom_highbd_calc##S##x##S##var_sse2(src, src_stride, ref, ref_stride, sse, sum); \
    }                                                                                   \
                                                                                        \
    void eb_aom_highbd_10_get##S##x##S##var_sse2(const uint8_t *src8,                   \
                                                 int32_t        src_stride,             \
                                                 const uint8_t *ref8,                   \
                                                 int32_t        ref_stride,             \
                                                 uint32_t *     sse,                    \
                                                 int32_t *      sum) {                        \
        uint16_t *src = CONVERT_TO_SHORTPTR(src8);                                      \
        uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);                                      \
        aom_highbd_calc##S##x##S##var_sse2(src, src_stride, ref, ref_stride, sse, sum); \
        *sum = ROUND_POWER_OF_TWO(*sum, 2);                                             \
        *sse = ROUND_POWER_OF_TWO(*sse, 4);                                             \
    }                                                                                   \
                                                                                        \
    void eb_aom_highbd_12_get##S##x##S##var_sse2(const uint8_t *src8,                   \
                                                 int32_t        src_stride,             \
                                                 const uint8_t *ref8,                   \
                                                 int32_t        ref_stride,             \
                                                 uint32_t *     sse,                    \
                                                 int32_t *      sum) {                        \
        uint16_t *src = CONVERT_TO_SHORTPTR(src8);                                      \
        uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);                                      \
        aom_highbd_calc##S##x##S##var_sse2(src, src_stride, ref, ref_stride, sse, sum); \
        *sum = ROUND_POWER_OF_TWO(*sum, 4);                                             \
        *sse = ROUND_POWER_OF_TWO(*sse, 8);                                             \
    }

HIGH_GET_VAR(16);
HIGH_GET_VAR(8);

#undef HIGH_GET_VAR

#define VAR_FN(w, h, block_size, shift)                                               \
    uint32_t eb_aom_highbd_8_variance##w##x##h##_sse2(const uint8_t *src8,            \
                                                      int32_t        src_stride,      \
                                                      const uint8_t *ref8,            \
                                                      int32_t        ref_stride,      \
                                                      uint32_t *     sse) {                \
        int32_t   sum;                                                                \
        uint16_t *src = CONVERT_TO_SHORTPTR(src8);                                    \
        uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);                                    \
        highbd_8_variance_sse2(src,                                                   \
                               src_stride,                                            \
                               ref,                                                   \
                               ref_stride,                                            \
                               w,                                                     \
                               h,                                                     \
                               sse,                                                   \
                               &sum,                                                  \
                               aom_highbd_calc##block_size##x##block_size##var_sse2,  \
                               block_size);                                           \
        return *sse - (uint32_t)(((int64_t)sum * sum) >> shift);                      \
    }                                                                                 \
                                                                                      \
    uint32_t eb_aom_highbd_10_variance##w##x##h##_sse2(const uint8_t *src8,           \
                                                       int32_t        src_stride,     \
                                                       const uint8_t *ref8,           \
                                                       int32_t        ref_stride,     \
                                                       uint32_t *     sse) {               \
        int32_t   sum;                                                                \
        int64_t   var;                                                                \
        uint16_t *src = CONVERT_TO_SHORTPTR(src8);                                    \
        uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);                                    \
        highbd_10_variance_sse2(src,                                                  \
                                src_stride,                                           \
                                ref,                                                  \
                                ref_stride,                                           \
                                w,                                                    \
                                h,                                                    \
                                sse,                                                  \
                                &sum,                                                 \
                                aom_highbd_calc##block_size##x##block_size##var_sse2, \
                                block_size);                                          \
        var = (int64_t)(*sse) - (((int64_t)sum * sum) >> shift);                      \
        return (var >= 0) ? (uint32_t)var : 0;                                        \
    }                                                                                 \
                                                                                      \
    uint32_t eb_aom_highbd_12_variance##w##x##h##_sse2(const uint8_t *src8,           \
                                                       int32_t        src_stride,     \
                                                       const uint8_t *ref8,           \
                                                       int32_t        ref_stride,     \
                                                       uint32_t *     sse) {               \
        int32_t   sum;                                                                \
        int64_t   var;                                                                \
        uint16_t *src = CONVERT_TO_SHORTPTR(src8);                                    \
        uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);                                    \
        highbd_12_variance_sse2(src,                                                  \
                                src_stride,                                           \
                                ref,                                                  \
                                ref_stride,                                           \
                                w,                                                    \
                                h,                                                    \
                                sse,                                                  \
                                &sum,                                                 \
                                aom_highbd_calc##block_size##x##block_size##var_sse2, \
                                block_size);                                          \
        var = (int64_t)(*sse) - (((int64_t)sum * sum) >> shift);                      \
        return (var >= 0) ? (uint32_t)var : 0;                                        \
    }

VAR_FN(64, 64, 16, 12);
VAR_FN(64, 32, 16, 11);
VAR_FN(32, 64, 16, 11);
VAR_FN(32, 32, 16, 10);
VAR_FN(32, 16, 16, 9);
VAR_FN(16, 32, 16, 9);
VAR_FN(16, 16, 16, 8);
VAR_FN(16, 8, 8, 7);
VAR_FN(8, 16, 8, 7);
VAR_FN(8, 8, 8, 6);
VAR_FN(16, 4, 4, 6);
VAR_FN(8, 32, 8, 8);
VAR_FN(32, 8, 8, 8);
VAR_FN(16, 64, 16, 10);
VAR_FN(64, 16, 16, 10);

#undef VAR_FN

void eb_aom_highbd_8_mse16x16_sse2(const uint8_t *src8, int32_t src_stride, const uint8_t *ref8,
                                   int32_t ref_stride, uint32_t *sse) {
    int32_t   sum;
    uint16_t *src = CONVERT_TO_SHORTPTR(src8);
    uint16_t *ref = CONVERT_TO_SHORTPTR(ref8);

    /*!< TODO: Remove calculate unused sum. */
    highbd_8_variance_sse2(
        src, src_stride, ref, ref_stride, 16, 16, sse, &sum, aom_highbd_calc16x16var_sse2, 16);
}
