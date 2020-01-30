/*!< Copyright(c) 2019 Intel Corporation
 * SPDX - License - Identifier: BSD - 2 - Clause - Patent */

/*!< Copyright (c) 2016, Alliance for Open Media. All rights reserved
 *
 * This source code is subject to the terms of the BSD 2 Clause License and
 * the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
 * was not distributed with this source code in the LICENSE file, you can
 * obtain it at www.aomedia.org/license/software. If the Alliance for Open
 * Media Patent License 1.0 was not distributed with this source code in the
 * PATENTS file, you can obtain it at www.aomedia.org/license/patent. */

#include <stdlib.h>
#include "EbTransforms.h"
#include "aom_dsp_rtcd.h"

const int8_t *eb_inv_txfm_shift_ls[TX_SIZES_ALL] = {
    inv_shift_4x4,   inv_shift_8x8,   inv_shift_16x16, inv_shift_32x32, inv_shift_64x64,
    inv_shift_4x8,   inv_shift_8x4,   inv_shift_8x16,  inv_shift_16x8,  inv_shift_16x32,
    inv_shift_32x16, inv_shift_32x64, inv_shift_64x32, inv_shift_4x16,  inv_shift_16x4,
    inv_shift_8x32,  inv_shift_32x8,  inv_shift_16x64, inv_shift_64x16,
};

static const int8_t *fwd_txfm_range_mult2_list[TXFM_TYPES] = {fdct4_range_mult2,
                                                              fdct8_range_mult2,
                                                              fdct16_range_mult2,
                                                              fdct32_range_mult2,
                                                              fdct64_range_mult2,
                                                              fadst4_range_mult2,
                                                              fadst8_range_mult2,
                                                              fadst16_range_mult2,
                                                              fadst32_range_mult2,
                                                              fidtx4_range_mult2,
                                                              fidtx8_range_mult2,
                                                              fidtx16_range_mult2,
                                                              fidtx32_range_mult2,
                                                              fidtx64_range_mult2};

static const int8_t *fwd_txfm_shift_ls[TX_SIZES_ALL] = {
    fwd_shift_4x4,   fwd_shift_8x8,   fwd_shift_16x16, fwd_shift_32x32, fwd_shift_64x64,
    fwd_shift_4x8,   fwd_shift_8x4,   fwd_shift_8x16,  fwd_shift_16x8,  fwd_shift_16x32,
    fwd_shift_32x16, fwd_shift_32x64, fwd_shift_64x32, fwd_shift_4x16,  fwd_shift_16x4,
    fwd_shift_8x32,  fwd_shift_32x8,  fwd_shift_16x64, fwd_shift_64x16,
};

/*****************************/
/*!< Defines */
/*****************************/

#define BETA_P 1
#define BETA_N 3

/********************************************/
/*!< Constants */
/********************************************/

#define ALPHA_0000 0
#define ALPHA_0050 50

#define ALPHA_0100 100
#define ALPHA_0200 200
#define ALPHA_0300 300
#define ALPHA_0500 500
#define ALPHA_1000 1000

void eb_av1_gen_fwd_stage_range(int8_t *stage_range_col, int8_t *stage_range_row,
                                const Txfm2dFlipCfg *cfg, int32_t bd) {
    /*!< Take the shift from the larger dimension in the rectangular case. */
    const int8_t *shift = cfg->shift;
    /*!< i < MAX_TXFM_STAGE_NUM will mute above array bounds warning */
    for (int32_t i = 0; i < cfg->stage_num_col && i < MAX_TXFM_STAGE_NUM; ++i)
        stage_range_col[i] = (int8_t)(cfg->stage_range_col[i] + shift[0] + bd + 1);
    /*!< i < MAX_TXFM_STAGE_NUM will mute above array bounds warning */
    for (int32_t i = 0; i < cfg->stage_num_row && i < MAX_TXFM_STAGE_NUM; ++i)
        stage_range_row[i] = (int8_t)(cfg->stage_range_row[i] + shift[0] + shift[1] + bd + 1);
}

typedef void (*TxfmFunc)(const int32_t *input, int32_t *output, int8_t cos_bit,
                         const int8_t *stage_range);

#define range_check(stage, input, buf, size, bit) \
    {                                             \
        (void)stage;                              \
        (void)input;                              \
        (void)buf;                                \
        (void)size;                               \
        (void)bit;                                \
    }

// av1_cospi_arr[i][j] = (int32_t)round(cos(M_PI*j/128) * (1<<(cos_bit_min+i)));
const int32_t eb_av1_cospi_arr_data[7][64] = {
    {1024, 1024, 1023, 1021, 1019, 1016, 1013, 1009, 1004, 999, 993, 987, 980, 972, 964, 955,
     946,  936,  926,  915,  903,  891,  878,  865,  851,  837, 822, 807, 792, 775, 759, 742,
     724,  706,  688,  669,  650,  630,  610,  590,  569,  548, 526, 505, 483, 460, 438, 415,
     392,  369,  345,  321,  297,  273,  249,  224,  200,  175, 150, 125, 100, 75,  50,  25},
    {2048, 2047, 2046, 2042, 2038, 2033, 2026, 2018, 2009, 1998, 1987, 1974, 1960, 1945, 1928, 1911,
     1892, 1872, 1851, 1829, 1806, 1782, 1757, 1730, 1703, 1674, 1645, 1615, 1583, 1551, 1517, 1483,
     1448, 1412, 1375, 1338, 1299, 1260, 1220, 1179, 1138, 1096, 1053, 1009, 965,  921,  876,  830,
     784,  737,  690,  642,  595,  546,  498,  449,  400,  350,  301,  251,  201,  151,  100,  50},
    {4096, 4095, 4091, 4085, 4076, 4065, 4052, 4036, 4017, 3996, 3973, 3948, 3920, 3889, 3857, 3822,
     3784, 3745, 3703, 3659, 3612, 3564, 3513, 3461, 3406, 3349, 3290, 3229, 3166, 3102, 3035, 2967,
     2896, 2824, 2751, 2675, 2598, 2520, 2440, 2359, 2276, 2191, 2106, 2019, 1931, 1842, 1751, 1660,
     1567, 1474, 1380, 1285, 1189, 1092, 995,  897,  799,  700,  601,  501,  401,  301,  201,  101},
    {8192, 8190, 8182, 8170, 8153, 8130, 8103, 8071, 8035, 7993, 7946, 7895, 7839, 7779, 7713, 7643,
     7568, 7489, 7405, 7317, 7225, 7128, 7027, 6921, 6811, 6698, 6580, 6458, 6333, 6203, 6070, 5933,
     5793, 5649, 5501, 5351, 5197, 5040, 4880, 4717, 4551, 4383, 4212, 4038, 3862, 3683, 3503, 3320,
     3135, 2948, 2760, 2570, 2378, 2185, 1990, 1795, 1598, 1401, 1202, 1003, 803,  603,  402,  201},
    {16384, 16379, 16364, 16340, 16305, 16261, 16207, 16143, 16069, 15986, 15893, 15791, 15679,
     15557, 15426, 15286, 15137, 14978, 14811, 14635, 14449, 14256, 14053, 13842, 13623, 13395,
     13160, 12916, 12665, 12406, 12140, 11866, 11585, 11297, 11003, 10702, 10394, 10080, 9760,
     9434,  9102,  8765,  8423,  8076,  7723,  7366,  7005,  6639,  6270,  5897,  5520,  5139,
     4756,  4370,  3981,  3590,  3196,  2801,  2404,  2006,  1606,  1205,  804,   402},
    {32768, 32758, 32729, 32679, 32610, 32522, 32413, 32286, 32138, 31972, 31786, 31581, 31357,
     31114, 30853, 30572, 30274, 29957, 29622, 29269, 28899, 28511, 28106, 27684, 27246, 26791,
     26320, 25833, 25330, 24812, 24279, 23732, 23170, 22595, 22006, 21403, 20788, 20160, 19520,
     18868, 18205, 17531, 16846, 16151, 15447, 14733, 14010, 13279, 12540, 11793, 11039, 10279,
     9512,  8740,  7962,  7180,  6393,  5602,  4808,  4011,  3212,  2411,  1608,  804},
    {65536, 65516, 65457, 65358, 65220, 65043, 64827, 64571, 64277, 63944, 63572, 63162, 62714,
     62228, 61705, 61145, 60547, 59914, 59244, 58538, 57798, 57022, 56212, 55368, 54491, 53581,
     52639, 51665, 50660, 49624, 48559, 47464, 46341, 45190, 44011, 42806, 41576, 40320, 39040,
     37736, 36410, 35062, 33692, 32303, 30893, 29466, 28020, 26558, 25080, 23586, 22078, 20557,
     19024, 17479, 15924, 14359, 12785, 11204, 9616,  8022,  6424,  4821,  3216,  1608}};
static INLINE int32_t round_shift(int64_t value, int32_t bit) {
    assert(bit >= 1);
    return (int32_t)((value + (1ll << (bit - 1))) >> bit);
}
static INLINE int32_t half_btf(int32_t w0, int32_t in0, int32_t w1, int32_t in1, int32_t bit) {
    int64_t result_64 = (int64_t)(w0 * in0) + (int64_t)(w1 * in1);
#if CONFIG_COEFFICIENT_RANGE_CHECKING
    assert(result_64 >= INT32_MIN && result_64 <= INT32_MAX);
#endif
    return round_shift(result_64, bit);
}

/*!< eb_av1_sinpi_arr_data[i][j] = (int32_t)round((sqrt(2) * sin(j*Pi/9) * 2 / 3) * (1
 *   << (cos_bit_min + i))) modified so that elements j=1,2 sum to element j=4. */
const int32_t eb_av1_sinpi_arr_data[7][5] = {{0, 330, 621, 836, 951},
                                             {0, 660, 1241, 1672, 1901},
                                             {0, 1321, 2482, 3344, 3803},
                                             {0, 2642, 4964, 6689, 7606},
                                             {0, 5283, 9929, 13377, 15212},
                                             {0, 10566, 19858, 26755, 30424},
                                             {0, 21133, 39716, 53510, 60849}};

void eb_av1_fdct4_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                      const int8_t *stage_range) {
    const int32_t  size = 4;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[4];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    bf1    = output;
    bf1[0] = input[0] + input[3];
    bf1[1] = input[1] + input[2];
    bf1[2] = -input[2] + input[1];
    bf1[3] = -input[3] + input[0];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = output;
    bf1    = step;
    bf1[0] = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1] = half_btf(-cospi[32], bf0[1], cospi[32], bf0[0], cos_bit);
    bf1[2] = half_btf(cospi[48], bf0[2], cospi[16], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[48], bf0[3], -cospi[16], bf0[2], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = bf0[0];
    bf1[1] = bf0[2];
    bf1[2] = bf0[1];
    bf1[3] = bf0[3];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void eb_av1_fdct8_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                      const int8_t *stage_range) {
    const int32_t  size = 8;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[8];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    bf1    = output;
    bf1[0] = input[0] + input[7];
    bf1[1] = input[1] + input[6];
    bf1[2] = input[2] + input[5];
    bf1[3] = input[3] + input[4];
    bf1[4] = -input[4] + input[3];
    bf1[5] = -input[5] + input[2];
    bf1[6] = -input[6] + input[1];
    bf1[7] = -input[7] + input[0];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0] + bf0[3];
    bf1[1] = bf0[1] + bf0[2];
    bf1[2] = -bf0[2] + bf0[1];
    bf1[3] = -bf0[3] + bf0[0];
    bf1[4] = bf0[4];
    bf1[5] = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6] = half_btf(cospi[32], bf0[6], cospi[32], bf0[5], cos_bit);
    bf1[7] = bf0[7];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = step;
    bf1    = output;
    bf1[0] = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1] = half_btf(-cospi[32], bf0[1], cospi[32], bf0[0], cos_bit);
    bf1[2] = half_btf(cospi[48], bf0[2], cospi[16], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[48], bf0[3], -cospi[16], bf0[2], cos_bit);
    bf1[4] = bf0[4] + bf0[5];
    bf1[5] = -bf0[5] + bf0[4];
    bf1[6] = -bf0[6] + bf0[7];
    bf1[7] = bf0[7] + bf0[6];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0];
    bf1[1] = bf0[1];
    bf1[2] = bf0[2];
    bf1[3] = bf0[3];
    bf1[4] = half_btf(cospi[56], bf0[4], cospi[8], bf0[7], cos_bit);
    bf1[5] = half_btf(cospi[24], bf0[5], cospi[40], bf0[6], cos_bit);
    bf1[6] = half_btf(cospi[24], bf0[6], -cospi[40], bf0[5], cos_bit);
    bf1[7] = half_btf(cospi[56], bf0[7], -cospi[8], bf0[4], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = bf0[0];
    bf1[1] = bf0[4];
    bf1[2] = bf0[2];
    bf1[3] = bf0[6];
    bf1[4] = bf0[1];
    bf1[5] = bf0[5];
    bf1[6] = bf0[3];
    bf1[7] = bf0[7];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void eb_av1_fdct16_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    const int32_t  size = 16;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[16];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[0] + input[15];
    bf1[1]  = input[1] + input[14];
    bf1[2]  = input[2] + input[13];
    bf1[3]  = input[3] + input[12];
    bf1[4]  = input[4] + input[11];
    bf1[5]  = input[5] + input[10];
    bf1[6]  = input[6] + input[9];
    bf1[7]  = input[7] + input[8];
    bf1[8]  = -input[8] + input[7];
    bf1[9]  = -input[9] + input[6];
    bf1[10] = -input[10] + input[5];
    bf1[11] = -input[11] + input[4];
    bf1[12] = -input[12] + input[3];
    bf1[13] = -input[13] + input[2];
    bf1[14] = -input[14] + input[1];
    bf1[15] = -input[15] + input[0];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0] + bf0[7];
    bf1[1]  = bf0[1] + bf0[6];
    bf1[2]  = bf0[2] + bf0[5];
    bf1[3]  = bf0[3] + bf0[4];
    bf1[4]  = -bf0[4] + bf0[3];
    bf1[5]  = -bf0[5] + bf0[2];
    bf1[6]  = -bf0[6] + bf0[1];
    bf1[7]  = -bf0[7] + bf0[0];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(-cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[32], bf0[12], cospi[32], bf0[11], cos_bit);
    bf1[13] = half_btf(cospi[32], bf0[13], cospi[32], bf0[10], cos_bit);
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[3];
    bf1[1]  = bf0[1] + bf0[2];
    bf1[2]  = -bf0[2] + bf0[1];
    bf1[3]  = -bf0[3] + bf0[0];
    bf1[4]  = bf0[4];
    bf1[5]  = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[5], cos_bit);
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8] + bf0[11];
    bf1[9]  = bf0[9] + bf0[10];
    bf1[10] = -bf0[10] + bf0[9];
    bf1[11] = -bf0[11] + bf0[8];
    bf1[12] = -bf0[12] + bf0[15];
    bf1[13] = -bf0[13] + bf0[14];
    bf1[14] = bf0[14] + bf0[13];
    bf1[15] = bf0[15] + bf0[12];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1]  = half_btf(-cospi[32], bf0[1], cospi[32], bf0[0], cos_bit);
    bf1[2]  = half_btf(cospi[48], bf0[2], cospi[16], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[48], bf0[3], -cospi[16], bf0[2], cos_bit);
    bf1[4]  = bf0[4] + bf0[5];
    bf1[5]  = -bf0[5] + bf0[4];
    bf1[6]  = -bf0[6] + bf0[7];
    bf1[7]  = bf0[7] + bf0[6];
    bf1[8]  = bf0[8];
    bf1[9]  = half_btf(-cospi[16], bf0[9], cospi[48], bf0[14], cos_bit);
    bf1[10] = half_btf(-cospi[48], bf0[10], -cospi[16], bf0[13], cos_bit);
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = half_btf(cospi[48], bf0[13], -cospi[16], bf0[10], cos_bit);
    bf1[14] = half_btf(cospi[16], bf0[14], cospi[48], bf0[9], cos_bit);
    bf1[15] = bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[56], bf0[4], cospi[8], bf0[7], cos_bit);
    bf1[5]  = half_btf(cospi[24], bf0[5], cospi[40], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[24], bf0[6], -cospi[40], bf0[5], cos_bit);
    bf1[7]  = half_btf(cospi[56], bf0[7], -cospi[8], bf0[4], cos_bit);
    bf1[8]  = bf0[8] + bf0[9];
    bf1[9]  = -bf0[9] + bf0[8];
    bf1[10] = -bf0[10] + bf0[11];
    bf1[11] = bf0[11] + bf0[10];
    bf1[12] = bf0[12] + bf0[13];
    bf1[13] = -bf0[13] + bf0[12];
    bf1[14] = -bf0[14] + bf0[15];
    bf1[15] = bf0[15] + bf0[14];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[60], bf0[8], cospi[4], bf0[15], cos_bit);
    bf1[9]  = half_btf(cospi[28], bf0[9], cospi[36], bf0[14], cos_bit);
    bf1[10] = half_btf(cospi[44], bf0[10], cospi[20], bf0[13], cos_bit);
    bf1[11] = half_btf(cospi[12], bf0[11], cospi[52], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[12], bf0[12], -cospi[52], bf0[11], cos_bit);
    bf1[13] = half_btf(cospi[44], bf0[13], -cospi[20], bf0[10], cos_bit);
    bf1[14] = half_btf(cospi[28], bf0[14], -cospi[36], bf0[9], cos_bit);
    bf1[15] = half_btf(cospi[60], bf0[15], -cospi[4], bf0[8], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[8];
    bf1[2]  = bf0[4];
    bf1[3]  = bf0[12];
    bf1[4]  = bf0[2];
    bf1[5]  = bf0[10];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[14];
    bf1[8]  = bf0[1];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[5];
    bf1[11] = bf0[13];
    bf1[12] = bf0[3];
    bf1[13] = bf0[11];
    bf1[14] = bf0[7];
    bf1[15] = bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void eb_av1_fdct32_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    const int32_t  size = 32;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[32];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[0] + input[31];
    bf1[1]  = input[1] + input[30];
    bf1[2]  = input[2] + input[29];
    bf1[3]  = input[3] + input[28];
    bf1[4]  = input[4] + input[27];
    bf1[5]  = input[5] + input[26];
    bf1[6]  = input[6] + input[25];
    bf1[7]  = input[7] + input[24];
    bf1[8]  = input[8] + input[23];
    bf1[9]  = input[9] + input[22];
    bf1[10] = input[10] + input[21];
    bf1[11] = input[11] + input[20];
    bf1[12] = input[12] + input[19];
    bf1[13] = input[13] + input[18];
    bf1[14] = input[14] + input[17];
    bf1[15] = input[15] + input[16];
    bf1[16] = -input[16] + input[15];
    bf1[17] = -input[17] + input[14];
    bf1[18] = -input[18] + input[13];
    bf1[19] = -input[19] + input[12];
    bf1[20] = -input[20] + input[11];
    bf1[21] = -input[21] + input[10];
    bf1[22] = -input[22] + input[9];
    bf1[23] = -input[23] + input[8];
    bf1[24] = -input[24] + input[7];
    bf1[25] = -input[25] + input[6];
    bf1[26] = -input[26] + input[5];
    bf1[27] = -input[27] + input[4];
    bf1[28] = -input[28] + input[3];
    bf1[29] = -input[29] + input[2];
    bf1[30] = -input[30] + input[1];
    bf1[31] = -input[31] + input[0];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0] + bf0[15];
    bf1[1]  = bf0[1] + bf0[14];
    bf1[2]  = bf0[2] + bf0[13];
    bf1[3]  = bf0[3] + bf0[12];
    bf1[4]  = bf0[4] + bf0[11];
    bf1[5]  = bf0[5] + bf0[10];
    bf1[6]  = bf0[6] + bf0[9];
    bf1[7]  = bf0[7] + bf0[8];
    bf1[8]  = -bf0[8] + bf0[7];
    bf1[9]  = -bf0[9] + bf0[6];
    bf1[10] = -bf0[10] + bf0[5];
    bf1[11] = -bf0[11] + bf0[4];
    bf1[12] = -bf0[12] + bf0[3];
    bf1[13] = -bf0[13] + bf0[2];
    bf1[14] = -bf0[14] + bf0[1];
    bf1[15] = -bf0[15] + bf0[0];
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = half_btf(-cospi[32], bf0[20], cospi[32], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[32], bf0[21], cospi[32], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[32], bf0[22], cospi[32], bf0[25], cos_bit);
    bf1[23] = half_btf(-cospi[32], bf0[23], cospi[32], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[32], bf0[24], cospi[32], bf0[23], cos_bit);
    bf1[25] = half_btf(cospi[32], bf0[25], cospi[32], bf0[22], cos_bit);
    bf1[26] = half_btf(cospi[32], bf0[26], cospi[32], bf0[21], cos_bit);
    bf1[27] = half_btf(cospi[32], bf0[27], cospi[32], bf0[20], cos_bit);
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[7];
    bf1[1]  = bf0[1] + bf0[6];
    bf1[2]  = bf0[2] + bf0[5];
    bf1[3]  = bf0[3] + bf0[4];
    bf1[4]  = -bf0[4] + bf0[3];
    bf1[5]  = -bf0[5] + bf0[2];
    bf1[6]  = -bf0[6] + bf0[1];
    bf1[7]  = -bf0[7] + bf0[0];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(-cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[32], bf0[12], cospi[32], bf0[11], cos_bit);
    bf1[13] = half_btf(cospi[32], bf0[13], cospi[32], bf0[10], cos_bit);
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = bf0[16] + bf0[23];
    bf1[17] = bf0[17] + bf0[22];
    bf1[18] = bf0[18] + bf0[21];
    bf1[19] = bf0[19] + bf0[20];
    bf1[20] = -bf0[20] + bf0[19];
    bf1[21] = -bf0[21] + bf0[18];
    bf1[22] = -bf0[22] + bf0[17];
    bf1[23] = -bf0[23] + bf0[16];
    bf1[24] = -bf0[24] + bf0[31];
    bf1[25] = -bf0[25] + bf0[30];
    bf1[26] = -bf0[26] + bf0[29];
    bf1[27] = -bf0[27] + bf0[28];
    bf1[28] = bf0[28] + bf0[27];
    bf1[29] = bf0[29] + bf0[26];
    bf1[30] = bf0[30] + bf0[25];
    bf1[31] = bf0[31] + bf0[24];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0] + bf0[3];
    bf1[1]  = bf0[1] + bf0[2];
    bf1[2]  = -bf0[2] + bf0[1];
    bf1[3]  = -bf0[3] + bf0[0];
    bf1[4]  = bf0[4];
    bf1[5]  = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[5], cos_bit);
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8] + bf0[11];
    bf1[9]  = bf0[9] + bf0[10];
    bf1[10] = -bf0[10] + bf0[9];
    bf1[11] = -bf0[11] + bf0[8];
    bf1[12] = -bf0[12] + bf0[15];
    bf1[13] = -bf0[13] + bf0[14];
    bf1[14] = bf0[14] + bf0[13];
    bf1[15] = bf0[15] + bf0[12];
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = half_btf(-cospi[16], bf0[18], cospi[48], bf0[29], cos_bit);
    bf1[19] = half_btf(-cospi[16], bf0[19], cospi[48], bf0[28], cos_bit);
    bf1[20] = half_btf(-cospi[48], bf0[20], -cospi[16], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[48], bf0[21], -cospi[16], bf0[26], cos_bit);
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = half_btf(cospi[48], bf0[26], -cospi[16], bf0[21], cos_bit);
    bf1[27] = half_btf(cospi[48], bf0[27], -cospi[16], bf0[20], cos_bit);
    bf1[28] = half_btf(cospi[16], bf0[28], cospi[48], bf0[19], cos_bit);
    bf1[29] = half_btf(cospi[16], bf0[29], cospi[48], bf0[18], cos_bit);
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1]  = half_btf(-cospi[32], bf0[1], cospi[32], bf0[0], cos_bit);
    bf1[2]  = half_btf(cospi[48], bf0[2], cospi[16], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[48], bf0[3], -cospi[16], bf0[2], cos_bit);
    bf1[4]  = bf0[4] + bf0[5];
    bf1[5]  = -bf0[5] + bf0[4];
    bf1[6]  = -bf0[6] + bf0[7];
    bf1[7]  = bf0[7] + bf0[6];
    bf1[8]  = bf0[8];
    bf1[9]  = half_btf(-cospi[16], bf0[9], cospi[48], bf0[14], cos_bit);
    bf1[10] = half_btf(-cospi[48], bf0[10], -cospi[16], bf0[13], cos_bit);
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = half_btf(cospi[48], bf0[13], -cospi[16], bf0[10], cos_bit);
    bf1[14] = half_btf(cospi[16], bf0[14], cospi[48], bf0[9], cos_bit);
    bf1[15] = bf0[15];
    bf1[16] = bf0[16] + bf0[19];
    bf1[17] = bf0[17] + bf0[18];
    bf1[18] = -bf0[18] + bf0[17];
    bf1[19] = -bf0[19] + bf0[16];
    bf1[20] = -bf0[20] + bf0[23];
    bf1[21] = -bf0[21] + bf0[22];
    bf1[22] = bf0[22] + bf0[21];
    bf1[23] = bf0[23] + bf0[20];
    bf1[24] = bf0[24] + bf0[27];
    bf1[25] = bf0[25] + bf0[26];
    bf1[26] = -bf0[26] + bf0[25];
    bf1[27] = -bf0[27] + bf0[24];
    bf1[28] = -bf0[28] + bf0[31];
    bf1[29] = -bf0[29] + bf0[30];
    bf1[30] = bf0[30] + bf0[29];
    bf1[31] = bf0[31] + bf0[28];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[56], bf0[4], cospi[8], bf0[7], cos_bit);
    bf1[5]  = half_btf(cospi[24], bf0[5], cospi[40], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[24], bf0[6], -cospi[40], bf0[5], cos_bit);
    bf1[7]  = half_btf(cospi[56], bf0[7], -cospi[8], bf0[4], cos_bit);
    bf1[8]  = bf0[8] + bf0[9];
    bf1[9]  = -bf0[9] + bf0[8];
    bf1[10] = -bf0[10] + bf0[11];
    bf1[11] = bf0[11] + bf0[10];
    bf1[12] = bf0[12] + bf0[13];
    bf1[13] = -bf0[13] + bf0[12];
    bf1[14] = -bf0[14] + bf0[15];
    bf1[15] = bf0[15] + bf0[14];
    bf1[16] = bf0[16];
    bf1[17] = half_btf(-cospi[8], bf0[17], cospi[56], bf0[30], cos_bit);
    bf1[18] = half_btf(-cospi[56], bf0[18], -cospi[8], bf0[29], cos_bit);
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = half_btf(-cospi[40], bf0[21], cospi[24], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[24], bf0[22], -cospi[40], bf0[25], cos_bit);
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = half_btf(cospi[24], bf0[25], -cospi[40], bf0[22], cos_bit);
    bf1[26] = half_btf(cospi[40], bf0[26], cospi[24], bf0[21], cos_bit);
    bf1[27] = bf0[27];
    bf1[28] = bf0[28];
    bf1[29] = half_btf(cospi[56], bf0[29], -cospi[8], bf0[18], cos_bit);
    bf1[30] = half_btf(cospi[8], bf0[30], cospi[56], bf0[17], cos_bit);
    bf1[31] = bf0[31];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[60], bf0[8], cospi[4], bf0[15], cos_bit);
    bf1[9]  = half_btf(cospi[28], bf0[9], cospi[36], bf0[14], cos_bit);
    bf1[10] = half_btf(cospi[44], bf0[10], cospi[20], bf0[13], cos_bit);
    bf1[11] = half_btf(cospi[12], bf0[11], cospi[52], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[12], bf0[12], -cospi[52], bf0[11], cos_bit);
    bf1[13] = half_btf(cospi[44], bf0[13], -cospi[20], bf0[10], cos_bit);
    bf1[14] = half_btf(cospi[28], bf0[14], -cospi[36], bf0[9], cos_bit);
    bf1[15] = half_btf(cospi[60], bf0[15], -cospi[4], bf0[8], cos_bit);
    bf1[16] = bf0[16] + bf0[17];
    bf1[17] = -bf0[17] + bf0[16];
    bf1[18] = -bf0[18] + bf0[19];
    bf1[19] = bf0[19] + bf0[18];
    bf1[20] = bf0[20] + bf0[21];
    bf1[21] = -bf0[21] + bf0[20];
    bf1[22] = -bf0[22] + bf0[23];
    bf1[23] = bf0[23] + bf0[22];
    bf1[24] = bf0[24] + bf0[25];
    bf1[25] = -bf0[25] + bf0[24];
    bf1[26] = -bf0[26] + bf0[27];
    bf1[27] = bf0[27] + bf0[26];
    bf1[28] = bf0[28] + bf0[29];
    bf1[29] = -bf0[29] + bf0[28];
    bf1[30] = -bf0[30] + bf0[31];
    bf1[31] = bf0[31] + bf0[30];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = half_btf(cospi[62], bf0[16], cospi[2], bf0[31], cos_bit);
    bf1[17] = half_btf(cospi[30], bf0[17], cospi[34], bf0[30], cos_bit);
    bf1[18] = half_btf(cospi[46], bf0[18], cospi[18], bf0[29], cos_bit);
    bf1[19] = half_btf(cospi[14], bf0[19], cospi[50], bf0[28], cos_bit);
    bf1[20] = half_btf(cospi[54], bf0[20], cospi[10], bf0[27], cos_bit);
    bf1[21] = half_btf(cospi[22], bf0[21], cospi[42], bf0[26], cos_bit);
    bf1[22] = half_btf(cospi[38], bf0[22], cospi[26], bf0[25], cos_bit);
    bf1[23] = half_btf(cospi[6], bf0[23], cospi[58], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[6], bf0[24], -cospi[58], bf0[23], cos_bit);
    bf1[25] = half_btf(cospi[38], bf0[25], -cospi[26], bf0[22], cos_bit);
    bf1[26] = half_btf(cospi[22], bf0[26], -cospi[42], bf0[21], cos_bit);
    bf1[27] = half_btf(cospi[54], bf0[27], -cospi[10], bf0[20], cos_bit);
    bf1[28] = half_btf(cospi[14], bf0[28], -cospi[50], bf0[19], cos_bit);
    bf1[29] = half_btf(cospi[46], bf0[29], -cospi[18], bf0[18], cos_bit);
    bf1[30] = half_btf(cospi[30], bf0[30], -cospi[34], bf0[17], cos_bit);
    bf1[31] = half_btf(cospi[62], bf0[31], -cospi[2], bf0[16], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[16];
    bf1[2]  = bf0[8];
    bf1[3]  = bf0[24];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[20];
    bf1[6]  = bf0[12];
    bf1[7]  = bf0[28];
    bf1[8]  = bf0[2];
    bf1[9]  = bf0[18];
    bf1[10] = bf0[10];
    bf1[11] = bf0[26];
    bf1[12] = bf0[6];
    bf1[13] = bf0[22];
    bf1[14] = bf0[14];
    bf1[15] = bf0[30];
    bf1[16] = bf0[1];
    bf1[17] = bf0[17];
    bf1[18] = bf0[9];
    bf1[19] = bf0[25];
    bf1[20] = bf0[5];
    bf1[21] = bf0[21];
    bf1[22] = bf0[13];
    bf1[23] = bf0[29];
    bf1[24] = bf0[3];
    bf1[25] = bf0[19];
    bf1[26] = bf0[11];
    bf1[27] = bf0[27];
    bf1[28] = bf0[7];
    bf1[29] = bf0[23];
    bf1[30] = bf0[15];
    bf1[31] = bf0[31];
    range_check(stage, input, bf1, size, stage_range[stage]);
}
void eb_av1_fdct64_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    const int32_t  size = 64;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[64];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[0] + input[63];
    bf1[1]  = input[1] + input[62];
    bf1[2]  = input[2] + input[61];
    bf1[3]  = input[3] + input[60];
    bf1[4]  = input[4] + input[59];
    bf1[5]  = input[5] + input[58];
    bf1[6]  = input[6] + input[57];
    bf1[7]  = input[7] + input[56];
    bf1[8]  = input[8] + input[55];
    bf1[9]  = input[9] + input[54];
    bf1[10] = input[10] + input[53];
    bf1[11] = input[11] + input[52];
    bf1[12] = input[12] + input[51];
    bf1[13] = input[13] + input[50];
    bf1[14] = input[14] + input[49];
    bf1[15] = input[15] + input[48];
    bf1[16] = input[16] + input[47];
    bf1[17] = input[17] + input[46];
    bf1[18] = input[18] + input[45];
    bf1[19] = input[19] + input[44];
    bf1[20] = input[20] + input[43];
    bf1[21] = input[21] + input[42];
    bf1[22] = input[22] + input[41];
    bf1[23] = input[23] + input[40];
    bf1[24] = input[24] + input[39];
    bf1[25] = input[25] + input[38];
    bf1[26] = input[26] + input[37];
    bf1[27] = input[27] + input[36];
    bf1[28] = input[28] + input[35];
    bf1[29] = input[29] + input[34];
    bf1[30] = input[30] + input[33];
    bf1[31] = input[31] + input[32];
    bf1[32] = -input[32] + input[31];
    bf1[33] = -input[33] + input[30];
    bf1[34] = -input[34] + input[29];
    bf1[35] = -input[35] + input[28];
    bf1[36] = -input[36] + input[27];
    bf1[37] = -input[37] + input[26];
    bf1[38] = -input[38] + input[25];
    bf1[39] = -input[39] + input[24];
    bf1[40] = -input[40] + input[23];
    bf1[41] = -input[41] + input[22];
    bf1[42] = -input[42] + input[21];
    bf1[43] = -input[43] + input[20];
    bf1[44] = -input[44] + input[19];
    bf1[45] = -input[45] + input[18];
    bf1[46] = -input[46] + input[17];
    bf1[47] = -input[47] + input[16];
    bf1[48] = -input[48] + input[15];
    bf1[49] = -input[49] + input[14];
    bf1[50] = -input[50] + input[13];
    bf1[51] = -input[51] + input[12];
    bf1[52] = -input[52] + input[11];
    bf1[53] = -input[53] + input[10];
    bf1[54] = -input[54] + input[9];
    bf1[55] = -input[55] + input[8];
    bf1[56] = -input[56] + input[7];
    bf1[57] = -input[57] + input[6];
    bf1[58] = -input[58] + input[5];
    bf1[59] = -input[59] + input[4];
    bf1[60] = -input[60] + input[3];
    bf1[61] = -input[61] + input[2];
    bf1[62] = -input[62] + input[1];
    bf1[63] = -input[63] + input[0];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0] + bf0[31];
    bf1[1]  = bf0[1] + bf0[30];
    bf1[2]  = bf0[2] + bf0[29];
    bf1[3]  = bf0[3] + bf0[28];
    bf1[4]  = bf0[4] + bf0[27];
    bf1[5]  = bf0[5] + bf0[26];
    bf1[6]  = bf0[6] + bf0[25];
    bf1[7]  = bf0[7] + bf0[24];
    bf1[8]  = bf0[8] + bf0[23];
    bf1[9]  = bf0[9] + bf0[22];
    bf1[10] = bf0[10] + bf0[21];
    bf1[11] = bf0[11] + bf0[20];
    bf1[12] = bf0[12] + bf0[19];
    bf1[13] = bf0[13] + bf0[18];
    bf1[14] = bf0[14] + bf0[17];
    bf1[15] = bf0[15] + bf0[16];
    bf1[16] = -bf0[16] + bf0[15];
    bf1[17] = -bf0[17] + bf0[14];
    bf1[18] = -bf0[18] + bf0[13];
    bf1[19] = -bf0[19] + bf0[12];
    bf1[20] = -bf0[20] + bf0[11];
    bf1[21] = -bf0[21] + bf0[10];
    bf1[22] = -bf0[22] + bf0[9];
    bf1[23] = -bf0[23] + bf0[8];
    bf1[24] = -bf0[24] + bf0[7];
    bf1[25] = -bf0[25] + bf0[6];
    bf1[26] = -bf0[26] + bf0[5];
    bf1[27] = -bf0[27] + bf0[4];
    bf1[28] = -bf0[28] + bf0[3];
    bf1[29] = -bf0[29] + bf0[2];
    bf1[30] = -bf0[30] + bf0[1];
    bf1[31] = -bf0[31] + bf0[0];
    bf1[32] = bf0[32];
    bf1[33] = bf0[33];
    bf1[34] = bf0[34];
    bf1[35] = bf0[35];
    bf1[36] = bf0[36];
    bf1[37] = bf0[37];
    bf1[38] = bf0[38];
    bf1[39] = bf0[39];
    bf1[40] = half_btf(-cospi[32], bf0[40], cospi[32], bf0[55], cos_bit);
    bf1[41] = half_btf(-cospi[32], bf0[41], cospi[32], bf0[54], cos_bit);
    bf1[42] = half_btf(-cospi[32], bf0[42], cospi[32], bf0[53], cos_bit);
    bf1[43] = half_btf(-cospi[32], bf0[43], cospi[32], bf0[52], cos_bit);
    bf1[44] = half_btf(-cospi[32], bf0[44], cospi[32], bf0[51], cos_bit);
    bf1[45] = half_btf(-cospi[32], bf0[45], cospi[32], bf0[50], cos_bit);
    bf1[46] = half_btf(-cospi[32], bf0[46], cospi[32], bf0[49], cos_bit);
    bf1[47] = half_btf(-cospi[32], bf0[47], cospi[32], bf0[48], cos_bit);
    bf1[48] = half_btf(cospi[32], bf0[48], cospi[32], bf0[47], cos_bit);
    bf1[49] = half_btf(cospi[32], bf0[49], cospi[32], bf0[46], cos_bit);
    bf1[50] = half_btf(cospi[32], bf0[50], cospi[32], bf0[45], cos_bit);
    bf1[51] = half_btf(cospi[32], bf0[51], cospi[32], bf0[44], cos_bit);
    bf1[52] = half_btf(cospi[32], bf0[52], cospi[32], bf0[43], cos_bit);
    bf1[53] = half_btf(cospi[32], bf0[53], cospi[32], bf0[42], cos_bit);
    bf1[54] = half_btf(cospi[32], bf0[54], cospi[32], bf0[41], cos_bit);
    bf1[55] = half_btf(cospi[32], bf0[55], cospi[32], bf0[40], cos_bit);
    bf1[56] = bf0[56];
    bf1[57] = bf0[57];
    bf1[58] = bf0[58];
    bf1[59] = bf0[59];
    bf1[60] = bf0[60];
    bf1[61] = bf0[61];
    bf1[62] = bf0[62];
    bf1[63] = bf0[63];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[15];
    bf1[1]  = bf0[1] + bf0[14];
    bf1[2]  = bf0[2] + bf0[13];
    bf1[3]  = bf0[3] + bf0[12];
    bf1[4]  = bf0[4] + bf0[11];
    bf1[5]  = bf0[5] + bf0[10];
    bf1[6]  = bf0[6] + bf0[9];
    bf1[7]  = bf0[7] + bf0[8];
    bf1[8]  = -bf0[8] + bf0[7];
    bf1[9]  = -bf0[9] + bf0[6];
    bf1[10] = -bf0[10] + bf0[5];
    bf1[11] = -bf0[11] + bf0[4];
    bf1[12] = -bf0[12] + bf0[3];
    bf1[13] = -bf0[13] + bf0[2];
    bf1[14] = -bf0[14] + bf0[1];
    bf1[15] = -bf0[15] + bf0[0];
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = half_btf(-cospi[32], bf0[20], cospi[32], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[32], bf0[21], cospi[32], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[32], bf0[22], cospi[32], bf0[25], cos_bit);
    bf1[23] = half_btf(-cospi[32], bf0[23], cospi[32], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[32], bf0[24], cospi[32], bf0[23], cos_bit);
    bf1[25] = half_btf(cospi[32], bf0[25], cospi[32], bf0[22], cos_bit);
    bf1[26] = half_btf(cospi[32], bf0[26], cospi[32], bf0[21], cos_bit);
    bf1[27] = half_btf(cospi[32], bf0[27], cospi[32], bf0[20], cos_bit);
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    bf1[32] = bf0[32] + bf0[47];
    bf1[33] = bf0[33] + bf0[46];
    bf1[34] = bf0[34] + bf0[45];
    bf1[35] = bf0[35] + bf0[44];
    bf1[36] = bf0[36] + bf0[43];
    bf1[37] = bf0[37] + bf0[42];
    bf1[38] = bf0[38] + bf0[41];
    bf1[39] = bf0[39] + bf0[40];
    bf1[40] = -bf0[40] + bf0[39];
    bf1[41] = -bf0[41] + bf0[38];
    bf1[42] = -bf0[42] + bf0[37];
    bf1[43] = -bf0[43] + bf0[36];
    bf1[44] = -bf0[44] + bf0[35];
    bf1[45] = -bf0[45] + bf0[34];
    bf1[46] = -bf0[46] + bf0[33];
    bf1[47] = -bf0[47] + bf0[32];
    bf1[48] = -bf0[48] + bf0[63];
    bf1[49] = -bf0[49] + bf0[62];
    bf1[50] = -bf0[50] + bf0[61];
    bf1[51] = -bf0[51] + bf0[60];
    bf1[52] = -bf0[52] + bf0[59];
    bf1[53] = -bf0[53] + bf0[58];
    bf1[54] = -bf0[54] + bf0[57];
    bf1[55] = -bf0[55] + bf0[56];
    bf1[56] = bf0[56] + bf0[55];
    bf1[57] = bf0[57] + bf0[54];
    bf1[58] = bf0[58] + bf0[53];
    bf1[59] = bf0[59] + bf0[52];
    bf1[60] = bf0[60] + bf0[51];
    bf1[61] = bf0[61] + bf0[50];
    bf1[62] = bf0[62] + bf0[49];
    bf1[63] = bf0[63] + bf0[48];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0] + bf0[7];
    bf1[1]  = bf0[1] + bf0[6];
    bf1[2]  = bf0[2] + bf0[5];
    bf1[3]  = bf0[3] + bf0[4];
    bf1[4]  = -bf0[4] + bf0[3];
    bf1[5]  = -bf0[5] + bf0[2];
    bf1[6]  = -bf0[6] + bf0[1];
    bf1[7]  = -bf0[7] + bf0[0];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(-cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[32], bf0[12], cospi[32], bf0[11], cos_bit);
    bf1[13] = half_btf(cospi[32], bf0[13], cospi[32], bf0[10], cos_bit);
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = bf0[16] + bf0[23];
    bf1[17] = bf0[17] + bf0[22];
    bf1[18] = bf0[18] + bf0[21];
    bf1[19] = bf0[19] + bf0[20];
    bf1[20] = -bf0[20] + bf0[19];
    bf1[21] = -bf0[21] + bf0[18];
    bf1[22] = -bf0[22] + bf0[17];
    bf1[23] = -bf0[23] + bf0[16];
    bf1[24] = -bf0[24] + bf0[31];
    bf1[25] = -bf0[25] + bf0[30];
    bf1[26] = -bf0[26] + bf0[29];
    bf1[27] = -bf0[27] + bf0[28];
    bf1[28] = bf0[28] + bf0[27];
    bf1[29] = bf0[29] + bf0[26];
    bf1[30] = bf0[30] + bf0[25];
    bf1[31] = bf0[31] + bf0[24];
    bf1[32] = bf0[32];
    bf1[33] = bf0[33];
    bf1[34] = bf0[34];
    bf1[35] = bf0[35];
    bf1[36] = half_btf(-cospi[16], bf0[36], cospi[48], bf0[59], cos_bit);
    bf1[37] = half_btf(-cospi[16], bf0[37], cospi[48], bf0[58], cos_bit);
    bf1[38] = half_btf(-cospi[16], bf0[38], cospi[48], bf0[57], cos_bit);
    bf1[39] = half_btf(-cospi[16], bf0[39], cospi[48], bf0[56], cos_bit);
    bf1[40] = half_btf(-cospi[48], bf0[40], -cospi[16], bf0[55], cos_bit);
    bf1[41] = half_btf(-cospi[48], bf0[41], -cospi[16], bf0[54], cos_bit);
    bf1[42] = half_btf(-cospi[48], bf0[42], -cospi[16], bf0[53], cos_bit);
    bf1[43] = half_btf(-cospi[48], bf0[43], -cospi[16], bf0[52], cos_bit);
    bf1[44] = bf0[44];
    bf1[45] = bf0[45];
    bf1[46] = bf0[46];
    bf1[47] = bf0[47];
    bf1[48] = bf0[48];
    bf1[49] = bf0[49];
    bf1[50] = bf0[50];
    bf1[51] = bf0[51];
    bf1[52] = half_btf(cospi[48], bf0[52], -cospi[16], bf0[43], cos_bit);
    bf1[53] = half_btf(cospi[48], bf0[53], -cospi[16], bf0[42], cos_bit);
    bf1[54] = half_btf(cospi[48], bf0[54], -cospi[16], bf0[41], cos_bit);
    bf1[55] = half_btf(cospi[48], bf0[55], -cospi[16], bf0[40], cos_bit);
    bf1[56] = half_btf(cospi[16], bf0[56], cospi[48], bf0[39], cos_bit);
    bf1[57] = half_btf(cospi[16], bf0[57], cospi[48], bf0[38], cos_bit);
    bf1[58] = half_btf(cospi[16], bf0[58], cospi[48], bf0[37], cos_bit);
    bf1[59] = half_btf(cospi[16], bf0[59], cospi[48], bf0[36], cos_bit);
    bf1[60] = bf0[60];
    bf1[61] = bf0[61];
    bf1[62] = bf0[62];
    bf1[63] = bf0[63];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[3];
    bf1[1]  = bf0[1] + bf0[2];
    bf1[2]  = -bf0[2] + bf0[1];
    bf1[3]  = -bf0[3] + bf0[0];
    bf1[4]  = bf0[4];
    bf1[5]  = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[5], cos_bit);
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8] + bf0[11];
    bf1[9]  = bf0[9] + bf0[10];
    bf1[10] = -bf0[10] + bf0[9];
    bf1[11] = -bf0[11] + bf0[8];
    bf1[12] = -bf0[12] + bf0[15];
    bf1[13] = -bf0[13] + bf0[14];
    bf1[14] = bf0[14] + bf0[13];
    bf1[15] = bf0[15] + bf0[12];
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = half_btf(-cospi[16], bf0[18], cospi[48], bf0[29], cos_bit);
    bf1[19] = half_btf(-cospi[16], bf0[19], cospi[48], bf0[28], cos_bit);
    bf1[20] = half_btf(-cospi[48], bf0[20], -cospi[16], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[48], bf0[21], -cospi[16], bf0[26], cos_bit);
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = half_btf(cospi[48], bf0[26], -cospi[16], bf0[21], cos_bit);
    bf1[27] = half_btf(cospi[48], bf0[27], -cospi[16], bf0[20], cos_bit);
    bf1[28] = half_btf(cospi[16], bf0[28], cospi[48], bf0[19], cos_bit);
    bf1[29] = half_btf(cospi[16], bf0[29], cospi[48], bf0[18], cos_bit);
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    bf1[32] = bf0[32] + bf0[39];
    bf1[33] = bf0[33] + bf0[38];
    bf1[34] = bf0[34] + bf0[37];
    bf1[35] = bf0[35] + bf0[36];
    bf1[36] = -bf0[36] + bf0[35];
    bf1[37] = -bf0[37] + bf0[34];
    bf1[38] = -bf0[38] + bf0[33];
    bf1[39] = -bf0[39] + bf0[32];
    bf1[40] = -bf0[40] + bf0[47];
    bf1[41] = -bf0[41] + bf0[46];
    bf1[42] = -bf0[42] + bf0[45];
    bf1[43] = -bf0[43] + bf0[44];
    bf1[44] = bf0[44] + bf0[43];
    bf1[45] = bf0[45] + bf0[42];
    bf1[46] = bf0[46] + bf0[41];
    bf1[47] = bf0[47] + bf0[40];
    bf1[48] = bf0[48] + bf0[55];
    bf1[49] = bf0[49] + bf0[54];
    bf1[50] = bf0[50] + bf0[53];
    bf1[51] = bf0[51] + bf0[52];
    bf1[52] = -bf0[52] + bf0[51];
    bf1[53] = -bf0[53] + bf0[50];
    bf1[54] = -bf0[54] + bf0[49];
    bf1[55] = -bf0[55] + bf0[48];
    bf1[56] = -bf0[56] + bf0[63];
    bf1[57] = -bf0[57] + bf0[62];
    bf1[58] = -bf0[58] + bf0[61];
    bf1[59] = -bf0[59] + bf0[60];
    bf1[60] = bf0[60] + bf0[59];
    bf1[61] = bf0[61] + bf0[58];
    bf1[62] = bf0[62] + bf0[57];
    bf1[63] = bf0[63] + bf0[56];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1]  = half_btf(-cospi[32], bf0[1], cospi[32], bf0[0], cos_bit);
    bf1[2]  = half_btf(cospi[48], bf0[2], cospi[16], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[48], bf0[3], -cospi[16], bf0[2], cos_bit);
    bf1[4]  = bf0[4] + bf0[5];
    bf1[5]  = -bf0[5] + bf0[4];
    bf1[6]  = -bf0[6] + bf0[7];
    bf1[7]  = bf0[7] + bf0[6];
    bf1[8]  = bf0[8];
    bf1[9]  = half_btf(-cospi[16], bf0[9], cospi[48], bf0[14], cos_bit);
    bf1[10] = half_btf(-cospi[48], bf0[10], -cospi[16], bf0[13], cos_bit);
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = half_btf(cospi[48], bf0[13], -cospi[16], bf0[10], cos_bit);
    bf1[14] = half_btf(cospi[16], bf0[14], cospi[48], bf0[9], cos_bit);
    bf1[15] = bf0[15];
    bf1[16] = bf0[16] + bf0[19];
    bf1[17] = bf0[17] + bf0[18];
    bf1[18] = -bf0[18] + bf0[17];
    bf1[19] = -bf0[19] + bf0[16];
    bf1[20] = -bf0[20] + bf0[23];
    bf1[21] = -bf0[21] + bf0[22];
    bf1[22] = bf0[22] + bf0[21];
    bf1[23] = bf0[23] + bf0[20];
    bf1[24] = bf0[24] + bf0[27];
    bf1[25] = bf0[25] + bf0[26];
    bf1[26] = -bf0[26] + bf0[25];
    bf1[27] = -bf0[27] + bf0[24];
    bf1[28] = -bf0[28] + bf0[31];
    bf1[29] = -bf0[29] + bf0[30];
    bf1[30] = bf0[30] + bf0[29];
    bf1[31] = bf0[31] + bf0[28];
    bf1[32] = bf0[32];
    bf1[33] = bf0[33];
    bf1[34] = half_btf(-cospi[8], bf0[34], cospi[56], bf0[61], cos_bit);
    bf1[35] = half_btf(-cospi[8], bf0[35], cospi[56], bf0[60], cos_bit);
    bf1[36] = half_btf(-cospi[56], bf0[36], -cospi[8], bf0[59], cos_bit);
    bf1[37] = half_btf(-cospi[56], bf0[37], -cospi[8], bf0[58], cos_bit);
    bf1[38] = bf0[38];
    bf1[39] = bf0[39];
    bf1[40] = bf0[40];
    bf1[41] = bf0[41];
    bf1[42] = half_btf(-cospi[40], bf0[42], cospi[24], bf0[53], cos_bit);
    bf1[43] = half_btf(-cospi[40], bf0[43], cospi[24], bf0[52], cos_bit);
    bf1[44] = half_btf(-cospi[24], bf0[44], -cospi[40], bf0[51], cos_bit);
    bf1[45] = half_btf(-cospi[24], bf0[45], -cospi[40], bf0[50], cos_bit);
    bf1[46] = bf0[46];
    bf1[47] = bf0[47];
    bf1[48] = bf0[48];
    bf1[49] = bf0[49];
    bf1[50] = half_btf(cospi[24], bf0[50], -cospi[40], bf0[45], cos_bit);
    bf1[51] = half_btf(cospi[24], bf0[51], -cospi[40], bf0[44], cos_bit);
    bf1[52] = half_btf(cospi[40], bf0[52], cospi[24], bf0[43], cos_bit);
    bf1[53] = half_btf(cospi[40], bf0[53], cospi[24], bf0[42], cos_bit);
    bf1[54] = bf0[54];
    bf1[55] = bf0[55];
    bf1[56] = bf0[56];
    bf1[57] = bf0[57];
    bf1[58] = half_btf(cospi[56], bf0[58], -cospi[8], bf0[37], cos_bit);
    bf1[59] = half_btf(cospi[56], bf0[59], -cospi[8], bf0[36], cos_bit);
    bf1[60] = half_btf(cospi[8], bf0[60], cospi[56], bf0[35], cos_bit);
    bf1[61] = half_btf(cospi[8], bf0[61], cospi[56], bf0[34], cos_bit);
    bf1[62] = bf0[62];
    bf1[63] = bf0[63];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[56], bf0[4], cospi[8], bf0[7], cos_bit);
    bf1[5]  = half_btf(cospi[24], bf0[5], cospi[40], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[24], bf0[6], -cospi[40], bf0[5], cos_bit);
    bf1[7]  = half_btf(cospi[56], bf0[7], -cospi[8], bf0[4], cos_bit);
    bf1[8]  = bf0[8] + bf0[9];
    bf1[9]  = -bf0[9] + bf0[8];
    bf1[10] = -bf0[10] + bf0[11];
    bf1[11] = bf0[11] + bf0[10];
    bf1[12] = bf0[12] + bf0[13];
    bf1[13] = -bf0[13] + bf0[12];
    bf1[14] = -bf0[14] + bf0[15];
    bf1[15] = bf0[15] + bf0[14];
    bf1[16] = bf0[16];
    bf1[17] = half_btf(-cospi[8], bf0[17], cospi[56], bf0[30], cos_bit);
    bf1[18] = half_btf(-cospi[56], bf0[18], -cospi[8], bf0[29], cos_bit);
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = half_btf(-cospi[40], bf0[21], cospi[24], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[24], bf0[22], -cospi[40], bf0[25], cos_bit);
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = half_btf(cospi[24], bf0[25], -cospi[40], bf0[22], cos_bit);
    bf1[26] = half_btf(cospi[40], bf0[26], cospi[24], bf0[21], cos_bit);
    bf1[27] = bf0[27];
    bf1[28] = bf0[28];
    bf1[29] = half_btf(cospi[56], bf0[29], -cospi[8], bf0[18], cos_bit);
    bf1[30] = half_btf(cospi[8], bf0[30], cospi[56], bf0[17], cos_bit);
    bf1[31] = bf0[31];
    bf1[32] = bf0[32] + bf0[35];
    bf1[33] = bf0[33] + bf0[34];
    bf1[34] = -bf0[34] + bf0[33];
    bf1[35] = -bf0[35] + bf0[32];
    bf1[36] = -bf0[36] + bf0[39];
    bf1[37] = -bf0[37] + bf0[38];
    bf1[38] = bf0[38] + bf0[37];
    bf1[39] = bf0[39] + bf0[36];
    bf1[40] = bf0[40] + bf0[43];
    bf1[41] = bf0[41] + bf0[42];
    bf1[42] = -bf0[42] + bf0[41];
    bf1[43] = -bf0[43] + bf0[40];
    bf1[44] = -bf0[44] + bf0[47];
    bf1[45] = -bf0[45] + bf0[46];
    bf1[46] = bf0[46] + bf0[45];
    bf1[47] = bf0[47] + bf0[44];
    bf1[48] = bf0[48] + bf0[51];
    bf1[49] = bf0[49] + bf0[50];
    bf1[50] = -bf0[50] + bf0[49];
    bf1[51] = -bf0[51] + bf0[48];
    bf1[52] = -bf0[52] + bf0[55];
    bf1[53] = -bf0[53] + bf0[54];
    bf1[54] = bf0[54] + bf0[53];
    bf1[55] = bf0[55] + bf0[52];
    bf1[56] = bf0[56] + bf0[59];
    bf1[57] = bf0[57] + bf0[58];
    bf1[58] = -bf0[58] + bf0[57];
    bf1[59] = -bf0[59] + bf0[56];
    bf1[60] = -bf0[60] + bf0[63];
    bf1[61] = -bf0[61] + bf0[62];
    bf1[62] = bf0[62] + bf0[61];
    bf1[63] = bf0[63] + bf0[60];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[60], bf0[8], cospi[4], bf0[15], cos_bit);
    bf1[9]  = half_btf(cospi[28], bf0[9], cospi[36], bf0[14], cos_bit);
    bf1[10] = half_btf(cospi[44], bf0[10], cospi[20], bf0[13], cos_bit);
    bf1[11] = half_btf(cospi[12], bf0[11], cospi[52], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[12], bf0[12], -cospi[52], bf0[11], cos_bit);
    bf1[13] = half_btf(cospi[44], bf0[13], -cospi[20], bf0[10], cos_bit);
    bf1[14] = half_btf(cospi[28], bf0[14], -cospi[36], bf0[9], cos_bit);
    bf1[15] = half_btf(cospi[60], bf0[15], -cospi[4], bf0[8], cos_bit);
    bf1[16] = bf0[16] + bf0[17];
    bf1[17] = -bf0[17] + bf0[16];
    bf1[18] = -bf0[18] + bf0[19];
    bf1[19] = bf0[19] + bf0[18];
    bf1[20] = bf0[20] + bf0[21];
    bf1[21] = -bf0[21] + bf0[20];
    bf1[22] = -bf0[22] + bf0[23];
    bf1[23] = bf0[23] + bf0[22];
    bf1[24] = bf0[24] + bf0[25];
    bf1[25] = -bf0[25] + bf0[24];
    bf1[26] = -bf0[26] + bf0[27];
    bf1[27] = bf0[27] + bf0[26];
    bf1[28] = bf0[28] + bf0[29];
    bf1[29] = -bf0[29] + bf0[28];
    bf1[30] = -bf0[30] + bf0[31];
    bf1[31] = bf0[31] + bf0[30];
    bf1[32] = bf0[32];
    bf1[33] = half_btf(-cospi[4], bf0[33], cospi[60], bf0[62], cos_bit);
    bf1[34] = half_btf(-cospi[60], bf0[34], -cospi[4], bf0[61], cos_bit);
    bf1[35] = bf0[35];
    bf1[36] = bf0[36];
    bf1[37] = half_btf(-cospi[36], bf0[37], cospi[28], bf0[58], cos_bit);
    bf1[38] = half_btf(-cospi[28], bf0[38], -cospi[36], bf0[57], cos_bit);
    bf1[39] = bf0[39];
    bf1[40] = bf0[40];
    bf1[41] = half_btf(-cospi[20], bf0[41], cospi[44], bf0[54], cos_bit);
    bf1[42] = half_btf(-cospi[44], bf0[42], -cospi[20], bf0[53], cos_bit);
    bf1[43] = bf0[43];
    bf1[44] = bf0[44];
    bf1[45] = half_btf(-cospi[52], bf0[45], cospi[12], bf0[50], cos_bit);
    bf1[46] = half_btf(-cospi[12], bf0[46], -cospi[52], bf0[49], cos_bit);
    bf1[47] = bf0[47];
    bf1[48] = bf0[48];
    bf1[49] = half_btf(cospi[12], bf0[49], -cospi[52], bf0[46], cos_bit);
    bf1[50] = half_btf(cospi[52], bf0[50], cospi[12], bf0[45], cos_bit);
    bf1[51] = bf0[51];
    bf1[52] = bf0[52];
    bf1[53] = half_btf(cospi[44], bf0[53], -cospi[20], bf0[42], cos_bit);
    bf1[54] = half_btf(cospi[20], bf0[54], cospi[44], bf0[41], cos_bit);
    bf1[55] = bf0[55];
    bf1[56] = bf0[56];
    bf1[57] = half_btf(cospi[28], bf0[57], -cospi[36], bf0[38], cos_bit);
    bf1[58] = half_btf(cospi[36], bf0[58], cospi[28], bf0[37], cos_bit);
    bf1[59] = bf0[59];
    bf1[60] = bf0[60];
    bf1[61] = half_btf(cospi[60], bf0[61], -cospi[4], bf0[34], cos_bit);
    bf1[62] = half_btf(cospi[4], bf0[62], cospi[60], bf0[33], cos_bit);
    bf1[63] = bf0[63];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = half_btf(cospi[62], bf0[16], cospi[2], bf0[31], cos_bit);
    bf1[17] = half_btf(cospi[30], bf0[17], cospi[34], bf0[30], cos_bit);
    bf1[18] = half_btf(cospi[46], bf0[18], cospi[18], bf0[29], cos_bit);
    bf1[19] = half_btf(cospi[14], bf0[19], cospi[50], bf0[28], cos_bit);
    bf1[20] = half_btf(cospi[54], bf0[20], cospi[10], bf0[27], cos_bit);
    bf1[21] = half_btf(cospi[22], bf0[21], cospi[42], bf0[26], cos_bit);
    bf1[22] = half_btf(cospi[38], bf0[22], cospi[26], bf0[25], cos_bit);
    bf1[23] = half_btf(cospi[6], bf0[23], cospi[58], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[6], bf0[24], -cospi[58], bf0[23], cos_bit);
    bf1[25] = half_btf(cospi[38], bf0[25], -cospi[26], bf0[22], cos_bit);
    bf1[26] = half_btf(cospi[22], bf0[26], -cospi[42], bf0[21], cos_bit);
    bf1[27] = half_btf(cospi[54], bf0[27], -cospi[10], bf0[20], cos_bit);
    bf1[28] = half_btf(cospi[14], bf0[28], -cospi[50], bf0[19], cos_bit);
    bf1[29] = half_btf(cospi[46], bf0[29], -cospi[18], bf0[18], cos_bit);
    bf1[30] = half_btf(cospi[30], bf0[30], -cospi[34], bf0[17], cos_bit);
    bf1[31] = half_btf(cospi[62], bf0[31], -cospi[2], bf0[16], cos_bit);
    bf1[32] = bf0[32] + bf0[33];
    bf1[33] = -bf0[33] + bf0[32];
    bf1[34] = -bf0[34] + bf0[35];
    bf1[35] = bf0[35] + bf0[34];
    bf1[36] = bf0[36] + bf0[37];
    bf1[37] = -bf0[37] + bf0[36];
    bf1[38] = -bf0[38] + bf0[39];
    bf1[39] = bf0[39] + bf0[38];
    bf1[40] = bf0[40] + bf0[41];
    bf1[41] = -bf0[41] + bf0[40];
    bf1[42] = -bf0[42] + bf0[43];
    bf1[43] = bf0[43] + bf0[42];
    bf1[44] = bf0[44] + bf0[45];
    bf1[45] = -bf0[45] + bf0[44];
    bf1[46] = -bf0[46] + bf0[47];
    bf1[47] = bf0[47] + bf0[46];
    bf1[48] = bf0[48] + bf0[49];
    bf1[49] = -bf0[49] + bf0[48];
    bf1[50] = -bf0[50] + bf0[51];
    bf1[51] = bf0[51] + bf0[50];
    bf1[52] = bf0[52] + bf0[53];
    bf1[53] = -bf0[53] + bf0[52];
    bf1[54] = -bf0[54] + bf0[55];
    bf1[55] = bf0[55] + bf0[54];
    bf1[56] = bf0[56] + bf0[57];
    bf1[57] = -bf0[57] + bf0[56];
    bf1[58] = -bf0[58] + bf0[59];
    bf1[59] = bf0[59] + bf0[58];
    bf1[60] = bf0[60] + bf0[61];
    bf1[61] = -bf0[61] + bf0[60];
    bf1[62] = -bf0[62] + bf0[63];
    bf1[63] = bf0[63] + bf0[62];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 10 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = bf0[21];
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = bf0[26];
    bf1[27] = bf0[27];
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    bf1[32] = half_btf(cospi[63], bf0[32], cospi[1], bf0[63], cos_bit);
    bf1[33] = half_btf(cospi[31], bf0[33], cospi[33], bf0[62], cos_bit);
    bf1[34] = half_btf(cospi[47], bf0[34], cospi[17], bf0[61], cos_bit);
    bf1[35] = half_btf(cospi[15], bf0[35], cospi[49], bf0[60], cos_bit);
    bf1[36] = half_btf(cospi[55], bf0[36], cospi[9], bf0[59], cos_bit);
    bf1[37] = half_btf(cospi[23], bf0[37], cospi[41], bf0[58], cos_bit);
    bf1[38] = half_btf(cospi[39], bf0[38], cospi[25], bf0[57], cos_bit);
    bf1[39] = half_btf(cospi[7], bf0[39], cospi[57], bf0[56], cos_bit);
    bf1[40] = half_btf(cospi[59], bf0[40], cospi[5], bf0[55], cos_bit);
    bf1[41] = half_btf(cospi[27], bf0[41], cospi[37], bf0[54], cos_bit);
    bf1[42] = half_btf(cospi[43], bf0[42], cospi[21], bf0[53], cos_bit);
    bf1[43] = half_btf(cospi[11], bf0[43], cospi[53], bf0[52], cos_bit);
    bf1[44] = half_btf(cospi[51], bf0[44], cospi[13], bf0[51], cos_bit);
    bf1[45] = half_btf(cospi[19], bf0[45], cospi[45], bf0[50], cos_bit);
    bf1[46] = half_btf(cospi[35], bf0[46], cospi[29], bf0[49], cos_bit);
    bf1[47] = half_btf(cospi[3], bf0[47], cospi[61], bf0[48], cos_bit);
    bf1[48] = half_btf(cospi[3], bf0[48], -cospi[61], bf0[47], cos_bit);
    bf1[49] = half_btf(cospi[35], bf0[49], -cospi[29], bf0[46], cos_bit);
    bf1[50] = half_btf(cospi[19], bf0[50], -cospi[45], bf0[45], cos_bit);
    bf1[51] = half_btf(cospi[51], bf0[51], -cospi[13], bf0[44], cos_bit);
    bf1[52] = half_btf(cospi[11], bf0[52], -cospi[53], bf0[43], cos_bit);
    bf1[53] = half_btf(cospi[43], bf0[53], -cospi[21], bf0[42], cos_bit);
    bf1[54] = half_btf(cospi[27], bf0[54], -cospi[37], bf0[41], cos_bit);
    bf1[55] = half_btf(cospi[59], bf0[55], -cospi[5], bf0[40], cos_bit);
    bf1[56] = half_btf(cospi[7], bf0[56], -cospi[57], bf0[39], cos_bit);
    bf1[57] = half_btf(cospi[39], bf0[57], -cospi[25], bf0[38], cos_bit);
    bf1[58] = half_btf(cospi[23], bf0[58], -cospi[41], bf0[37], cos_bit);
    bf1[59] = half_btf(cospi[55], bf0[59], -cospi[9], bf0[36], cos_bit);
    bf1[60] = half_btf(cospi[15], bf0[60], -cospi[49], bf0[35], cos_bit);
    bf1[61] = half_btf(cospi[47], bf0[61], -cospi[17], bf0[34], cos_bit);
    bf1[62] = half_btf(cospi[31], bf0[62], -cospi[33], bf0[33], cos_bit);
    bf1[63] = half_btf(cospi[63], bf0[63], -cospi[1], bf0[32], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 11 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[32];
    bf1[2]  = bf0[16];
    bf1[3]  = bf0[48];
    bf1[4]  = bf0[8];
    bf1[5]  = bf0[40];
    bf1[6]  = bf0[24];
    bf1[7]  = bf0[56];
    bf1[8]  = bf0[4];
    bf1[9]  = bf0[36];
    bf1[10] = bf0[20];
    bf1[11] = bf0[52];
    bf1[12] = bf0[12];
    bf1[13] = bf0[44];
    bf1[14] = bf0[28];
    bf1[15] = bf0[60];
    bf1[16] = bf0[2];
    bf1[17] = bf0[34];
    bf1[18] = bf0[18];
    bf1[19] = bf0[50];
    bf1[20] = bf0[10];
    bf1[21] = bf0[42];
    bf1[22] = bf0[26];
    bf1[23] = bf0[58];
    bf1[24] = bf0[6];
    bf1[25] = bf0[38];
    bf1[26] = bf0[22];
    bf1[27] = bf0[54];
    bf1[28] = bf0[14];
    bf1[29] = bf0[46];
    bf1[30] = bf0[30];
    bf1[31] = bf0[62];
    bf1[32] = bf0[1];
    bf1[33] = bf0[33];
    bf1[34] = bf0[17];
    bf1[35] = bf0[49];
    bf1[36] = bf0[9];
    bf1[37] = bf0[41];
    bf1[38] = bf0[25];
    bf1[39] = bf0[57];
    bf1[40] = bf0[5];
    bf1[41] = bf0[37];
    bf1[42] = bf0[21];
    bf1[43] = bf0[53];
    bf1[44] = bf0[13];
    bf1[45] = bf0[45];
    bf1[46] = bf0[29];
    bf1[47] = bf0[61];
    bf1[48] = bf0[3];
    bf1[49] = bf0[35];
    bf1[50] = bf0[19];
    bf1[51] = bf0[51];
    bf1[52] = bf0[11];
    bf1[53] = bf0[43];
    bf1[54] = bf0[27];
    bf1[55] = bf0[59];
    bf1[56] = bf0[7];
    bf1[57] = bf0[39];
    bf1[58] = bf0[23];
    bf1[59] = bf0[55];
    bf1[60] = bf0[15];
    bf1[61] = bf0[47];
    bf1[62] = bf0[31];
    bf1[63] = bf0[63];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void eb_av1_fadst4_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    int32_t        bit   = cos_bit;
    const int32_t *sinpi = sinpi_arr(bit);
    int32_t        x0, x1, x2, x3;
    int32_t        s0, s1, s2, s3, s4, s5, s6, s7;

    /*!< stage 0 */
    range_check(0, input, input, 4, stage_range[0]);
    x0 = input[0];
    x1 = input[1];
    x2 = input[2];
    x3 = input[3];

    if (!(x0 | x1 | x2 | x3)) {
        output[0] = output[1] = output[2] = output[3] = 0;
        return;
    }

    /*!< stage 1 */
    //s0 = range_check_value(sinpi[1] * x0, bit + stage_range[1]);
    //s1 = range_check_value(sinpi[4] * x0, bit + stage_range[1]);
    //s2 = range_check_value(sinpi[2] * x1, bit + stage_range[1]);
    //s3 = range_check_value(sinpi[1] * x1, bit + stage_range[1]);
    //s4 = range_check_value(sinpi[3] * x2, bit + stage_range[1]);
    //s5 = range_check_value(sinpi[4] * x3, bit + stage_range[1]);
    //s6 = range_check_value(sinpi[2] * x3, bit + stage_range[1]);
    //s7 = range_check_value(x0 + x1, stage_range[1]);

    /*!< stage 2 */
    //s7 = range_check_value(s7 - x3, stage_range[2]);

    /*!< stage 3 */
    //x0 = range_check_value(s0 + s2, bit + stage_range[3]);
    //x1 = range_check_value(sinpi[3] * s7, bit + stage_range[3]);
    //x2 = range_check_value(s1 - s3, bit + stage_range[3]);
    //x3 = range_check_value(s4, bit + stage_range[3]);

    /*!< stage 4 */
    //x0 = range_check_value(x0 + s5, bit + stage_range[4]);
    //x2 = range_check_value(x2 + s6, bit + stage_range[4]);

    /*!< stage 5 */
    //s0 = range_check_value(x0 + x3, bit + stage_range[5]);
    //s1 = range_check_value(x1, bit + stage_range[5]);
    //s2 = range_check_value(x2 - x3, bit + stage_range[5]);
    //s3 = range_check_value(x2 - x0, bit + stage_range[5]);

    /*!< stage 6 */
    //s3 = range_check_value(s3 + x3, bit + stage_range[6]);

    /*!< stage 1 */
    s0 = sinpi[1] * x0;
    s1 = sinpi[4] * x0;
    s2 = sinpi[2] * x1;
    s3 = sinpi[1] * x1;
    s4 = sinpi[3] * x2;
    s5 = sinpi[4] * x3;
    s6 = sinpi[2] * x3;
    s7 = x0 + x1;

    /*!< stage 2 */
    s7 = s7 - x3;

    /*!< stage 3 */
    x0 = s0 + s2;
    x1 = sinpi[3] * s7;
    x2 = s1 - s3;
    x3 = s4;

    /*!< stage 4 */
    x0 = x0 + s5;
    x2 = x2 + s6;

    /*!< stage 5 */
    s0 = x0 + x3;
    s1 = x1;
    s2 = x2 - x3;
    s3 = x2 - x0;

    /*!< stage 6 */
    s3 = s3 + x3;

    /*!< 1-D transform scaling factor is sqrt(2). */
    output[0] = round_shift(s0, bit);
    output[1] = round_shift(s1, bit);
    output[2] = round_shift(s2, bit);
    output[3] = round_shift(s3, bit);
    range_check(6, input, output, 4, stage_range[6]);
}

void eb_av1_fadst8_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    const int32_t  size = 8;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[8];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    assert(output != input);
    bf1    = output;
    bf1[0] = input[0];
    bf1[1] = -input[7];
    bf1[2] = -input[3];
    bf1[3] = input[4];
    bf1[4] = -input[1];
    bf1[5] = input[6];
    bf1[6] = input[2];
    bf1[7] = -input[5];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0];
    bf1[1] = bf0[1];
    bf1[2] = half_btf(cospi[32], bf0[2], cospi[32], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[32], bf0[2], -cospi[32], bf0[3], cos_bit);
    bf1[4] = bf0[4];
    bf1[5] = bf0[5];
    bf1[6] = half_btf(cospi[32], bf0[6], cospi[32], bf0[7], cos_bit);
    bf1[7] = half_btf(cospi[32], bf0[6], -cospi[32], bf0[7], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = bf0[0] + bf0[2];
    bf1[1] = bf0[1] + bf0[3];
    bf1[2] = bf0[0] - bf0[2];
    bf1[3] = bf0[1] - bf0[3];
    bf1[4] = bf0[4] + bf0[6];
    bf1[5] = bf0[5] + bf0[7];
    bf1[6] = bf0[4] - bf0[6];
    bf1[7] = bf0[5] - bf0[7];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0];
    bf1[1] = bf0[1];
    bf1[2] = bf0[2];
    bf1[3] = bf0[3];
    bf1[4] = half_btf(cospi[16], bf0[4], cospi[48], bf0[5], cos_bit);
    bf1[5] = half_btf(cospi[48], bf0[4], -cospi[16], bf0[5], cos_bit);
    bf1[6] = half_btf(-cospi[48], bf0[6], cospi[16], bf0[7], cos_bit);
    bf1[7] = half_btf(cospi[16], bf0[6], cospi[48], bf0[7], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = bf0[0] + bf0[4];
    bf1[1] = bf0[1] + bf0[5];
    bf1[2] = bf0[2] + bf0[6];
    bf1[3] = bf0[3] + bf0[7];
    bf1[4] = bf0[0] - bf0[4];
    bf1[5] = bf0[1] - bf0[5];
    bf1[6] = bf0[2] - bf0[6];
    bf1[7] = bf0[3] - bf0[7];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi  = cospi_arr(cos_bit);
    bf0    = output;
    bf1    = step;
    bf1[0] = half_btf(cospi[4], bf0[0], cospi[60], bf0[1], cos_bit);
    bf1[1] = half_btf(cospi[60], bf0[0], -cospi[4], bf0[1], cos_bit);
    bf1[2] = half_btf(cospi[20], bf0[2], cospi[44], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[44], bf0[2], -cospi[20], bf0[3], cos_bit);
    bf1[4] = half_btf(cospi[36], bf0[4], cospi[28], bf0[5], cos_bit);
    bf1[5] = half_btf(cospi[28], bf0[4], -cospi[36], bf0[5], cos_bit);
    bf1[6] = half_btf(cospi[52], bf0[6], cospi[12], bf0[7], cos_bit);
    bf1[7] = half_btf(cospi[12], bf0[6], -cospi[52], bf0[7], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = bf0[1];
    bf1[1] = bf0[6];
    bf1[2] = bf0[3];
    bf1[3] = bf0[4];
    bf1[4] = bf0[5];
    bf1[5] = bf0[2];
    bf1[6] = bf0[7];
    bf1[7] = bf0[0];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void eb_av1_fadst16_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                        const int8_t *stage_range) {
    const int32_t  size = 16;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[16];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    assert(output != input);
    bf1     = output;
    bf1[0]  = input[0];
    bf1[1]  = -input[15];
    bf1[2]  = -input[7];
    bf1[3]  = input[8];
    bf1[4]  = -input[3];
    bf1[5]  = input[12];
    bf1[6]  = input[4];
    bf1[7]  = -input[11];
    bf1[8]  = -input[1];
    bf1[9]  = input[14];
    bf1[10] = input[6];
    bf1[11] = -input[9];
    bf1[12] = input[2];
    bf1[13] = -input[13];
    bf1[14] = -input[5];
    bf1[15] = input[10];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = half_btf(cospi[32], bf0[2], cospi[32], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[32], bf0[2], -cospi[32], bf0[3], cos_bit);
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[32], bf0[6], -cospi[32], bf0[7], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(cospi[32], bf0[10], cospi[32], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[32], bf0[10], -cospi[32], bf0[11], cos_bit);
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = half_btf(cospi[32], bf0[14], cospi[32], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[32], bf0[14], -cospi[32], bf0[15], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[2];
    bf1[1]  = bf0[1] + bf0[3];
    bf1[2]  = bf0[0] - bf0[2];
    bf1[3]  = bf0[1] - bf0[3];
    bf1[4]  = bf0[4] + bf0[6];
    bf1[5]  = bf0[5] + bf0[7];
    bf1[6]  = bf0[4] - bf0[6];
    bf1[7]  = bf0[5] - bf0[7];
    bf1[8]  = bf0[8] + bf0[10];
    bf1[9]  = bf0[9] + bf0[11];
    bf1[10] = bf0[8] - bf0[10];
    bf1[11] = bf0[9] - bf0[11];
    bf1[12] = bf0[12] + bf0[14];
    bf1[13] = bf0[13] + bf0[15];
    bf1[14] = bf0[12] - bf0[14];
    bf1[15] = bf0[13] - bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[16], bf0[4], cospi[48], bf0[5], cos_bit);
    bf1[5]  = half_btf(cospi[48], bf0[4], -cospi[16], bf0[5], cos_bit);
    bf1[6]  = half_btf(-cospi[48], bf0[6], cospi[16], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[16], bf0[6], cospi[48], bf0[7], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = half_btf(cospi[16], bf0[12], cospi[48], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[48], bf0[12], -cospi[16], bf0[13], cos_bit);
    bf1[14] = half_btf(-cospi[48], bf0[14], cospi[16], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[16], bf0[14], cospi[48], bf0[15], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[4];
    bf1[1]  = bf0[1] + bf0[5];
    bf1[2]  = bf0[2] + bf0[6];
    bf1[3]  = bf0[3] + bf0[7];
    bf1[4]  = bf0[0] - bf0[4];
    bf1[5]  = bf0[1] - bf0[5];
    bf1[6]  = bf0[2] - bf0[6];
    bf1[7]  = bf0[3] - bf0[7];
    bf1[8]  = bf0[8] + bf0[12];
    bf1[9]  = bf0[9] + bf0[13];
    bf1[10] = bf0[10] + bf0[14];
    bf1[11] = bf0[11] + bf0[15];
    bf1[12] = bf0[8] - bf0[12];
    bf1[13] = bf0[9] - bf0[13];
    bf1[14] = bf0[10] - bf0[14];
    bf1[15] = bf0[11] - bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[8], bf0[8], cospi[56], bf0[9], cos_bit);
    bf1[9]  = half_btf(cospi[56], bf0[8], -cospi[8], bf0[9], cos_bit);
    bf1[10] = half_btf(cospi[40], bf0[10], cospi[24], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[24], bf0[10], -cospi[40], bf0[11], cos_bit);
    bf1[12] = half_btf(-cospi[56], bf0[12], cospi[8], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[8], bf0[12], cospi[56], bf0[13], cos_bit);
    bf1[14] = half_btf(-cospi[24], bf0[14], cospi[40], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[40], bf0[14], cospi[24], bf0[15], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[8];
    bf1[1]  = bf0[1] + bf0[9];
    bf1[2]  = bf0[2] + bf0[10];
    bf1[3]  = bf0[3] + bf0[11];
    bf1[4]  = bf0[4] + bf0[12];
    bf1[5]  = bf0[5] + bf0[13];
    bf1[6]  = bf0[6] + bf0[14];
    bf1[7]  = bf0[7] + bf0[15];
    bf1[8]  = bf0[0] - bf0[8];
    bf1[9]  = bf0[1] - bf0[9];
    bf1[10] = bf0[2] - bf0[10];
    bf1[11] = bf0[3] - bf0[11];
    bf1[12] = bf0[4] - bf0[12];
    bf1[13] = bf0[5] - bf0[13];
    bf1[14] = bf0[6] - bf0[14];
    bf1[15] = bf0[7] - bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[2], bf0[0], cospi[62], bf0[1], cos_bit);
    bf1[1]  = half_btf(cospi[62], bf0[0], -cospi[2], bf0[1], cos_bit);
    bf1[2]  = half_btf(cospi[10], bf0[2], cospi[54], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[54], bf0[2], -cospi[10], bf0[3], cos_bit);
    bf1[4]  = half_btf(cospi[18], bf0[4], cospi[46], bf0[5], cos_bit);
    bf1[5]  = half_btf(cospi[46], bf0[4], -cospi[18], bf0[5], cos_bit);
    bf1[6]  = half_btf(cospi[26], bf0[6], cospi[38], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[38], bf0[6], -cospi[26], bf0[7], cos_bit);
    bf1[8]  = half_btf(cospi[34], bf0[8], cospi[30], bf0[9], cos_bit);
    bf1[9]  = half_btf(cospi[30], bf0[8], -cospi[34], bf0[9], cos_bit);
    bf1[10] = half_btf(cospi[42], bf0[10], cospi[22], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[22], bf0[10], -cospi[42], bf0[11], cos_bit);
    bf1[12] = half_btf(cospi[50], bf0[12], cospi[14], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[14], bf0[12], -cospi[50], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[58], bf0[14], cospi[6], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[6], bf0[14], -cospi[58], bf0[15], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[1];
    bf1[1]  = bf0[14];
    bf1[2]  = bf0[3];
    bf1[3]  = bf0[12];
    bf1[4]  = bf0[5];
    bf1[5]  = bf0[10];
    bf1[6]  = bf0[7];
    bf1[7]  = bf0[8];
    bf1[8]  = bf0[9];
    bf1[9]  = bf0[6];
    bf1[10] = bf0[11];
    bf1[11] = bf0[4];
    bf1[12] = bf0[13];
    bf1[13] = bf0[2];
    bf1[14] = bf0[15];
    bf1[15] = bf0[0];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void av1_fadst32_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                     const int8_t *stage_range) {
    const int32_t  size = 32;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[32];

    /*!< stage 0 */
    range_check(stage, input, input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[31];
    bf1[1]  = input[0];
    bf1[2]  = input[29];
    bf1[3]  = input[2];
    bf1[4]  = input[27];
    bf1[5]  = input[4];
    bf1[6]  = input[25];
    bf1[7]  = input[6];
    bf1[8]  = input[23];
    bf1[9]  = input[8];
    bf1[10] = input[21];
    bf1[11] = input[10];
    bf1[12] = input[19];
    bf1[13] = input[12];
    bf1[14] = input[17];
    bf1[15] = input[14];
    bf1[16] = input[15];
    bf1[17] = input[16];
    bf1[18] = input[13];
    bf1[19] = input[18];
    bf1[20] = input[11];
    bf1[21] = input[20];
    bf1[22] = input[9];
    bf1[23] = input[22];
    bf1[24] = input[7];
    bf1[25] = input[24];
    bf1[26] = input[5];
    bf1[27] = input[26];
    bf1[28] = input[3];
    bf1[29] = input[28];
    bf1[30] = input[1];
    bf1[31] = input[30];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[1], bf0[0], cospi[63], bf0[1], cos_bit);
    bf1[1]  = half_btf(-cospi[1], bf0[1], cospi[63], bf0[0], cos_bit);
    bf1[2]  = half_btf(cospi[5], bf0[2], cospi[59], bf0[3], cos_bit);
    bf1[3]  = half_btf(-cospi[5], bf0[3], cospi[59], bf0[2], cos_bit);
    bf1[4]  = half_btf(cospi[9], bf0[4], cospi[55], bf0[5], cos_bit);
    bf1[5]  = half_btf(-cospi[9], bf0[5], cospi[55], bf0[4], cos_bit);
    bf1[6]  = half_btf(cospi[13], bf0[6], cospi[51], bf0[7], cos_bit);
    bf1[7]  = half_btf(-cospi[13], bf0[7], cospi[51], bf0[6], cos_bit);
    bf1[8]  = half_btf(cospi[17], bf0[8], cospi[47], bf0[9], cos_bit);
    bf1[9]  = half_btf(-cospi[17], bf0[9], cospi[47], bf0[8], cos_bit);
    bf1[10] = half_btf(cospi[21], bf0[10], cospi[43], bf0[11], cos_bit);
    bf1[11] = half_btf(-cospi[21], bf0[11], cospi[43], bf0[10], cos_bit);
    bf1[12] = half_btf(cospi[25], bf0[12], cospi[39], bf0[13], cos_bit);
    bf1[13] = half_btf(-cospi[25], bf0[13], cospi[39], bf0[12], cos_bit);
    bf1[14] = half_btf(cospi[29], bf0[14], cospi[35], bf0[15], cos_bit);
    bf1[15] = half_btf(-cospi[29], bf0[15], cospi[35], bf0[14], cos_bit);
    bf1[16] = half_btf(cospi[33], bf0[16], cospi[31], bf0[17], cos_bit);
    bf1[17] = half_btf(-cospi[33], bf0[17], cospi[31], bf0[16], cos_bit);
    bf1[18] = half_btf(cospi[37], bf0[18], cospi[27], bf0[19], cos_bit);
    bf1[19] = half_btf(-cospi[37], bf0[19], cospi[27], bf0[18], cos_bit);
    bf1[20] = half_btf(cospi[41], bf0[20], cospi[23], bf0[21], cos_bit);
    bf1[21] = half_btf(-cospi[41], bf0[21], cospi[23], bf0[20], cos_bit);
    bf1[22] = half_btf(cospi[45], bf0[22], cospi[19], bf0[23], cos_bit);
    bf1[23] = half_btf(-cospi[45], bf0[23], cospi[19], bf0[22], cos_bit);
    bf1[24] = half_btf(cospi[49], bf0[24], cospi[15], bf0[25], cos_bit);
    bf1[25] = half_btf(-cospi[49], bf0[25], cospi[15], bf0[24], cos_bit);
    bf1[26] = half_btf(cospi[53], bf0[26], cospi[11], bf0[27], cos_bit);
    bf1[27] = half_btf(-cospi[53], bf0[27], cospi[11], bf0[26], cos_bit);
    bf1[28] = half_btf(cospi[57], bf0[28], cospi[7], bf0[29], cos_bit);
    bf1[29] = half_btf(-cospi[57], bf0[29], cospi[7], bf0[28], cos_bit);
    bf1[30] = half_btf(cospi[61], bf0[30], cospi[3], bf0[31], cos_bit);
    bf1[31] = half_btf(-cospi[61], bf0[31], cospi[3], bf0[30], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[16];
    bf1[1]  = bf0[1] + bf0[17];
    bf1[2]  = bf0[2] + bf0[18];
    bf1[3]  = bf0[3] + bf0[19];
    bf1[4]  = bf0[4] + bf0[20];
    bf1[5]  = bf0[5] + bf0[21];
    bf1[6]  = bf0[6] + bf0[22];
    bf1[7]  = bf0[7] + bf0[23];
    bf1[8]  = bf0[8] + bf0[24];
    bf1[9]  = bf0[9] + bf0[25];
    bf1[10] = bf0[10] + bf0[26];
    bf1[11] = bf0[11] + bf0[27];
    bf1[12] = bf0[12] + bf0[28];
    bf1[13] = bf0[13] + bf0[29];
    bf1[14] = bf0[14] + bf0[30];
    bf1[15] = bf0[15] + bf0[31];
    bf1[16] = -bf0[16] + bf0[0];
    bf1[17] = -bf0[17] + bf0[1];
    bf1[18] = -bf0[18] + bf0[2];
    bf1[19] = -bf0[19] + bf0[3];
    bf1[20] = -bf0[20] + bf0[4];
    bf1[21] = -bf0[21] + bf0[5];
    bf1[22] = -bf0[22] + bf0[6];
    bf1[23] = -bf0[23] + bf0[7];
    bf1[24] = -bf0[24] + bf0[8];
    bf1[25] = -bf0[25] + bf0[9];
    bf1[26] = -bf0[26] + bf0[10];
    bf1[27] = -bf0[27] + bf0[11];
    bf1[28] = -bf0[28] + bf0[12];
    bf1[29] = -bf0[29] + bf0[13];
    bf1[30] = -bf0[30] + bf0[14];
    bf1[31] = -bf0[31] + bf0[15];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = half_btf(cospi[4], bf0[16], cospi[60], bf0[17], cos_bit);
    bf1[17] = half_btf(-cospi[4], bf0[17], cospi[60], bf0[16], cos_bit);
    bf1[18] = half_btf(cospi[20], bf0[18], cospi[44], bf0[19], cos_bit);
    bf1[19] = half_btf(-cospi[20], bf0[19], cospi[44], bf0[18], cos_bit);
    bf1[20] = half_btf(cospi[36], bf0[20], cospi[28], bf0[21], cos_bit);
    bf1[21] = half_btf(-cospi[36], bf0[21], cospi[28], bf0[20], cos_bit);
    bf1[22] = half_btf(cospi[52], bf0[22], cospi[12], bf0[23], cos_bit);
    bf1[23] = half_btf(-cospi[52], bf0[23], cospi[12], bf0[22], cos_bit);
    bf1[24] = half_btf(-cospi[60], bf0[24], cospi[4], bf0[25], cos_bit);
    bf1[25] = half_btf(cospi[60], bf0[25], cospi[4], bf0[24], cos_bit);
    bf1[26] = half_btf(-cospi[44], bf0[26], cospi[20], bf0[27], cos_bit);
    bf1[27] = half_btf(cospi[44], bf0[27], cospi[20], bf0[26], cos_bit);
    bf1[28] = half_btf(-cospi[28], bf0[28], cospi[36], bf0[29], cos_bit);
    bf1[29] = half_btf(cospi[28], bf0[29], cospi[36], bf0[28], cos_bit);
    bf1[30] = half_btf(-cospi[12], bf0[30], cospi[52], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[12], bf0[31], cospi[52], bf0[30], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[8];
    bf1[1]  = bf0[1] + bf0[9];
    bf1[2]  = bf0[2] + bf0[10];
    bf1[3]  = bf0[3] + bf0[11];
    bf1[4]  = bf0[4] + bf0[12];
    bf1[5]  = bf0[5] + bf0[13];
    bf1[6]  = bf0[6] + bf0[14];
    bf1[7]  = bf0[7] + bf0[15];
    bf1[8]  = -bf0[8] + bf0[0];
    bf1[9]  = -bf0[9] + bf0[1];
    bf1[10] = -bf0[10] + bf0[2];
    bf1[11] = -bf0[11] + bf0[3];
    bf1[12] = -bf0[12] + bf0[4];
    bf1[13] = -bf0[13] + bf0[5];
    bf1[14] = -bf0[14] + bf0[6];
    bf1[15] = -bf0[15] + bf0[7];
    bf1[16] = bf0[16] + bf0[24];
    bf1[17] = bf0[17] + bf0[25];
    bf1[18] = bf0[18] + bf0[26];
    bf1[19] = bf0[19] + bf0[27];
    bf1[20] = bf0[20] + bf0[28];
    bf1[21] = bf0[21] + bf0[29];
    bf1[22] = bf0[22] + bf0[30];
    bf1[23] = bf0[23] + bf0[31];
    bf1[24] = -bf0[24] + bf0[16];
    bf1[25] = -bf0[25] + bf0[17];
    bf1[26] = -bf0[26] + bf0[18];
    bf1[27] = -bf0[27] + bf0[19];
    bf1[28] = -bf0[28] + bf0[20];
    bf1[29] = -bf0[29] + bf0[21];
    bf1[30] = -bf0[30] + bf0[22];
    bf1[31] = -bf0[31] + bf0[23];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[8], bf0[8], cospi[56], bf0[9], cos_bit);
    bf1[9]  = half_btf(-cospi[8], bf0[9], cospi[56], bf0[8], cos_bit);
    bf1[10] = half_btf(cospi[40], bf0[10], cospi[24], bf0[11], cos_bit);
    bf1[11] = half_btf(-cospi[40], bf0[11], cospi[24], bf0[10], cos_bit);
    bf1[12] = half_btf(-cospi[56], bf0[12], cospi[8], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[56], bf0[13], cospi[8], bf0[12], cos_bit);
    bf1[14] = half_btf(-cospi[24], bf0[14], cospi[40], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[24], bf0[15], cospi[40], bf0[14], cos_bit);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = bf0[21];
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = half_btf(cospi[8], bf0[24], cospi[56], bf0[25], cos_bit);
    bf1[25] = half_btf(-cospi[8], bf0[25], cospi[56], bf0[24], cos_bit);
    bf1[26] = half_btf(cospi[40], bf0[26], cospi[24], bf0[27], cos_bit);
    bf1[27] = half_btf(-cospi[40], bf0[27], cospi[24], bf0[26], cos_bit);
    bf1[28] = half_btf(-cospi[56], bf0[28], cospi[8], bf0[29], cos_bit);
    bf1[29] = half_btf(cospi[56], bf0[29], cospi[8], bf0[28], cos_bit);
    bf1[30] = half_btf(-cospi[24], bf0[30], cospi[40], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[24], bf0[31], cospi[40], bf0[30], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[4];
    bf1[1]  = bf0[1] + bf0[5];
    bf1[2]  = bf0[2] + bf0[6];
    bf1[3]  = bf0[3] + bf0[7];
    bf1[4]  = -bf0[4] + bf0[0];
    bf1[5]  = -bf0[5] + bf0[1];
    bf1[6]  = -bf0[6] + bf0[2];
    bf1[7]  = -bf0[7] + bf0[3];
    bf1[8]  = bf0[8] + bf0[12];
    bf1[9]  = bf0[9] + bf0[13];
    bf1[10] = bf0[10] + bf0[14];
    bf1[11] = bf0[11] + bf0[15];
    bf1[12] = -bf0[12] + bf0[8];
    bf1[13] = -bf0[13] + bf0[9];
    bf1[14] = -bf0[14] + bf0[10];
    bf1[15] = -bf0[15] + bf0[11];
    bf1[16] = bf0[16] + bf0[20];
    bf1[17] = bf0[17] + bf0[21];
    bf1[18] = bf0[18] + bf0[22];
    bf1[19] = bf0[19] + bf0[23];
    bf1[20] = -bf0[20] + bf0[16];
    bf1[21] = -bf0[21] + bf0[17];
    bf1[22] = -bf0[22] + bf0[18];
    bf1[23] = -bf0[23] + bf0[19];
    bf1[24] = bf0[24] + bf0[28];
    bf1[25] = bf0[25] + bf0[29];
    bf1[26] = bf0[26] + bf0[30];
    bf1[27] = bf0[27] + bf0[31];
    bf1[28] = -bf0[28] + bf0[24];
    bf1[29] = -bf0[29] + bf0[25];
    bf1[30] = -bf0[30] + bf0[26];
    bf1[31] = -bf0[31] + bf0[27];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[16], bf0[4], cospi[48], bf0[5], cos_bit);
    bf1[5]  = half_btf(-cospi[16], bf0[5], cospi[48], bf0[4], cos_bit);
    bf1[6]  = half_btf(-cospi[48], bf0[6], cospi[16], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[48], bf0[7], cospi[16], bf0[6], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = half_btf(cospi[16], bf0[12], cospi[48], bf0[13], cos_bit);
    bf1[13] = half_btf(-cospi[16], bf0[13], cospi[48], bf0[12], cos_bit);
    bf1[14] = half_btf(-cospi[48], bf0[14], cospi[16], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[48], bf0[15], cospi[16], bf0[14], cos_bit);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = half_btf(cospi[16], bf0[20], cospi[48], bf0[21], cos_bit);
    bf1[21] = half_btf(-cospi[16], bf0[21], cospi[48], bf0[20], cos_bit);
    bf1[22] = half_btf(-cospi[48], bf0[22], cospi[16], bf0[23], cos_bit);
    bf1[23] = half_btf(cospi[48], bf0[23], cospi[16], bf0[22], cos_bit);
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = bf0[26];
    bf1[27] = bf0[27];
    bf1[28] = half_btf(cospi[16], bf0[28], cospi[48], bf0[29], cos_bit);
    bf1[29] = half_btf(-cospi[16], bf0[29], cospi[48], bf0[28], cos_bit);
    bf1[30] = half_btf(-cospi[48], bf0[30], cospi[16], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[48], bf0[31], cospi[16], bf0[30], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[2];
    bf1[1]  = bf0[1] + bf0[3];
    bf1[2]  = -bf0[2] + bf0[0];
    bf1[3]  = -bf0[3] + bf0[1];
    bf1[4]  = bf0[4] + bf0[6];
    bf1[5]  = bf0[5] + bf0[7];
    bf1[6]  = -bf0[6] + bf0[4];
    bf1[7]  = -bf0[7] + bf0[5];
    bf1[8]  = bf0[8] + bf0[10];
    bf1[9]  = bf0[9] + bf0[11];
    bf1[10] = -bf0[10] + bf0[8];
    bf1[11] = -bf0[11] + bf0[9];
    bf1[12] = bf0[12] + bf0[14];
    bf1[13] = bf0[13] + bf0[15];
    bf1[14] = -bf0[14] + bf0[12];
    bf1[15] = -bf0[15] + bf0[13];
    bf1[16] = bf0[16] + bf0[18];
    bf1[17] = bf0[17] + bf0[19];
    bf1[18] = -bf0[18] + bf0[16];
    bf1[19] = -bf0[19] + bf0[17];
    bf1[20] = bf0[20] + bf0[22];
    bf1[21] = bf0[21] + bf0[23];
    bf1[22] = -bf0[22] + bf0[20];
    bf1[23] = -bf0[23] + bf0[21];
    bf1[24] = bf0[24] + bf0[26];
    bf1[25] = bf0[25] + bf0[27];
    bf1[26] = -bf0[26] + bf0[24];
    bf1[27] = -bf0[27] + bf0[25];
    bf1[28] = bf0[28] + bf0[30];
    bf1[29] = bf0[29] + bf0[31];
    bf1[30] = -bf0[30] + bf0[28];
    bf1[31] = -bf0[31] + bf0[29];
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 10 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = half_btf(cospi[32], bf0[2], cospi[32], bf0[3], cos_bit);
    bf1[3]  = half_btf(-cospi[32], bf0[3], cospi[32], bf0[2], cos_bit);
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[7], cos_bit);
    bf1[7]  = half_btf(-cospi[32], bf0[7], cospi[32], bf0[6], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(cospi[32], bf0[10], cospi[32], bf0[11], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[10], cos_bit);
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = half_btf(cospi[32], bf0[14], cospi[32], bf0[15], cos_bit);
    bf1[15] = half_btf(-cospi[32], bf0[15], cospi[32], bf0[14], cos_bit);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = half_btf(cospi[32], bf0[18], cospi[32], bf0[19], cos_bit);
    bf1[19] = half_btf(-cospi[32], bf0[19], cospi[32], bf0[18], cos_bit);
    bf1[20] = bf0[20];
    bf1[21] = bf0[21];
    bf1[22] = half_btf(cospi[32], bf0[22], cospi[32], bf0[23], cos_bit);
    bf1[23] = half_btf(-cospi[32], bf0[23], cospi[32], bf0[22], cos_bit);
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = half_btf(cospi[32], bf0[26], cospi[32], bf0[27], cos_bit);
    bf1[27] = half_btf(-cospi[32], bf0[27], cospi[32], bf0[26], cos_bit);
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = half_btf(cospi[32], bf0[30], cospi[32], bf0[31], cos_bit);
    bf1[31] = half_btf(-cospi[32], bf0[31], cospi[32], bf0[30], cos_bit);
    range_check(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 11 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = -bf0[16];
    bf1[2]  = bf0[24];
    bf1[3]  = -bf0[8];
    bf1[4]  = bf0[12];
    bf1[5]  = -bf0[28];
    bf1[6]  = bf0[20];
    bf1[7]  = -bf0[4];
    bf1[8]  = bf0[6];
    bf1[9]  = -bf0[22];
    bf1[10] = bf0[30];
    bf1[11] = -bf0[14];
    bf1[12] = bf0[10];
    bf1[13] = -bf0[26];
    bf1[14] = bf0[18];
    bf1[15] = -bf0[2];
    bf1[16] = bf0[3];
    bf1[17] = -bf0[19];
    bf1[18] = bf0[27];
    bf1[19] = -bf0[11];
    bf1[20] = bf0[15];
    bf1[21] = -bf0[31];
    bf1[22] = bf0[23];
    bf1[23] = -bf0[7];
    bf1[24] = bf0[5];
    bf1[25] = -bf0[21];
    bf1[26] = bf0[29];
    bf1[27] = -bf0[13];
    bf1[28] = bf0[9];
    bf1[29] = -bf0[25];
    bf1[30] = bf0[17];
    bf1[31] = -bf0[1];
    range_check(stage, input, bf1, size, stage_range[stage]);
}

void eb_av1_fidentity4_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                         const int8_t *stage_range) {
    (void)cos_bit;
    for (int32_t i = 0; i < 4; ++i)
        output[i] = round_shift((int64_t)input[i] * new_sqrt2, new_sqrt2_bits);
    assert(stage_range[0] + new_sqrt2_bits <= 32);
    range_check(0, input, output, 4, stage_range[0]);
}

void eb_av1_fidentity8_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                         const int8_t *stage_range) {
    (void)cos_bit;
    for (int32_t i = 0; i < 8; ++i) output[i] = input[i] * 2;
    range_check(0, input, output, 8, stage_range[0]);
}

void eb_av1_fidentity16_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                          const int8_t *stage_range) {
    (void)cos_bit;
    for (int32_t i = 0; i < 16; ++i)
        output[i] = round_shift((int64_t)input[i] * 2 * new_sqrt2, new_sqrt2_bits);
    assert(stage_range[0] + new_sqrt2_bits <= 32);
    range_check(0, input, output, 16, stage_range[0]);
}

void eb_av1_fidentity32_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                          const int8_t *stage_range) {
    (void)cos_bit;
    for (int32_t i = 0; i < 32; ++i) output[i] = input[i] * 4;
    range_check(0, input, output, 32, stage_range[0]);
}

void av1_fidentity64_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    (void)cos_bit;
    for (int32_t i = 0; i < 64; ++i)
        output[i] = round_shift((int64_t)input[i] * 4 * new_sqrt2, new_sqrt2_bits);
    assert(stage_range[0] + new_sqrt2_bits <= 32);
    range_check(0, input, output, 64, stage_range[0]);
}

static INLINE TxfmFunc fwd_txfm_type_to_func(TxfmType TxfmType) {
    switch (TxfmType) {
    case TXFM_TYPE_DCT4: return eb_av1_fdct4_new;
    case TXFM_TYPE_DCT8: return eb_av1_fdct8_new;
    case TXFM_TYPE_DCT16: return eb_av1_fdct16_new;
    case TXFM_TYPE_DCT32: return eb_av1_fdct32_new;
    case TXFM_TYPE_DCT64: return eb_av1_fdct64_new;
    case TXFM_TYPE_ADST4: return eb_av1_fadst4_new;
    case TXFM_TYPE_ADST8: return eb_av1_fadst8_new;
    case TXFM_TYPE_ADST16: return eb_av1_fadst16_new;
    case TXFM_TYPE_ADST32: return av1_fadst32_new;
    case TXFM_TYPE_IDENTITY4: return eb_av1_fidentity4_c;
    case TXFM_TYPE_IDENTITY8: return eb_av1_fidentity8_c;
    case TXFM_TYPE_IDENTITY16: return eb_av1_fidentity16_c;
    case TXFM_TYPE_IDENTITY32: return eb_av1_fidentity32_c;
    case TXFM_TYPE_IDENTITY64: return av1_fidentity64_c;
    default: assert(0); return NULL;
    }
}

void eb_av1_round_shift_array_c(int32_t *arr, int32_t size, int32_t bit) {
    int32_t i;
    if (bit == 0)
        return;
    else {
        if (bit > 0) {
            for (i = 0; i < size; i++) arr[i] = round_shift(arr[i], bit);
        } else {
            for (i = 0; i < size; i++) arr[i] = arr[i] * (1 << (-bit));
        }
    }
}
/*fwd_txfm2d_c*/
static INLINE void av1_tranform_two_d_core_c(int16_t *input, uint32_t input_stride, int32_t *output,
                                             const Txfm2dFlipCfg *cfg, int32_t *buf,
                                             uint8_t bit_depth) {
    int32_t c, r;
    /*!< Note when assigning txfm_size_col, we use the txfm_size from the
     *   row configuration and vice versa. This is intentionally done to
     *   accurately perform rectangular transforms. When the transform is
     *   rectangular, the number of columns will be the same as the
     *   txfm_size stored in the row cfg struct. It will make no difference
     *   for square transforms. */
    const int32_t txfm_size_col = tx_size_wide[cfg->tx_size];
    const int32_t txfm_size_row = tx_size_high[cfg->tx_size];
    /*!< Take the shift from the larger dimension in the rectangular case. */
    const int8_t *shift     = cfg->shift;
    const int32_t rect_type = get_rect_tx_log_ratio(txfm_size_col, txfm_size_row);
    int8_t        stage_range_col[MAX_TXFM_STAGE_NUM];
    int8_t        stage_range_row[MAX_TXFM_STAGE_NUM];
    assert(cfg->stage_num_col <= MAX_TXFM_STAGE_NUM);
    assert(cfg->stage_num_row <= MAX_TXFM_STAGE_NUM);
    eb_av1_gen_fwd_stage_range(stage_range_col, stage_range_row, cfg, bit_depth);

    const int8_t   cos_bit_col   = cfg->cos_bit_col;
    const int8_t   cos_bit_row   = cfg->cos_bit_row;
    const TxfmFunc txfm_func_col = fwd_txfm_type_to_func(cfg->txfm_type_col);
    const TxfmFunc txfm_func_row = fwd_txfm_type_to_func(cfg->txfm_type_row);
    ASSERT(txfm_func_col != NULL);
    ASSERT(txfm_func_row != NULL);
    /*!< use output buffer as temp buffer */
    int32_t *temp_in  = output;
    int32_t *temp_out = output + txfm_size_row;

    /*!< Columns */
    for (c = 0; c < txfm_size_col; ++c) {
        if (cfg->ud_flip == 0)
            for (r = 0; r < txfm_size_row; ++r) temp_in[r] = input[r * input_stride + c];
        else {
            for (r = 0; r < txfm_size_row; ++r)
                /*!< flip upside down */
                temp_in[r] = input[(txfm_size_row - r - 1) * input_stride + c];
        }
        eb_av1_round_shift_array_c(
            temp_in, txfm_size_row, -shift[0]); /*!< NM eb_av1_round_shift_array_c */
        txfm_func_col(temp_in, temp_out, cos_bit_col, stage_range_col);
        eb_av1_round_shift_array_c(
            temp_out, txfm_size_row, -shift[1]); /*!< NM eb_av1_round_shift_array_c */
        if (cfg->lr_flip == 0) {
            for (r = 0; r < txfm_size_row; ++r) buf[r * txfm_size_col + c] = temp_out[r];
        } else {
            for (r = 0; r < txfm_size_row; ++r)
                /*!< flip from left to right */
                buf[r * txfm_size_col + (txfm_size_col - c - 1)] = temp_out[r];
        }
    }

    /*!< Rows */
    for (r = 0; r < txfm_size_row; ++r) {
        txfm_func_row(
            buf + r * txfm_size_col, output + r * txfm_size_col, cos_bit_row, stage_range_row);
        eb_av1_round_shift_array_c(output + r * txfm_size_col, txfm_size_col, -shift[2]);

        if (abs(rect_type) == 1) {
            /*!< Multiply everything by Sqrt2 if the transform is rectangular and the
             *   size difference is a factor of 2. */
            for (c = 0; c < txfm_size_col; ++c) {
                output[r * txfm_size_col + c] =
                    round_shift((int64_t)output[r * txfm_size_col + c] * new_sqrt2, new_sqrt2_bits);
            }
        }
    }
}

static INLINE void set_flip_cfg(TxType tx_type, Txfm2dFlipCfg *cfg) {
    get_flip_cfg(tx_type, &cfg->ud_flip, &cfg->lr_flip);
}
static INLINE void set_fwd_txfm_non_scale_range(Txfm2dFlipCfg *cfg) {
    const int32_t txh_idx = get_txh_idx(cfg->tx_size);
    av1_zero(cfg->stage_range_col);
    av1_zero(cfg->stage_range_row);
    assert(cfg->txfm_type_col < TXFM_TYPES);
    if (cfg->txfm_type_col != TXFM_TYPE_INVALID) {
        int32_t       stage_num_col   = cfg->stage_num_col;
        const int8_t *range_mult2_col = fwd_txfm_range_mult2_list[cfg->txfm_type_col];
        for (int32_t i = 0; i < stage_num_col; ++i)
            cfg->stage_range_col[i] = (range_mult2_col[i] + 1) >> 1;
    }

    if (cfg->txfm_type_row != TXFM_TYPE_INVALID) {
        int32_t stage_num_row = cfg->stage_num_row;
        assert(cfg->txfm_type_row < TXFM_TYPES);
        const int8_t *range_mult2_row = fwd_txfm_range_mult2_list[cfg->txfm_type_row];
        for (int32_t i = 0; i < stage_num_row; ++i)
            cfg->stage_range_row[i] =
                (max_fwd_range_mult2_col[txh_idx] + range_mult2_row[i] + 1) >> 1;
    }
}

void av1_transform_config(TxType tx_type, TxSize tx_size, Txfm2dFlipCfg *cfg) {
    assert(cfg != NULL);
    cfg->tx_size = tx_size;
    set_flip_cfg(tx_type, cfg);
    const TxType1D tx_type_1d_col = vtx_tab[tx_type];
    const TxType1D tx_type_1d_row = htx_tab[tx_type];
    const int32_t  txw_idx        = tx_size_wide_log2[tx_size] - tx_size_wide_log2[0];
    const int32_t  txh_idx        = tx_size_high_log2[tx_size] - tx_size_high_log2[0];
    cfg->shift                    = fwd_txfm_shift_ls[tx_size];
    cfg->cos_bit_col              = fwd_cos_bit_col[txw_idx][txh_idx];
    cfg->cos_bit_row              = fwd_cos_bit_row[txw_idx][txh_idx];
    cfg->txfm_type_col            = av1_txfm_type_ls[txh_idx][tx_type_1d_col];
    cfg->txfm_type_row            = av1_txfm_type_ls[txw_idx][tx_type_1d_row];
    cfg->stage_num_col            = av1_txfm_stage_num_list[cfg->txfm_type_col];
    cfg->stage_num_row            = av1_txfm_stage_num_list[cfg->txfm_type_row];
    set_fwd_txfm_non_scale_range(cfg);
}

static uint64_t energy_computation(int32_t *coeff, uint32_t coeff_stride, uint32_t area_width,
                                   uint32_t area_height) {
    uint32_t column_index;
    uint32_t row_index             = 0;
    uint64_t prediction_distortion = 0;

    while (row_index < area_height) {
        column_index = 0;
        while (column_index < area_width) {
            prediction_distortion += (int64_t)SQR((int64_t)(coeff[column_index]));
            ++column_index;
        }

        coeff += coeff_stride;
        ++row_index;
    }

    return prediction_distortion;
}

uint64_t handle_transform64x64_c(int32_t *output) {
    uint64_t three_quad_energy;

    /*!< top - right 32x32 area. */
    three_quad_energy = energy_computation(output + 32, 64, 32, 32);
    /*!< bottom 64x32 area. */
    three_quad_energy += energy_computation(output + 32 * 64, 64, 64, 32);

    /*!< zero out top-right 32x32 area. */
    for (int32_t row = 0; row < 32; ++row) memset(output + row * 64 + 32, 0, 32 * sizeof(*output));

    /*!< zero out the bottom 64x32 area. */
    memset(output + 32 * 64, 0, 32 * 64 * sizeof(*output));

    /*!< Re-pack non-zero coeffs in the first 32x32 indices. */
    for (int32_t row = 1; row < 32; ++row)
        memcpy(output + row * 32, output + row * 64, 32 * sizeof(*output));

    return three_quad_energy;
}

void av1_transform_two_d_64x64_c(int16_t *input, int32_t *output, uint32_t input_stride,
                                 TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[64 * 64];
    Txfm2dFlipCfg cfg;
    // av1_get_fwd_txfm_cfg
    av1_transform_config(transform_type, TX_64X64, &cfg);
    // fwd_txfm2d_c
    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void av1_transform_two_d_32x32_c(int16_t *input, int32_t *output, uint32_t input_stride,
                                 TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[32 * 32];
    Txfm2dFlipCfg cfg;

    av1_transform_config(transform_type, TX_32X32, &cfg);

    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}
void av1_transform_two_d_16x16_c(int16_t *input, int32_t *output, uint32_t input_stride,
                                 TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[16 * 16];
    Txfm2dFlipCfg cfg;

    av1_transform_config(transform_type, TX_16X16, &cfg);

    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void av1_transform_two_d_8x8_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[8 * 8];
    Txfm2dFlipCfg cfg;

    av1_transform_config(transform_type, TX_8X8, &cfg);

    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void av1_transform_two_d_4x4_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[4 * 4];
    Txfm2dFlipCfg cfg;

    av1_transform_config(transform_type, TX_4X4, &cfg);

    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

/*********************************************************************/
/*!< Calculate CBF */
/*********************************************************************/
void eb_av1_fwd_txfm2d_64x32_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[64 * 32];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/
    av1_transform_config(transform_type, TX_64X32, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

uint64_t handle_transform64x32_c(int32_t *output) {
    /*!< top - right 32x32 area. */
    const uint64_t three_quad_energy = energy_computation(output + 32, 64, 32, 32);

    /*!< zero out right 32x32 area. */
    for (int32_t row = 0; row < 32; ++row) memset(output + row * 64 + 32, 0, 32 * sizeof(*output));

    /*!< Re-pack non-zero coeffs in the first 32x32 indices. */
    for (int32_t row = 1; row < 32; ++row)
        memcpy(output + row * 32, output + row * 64, 32 * sizeof(*output));

    return three_quad_energy;
}

void eb_av1_fwd_txfm2d_32x64_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t intermediate_transform_buffer[32 * 64];

    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/
    av1_transform_config(transform_type, TX_32X64, &cfg);
    /*fwd_txfm2d_c*/
    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

uint64_t handle_transform32x64_c(int32_t *output) {
    /*!< bottom 32x32 area. */
    const uint64_t three_quad_energy = energy_computation(output + 32 * 32, 32, 32, 32);

    /*!< zero out the bottom 32x32 area. */
    memset(output + 32 * 32, 0, 32 * 32 * sizeof(*output));

    return three_quad_energy;
}

void eb_av1_fwd_txfm2d_64x16_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[64 * 16];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/
    av1_transform_config(transform_type, TX_64X16, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

uint64_t handle_transform64x16_c(int32_t *output) {
    /*!< top - right 32x16 area. */
    const uint64_t three_quad_energy = energy_computation(output + 32, 64, 32, 16);

    /*!< zero out right 32x16 area. */
    for (int32_t row = 0; row < 16; ++row) memset(output + row * 64 + 32, 0, 32 * sizeof(*output));

    /*!< Re-pack non-zero coeffs in the first 32x16 indices. */
    for (int32_t row = 1; row < 16; ++row)
        memcpy(output + row * 32, output + row * 64, 32 * sizeof(*output));

    return three_quad_energy;
}

void eb_av1_fwd_txfm2d_16x64_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t intermediate_transform_buffer[16 * 64];

    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/
    av1_transform_config(transform_type, TX_16X64, &cfg);
    /*fwd_txfm2d_c*/
    av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

uint64_t handle_transform16x64_c(int32_t *output) {
    /*!< bottom 16x32 area. */
    const uint64_t three_quad_energy = energy_computation(output + 16 * 32, 16, 16, 32);

    /*!< zero out the bottom 16x32 area. */
    memset(output + 16 * 32, 0, 16 * 32 * sizeof(*output));

    return three_quad_energy;
}

void eb_av1_fwd_txfm2d_32x16_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[32 * 16];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_32X16, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_16x32_c(int16_t *input, int32_t *output, uint32_t input_stride,
                               TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[16 * 32];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_16X32, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_16x8_c(int16_t *input, int32_t *output, uint32_t input_stride,
                              TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[16 * 8];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_16X8, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_8x16_c(int16_t *input, int32_t *output, uint32_t input_stride,
                              TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[8 * 16];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_8X16, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_32x8_c(int16_t *input, int32_t *output, uint32_t input_stride,
                              TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[32 * 8];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_32X8, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_8x32_c(int16_t *input, int32_t *output, uint32_t input_stride,
                              TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[8 * 32];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_8X32, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_16x4_c(int16_t *input, int32_t *output, uint32_t input_stride,
                              TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[16 * 4];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_16X4, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_4x16_c(int16_t *input, int32_t *output, uint32_t input_stride,
                              TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[4 * 16];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_4X16, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_8x4_c(int16_t *input, int32_t *output, uint32_t input_stride,
                             TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[8 * 4];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_8X4, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

void eb_av1_fwd_txfm2d_4x8_c(int16_t *input, int32_t *output, uint32_t input_stride,
                             TxType transform_type, uint8_t bit_depth) {
    int32_t       intermediate_transform_buffer[4 * 8];
    Txfm2dFlipCfg cfg;
    /*av1_get_fwd_txfm_cfg*/ av1_transform_config(transform_type, TX_4X8, &cfg);
    /*fwd_txfm2d_c*/ av1_tranform_two_d_core_c(
        input, input_stride, output, &cfg, intermediate_transform_buffer, bit_depth);
}

/*********************************************************************/
/*!< * Transform
 *   *   Note there is an implicit assumption that TU Size <= PU Size,
 *   *   which is different than the HEVC requirements. */
/*********************************************************************/
EbErrorType av1_estimate_transform(int16_t *residual_buffer, uint32_t residual_stride,
                                   int32_t *coeff_buffer, uint32_t coeff_stride,
                                   TxSize transform_size, uint64_t *three_quad_energy,
                                   int16_t *transform_inner_array_ptr, uint32_t bit_increment,
                                   TxType transform_type, PlaneType component_type,
                                   EB_TRANS_COEFF_SHAPE trans_coeff_shape)

{
    (void)trans_coeff_shape;
    EbErrorType return_error = EB_ErrorNone;

    (void)transform_inner_array_ptr;
    (void)coeff_stride;
    (void)component_type;
    uint8_t bit_depth = bit_increment ? 10 : 8; /*!< NM - Set to zero for the moment */

    switch (transform_size) {
    case TX_64X32:
        if (transform_type == DCT_DCT)
            eb_av1_fwd_txfm2d_64x32(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_64x32_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        *three_quad_energy = handle_transform64x32(coeff_buffer);

        break;

    case TX_32X64:
        if (transform_type == DCT_DCT)
            eb_av1_fwd_txfm2d_32x64(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_32x64_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        *three_quad_energy = handle_transform32x64(coeff_buffer);

        break;

    case TX_64X16:
        if (transform_type == DCT_DCT)
            eb_av1_fwd_txfm2d_64x16(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_64x16_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        *three_quad_energy = handle_transform64x16(coeff_buffer);

        break;

    case TX_16X64:
        if (transform_type == DCT_DCT)
            eb_av1_fwd_txfm2d_16x64(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_16x64_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        *three_quad_energy = handle_transform16x64(coeff_buffer);

        break;

    case TX_32X16:
        /*!< TTK */
        if ((transform_type == DCT_DCT) || (transform_type == IDTX))
            eb_av1_fwd_txfm2d_32x16(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_32x16_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;

    case TX_16X32:
        if ((transform_type == DCT_DCT) || (transform_type == IDTX))
            eb_av1_fwd_txfm2d_16x32(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_16x32_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;

    case TX_16X8:
        eb_av1_fwd_txfm2d_16x8(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;

    case TX_8X16:
        eb_av1_fwd_txfm2d_8x16(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;

    case TX_32X8:
        if ((transform_type == DCT_DCT) || (transform_type == IDTX))
            eb_av1_fwd_txfm2d_32x8(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_32x8_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;

    case TX_8X32:
        if ((transform_type == DCT_DCT) || (transform_type == IDTX))
            eb_av1_fwd_txfm2d_8x32(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        else
            eb_av1_fwd_txfm2d_8x32_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;
    case TX_16X4:
        eb_av1_fwd_txfm2d_16x4(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;
    case TX_4X16:
        eb_av1_fwd_txfm2d_4x16(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        break;
    case TX_8X4:

        eb_av1_fwd_txfm2d_8x4(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        break;
    case TX_4X8:

        eb_av1_fwd_txfm2d_4x8(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        break;

    case TX_64X64:

        eb_av1_fwd_txfm2d_64x64(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        *three_quad_energy = handle_transform64x64(coeff_buffer);

        break;

    case TX_32X32:
        if (transform_type == V_DCT || transform_type == H_DCT || transform_type == V_ADST ||
            transform_type == H_ADST || transform_type == V_FLIPADST ||
            transform_type == H_FLIPADST)
            /*!< Tahani: I believe those cases are never hit */
            av1_transform_two_d_32x32_c(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        else {
            eb_av1_fwd_txfm2d_32x32(
                residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);
        }

        break;

    case TX_16X16:

        eb_av1_fwd_txfm2d_16x16(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        break;
    case TX_8X8:

        eb_av1_fwd_txfm2d_8x8(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        break;
    case TX_4X4:

        eb_av1_fwd_txfm2d_4x4(
            residual_buffer, coeff_buffer, residual_stride, transform_type, bit_depth);

        break;
    default: assert(0); break;
    }

    return return_error;
}

void eb_av1_gen_inv_stage_range(int8_t *stage_range_col, int8_t *stage_range_row,
                                const Txfm2dFlipCfg *cfg, TxSize tx_size, int32_t bd) {
    const int32_t fwd_shift = inv_start_range[tx_size];
    const int8_t *shift     = cfg->shift;
    int8_t        opt_range_row, opt_range_col;
    if (bd == 8) {
        opt_range_row = 16;
        opt_range_col = 16;
    } else if (bd == 10) {
        opt_range_row = 18;
        opt_range_col = 16;
    } else {
        assert(bd == 12);
        opt_range_row = 20;
        opt_range_col = 18;
    }
    /*!< i < MAX_TXFM_STAGE_NUM will mute above array bounds warning */
    for (int32_t i = 0; i < cfg->stage_num_row && i < MAX_TXFM_STAGE_NUM; ++i) {
        int32_t real_range_row = cfg->stage_range_row[i] + fwd_shift + bd + 1;
        (void)real_range_row;
        if (cfg->txfm_type_row == TXFM_TYPE_ADST4 && i == 1) {
            /*!< the adst4 may use 1 extra bit on top of opt_range_row at stage 1
             *   so opt_range_col >= real_range_col will not hold */
            stage_range_row[i] = opt_range_row;
        } else {
            assert(opt_range_row >= real_range_row);
            stage_range_row[i] = opt_range_row;
        }
    }
    /*!< i < MAX_TXFM_STAGE_NUM will mute above array bounds warning */
    for (int32_t i = 0; i < cfg->stage_num_col && i < MAX_TXFM_STAGE_NUM; ++i) {
        int32_t real_range_col = cfg->stage_range_col[i] + fwd_shift + shift[0] + bd + 1;
        (void)real_range_col;
        if (cfg->txfm_type_col == TXFM_TYPE_ADST4 && i == 1) {
            /*!< the adst4 may use 1 extra bit on top of opt_range_row at stage 1
             *   so opt_range_col >= real_range_col will not hold */
            stage_range_col[i] = opt_range_col;
        } else {
            assert(opt_range_col >= real_range_col);
            stage_range_col[i] = opt_range_col;
        }
    }
}

static INLINE int32_t clamp_value(int32_t value, int8_t bit) {
    if (bit <= 0) return value; /*!< Do nothing for invalid clamp bit. */
    const int64_t max_value = (1LL << (bit - 1)) - 1;
    const int64_t min_value = -(1LL << (bit - 1));
    return (int32_t)clamp64(value, min_value, max_value);
}

void eb_av1_idct4_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                      const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[4];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1    = output;
    bf1[0] = input[0];
    bf1[1] = input[2];
    bf1[2] = input[1];
    bf1[3] = input[3];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0    = output;
    bf1    = step;
    bf1[0] = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1] = half_btf(cospi[32], bf0[0], -cospi[32], bf0[1], cos_bit);
    bf1[2] = half_btf(cospi[48], bf0[2], -cospi[16], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[16], bf0[2], cospi[48], bf0[3], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = clamp_value(bf0[0] + bf0[3], stage_range[stage]);
    bf1[1] = clamp_value(bf0[1] + bf0[2], stage_range[stage]);
    bf1[2] = clamp_value(bf0[1] - bf0[2], stage_range[stage]);
    bf1[3] = clamp_value(bf0[0] - bf0[3], stage_range[stage]);
}
void eb_av1_idct8_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                      const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[8];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1    = output;
    bf1[0] = input[0];
    bf1[1] = input[4];
    bf1[2] = input[2];
    bf1[3] = input[6];
    bf1[4] = input[1];
    bf1[5] = input[5];
    bf1[6] = input[3];
    bf1[7] = input[7];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0];
    bf1[1] = bf0[1];
    bf1[2] = bf0[2];
    bf1[3] = bf0[3];
    bf1[4] = half_btf(cospi[56], bf0[4], -cospi[8], bf0[7], cos_bit);
    bf1[5] = half_btf(cospi[24], bf0[5], -cospi[40], bf0[6], cos_bit);
    bf1[6] = half_btf(cospi[40], bf0[5], cospi[24], bf0[6], cos_bit);
    bf1[7] = half_btf(cospi[8], bf0[4], cospi[56], bf0[7], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1] = half_btf(cospi[32], bf0[0], -cospi[32], bf0[1], cos_bit);
    bf1[2] = half_btf(cospi[48], bf0[2], -cospi[16], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[16], bf0[2], cospi[48], bf0[3], cos_bit);
    bf1[4] = clamp_value(bf0[4] + bf0[5], stage_range[stage]);
    bf1[5] = clamp_value(bf0[4] - bf0[5], stage_range[stage]);
    bf1[6] = clamp_value(-bf0[6] + bf0[7], stage_range[stage]);
    bf1[7] = clamp_value(bf0[6] + bf0[7], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    bf0    = output;
    bf1    = step;
    bf1[0] = clamp_value(bf0[0] + bf0[3], stage_range[stage]);
    bf1[1] = clamp_value(bf0[1] + bf0[2], stage_range[stage]);
    bf1[2] = clamp_value(bf0[1] - bf0[2], stage_range[stage]);
    bf1[3] = clamp_value(bf0[0] - bf0[3], stage_range[stage]);
    bf1[4] = bf0[4];
    bf1[5] = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6] = half_btf(cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[7] = bf0[7];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = clamp_value(bf0[0] + bf0[7], stage_range[stage]);
    bf1[1] = clamp_value(bf0[1] + bf0[6], stage_range[stage]);
    bf1[2] = clamp_value(bf0[2] + bf0[5], stage_range[stage]);
    bf1[3] = clamp_value(bf0[3] + bf0[4], stage_range[stage]);
    bf1[4] = clamp_value(bf0[3] - bf0[4], stage_range[stage]);
    bf1[5] = clamp_value(bf0[2] - bf0[5], stage_range[stage]);
    bf1[6] = clamp_value(bf0[1] - bf0[6], stage_range[stage]);
    bf1[7] = clamp_value(bf0[0] - bf0[7], stage_range[stage]);
}
void eb_av1_idct16_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[16];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[0];
    bf1[1]  = input[8];
    bf1[2]  = input[4];
    bf1[3]  = input[12];
    bf1[4]  = input[2];
    bf1[5]  = input[10];
    bf1[6]  = input[6];
    bf1[7]  = input[14];
    bf1[8]  = input[1];
    bf1[9]  = input[9];
    bf1[10] = input[5];
    bf1[11] = input[13];
    bf1[12] = input[3];
    bf1[13] = input[11];
    bf1[14] = input[7];
    bf1[15] = input[15];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[60], bf0[8], -cospi[4], bf0[15], cos_bit);
    bf1[9]  = half_btf(cospi[28], bf0[9], -cospi[36], bf0[14], cos_bit);
    bf1[10] = half_btf(cospi[44], bf0[10], -cospi[20], bf0[13], cos_bit);
    bf1[11] = half_btf(cospi[12], bf0[11], -cospi[52], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[52], bf0[11], cospi[12], bf0[12], cos_bit);
    bf1[13] = half_btf(cospi[20], bf0[10], cospi[44], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[36], bf0[9], cospi[28], bf0[14], cos_bit);
    bf1[15] = half_btf(cospi[4], bf0[8], cospi[60], bf0[15], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[56], bf0[4], -cospi[8], bf0[7], cos_bit);
    bf1[5]  = half_btf(cospi[24], bf0[5], -cospi[40], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[40], bf0[5], cospi[24], bf0[6], cos_bit);
    bf1[7]  = half_btf(cospi[8], bf0[4], cospi[56], bf0[7], cos_bit);
    bf1[8]  = clamp_value(bf0[8] + bf0[9], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[8] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(-bf0[10] + bf0[11], stage_range[stage]);
    bf1[11] = clamp_value(bf0[10] + bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[13], stage_range[stage]);
    bf1[13] = clamp_value(bf0[12] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(-bf0[14] + bf0[15], stage_range[stage]);
    bf1[15] = clamp_value(bf0[14] + bf0[15], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1]  = half_btf(cospi[32], bf0[0], -cospi[32], bf0[1], cos_bit);
    bf1[2]  = half_btf(cospi[48], bf0[2], -cospi[16], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[16], bf0[2], cospi[48], bf0[3], cos_bit);
    bf1[4]  = clamp_value(bf0[4] + bf0[5], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[4] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(-bf0[6] + bf0[7], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[6] + bf0[7], stage_range[stage]);
    bf1[8]  = bf0[8];
    bf1[9]  = half_btf(-cospi[16], bf0[9], cospi[48], bf0[14], cos_bit);
    bf1[10] = half_btf(-cospi[48], bf0[10], -cospi[16], bf0[13], cos_bit);
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = half_btf(-cospi[16], bf0[10], cospi[48], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[48], bf0[9], cospi[16], bf0[14], cos_bit);
    bf1[15] = bf0[15];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[3], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[2], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[1] - bf0[2], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[0] - bf0[3], stage_range[stage]);
    bf1[4]  = bf0[4];
    bf1[5]  = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[7]  = bf0[7];
    bf1[8]  = clamp_value(bf0[8] + bf0[11], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[10], stage_range[stage]);
    bf1[10] = clamp_value(bf0[9] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[8] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(-bf0[12] + bf0[15], stage_range[stage]);
    bf1[13] = clamp_value(-bf0[13] + bf0[14], stage_range[stage]);
    bf1[14] = clamp_value(bf0[13] + bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[12] + bf0[15], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = clamp_value(bf0[0] + bf0[7], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[6], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[5], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[4], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[3] - bf0[4], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[2] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[1] - bf0[6], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[0] - bf0[7], stage_range[stage]);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(-cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[13] = half_btf(cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[15], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[14], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[13], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[12], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[11], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[10], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[9], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[8], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[7] - bf0[8], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[6] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(bf0[5] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[4] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[3] - bf0[12], stage_range[stage]);
    bf1[13] = clamp_value(bf0[2] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(bf0[1] - bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[0] - bf0[15], stage_range[stage]);
}
void eb_av1_idct32_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[32];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[0];
    bf1[1]  = input[16];
    bf1[2]  = input[8];
    bf1[3]  = input[24];
    bf1[4]  = input[4];
    bf1[5]  = input[20];
    bf1[6]  = input[12];
    bf1[7]  = input[28];
    bf1[8]  = input[2];
    bf1[9]  = input[18];
    bf1[10] = input[10];
    bf1[11] = input[26];
    bf1[12] = input[6];
    bf1[13] = input[22];
    bf1[14] = input[14];
    bf1[15] = input[30];
    bf1[16] = input[1];
    bf1[17] = input[17];
    bf1[18] = input[9];
    bf1[19] = input[25];
    bf1[20] = input[5];
    bf1[21] = input[21];
    bf1[22] = input[13];
    bf1[23] = input[29];
    bf1[24] = input[3];
    bf1[25] = input[19];
    bf1[26] = input[11];
    bf1[27] = input[27];
    bf1[28] = input[7];
    bf1[29] = input[23];
    bf1[30] = input[15];
    bf1[31] = input[31];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = half_btf(cospi[62], bf0[16], -cospi[2], bf0[31], cos_bit);
    bf1[17] = half_btf(cospi[30], bf0[17], -cospi[34], bf0[30], cos_bit);
    bf1[18] = half_btf(cospi[46], bf0[18], -cospi[18], bf0[29], cos_bit);
    bf1[19] = half_btf(cospi[14], bf0[19], -cospi[50], bf0[28], cos_bit);
    bf1[20] = half_btf(cospi[54], bf0[20], -cospi[10], bf0[27], cos_bit);
    bf1[21] = half_btf(cospi[22], bf0[21], -cospi[42], bf0[26], cos_bit);
    bf1[22] = half_btf(cospi[38], bf0[22], -cospi[26], bf0[25], cos_bit);
    bf1[23] = half_btf(cospi[6], bf0[23], -cospi[58], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[58], bf0[23], cospi[6], bf0[24], cos_bit);
    bf1[25] = half_btf(cospi[26], bf0[22], cospi[38], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[42], bf0[21], cospi[22], bf0[26], cos_bit);
    bf1[27] = half_btf(cospi[10], bf0[20], cospi[54], bf0[27], cos_bit);
    bf1[28] = half_btf(cospi[50], bf0[19], cospi[14], bf0[28], cos_bit);
    bf1[29] = half_btf(cospi[18], bf0[18], cospi[46], bf0[29], cos_bit);
    bf1[30] = half_btf(cospi[34], bf0[17], cospi[30], bf0[30], cos_bit);
    bf1[31] = half_btf(cospi[2], bf0[16], cospi[62], bf0[31], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[60], bf0[8], -cospi[4], bf0[15], cos_bit);
    bf1[9]  = half_btf(cospi[28], bf0[9], -cospi[36], bf0[14], cos_bit);
    bf1[10] = half_btf(cospi[44], bf0[10], -cospi[20], bf0[13], cos_bit);
    bf1[11] = half_btf(cospi[12], bf0[11], -cospi[52], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[52], bf0[11], cospi[12], bf0[12], cos_bit);
    bf1[13] = half_btf(cospi[20], bf0[10], cospi[44], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[36], bf0[9], cospi[28], bf0[14], cos_bit);
    bf1[15] = half_btf(cospi[4], bf0[8], cospi[60], bf0[15], cos_bit);
    bf1[16] = clamp_value(bf0[16] + bf0[17], stage_range[stage]);
    bf1[17] = clamp_value(bf0[16] - bf0[17], stage_range[stage]);
    bf1[18] = clamp_value(-bf0[18] + bf0[19], stage_range[stage]);
    bf1[19] = clamp_value(bf0[18] + bf0[19], stage_range[stage]);
    bf1[20] = clamp_value(bf0[20] + bf0[21], stage_range[stage]);
    bf1[21] = clamp_value(bf0[20] - bf0[21], stage_range[stage]);
    bf1[22] = clamp_value(-bf0[22] + bf0[23], stage_range[stage]);
    bf1[23] = clamp_value(bf0[22] + bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(bf0[24] + bf0[25], stage_range[stage]);
    bf1[25] = clamp_value(bf0[24] - bf0[25], stage_range[stage]);
    bf1[26] = clamp_value(-bf0[26] + bf0[27], stage_range[stage]);
    bf1[27] = clamp_value(bf0[26] + bf0[27], stage_range[stage]);
    bf1[28] = clamp_value(bf0[28] + bf0[29], stage_range[stage]);
    bf1[29] = clamp_value(bf0[28] - bf0[29], stage_range[stage]);
    bf1[30] = clamp_value(-bf0[30] + bf0[31], stage_range[stage]);
    bf1[31] = clamp_value(bf0[30] + bf0[31], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[56], bf0[4], -cospi[8], bf0[7], cos_bit);
    bf1[5]  = half_btf(cospi[24], bf0[5], -cospi[40], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[40], bf0[5], cospi[24], bf0[6], cos_bit);
    bf1[7]  = half_btf(cospi[8], bf0[4], cospi[56], bf0[7], cos_bit);
    bf1[8]  = clamp_value(bf0[8] + bf0[9], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[8] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(-bf0[10] + bf0[11], stage_range[stage]);
    bf1[11] = clamp_value(bf0[10] + bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[13], stage_range[stage]);
    bf1[13] = clamp_value(bf0[12] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(-bf0[14] + bf0[15], stage_range[stage]);
    bf1[15] = clamp_value(bf0[14] + bf0[15], stage_range[stage]);
    bf1[16] = bf0[16];
    bf1[17] = half_btf(-cospi[8], bf0[17], cospi[56], bf0[30], cos_bit);
    bf1[18] = half_btf(-cospi[56], bf0[18], -cospi[8], bf0[29], cos_bit);
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = half_btf(-cospi[40], bf0[21], cospi[24], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[24], bf0[22], -cospi[40], bf0[25], cos_bit);
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = half_btf(-cospi[40], bf0[22], cospi[24], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[24], bf0[21], cospi[40], bf0[26], cos_bit);
    bf1[27] = bf0[27];
    bf1[28] = bf0[28];
    bf1[29] = half_btf(-cospi[8], bf0[18], cospi[56], bf0[29], cos_bit);
    bf1[30] = half_btf(cospi[56], bf0[17], cospi[8], bf0[30], cos_bit);
    bf1[31] = bf0[31];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1]  = half_btf(cospi[32], bf0[0], -cospi[32], bf0[1], cos_bit);
    bf1[2]  = half_btf(cospi[48], bf0[2], -cospi[16], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[16], bf0[2], cospi[48], bf0[3], cos_bit);
    bf1[4]  = clamp_value(bf0[4] + bf0[5], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[4] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(-bf0[6] + bf0[7], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[6] + bf0[7], stage_range[stage]);
    bf1[8]  = bf0[8];
    bf1[9]  = half_btf(-cospi[16], bf0[9], cospi[48], bf0[14], cos_bit);
    bf1[10] = half_btf(-cospi[48], bf0[10], -cospi[16], bf0[13], cos_bit);
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = half_btf(-cospi[16], bf0[10], cospi[48], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[48], bf0[9], cospi[16], bf0[14], cos_bit);
    bf1[15] = bf0[15];
    bf1[16] = clamp_value(bf0[16] + bf0[19], stage_range[stage]);
    bf1[17] = clamp_value(bf0[17] + bf0[18], stage_range[stage]);
    bf1[18] = clamp_value(bf0[17] - bf0[18], stage_range[stage]);
    bf1[19] = clamp_value(bf0[16] - bf0[19], stage_range[stage]);
    bf1[20] = clamp_value(-bf0[20] + bf0[23], stage_range[stage]);
    bf1[21] = clamp_value(-bf0[21] + bf0[22], stage_range[stage]);
    bf1[22] = clamp_value(bf0[21] + bf0[22], stage_range[stage]);
    bf1[23] = clamp_value(bf0[20] + bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(bf0[24] + bf0[27], stage_range[stage]);
    bf1[25] = clamp_value(bf0[25] + bf0[26], stage_range[stage]);
    bf1[26] = clamp_value(bf0[25] - bf0[26], stage_range[stage]);
    bf1[27] = clamp_value(bf0[24] - bf0[27], stage_range[stage]);
    bf1[28] = clamp_value(-bf0[28] + bf0[31], stage_range[stage]);
    bf1[29] = clamp_value(-bf0[29] + bf0[30], stage_range[stage]);
    bf1[30] = clamp_value(bf0[29] + bf0[30], stage_range[stage]);
    bf1[31] = clamp_value(bf0[28] + bf0[31], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = clamp_value(bf0[0] + bf0[3], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[2], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[1] - bf0[2], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[0] - bf0[3], stage_range[stage]);
    bf1[4]  = bf0[4];
    bf1[5]  = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[7]  = bf0[7];
    bf1[8]  = clamp_value(bf0[8] + bf0[11], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[10], stage_range[stage]);
    bf1[10] = clamp_value(bf0[9] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[8] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(-bf0[12] + bf0[15], stage_range[stage]);
    bf1[13] = clamp_value(-bf0[13] + bf0[14], stage_range[stage]);
    bf1[14] = clamp_value(bf0[13] + bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[12] + bf0[15], stage_range[stage]);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = half_btf(-cospi[16], bf0[18], cospi[48], bf0[29], cos_bit);
    bf1[19] = half_btf(-cospi[16], bf0[19], cospi[48], bf0[28], cos_bit);
    bf1[20] = half_btf(-cospi[48], bf0[20], -cospi[16], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[48], bf0[21], -cospi[16], bf0[26], cos_bit);
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = half_btf(-cospi[16], bf0[21], cospi[48], bf0[26], cos_bit);
    bf1[27] = half_btf(-cospi[16], bf0[20], cospi[48], bf0[27], cos_bit);
    bf1[28] = half_btf(cospi[48], bf0[19], cospi[16], bf0[28], cos_bit);
    bf1[29] = half_btf(cospi[48], bf0[18], cospi[16], bf0[29], cos_bit);
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[7], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[6], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[5], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[4], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[3] - bf0[4], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[2] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[1] - bf0[6], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[0] - bf0[7], stage_range[stage]);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(-cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[13] = half_btf(cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = clamp_value(bf0[16] + bf0[23], stage_range[stage]);
    bf1[17] = clamp_value(bf0[17] + bf0[22], stage_range[stage]);
    bf1[18] = clamp_value(bf0[18] + bf0[21], stage_range[stage]);
    bf1[19] = clamp_value(bf0[19] + bf0[20], stage_range[stage]);
    bf1[20] = clamp_value(bf0[19] - bf0[20], stage_range[stage]);
    bf1[21] = clamp_value(bf0[18] - bf0[21], stage_range[stage]);
    bf1[22] = clamp_value(bf0[17] - bf0[22], stage_range[stage]);
    bf1[23] = clamp_value(bf0[16] - bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(-bf0[24] + bf0[31], stage_range[stage]);
    bf1[25] = clamp_value(-bf0[25] + bf0[30], stage_range[stage]);
    bf1[26] = clamp_value(-bf0[26] + bf0[29], stage_range[stage]);
    bf1[27] = clamp_value(-bf0[27] + bf0[28], stage_range[stage]);
    bf1[28] = clamp_value(bf0[27] + bf0[28], stage_range[stage]);
    bf1[29] = clamp_value(bf0[26] + bf0[29], stage_range[stage]);
    bf1[30] = clamp_value(bf0[25] + bf0[30], stage_range[stage]);
    bf1[31] = clamp_value(bf0[24] + bf0[31], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = clamp_value(bf0[0] + bf0[15], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[14], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[13], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[12], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[11], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[10], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[9], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[8], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[7] - bf0[8], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[6] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(bf0[5] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[4] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[3] - bf0[12], stage_range[stage]);
    bf1[13] = clamp_value(bf0[2] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(bf0[1] - bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[0] - bf0[15], stage_range[stage]);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = half_btf(-cospi[32], bf0[20], cospi[32], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[32], bf0[21], cospi[32], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[32], bf0[22], cospi[32], bf0[25], cos_bit);
    bf1[23] = half_btf(-cospi[32], bf0[23], cospi[32], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[32], bf0[23], cospi[32], bf0[24], cos_bit);
    bf1[25] = half_btf(cospi[32], bf0[22], cospi[32], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[32], bf0[21], cospi[32], bf0[26], cos_bit);
    bf1[27] = half_btf(cospi[32], bf0[20], cospi[32], bf0[27], cos_bit);
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[31], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[30], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[29], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[28], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[27], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[26], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[25], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[24], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[8] + bf0[23], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[22], stage_range[stage]);
    bf1[10] = clamp_value(bf0[10] + bf0[21], stage_range[stage]);
    bf1[11] = clamp_value(bf0[11] + bf0[20], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[19], stage_range[stage]);
    bf1[13] = clamp_value(bf0[13] + bf0[18], stage_range[stage]);
    bf1[14] = clamp_value(bf0[14] + bf0[17], stage_range[stage]);
    bf1[15] = clamp_value(bf0[15] + bf0[16], stage_range[stage]);
    bf1[16] = clamp_value(bf0[15] - bf0[16], stage_range[stage]);
    bf1[17] = clamp_value(bf0[14] - bf0[17], stage_range[stage]);
    bf1[18] = clamp_value(bf0[13] - bf0[18], stage_range[stage]);
    bf1[19] = clamp_value(bf0[12] - bf0[19], stage_range[stage]);
    bf1[20] = clamp_value(bf0[11] - bf0[20], stage_range[stage]);
    bf1[21] = clamp_value(bf0[10] - bf0[21], stage_range[stage]);
    bf1[22] = clamp_value(bf0[9] - bf0[22], stage_range[stage]);
    bf1[23] = clamp_value(bf0[8] - bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(bf0[7] - bf0[24], stage_range[stage]);
    bf1[25] = clamp_value(bf0[6] - bf0[25], stage_range[stage]);
    bf1[26] = clamp_value(bf0[5] - bf0[26], stage_range[stage]);
    bf1[27] = clamp_value(bf0[4] - bf0[27], stage_range[stage]);
    bf1[28] = clamp_value(bf0[3] - bf0[28], stage_range[stage]);
    bf1[29] = clamp_value(bf0[2] - bf0[29], stage_range[stage]);
    bf1[30] = clamp_value(bf0[1] - bf0[30], stage_range[stage]);
    bf1[31] = clamp_value(bf0[0] - bf0[31], stage_range[stage]);
}
void eb_av1_iadst4_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    (void)stage_range;
    int32_t        bit   = cos_bit;
    const int32_t *sinpi = sinpi_arr(bit);
    int32_t        s0, s1, s2, s3, s4, s5, s6, s7;

    int32_t x0 = input[0];
    int32_t x1 = input[1];
    int32_t x2 = input[2];
    int32_t x3 = input[3];

    if (!(x0 | x1 | x2 | x3)) {
        output[0] = output[1] = output[2] = output[3] = 0;
        return;
    }

    assert(sinpi[1] + sinpi[2] == sinpi[4]);

    /*!< stage 1 */
    //s0 = range_check_value(sinpi[1] * x0, stage_range[1] + bit);
    //s1 = range_check_value(sinpi[2] * x0, stage_range[1] + bit);
    //s2 = range_check_value(sinpi[3] * x1, stage_range[1] + bit);
    //s3 = range_check_value(sinpi[4] * x2, stage_range[1] + bit);
    //s4 = range_check_value(sinpi[1] * x2, stage_range[1] + bit);
    //s5 = range_check_value(sinpi[2] * x3, stage_range[1] + bit);
    //s6 = range_check_value(sinpi[4] * x3, stage_range[1] + bit);

    s0 = sinpi[1] * x0;
    s1 = sinpi[2] * x0;
    s2 = sinpi[3] * x1;
    s3 = sinpi[4] * x2;
    s4 = sinpi[1] * x2;
    s5 = sinpi[2] * x3;
    s6 = sinpi[4] * x3;

    /*!< stage 2 */
    /*!< NOTICE: (x0 - x2) here may use one extra bit compared to the
     *   opt_range_row/col specified in eb_av1_gen_inv_stage_range() */
    //s7 = range_check_value((x0 - x2) + x3, stage_range[2]);

    /*!< stage 3 */
    //s0 = range_check_value(s0 + s3, stage_range[3] + bit);
    //s1 = range_check_value(s1 - s4, stage_range[3] + bit);
    //s3 = range_check_value(s2, stage_range[3] + bit);
    //s2 = range_check_value(sinpi[3] * s7, stage_range[3] + bit);

    /*!< stage 4 */
    //s0 = range_check_value(s0 + s5, stage_range[4] + bit);
    //s1 = range_check_value(s1 - s6, stage_range[4] + bit);

    /*!< stage 5 */
    //x0 = range_check_value(s0 + s3, stage_range[5] + bit);
    //x1 = range_check_value(s1 + s3, stage_range[5] + bit);
    //x2 = range_check_value(s2, stage_range[5] + bit);
    //x3 = range_check_value(s0 + s1, stage_range[5] + bit);

    /*!< stage 6 */
    //x3 = range_check_value(x3 - s3, stage_range[6] + bit);

    s7 = (x0 - x2) + x3;

    /*!< stage 3 */
    s0 = s0 + s3;
    s1 = s1 - s4;
    s3 = s2;
    s2 = sinpi[3] * s7;

    /*!< stage 4 */
    s0 = s0 + s5;
    s1 = s1 - s6;

    /*!< stage 5 */
    x0 = s0 + s3;
    x1 = s1 + s3;
    x2 = s2;
    x3 = s0 + s1;

    /*!< stage 6 */
    x3 = x3 - s3;

    output[0] = round_shift(x0, bit);
    output[1] = round_shift(x1, bit);
    output[2] = round_shift(x2, bit);
    output[3] = round_shift(x3, bit);
    //range_check_buf(6, input, output, 4, stage_range[6]);
}
static INLINE void clamp_buf(int32_t *buf, int32_t size, int8_t bit) {
    for (int32_t i = 0; i < size; ++i) buf[i] = clamp_value(buf[i], bit);
}
void eb_av1_iadst8_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[8];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1    = output;
    bf1[0] = input[7];
    bf1[1] = input[0];
    bf1[2] = input[5];
    bf1[3] = input[2];
    bf1[4] = input[3];
    bf1[5] = input[4];
    bf1[6] = input[1];
    bf1[7] = input[6];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0    = output;
    bf1    = step;
    bf1[0] = half_btf(cospi[4], bf0[0], cospi[60], bf0[1], cos_bit);
    bf1[1] = half_btf(cospi[60], bf0[0], -cospi[4], bf0[1], cos_bit);
    bf1[2] = half_btf(cospi[20], bf0[2], cospi[44], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[44], bf0[2], -cospi[20], bf0[3], cos_bit);
    bf1[4] = half_btf(cospi[36], bf0[4], cospi[28], bf0[5], cos_bit);
    bf1[5] = half_btf(cospi[28], bf0[4], -cospi[36], bf0[5], cos_bit);
    bf1[6] = half_btf(cospi[52], bf0[6], cospi[12], bf0[7], cos_bit);
    bf1[7] = half_btf(cospi[12], bf0[6], -cospi[52], bf0[7], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = clamp_value(bf0[0] + bf0[4], stage_range[stage]);
    bf1[1] = clamp_value(bf0[1] + bf0[5], stage_range[stage]);
    bf1[2] = clamp_value(bf0[2] + bf0[6], stage_range[stage]);
    bf1[3] = clamp_value(bf0[3] + bf0[7], stage_range[stage]);
    bf1[4] = clamp_value(bf0[0] - bf0[4], stage_range[stage]);
    bf1[5] = clamp_value(bf0[1] - bf0[5], stage_range[stage]);
    bf1[6] = clamp_value(bf0[2] - bf0[6], stage_range[stage]);
    bf1[7] = clamp_value(bf0[3] - bf0[7], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0];
    bf1[1] = bf0[1];
    bf1[2] = bf0[2];
    bf1[3] = bf0[3];
    bf1[4] = half_btf(cospi[16], bf0[4], cospi[48], bf0[5], cos_bit);
    bf1[5] = half_btf(cospi[48], bf0[4], -cospi[16], bf0[5], cos_bit);
    bf1[6] = half_btf(-cospi[48], bf0[6], cospi[16], bf0[7], cos_bit);
    bf1[7] = half_btf(cospi[16], bf0[6], cospi[48], bf0[7], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = clamp_value(bf0[0] + bf0[2], stage_range[stage]);
    bf1[1] = clamp_value(bf0[1] + bf0[3], stage_range[stage]);
    bf1[2] = clamp_value(bf0[0] - bf0[2], stage_range[stage]);
    bf1[3] = clamp_value(bf0[1] - bf0[3], stage_range[stage]);
    bf1[4] = clamp_value(bf0[4] + bf0[6], stage_range[stage]);
    bf1[5] = clamp_value(bf0[5] + bf0[7], stage_range[stage]);
    bf1[6] = clamp_value(bf0[4] - bf0[6], stage_range[stage]);
    bf1[7] = clamp_value(bf0[5] - bf0[7], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    bf0    = output;
    bf1    = step;
    bf1[0] = bf0[0];
    bf1[1] = bf0[1];
    bf1[2] = half_btf(cospi[32], bf0[2], cospi[32], bf0[3], cos_bit);
    bf1[3] = half_btf(cospi[32], bf0[2], -cospi[32], bf0[3], cos_bit);
    bf1[4] = bf0[4];
    bf1[5] = bf0[5];
    bf1[6] = half_btf(cospi[32], bf0[6], cospi[32], bf0[7], cos_bit);
    bf1[7] = half_btf(cospi[32], bf0[6], -cospi[32], bf0[7], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0    = step;
    bf1    = output;
    bf1[0] = bf0[0];
    bf1[1] = -bf0[4];
    bf1[2] = bf0[6];
    bf1[3] = -bf0[2];
    bf1[4] = bf0[3];
    bf1[5] = -bf0[7];
    bf1[6] = bf0[5];
    bf1[7] = -bf0[1];
}
void eb_av1_iadst16_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                        const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[16];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[15];
    bf1[1]  = input[0];
    bf1[2]  = input[13];
    bf1[3]  = input[2];
    bf1[4]  = input[11];
    bf1[5]  = input[4];
    bf1[6]  = input[9];
    bf1[7]  = input[6];
    bf1[8]  = input[7];
    bf1[9]  = input[8];
    bf1[10] = input[5];
    bf1[11] = input[10];
    bf1[12] = input[3];
    bf1[13] = input[12];
    bf1[14] = input[1];
    bf1[15] = input[14];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[2], bf0[0], cospi[62], bf0[1], cos_bit);
    bf1[1]  = half_btf(cospi[62], bf0[0], -cospi[2], bf0[1], cos_bit);
    bf1[2]  = half_btf(cospi[10], bf0[2], cospi[54], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[54], bf0[2], -cospi[10], bf0[3], cos_bit);
    bf1[4]  = half_btf(cospi[18], bf0[4], cospi[46], bf0[5], cos_bit);
    bf1[5]  = half_btf(cospi[46], bf0[4], -cospi[18], bf0[5], cos_bit);
    bf1[6]  = half_btf(cospi[26], bf0[6], cospi[38], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[38], bf0[6], -cospi[26], bf0[7], cos_bit);
    bf1[8]  = half_btf(cospi[34], bf0[8], cospi[30], bf0[9], cos_bit);
    bf1[9]  = half_btf(cospi[30], bf0[8], -cospi[34], bf0[9], cos_bit);
    bf1[10] = half_btf(cospi[42], bf0[10], cospi[22], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[22], bf0[10], -cospi[42], bf0[11], cos_bit);
    bf1[12] = half_btf(cospi[50], bf0[12], cospi[14], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[14], bf0[12], -cospi[50], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[58], bf0[14], cospi[6], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[6], bf0[14], -cospi[58], bf0[15], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[8], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[9], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[10], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[11], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[12], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[13], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[14], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[15], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[0] - bf0[8], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[1] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(bf0[2] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[3] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[4] - bf0[12], stage_range[stage]);
    bf1[13] = clamp_value(bf0[5] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(bf0[6] - bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[7] - bf0[15], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[8], bf0[8], cospi[56], bf0[9], cos_bit);
    bf1[9]  = half_btf(cospi[56], bf0[8], -cospi[8], bf0[9], cos_bit);
    bf1[10] = half_btf(cospi[40], bf0[10], cospi[24], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[24], bf0[10], -cospi[40], bf0[11], cos_bit);
    bf1[12] = half_btf(-cospi[56], bf0[12], cospi[8], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[8], bf0[12], cospi[56], bf0[13], cos_bit);
    bf1[14] = half_btf(-cospi[24], bf0[14], cospi[40], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[40], bf0[14], cospi[24], bf0[15], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[4], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[5], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[6], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[7], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[0] - bf0[4], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[1] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[2] - bf0[6], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[3] - bf0[7], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[8] + bf0[12], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[13], stage_range[stage]);
    bf1[10] = clamp_value(bf0[10] + bf0[14], stage_range[stage]);
    bf1[11] = clamp_value(bf0[11] + bf0[15], stage_range[stage]);
    bf1[12] = clamp_value(bf0[8] - bf0[12], stage_range[stage]);
    bf1[13] = clamp_value(bf0[9] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(bf0[10] - bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[11] - bf0[15], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[16], bf0[4], cospi[48], bf0[5], cos_bit);
    bf1[5]  = half_btf(cospi[48], bf0[4], -cospi[16], bf0[5], cos_bit);
    bf1[6]  = half_btf(-cospi[48], bf0[6], cospi[16], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[16], bf0[6], cospi[48], bf0[7], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = half_btf(cospi[16], bf0[12], cospi[48], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[48], bf0[12], -cospi[16], bf0[13], cos_bit);
    bf1[14] = half_btf(-cospi[48], bf0[14], cospi[16], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[16], bf0[14], cospi[48], bf0[15], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[2], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[3], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[0] - bf0[2], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[1] - bf0[3], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[6], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[7], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[4] - bf0[6], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[5] - bf0[7], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[8] + bf0[10], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[11], stage_range[stage]);
    bf1[10] = clamp_value(bf0[8] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[9] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[14], stage_range[stage]);
    bf1[13] = clamp_value(bf0[13] + bf0[15], stage_range[stage]);
    bf1[14] = clamp_value(bf0[12] - bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[13] - bf0[15], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = half_btf(cospi[32], bf0[2], cospi[32], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[32], bf0[2], -cospi[32], bf0[3], cos_bit);
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[32], bf0[6], -cospi[32], bf0[7], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(cospi[32], bf0[10], cospi[32], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[32], bf0[10], -cospi[32], bf0[11], cos_bit);
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = half_btf(cospi[32], bf0[14], cospi[32], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[32], bf0[14], -cospi[32], bf0[15], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = -bf0[8];
    bf1[2]  = bf0[12];
    bf1[3]  = -bf0[4];
    bf1[4]  = bf0[6];
    bf1[5]  = -bf0[14];
    bf1[6]  = bf0[10];
    bf1[7]  = -bf0[2];
    bf1[8]  = bf0[3];
    bf1[9]  = -bf0[11];
    bf1[10] = bf0[15];
    bf1[11] = -bf0[7];
    bf1[12] = bf0[5];
    bf1[13] = -bf0[13];
    bf1[14] = bf0[9];
    bf1[15] = -bf0[1];
}
void av1_iadst32_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                     const int8_t *stage_range) {
    const int32_t  size = 32;
    const int32_t *cospi;

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[32];

    /*!< stage 0 */
    clamp_buf((int32_t *)input, size, stage_range[stage]);

    /*!< stage 1 */
    stage++;
    assert(output != input);
    bf1     = output;
    bf1[0]  = input[0];
    bf1[1]  = -input[31];
    bf1[2]  = -input[15];
    bf1[3]  = input[16];
    bf1[4]  = -input[7];
    bf1[5]  = input[24];
    bf1[6]  = input[8];
    bf1[7]  = -input[23];
    bf1[8]  = -input[3];
    bf1[9]  = input[28];
    bf1[10] = input[12];
    bf1[11] = -input[19];
    bf1[12] = input[4];
    bf1[13] = -input[27];
    bf1[14] = -input[11];
    bf1[15] = input[20];
    bf1[16] = -input[1];
    bf1[17] = input[30];
    bf1[18] = input[14];
    bf1[19] = -input[17];
    bf1[20] = input[6];
    bf1[21] = -input[25];
    bf1[22] = -input[9];
    bf1[23] = input[22];
    bf1[24] = input[2];
    bf1[25] = -input[29];
    bf1[26] = -input[13];
    bf1[27] = input[18];
    bf1[28] = -input[5];
    bf1[29] = input[26];
    bf1[30] = input[10];
    bf1[31] = -input[21];
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = half_btf(cospi[32], bf0[2], cospi[32], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[32], bf0[2], -cospi[32], bf0[3], cos_bit);
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = half_btf(cospi[32], bf0[6], cospi[32], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[32], bf0[6], -cospi[32], bf0[7], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(cospi[32], bf0[10], cospi[32], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[32], bf0[10], -cospi[32], bf0[11], cos_bit);
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = half_btf(cospi[32], bf0[14], cospi[32], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[32], bf0[14], -cospi[32], bf0[15], cos_bit);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = half_btf(cospi[32], bf0[18], cospi[32], bf0[19], cos_bit);
    bf1[19] = half_btf(cospi[32], bf0[18], -cospi[32], bf0[19], cos_bit);
    bf1[20] = bf0[20];
    bf1[21] = bf0[21];
    bf1[22] = half_btf(cospi[32], bf0[22], cospi[32], bf0[23], cos_bit);
    bf1[23] = half_btf(cospi[32], bf0[22], -cospi[32], bf0[23], cos_bit);
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = half_btf(cospi[32], bf0[26], cospi[32], bf0[27], cos_bit);
    bf1[27] = half_btf(cospi[32], bf0[26], -cospi[32], bf0[27], cos_bit);
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = half_btf(cospi[32], bf0[30], cospi[32], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[32], bf0[30], -cospi[32], bf0[31], cos_bit);
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[2];
    bf1[1]  = bf0[1] + bf0[3];
    bf1[2]  = bf0[0] - bf0[2];
    bf1[3]  = bf0[1] - bf0[3];
    bf1[4]  = bf0[4] + bf0[6];
    bf1[5]  = bf0[5] + bf0[7];
    bf1[6]  = bf0[4] - bf0[6];
    bf1[7]  = bf0[5] - bf0[7];
    bf1[8]  = bf0[8] + bf0[10];
    bf1[9]  = bf0[9] + bf0[11];
    bf1[10] = bf0[8] - bf0[10];
    bf1[11] = bf0[9] - bf0[11];
    bf1[12] = bf0[12] + bf0[14];
    bf1[13] = bf0[13] + bf0[15];
    bf1[14] = bf0[12] - bf0[14];
    bf1[15] = bf0[13] - bf0[15];
    bf1[16] = bf0[16] + bf0[18];
    bf1[17] = bf0[17] + bf0[19];
    bf1[18] = bf0[16] - bf0[18];
    bf1[19] = bf0[17] - bf0[19];
    bf1[20] = bf0[20] + bf0[22];
    bf1[21] = bf0[21] + bf0[23];
    bf1[22] = bf0[20] - bf0[22];
    bf1[23] = bf0[21] - bf0[23];
    bf1[24] = bf0[24] + bf0[26];
    bf1[25] = bf0[25] + bf0[27];
    bf1[26] = bf0[24] - bf0[26];
    bf1[27] = bf0[25] - bf0[27];
    bf1[28] = bf0[28] + bf0[30];
    bf1[29] = bf0[29] + bf0[31];
    bf1[30] = bf0[28] - bf0[30];
    bf1[31] = bf0[29] - bf0[31];
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[16], bf0[4], cospi[48], bf0[5], cos_bit);
    bf1[5]  = half_btf(cospi[48], bf0[4], -cospi[16], bf0[5], cos_bit);
    bf1[6]  = half_btf(-cospi[48], bf0[6], cospi[16], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[16], bf0[6], cospi[48], bf0[7], cos_bit);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = half_btf(cospi[16], bf0[12], cospi[48], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[48], bf0[12], -cospi[16], bf0[13], cos_bit);
    bf1[14] = half_btf(-cospi[48], bf0[14], cospi[16], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[16], bf0[14], cospi[48], bf0[15], cos_bit);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = half_btf(cospi[16], bf0[20], cospi[48], bf0[21], cos_bit);
    bf1[21] = half_btf(cospi[48], bf0[20], -cospi[16], bf0[21], cos_bit);
    bf1[22] = half_btf(-cospi[48], bf0[22], cospi[16], bf0[23], cos_bit);
    bf1[23] = half_btf(cospi[16], bf0[22], cospi[48], bf0[23], cos_bit);
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = bf0[26];
    bf1[27] = bf0[27];
    bf1[28] = half_btf(cospi[16], bf0[28], cospi[48], bf0[29], cos_bit);
    bf1[29] = half_btf(cospi[48], bf0[28], -cospi[16], bf0[29], cos_bit);
    bf1[30] = half_btf(-cospi[48], bf0[30], cospi[16], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[16], bf0[30], cospi[48], bf0[31], cos_bit);
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[4];
    bf1[1]  = bf0[1] + bf0[5];
    bf1[2]  = bf0[2] + bf0[6];
    bf1[3]  = bf0[3] + bf0[7];
    bf1[4]  = bf0[0] - bf0[4];
    bf1[5]  = bf0[1] - bf0[5];
    bf1[6]  = bf0[2] - bf0[6];
    bf1[7]  = bf0[3] - bf0[7];
    bf1[8]  = bf0[8] + bf0[12];
    bf1[9]  = bf0[9] + bf0[13];
    bf1[10] = bf0[10] + bf0[14];
    bf1[11] = bf0[11] + bf0[15];
    bf1[12] = bf0[8] - bf0[12];
    bf1[13] = bf0[9] - bf0[13];
    bf1[14] = bf0[10] - bf0[14];
    bf1[15] = bf0[11] - bf0[15];
    bf1[16] = bf0[16] + bf0[20];
    bf1[17] = bf0[17] + bf0[21];
    bf1[18] = bf0[18] + bf0[22];
    bf1[19] = bf0[19] + bf0[23];
    bf1[20] = bf0[16] - bf0[20];
    bf1[21] = bf0[17] - bf0[21];
    bf1[22] = bf0[18] - bf0[22];
    bf1[23] = bf0[19] - bf0[23];
    bf1[24] = bf0[24] + bf0[28];
    bf1[25] = bf0[25] + bf0[29];
    bf1[26] = bf0[26] + bf0[30];
    bf1[27] = bf0[27] + bf0[31];
    bf1[28] = bf0[24] - bf0[28];
    bf1[29] = bf0[25] - bf0[29];
    bf1[30] = bf0[26] - bf0[30];
    bf1[31] = bf0[27] - bf0[31];
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[8], bf0[8], cospi[56], bf0[9], cos_bit);
    bf1[9]  = half_btf(cospi[56], bf0[8], -cospi[8], bf0[9], cos_bit);
    bf1[10] = half_btf(cospi[40], bf0[10], cospi[24], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[24], bf0[10], -cospi[40], bf0[11], cos_bit);
    bf1[12] = half_btf(-cospi[56], bf0[12], cospi[8], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[8], bf0[12], cospi[56], bf0[13], cos_bit);
    bf1[14] = half_btf(-cospi[24], bf0[14], cospi[40], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[40], bf0[14], cospi[24], bf0[15], cos_bit);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = bf0[21];
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = half_btf(cospi[8], bf0[24], cospi[56], bf0[25], cos_bit);
    bf1[25] = half_btf(cospi[56], bf0[24], -cospi[8], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[40], bf0[26], cospi[24], bf0[27], cos_bit);
    bf1[27] = half_btf(cospi[24], bf0[26], -cospi[40], bf0[27], cos_bit);
    bf1[28] = half_btf(-cospi[56], bf0[28], cospi[8], bf0[29], cos_bit);
    bf1[29] = half_btf(cospi[8], bf0[28], cospi[56], bf0[29], cos_bit);
    bf1[30] = half_btf(-cospi[24], bf0[30], cospi[40], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[40], bf0[30], cospi[24], bf0[31], cos_bit);
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[8];
    bf1[1]  = bf0[1] + bf0[9];
    bf1[2]  = bf0[2] + bf0[10];
    bf1[3]  = bf0[3] + bf0[11];
    bf1[4]  = bf0[4] + bf0[12];
    bf1[5]  = bf0[5] + bf0[13];
    bf1[6]  = bf0[6] + bf0[14];
    bf1[7]  = bf0[7] + bf0[15];
    bf1[8]  = bf0[0] - bf0[8];
    bf1[9]  = bf0[1] - bf0[9];
    bf1[10] = bf0[2] - bf0[10];
    bf1[11] = bf0[3] - bf0[11];
    bf1[12] = bf0[4] - bf0[12];
    bf1[13] = bf0[5] - bf0[13];
    bf1[14] = bf0[6] - bf0[14];
    bf1[15] = bf0[7] - bf0[15];
    bf1[16] = bf0[16] + bf0[24];
    bf1[17] = bf0[17] + bf0[25];
    bf1[18] = bf0[18] + bf0[26];
    bf1[19] = bf0[19] + bf0[27];
    bf1[20] = bf0[20] + bf0[28];
    bf1[21] = bf0[21] + bf0[29];
    bf1[22] = bf0[22] + bf0[30];
    bf1[23] = bf0[23] + bf0[31];
    bf1[24] = bf0[16] - bf0[24];
    bf1[25] = bf0[17] - bf0[25];
    bf1[26] = bf0[18] - bf0[26];
    bf1[27] = bf0[19] - bf0[27];
    bf1[28] = bf0[20] - bf0[28];
    bf1[29] = bf0[21] - bf0[29];
    bf1[30] = bf0[22] - bf0[30];
    bf1[31] = bf0[23] - bf0[31];
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = half_btf(cospi[4], bf0[16], cospi[60], bf0[17], cos_bit);
    bf1[17] = half_btf(cospi[60], bf0[16], -cospi[4], bf0[17], cos_bit);
    bf1[18] = half_btf(cospi[20], bf0[18], cospi[44], bf0[19], cos_bit);
    bf1[19] = half_btf(cospi[44], bf0[18], -cospi[20], bf0[19], cos_bit);
    bf1[20] = half_btf(cospi[36], bf0[20], cospi[28], bf0[21], cos_bit);
    bf1[21] = half_btf(cospi[28], bf0[20], -cospi[36], bf0[21], cos_bit);
    bf1[22] = half_btf(cospi[52], bf0[22], cospi[12], bf0[23], cos_bit);
    bf1[23] = half_btf(cospi[12], bf0[22], -cospi[52], bf0[23], cos_bit);
    bf1[24] = half_btf(-cospi[60], bf0[24], cospi[4], bf0[25], cos_bit);
    bf1[25] = half_btf(cospi[4], bf0[24], cospi[60], bf0[25], cos_bit);
    bf1[26] = half_btf(-cospi[44], bf0[26], cospi[20], bf0[27], cos_bit);
    bf1[27] = half_btf(cospi[20], bf0[26], cospi[44], bf0[27], cos_bit);
    bf1[28] = half_btf(-cospi[28], bf0[28], cospi[36], bf0[29], cos_bit);
    bf1[29] = half_btf(cospi[36], bf0[28], cospi[28], bf0[29], cos_bit);
    bf1[30] = half_btf(-cospi[12], bf0[30], cospi[52], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[52], bf0[30], cospi[12], bf0[31], cos_bit);
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0] + bf0[16];
    bf1[1]  = bf0[1] + bf0[17];
    bf1[2]  = bf0[2] + bf0[18];
    bf1[3]  = bf0[3] + bf0[19];
    bf1[4]  = bf0[4] + bf0[20];
    bf1[5]  = bf0[5] + bf0[21];
    bf1[6]  = bf0[6] + bf0[22];
    bf1[7]  = bf0[7] + bf0[23];
    bf1[8]  = bf0[8] + bf0[24];
    bf1[9]  = bf0[9] + bf0[25];
    bf1[10] = bf0[10] + bf0[26];
    bf1[11] = bf0[11] + bf0[27];
    bf1[12] = bf0[12] + bf0[28];
    bf1[13] = bf0[13] + bf0[29];
    bf1[14] = bf0[14] + bf0[30];
    bf1[15] = bf0[15] + bf0[31];
    bf1[16] = bf0[0] - bf0[16];
    bf1[17] = bf0[1] - bf0[17];
    bf1[18] = bf0[2] - bf0[18];
    bf1[19] = bf0[3] - bf0[19];
    bf1[20] = bf0[4] - bf0[20];
    bf1[21] = bf0[5] - bf0[21];
    bf1[22] = bf0[6] - bf0[22];
    bf1[23] = bf0[7] - bf0[23];
    bf1[24] = bf0[8] - bf0[24];
    bf1[25] = bf0[9] - bf0[25];
    bf1[26] = bf0[10] - bf0[26];
    bf1[27] = bf0[11] - bf0[27];
    bf1[28] = bf0[12] - bf0[28];
    bf1[29] = bf0[13] - bf0[29];
    bf1[30] = bf0[14] - bf0[30];
    bf1[31] = bf0[15] - bf0[31];
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 10 */
    stage++;
    cospi   = cospi_arr(cos_bit);
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[1], bf0[0], cospi[63], bf0[1], cos_bit);
    bf1[1]  = half_btf(cospi[63], bf0[0], -cospi[1], bf0[1], cos_bit);
    bf1[2]  = half_btf(cospi[5], bf0[2], cospi[59], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[59], bf0[2], -cospi[5], bf0[3], cos_bit);
    bf1[4]  = half_btf(cospi[9], bf0[4], cospi[55], bf0[5], cos_bit);
    bf1[5]  = half_btf(cospi[55], bf0[4], -cospi[9], bf0[5], cos_bit);
    bf1[6]  = half_btf(cospi[13], bf0[6], cospi[51], bf0[7], cos_bit);
    bf1[7]  = half_btf(cospi[51], bf0[6], -cospi[13], bf0[7], cos_bit);
    bf1[8]  = half_btf(cospi[17], bf0[8], cospi[47], bf0[9], cos_bit);
    bf1[9]  = half_btf(cospi[47], bf0[8], -cospi[17], bf0[9], cos_bit);
    bf1[10] = half_btf(cospi[21], bf0[10], cospi[43], bf0[11], cos_bit);
    bf1[11] = half_btf(cospi[43], bf0[10], -cospi[21], bf0[11], cos_bit);
    bf1[12] = half_btf(cospi[25], bf0[12], cospi[39], bf0[13], cos_bit);
    bf1[13] = half_btf(cospi[39], bf0[12], -cospi[25], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[29], bf0[14], cospi[35], bf0[15], cos_bit);
    bf1[15] = half_btf(cospi[35], bf0[14], -cospi[29], bf0[15], cos_bit);
    bf1[16] = half_btf(cospi[33], bf0[16], cospi[31], bf0[17], cos_bit);
    bf1[17] = half_btf(cospi[31], bf0[16], -cospi[33], bf0[17], cos_bit);
    bf1[18] = half_btf(cospi[37], bf0[18], cospi[27], bf0[19], cos_bit);
    bf1[19] = half_btf(cospi[27], bf0[18], -cospi[37], bf0[19], cos_bit);
    bf1[20] = half_btf(cospi[41], bf0[20], cospi[23], bf0[21], cos_bit);
    bf1[21] = half_btf(cospi[23], bf0[20], -cospi[41], bf0[21], cos_bit);
    bf1[22] = half_btf(cospi[45], bf0[22], cospi[19], bf0[23], cos_bit);
    bf1[23] = half_btf(cospi[19], bf0[22], -cospi[45], bf0[23], cos_bit);
    bf1[24] = half_btf(cospi[49], bf0[24], cospi[15], bf0[25], cos_bit);
    bf1[25] = half_btf(cospi[15], bf0[24], -cospi[49], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[53], bf0[26], cospi[11], bf0[27], cos_bit);
    bf1[27] = half_btf(cospi[11], bf0[26], -cospi[53], bf0[27], cos_bit);
    bf1[28] = half_btf(cospi[57], bf0[28], cospi[7], bf0[29], cos_bit);
    bf1[29] = half_btf(cospi[7], bf0[28], -cospi[57], bf0[29], cos_bit);
    bf1[30] = half_btf(cospi[61], bf0[30], cospi[3], bf0[31], cos_bit);
    bf1[31] = half_btf(cospi[3], bf0[30], -cospi[61], bf0[31], cos_bit);
    clamp_buf(bf1, size, stage_range[stage]);

    /*!< stage 11 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[1];
    bf1[1]  = bf0[30];
    bf1[2]  = bf0[3];
    bf1[3]  = bf0[28];
    bf1[4]  = bf0[5];
    bf1[5]  = bf0[26];
    bf1[6]  = bf0[7];
    bf1[7]  = bf0[24];
    bf1[8]  = bf0[9];
    bf1[9]  = bf0[22];
    bf1[10] = bf0[11];
    bf1[11] = bf0[20];
    bf1[12] = bf0[13];
    bf1[13] = bf0[18];
    bf1[14] = bf0[15];
    bf1[15] = bf0[16];
    bf1[16] = bf0[17];
    bf1[17] = bf0[14];
    bf1[18] = bf0[19];
    bf1[19] = bf0[12];
    bf1[20] = bf0[21];
    bf1[21] = bf0[10];
    bf1[22] = bf0[23];
    bf1[23] = bf0[8];
    bf1[24] = bf0[25];
    bf1[25] = bf0[6];
    bf1[26] = bf0[27];
    bf1[27] = bf0[4];
    bf1[28] = bf0[29];
    bf1[29] = bf0[2];
    bf1[30] = bf0[31];
    bf1[31] = bf0[0];
    clamp_buf(bf1, size, stage_range[stage]);
}
void eb_av1_idct64_new(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    assert(output != input);
    const int32_t *cospi = cospi_arr(cos_bit);

    int32_t  stage = 0;
    int32_t *bf0, *bf1;
    int32_t  step[64];

    /*!< stage 0 */

    /*!< stage 1 */
    stage++;
    bf1     = output;
    bf1[0]  = input[0];
    bf1[1]  = input[32];
    bf1[2]  = input[16];
    bf1[3]  = input[48];
    bf1[4]  = input[8];
    bf1[5]  = input[40];
    bf1[6]  = input[24];
    bf1[7]  = input[56];
    bf1[8]  = input[4];
    bf1[9]  = input[36];
    bf1[10] = input[20];
    bf1[11] = input[52];
    bf1[12] = input[12];
    bf1[13] = input[44];
    bf1[14] = input[28];
    bf1[15] = input[60];
    bf1[16] = input[2];
    bf1[17] = input[34];
    bf1[18] = input[18];
    bf1[19] = input[50];
    bf1[20] = input[10];
    bf1[21] = input[42];
    bf1[22] = input[26];
    bf1[23] = input[58];
    bf1[24] = input[6];
    bf1[25] = input[38];
    bf1[26] = input[22];
    bf1[27] = input[54];
    bf1[28] = input[14];
    bf1[29] = input[46];
    bf1[30] = input[30];
    bf1[31] = input[62];
    bf1[32] = input[1];
    bf1[33] = input[33];
    bf1[34] = input[17];
    bf1[35] = input[49];
    bf1[36] = input[9];
    bf1[37] = input[41];
    bf1[38] = input[25];
    bf1[39] = input[57];
    bf1[40] = input[5];
    bf1[41] = input[37];
    bf1[42] = input[21];
    bf1[43] = input[53];
    bf1[44] = input[13];
    bf1[45] = input[45];
    bf1[46] = input[29];
    bf1[47] = input[61];
    bf1[48] = input[3];
    bf1[49] = input[35];
    bf1[50] = input[19];
    bf1[51] = input[51];
    bf1[52] = input[11];
    bf1[53] = input[43];
    bf1[54] = input[27];
    bf1[55] = input[59];
    bf1[56] = input[7];
    bf1[57] = input[39];
    bf1[58] = input[23];
    bf1[59] = input[55];
    bf1[60] = input[15];
    bf1[61] = input[47];
    bf1[62] = input[31];
    bf1[63] = input[63];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 2 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = bf0[21];
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = bf0[26];
    bf1[27] = bf0[27];
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    bf1[32] = half_btf(cospi[63], bf0[32], -cospi[1], bf0[63], cos_bit);
    bf1[33] = half_btf(cospi[31], bf0[33], -cospi[33], bf0[62], cos_bit);
    bf1[34] = half_btf(cospi[47], bf0[34], -cospi[17], bf0[61], cos_bit);
    bf1[35] = half_btf(cospi[15], bf0[35], -cospi[49], bf0[60], cos_bit);
    bf1[36] = half_btf(cospi[55], bf0[36], -cospi[9], bf0[59], cos_bit);
    bf1[37] = half_btf(cospi[23], bf0[37], -cospi[41], bf0[58], cos_bit);
    bf1[38] = half_btf(cospi[39], bf0[38], -cospi[25], bf0[57], cos_bit);
    bf1[39] = half_btf(cospi[7], bf0[39], -cospi[57], bf0[56], cos_bit);
    bf1[40] = half_btf(cospi[59], bf0[40], -cospi[5], bf0[55], cos_bit);
    bf1[41] = half_btf(cospi[27], bf0[41], -cospi[37], bf0[54], cos_bit);
    bf1[42] = half_btf(cospi[43], bf0[42], -cospi[21], bf0[53], cos_bit);
    bf1[43] = half_btf(cospi[11], bf0[43], -cospi[53], bf0[52], cos_bit);
    bf1[44] = half_btf(cospi[51], bf0[44], -cospi[13], bf0[51], cos_bit);
    bf1[45] = half_btf(cospi[19], bf0[45], -cospi[45], bf0[50], cos_bit);
    bf1[46] = half_btf(cospi[35], bf0[46], -cospi[29], bf0[49], cos_bit);
    bf1[47] = half_btf(cospi[3], bf0[47], -cospi[61], bf0[48], cos_bit);
    bf1[48] = half_btf(cospi[61], bf0[47], cospi[3], bf0[48], cos_bit);
    bf1[49] = half_btf(cospi[29], bf0[46], cospi[35], bf0[49], cos_bit);
    bf1[50] = half_btf(cospi[45], bf0[45], cospi[19], bf0[50], cos_bit);
    bf1[51] = half_btf(cospi[13], bf0[44], cospi[51], bf0[51], cos_bit);
    bf1[52] = half_btf(cospi[53], bf0[43], cospi[11], bf0[52], cos_bit);
    bf1[53] = half_btf(cospi[21], bf0[42], cospi[43], bf0[53], cos_bit);
    bf1[54] = half_btf(cospi[37], bf0[41], cospi[27], bf0[54], cos_bit);
    bf1[55] = half_btf(cospi[5], bf0[40], cospi[59], bf0[55], cos_bit);
    bf1[56] = half_btf(cospi[57], bf0[39], cospi[7], bf0[56], cos_bit);
    bf1[57] = half_btf(cospi[25], bf0[38], cospi[39], bf0[57], cos_bit);
    bf1[58] = half_btf(cospi[41], bf0[37], cospi[23], bf0[58], cos_bit);
    bf1[59] = half_btf(cospi[9], bf0[36], cospi[55], bf0[59], cos_bit);
    bf1[60] = half_btf(cospi[49], bf0[35], cospi[15], bf0[60], cos_bit);
    bf1[61] = half_btf(cospi[17], bf0[34], cospi[47], bf0[61], cos_bit);
    bf1[62] = half_btf(cospi[33], bf0[33], cospi[31], bf0[62], cos_bit);
    bf1[63] = half_btf(cospi[1], bf0[32], cospi[63], bf0[63], cos_bit);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 3 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = bf0[10];
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = bf0[13];
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = half_btf(cospi[62], bf0[16], -cospi[2], bf0[31], cos_bit);
    bf1[17] = half_btf(cospi[30], bf0[17], -cospi[34], bf0[30], cos_bit);
    bf1[18] = half_btf(cospi[46], bf0[18], -cospi[18], bf0[29], cos_bit);
    bf1[19] = half_btf(cospi[14], bf0[19], -cospi[50], bf0[28], cos_bit);
    bf1[20] = half_btf(cospi[54], bf0[20], -cospi[10], bf0[27], cos_bit);
    bf1[21] = half_btf(cospi[22], bf0[21], -cospi[42], bf0[26], cos_bit);
    bf1[22] = half_btf(cospi[38], bf0[22], -cospi[26], bf0[25], cos_bit);
    bf1[23] = half_btf(cospi[6], bf0[23], -cospi[58], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[58], bf0[23], cospi[6], bf0[24], cos_bit);
    bf1[25] = half_btf(cospi[26], bf0[22], cospi[38], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[42], bf0[21], cospi[22], bf0[26], cos_bit);
    bf1[27] = half_btf(cospi[10], bf0[20], cospi[54], bf0[27], cos_bit);
    bf1[28] = half_btf(cospi[50], bf0[19], cospi[14], bf0[28], cos_bit);
    bf1[29] = half_btf(cospi[18], bf0[18], cospi[46], bf0[29], cos_bit);
    bf1[30] = half_btf(cospi[34], bf0[17], cospi[30], bf0[30], cos_bit);
    bf1[31] = half_btf(cospi[2], bf0[16], cospi[62], bf0[31], cos_bit);
    bf1[32] = clamp_value(bf0[32] + bf0[33], stage_range[stage]);
    bf1[33] = clamp_value(bf0[32] - bf0[33], stage_range[stage]);
    bf1[34] = clamp_value(-bf0[34] + bf0[35], stage_range[stage]);
    bf1[35] = clamp_value(bf0[34] + bf0[35], stage_range[stage]);
    bf1[36] = clamp_value(bf0[36] + bf0[37], stage_range[stage]);
    bf1[37] = clamp_value(bf0[36] - bf0[37], stage_range[stage]);
    bf1[38] = clamp_value(-bf0[38] + bf0[39], stage_range[stage]);
    bf1[39] = clamp_value(bf0[38] + bf0[39], stage_range[stage]);
    bf1[40] = clamp_value(bf0[40] + bf0[41], stage_range[stage]);
    bf1[41] = clamp_value(bf0[40] - bf0[41], stage_range[stage]);
    bf1[42] = clamp_value(-bf0[42] + bf0[43], stage_range[stage]);
    bf1[43] = clamp_value(bf0[42] + bf0[43], stage_range[stage]);
    bf1[44] = clamp_value(bf0[44] + bf0[45], stage_range[stage]);
    bf1[45] = clamp_value(bf0[44] - bf0[45], stage_range[stage]);
    bf1[46] = clamp_value(-bf0[46] + bf0[47], stage_range[stage]);
    bf1[47] = clamp_value(bf0[46] + bf0[47], stage_range[stage]);
    bf1[48] = clamp_value(bf0[48] + bf0[49], stage_range[stage]);
    bf1[49] = clamp_value(bf0[48] - bf0[49], stage_range[stage]);
    bf1[50] = clamp_value(-bf0[50] + bf0[51], stage_range[stage]);
    bf1[51] = clamp_value(bf0[50] + bf0[51], stage_range[stage]);
    bf1[52] = clamp_value(bf0[52] + bf0[53], stage_range[stage]);
    bf1[53] = clamp_value(bf0[52] - bf0[53], stage_range[stage]);
    bf1[54] = clamp_value(-bf0[54] + bf0[55], stage_range[stage]);
    bf1[55] = clamp_value(bf0[54] + bf0[55], stage_range[stage]);
    bf1[56] = clamp_value(bf0[56] + bf0[57], stage_range[stage]);
    bf1[57] = clamp_value(bf0[56] - bf0[57], stage_range[stage]);
    bf1[58] = clamp_value(-bf0[58] + bf0[59], stage_range[stage]);
    bf1[59] = clamp_value(bf0[58] + bf0[59], stage_range[stage]);
    bf1[60] = clamp_value(bf0[60] + bf0[61], stage_range[stage]);
    bf1[61] = clamp_value(bf0[60] - bf0[61], stage_range[stage]);
    bf1[62] = clamp_value(-bf0[62] + bf0[63], stage_range[stage]);
    bf1[63] = clamp_value(bf0[62] + bf0[63], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 4 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = bf0[4];
    bf1[5]  = bf0[5];
    bf1[6]  = bf0[6];
    bf1[7]  = bf0[7];
    bf1[8]  = half_btf(cospi[60], bf0[8], -cospi[4], bf0[15], cos_bit);
    bf1[9]  = half_btf(cospi[28], bf0[9], -cospi[36], bf0[14], cos_bit);
    bf1[10] = half_btf(cospi[44], bf0[10], -cospi[20], bf0[13], cos_bit);
    bf1[11] = half_btf(cospi[12], bf0[11], -cospi[52], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[52], bf0[11], cospi[12], bf0[12], cos_bit);
    bf1[13] = half_btf(cospi[20], bf0[10], cospi[44], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[36], bf0[9], cospi[28], bf0[14], cos_bit);
    bf1[15] = half_btf(cospi[4], bf0[8], cospi[60], bf0[15], cos_bit);
    bf1[16] = clamp_value(bf0[16] + bf0[17], stage_range[stage]);
    bf1[17] = clamp_value(bf0[16] - bf0[17], stage_range[stage]);
    bf1[18] = clamp_value(-bf0[18] + bf0[19], stage_range[stage]);
    bf1[19] = clamp_value(bf0[18] + bf0[19], stage_range[stage]);
    bf1[20] = clamp_value(bf0[20] + bf0[21], stage_range[stage]);
    bf1[21] = clamp_value(bf0[20] - bf0[21], stage_range[stage]);
    bf1[22] = clamp_value(-bf0[22] + bf0[23], stage_range[stage]);
    bf1[23] = clamp_value(bf0[22] + bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(bf0[24] + bf0[25], stage_range[stage]);
    bf1[25] = clamp_value(bf0[24] - bf0[25], stage_range[stage]);
    bf1[26] = clamp_value(-bf0[26] + bf0[27], stage_range[stage]);
    bf1[27] = clamp_value(bf0[26] + bf0[27], stage_range[stage]);
    bf1[28] = clamp_value(bf0[28] + bf0[29], stage_range[stage]);
    bf1[29] = clamp_value(bf0[28] - bf0[29], stage_range[stage]);
    bf1[30] = clamp_value(-bf0[30] + bf0[31], stage_range[stage]);
    bf1[31] = clamp_value(bf0[30] + bf0[31], stage_range[stage]);
    bf1[32] = bf0[32];
    bf1[33] = half_btf(-cospi[4], bf0[33], cospi[60], bf0[62], cos_bit);
    bf1[34] = half_btf(-cospi[60], bf0[34], -cospi[4], bf0[61], cos_bit);
    bf1[35] = bf0[35];
    bf1[36] = bf0[36];
    bf1[37] = half_btf(-cospi[36], bf0[37], cospi[28], bf0[58], cos_bit);
    bf1[38] = half_btf(-cospi[28], bf0[38], -cospi[36], bf0[57], cos_bit);
    bf1[39] = bf0[39];
    bf1[40] = bf0[40];
    bf1[41] = half_btf(-cospi[20], bf0[41], cospi[44], bf0[54], cos_bit);
    bf1[42] = half_btf(-cospi[44], bf0[42], -cospi[20], bf0[53], cos_bit);
    bf1[43] = bf0[43];
    bf1[44] = bf0[44];
    bf1[45] = half_btf(-cospi[52], bf0[45], cospi[12], bf0[50], cos_bit);
    bf1[46] = half_btf(-cospi[12], bf0[46], -cospi[52], bf0[49], cos_bit);
    bf1[47] = bf0[47];
    bf1[48] = bf0[48];
    bf1[49] = half_btf(-cospi[52], bf0[46], cospi[12], bf0[49], cos_bit);
    bf1[50] = half_btf(cospi[12], bf0[45], cospi[52], bf0[50], cos_bit);
    bf1[51] = bf0[51];
    bf1[52] = bf0[52];
    bf1[53] = half_btf(-cospi[20], bf0[42], cospi[44], bf0[53], cos_bit);
    bf1[54] = half_btf(cospi[44], bf0[41], cospi[20], bf0[54], cos_bit);
    bf1[55] = bf0[55];
    bf1[56] = bf0[56];
    bf1[57] = half_btf(-cospi[36], bf0[38], cospi[28], bf0[57], cos_bit);
    bf1[58] = half_btf(cospi[28], bf0[37], cospi[36], bf0[58], cos_bit);
    bf1[59] = bf0[59];
    bf1[60] = bf0[60];
    bf1[61] = half_btf(-cospi[4], bf0[34], cospi[60], bf0[61], cos_bit);
    bf1[62] = half_btf(cospi[60], bf0[33], cospi[4], bf0[62], cos_bit);
    bf1[63] = bf0[63];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 5 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = bf0[0];
    bf1[1]  = bf0[1];
    bf1[2]  = bf0[2];
    bf1[3]  = bf0[3];
    bf1[4]  = half_btf(cospi[56], bf0[4], -cospi[8], bf0[7], cos_bit);
    bf1[5]  = half_btf(cospi[24], bf0[5], -cospi[40], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[40], bf0[5], cospi[24], bf0[6], cos_bit);
    bf1[7]  = half_btf(cospi[8], bf0[4], cospi[56], bf0[7], cos_bit);
    bf1[8]  = clamp_value(bf0[8] + bf0[9], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[8] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(-bf0[10] + bf0[11], stage_range[stage]);
    bf1[11] = clamp_value(bf0[10] + bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[13], stage_range[stage]);
    bf1[13] = clamp_value(bf0[12] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(-bf0[14] + bf0[15], stage_range[stage]);
    bf1[15] = clamp_value(bf0[14] + bf0[15], stage_range[stage]);
    bf1[16] = bf0[16];
    bf1[17] = half_btf(-cospi[8], bf0[17], cospi[56], bf0[30], cos_bit);
    bf1[18] = half_btf(-cospi[56], bf0[18], -cospi[8], bf0[29], cos_bit);
    bf1[19] = bf0[19];
    bf1[20] = bf0[20];
    bf1[21] = half_btf(-cospi[40], bf0[21], cospi[24], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[24], bf0[22], -cospi[40], bf0[25], cos_bit);
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = half_btf(-cospi[40], bf0[22], cospi[24], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[24], bf0[21], cospi[40], bf0[26], cos_bit);
    bf1[27] = bf0[27];
    bf1[28] = bf0[28];
    bf1[29] = half_btf(-cospi[8], bf0[18], cospi[56], bf0[29], cos_bit);
    bf1[30] = half_btf(cospi[56], bf0[17], cospi[8], bf0[30], cos_bit);
    bf1[31] = bf0[31];
    bf1[32] = clamp_value(bf0[32] + bf0[35], stage_range[stage]);
    bf1[33] = clamp_value(bf0[33] + bf0[34], stage_range[stage]);
    bf1[34] = clamp_value(bf0[33] - bf0[34], stage_range[stage]);
    bf1[35] = clamp_value(bf0[32] - bf0[35], stage_range[stage]);
    bf1[36] = clamp_value(-bf0[36] + bf0[39], stage_range[stage]);
    bf1[37] = clamp_value(-bf0[37] + bf0[38], stage_range[stage]);
    bf1[38] = clamp_value(bf0[37] + bf0[38], stage_range[stage]);
    bf1[39] = clamp_value(bf0[36] + bf0[39], stage_range[stage]);
    bf1[40] = clamp_value(bf0[40] + bf0[43], stage_range[stage]);
    bf1[41] = clamp_value(bf0[41] + bf0[42], stage_range[stage]);
    bf1[42] = clamp_value(bf0[41] - bf0[42], stage_range[stage]);
    bf1[43] = clamp_value(bf0[40] - bf0[43], stage_range[stage]);
    bf1[44] = clamp_value(-bf0[44] + bf0[47], stage_range[stage]);
    bf1[45] = clamp_value(-bf0[45] + bf0[46], stage_range[stage]);
    bf1[46] = clamp_value(bf0[45] + bf0[46], stage_range[stage]);
    bf1[47] = clamp_value(bf0[44] + bf0[47], stage_range[stage]);
    bf1[48] = clamp_value(bf0[48] + bf0[51], stage_range[stage]);
    bf1[49] = clamp_value(bf0[49] + bf0[50], stage_range[stage]);
    bf1[50] = clamp_value(bf0[49] - bf0[50], stage_range[stage]);
    bf1[51] = clamp_value(bf0[48] - bf0[51], stage_range[stage]);
    bf1[52] = clamp_value(-bf0[52] + bf0[55], stage_range[stage]);
    bf1[53] = clamp_value(-bf0[53] + bf0[54], stage_range[stage]);
    bf1[54] = clamp_value(bf0[53] + bf0[54], stage_range[stage]);
    bf1[55] = clamp_value(bf0[52] + bf0[55], stage_range[stage]);
    bf1[56] = clamp_value(bf0[56] + bf0[59], stage_range[stage]);
    bf1[57] = clamp_value(bf0[57] + bf0[58], stage_range[stage]);
    bf1[58] = clamp_value(bf0[57] - bf0[58], stage_range[stage]);
    bf1[59] = clamp_value(bf0[56] - bf0[59], stage_range[stage]);
    bf1[60] = clamp_value(-bf0[60] + bf0[63], stage_range[stage]);
    bf1[61] = clamp_value(-bf0[61] + bf0[62], stage_range[stage]);
    bf1[62] = clamp_value(bf0[61] + bf0[62], stage_range[stage]);
    bf1[63] = clamp_value(bf0[60] + bf0[63], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 6 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = half_btf(cospi[32], bf0[0], cospi[32], bf0[1], cos_bit);
    bf1[1]  = half_btf(cospi[32], bf0[0], -cospi[32], bf0[1], cos_bit);
    bf1[2]  = half_btf(cospi[48], bf0[2], -cospi[16], bf0[3], cos_bit);
    bf1[3]  = half_btf(cospi[16], bf0[2], cospi[48], bf0[3], cos_bit);
    bf1[4]  = clamp_value(bf0[4] + bf0[5], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[4] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(-bf0[6] + bf0[7], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[6] + bf0[7], stage_range[stage]);
    bf1[8]  = bf0[8];
    bf1[9]  = half_btf(-cospi[16], bf0[9], cospi[48], bf0[14], cos_bit);
    bf1[10] = half_btf(-cospi[48], bf0[10], -cospi[16], bf0[13], cos_bit);
    bf1[11] = bf0[11];
    bf1[12] = bf0[12];
    bf1[13] = half_btf(-cospi[16], bf0[10], cospi[48], bf0[13], cos_bit);
    bf1[14] = half_btf(cospi[48], bf0[9], cospi[16], bf0[14], cos_bit);
    bf1[15] = bf0[15];
    bf1[16] = clamp_value(bf0[16] + bf0[19], stage_range[stage]);
    bf1[17] = clamp_value(bf0[17] + bf0[18], stage_range[stage]);
    bf1[18] = clamp_value(bf0[17] - bf0[18], stage_range[stage]);
    bf1[19] = clamp_value(bf0[16] - bf0[19], stage_range[stage]);
    bf1[20] = clamp_value(-bf0[20] + bf0[23], stage_range[stage]);
    bf1[21] = clamp_value(-bf0[21] + bf0[22], stage_range[stage]);
    bf1[22] = clamp_value(bf0[21] + bf0[22], stage_range[stage]);
    bf1[23] = clamp_value(bf0[20] + bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(bf0[24] + bf0[27], stage_range[stage]);
    bf1[25] = clamp_value(bf0[25] + bf0[26], stage_range[stage]);
    bf1[26] = clamp_value(bf0[25] - bf0[26], stage_range[stage]);
    bf1[27] = clamp_value(bf0[24] - bf0[27], stage_range[stage]);
    bf1[28] = clamp_value(-bf0[28] + bf0[31], stage_range[stage]);
    bf1[29] = clamp_value(-bf0[29] + bf0[30], stage_range[stage]);
    bf1[30] = clamp_value(bf0[29] + bf0[30], stage_range[stage]);
    bf1[31] = clamp_value(bf0[28] + bf0[31], stage_range[stage]);
    bf1[32] = bf0[32];
    bf1[33] = bf0[33];
    bf1[34] = half_btf(-cospi[8], bf0[34], cospi[56], bf0[61], cos_bit);
    bf1[35] = half_btf(-cospi[8], bf0[35], cospi[56], bf0[60], cos_bit);
    bf1[36] = half_btf(-cospi[56], bf0[36], -cospi[8], bf0[59], cos_bit);
    bf1[37] = half_btf(-cospi[56], bf0[37], -cospi[8], bf0[58], cos_bit);
    bf1[38] = bf0[38];
    bf1[39] = bf0[39];
    bf1[40] = bf0[40];
    bf1[41] = bf0[41];
    bf1[42] = half_btf(-cospi[40], bf0[42], cospi[24], bf0[53], cos_bit);
    bf1[43] = half_btf(-cospi[40], bf0[43], cospi[24], bf0[52], cos_bit);
    bf1[44] = half_btf(-cospi[24], bf0[44], -cospi[40], bf0[51], cos_bit);
    bf1[45] = half_btf(-cospi[24], bf0[45], -cospi[40], bf0[50], cos_bit);
    bf1[46] = bf0[46];
    bf1[47] = bf0[47];
    bf1[48] = bf0[48];
    bf1[49] = bf0[49];
    bf1[50] = half_btf(-cospi[40], bf0[45], cospi[24], bf0[50], cos_bit);
    bf1[51] = half_btf(-cospi[40], bf0[44], cospi[24], bf0[51], cos_bit);
    bf1[52] = half_btf(cospi[24], bf0[43], cospi[40], bf0[52], cos_bit);
    bf1[53] = half_btf(cospi[24], bf0[42], cospi[40], bf0[53], cos_bit);
    bf1[54] = bf0[54];
    bf1[55] = bf0[55];
    bf1[56] = bf0[56];
    bf1[57] = bf0[57];
    bf1[58] = half_btf(-cospi[8], bf0[37], cospi[56], bf0[58], cos_bit);
    bf1[59] = half_btf(-cospi[8], bf0[36], cospi[56], bf0[59], cos_bit);
    bf1[60] = half_btf(cospi[56], bf0[35], cospi[8], bf0[60], cos_bit);
    bf1[61] = half_btf(cospi[56], bf0[34], cospi[8], bf0[61], cos_bit);
    bf1[62] = bf0[62];
    bf1[63] = bf0[63];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 7 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[3], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[2], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[1] - bf0[2], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[0] - bf0[3], stage_range[stage]);
    bf1[4]  = bf0[4];
    bf1[5]  = half_btf(-cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[6]  = half_btf(cospi[32], bf0[5], cospi[32], bf0[6], cos_bit);
    bf1[7]  = bf0[7];
    bf1[8]  = clamp_value(bf0[8] + bf0[11], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[10], stage_range[stage]);
    bf1[10] = clamp_value(bf0[9] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[8] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(-bf0[12] + bf0[15], stage_range[stage]);
    bf1[13] = clamp_value(-bf0[13] + bf0[14], stage_range[stage]);
    bf1[14] = clamp_value(bf0[13] + bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[12] + bf0[15], stage_range[stage]);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = half_btf(-cospi[16], bf0[18], cospi[48], bf0[29], cos_bit);
    bf1[19] = half_btf(-cospi[16], bf0[19], cospi[48], bf0[28], cos_bit);
    bf1[20] = half_btf(-cospi[48], bf0[20], -cospi[16], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[48], bf0[21], -cospi[16], bf0[26], cos_bit);
    bf1[22] = bf0[22];
    bf1[23] = bf0[23];
    bf1[24] = bf0[24];
    bf1[25] = bf0[25];
    bf1[26] = half_btf(-cospi[16], bf0[21], cospi[48], bf0[26], cos_bit);
    bf1[27] = half_btf(-cospi[16], bf0[20], cospi[48], bf0[27], cos_bit);
    bf1[28] = half_btf(cospi[48], bf0[19], cospi[16], bf0[28], cos_bit);
    bf1[29] = half_btf(cospi[48], bf0[18], cospi[16], bf0[29], cos_bit);
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    bf1[32] = clamp_value(bf0[32] + bf0[39], stage_range[stage]);
    bf1[33] = clamp_value(bf0[33] + bf0[38], stage_range[stage]);
    bf1[34] = clamp_value(bf0[34] + bf0[37], stage_range[stage]);
    bf1[35] = clamp_value(bf0[35] + bf0[36], stage_range[stage]);
    bf1[36] = clamp_value(bf0[35] - bf0[36], stage_range[stage]);
    bf1[37] = clamp_value(bf0[34] - bf0[37], stage_range[stage]);
    bf1[38] = clamp_value(bf0[33] - bf0[38], stage_range[stage]);
    bf1[39] = clamp_value(bf0[32] - bf0[39], stage_range[stage]);
    bf1[40] = clamp_value(-bf0[40] + bf0[47], stage_range[stage]);
    bf1[41] = clamp_value(-bf0[41] + bf0[46], stage_range[stage]);
    bf1[42] = clamp_value(-bf0[42] + bf0[45], stage_range[stage]);
    bf1[43] = clamp_value(-bf0[43] + bf0[44], stage_range[stage]);
    bf1[44] = clamp_value(bf0[43] + bf0[44], stage_range[stage]);
    bf1[45] = clamp_value(bf0[42] + bf0[45], stage_range[stage]);
    bf1[46] = clamp_value(bf0[41] + bf0[46], stage_range[stage]);
    bf1[47] = clamp_value(bf0[40] + bf0[47], stage_range[stage]);
    bf1[48] = clamp_value(bf0[48] + bf0[55], stage_range[stage]);
    bf1[49] = clamp_value(bf0[49] + bf0[54], stage_range[stage]);
    bf1[50] = clamp_value(bf0[50] + bf0[53], stage_range[stage]);
    bf1[51] = clamp_value(bf0[51] + bf0[52], stage_range[stage]);
    bf1[52] = clamp_value(bf0[51] - bf0[52], stage_range[stage]);
    bf1[53] = clamp_value(bf0[50] - bf0[53], stage_range[stage]);
    bf1[54] = clamp_value(bf0[49] - bf0[54], stage_range[stage]);
    bf1[55] = clamp_value(bf0[48] - bf0[55], stage_range[stage]);
    bf1[56] = clamp_value(-bf0[56] + bf0[63], stage_range[stage]);
    bf1[57] = clamp_value(-bf0[57] + bf0[62], stage_range[stage]);
    bf1[58] = clamp_value(-bf0[58] + bf0[61], stage_range[stage]);
    bf1[59] = clamp_value(-bf0[59] + bf0[60], stage_range[stage]);
    bf1[60] = clamp_value(bf0[59] + bf0[60], stage_range[stage]);
    bf1[61] = clamp_value(bf0[58] + bf0[61], stage_range[stage]);
    bf1[62] = clamp_value(bf0[57] + bf0[62], stage_range[stage]);
    bf1[63] = clamp_value(bf0[56] + bf0[63], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 8 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = clamp_value(bf0[0] + bf0[7], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[6], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[5], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[4], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[3] - bf0[4], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[2] - bf0[5], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[1] - bf0[6], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[0] - bf0[7], stage_range[stage]);
    bf1[8]  = bf0[8];
    bf1[9]  = bf0[9];
    bf1[10] = half_btf(-cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[11] = half_btf(-cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[12] = half_btf(cospi[32], bf0[11], cospi[32], bf0[12], cos_bit);
    bf1[13] = half_btf(cospi[32], bf0[10], cospi[32], bf0[13], cos_bit);
    bf1[14] = bf0[14];
    bf1[15] = bf0[15];
    bf1[16] = clamp_value(bf0[16] + bf0[23], stage_range[stage]);
    bf1[17] = clamp_value(bf0[17] + bf0[22], stage_range[stage]);
    bf1[18] = clamp_value(bf0[18] + bf0[21], stage_range[stage]);
    bf1[19] = clamp_value(bf0[19] + bf0[20], stage_range[stage]);
    bf1[20] = clamp_value(bf0[19] - bf0[20], stage_range[stage]);
    bf1[21] = clamp_value(bf0[18] - bf0[21], stage_range[stage]);
    bf1[22] = clamp_value(bf0[17] - bf0[22], stage_range[stage]);
    bf1[23] = clamp_value(bf0[16] - bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(-bf0[24] + bf0[31], stage_range[stage]);
    bf1[25] = clamp_value(-bf0[25] + bf0[30], stage_range[stage]);
    bf1[26] = clamp_value(-bf0[26] + bf0[29], stage_range[stage]);
    bf1[27] = clamp_value(-bf0[27] + bf0[28], stage_range[stage]);
    bf1[28] = clamp_value(bf0[27] + bf0[28], stage_range[stage]);
    bf1[29] = clamp_value(bf0[26] + bf0[29], stage_range[stage]);
    bf1[30] = clamp_value(bf0[25] + bf0[30], stage_range[stage]);
    bf1[31] = clamp_value(bf0[24] + bf0[31], stage_range[stage]);
    bf1[32] = bf0[32];
    bf1[33] = bf0[33];
    bf1[34] = bf0[34];
    bf1[35] = bf0[35];
    bf1[36] = half_btf(-cospi[16], bf0[36], cospi[48], bf0[59], cos_bit);
    bf1[37] = half_btf(-cospi[16], bf0[37], cospi[48], bf0[58], cos_bit);
    bf1[38] = half_btf(-cospi[16], bf0[38], cospi[48], bf0[57], cos_bit);
    bf1[39] = half_btf(-cospi[16], bf0[39], cospi[48], bf0[56], cos_bit);
    bf1[40] = half_btf(-cospi[48], bf0[40], -cospi[16], bf0[55], cos_bit);
    bf1[41] = half_btf(-cospi[48], bf0[41], -cospi[16], bf0[54], cos_bit);
    bf1[42] = half_btf(-cospi[48], bf0[42], -cospi[16], bf0[53], cos_bit);
    bf1[43] = half_btf(-cospi[48], bf0[43], -cospi[16], bf0[52], cos_bit);
    bf1[44] = bf0[44];
    bf1[45] = bf0[45];
    bf1[46] = bf0[46];
    bf1[47] = bf0[47];
    bf1[48] = bf0[48];
    bf1[49] = bf0[49];
    bf1[50] = bf0[50];
    bf1[51] = bf0[51];
    bf1[52] = half_btf(-cospi[16], bf0[43], cospi[48], bf0[52], cos_bit);
    bf1[53] = half_btf(-cospi[16], bf0[42], cospi[48], bf0[53], cos_bit);
    bf1[54] = half_btf(-cospi[16], bf0[41], cospi[48], bf0[54], cos_bit);
    bf1[55] = half_btf(-cospi[16], bf0[40], cospi[48], bf0[55], cos_bit);
    bf1[56] = half_btf(cospi[48], bf0[39], cospi[16], bf0[56], cos_bit);
    bf1[57] = half_btf(cospi[48], bf0[38], cospi[16], bf0[57], cos_bit);
    bf1[58] = half_btf(cospi[48], bf0[37], cospi[16], bf0[58], cos_bit);
    bf1[59] = half_btf(cospi[48], bf0[36], cospi[16], bf0[59], cos_bit);
    bf1[60] = bf0[60];
    bf1[61] = bf0[61];
    bf1[62] = bf0[62];
    bf1[63] = bf0[63];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 9 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[15], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[14], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[13], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[12], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[11], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[10], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[9], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[8], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[7] - bf0[8], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[6] - bf0[9], stage_range[stage]);
    bf1[10] = clamp_value(bf0[5] - bf0[10], stage_range[stage]);
    bf1[11] = clamp_value(bf0[4] - bf0[11], stage_range[stage]);
    bf1[12] = clamp_value(bf0[3] - bf0[12], stage_range[stage]);
    bf1[13] = clamp_value(bf0[2] - bf0[13], stage_range[stage]);
    bf1[14] = clamp_value(bf0[1] - bf0[14], stage_range[stage]);
    bf1[15] = clamp_value(bf0[0] - bf0[15], stage_range[stage]);
    bf1[16] = bf0[16];
    bf1[17] = bf0[17];
    bf1[18] = bf0[18];
    bf1[19] = bf0[19];
    bf1[20] = half_btf(-cospi[32], bf0[20], cospi[32], bf0[27], cos_bit);
    bf1[21] = half_btf(-cospi[32], bf0[21], cospi[32], bf0[26], cos_bit);
    bf1[22] = half_btf(-cospi[32], bf0[22], cospi[32], bf0[25], cos_bit);
    bf1[23] = half_btf(-cospi[32], bf0[23], cospi[32], bf0[24], cos_bit);
    bf1[24] = half_btf(cospi[32], bf0[23], cospi[32], bf0[24], cos_bit);
    bf1[25] = half_btf(cospi[32], bf0[22], cospi[32], bf0[25], cos_bit);
    bf1[26] = half_btf(cospi[32], bf0[21], cospi[32], bf0[26], cos_bit);
    bf1[27] = half_btf(cospi[32], bf0[20], cospi[32], bf0[27], cos_bit);
    bf1[28] = bf0[28];
    bf1[29] = bf0[29];
    bf1[30] = bf0[30];
    bf1[31] = bf0[31];
    bf1[32] = clamp_value(bf0[32] + bf0[47], stage_range[stage]);
    bf1[33] = clamp_value(bf0[33] + bf0[46], stage_range[stage]);
    bf1[34] = clamp_value(bf0[34] + bf0[45], stage_range[stage]);
    bf1[35] = clamp_value(bf0[35] + bf0[44], stage_range[stage]);
    bf1[36] = clamp_value(bf0[36] + bf0[43], stage_range[stage]);
    bf1[37] = clamp_value(bf0[37] + bf0[42], stage_range[stage]);
    bf1[38] = clamp_value(bf0[38] + bf0[41], stage_range[stage]);
    bf1[39] = clamp_value(bf0[39] + bf0[40], stage_range[stage]);
    bf1[40] = clamp_value(bf0[39] - bf0[40], stage_range[stage]);
    bf1[41] = clamp_value(bf0[38] - bf0[41], stage_range[stage]);
    bf1[42] = clamp_value(bf0[37] - bf0[42], stage_range[stage]);
    bf1[43] = clamp_value(bf0[36] - bf0[43], stage_range[stage]);
    bf1[44] = clamp_value(bf0[35] - bf0[44], stage_range[stage]);
    bf1[45] = clamp_value(bf0[34] - bf0[45], stage_range[stage]);
    bf1[46] = clamp_value(bf0[33] - bf0[46], stage_range[stage]);
    bf1[47] = clamp_value(bf0[32] - bf0[47], stage_range[stage]);
    bf1[48] = clamp_value(-bf0[48] + bf0[63], stage_range[stage]);
    bf1[49] = clamp_value(-bf0[49] + bf0[62], stage_range[stage]);
    bf1[50] = clamp_value(-bf0[50] + bf0[61], stage_range[stage]);
    bf1[51] = clamp_value(-bf0[51] + bf0[60], stage_range[stage]);
    bf1[52] = clamp_value(-bf0[52] + bf0[59], stage_range[stage]);
    bf1[53] = clamp_value(-bf0[53] + bf0[58], stage_range[stage]);
    bf1[54] = clamp_value(-bf0[54] + bf0[57], stage_range[stage]);
    bf1[55] = clamp_value(-bf0[55] + bf0[56], stage_range[stage]);
    bf1[56] = clamp_value(bf0[55] + bf0[56], stage_range[stage]);
    bf1[57] = clamp_value(bf0[54] + bf0[57], stage_range[stage]);
    bf1[58] = clamp_value(bf0[53] + bf0[58], stage_range[stage]);
    bf1[59] = clamp_value(bf0[52] + bf0[59], stage_range[stage]);
    bf1[60] = clamp_value(bf0[51] + bf0[60], stage_range[stage]);
    bf1[61] = clamp_value(bf0[50] + bf0[61], stage_range[stage]);
    bf1[62] = clamp_value(bf0[49] + bf0[62], stage_range[stage]);
    bf1[63] = clamp_value(bf0[48] + bf0[63], stage_range[stage]);
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 10 */
    stage++;
    bf0     = output;
    bf1     = step;
    bf1[0]  = clamp_value(bf0[0] + bf0[31], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[30], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[29], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[28], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[27], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[26], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[25], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[24], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[8] + bf0[23], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[22], stage_range[stage]);
    bf1[10] = clamp_value(bf0[10] + bf0[21], stage_range[stage]);
    bf1[11] = clamp_value(bf0[11] + bf0[20], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[19], stage_range[stage]);
    bf1[13] = clamp_value(bf0[13] + bf0[18], stage_range[stage]);
    bf1[14] = clamp_value(bf0[14] + bf0[17], stage_range[stage]);
    bf1[15] = clamp_value(bf0[15] + bf0[16], stage_range[stage]);
    bf1[16] = clamp_value(bf0[15] - bf0[16], stage_range[stage]);
    bf1[17] = clamp_value(bf0[14] - bf0[17], stage_range[stage]);
    bf1[18] = clamp_value(bf0[13] - bf0[18], stage_range[stage]);
    bf1[19] = clamp_value(bf0[12] - bf0[19], stage_range[stage]);
    bf1[20] = clamp_value(bf0[11] - bf0[20], stage_range[stage]);
    bf1[21] = clamp_value(bf0[10] - bf0[21], stage_range[stage]);
    bf1[22] = clamp_value(bf0[9] - bf0[22], stage_range[stage]);
    bf1[23] = clamp_value(bf0[8] - bf0[23], stage_range[stage]);
    bf1[24] = clamp_value(bf0[7] - bf0[24], stage_range[stage]);
    bf1[25] = clamp_value(bf0[6] - bf0[25], stage_range[stage]);
    bf1[26] = clamp_value(bf0[5] - bf0[26], stage_range[stage]);
    bf1[27] = clamp_value(bf0[4] - bf0[27], stage_range[stage]);
    bf1[28] = clamp_value(bf0[3] - bf0[28], stage_range[stage]);
    bf1[29] = clamp_value(bf0[2] - bf0[29], stage_range[stage]);
    bf1[30] = clamp_value(bf0[1] - bf0[30], stage_range[stage]);
    bf1[31] = clamp_value(bf0[0] - bf0[31], stage_range[stage]);
    bf1[32] = bf0[32];
    bf1[33] = bf0[33];
    bf1[34] = bf0[34];
    bf1[35] = bf0[35];
    bf1[36] = bf0[36];
    bf1[37] = bf0[37];
    bf1[38] = bf0[38];
    bf1[39] = bf0[39];
    bf1[40] = half_btf(-cospi[32], bf0[40], cospi[32], bf0[55], cos_bit);
    bf1[41] = half_btf(-cospi[32], bf0[41], cospi[32], bf0[54], cos_bit);
    bf1[42] = half_btf(-cospi[32], bf0[42], cospi[32], bf0[53], cos_bit);
    bf1[43] = half_btf(-cospi[32], bf0[43], cospi[32], bf0[52], cos_bit);
    bf1[44] = half_btf(-cospi[32], bf0[44], cospi[32], bf0[51], cos_bit);
    bf1[45] = half_btf(-cospi[32], bf0[45], cospi[32], bf0[50], cos_bit);
    bf1[46] = half_btf(-cospi[32], bf0[46], cospi[32], bf0[49], cos_bit);
    bf1[47] = half_btf(-cospi[32], bf0[47], cospi[32], bf0[48], cos_bit);
    bf1[48] = half_btf(cospi[32], bf0[47], cospi[32], bf0[48], cos_bit);
    bf1[49] = half_btf(cospi[32], bf0[46], cospi[32], bf0[49], cos_bit);
    bf1[50] = half_btf(cospi[32], bf0[45], cospi[32], bf0[50], cos_bit);
    bf1[51] = half_btf(cospi[32], bf0[44], cospi[32], bf0[51], cos_bit);
    bf1[52] = half_btf(cospi[32], bf0[43], cospi[32], bf0[52], cos_bit);
    bf1[53] = half_btf(cospi[32], bf0[42], cospi[32], bf0[53], cos_bit);
    bf1[54] = half_btf(cospi[32], bf0[41], cospi[32], bf0[54], cos_bit);
    bf1[55] = half_btf(cospi[32], bf0[40], cospi[32], bf0[55], cos_bit);
    bf1[56] = bf0[56];
    bf1[57] = bf0[57];
    bf1[58] = bf0[58];
    bf1[59] = bf0[59];
    bf1[60] = bf0[60];
    bf1[61] = bf0[61];
    bf1[62] = bf0[62];
    bf1[63] = bf0[63];
    //range_check_buf(stage, input, bf1, size, stage_range[stage]);

    /*!< stage 11 */
    stage++;
    bf0     = step;
    bf1     = output;
    bf1[0]  = clamp_value(bf0[0] + bf0[63], stage_range[stage]);
    bf1[1]  = clamp_value(bf0[1] + bf0[62], stage_range[stage]);
    bf1[2]  = clamp_value(bf0[2] + bf0[61], stage_range[stage]);
    bf1[3]  = clamp_value(bf0[3] + bf0[60], stage_range[stage]);
    bf1[4]  = clamp_value(bf0[4] + bf0[59], stage_range[stage]);
    bf1[5]  = clamp_value(bf0[5] + bf0[58], stage_range[stage]);
    bf1[6]  = clamp_value(bf0[6] + bf0[57], stage_range[stage]);
    bf1[7]  = clamp_value(bf0[7] + bf0[56], stage_range[stage]);
    bf1[8]  = clamp_value(bf0[8] + bf0[55], stage_range[stage]);
    bf1[9]  = clamp_value(bf0[9] + bf0[54], stage_range[stage]);
    bf1[10] = clamp_value(bf0[10] + bf0[53], stage_range[stage]);
    bf1[11] = clamp_value(bf0[11] + bf0[52], stage_range[stage]);
    bf1[12] = clamp_value(bf0[12] + bf0[51], stage_range[stage]);
    bf1[13] = clamp_value(bf0[13] + bf0[50], stage_range[stage]);
    bf1[14] = clamp_value(bf0[14] + bf0[49], stage_range[stage]);
    bf1[15] = clamp_value(bf0[15] + bf0[48], stage_range[stage]);
    bf1[16] = clamp_value(bf0[16] + bf0[47], stage_range[stage]);
    bf1[17] = clamp_value(bf0[17] + bf0[46], stage_range[stage]);
    bf1[18] = clamp_value(bf0[18] + bf0[45], stage_range[stage]);
    bf1[19] = clamp_value(bf0[19] + bf0[44], stage_range[stage]);
    bf1[20] = clamp_value(bf0[20] + bf0[43], stage_range[stage]);
    bf1[21] = clamp_value(bf0[21] + bf0[42], stage_range[stage]);
    bf1[22] = clamp_value(bf0[22] + bf0[41], stage_range[stage]);
    bf1[23] = clamp_value(bf0[23] + bf0[40], stage_range[stage]);
    bf1[24] = clamp_value(bf0[24] + bf0[39], stage_range[stage]);
    bf1[25] = clamp_value(bf0[25] + bf0[38], stage_range[stage]);
    bf1[26] = clamp_value(bf0[26] + bf0[37], stage_range[stage]);
    bf1[27] = clamp_value(bf0[27] + bf0[36], stage_range[stage]);
    bf1[28] = clamp_value(bf0[28] + bf0[35], stage_range[stage]);
    bf1[29] = clamp_value(bf0[29] + bf0[34], stage_range[stage]);
    bf1[30] = clamp_value(bf0[30] + bf0[33], stage_range[stage]);
    bf1[31] = clamp_value(bf0[31] + bf0[32], stage_range[stage]);
    bf1[32] = clamp_value(bf0[31] - bf0[32], stage_range[stage]);
    bf1[33] = clamp_value(bf0[30] - bf0[33], stage_range[stage]);
    bf1[34] = clamp_value(bf0[29] - bf0[34], stage_range[stage]);
    bf1[35] = clamp_value(bf0[28] - bf0[35], stage_range[stage]);
    bf1[36] = clamp_value(bf0[27] - bf0[36], stage_range[stage]);
    bf1[37] = clamp_value(bf0[26] - bf0[37], stage_range[stage]);
    bf1[38] = clamp_value(bf0[25] - bf0[38], stage_range[stage]);
    bf1[39] = clamp_value(bf0[24] - bf0[39], stage_range[stage]);
    bf1[40] = clamp_value(bf0[23] - bf0[40], stage_range[stage]);
    bf1[41] = clamp_value(bf0[22] - bf0[41], stage_range[stage]);
    bf1[42] = clamp_value(bf0[21] - bf0[42], stage_range[stage]);
    bf1[43] = clamp_value(bf0[20] - bf0[43], stage_range[stage]);
    bf1[44] = clamp_value(bf0[19] - bf0[44], stage_range[stage]);
    bf1[45] = clamp_value(bf0[18] - bf0[45], stage_range[stage]);
    bf1[46] = clamp_value(bf0[17] - bf0[46], stage_range[stage]);
    bf1[47] = clamp_value(bf0[16] - bf0[47], stage_range[stage]);
    bf1[48] = clamp_value(bf0[15] - bf0[48], stage_range[stage]);
    bf1[49] = clamp_value(bf0[14] - bf0[49], stage_range[stage]);
    bf1[50] = clamp_value(bf0[13] - bf0[50], stage_range[stage]);
    bf1[51] = clamp_value(bf0[12] - bf0[51], stage_range[stage]);
    bf1[52] = clamp_value(bf0[11] - bf0[52], stage_range[stage]);
    bf1[53] = clamp_value(bf0[10] - bf0[53], stage_range[stage]);
    bf1[54] = clamp_value(bf0[9] - bf0[54], stage_range[stage]);
    bf1[55] = clamp_value(bf0[8] - bf0[55], stage_range[stage]);
    bf1[56] = clamp_value(bf0[7] - bf0[56], stage_range[stage]);
    bf1[57] = clamp_value(bf0[6] - bf0[57], stage_range[stage]);
    bf1[58] = clamp_value(bf0[5] - bf0[58], stage_range[stage]);
    bf1[59] = clamp_value(bf0[4] - bf0[59], stage_range[stage]);
    bf1[60] = clamp_value(bf0[3] - bf0[60], stage_range[stage]);
    bf1[61] = clamp_value(bf0[2] - bf0[61], stage_range[stage]);
    bf1[62] = clamp_value(bf0[1] - bf0[62], stage_range[stage]);
    bf1[63] = clamp_value(bf0[0] - bf0[63], stage_range[stage]);
}
void eb_av1_iidentity4_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                         const int8_t *stage_range) {
    (void)cos_bit;
    (void)stage_range;
    for (int32_t i = 0; i < 4; ++i) {
        /*!< Normal input should fit into 32-bit. Cast to 64-bit here to avoid
         *   overflow with corrupted/fuzzed input. The same for av1_iidentity/16/64_c. */
        output[i] = round_shift((int64_t)new_sqrt2 * input[i], new_sqrt2_bits);
    }
    assert(stage_range[0] + new_sqrt2_bits <= 32);
}
void eb_av1_iidentity8_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                         const int8_t *stage_range) {
    (void)cos_bit;
    (void)stage_range;
    for (int32_t i = 0; i < 8; ++i) output[i] = (int32_t)((int64_t)input[i] * 2);
}
void eb_av1_iidentity16_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                          const int8_t *stage_range) {
    (void)cos_bit;
    (void)stage_range;
    for (int32_t i = 0; i < 16; ++i)
        output[i] = round_shift((int64_t)new_sqrt2 * 2 * input[i], new_sqrt2_bits);
    assert(stage_range[0] + new_sqrt2_bits <= 32);
}
void eb_av1_iidentity32_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                          const int8_t *stage_range) {
    (void)cos_bit;
    (void)stage_range;
    for (int32_t i = 0; i < 32; ++i) output[i] = (int32_t)((int64_t)input[i] * 4);
}
void av1_iidentity64_c(const int32_t *input, int32_t *output, int8_t cos_bit,
                       const int8_t *stage_range) {
    (void)cos_bit;
    (void)stage_range;
    for (int32_t i = 0; i < 64; ++i)
        output[i] = round_shift((int64_t)new_sqrt2 * 4 * input[i], new_sqrt2_bits);
    assert(stage_range[0] + new_sqrt2_bits <= 32);
}
static INLINE TxfmFunc inv_txfm_type_to_func(TxfmType TxfmType) {
    switch (TxfmType) {
    case TXFM_TYPE_DCT4: return eb_av1_idct4_new;
    case TXFM_TYPE_DCT8: return eb_av1_idct8_new;
    case TXFM_TYPE_DCT16: return eb_av1_idct16_new;
    case TXFM_TYPE_DCT32: return eb_av1_idct32_new;
    case TXFM_TYPE_DCT64: return eb_av1_idct64_new;
    case TXFM_TYPE_ADST4: return eb_av1_iadst4_new;
    case TXFM_TYPE_ADST8: return eb_av1_iadst8_new;
    case TXFM_TYPE_ADST16: return eb_av1_iadst16_new;
    case TXFM_TYPE_ADST32: return av1_iadst32_new;
    case TXFM_TYPE_IDENTITY4: return eb_av1_iidentity4_c;
    case TXFM_TYPE_IDENTITY8: return eb_av1_iidentity8_c;
    case TXFM_TYPE_IDENTITY16: return eb_av1_iidentity16_c;
    case TXFM_TYPE_IDENTITY32: return eb_av1_iidentity32_c;
    case TXFM_TYPE_IDENTITY64: return av1_iidentity64_c;
    default: assert(0); return NULL;
    }
}

//void eb_av1_round_shift_array_c(int32_t *arr, int32_t size, int32_t bit) {
//    int32_t i;
//    if (bit == 0) {
//        return;
//    }
//    else {
//        if (bit > 0) {
//            for (i = 0; i < size; i++) {
//                arr[i] = round_shift(arr[i], bit);
//            }
//        }
//        else {
//            for (i = 0; i < size; i++) {
//                arr[i] = arr[i] * (1 << (-bit));
//            }
//        }
//    }
//}
static INLINE TranHigh check_range(TranHigh input, int32_t bd) {
    /*!< AV1 TX case
     *   - 8 bit: signed 16 bit integer
     *   - 10 bit: signed 18 bit integer
     *   - 12 bit: signed 20 bit integer
     *   - max quantization error = 1828 << (bd - 8) */
    const int32_t int_max = (1 << (7 + bd)) - 1 + (914 << (bd - 7));
    const int32_t int_min = -int_max - 1;
#if CONFIG_COEFFICIENT_RANGE_CHECKING
    assert(int_min <= input);
    assert(input <= int_max);
#endif // CONFIG_COEFFICIENT_RANGE_CHECKING
    return (TranHigh)clamp64(input, int_min, int_max);
}
#define HIGHBD_WRAPLOW(x, bd) ((int32_t)check_range((x), bd))
static INLINE uint16_t highbd_clip_pixel_add(uint16_t dest, TranHigh trans, int32_t bd) {
    trans = HIGHBD_WRAPLOW(trans, bd);
    return clip_pixel_highbd(dest + (int32_t)trans, bd);
}
static const int32_t *cast_to_int32(const TranLow *input) {
    assert(sizeof(int32_t) == sizeof(TranLow));
    return (const int32_t *)input;
}
void eb_av1_get_inv_txfm_cfg(TxType tx_type, TxSize tx_size, Txfm2dFlipCfg *cfg) {
    assert(cfg != NULL);
    cfg->tx_size = tx_size;
    set_flip_cfg(tx_type, cfg);
    av1_zero(cfg->stage_range_col);
    av1_zero(cfg->stage_range_row);
    set_flip_cfg(tx_type, cfg);
    const TxType1D tx_type_1d_col = vtx_tab[tx_type];
    const TxType1D tx_type_1d_row = htx_tab[tx_type];
    cfg->shift                    = eb_inv_txfm_shift_ls[tx_size];
    const int32_t txw_idx         = get_txw_idx(tx_size);
    const int32_t txh_idx         = get_txh_idx(tx_size);
    cfg->cos_bit_col              = inv_cos_bit_col[txw_idx][txh_idx];
    cfg->cos_bit_row              = inv_cos_bit_row[txw_idx][txh_idx];
    cfg->txfm_type_col            = av1_txfm_type_ls[txh_idx][tx_type_1d_col];
    if (cfg->txfm_type_col == TXFM_TYPE_ADST4)
        memcpy(cfg->stage_range_col, iadst4_range, sizeof(iadst4_range));
    cfg->txfm_type_row = av1_txfm_type_ls[txw_idx][tx_type_1d_row];
    if (cfg->txfm_type_row == TXFM_TYPE_ADST4)
        memcpy(cfg->stage_range_row, iadst4_range, sizeof(iadst4_range));
    cfg->stage_num_col = av1_txfm_stage_num_list[cfg->txfm_type_col];
    cfg->stage_num_row = av1_txfm_stage_num_list[cfg->txfm_type_row];
}
static INLINE void inv_txfm2d_add_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                    uint16_t *output_w, int32_t stride_w, Txfm2dFlipCfg *cfg,
                                    int32_t *txfm_buf, TxSize tx_size, int32_t bd) {
    /*!< Note when assigning txfm_size_col, we use the txfm_size from the
     *   row configuration and vice versa. This is intentionally done to
     *   accurately perform rectangular transforms. When the transform is
     *   rectangular, the number of columns will be the same as the
     *   txfm_size stored in the row cfg struct. It will make no difference
     *   for square transforms. */
    const int32_t txfm_size_col = tx_size_wide[cfg->tx_size];
    const int32_t txfm_size_row = tx_size_high[cfg->tx_size];
    /*!< Take the shift from the larger dimension in the rectangular case.*/
    const int8_t *shift     = cfg->shift;
    const int32_t rect_type = get_rect_tx_log_ratio(txfm_size_col, txfm_size_row);
    int8_t        stage_range_row[MAX_TXFM_STAGE_NUM];
    int8_t        stage_range_col[MAX_TXFM_STAGE_NUM];
    assert(cfg->stage_num_row <= MAX_TXFM_STAGE_NUM);
    assert(cfg->stage_num_col <= MAX_TXFM_STAGE_NUM);
    eb_av1_gen_inv_stage_range(stage_range_col, stage_range_row, cfg, tx_size, bd);

    const int8_t   cos_bit_col   = cfg->cos_bit_col;
    const int8_t   cos_bit_row   = cfg->cos_bit_row;
    const TxfmFunc txfm_func_col = inv_txfm_type_to_func(cfg->txfm_type_col);
    const TxfmFunc txfm_func_row = inv_txfm_type_to_func(cfg->txfm_type_row);
    ASSERT(txfm_func_col);
    ASSERT(txfm_func_row);
    /*!< txfm_buf's length is  txfm_size_row * txfm_size_col + 2 *
     *   AOMMAX(txfm_size_row, txfm_size_col)
     *   it is used for intermediate data buffering */
    const int32_t buf_offset = AOMMAX(txfm_size_row, txfm_size_col);
    int32_t *     temp_in    = txfm_buf;
    int32_t *     temp_out   = temp_in + buf_offset;
    int32_t *     buf        = temp_out + buf_offset;
    int32_t *     buf_ptr    = buf;
    int32_t       c, r;

    /*!< Rows */
    for (r = 0; r < txfm_size_row; ++r) {
        if (abs(rect_type) == 1) {
            for (c = 0; c < txfm_size_col; ++c)
                temp_in[c] = round_shift((int64_t)input[c] * new_inv_sqrt2, new_sqrt2_bits);
            clamp_buf(temp_in, txfm_size_col, (int8_t)(bd + 8));
            txfm_func_row(temp_in, buf_ptr, cos_bit_row, stage_range_row);
        } else {
            for (c = 0; c < txfm_size_col; ++c) temp_in[c] = input[c];
            clamp_buf(temp_in, txfm_size_col, (int8_t)(bd + 8));
            txfm_func_row(temp_in, buf_ptr, cos_bit_row, stage_range_row);
        }
        eb_av1_round_shift_array_c(buf_ptr, txfm_size_col, -shift[0]);
        input += txfm_size_col;
        buf_ptr += txfm_size_col;
    }

    /*!< Columns */
    for (c = 0; c < txfm_size_col; ++c) {
        if (cfg->lr_flip == 0) {
            for (r = 0; r < txfm_size_row; ++r) temp_in[r] = buf[r * txfm_size_col + c];
        } else {
            /*!< flip left right */
            for (r = 0; r < txfm_size_row; ++r)
                temp_in[r] = buf[r * txfm_size_col + (txfm_size_col - c - 1)];
        }
        clamp_buf(temp_in, txfm_size_row, (int8_t)(AOMMAX(bd + 6, 16)));
        txfm_func_col(temp_in, temp_out, cos_bit_col, stage_range_col);
        eb_av1_round_shift_array_c(temp_out, txfm_size_row, -shift[1]);
        if (cfg->ud_flip == 0) {
            for (r = 0; r < txfm_size_row; ++r) {
                output_w[r * stride_w + c] =
                    highbd_clip_pixel_add(output_r[r * stride_r + c], temp_out[r], bd);
            }
        } else {
            /*!< flip upside down */
            for (r = 0; r < txfm_size_row; ++r) {
                output_w[r * stride_w + c] = highbd_clip_pixel_add(
                    output_r[r * stride_r + c], temp_out[txfm_size_row - r - 1], bd);
            }
        }
    }
}
static INLINE void inv_txfm2d_add_facade(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                         uint16_t *output_w, int32_t stride_w, int32_t *txfm_buf,
                                         TxType tx_type, TxSize tx_size, int32_t bd) {
    Txfm2dFlipCfg cfg;
    eb_av1_get_inv_txfm_cfg(tx_type, tx_size, &cfg);
    /*!< Forward shift sum uses larger square size, to be consistent with what
     *   eb_av1_gen_inv_stage_range() does for inverse shifts. */
    inv_txfm2d_add_c(input, output_r, stride_r, output_w, stride_w, &cfg, txfm_buf, tx_size, bd);
}
void eb_av1_inv_txfm2d_add_4x4_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                 uint16_t *output_w, int32_t stride_w, TxType tx_type, int32_t bd) {
    DECLARE_ALIGNED(32, int32_t, txfm_buf[4 * 4 + 4 + 4]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_4X4, bd);
}
void eb_av1_inv_txfm2d_add_8x8_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                 uint16_t *output_w, int32_t stride_w, TxType tx_type, int32_t bd) {
    DECLARE_ALIGNED(32, int32_t, txfm_buf[8 * 8 + 8 + 8]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_8X8, bd);
}
void eb_av1_inv_txfm2d_add_16x16_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   int32_t bd) {
    DECLARE_ALIGNED(32, int32_t, txfm_buf[16 * 16 + 16 + 16]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_16X16, bd);
}

void eb_av1_inv_txfm2d_add_32x32_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   int32_t bd) {
    DECLARE_ALIGNED(32, int32_t, txfm_buf[32 * 32 + 32 + 32]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_32X32, bd);
}

void eb_av1_inv_txfm2d_add_64x64_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   int32_t bd) {
    /*!< TODO(urvang): Can the same array be reused, instead of using a new array?
     *   Remap 32x32 input into a modified 64x64 by:
     *   - Copying over these values in top-left 32x32 locations.
     *   - Setting the rest of the locations to 0. */
    int32_t mod_input[64 * 64];
    for (int32_t row = 0; row < 32; ++row) {
        memcpy(mod_input + row * 64, input + row * 32, 32 * sizeof(*mod_input));
        memset(mod_input + row * 64 + 32, 0, 32 * sizeof(*mod_input));
    }
    memset(mod_input + 32 * 64, 0, 32 * 64 * sizeof(*mod_input));
    DECLARE_ALIGNED(32, int32_t, txfm_buf[64 * 64 + 64 + 64]);
    inv_txfm2d_add_facade(
        mod_input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_64X64, bd);
}

void eb_av1_inv_txfm2d_add_4x8_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                 uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                 TxSize tx_size, int32_t bd) {
    (void)tx_size;
    DECLARE_ALIGNED(32, int32_t, txfm_buf[4 * 8 + 8 + 8]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_4X8, bd);
}

void eb_av1_inv_txfm2d_add_8x4_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                 uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                 TxSize tx_size, int32_t bd) {
    (void)tx_size;
    DECLARE_ALIGNED(32, int32_t, txfm_buf[8 * 4 + 8 + 8]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_8X4, bd);
}

void eb_av1_inv_txfm2d_add_8x16_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                  uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                  TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[8 * 16 + 16 + 16]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_8X16, bd);
}

void eb_av1_inv_txfm2d_add_16x8_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                  uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                  TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[16 * 8 + 16 + 16]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_16X8, bd);
}

void eb_av1_inv_txfm2d_add_16x32_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[16 * 32 + 32 + 32]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_16X32, bd);
}

void eb_av1_inv_txfm2d_add_32x16_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[32 * 16 + 32 + 32]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_32X16, bd);
}

void eb_av1_inv_txfm2d_add_64x32_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    /*!< Remap 32x32 input into a modified 64x32 by:
     *   - Copying over these values in top-left 32x32 locations.
     *   - Setting the rest of the locations to 0. */
    int32_t mod_input[64 * 32];
    for (int32_t row = 0; row < 32; ++row) {
        memcpy(mod_input + row * 64, input + row * 32, 32 * sizeof(*mod_input));
        memset(mod_input + row * 64 + 32, 0, 32 * sizeof(*mod_input));
    }
    DECLARE_ALIGNED(32, int32_t, txfm_buf[64 * 32 + 64 + 64]);
    inv_txfm2d_add_facade(
        mod_input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_64X32, bd);
}

void eb_av1_inv_txfm2d_add_32x64_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    /*!< Remap 32x32 input into a modified 32x64 input by:
     *   - Copying over these values in top-left 32x32 locations.
     *   - Setting the rest of the locations to 0. */
    int32_t mod_input[32 * 64];
    memcpy(mod_input, input, 32 * 32 * sizeof(*mod_input));
    memset(mod_input + 32 * 32, 0, 32 * 32 * sizeof(*mod_input));
    DECLARE_ALIGNED(32, int32_t, txfm_buf[64 * 32 + 64 + 64]);
    inv_txfm2d_add_facade(
        mod_input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_32X64, bd);
}

void eb_av1_inv_txfm2d_add_16x64_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    /*!< Remap 16x32 input into a modified 16x64 input by:
     *   - Copying over these values in top-left 16x32 locations.
     *   - Setting the rest of the locations to 0. */
    int32_t mod_input[16 * 64];
    memcpy(mod_input, input, 16 * 32 * sizeof(*mod_input));
    memset(mod_input + 16 * 32, 0, 16 * 32 * sizeof(*mod_input));
    DECLARE_ALIGNED(32, int32_t, txfm_buf[16 * 64 + 64 + 64]);
    inv_txfm2d_add_facade(
        mod_input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_16X64, bd);
}

void eb_av1_inv_txfm2d_add_64x16_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                   uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                   TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    /*!< Remap 32x16 input into a modified 64x16 by:
     *   - Copying over these values in top-left 32x16 locations.
     *   - Setting the rest of the locations to 0. */
    int32_t mod_input[64 * 16];
    for (int32_t row = 0; row < 16; ++row) {
        memcpy(mod_input + row * 64, input + row * 32, 32 * sizeof(*mod_input));
        memset(mod_input + row * 64 + 32, 0, 32 * sizeof(*mod_input));
    }
    DECLARE_ALIGNED(32, int32_t, txfm_buf[16 * 64 + 64 + 64]);
    inv_txfm2d_add_facade(
        mod_input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_64X16, bd);
}

void eb_av1_inv_txfm2d_add_4x16_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                  uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                  TxSize tx_size, int32_t bd) {
    UNUSED(tx_size);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[4 * 16 + 16 + 16]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_4X16, bd);
}

void eb_av1_inv_txfm2d_add_16x4_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                  uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                  TxSize tx_size, int32_t bd) {
    UNUSED(tx_size);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[4 * 16 + 16 + 16]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_16X4, bd);
}

void eb_av1_inv_txfm2d_add_8x32_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                  uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                  TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[8 * 32 + 32 + 32]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_8X32, bd);
}

void eb_av1_inv_txfm2d_add_32x8_c(const int32_t *input, uint16_t *output_r, int32_t stride_r,
                                  uint16_t *output_w, int32_t stride_w, TxType tx_type,
                                  TxSize tx_size, int32_t eob, int32_t bd) {
    UNUSED(tx_size);
    UNUSED(eob);
    DECLARE_ALIGNED(32, int32_t, txfm_buf[8 * 32 + 32 + 32]);
    inv_txfm2d_add_facade(
        input, output_r, stride_r, output_w, stride_w, txfm_buf, tx_type, TX_32X8, bd);
}

static INLINE int32_t range_check_value(int32_t value, int8_t bit) {
#if CONFIG_COEFFICIENT_RANGE_CHECKING
    const int64_t max_value = (1LL << (bit - 1)) - 1;
    const int64_t min_value = -(1LL << (bit - 1));
    if (value < min_value || value > max_value) {
        SVT_ERROR("coeff out of bit range, value: %d bit %d\n", value, bit);
        assert(0);
    }
#endif // CONFIG_COEFFICIENT_RANGE_CHECKING
#if DO_RANGE_CHECK_CLAMP
    bit = AOMMIN(bit, 31);
    return clamp(value, (1 << (bit - 1)) - 1, -(1 << (bit - 1)));
#endif // DO_RANGE_CHECK_CLAMP
    (void)bit;
    return value;
}

void eb_av1_highbd_iwht4x4_16_add_c(const TranLow *input, uint8_t *dest8_r, int32_t stride_r,
                                    uint8_t *dest8_w, int32_t stride_w, int32_t bd) {
    /*!< 4-point reversible, orthonormal inverse Walsh-Hadamard in 3.5 adds,
     *   0.5 shifts per pixel. */
    int32_t        i;
    TranLow        output[16];
    TranLow        a1, b1, c1, d1, e1;
    const TranLow *ip     = input;
    TranLow *      op     = output;
    uint16_t *     dest_r = CONVERT_TO_SHORTPTR(dest8_r);
    uint16_t *     dest_w = CONVERT_TO_SHORTPTR(dest8_w);

    for (i = 0; i < 4; i++) {
        a1 = ip[0] >> UNIT_QUANT_SHIFT;
        c1 = ip[1] >> UNIT_QUANT_SHIFT;
        d1 = ip[2] >> UNIT_QUANT_SHIFT;
        b1 = ip[3] >> UNIT_QUANT_SHIFT;
        a1 += c1;
        d1 -= b1;
        e1 = (a1 - d1) >> 1;
        b1 = e1 - b1;
        c1 = e1 - c1;
        a1 -= b1;
        d1 += c1;
        op[0] = a1;
        op[1] = b1;
        op[2] = c1;
        op[3] = d1;
        ip += 4;
        op += 4;
    }

    ip = output;
    for (i = 0; i < 4; i++) {
        a1 = ip[4 * 0];
        c1 = ip[4 * 1];
        d1 = ip[4 * 2];
        b1 = ip[4 * 3];
        a1 += c1;
        d1 -= b1;
        e1 = (a1 - d1) >> 1;
        b1 = e1 - b1;
        c1 = e1 - c1;
        a1 -= b1;
        d1 += c1;
        range_check_value(a1, (int8_t)(bd + 1));
        range_check_value(b1, (int8_t)(bd + 1));
        range_check_value(c1, (int8_t)(bd + 1));
        range_check_value(d1, (int8_t)(bd + 1));

        dest_w[stride_w * 0] = highbd_clip_pixel_add(dest_r[stride_r * 0], a1, bd);
        dest_w[stride_w * 1] = highbd_clip_pixel_add(dest_r[stride_r * 1], b1, bd);
        dest_w[stride_w * 2] = highbd_clip_pixel_add(dest_r[stride_r * 2], c1, bd);
        dest_w[stride_w * 3] = highbd_clip_pixel_add(dest_r[stride_r * 3], d1, bd);

        ip++;
        dest_r++;
        dest_w++;
    }
}

void eb_av1_highbd_iwht4x4_1_add_c(const TranLow *in, uint8_t *dest8_r, int32_t dest_stride_r,
                                   uint8_t *dest8_w, int32_t dest_stride_w, int32_t bd) {
    int32_t        i;
    TranLow        a1, e1;
    TranLow        tmp[4];
    const TranLow *ip     = in;
    TranLow *      op     = tmp;
    uint16_t *     dest_r = CONVERT_TO_SHORTPTR(dest8_r);
    uint16_t *     dest_w = CONVERT_TO_SHORTPTR(dest8_w);
    (void)bd;

    a1 = ip[0] >> UNIT_QUANT_SHIFT;
    e1 = a1 >> 1;
    a1 -= e1;
    op[0] = a1;
    op[1] = op[2] = op[3] = e1;

    ip = tmp;
    for (i = 0; i < 4; i++) {
        e1                        = ip[0] >> 1;
        a1                        = ip[0] - e1;
        dest_w[dest_stride_w * 0] = highbd_clip_pixel_add(dest_r[dest_stride_r * 0], a1, bd);
        dest_w[dest_stride_w * 1] = highbd_clip_pixel_add(dest_r[dest_stride_r * 1], e1, bd);
        dest_w[dest_stride_w * 2] = highbd_clip_pixel_add(dest_r[dest_stride_r * 2], e1, bd);
        dest_w[dest_stride_w * 3] = highbd_clip_pixel_add(dest_r[dest_stride_r * 3], e1, bd);
        ip++;
        dest_r++;
        dest_w++;
    }
}
static void highbd_iwht4x4_add(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                               uint8_t *dest_w, int32_t stride_w, int32_t eob, int32_t bd) {
    if (eob > 1)
        eb_av1_highbd_iwht4x4_16_add_c(input, dest_r, stride_r, dest_w, stride_w, bd);
    else
        eb_av1_highbd_iwht4x4_1_add_c(input, dest_r, stride_r, dest_w, stride_w, bd);
}
void eb_av1_highbd_inv_txfm_add_4x4(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                    uint8_t *dest_w, int32_t stride_w,
                                    const TxfmParam *txfm_param) {
    // assert(av1_ext_tx_used[txfm_param->tx_set_type][txfm_param->tx_type]);
    int32_t        eob      = txfm_param->eob;
    int32_t        bd       = txfm_param->bd;
    int32_t        lossless = txfm_param->lossless;
    const int32_t *src      = cast_to_int32(input);
    const TxType   tx_type  = txfm_param->tx_type;
    if (lossless) {
        assert(tx_type == DCT_DCT);
        highbd_iwht4x4_add(input, dest_r, stride_r, dest_w, stride_w, eob, bd);
        return;
    }
    eb_av1_inv_txfm2d_add_4x4(src,
                              CONVERT_TO_SHORTPTR(dest_r),
                              stride_r,
                              CONVERT_TO_SHORTPTR(dest_w),
                              stride_w,
                              tx_type,
                              bd);
}
static void highbd_inv_txfm_add_8x8(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                    uint8_t *dest_w, int32_t stride_w,
                                    const TxfmParam *txfm_param) {
    int32_t        bd      = txfm_param->bd;
    const TxType   tx_type = txfm_param->tx_type;
    const int32_t *src     = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_8x8(src,
                              CONVERT_TO_SHORTPTR(dest_r),
                              stride_r,
                              CONVERT_TO_SHORTPTR(dest_w),
                              stride_w,
                              tx_type,
                              bd);
}

static void highbd_inv_txfm_add_16x16(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    int32_t        bd      = txfm_param->bd;
    const TxType   tx_type = txfm_param->tx_type;
    const int32_t *src     = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_16x16(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                tx_type,
                                bd);
}

static void highbd_inv_txfm_add_32x32(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t  bd      = txfm_param->bd;
    const TxType   tx_type = txfm_param->tx_type;
    const int32_t *src     = cast_to_int32(input);
    switch (tx_type) {
    case DCT_DCT:
    case IDTX:
        eb_av1_inv_txfm2d_add_32x32(src,
                                    CONVERT_TO_SHORTPTR(dest_r),
                                    stride_r,
                                    CONVERT_TO_SHORTPTR(dest_w),
                                    stride_w,
                                    tx_type,
                                    bd);
        break;
    default: assert(0);
    }
}

static void highbd_inv_txfm_add_64x64(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t  bd      = txfm_param->bd;
    const TxType   tx_type = txfm_param->tx_type;
    const int32_t *src     = cast_to_int32(input);
    assert(tx_type == DCT_DCT);
    eb_av1_inv_txfm2d_add_64x64(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                tx_type,
                                bd);
}

static void highbd_inv_txfm_add_4x8(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                    uint8_t *dest_w, int32_t stride_w,
                                    const TxfmParam *txfm_param) {
    /*!< TODO: add this assert once we fill tx_set_type    assert(av1_ext_tx_used[txfm_param->tx_set_type][txfm_param->tx_type]); */
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_4x8(src,
                              CONVERT_TO_SHORTPTR(dest_r),
                              stride_r,
                              CONVERT_TO_SHORTPTR(dest_w),
                              stride_w,
                              txfm_param->tx_type,
                              txfm_param->tx_size,
                              txfm_param->bd);
}

static void highbd_inv_txfm_add_8x4(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                    uint8_t *dest_w, int32_t stride_w,
                                    const TxfmParam *txfm_param) {
    /*!< TODO: add this assert once we fill tx_set_type    assert(av1_ext_tx_used[txfm_param->tx_set_type][txfm_param->tx_type]); */
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_8x4(src,
                              CONVERT_TO_SHORTPTR(dest_r),
                              stride_r,
                              CONVERT_TO_SHORTPTR(dest_w),
                              stride_w,
                              txfm_param->tx_type,
                              txfm_param->tx_size,
                              txfm_param->bd);
}

static void highbd_inv_txfm_add_8x16(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                     uint8_t *dest_w, int32_t stride_w,
                                     const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_8x16(src,
                               CONVERT_TO_SHORTPTR(dest_r),
                               stride_r,
                               CONVERT_TO_SHORTPTR(dest_w),
                               stride_w,
                               txfm_param->tx_type,
                               txfm_param->tx_size,
                               txfm_param->eob,
                               txfm_param->bd);
}

static void highbd_inv_txfm_add_16x8(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                     uint8_t *dest_w, int32_t stride_w,
                                     const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_16x8(src,
                               CONVERT_TO_SHORTPTR(dest_r),
                               stride_r,
                               CONVERT_TO_SHORTPTR(dest_w),
                               stride_w,
                               txfm_param->tx_type,
                               txfm_param->tx_size,
                               txfm_param->eob,
                               txfm_param->bd);
}

static void highbd_inv_txfm_add_16x32(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_16x32(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                txfm_param->tx_type,
                                txfm_param->tx_size,
                                txfm_param->eob,
                                txfm_param->bd);
}

static void highbd_inv_txfm_add_32x16(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_32x16(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                txfm_param->tx_type,
                                txfm_param->tx_size,
                                txfm_param->eob,
                                txfm_param->bd);
}

static void highbd_inv_txfm_add_16x4(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                     uint8_t *dest_w, int32_t stride_w,
                                     const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_16x4(src,
                               CONVERT_TO_SHORTPTR(dest_r),
                               stride_r,
                               CONVERT_TO_SHORTPTR(dest_w),
                               stride_w,
                               txfm_param->tx_type,
                               txfm_param->tx_size,
                               txfm_param->bd);
}

static void highbd_inv_txfm_add_4x16(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                     uint8_t *dest_w, int32_t stride_w,
                                     const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_4x16(src,
                               CONVERT_TO_SHORTPTR(dest_r),
                               stride_r,
                               CONVERT_TO_SHORTPTR(dest_w),
                               stride_w,
                               txfm_param->tx_type,
                               txfm_param->tx_size,
                               txfm_param->bd);
}

static void highbd_inv_txfm_add_32x8(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                     uint8_t *dest_w, int32_t stride_w,
                                     const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_32x8(src,
                               CONVERT_TO_SHORTPTR(dest_r),
                               stride_r,
                               CONVERT_TO_SHORTPTR(dest_w),
                               stride_w,
                               txfm_param->tx_type,
                               txfm_param->tx_size,
                               txfm_param->eob,
                               txfm_param->bd);
}

static void highbd_inv_txfm_add_8x32(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                     uint8_t *dest_w, int32_t stride_w,
                                     const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_8x32(src,
                               CONVERT_TO_SHORTPTR(dest_r),
                               stride_r,
                               CONVERT_TO_SHORTPTR(dest_w),
                               stride_w,
                               txfm_param->tx_type,
                               txfm_param->tx_size,
                               txfm_param->eob,
                               txfm_param->bd);
}

static void highbd_inv_txfm_add_32x64(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_32x64(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                txfm_param->tx_type,
                                txfm_param->tx_size,
                                txfm_param->eob,
                                txfm_param->bd);
}

static void highbd_inv_txfm_add_64x32(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_64x32(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                txfm_param->tx_type,
                                txfm_param->tx_size,
                                txfm_param->eob,
                                txfm_param->bd);
}

static void highbd_inv_txfm_add_16x64(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_16x64(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                txfm_param->tx_type,
                                txfm_param->tx_size,
                                txfm_param->eob,
                                txfm_param->bd);
}

static void highbd_inv_txfm_add_64x16(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                      uint8_t *dest_w, int32_t stride_w,
                                      const TxfmParam *txfm_param) {
    const int32_t *src = cast_to_int32(input);
    eb_av1_inv_txfm2d_add_64x16(src,
                                CONVERT_TO_SHORTPTR(dest_r),
                                stride_r,
                                CONVERT_TO_SHORTPTR(dest_w),
                                stride_w,
                                txfm_param->tx_type,
                                txfm_param->tx_size,
                                txfm_param->eob,
                                txfm_param->bd);
}

static void highbd_inv_txfm_add(const TranLow *input, uint8_t *dest_r, int32_t stride_r,
                                uint8_t *dest_w, int32_t stride_w, const TxfmParam *txfm_param) {
    //assert(av1_ext_tx_used[txfm_param->tx_set_type][txfm_param->tx_type]);
    const TxSize tx_size = txfm_param->tx_size;
    switch (tx_size) {
    case TX_32X32:
        highbd_inv_txfm_add_32x32(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_16X16:
        highbd_inv_txfm_add_16x16(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_8X8:
        highbd_inv_txfm_add_8x8(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_4X8:
        highbd_inv_txfm_add_4x8(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_8X4:
        highbd_inv_txfm_add_8x4(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_8X16:
        highbd_inv_txfm_add_8x16(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_16X8:
        highbd_inv_txfm_add_16x8(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_16X32:
        highbd_inv_txfm_add_16x32(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_32X16:
        highbd_inv_txfm_add_32x16(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_64X64:
        highbd_inv_txfm_add_64x64(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_32X64:
        highbd_inv_txfm_add_32x64(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_64X32:
        highbd_inv_txfm_add_64x32(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_16X64:
        highbd_inv_txfm_add_16x64(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_64X16:
        highbd_inv_txfm_add_64x16(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_4X4:
        /*!< this is like av1_short_idct4x4 but has a special case around eob<=1
         *   which is significant (not just an optimization) for the lossless case. */
        eb_av1_highbd_inv_txfm_add_4x4(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_16X4:
        highbd_inv_txfm_add_16x4(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_4X16:
        highbd_inv_txfm_add_4x16(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_8X32:
        highbd_inv_txfm_add_8x32(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    case TX_32X8:
        highbd_inv_txfm_add_32x8(input, dest_r, stride_r, dest_w, stride_w, txfm_param);
        break;
    default: assert(0 && "Invalid transform size"); break;
    }
}

void eb_av1_inv_txfm_add_c(const TranLow *dqcoeff, uint8_t *dst_r, int32_t stride_r, uint8_t *dst_w,
                           int32_t stride_w, const TxfmParam *txfm_param) {
    const TxSize tx_size = txfm_param->tx_size;
    DECLARE_ALIGNED(32, uint16_t, tmp[MAX_TX_SQUARE]);
    int32_t tmp_stride = MAX_TX_SIZE;
    int32_t w          = tx_size_wide[tx_size];
    int32_t h          = tx_size_high[tx_size];
    for (int32_t r = 0; r < h; ++r) {
        for (int32_t c = 0; c < w; ++c) tmp[r * tmp_stride + c] = dst_r[r * stride_r + c];
    }

    highbd_inv_txfm_add(dqcoeff,
                        CONVERT_TO_BYTEPTR(tmp),
                        tmp_stride,
                        CONVERT_TO_BYTEPTR(tmp),
                        tmp_stride,
                        txfm_param);

    for (int32_t r = 0; r < h; ++r) {
        for (int32_t c = 0; c < w; ++c) dst_w[r * stride_w + c] = (uint8_t)tmp[r * tmp_stride + c];
    }
}

EbErrorType av1_inv_transform_recon(int32_t *coeff_buffer, /*!< 1D buffer*/
                                    uint8_t *recon_buffer_r, uint32_t recon_stride_r,
                                    uint8_t *recon_buffer_w, uint32_t recon_stride_w, TxSize txsize,
                                    uint32_t bit_increment, TxType transform_type,
                                    PlaneType component_type, uint32_t eob, uint8_t lossless) {
    UNUSED(component_type);
    EbErrorType return_error = EB_ErrorNone;
    TxfmParam   txfm_param;
    txfm_param.tx_type  = transform_type;
    txfm_param.tx_size  = txsize;
    txfm_param.eob      = eob;
    txfm_param.lossless = lossless;
    txfm_param.bd       = bit_increment + EB_8BIT;
    txfm_param.is_hbd   = 1;
    //TxfmParam.tx_set_type = av1_get_ext_tx_set_type(   txfm_param->tx_size, is_inter_block(xd->mi[0]), reduced_tx_set);

    if (recon_buffer_r != recon_buffer_w) {
        /*!< When output pointers to read and write are differents,
         *   then kernel copy also all buffer from read to write,
         *   and cannot be limited by End Of Buffer calculations. */
        txfm_param.eob = av1_get_max_eob(txsize);
    }

    highbd_inv_txfm_add((const TranLow *)coeff_buffer,
                        recon_buffer_r,
                        recon_stride_r,
                        recon_buffer_w,
                        recon_stride_w,
                        &txfm_param);

    return return_error;
}

EbErrorType av1_inv_transform_recon8bit(int32_t *coeff_buffer, /*!< 1D buffer */
                                        uint8_t *recon_buffer_r, uint32_t recon_stride_r,
                                        uint8_t *recon_buffer_w, uint32_t recon_stride_w,
                                        TxSize txsize, TxType transform_type,
                                        PlaneType component_type, uint32_t eob, uint8_t lossless) {
    UNUSED(component_type);
    EbErrorType return_error = EB_ErrorNone;
    TxfmParam   txfm_param;
    txfm_param.tx_type  = transform_type;
    txfm_param.tx_size  = txsize;
    txfm_param.eob      = eob;
    txfm_param.lossless = lossless;
    txfm_param.bd       = 8;
    txfm_param.is_hbd   = 1;
    //TxfmParam.tx_set_type = av1_get_ext_tx_set_type(   txfm_param->tx_size, is_inter_block(xd->mi[0]), reduced_tx_set);

    if (recon_buffer_r != recon_buffer_w) {
        /*!< When output pointers to read and write are differents,
         *   then kernel copy also all buffer from read to write,
         *   and cannot be limited by End Of Buffer calculations. */
        txfm_param.eob = av1_get_max_eob(txsize);
    }

    eb_av1_inv_txfm_add((const TranLow *)coeff_buffer,
                        recon_buffer_r,
                        recon_stride_r,
                        recon_buffer_w,
                        recon_stride_w,
                        &txfm_param);

    return return_error;
}
