/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

/*
* Copyright (c) 2016, Alliance for Open Media. All rights reserved
*
* This source code is subject to the terms of the BSD 2 Clause License and
* the Alliance for Open Media Patent License 1.0. If the BSD 2 Clause License
* was not distributed with this source code in the LICENSE file, you can
* obtain it at www.aomedia.org/license/software. If the Alliance for Open
* Media Patent License 1.0 was not distributed with this source code in the
* PATENTS file, you can obtain it at www.aomedia.org/license/patent.
*/
#include <string.h>

#include "EbCodingLoop.h"
#include "EbUtility.h"
#include "EbTransformUnit.h"
#include "EbRateDistortionCost.h"
#include "EbDeblockingFilter.h"
#include "EbPictureOperators.h"
#include "EbSegmentation.h"
#include "EbEncDecProcess.h"
#include "EbSvtAv1ErrorCodes.h"
#include "EbTransforms.h"
#include "EbInvTransforms.h"
#include "EbModeDecisionConfigurationProcess.h"
#include "EbIntraPrediction.h"
#include "aom_dsp_rtcd.h"
#include "EbMdRateEstimation.h"
void av1_set_ref_frame(MvReferenceFrame *rf, int8_t ref_frame_type);

/*******************************************
* set Penalize Skip Flag
*
* Summary: Set the penalize_skipflag to true
* When there is luminance/chrominance change
* or in noisy clip with low motion at meduim
* varince area
*
*******************************************/

#define S32 32 * 32
#define S16 16 * 16
#define S8 8 * 8
#define S4 4 * 4

static EbAv1InterPredictionFuncPtr av1_inter_prediction_function_table[2] = {
    av1_inter_prediction, av1_inter_prediction_hbd};

typedef void (*EbAv1EncodeLoopFuncPtr)(PictureControlSet *pcs_ptr, EncDecContext *context_ptr,
                                       SuperBlock *sb_ptr, uint32_t origin_x, uint32_t origin_y,
                                       uint32_t             cb_qp,
                                       EbPictureBufferDesc *pred_samples, // no basis/offset
                                       EbPictureBufferDesc *coeff_samples_sb, // sb based
                                       EbPictureBufferDesc *residual16bit, // no basis/offset
                                       EbPictureBufferDesc *transform16bit, // no basis/offset
                                       EbPictureBufferDesc *inverse_quant_buffer,
                                       int16_t *            transform_scratch_buffer,
                                       uint32_t *count_non_zero_coeffs, uint32_t component_mask,
                                       uint16_t *eob, MacroblockPlane *candidate_plane);

typedef void (*EbAv1GenerateReconFuncPtr)(EncDecContext *context_ptr, uint32_t origin_x,
                                          uint32_t             origin_y,
                                          EbPictureBufferDesc *pred_samples, // no basis/offset
                                          EbPictureBufferDesc *residual16bit, // no basis/offset
                                          int16_t *            transform_scratch_buffer,
                                          uint32_t component_mask, uint16_t *eob);

/*******************************************
* Residual Kernel 8-16bit
    Computes the residual data
*******************************************/
void residual_kernel(uint8_t *input, uint32_t input_offset, uint32_t input_stride, uint8_t *pred,
                     uint32_t pred_offset, uint32_t pred_stride, int16_t *residual,
                     uint32_t residual_offset, uint32_t residual_stride, EbBool hbd,
                     uint32_t area_width, uint32_t area_height) {
    if (hbd) {
        residual_kernel16bit(((uint16_t *)input) + input_offset,
                             input_stride,
                             ((uint16_t *)pred) + pred_offset,
                             pred_stride,
                             residual + residual_offset,
                             residual_stride,
                             area_width,
                             area_height);
    } else {
        residual_kernel8bit(&(input[input_offset]),
                            input_stride,
                            &(pred[pred_offset]),
                            pred_stride,
                            residual + residual_offset,
                            residual_stride,
                            area_width,
                            area_height);
    }
}

/***************************************************
* Update Intra Mode Neighbor Arrays
***************************************************/
static void encode_pass_update_intra_mode_neighbor_arrays(
    NeighborArrayUnit *mode_type_neighbor_array, NeighborArrayUnit *intra_luma_mode_neighbor_array,
    NeighborArrayUnit *intra_chroma_mode_neighbor_array, uint8_t luma_mode, uint8_t chroma_mode,
    uint32_t origin_x, uint32_t origin_y, uint32_t width, uint32_t height, uint32_t width_uv,
    uint32_t height_uv, uint32_t component_mask) {
    uint8_t mode_type = INTRA_MODE;

    if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
        // Mode Type Update
        neighbor_array_unit_mode_write(mode_type_neighbor_array,
                                       &mode_type,
                                       origin_x,
                                       origin_y,
                                       width,
                                       height,
                                       NEIGHBOR_ARRAY_UNIT_FULL_MASK);

        // Intra Luma Mode Update
        neighbor_array_unit_mode_write(intra_luma_mode_neighbor_array,
                                       &luma_mode,
                                       origin_x,
                                       origin_y,
                                       width,
                                       height,
                                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }
    if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK) {
        // Intra Luma Mode Update
        neighbor_array_unit_mode_write(intra_chroma_mode_neighbor_array,
                                       &chroma_mode,
                                       ((origin_x >> 3) << 3) / 2,
                                       ((origin_y >> 3) << 3) / 2,
                                       width_uv,
                                       height_uv,
                                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }

    return;
}

/***************************************************
* Update Inter Mode Neighbor Arrays
***************************************************/
static void encode_pass_update_inter_mode_neighbor_arrays(
    NeighborArrayUnit *mode_type_neighbor_array, NeighborArrayUnit *mv_neighbor_array,
    NeighborArrayUnit *skipNeighborArray, MvUnit *mv_unit, uint8_t *skip_flag, uint32_t origin_x,
    uint32_t origin_y, uint32_t bwidth, uint32_t bheight) {
    uint8_t mode_type = INTER_MODE;

    // Mode Type Update
    neighbor_array_unit_mode_write(mode_type_neighbor_array,
                                   &mode_type,
                                   origin_x,
                                   origin_y,
                                   bwidth,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    // Motion Vector Unit
    neighbor_array_unit_mode_write(mv_neighbor_array,
                                   (uint8_t *)mv_unit,
                                   origin_x,
                                   origin_y,
                                   bwidth,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    // Skip Flag
    neighbor_array_unit_mode_write(skipNeighborArray,
                                   skip_flag,
                                   origin_x,
                                   origin_y,
                                   bwidth,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    return;
}

/***************************************************
* Update Recon Samples Neighbor Arrays
***************************************************/
static void encode_pass_update_recon_sample_neighbour_arrays(
    NeighborArrayUnit *lumaReconSampleNeighborArray, NeighborArrayUnit *cbReconSampleNeighborArray,
    NeighborArrayUnit *crReconSampleNeighborArray, EbPictureBufferDesc *recon_buffer,
    uint32_t origin_x, uint32_t origin_y, uint32_t width, uint32_t height, uint32_t bwidth_uv,
    uint32_t bheight_uv, uint32_t component_mask, EbBool is_16bit) {
    uint32_t round_origin_x = (origin_x >> 3) << 3; // for Chroma blocks with size of 4
    uint32_t round_origin_y = (origin_y >> 3) << 3; // for Chroma blocks with size of 4

    if (is_16bit == EB_TRUE) {
        if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
            // Recon Samples - Luma
            neighbor_array_unit16bit_sample_write(lumaReconSampleNeighborArray,
                                                  (uint16_t *)(recon_buffer->buffer_y),
                                                  recon_buffer->stride_y,
                                                  recon_buffer->origin_x + origin_x,
                                                  recon_buffer->origin_y + origin_y,
                                                  origin_x,
                                                  origin_y,
                                                  width,
                                                  height,
                                                  NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }

        if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK) {
            // Recon Samples - Cb
            neighbor_array_unit16bit_sample_write(cbReconSampleNeighborArray,
                                                  (uint16_t *)(recon_buffer->buffer_cb),
                                                  recon_buffer->stride_cb,
                                                  (recon_buffer->origin_x + round_origin_x) >> 1,
                                                  (recon_buffer->origin_y + round_origin_y) >> 1,
                                                  round_origin_x >> 1,
                                                  round_origin_y >> 1,
                                                  bwidth_uv,
                                                  bheight_uv,
                                                  NEIGHBOR_ARRAY_UNIT_FULL_MASK);

            // Recon Samples - Cr
            neighbor_array_unit16bit_sample_write(crReconSampleNeighborArray,
                                                  (uint16_t *)(recon_buffer->buffer_cr),
                                                  recon_buffer->stride_cr,
                                                  (recon_buffer->origin_x + round_origin_x) >> 1,
                                                  (recon_buffer->origin_y + round_origin_y) >> 1,
                                                  round_origin_x >> 1,
                                                  round_origin_y >> 1,
                                                  bwidth_uv,
                                                  bheight_uv,
                                                  NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }
    } else {
        if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
            // Recon Samples - Luma
            neighbor_array_unit_sample_write(lumaReconSampleNeighborArray,
                                             recon_buffer->buffer_y,
                                             recon_buffer->stride_y,
                                             recon_buffer->origin_x + origin_x,
                                             recon_buffer->origin_y + origin_y,
                                             origin_x,
                                             origin_y,
                                             width,
                                             height,
                                             NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }

        if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK) {
            // Recon Samples - Cb
            neighbor_array_unit_sample_write(cbReconSampleNeighborArray,
                                             recon_buffer->buffer_cb,
                                             recon_buffer->stride_cb,
                                             (recon_buffer->origin_x + round_origin_x) >> 1,
                                             (recon_buffer->origin_y + round_origin_y) >> 1,
                                             round_origin_x >> 1,
                                             round_origin_y >> 1,
                                             bwidth_uv,
                                             bheight_uv,
                                             NEIGHBOR_ARRAY_UNIT_FULL_MASK);

            // Recon Samples - Cr
            neighbor_array_unit_sample_write(crReconSampleNeighborArray,
                                             recon_buffer->buffer_cr,
                                             recon_buffer->stride_cr,
                                             (recon_buffer->origin_x + round_origin_x) >> 1,
                                             (recon_buffer->origin_y + round_origin_y) >> 1,
                                             round_origin_x >> 1,
                                             round_origin_y >> 1,
                                             bwidth_uv,
                                             bheight_uv,
                                             NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }
    }

    return;
}

/************************************************************
* Update Intra Luma Neighbor Modes
************************************************************/
void generate_pu_intra_luma_neighbor_nodes(BlkStruct *blk_ptr, uint32_t pu_origin_x,
                                           uint32_t pu_origin_y, uint32_t sb_sz,
                                           NeighborArrayUnit *intraLumaNeighborArray,
                                           NeighborArrayUnit *intraChromaNeighborArray,
                                           NeighborArrayUnit *mode_type_neighbor_array) {
    (void)sb_sz;

    uint32_t mode_type_left_neighbor_index =
        get_neighbor_array_unit_left_index(mode_type_neighbor_array, pu_origin_y);
    uint32_t mode_type_top_neighbor_index =
        get_neighbor_array_unit_top_index(mode_type_neighbor_array, pu_origin_x);
    uint32_t intra_luma_mode_left_neighbor_index =
        get_neighbor_array_unit_left_index(intraLumaNeighborArray, pu_origin_y);
    uint32_t intra_luma_mode_top_neighbor_index =
        get_neighbor_array_unit_top_index(intraLumaNeighborArray, pu_origin_x);

    uint32_t pu_origin_x_round = (pu_origin_x >> 3) << 3;
    uint32_t pu_origin_y_round = (pu_origin_y >> 3) << 3;

    uint32_t intra_chroma_mode_left_neighbor_index =
        get_neighbor_array_unit_left_index(intraChromaNeighborArray, pu_origin_y_round >> 1);
    uint32_t intra_chroma_mode_top_neighbor_index =
        get_neighbor_array_unit_top_index(intraChromaNeighborArray, pu_origin_x_round >> 1);

    (&blk_ptr->prediction_unit_array[0])->intra_luma_left_mode = (uint32_t)(
        (mode_type_neighbor_array->left_array[mode_type_left_neighbor_index] != INTRA_MODE)
            ? DC_PRED /*EB_INTRA_DC*/
            : (uint32_t)intraLumaNeighborArray->left_array[intra_luma_mode_left_neighbor_index]);

    (&blk_ptr->prediction_unit_array[0])->intra_luma_top_mode = (uint32_t)(
        (mode_type_neighbor_array->top_array[mode_type_top_neighbor_index] != INTRA_MODE)
            ? DC_PRED /*EB_INTRA_DC*/
            : (uint32_t)intraLumaNeighborArray->top_array
                  [intra_luma_mode_top_neighbor_index]); //   use DC. This seems like we could use a SB-width

    uint32_t mode_type_left_neighbor_index_round =
        get_neighbor_array_unit_left_index(mode_type_neighbor_array, pu_origin_y_round);
    uint32_t mode_type_top_neighbor_index_round =
        get_neighbor_array_unit_top_index(mode_type_neighbor_array, pu_origin_x_round);

    (&blk_ptr->prediction_unit_array[0])->intra_chroma_left_mode = (uint32_t)(
        (mode_type_neighbor_array->left_array[mode_type_left_neighbor_index_round] != INTRA_MODE)
            ? UV_DC_PRED
            : (uint32_t)
                  intraChromaNeighborArray->left_array[intra_chroma_mode_left_neighbor_index]);

    (&blk_ptr->prediction_unit_array[0])->intra_chroma_top_mode = (uint32_t)(
        (mode_type_neighbor_array->top_array[mode_type_top_neighbor_index_round] != INTRA_MODE)
            ? UV_DC_PRED
            : (uint32_t)intraChromaNeighborArray->top_array
                  [intra_chroma_mode_top_neighbor_index]); //   use DC. This seems like we could use a SB-width

    return;
}

void encode_pass_tx_search(PictureControlSet *pcs_ptr, EncDecContext *context_ptr,
                           SuperBlock *sb_ptr, uint32_t cb_qp,
                           EbPictureBufferDesc *coeff_samples_sb,
                           EbPictureBufferDesc *residual16bit, EbPictureBufferDesc *transform16bit,
                           EbPictureBufferDesc *inverse_quant_buffer,
                           int16_t *transform_scratch_buffer, uint32_t *count_non_zero_coeffs,
                           uint32_t component_mask, uint16_t *eob,
                           MacroblockPlane *candidate_plane);

/**********************************************************
* Encode Loop
*
* Summary: Performs an AV1 conformant
*   Transform, Quantization  and Inverse Quantization of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - input samples (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Inverse quantized coeff - quantization indices (position sensitive)
*
**********************************************************/
static void av1_encode_loop(PictureControlSet *pcs_ptr, EncDecContext *context_ptr,
                            SuperBlock *         sb_ptr,
                            uint32_t             origin_x, //pic based tx org x
                            uint32_t             origin_y, //pic based tx org y
                            uint32_t             cb_qp,
                            EbPictureBufferDesc *pred_samples, // no basis/offset
                            EbPictureBufferDesc *coeff_samples_sb, // sb based
                            EbPictureBufferDesc *residual16bit, // no basis/offset
                            EbPictureBufferDesc *transform16bit, // no basis/offset
                            EbPictureBufferDesc *inverse_quant_buffer,
                            int16_t *transform_scratch_buffer, uint32_t *count_non_zero_coeffs,
                            uint32_t component_mask, uint16_t *eob,
                            MacroblockPlane *candidate_plane) {
    (void)cb_qp;

    //    uint32_t                 chroma_qp = cb_qp;
    BlkStruct *   blk_ptr = context_ptr->blk_ptr;
    TransformUnit *txb_ptr = &blk_ptr->txb_array[context_ptr->txb_itr];
    //    EB_SLICE               slice_type = sb_ptr->pcs_ptr->slice_type;
    //    uint32_t                 temporal_layer_index = sb_ptr->pcs_ptr->temporal_layer_index;
    uint32_t             qp            = blk_ptr->qp;
    EbPictureBufferDesc *input_samples = context_ptr->input_samples;

    uint32_t round_origin_x = (origin_x >> 3) << 3; // for Chroma blocks with size of 4
    uint32_t round_origin_y = (origin_y >> 3) << 3; // for Chroma blocks with size of 4

    const uint32_t input_luma_offset =
        ((origin_y + input_samples->origin_y) * input_samples->stride_y) +
        (origin_x + input_samples->origin_x);
    const uint32_t input_cb_offset =
        (((round_origin_y + input_samples->origin_y) >> 1) * input_samples->stride_cb) +
        ((round_origin_x + input_samples->origin_x) >> 1);
    const uint32_t input_cr_offset =
        (((round_origin_y + input_samples->origin_y) >> 1) * input_samples->stride_cr) +
        ((round_origin_x + input_samples->origin_x) >> 1);
    const uint32_t pred_luma_offset =
        ((pred_samples->origin_y + origin_y) * pred_samples->stride_y) +
        (pred_samples->origin_x + origin_x);
    const uint32_t pred_cb_offset =
        (((pred_samples->origin_y + round_origin_y) >> 1) * pred_samples->stride_cb) +
        ((pred_samples->origin_x + round_origin_x) >> 1);
    const uint32_t pred_cr_offset =
        (((pred_samples->origin_y + round_origin_y) >> 1) * pred_samples->stride_cr) +
        ((pred_samples->origin_x + round_origin_x) >> 1);
#if TX_ORG_INTERINTRA
    int32_t is_inter = (blk_ptr->prediction_mode_flag == INTER_MODE || blk_ptr->av1xd->use_intrabc)
                           ? EB_TRUE
                           : EB_FALSE;
    const uint32_t scratch_luma_offset =
        context_ptr->blk_geom->tx_org_x[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr] +
        context_ptr->blk_geom->tx_org_y[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr] *
            SB_STRIDE_Y;
    const uint32_t scratch_cb_offset =
        ROUND_UV(
            context_ptr->blk_geom->tx_org_x[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 +
        ROUND_UV(
            context_ptr->blk_geom->tx_org_y[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 * SB_STRIDE_UV;
    const uint32_t scratch_cr_offset =
        ROUND_UV(
            context_ptr->blk_geom->tx_org_x[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 +
        ROUND_UV(
            context_ptr->blk_geom->tx_org_y[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 * SB_STRIDE_UV;
#else
    const uint32_t scratch_luma_offset =
        context_ptr->blk_geom->tx_org_x[blk_ptr->tx_depth][context_ptr->txb_itr] +
        context_ptr->blk_geom->tx_org_y[blk_ptr->tx_depth][context_ptr->txb_itr] * SB_STRIDE_Y;
    const uint32_t scratch_cb_offset =
        ROUND_UV(context_ptr->blk_geom->tx_org_x[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 +
        ROUND_UV(context_ptr->blk_geom->tx_org_y[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 *
            SB_STRIDE_UV;
    const uint32_t scratch_cr_offset =
        ROUND_UV(context_ptr->blk_geom->tx_org_x[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 +
        ROUND_UV(context_ptr->blk_geom->tx_org_y[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 *
            SB_STRIDE_UV;
#endif
    const uint32_t coeff1d_offset = context_ptr->coded_area_sb;

    const uint32_t coeff1d_offset_chroma = context_ptr->coded_area_sb_uv;
    UNUSED(coeff1d_offset_chroma);

    context_ptr->three_quad_energy = 0;
    //**********************************
    // Luma
    //**********************************
    if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK ||
        component_mask == PICTURE_BUFFER_DESC_LUMA_MASK) {
        residual_kernel8bit(
            input_samples->buffer_y + input_luma_offset,
            input_samples->stride_y,
            pred_samples->buffer_y + pred_luma_offset,
            pred_samples->stride_y,
            ((int16_t *)residual16bit->buffer_y) + scratch_luma_offset,
            residual16bit->stride_y,
            context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr]);
        uint8_t tx_search_skip_flag =
            context_ptr->md_context->tx_search_level == TX_SEARCH_ENC_DEC
                ? get_skip_tx_search_flag(context_ptr->blk_geom->sq_size, MAX_MODE_COST, 0, 1)
                : 1;

        if (!tx_search_skip_flag) {
            encode_pass_tx_search(pcs_ptr,
                                  context_ptr,
                                  sb_ptr,
                                  cb_qp,
                                  coeff_samples_sb,
                                  residual16bit,
                                  transform16bit,
                                  inverse_quant_buffer,
                                  transform_scratch_buffer,
                                  count_non_zero_coeffs,
                                  component_mask,
                                  eob,
                                  candidate_plane);
        }

        av1_estimate_transform(
            ((int16_t *)residual16bit->buffer_y) + scratch_luma_offset,
            residual16bit->stride_y,
            ((TranLow *)transform16bit->buffer_y) + coeff1d_offset,
            NOT_USED_VALUE,
            context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
            &context_ptr->three_quad_energy,
            transform_scratch_buffer,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[PLANE_TYPE_Y],
            PLANE_TYPE_Y,
            DEFAULT_SHAPE);

        int32_t seg_qp = pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled
                             ? pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params
                                   .feature_data[context_ptr->blk_ptr->segment_id][SEG_LVL_ALT_Q]
                             : 0;

        blk_ptr->quantized_dc[0][context_ptr->txb_itr] = av1_quantize_inv_quantize(
            sb_ptr->pcs_ptr,
            context_ptr->md_context,
            ((TranLow *)transform16bit->buffer_y) + coeff1d_offset,
            NOT_USED_VALUE,
            ((int32_t *)coeff_samples_sb->buffer_y) + coeff1d_offset,
            ((int32_t *)inverse_quant_buffer->buffer_y) + coeff1d_offset,
            qp,
            seg_qp,
            context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
            &eob[0],
            &(count_non_zero_coeffs[0]),
            COMPONENT_LUMA,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[PLANE_TYPE_Y],
            &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]),
            context_ptr->md_context->luma_txb_skip_context,
            context_ptr->md_context->luma_dc_sign_context,
            blk_ptr->pred_mode,
            blk_ptr->av1xd->use_intrabc,
            EB_TRUE);

        if (context_ptr->md_skip_blk) {
            count_non_zero_coeffs[0] = 0;
            eob[0]                   = 0;
        }
        txb_ptr->y_has_coeff = count_non_zero_coeffs[0] ? EB_TRUE : EB_FALSE;

        if (count_non_zero_coeffs[0] == 0) {
            // INTER. Chroma follows Luma in transform type
            if (blk_ptr->prediction_mode_flag == INTER_MODE) {
                txb_ptr->transform_type[PLANE_TYPE_Y]  = DCT_DCT;
                txb_ptr->transform_type[PLANE_TYPE_UV] = DCT_DCT;
            } else { // INTRA
                txb_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
            }
        }
        txb_ptr->nz_coef_count[0] = (uint16_t)count_non_zero_coeffs[0];
    }

    if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK ||
        component_mask == PICTURE_BUFFER_DESC_CHROMA_MASK) {
        if (blk_ptr->prediction_mode_flag == INTRA_MODE &&
            (context_ptr->evaluate_cfl_ep ||
             blk_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED)) {
            EbPictureBufferDesc *recon_samples = pred_samples;
            uint32_t             recon_luma_offset =
                (recon_samples->origin_y + round_origin_y) * recon_samples->stride_y +
                (recon_samples->origin_x + round_origin_x);

            // Down sample Luma
            cfl_luma_subsampling_420_lbd_c(
                recon_samples->buffer_y + recon_luma_offset,
                recon_samples->stride_y,
                context_ptr->md_context->pred_buf_q3,
                context_ptr->blk_geom->bwidth_uv == context_ptr->blk_geom->bwidth
                    ? (context_ptr->blk_geom->bwidth_uv << 1)
                    : context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight_uv == context_ptr->blk_geom->bheight
                    ? (context_ptr->blk_geom->bheight_uv << 1)
                    : context_ptr->blk_geom->bheight);
            int32_t round_offset =
                ((context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr]) *
                 (context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr])) /
                2;

            eb_subtract_average(
                context_ptr->md_context->pred_buf_q3,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                round_offset,
                LOG2F(context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr]) +
                    LOG2F(context_ptr->blk_geom
                              ->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]));

            if (context_ptr->evaluate_cfl_ep) {
                // 3: Loop over alphas and find the best or choose DC
                // Use the 1st spot of the candidate buffer to hold cfl settings: (1) to use same kernel as MD for CFL evaluation: cfl_rd_pick_alpha() (toward unification), (2) to avoid dedicated buffers for CFL evaluation @ EP (toward less memory)
                ModeDecisionCandidateBuffer *candidate_buffer =
                    &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]);

                // Input(s)
                candidate_buffer->candidate_ptr->type            = INTRA_MODE;
                candidate_buffer->candidate_ptr->intra_luma_mode = blk_ptr->pred_mode;
                candidate_buffer->candidate_ptr->cfl_alpha_signs = 0;
                candidate_buffer->candidate_ptr->cfl_alpha_idx   = 0;
                context_ptr->md_context->blk_geom                = context_ptr->blk_geom;

                EbByte src_pred_ptr;
                EbByte dst_pred_ptr;

                // Copy Cb pred samples from ep buffer to md buffer
                src_pred_ptr = pred_samples->buffer_cb + pred_cb_offset;
                dst_pred_ptr = &(candidate_buffer->prediction_ptr->buffer_cb[scratch_cb_offset]);
                for (int i = 0; i < context_ptr->blk_geom->bheight_uv; i++) {
                    memcpy(dst_pred_ptr, src_pred_ptr, context_ptr->blk_geom->bwidth_uv);
                    src_pred_ptr += pred_samples->stride_cb;
                    dst_pred_ptr += candidate_buffer->prediction_ptr->stride_cb;
                }

                // Copy Cr pred samples from ep buffer to md buffer
                src_pred_ptr = pred_samples->buffer_cr + pred_cr_offset;
                dst_pred_ptr = &(candidate_buffer->prediction_ptr->buffer_cr[scratch_cr_offset]);
                for (int i = 0; i < context_ptr->blk_geom->bheight_uv; i++) {
                    memcpy(dst_pred_ptr, src_pred_ptr, context_ptr->blk_geom->bwidth_uv);
                    src_pred_ptr += pred_samples->stride_cr;
                    dst_pred_ptr += candidate_buffer->prediction_ptr->stride_cr;
                }

                cfl_rd_pick_alpha(pcs_ptr,
                                  candidate_buffer,
                                  sb_ptr,
                                  context_ptr->md_context,
                                  input_samples,
                                  input_cb_offset,
                                  scratch_cb_offset);

                // Output(s)
                if (candidate_buffer->candidate_ptr->intra_chroma_mode == UV_CFL_PRED) {
                    blk_ptr->prediction_unit_array->intra_chroma_mode = UV_CFL_PRED;
                    blk_ptr->prediction_unit_array->cfl_alpha_idx =
                        candidate_buffer->candidate_ptr->cfl_alpha_idx;
                    blk_ptr->prediction_unit_array->cfl_alpha_signs =
                        candidate_buffer->candidate_ptr->cfl_alpha_signs;
                    blk_ptr->prediction_unit_array->is_directional_chroma_mode_flag = EB_FALSE;
                }
            }

            if (blk_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED) {
                int32_t alpha_q3 = cfl_idx_to_alpha(blk_ptr->prediction_unit_array->cfl_alpha_idx,
                                                    blk_ptr->prediction_unit_array->cfl_alpha_signs,
                                                    CFL_PRED_U); // once for U, once for V

                //TOCHANGE
                //assert(chroma_size * CFL_BUF_LINE + chroma_size <= CFL_BUF_SQUARE);

                eb_cfl_predict_lbd(
                    context_ptr->md_context->pred_buf_q3,
                    pred_samples->buffer_cb + pred_cb_offset,
                    pred_samples->stride_cb,
                    pred_samples->buffer_cb + pred_cb_offset,
                    pred_samples->stride_cb,
                    alpha_q3,
                    8,
                    context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);
                alpha_q3 = cfl_idx_to_alpha(blk_ptr->prediction_unit_array->cfl_alpha_idx,
                                            blk_ptr->prediction_unit_array->cfl_alpha_signs,
                                            CFL_PRED_V); // once for U, once for V

                //TOCHANGE
                //assert(chroma_size * CFL_BUF_LINE + chroma_size <= CFL_BUF_SQUARE);

                eb_cfl_predict_lbd(
                    context_ptr->md_context->pred_buf_q3,
                    pred_samples->buffer_cr + pred_cr_offset,
                    pred_samples->stride_cr,
                    pred_samples->buffer_cr + pred_cr_offset,
                    pred_samples->stride_cr,
                    alpha_q3,
                    8,
                    context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);
            }
        }

        //**********************************
        // Cb
        //**********************************

        residual_kernel8bit(
            input_samples->buffer_cb + input_cb_offset,
            input_samples->stride_cb,
            pred_samples->buffer_cb + pred_cb_offset,
            pred_samples->stride_cb,
            ((int16_t *)residual16bit->buffer_cb) + scratch_cb_offset,
            residual16bit->stride_cb,
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);

        residual_kernel8bit(
            input_samples->buffer_cr + input_cr_offset,
            input_samples->stride_cr,
            pred_samples->buffer_cr + pred_cr_offset,
            pred_samples->stride_cr,
            ((int16_t *)residual16bit->buffer_cr) + scratch_cr_offset,
            residual16bit->stride_cr,
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);

        av1_estimate_transform(
            ((int16_t *)residual16bit->buffer_cb) + scratch_cb_offset,
            residual16bit->stride_cb,
            ((TranLow *)transform16bit->buffer_cb) + context_ptr->coded_area_sb_uv,
            NOT_USED_VALUE,
            context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            &context_ptr->three_quad_energy,
            transform_scratch_buffer,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[PLANE_TYPE_UV],
            PLANE_TYPE_UV,
            DEFAULT_SHAPE);

        int32_t seg_qp = pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled
                             ? pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params
                                   .feature_data[context_ptr->blk_ptr->segment_id][SEG_LVL_ALT_Q]
                             : 0;

        blk_ptr->quantized_dc[1][context_ptr->txb_itr] = av1_quantize_inv_quantize(
            sb_ptr->pcs_ptr,
            context_ptr->md_context,
            ((TranLow *)transform16bit->buffer_cb) + context_ptr->coded_area_sb_uv,
            NOT_USED_VALUE,
            ((int32_t *)coeff_samples_sb->buffer_cb) + context_ptr->coded_area_sb_uv,
            ((int32_t *)inverse_quant_buffer->buffer_cb) + context_ptr->coded_area_sb_uv,
            qp,
            seg_qp,
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            &eob[1],
            &(count_non_zero_coeffs[1]),
            COMPONENT_CHROMA_CB,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[PLANE_TYPE_UV],
            &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]),
            context_ptr->md_context->cb_txb_skip_context,
            context_ptr->md_context->cb_dc_sign_context,
            blk_ptr->pred_mode,
            blk_ptr->av1xd->use_intrabc,
            EB_TRUE);

        if (context_ptr->md_skip_blk) {
            count_non_zero_coeffs[1] = 0;
            eob[1]                   = 0;
        }
        txb_ptr->u_has_coeff = count_non_zero_coeffs[1] ? EB_TRUE : EB_FALSE;

        //**********************************
        // Cr
        //**********************************

        av1_estimate_transform(
            ((int16_t *)residual16bit->buffer_cr) + scratch_cb_offset,
            residual16bit->stride_cr,
            ((TranLow *)transform16bit->buffer_cr) + context_ptr->coded_area_sb_uv,
            NOT_USED_VALUE,
            context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            &context_ptr->three_quad_energy,
            transform_scratch_buffer,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[PLANE_TYPE_UV],
            PLANE_TYPE_UV,
            DEFAULT_SHAPE);
        blk_ptr->quantized_dc[2][context_ptr->txb_itr] = av1_quantize_inv_quantize(
            sb_ptr->pcs_ptr,
            context_ptr->md_context,
            ((TranLow *)transform16bit->buffer_cr) + context_ptr->coded_area_sb_uv,
            NOT_USED_VALUE,
            ((int32_t *)coeff_samples_sb->buffer_cr) + context_ptr->coded_area_sb_uv,
            ((TranLow *)inverse_quant_buffer->buffer_cr) + context_ptr->coded_area_sb_uv,
            qp,
            seg_qp,
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            &eob[2],
            &(count_non_zero_coeffs[2]),
            COMPONENT_CHROMA_CR,
            BIT_INCREMENT_8BIT,
            txb_ptr->transform_type[PLANE_TYPE_UV],
            &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]),
            context_ptr->md_context->cr_txb_skip_context,
            context_ptr->md_context->cr_dc_sign_context,
            blk_ptr->pred_mode,
            blk_ptr->av1xd->use_intrabc,
            EB_TRUE);
        if (context_ptr->md_skip_blk) {
            count_non_zero_coeffs[2] = 0;
            eob[2]                   = 0;
        }
        txb_ptr->v_has_coeff = count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;

        txb_ptr->nz_coef_count[1] = (uint16_t)count_non_zero_coeffs[1];
        txb_ptr->nz_coef_count[2] = (uint16_t)count_non_zero_coeffs[2];
    }
    return;
}

void encode_pass_tx_search_hbd(
    PictureControlSet *pcs_ptr, EncDecContext *context_ptr, SuperBlock *sb_ptr, uint32_t cb_qp,
    EbPictureBufferDesc *coeff_samples_sb, EbPictureBufferDesc *residual16bit,
    EbPictureBufferDesc *transform16bit, EbPictureBufferDesc *inverse_quant_buffer,
    int16_t *transform_scratch_buffer, uint32_t *count_non_zero_coeffs, uint32_t component_mask,
    uint16_t *eob, MacroblockPlane *candidate_plane);

/**********************************************************
* Encode Loop
*
* Summary: Performs an AV1 conformant
*   Transform, Quantization  and Inverse Quantization of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - input samples (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Inverse quantized coeff - quantization indices (position sensitive)
*
**********************************************************/
static void av1_encode_loop_16bit(PictureControlSet *pcs_ptr, EncDecContext *context_ptr,
                                  SuperBlock *sb_ptr, uint32_t origin_x, uint32_t origin_y,
                                  uint32_t             cb_qp,
                                  EbPictureBufferDesc *pred_samples, // no basis/offset
                                  EbPictureBufferDesc *coeff_samples_sb, // sb based
                                  EbPictureBufferDesc *residual16bit, // no basis/offset
                                  EbPictureBufferDesc *transform16bit, // no basis/offset
                                  EbPictureBufferDesc *inverse_quant_buffer,
                                  int16_t *            transform_scratch_buffer,
                                  uint32_t *count_non_zero_coeffs, uint32_t component_mask,
                                  uint16_t *eob, MacroblockPlane *candidate_plane)

{
    (void)cb_qp;

    BlkStruct *   blk_ptr = context_ptr->blk_ptr;
    TransformUnit *txb_ptr = &blk_ptr->txb_array[context_ptr->txb_itr];
    //    EB_SLICE               slice_type = sb_ptr->pcs_ptr->slice_type;
    //    uint32_t                 temporal_layer_index = sb_ptr->pcs_ptr->temporal_layer_index;
    uint32_t qp = blk_ptr->qp;

    EbPictureBufferDesc *input_samples16bit = context_ptr->input_sample16bit_buffer;
    EbPictureBufferDesc *pred_samples16bit  = pred_samples;
    uint32_t             round_origin_x = (origin_x >> 3) << 3; // for Chroma blocks with size of 4
    uint32_t             round_origin_y = (origin_y >> 3) << 3; // for Chroma blocks with size of 4
#if TX_ORG_INTERINTRA
    int32_t is_inter = (blk_ptr->prediction_mode_flag == INTER_MODE || blk_ptr->av1xd->use_intrabc)
                           ? EB_TRUE
                           : EB_FALSE;
    const uint32_t input_luma_offset =
        context_ptr->blk_geom->tx_org_x[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr] +
        context_ptr->blk_geom->tx_org_y[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr] *
            SB_STRIDE_Y;
    const uint32_t input_cb_offset =
        ROUND_UV(
            context_ptr->blk_geom->tx_org_x[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 +
        ROUND_UV(
            context_ptr->blk_geom->tx_org_y[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 * SB_STRIDE_UV;
    const uint32_t input_cr_offset =
        ROUND_UV(
            context_ptr->blk_geom->tx_org_x[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 +
        ROUND_UV(
            context_ptr->blk_geom->tx_org_y[is_inter][blk_ptr->tx_depth][context_ptr->txb_itr]) /
            2 * SB_STRIDE_UV;
#else
    const uint32_t input_luma_offset =
        context_ptr->blk_geom->tx_org_x[blk_ptr->tx_depth][context_ptr->txb_itr] +
        context_ptr->blk_geom->tx_org_y[blk_ptr->tx_depth][context_ptr->txb_itr] * SB_STRIDE_Y;
    const uint32_t input_cb_offset =
        ROUND_UV(context_ptr->blk_geom->tx_org_x[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 +
        ROUND_UV(context_ptr->blk_geom->tx_org_y[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 *
            SB_STRIDE_UV;
    const uint32_t input_cr_offset =
        ROUND_UV(context_ptr->blk_geom->tx_org_x[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 +
        ROUND_UV(context_ptr->blk_geom->tx_org_y[blk_ptr->tx_depth][context_ptr->txb_itr]) / 2 *
            SB_STRIDE_UV;
#endif
    const uint32_t pred_luma_offset =
        ((pred_samples16bit->origin_y + origin_y) * pred_samples16bit->stride_y) +
        (pred_samples16bit->origin_x + origin_x);
    const uint32_t pred_cb_offset =
        (((pred_samples16bit->origin_y + round_origin_y) >> 1) * pred_samples16bit->stride_cb) +
        ((pred_samples16bit->origin_x + round_origin_x) >> 1);
    const uint32_t pred_cr_offset =
        (((pred_samples16bit->origin_y + round_origin_y) >> 1) * pred_samples16bit->stride_cr) +
        ((pred_samples16bit->origin_x + round_origin_x) >> 1);
    const uint32_t scratch_luma_offset =
        context_ptr->blk_geom->origin_x + context_ptr->blk_geom->origin_y * SB_STRIDE_Y;
    const uint32_t scratch_cb_offset = ROUND_UV(context_ptr->blk_geom->origin_x) / 2 +
                                       ROUND_UV(context_ptr->blk_geom->origin_y) / 2 * SB_STRIDE_UV;
    const uint32_t scratch_cr_offset = ROUND_UV(context_ptr->blk_geom->origin_x) / 2 +
                                       ROUND_UV(context_ptr->blk_geom->origin_y) / 2 * SB_STRIDE_UV;
    const uint32_t coeff1d_offset        = context_ptr->coded_area_sb;
    const uint32_t coeff1d_offset_chroma = context_ptr->coded_area_sb_uv;
    UNUSED(coeff1d_offset_chroma);

    {
        //**********************************
        // Luma
        //**********************************
        if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK ||
            component_mask == PICTURE_BUFFER_DESC_LUMA_MASK) {
            residual_kernel16bit(
                ((uint16_t *)input_samples16bit->buffer_y) + input_luma_offset,
                input_samples16bit->stride_y,
                ((uint16_t *)pred_samples16bit->buffer_y) + pred_luma_offset,
                pred_samples16bit->stride_y,
                ((int16_t *)residual16bit->buffer_y) + scratch_luma_offset,
                residual16bit->stride_y,
                context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr]);
            uint8_t tx_search_skip_flag =
                context_ptr->md_context->tx_search_level == TX_SEARCH_ENC_DEC
                    ? get_skip_tx_search_flag(context_ptr->blk_geom->sq_size, MAX_MODE_COST, 0, 1)
                    : 1;

            if (!tx_search_skip_flag) {
                encode_pass_tx_search_hbd(pcs_ptr,
                                          context_ptr,
                                          sb_ptr,
                                          cb_qp,
                                          coeff_samples_sb,
                                          residual16bit,
                                          transform16bit,
                                          inverse_quant_buffer,
                                          transform_scratch_buffer,
                                          count_non_zero_coeffs,
                                          component_mask,
                                          eob,
                                          candidate_plane);
            }

            av1_estimate_transform(
                ((int16_t *)residual16bit->buffer_y) + scratch_luma_offset,
                residual16bit->stride_y,
                ((TranLow *)transform16bit->buffer_y) + coeff1d_offset,
                NOT_USED_VALUE,
                context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transform_scratch_buffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_Y],
                PLANE_TYPE_Y,
                DEFAULT_SHAPE);

            int32_t seg_qp =
                pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled
                    ? pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params
                          .feature_data[context_ptr->blk_ptr->segment_id][SEG_LVL_ALT_Q]
                    : 0;
            blk_ptr->quantized_dc[0][context_ptr->txb_itr] = av1_quantize_inv_quantize(
                sb_ptr->pcs_ptr,
                context_ptr->md_context,
                ((int32_t *)transform16bit->buffer_y) + coeff1d_offset,
                NOT_USED_VALUE,
                ((int32_t *)coeff_samples_sb->buffer_y) + coeff1d_offset,
                ((int32_t *)inverse_quant_buffer->buffer_y) + coeff1d_offset,
                qp,
                seg_qp,
                context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                &eob[0],
                &(count_non_zero_coeffs[0]),
                COMPONENT_LUMA,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_Y],
                &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]),
                context_ptr->md_context->luma_txb_skip_context,
                context_ptr->md_context->luma_dc_sign_context,
                blk_ptr->pred_mode,
                blk_ptr->av1xd->use_intrabc,
                EB_TRUE);
            if (context_ptr->md_skip_blk) {
                count_non_zero_coeffs[0] = 0;
                eob[0]                   = 0;
            }
            txb_ptr->y_has_coeff = count_non_zero_coeffs[0] ? EB_TRUE : EB_FALSE;
            if (count_non_zero_coeffs[0] == 0) {
                // INTER. Chroma follows Luma in transform type
                if (blk_ptr->prediction_mode_flag == INTER_MODE) {
                    txb_ptr->transform_type[PLANE_TYPE_Y]  = DCT_DCT;
                    txb_ptr->transform_type[PLANE_TYPE_UV] = DCT_DCT;
                } else { // INTRA
                    txb_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
                }
            }

            txb_ptr->nz_coef_count[0] = (uint16_t)count_non_zero_coeffs[0];
        }

        if (blk_ptr->prediction_mode_flag == INTRA_MODE &&
            blk_ptr->prediction_unit_array->intra_chroma_mode == UV_CFL_PRED) {
            EbPictureBufferDesc *recon_samples = pred_samples16bit;

            uint32_t recon_luma_offset =
                (recon_samples->origin_y + round_origin_y) * recon_samples->stride_y +
                (recon_samples->origin_x + round_origin_x);

            // Down sample Luma
            cfl_luma_subsampling_420_hbd_c(
                ((uint16_t *)recon_samples->buffer_y) + recon_luma_offset,
                recon_samples->stride_y,
                context_ptr->md_context->pred_buf_q3,
                context_ptr->blk_geom->bwidth_uv == context_ptr->blk_geom->bwidth
                    ? (context_ptr->blk_geom->bwidth_uv << 1)
                    : context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight_uv == context_ptr->blk_geom->bheight
                    ? (context_ptr->blk_geom->bheight_uv << 1)
                    : context_ptr->blk_geom->bheight);
            int32_t round_offset =
                ((context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr]) *
                 (context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr])) /
                2;

            eb_subtract_average(
                context_ptr->md_context->pred_buf_q3,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                round_offset,
                LOG2F(context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr]) +
                    LOG2F(context_ptr->blk_geom
                              ->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]));

            int32_t alpha_q3 = cfl_idx_to_alpha(blk_ptr->prediction_unit_array->cfl_alpha_idx,
                                                blk_ptr->prediction_unit_array->cfl_alpha_signs,
                                                CFL_PRED_U); // once for U, once for V
            // TOCHANGE
            // assert(chroma_size * CFL_BUF_LINE + chroma_size <=                CFL_BUF_SQUARE);

            eb_cfl_predict_hbd(
                context_ptr->md_context->pred_buf_q3,
                ((uint16_t *)pred_samples16bit->buffer_cb) + pred_cb_offset,
                pred_samples16bit->stride_cb,
                ((uint16_t *)pred_samples16bit->buffer_cb) + pred_cb_offset,
                pred_samples16bit->stride_cb,
                alpha_q3,
                10,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);

            alpha_q3 = cfl_idx_to_alpha(blk_ptr->prediction_unit_array->cfl_alpha_idx,
                                        blk_ptr->prediction_unit_array->cfl_alpha_signs,
                                        CFL_PRED_V); // once for U, once for V
            // TOCHANGE
            //assert(chroma_size * CFL_BUF_LINE + chroma_size <=                CFL_BUF_SQUARE);

            eb_cfl_predict_hbd(
                context_ptr->md_context->pred_buf_q3,
                ((uint16_t *)pred_samples16bit->buffer_cr) + pred_cr_offset,
                pred_samples16bit->stride_cr,
                ((uint16_t *)pred_samples16bit->buffer_cr) + pred_cr_offset,
                pred_samples16bit->stride_cr,
                alpha_q3,
                10,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);
        }

        if (component_mask == PICTURE_BUFFER_DESC_FULL_MASK ||
            component_mask == PICTURE_BUFFER_DESC_CHROMA_MASK) {
            //**********************************
            // Cb
            //**********************************
            residual_kernel16bit(
                ((uint16_t *)input_samples16bit->buffer_cb) + input_cb_offset,
                input_samples16bit->stride_cb,
                ((uint16_t *)pred_samples16bit->buffer_cb) + pred_cb_offset,
                pred_samples16bit->stride_cb,
                ((int16_t *)residual16bit->buffer_cb) + scratch_cb_offset,
                residual16bit->stride_cb,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);

            residual_kernel16bit(
                ((uint16_t *)input_samples16bit->buffer_cr) + input_cr_offset,
                input_samples16bit->stride_cr,
                ((uint16_t *)pred_samples16bit->buffer_cr) + pred_cr_offset,
                pred_samples16bit->stride_cr,
                ((int16_t *)residual16bit->buffer_cr) + scratch_cr_offset,
                residual16bit->stride_cr,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr]);

            av1_estimate_transform(
                ((int16_t *)residual16bit->buffer_cb) + scratch_cb_offset,
                residual16bit->stride_cb,
                ((TranLow *)transform16bit->buffer_cb) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transform_scratch_buffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                DEFAULT_SHAPE);
            int32_t seg_qp =
                pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled
                    ? pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params
                          .feature_data[context_ptr->blk_ptr->segment_id][SEG_LVL_ALT_Q]
                    : 0;

            blk_ptr->quantized_dc[1][context_ptr->txb_itr] = av1_quantize_inv_quantize(
                sb_ptr->pcs_ptr,
                context_ptr->md_context,
                ((int32_t *)transform16bit->buffer_cb) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
                ((int32_t *)coeff_samples_sb->buffer_cb) + context_ptr->coded_area_sb_uv,
                ((int32_t *)inverse_quant_buffer->buffer_cb) + context_ptr->coded_area_sb_uv,
                qp,
                seg_qp,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                &eob[1],
                &(count_non_zero_coeffs[1]),
                COMPONENT_CHROMA_CB,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]),
                context_ptr->md_context->cb_txb_skip_context,
                context_ptr->md_context->cb_dc_sign_context,
                blk_ptr->pred_mode,
                blk_ptr->av1xd->use_intrabc,
                EB_TRUE);

            if (context_ptr->md_skip_blk) {
                count_non_zero_coeffs[1] = 0;
                eob[1]                   = 0;
            }
            txb_ptr->u_has_coeff = count_non_zero_coeffs[1] ? EB_TRUE : EB_FALSE;

            //**********************************
            // Cr
            //**********************************

            av1_estimate_transform(
                ((int16_t *)residual16bit->buffer_cr) + scratch_cb_offset,
                residual16bit->stride_cr,
                ((TranLow *)transform16bit->buffer_cr) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                &context_ptr->three_quad_energy,
                transform_scratch_buffer,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                DEFAULT_SHAPE);

            blk_ptr->quantized_dc[2][context_ptr->txb_itr] = av1_quantize_inv_quantize(
                sb_ptr->pcs_ptr,
                context_ptr->md_context,
                ((int32_t *)transform16bit->buffer_cr) + context_ptr->coded_area_sb_uv,
                NOT_USED_VALUE,
                ((int32_t *)coeff_samples_sb->buffer_cr) + context_ptr->coded_area_sb_uv,
                ((int32_t *)inverse_quant_buffer->buffer_cr) + context_ptr->coded_area_sb_uv,
                qp,
                seg_qp,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                &eob[2],
                &(count_non_zero_coeffs[2]),
                COMPONENT_CHROMA_CR,
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                &(context_ptr->md_context->candidate_buffer_ptr_array[0][0]),
                context_ptr->md_context->cr_txb_skip_context,
                context_ptr->md_context->cr_dc_sign_context,
                blk_ptr->pred_mode,
                blk_ptr->av1xd->use_intrabc,
                EB_TRUE);
            if (context_ptr->md_skip_blk) {
                count_non_zero_coeffs[2] = 0;
                eob[2]                   = 0;
            }
            txb_ptr->v_has_coeff = count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;

            txb_ptr->nz_coef_count[1] = (uint16_t)count_non_zero_coeffs[1];
            txb_ptr->nz_coef_count[2] = (uint16_t)count_non_zero_coeffs[2];
        }
    }

    return;
}

/**********************************************************
* Encode Generate Recon
*
* Summary: Performs an AV1 conformant
*   Inverse Transform and generate
*   the reconstructed samples of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - Inverse Qunatized Coeff (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Recon  (position independent)
*
**********************************************************/
static void av1_encode_generate_recon(EncDecContext *context_ptr, uint32_t origin_x,
                                      uint32_t             origin_y,
                                      EbPictureBufferDesc *pred_samples, // no basis/offset
                                      EbPictureBufferDesc *residual16bit, // no basis/offset
                                      int16_t *transform_scratch_buffer, uint32_t component_mask,
                                      uint16_t *eob) {
    uint32_t       pred_luma_offset;
    uint32_t       pred_chroma_offset;
    BlkStruct *   blk_ptr = context_ptr->blk_ptr;
    TransformUnit *txb_ptr = &blk_ptr->txb_array[context_ptr->txb_itr];

    // *Note - The prediction is built in-place in the Recon buffer. It is overwritten with Reconstructed
    //   samples if the CBF==1 && SKIP==False

    //**********************************
    // Luma
    //**********************************
    if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
        {
            pred_luma_offset = (pred_samples->origin_y + origin_y) * pred_samples->stride_y +
                               (pred_samples->origin_x + origin_x);
            if (txb_ptr->y_has_coeff == EB_TRUE && blk_ptr->skip_flag == EB_FALSE) {
                (void)transform_scratch_buffer;
                uint8_t *pred_buffer = pred_samples->buffer_y + pred_luma_offset;
                av1_inv_transform_recon8bit(
                    ((int32_t *)residual16bit->buffer_y) + context_ptr->coded_area_sb,
                    pred_buffer,
                    pred_samples->stride_y,
                    pred_buffer,
                    pred_samples->stride_y,
                    context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                    txb_ptr->transform_type[PLANE_TYPE_Y],
                    PLANE_TYPE_Y,
                    eob[0],
                    0 /*lossless*/
                );
            }
        }
    }

    if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK) {
        //**********************************
        // Chroma
        //**********************************

        uint32_t round_origin_x = (origin_x >> 3) << 3; // for Chroma blocks with size of 4
        uint32_t round_origin_y = (origin_y >> 3) << 3; // for Chroma blocks with size of 4
        pred_chroma_offset =
            (((pred_samples->origin_y + round_origin_y) >> 1) * pred_samples->stride_cb) +
            ((pred_samples->origin_x + round_origin_x) >> 1);

        //**********************************
        // Cb
        //**********************************
        if (txb_ptr->u_has_coeff == EB_TRUE && blk_ptr->skip_flag == EB_FALSE) {
            uint8_t *pred_buffer = pred_samples->buffer_cb + pred_chroma_offset;

            av1_inv_transform_recon8bit(
                ((int32_t *)residual16bit->buffer_cb) + context_ptr->coded_area_sb_uv,
                pred_buffer,
                pred_samples->stride_cb,
                pred_buffer,
                pred_samples->stride_cb,
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[1],
                0 /*lossless*/);
        }

        //**********************************
        // Cr
        //**********************************
        pred_chroma_offset =
            (((pred_samples->origin_y + round_origin_y) >> 1) * pred_samples->stride_cr) +
            ((pred_samples->origin_x + round_origin_x) >> 1);

        if (txb_ptr->v_has_coeff == EB_TRUE && blk_ptr->skip_flag == EB_FALSE) {
            uint8_t *pred_buffer = pred_samples->buffer_cr + pred_chroma_offset;

            av1_inv_transform_recon8bit(
                ((int32_t *)residual16bit->buffer_cr) + context_ptr->coded_area_sb_uv,
                pred_buffer,
                pred_samples->stride_cr,
                pred_buffer,
                pred_samples->stride_cr,
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[2],
                0 /*lossless*/);
        }
    }

    return;
}

/**********************************************************
* Encode Generate Recon
*
* Summary: Performs an AV1 conformant
*   Inverse Transform and generate
*   the reconstructed samples of a TU.
*
* Inputs:
*   origin_x
*   origin_y
*   txb_size
*   sb_sz
*   input - Inverse Qunatized Coeff (position sensitive)
*   pred - prediction samples (position independent)
*
* Outputs:
*   Recon  (position independent)
*
**********************************************************/
static void av1_encode_generate_recon_16bit(EncDecContext *context_ptr, uint32_t origin_x,
                                            uint32_t             origin_y,
                                            EbPictureBufferDesc *pred_samples, // no basis/offset
                                            EbPictureBufferDesc *residual16bit, // no basis/offset
                                            int16_t *            transform_scratch_buffer,
                                            uint32_t component_mask, uint16_t *eob) {
    uint32_t pred_luma_offset;
    uint32_t pred_chroma_offset;

    BlkStruct *   blk_ptr = context_ptr->blk_ptr;
    TransformUnit *txb_ptr = &blk_ptr->txb_array[context_ptr->txb_itr];

    (void)transform_scratch_buffer;
    //**********************************
    // Luma
    //**********************************
    if (component_mask & PICTURE_BUFFER_DESC_LUMA_MASK) {
        {
            pred_luma_offset = (pred_samples->origin_y + origin_y) * pred_samples->stride_y +
                               (pred_samples->origin_x + origin_x);
            if (txb_ptr->y_has_coeff == EB_TRUE && blk_ptr->skip_flag == EB_FALSE) {
                uint16_t *pred_buffer = ((uint16_t *)pred_samples->buffer_y) + pred_luma_offset;
                av1_inv_transform_recon(
                    ((int32_t *)residual16bit->buffer_y) + context_ptr->coded_area_sb,
                    CONVERT_TO_BYTEPTR(pred_buffer),
                    pred_samples->stride_y,
                    CONVERT_TO_BYTEPTR(pred_buffer),
                    pred_samples->stride_y,
                    context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                    BIT_INCREMENT_10BIT,
                    txb_ptr->transform_type[PLANE_TYPE_Y],
                    PLANE_TYPE_Y,
                    eob[0],
                    0 /*lossless*/
                );
            }
        }
    }

    if (component_mask & PICTURE_BUFFER_DESC_CHROMA_MASK) {
        //**********************************
        // Chroma
        //**********************************

        //**********************************
        // Cb
        //**********************************

        uint32_t round_origin_x = (origin_x >> 3) << 3; // for Chroma blocks with size of 4
        uint32_t round_origin_y = (origin_y >> 3) << 3; // for Chroma blocks with size of 4

        pred_chroma_offset =
            (((pred_samples->origin_y + round_origin_y) >> 1) * pred_samples->stride_cb) +
            ((pred_samples->origin_x + round_origin_x) >> 1);

        if (txb_ptr->u_has_coeff == EB_TRUE && blk_ptr->skip_flag == EB_FALSE) {
            uint16_t *pred_buffer = ((uint16_t *)pred_samples->buffer_cb) + pred_chroma_offset;
            av1_inv_transform_recon(
                ((int32_t *)residual16bit->buffer_cb) + context_ptr->coded_area_sb_uv,
                CONVERT_TO_BYTEPTR(pred_buffer),
                pred_samples->stride_cb,
                CONVERT_TO_BYTEPTR(pred_buffer),
                pred_samples->stride_cb,
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[1],
                0 /*lossless*/);
        }

        //**********************************
        // Cr
        //**********************************
        pred_chroma_offset =
            (((pred_samples->origin_y + round_origin_y) >> 1) * pred_samples->stride_cr) +
            ((pred_samples->origin_x + round_origin_x) >> 1);
        if (txb_ptr->v_has_coeff == EB_TRUE && blk_ptr->skip_flag == EB_FALSE) {
            uint16_t *pred_buffer = ((uint16_t *)pred_samples->buffer_cr) + pred_chroma_offset;
            av1_inv_transform_recon(
                ((int32_t *)residual16bit->buffer_cr) + context_ptr->coded_area_sb_uv,
                CONVERT_TO_BYTEPTR(pred_buffer),
                pred_samples->stride_cr,
                CONVERT_TO_BYTEPTR(pred_buffer),
                pred_samples->stride_cr,
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                BIT_INCREMENT_10BIT,
                txb_ptr->transform_type[PLANE_TYPE_UV],
                PLANE_TYPE_UV,
                eob[2],
                0 /*lossless*/);
        }
    }

    return;
}
static EbAv1EncodeLoopFuncPtr av1_encode_loop_func_table[2] = {av1_encode_loop,
                                                               av1_encode_loop_16bit};

EbAv1GenerateReconFuncPtr av1_enc_gen_recon_func_ptr[2] = {av1_encode_generate_recon,
                                                           av1_encode_generate_recon_16bit};

void store16bit_input_src(EbPictureBufferDesc *input_sample16bit_buffer, PictureControlSet *pcs_ptr,
                          uint32_t sb_x, uint32_t sb_y, uint32_t sb_w, uint32_t sb_h) {
    uint32_t  row_it;
    uint16_t *from_ptr;
    uint16_t *to_ptr;

    from_ptr = (uint16_t *)input_sample16bit_buffer->buffer_y;
    to_ptr   = (uint16_t *)pcs_ptr->input_frame16bit->buffer_y +
             (sb_x + pcs_ptr->input_frame16bit->origin_x) +
             (sb_y + pcs_ptr->input_frame16bit->origin_y) * pcs_ptr->input_frame16bit->stride_y;

    for (row_it = 0; row_it < sb_h; row_it++)
        memcpy(to_ptr + row_it * pcs_ptr->input_frame16bit->stride_y,
               from_ptr + row_it * input_sample16bit_buffer->stride_y,
               sb_w * 2);

    sb_x = sb_x / 2;
    sb_y = sb_y / 2;
    sb_w = sb_w / 2;
    sb_h = sb_h / 2;

    from_ptr = (uint16_t *)input_sample16bit_buffer->buffer_cb;
    to_ptr =
        (uint16_t *)pcs_ptr->input_frame16bit->buffer_cb +
        (sb_x + pcs_ptr->input_frame16bit->origin_x / 2) +
        (sb_y + pcs_ptr->input_frame16bit->origin_y / 2) * pcs_ptr->input_frame16bit->stride_cb;

    for (row_it = 0; row_it < sb_h; row_it++)
        memcpy(to_ptr + row_it * pcs_ptr->input_frame16bit->stride_cb,
               from_ptr + row_it * input_sample16bit_buffer->stride_cb,
               sb_w * 2);

    from_ptr = (uint16_t *)input_sample16bit_buffer->buffer_cr;
    to_ptr =
        (uint16_t *)pcs_ptr->input_frame16bit->buffer_cr +
        (sb_x + pcs_ptr->input_frame16bit->origin_x / 2) +
        (sb_y + pcs_ptr->input_frame16bit->origin_y / 2) * pcs_ptr->input_frame16bit->stride_cb;

    for (row_it = 0; row_it < sb_h; row_it++)
        memcpy(to_ptr + row_it * pcs_ptr->input_frame16bit->stride_cr,
               from_ptr + row_it * input_sample16bit_buffer->stride_cr,
               sb_w * 2);
}

void update_av1_mi_map(BlkStruct *blk_ptr, uint32_t blk_origin_x, uint32_t blk_origin_y,
                       const BlockGeom *blk_geom, PictureControlSet *pcs_ptr);

void move_blk_data(PictureControlSet *pcs, EncDecContext *context_ptr, BlkStruct *src_cu,
                   BlkStruct *dst_cu);

void perform_intra_coding_loop(PictureControlSet *pcs_ptr, SuperBlock *sb_ptr, uint32_t sb_addr,
                               BlkStruct *blk_ptr, PredictionUnit *pu_ptr,
                               EncDecContext *context_ptr) {
    EbBool is_16bit = context_ptr->is_16bit;

    EbPictureBufferDesc *recon_buffer =
        is_16bit ? pcs_ptr->recon_picture16bit_ptr : pcs_ptr->recon_picture_ptr;
    EbPictureBufferDesc *coeff_buffer_sb = sb_ptr->quantized_coeff;

#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
    NeighborArrayUnit *ep_luma_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_luma_recon_neighbor_array16bit[tile_idx]
                 : pcs_ptr->ep_luma_recon_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_cb_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cb_recon_neighbor_array16bit[tile_idx]
                 : pcs_ptr->ep_cb_recon_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_cr_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cr_recon_neighbor_array16bit[tile_idx]
                 : pcs_ptr->ep_cr_recon_neighbor_array[tile_idx];
#else
    NeighborArrayUnit *ep_luma_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_luma_recon_neighbor_array16bit
                 : pcs_ptr->ep_luma_recon_neighbor_array;
    NeighborArrayUnit *ep_cb_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cb_recon_neighbor_array16bit : pcs_ptr->ep_cb_recon_neighbor_array;
    NeighborArrayUnit *ep_cr_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cr_recon_neighbor_array16bit : pcs_ptr->ep_cr_recon_neighbor_array;
#endif

    EbPictureBufferDesc *residual_buffer           = context_ptr->residual_buffer;
    EbPictureBufferDesc *transform_buffer          = context_ptr->transform_buffer;
    EbPictureBufferDesc *inverse_quant_buffer      = context_ptr->inverse_quant_buffer;
    int16_t *            transform_inner_array_ptr = context_ptr->transform_inner_array_ptr;

    uint32_t        count_non_zero_coeffs[3];
    MacroblockPlane blk_plane[3];
    uint16_t        eobs[MAX_TXB_COUNT][3];
    uint64_t        y_txb_coeff_bits;
    uint64_t        cb_txb_coeff_bits;
    uint64_t        cr_txb_coeff_bits;
    EntropyCoder *  coeff_est_entropy_coder_ptr = pcs_ptr->coeff_est_entropy_coder_ptr;

    if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
        //get the 16bit form of the input SB
        if (is_16bit)
            recon_buffer = ((EbReferenceObject *)
                                pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                               ->reference_picture16bit;
        else
            recon_buffer = ((EbReferenceObject *)
                                pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                               ->reference_picture;
    else // non ref pictures
        recon_buffer = is_16bit ? pcs_ptr->recon_picture16bit_ptr : pcs_ptr->recon_picture_ptr;

    uint32_t tot_tu = context_ptr->blk_geom->txb_count[blk_ptr->tx_depth];

    // Luma path
    for (context_ptr->txb_itr = 0; context_ptr->txb_itr < tot_tu; context_ptr->txb_itr++) {
#if TX_ORG_INTERINTRA
        uint16_t txb_origin_x =
            context_ptr->blk_origin_x +
            context_ptr->blk_geom->tx_org_x[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
            context_ptr->blk_geom->origin_x;
        uint16_t txb_origin_y =
            context_ptr->blk_origin_y +
            context_ptr->blk_geom->tx_org_y[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
            context_ptr->blk_geom->origin_y;
#else
        uint16_t txb_origin_x =
            context_ptr->blk_origin_x +
            context_ptr->blk_geom->tx_boff_x[blk_ptr->tx_depth][context_ptr->txb_itr];
        uint16_t txb_origin_y =
            context_ptr->blk_origin_y +
            context_ptr->blk_geom->tx_boff_y[blk_ptr->tx_depth][context_ptr->txb_itr];
#endif
        context_ptr->md_context->luma_txb_skip_context = 0;
        context_ptr->md_context->luma_dc_sign_context  = 0;
        get_txb_ctx(pcs_ptr,
                    COMPONENT_LUMA,
#if TILES_PARALLEL
                    pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                    pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                    txb_origin_x,
                    txb_origin_y,
                    context_ptr->blk_geom->bsize,
                    context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                    &context_ptr->md_context->luma_txb_skip_context,
                    &context_ptr->md_context->luma_dc_sign_context);
        if (is_16bit) {
            uint16_t       top_neigh_array[64 * 2 + 1];
            uint16_t       left_neigh_array[64 * 2 + 1];
            PredictionMode mode;

            TxSize tx_size = context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr];

            if (txb_origin_y != 0)
                memcpy(top_neigh_array + 1,
                       (uint16_t *)(ep_luma_recon_neighbor_array->top_array) + txb_origin_x,
                       context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr] *
                           2 * sizeof(uint16_t));
            if (txb_origin_x != 0)
                memcpy(left_neigh_array + 1,
                       (uint16_t *)(ep_luma_recon_neighbor_array->left_array) + txb_origin_y,
                       context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr] *
                           2 * sizeof(uint16_t));
            if (txb_origin_y != 0 && txb_origin_x != 0)
                top_neigh_array[0] = left_neigh_array[0] =
                    ((uint16_t *)(ep_luma_recon_neighbor_array->top_left_array) +
                     MAX_PICTURE_HEIGHT_SIZE + txb_origin_x - txb_origin_y)[0];

            mode = blk_ptr->pred_mode;

            eb_av1_predict_intra_block_16bit(
                &sb_ptr->tile_info,
                ED_STAGE,
                context_ptr->blk_geom,
                pcs_ptr->parent_pcs_ptr->av1_cm,
                context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight,
                tx_size,
                mode,
                pu_ptr->angle_delta[PLANE_TYPE_Y],
                blk_ptr->palette_info.pmi.palette_size[0] > 0,
                &blk_ptr->palette_info,
                blk_ptr->filter_intra_mode,
                top_neigh_array + 1,
                left_neigh_array + 1,
                recon_buffer,
#if TX_ORG_INTERINTRA
                (context_ptr->blk_geom->tx_org_x[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                 context_ptr->blk_geom->origin_x) >>
                    2,
                (context_ptr->blk_geom->tx_org_y[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                 context_ptr->blk_geom->origin_y) >>
                    2,
#else
                context_ptr->blk_geom->tx_boff_x[blk_ptr->tx_depth][context_ptr->txb_itr] >> 2,
                context_ptr->blk_geom->tx_boff_y[blk_ptr->tx_depth][context_ptr->txb_itr] >> 2,
#endif
                0,
                context_ptr->blk_geom->bsize,
                txb_origin_x,
                txb_origin_y,
                context_ptr->blk_origin_x,
                context_ptr->blk_origin_y,
                0,
                0);
        } else {
            uint8_t        top_neigh_array[64 * 2 + 1];
            uint8_t        left_neigh_array[64 * 2 + 1];
            PredictionMode mode;

            TxSize tx_size = context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr];

            if (txb_origin_y != 0)
                memcpy(
                    top_neigh_array + 1,
                    ep_luma_recon_neighbor_array->top_array + txb_origin_x,
                    context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr] * 2);

            if (txb_origin_x != 0)
                memcpy(
                    left_neigh_array + 1,
                    ep_luma_recon_neighbor_array->left_array + txb_origin_y,
                    context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr] * 2);

            if (txb_origin_y != 0 && txb_origin_x != 0)
                top_neigh_array[0] = left_neigh_array[0] =
                    ep_luma_recon_neighbor_array
                        ->top_left_array[MAX_PICTURE_HEIGHT_SIZE + txb_origin_x - txb_origin_y];

            mode = blk_ptr->pred_mode;

            // Hsan: if CHROMA_MODE_2, then CFL will be evaluated @ EP as no CHROMA @ MD
            // If that's the case then you should ensure than the 1st chroma prediction uses UV_DC_PRED (that's the default configuration for CHROMA_MODE_2 if CFL applicable (set @ fast loop candidates injection) then MD assumes chroma mode always UV_DC_PRED)
            eb_av1_predict_intra_block(
                &sb_ptr->tile_info,
                ED_STAGE,
                context_ptr->blk_geom,
                pcs_ptr->parent_pcs_ptr->av1_cm,
                context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight,
                tx_size,
                mode,
                pu_ptr->angle_delta[PLANE_TYPE_Y],
                blk_ptr->palette_info.pmi.palette_size[0] > 0,
                &blk_ptr->palette_info,
                blk_ptr->filter_intra_mode,
                top_neigh_array + 1,
                left_neigh_array + 1,
                recon_buffer,
#if TX_ORG_INTERINTRA
                (context_ptr->blk_geom->tx_org_x[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                 context_ptr->blk_geom->origin_x) >>
                    2,
                (context_ptr->blk_geom->tx_org_y[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                 context_ptr->blk_geom->origin_y) >>
                    2,
#else
                context_ptr->blk_geom->tx_boff_x[blk_ptr->tx_depth][context_ptr->txb_itr] >> 2,
                context_ptr->blk_geom->tx_boff_y[blk_ptr->tx_depth][context_ptr->txb_itr] >> 2,
#endif
                0,
                context_ptr->blk_geom->bsize,
                txb_origin_x,
                txb_origin_y,
                context_ptr->blk_origin_x,
                context_ptr->blk_origin_y,
                0,
                0);
        }
        // Encode Transform Unit -INTRA-

        uint16_t cb_qp = blk_ptr->qp;
        av1_encode_loop_func_table[is_16bit](pcs_ptr,
                                             context_ptr,
                                             sb_ptr,
                                             txb_origin_x,
                                             txb_origin_y,
                                             cb_qp,
                                             recon_buffer,
                                             coeff_buffer_sb,
                                             residual_buffer,
                                             transform_buffer,
                                             inverse_quant_buffer,
                                             transform_inner_array_ptr,
                                             count_non_zero_coeffs,
                                             PICTURE_BUFFER_DESC_LUMA_MASK,
                                             eobs[context_ptr->txb_itr],
                                             blk_plane);

        if (pcs_ptr->update_cdf) {
            ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
                context_ptr->md_context->candidate_buffer_ptr_array;
            ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
                &(candidate_buffer_ptr_array_base[0]);
            ModeDecisionCandidateBuffer *candidate_buffer;

            // Set the Candidate Buffer
            candidate_buffer = candidate_buffer_ptr_array[0];
            // Rate estimation function uses the values from CandidatePtr. The right values are copied from blk_ptr to CandidatePtr
            candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr] =
                blk_ptr->txb_array[context_ptr->txb_itr].transform_type[PLANE_TYPE_Y];
            candidate_buffer->candidate_ptr->transform_type_uv =
                blk_ptr->txb_array[context_ptr->txb_itr].transform_type[PLANE_TYPE_UV];
            candidate_buffer->candidate_ptr->type              = blk_ptr->prediction_mode_flag;
            candidate_buffer->candidate_ptr->pred_mode         = blk_ptr->pred_mode;
            candidate_buffer->candidate_ptr->filter_intra_mode = blk_ptr->filter_intra_mode;
            const uint32_t coeff1d_offset                      = context_ptr->coded_area_sb;

            av1_txb_estimate_coeff_bits(
                context_ptr->md_context,
                1, //allow_update_cdf,
                &pcs_ptr->ec_ctx_array[sb_addr],
                pcs_ptr,
                candidate_buffer,
                coeff1d_offset,
                context_ptr->coded_area_sb_uv,
                coeff_est_entropy_coder_ptr,
                coeff_buffer_sb,
                eobs[context_ptr->txb_itr][0],
                eobs[context_ptr->txb_itr][1],
                eobs[context_ptr->txb_itr][2],
                &y_txb_coeff_bits,
                &cb_txb_coeff_bits,
                &cr_txb_coeff_bits,
                context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr],
                candidate_buffer->candidate_ptr->transform_type_uv,
                COMPONENT_LUMA);
        }

        av1_enc_gen_recon_func_ptr[is_16bit](context_ptr,
                                             txb_origin_x,
                                             txb_origin_y,
                                             recon_buffer,
                                             inverse_quant_buffer,
                                             transform_inner_array_ptr,
                                             PICTURE_BUFFER_DESC_LUMA_MASK,
                                             eobs[context_ptr->txb_itr]);

        // Update Recon Samples-INTRA-
        encode_pass_update_recon_sample_neighbour_arrays(
            ep_luma_recon_neighbor_array,
            ep_cb_recon_neighbor_array,
            ep_cr_recon_neighbor_array,
            recon_buffer,
            txb_origin_x,
            txb_origin_y,
            context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            PICTURE_BUFFER_DESC_LUMA_MASK,
            is_16bit);

        context_ptr->coded_area_sb +=
            context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr] *
            context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr];

        // Update the luma Dc Sign Level Coeff Neighbor Array
        {
            uint8_t dc_sign_level_coeff = (uint8_t)blk_ptr->quantized_dc[0][context_ptr->txb_itr];

            neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                (uint8_t *)&dc_sign_level_coeff,
                txb_origin_x,
                txb_origin_y,
                context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }

    } // Transform Loop

    // Chroma path

    if (context_ptr->blk_geom->has_uv) {
        context_ptr->txb_itr = 0;
#if TX_ORG_INTERINTRA
        uint16_t txb_origin_x =
            context_ptr->blk_origin_x +
            context_ptr->blk_geom->tx_org_x[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
            context_ptr->blk_geom->origin_x;
        uint16_t txb_origin_y =
            context_ptr->blk_origin_y +
            context_ptr->blk_geom->tx_org_y[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
            context_ptr->blk_geom->origin_y;
#else
        uint16_t txb_origin_x =
            context_ptr->blk_origin_x +
            context_ptr->blk_geom->tx_boff_x[blk_ptr->tx_depth][context_ptr->txb_itr];
        uint16_t txb_origin_y =
            context_ptr->blk_origin_y +
            context_ptr->blk_geom->tx_boff_y[blk_ptr->tx_depth][context_ptr->txb_itr];
#endif
        uint32_t blk_originx_uv = (context_ptr->blk_origin_x >> 3 << 3) >> 1;
        uint32_t blk_originy_uv = (context_ptr->blk_origin_y >> 3 << 3) >> 1;

        context_ptr->md_context->cb_txb_skip_context = 0;
        context_ptr->md_context->cb_dc_sign_context  = 0;
        get_txb_ctx(pcs_ptr,
                    COMPONENT_CHROMA,
#if TILES_PARALLEL
                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                    blk_originx_uv,
                    blk_originy_uv,
                    context_ptr->blk_geom->bsize_uv,
                    context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                    &context_ptr->md_context->cb_txb_skip_context,
                    &context_ptr->md_context->cb_dc_sign_context);

        context_ptr->md_context->cr_txb_skip_context = 0;
        context_ptr->md_context->cr_dc_sign_context  = 0;
        get_txb_ctx(pcs_ptr,
            COMPONENT_CHROMA,
#if TILES_PARALLEL
            pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
            pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
            blk_originx_uv,
            blk_originy_uv,
            context_ptr->blk_geom->bsize_uv,
            context_ptr->blk_geom->txsize_uv[context_ptr->blk_ptr->tx_depth][context_ptr->txb_itr],
            &context_ptr->md_context->cr_txb_skip_context,
            &context_ptr->md_context->cr_dc_sign_context);

        if (is_16bit) {
            uint16_t       top_neigh_array[64 * 2 + 1];
            uint16_t       left_neigh_array[64 * 2 + 1];
            PredictionMode mode;

            int32_t plane_end = 2;

            for (int32_t plane = 1; plane <= plane_end; ++plane) {
                TxSize tx_size =
                    plane
                        ? context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr]
                        : context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr];

                if (plane == 1) {
                    if (blk_originy_uv != 0)
                        memcpy(top_neigh_array + 1,
                               (uint16_t *)(ep_cb_recon_neighbor_array->top_array) + blk_originx_uv,
                               context_ptr->blk_geom->bwidth_uv * 2 * sizeof(uint16_t));
                    if (blk_originx_uv != 0)
                        memcpy(left_neigh_array + 1,
                               (uint16_t *)(ep_cb_recon_neighbor_array->left_array) + blk_originy_uv,
                               context_ptr->blk_geom->bheight_uv * 2 * sizeof(uint16_t));
                    if (blk_originy_uv != 0 && blk_originx_uv != 0)
                        top_neigh_array[0] = left_neigh_array[0] =
                            ((uint16_t *)(ep_cb_recon_neighbor_array->top_left_array) +
                             MAX_PICTURE_HEIGHT_SIZE / 2 + blk_originx_uv - blk_originy_uv)[0];
                } else if (plane == 2) {
                    if (blk_originy_uv != 0)
                        memcpy(top_neigh_array + 1,
                               (uint16_t *)(ep_cr_recon_neighbor_array->top_array) + blk_originx_uv,
                               context_ptr->blk_geom->bwidth_uv * 2 * sizeof(uint16_t));
                    if (blk_originx_uv != 0)
                        memcpy(left_neigh_array + 1,
                               (uint16_t *)(ep_cr_recon_neighbor_array->left_array) + blk_originy_uv,
                               context_ptr->blk_geom->bheight_uv * 2 * sizeof(uint16_t));
                    if (blk_originy_uv != 0 && blk_originx_uv != 0)
                        top_neigh_array[0] = left_neigh_array[0] =
                            ((uint16_t *)(ep_cr_recon_neighbor_array->top_left_array) +
                             MAX_PICTURE_HEIGHT_SIZE / 2 + blk_originx_uv - blk_originy_uv)[0];
                }

                mode = (pu_ptr->intra_chroma_mode == UV_CFL_PRED)
                           ? (PredictionMode)UV_DC_PRED
                           : (PredictionMode)pu_ptr->intra_chroma_mode;

                eb_av1_predict_intra_block_16bit(
                    &sb_ptr->tile_info,
                    ED_STAGE,
                    context_ptr->blk_geom,
                    pcs_ptr->parent_pcs_ptr->av1_cm,
                    plane ? context_ptr->blk_geom->bwidth_uv : context_ptr->blk_geom->bwidth,
                    plane ? context_ptr->blk_geom->bheight_uv : context_ptr->blk_geom->bheight,
                    tx_size,
                    mode,
                    plane ? pu_ptr->angle_delta[PLANE_TYPE_UV] : pu_ptr->angle_delta[PLANE_TYPE_Y],
                    0, //chroma
                    &blk_ptr->palette_info,
                    FILTER_INTRA_MODES,
                    top_neigh_array + 1,
                    left_neigh_array + 1,
                    recon_buffer,
#if TX_ORG_INTERINTRA
                    plane ? 0
                          : (context_ptr->blk_geom
                                 ->tx_org_x[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                             context_ptr->blk_geom->origin_x) >>
                                2,
                    plane ? 0
                          : (context_ptr->blk_geom
                                 ->tx_org_y[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                             context_ptr->blk_geom->origin_y) >>
                                2,
#else
                    plane ? 0
                          : context_ptr->blk_geom
                                    ->tx_boff_x[blk_ptr->tx_depth][context_ptr->txb_itr] >>
                                2,
                    plane ? 0
                          : context_ptr->blk_geom
                                    ->tx_boff_y[blk_ptr->tx_depth][context_ptr->txb_itr] >>
                                2,
#endif
                    plane,
                    context_ptr->blk_geom->bsize,
                    txb_origin_x,
                    txb_origin_y,
                    plane ? context_ptr->blk_origin_x : context_ptr->blk_origin_x,
                    plane ? context_ptr->blk_origin_y : context_ptr->blk_origin_y,
                    0,
                    0);
            }
        } else {
            uint8_t        top_neigh_array[64 * 2 + 1];
            uint8_t        left_neigh_array[64 * 2 + 1];
            PredictionMode mode;

            // Partition Loop
            int32_t plane_end = 2;

            for (int32_t plane = 1; plane <= plane_end; ++plane) {
                TxSize tx_size =
                    plane
                        ? context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr]
                        : context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr];

                if (plane == 1) {
                    if (blk_originy_uv != 0)
                        memcpy(top_neigh_array + 1,
                               ep_cb_recon_neighbor_array->top_array + blk_originx_uv,
                               context_ptr->blk_geom->bwidth_uv * 2);

                    if (blk_originx_uv != 0)
                        memcpy(left_neigh_array + 1,
                               ep_cb_recon_neighbor_array->left_array + blk_originy_uv,
                               context_ptr->blk_geom->bheight_uv * 2);

                    if (blk_originy_uv != 0 && blk_originx_uv != 0)
                        top_neigh_array[0] = left_neigh_array[0] =
                            ep_cb_recon_neighbor_array
                                ->top_left_array[MAX_PICTURE_HEIGHT_SIZE / 2 + blk_originx_uv -
                                                 blk_originy_uv];
                } else {
                    if (blk_originy_uv != 0)
                        memcpy(top_neigh_array + 1,
                               ep_cr_recon_neighbor_array->top_array + blk_originx_uv,
                               context_ptr->blk_geom->bwidth_uv * 2);

                    if (blk_originx_uv != 0)
                        memcpy(left_neigh_array + 1,
                               ep_cr_recon_neighbor_array->left_array + blk_originy_uv,
                               context_ptr->blk_geom->bheight_uv * 2);

                    if (blk_originy_uv != 0 && blk_originx_uv != 0)
                        top_neigh_array[0] = left_neigh_array[0] =
                            ep_cr_recon_neighbor_array
                                ->top_left_array[MAX_PICTURE_HEIGHT_SIZE / 2 + blk_originx_uv -
                                                 blk_originy_uv];
                }

                mode = (pu_ptr->intra_chroma_mode == UV_CFL_PRED)
                           ? (PredictionMode)UV_DC_PRED
                           : (PredictionMode)pu_ptr->intra_chroma_mode;

                // Hsan: if CHROMA_MODE_2, then CFL will be evaluated @ EP as no CHROMA @ MD
                // If that's the case then you should ensure than the 1st chroma prediction uses UV_DC_PRED (that's the default configuration for CHROMA_MODE_2 if CFL applicable (set @ fast loop candidates injection) then MD assumes chroma mode always UV_DC_PRED)
                eb_av1_predict_intra_block(
                    &sb_ptr->tile_info,
                    ED_STAGE,
                    context_ptr->blk_geom,
                    pcs_ptr->parent_pcs_ptr->av1_cm,
                    plane ? context_ptr->blk_geom->bwidth_uv : context_ptr->blk_geom->bwidth,
                    plane ? context_ptr->blk_geom->bheight_uv : context_ptr->blk_geom->bheight,
                    tx_size,
                    mode,
                    plane ? pu_ptr->angle_delta[PLANE_TYPE_UV] : pu_ptr->angle_delta[PLANE_TYPE_Y],
                    0, //chroma
                    &blk_ptr->palette_info,
                    FILTER_INTRA_MODES,
                    top_neigh_array + 1,
                    left_neigh_array + 1,
                    recon_buffer,
#if TX_ORG_INTERINTRA
                    plane ? 0
                          : (context_ptr->blk_geom
                                 ->tx_org_x[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                             context_ptr->blk_geom->origin_x) >>
                                2,
                    plane ? 0
                          : (context_ptr->blk_geom
                                 ->tx_org_y[0][blk_ptr->tx_depth][context_ptr->txb_itr] -
                             context_ptr->blk_geom->origin_y) >>
                                2,
#else
                    plane ? 0
                          : context_ptr->blk_geom
                                    ->tx_boff_x[blk_ptr->tx_depth][context_ptr->txb_itr] >>
                                2,
                    plane ? 0
                          : context_ptr->blk_geom
                                    ->tx_boff_y[blk_ptr->tx_depth][context_ptr->txb_itr] >>
                                2,
#endif
                    plane,
                    context_ptr->blk_geom->bsize,
                    txb_origin_x,
                    txb_origin_y,
                    plane ? context_ptr->blk_origin_x : context_ptr->blk_origin_x,
                    plane ? context_ptr->blk_origin_y : context_ptr->blk_origin_y,
                    0,
                    0);
            }
        }

        // Encode Transform Unit -INTRA-
        uint16_t cb_qp = blk_ptr->qp;

        av1_encode_loop_func_table[is_16bit](pcs_ptr,
                                             context_ptr,
                                             sb_ptr,
                                             txb_origin_x,
                                             txb_origin_y,
                                             cb_qp,
                                             recon_buffer,
                                             coeff_buffer_sb,
                                             residual_buffer,
                                             transform_buffer,
                                             inverse_quant_buffer,
                                             transform_inner_array_ptr,
                                             count_non_zero_coeffs,
                                             PICTURE_BUFFER_DESC_CHROMA_MASK,
                                             eobs[context_ptr->txb_itr],
                                             blk_plane);

        if (pcs_ptr->update_cdf) {
            ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
                context_ptr->md_context->candidate_buffer_ptr_array;
            ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
                &(candidate_buffer_ptr_array_base[0]);
            ModeDecisionCandidateBuffer *candidate_buffer;

            // Set the Candidate Buffer
            candidate_buffer = candidate_buffer_ptr_array[0];
            // Rate estimation function uses the values from CandidatePtr. The right values are copied from blk_ptr to CandidatePtr
            candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr] =
                blk_ptr->txb_array[context_ptr->txb_itr].transform_type[PLANE_TYPE_Y];
            candidate_buffer->candidate_ptr->transform_type_uv =
                blk_ptr->txb_array[context_ptr->txb_itr].transform_type[PLANE_TYPE_UV];
            candidate_buffer->candidate_ptr->type              = blk_ptr->prediction_mode_flag;
            candidate_buffer->candidate_ptr->pred_mode         = blk_ptr->pred_mode;
            candidate_buffer->candidate_ptr->filter_intra_mode = blk_ptr->filter_intra_mode;
            const uint32_t coeff1d_offset                      = context_ptr->coded_area_sb;

            av1_txb_estimate_coeff_bits(
                context_ptr->md_context,
                1, //allow_update_cdf,
                &pcs_ptr->ec_ctx_array[sb_addr],
                pcs_ptr,
                candidate_buffer,
                coeff1d_offset,
                context_ptr->coded_area_sb_uv,
                coeff_est_entropy_coder_ptr,
                coeff_buffer_sb,
                eobs[context_ptr->txb_itr][0],
                eobs[context_ptr->txb_itr][1],
                eobs[context_ptr->txb_itr][2],
                &y_txb_coeff_bits,
                &cb_txb_coeff_bits,
                &cr_txb_coeff_bits,
                context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr],
                candidate_buffer->candidate_ptr->transform_type_uv,
                COMPONENT_CHROMA);
        }

        av1_enc_gen_recon_func_ptr[is_16bit](context_ptr,
                                             txb_origin_x,
                                             txb_origin_y,
                                             recon_buffer,
                                             inverse_quant_buffer,
                                             transform_inner_array_ptr,
                                             PICTURE_BUFFER_DESC_CHROMA_MASK,
                                             eobs[context_ptr->txb_itr]);

        // Update Recon Samples-INTRA-
        encode_pass_update_recon_sample_neighbour_arrays(
            ep_luma_recon_neighbor_array,
            ep_cb_recon_neighbor_array,
            ep_cr_recon_neighbor_array,
            recon_buffer,
            txb_origin_x,
            txb_origin_y,
            context_ptr->blk_geom->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
            PICTURE_BUFFER_DESC_CHROMA_MASK,
            is_16bit);

        context_ptr->coded_area_sb_uv +=
            context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr] *
            context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr];

        // Update the cb Dc Sign Level Coeff Neighbor Array
        {
            uint8_t dc_sign_level_coeff = (uint8_t)blk_ptr->quantized_dc[1][context_ptr->txb_itr];
            neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                (uint8_t *)&dc_sign_level_coeff,
                ROUND_UV(txb_origin_x) >> 1,
                ROUND_UV(txb_origin_y) >> 1,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }

        // Update the cr DC Sign Level Coeff Neighbor Array
        {
            uint8_t dc_sign_level_coeff = (uint8_t)blk_ptr->quantized_dc[2][context_ptr->txb_itr];
            neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                (uint8_t *)&dc_sign_level_coeff,
                ROUND_UV(txb_origin_x) >> 1,
                ROUND_UV(txb_origin_y) >> 1,
                context_ptr->blk_geom->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }

    } // Transform Loop

    for (context_ptr->txb_itr = 0; context_ptr->txb_itr < tot_tu; context_ptr->txb_itr++) {
        uint8_t uv_pass = blk_ptr->tx_depth && context_ptr->txb_itr ? 0 : 1;

        if (context_ptr->blk_geom->has_uv && uv_pass) {
            blk_ptr->block_has_coeff = blk_ptr->block_has_coeff |
                                       blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff |
                                       blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff |
                                       blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff;

            if (blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff)
                blk_ptr->txb_array[0].u_has_coeff = EB_TRUE;
            if (blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff)
                blk_ptr->txb_array[0].v_has_coeff = EB_TRUE;
        } else {
            blk_ptr->block_has_coeff =
                blk_ptr->block_has_coeff | blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff;
        }
    } // Transform Loop
}
#define REFMVS_LIMIT ((1 << 12) - 1)

void av1_copy_frame_mvs(PictureControlSet *pcs_ptr, const Av1Common *const cm, MbModeInfo mi,
                        int mi_row, int mi_col, int x_mis, int y_mis,
                        EbReferenceObject *object_ptr) {
    const int frame_mvs_stride = ROUND_POWER_OF_TWO(cm->mi_cols, 1);
    MV_REF *  frame_mvs        = object_ptr->mvs + (mi_row >> 1) * frame_mvs_stride + (mi_col >> 1);
    x_mis                      = ROUND_POWER_OF_TWO(x_mis, 1);
    y_mis                      = ROUND_POWER_OF_TWO(y_mis, 1);
    int w, h;

    for (h = 0; h < y_mis; h++) {
        MV_REF *mv = frame_mvs;
        for (w = 0; w < x_mis; w++) {
            mv->ref_frame = NONE_FRAME;
            mv->mv.as_int = 0;

            for (int idx = 0; idx < 2; ++idx) {
                MvReferenceFrame ref_frame = mi.block_mi.ref_frame[idx];
                if (ref_frame > INTRA_FRAME) {
                    int8_t ref_idx = pcs_ptr->ref_frame_side[ref_frame];
                    if (ref_idx) continue;
                    if ((abs(mi.block_mi.mv[idx].as_mv.row) > REFMVS_LIMIT) ||
                        (abs(mi.block_mi.mv[idx].as_mv.col) > REFMVS_LIMIT))
                        continue;
                    mv->ref_frame = ref_frame;
                    mv->mv.as_int = mi.block_mi.mv[idx].as_int;
                }
            }
            mv++;
        }
        frame_mvs += frame_mvs_stride;
    }
}
/*******************************************
* Encode Pass
*
* Summary: Performs an AV1 conformant
*   reconstruction based on the SB
*   mode decision.
*
* Inputs:
*   SourcePic
*   Coding Results
*   SB Location
*   Sequence Control Set
*   Picture Control Set
*
* Outputs:
*   Reconstructed Samples
*   Coefficient Samples
*
*******************************************/
EB_EXTERN void av1_encode_pass(SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                               SuperBlock *sb_ptr, uint32_t sb_addr, uint32_t sb_origin_x,
                               uint32_t sb_origin_y, EncDecContext *context_ptr) {
    EbBool               is_16bit = context_ptr->is_16bit;
    EbPictureBufferDesc *recon_buffer =
        is_16bit ? pcs_ptr->recon_picture16bit_ptr : pcs_ptr->recon_picture_ptr;
    EbPictureBufferDesc *coeff_buffer_sb = sb_ptr->quantized_coeff;
    EbPictureBufferDesc *input_picture;
    ModeDecisionContext *md_context_ptr;
    md_context_ptr = context_ptr->md_context;
    input_picture  = context_ptr->input_samples =
        (EbPictureBufferDesc *)pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    // SB Stats
    uint32_t sb_width =
        MIN(scs_ptr->sb_size_pix, pcs_ptr->parent_pcs_ptr->aligned_width - sb_origin_x);
    uint32_t sb_height =
        MIN(scs_ptr->sb_size_pix, pcs_ptr->parent_pcs_ptr->aligned_height - sb_origin_y);
    // MV merge mode
    uint32_t              y_has_coeff;
    uint32_t              u_has_coeff;
    uint32_t              v_has_coeff;
    uint64_t              y_coeff_bits;
    uint64_t              cb_coeff_bits;
    uint64_t              cr_coeff_bits;
    uint64_t              y_full_distortion[DIST_CALC_TOTAL];
    EB_ALIGN(16) uint64_t y_tu_full_distortion[DIST_CALC_TOTAL];
    uint32_t              count_non_zero_coeffs[3];
    MacroblockPlane       blk_plane[3];
    uint16_t              eobs[MAX_TXB_COUNT][3];
    uint64_t              y_txb_coeff_bits;
    uint64_t              cb_txb_coeff_bits;
    uint64_t              cr_txb_coeff_bits;
    EncodeContext *       encode_context_ptr;
    // Dereferencing early
#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
    uint16_t total_tile_cnt = pcs_ptr->parent_pcs_ptr->av1_cm->tiles_info.tile_cols *
        pcs_ptr->parent_pcs_ptr->av1_cm->tiles_info.tile_rows;
    NeighborArrayUnit *ep_mode_type_neighbor_array = pcs_ptr->ep_mode_type_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_intra_luma_mode_neighbor_array =
        pcs_ptr->ep_intra_luma_mode_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_intra_chroma_mode_neighbor_array =
        pcs_ptr->ep_intra_chroma_mode_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_mv_neighbor_array = pcs_ptr->ep_mv_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_luma_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_luma_recon_neighbor_array16bit[tile_idx]
                 : pcs_ptr->ep_luma_recon_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_cb_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cb_recon_neighbor_array16bit[tile_idx]
                 : pcs_ptr->ep_cb_recon_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_cr_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cr_recon_neighbor_array16bit[tile_idx]
                 : pcs_ptr->ep_cr_recon_neighbor_array[tile_idx];
    NeighborArrayUnit *ep_skip_flag_neighbor_array = pcs_ptr->ep_skip_flag_neighbor_array[tile_idx];
#else
    NeighborArrayUnit *ep_mode_type_neighbor_array = pcs_ptr->ep_mode_type_neighbor_array;
    NeighborArrayUnit *ep_intra_luma_mode_neighbor_array =
        pcs_ptr->ep_intra_luma_mode_neighbor_array;
    NeighborArrayUnit *ep_intra_chroma_mode_neighbor_array =
        pcs_ptr->ep_intra_chroma_mode_neighbor_array;
    NeighborArrayUnit *ep_mv_neighbor_array = pcs_ptr->ep_mv_neighbor_array;
    NeighborArrayUnit *ep_luma_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_luma_recon_neighbor_array16bit
                 : pcs_ptr->ep_luma_recon_neighbor_array;
    NeighborArrayUnit *ep_cb_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cb_recon_neighbor_array16bit : pcs_ptr->ep_cb_recon_neighbor_array;
    NeighborArrayUnit *ep_cr_recon_neighbor_array =
        is_16bit ? pcs_ptr->ep_cr_recon_neighbor_array16bit : pcs_ptr->ep_cr_recon_neighbor_array;
    NeighborArrayUnit *ep_skip_flag_neighbor_array = pcs_ptr->ep_skip_flag_neighbor_array;
#endif

    EbBool       dlf_enable_flag = (EbBool)pcs_ptr->parent_pcs_ptr->loop_filter_mode;
    const EbBool is_intra_sb     = pcs_ptr->limit_intra ? EB_FALSE : EB_TRUE;

    EbBool do_recon =
        (EbBool)((pcs_ptr->limit_intra == 0 || is_intra_sb == 1) ||
                 pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag ||
                 scs_ptr->static_config.recon_enabled || scs_ptr->static_config.stat_report);

    EntropyCoder *coeff_est_entropy_coder_ptr = pcs_ptr->coeff_est_entropy_coder_ptr;

    encode_context_ptr =
        ((SequenceControlSet *)(pcs_ptr->scs_wrapper_ptr->object_ptr))->encode_context_ptr;

    if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
        //get the 16bit form of the input SB
        if (is_16bit)
            recon_buffer = ((EbReferenceObject *)
                                pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                               ->reference_picture16bit;
        else
            recon_buffer = ((EbReferenceObject *)
                                pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                               ->reference_picture;
    else // non ref pictures
        recon_buffer = is_16bit ? pcs_ptr->recon_picture16bit_ptr : pcs_ptr->recon_picture_ptr;

    // DeriveZeroLumaCbf
    EbBool high_intra_ref      = EB_FALSE;
    EbBool check_zero_luma_cbf = EB_FALSE;

    if (is_16bit) {
        //SB128_TODO change 10bit SB creation

        if ((scs_ptr->static_config.ten_bit_format == 1) ||
            (scs_ptr->static_config.compressed_ten_bit_format == 1)) {
            const uint32_t input_luma_offset =
                ((sb_origin_y + input_picture->origin_y) * input_picture->stride_y) +
                (sb_origin_x + input_picture->origin_x);
            const uint32_t input_cb_offset =
                (((sb_origin_y + input_picture->origin_y) >> 1) * input_picture->stride_cb) +
                ((sb_origin_x + input_picture->origin_x) >> 1);
            const uint32_t input_cr_offset =
                (((sb_origin_y + input_picture->origin_y) >> 1) * input_picture->stride_cr) +
                ((sb_origin_x + input_picture->origin_x) >> 1);
            const uint16_t luma_2bit_width = input_picture->width / 4;
            const uint16_t chroma_2bit_width = input_picture->width / 8;

            compressed_pack_sb(input_picture->buffer_y + input_luma_offset,
                               input_picture->stride_y,
                               input_picture->buffer_bit_inc_y + sb_origin_y * luma_2bit_width +
                                   (sb_origin_x / 4) * sb_height,
                               sb_width / 4,
                               (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_y,
                               context_ptr->input_sample16bit_buffer->stride_y,
                               sb_width,
                               sb_height);

            compressed_pack_sb(input_picture->buffer_cb + input_cb_offset,
                               input_picture->stride_cb,
                               input_picture->buffer_bit_inc_cb +
                                   sb_origin_y / 2 * chroma_2bit_width +
                                   (sb_origin_x / 8) * (sb_height / 2),
                               sb_width / 8,
                               (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_cb,
                               context_ptr->input_sample16bit_buffer->stride_cb,
                               sb_width >> 1,
                               sb_height >> 1);

            compressed_pack_sb(input_picture->buffer_cr + input_cr_offset,
                               input_picture->stride_cr,
                               input_picture->buffer_bit_inc_cr +
                                   sb_origin_y / 2 * chroma_2bit_width +
                                   (sb_origin_x / 8) * (sb_height / 2),
                               sb_width / 8,
                               (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_cr,
                               context_ptr->input_sample16bit_buffer->stride_cr,
                               sb_width >> 1,
                               sb_height >> 1);
        } else {
            const uint32_t input_luma_offset =
                ((sb_origin_y + input_picture->origin_y) * input_picture->stride_y) +
                (sb_origin_x + input_picture->origin_x);
            const uint32_t input_bit_inc_luma_offset =
                ((sb_origin_y + input_picture->origin_y) * input_picture->stride_bit_inc_y) +
                (sb_origin_x + input_picture->origin_x);
            const uint32_t input_cb_offset =
                (((sb_origin_y + input_picture->origin_y) >> 1) * input_picture->stride_cb) +
                ((sb_origin_x + input_picture->origin_x) >> 1);
            const uint32_t input_bit_inc_cb_offset =
                (((sb_origin_y + input_picture->origin_y) >> 1) *
                 input_picture->stride_bit_inc_cb) +
                ((sb_origin_x + input_picture->origin_x) >> 1);
            const uint32_t input_cr_offset =
                (((sb_origin_y + input_picture->origin_y) >> 1) * input_picture->stride_cr) +
                ((sb_origin_x + input_picture->origin_x) >> 1);
            const uint32_t input_bit_inc_cr_offset = (((sb_origin_y + input_picture->origin_y) >> 1) *
                                                  input_picture->stride_bit_inc_cr) +
                                                 ((sb_origin_x + input_picture->origin_x) >> 1);

            pack2d_src(input_picture->buffer_y + input_luma_offset,
                       input_picture->stride_y,
                       input_picture->buffer_bit_inc_y + input_bit_inc_luma_offset,
                       input_picture->stride_bit_inc_y,
                       (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_y,
                       context_ptr->input_sample16bit_buffer->stride_y,
                       sb_width,
                       sb_height);

            pack2d_src(input_picture->buffer_cb + input_cb_offset,
                       input_picture->stride_cr,
                       input_picture->buffer_bit_inc_cb + input_bit_inc_cb_offset,
                       input_picture->stride_bit_inc_cr,
                       (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_cb,
                       context_ptr->input_sample16bit_buffer->stride_cb,
                       sb_width >> 1,
                       sb_height >> 1);

            pack2d_src(input_picture->buffer_cr + input_cr_offset,
                       input_picture->stride_cr,
                       input_picture->buffer_bit_inc_cr + input_bit_inc_cr_offset,
                       input_picture->stride_bit_inc_cr,
                       (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_cr,
                       context_ptr->input_sample16bit_buffer->stride_cr,
                       sb_width >> 1,
                       sb_height >> 1);
        }

        if (context_ptr->md_context->hbd_mode_decision == 0)
            store16bit_input_src(context_ptr->input_sample16bit_buffer,
                                 pcs_ptr,
                                 sb_origin_x,
                                 sb_origin_y,
                                 sb_width,
                                 sb_height);
    }

    if ((scs_ptr->input_resolution == INPUT_SIZE_4K_RANGE) &&
        !pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) {
            if (pcs_ptr->slice_type == B_SLICE) {
                EbReferenceObject *ref_obj_l0 =
                    (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[REF_LIST_0][0]->object_ptr;
                EbReferenceObject *ref_obj_l1 =
                    (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[REF_LIST_1][0]->object_ptr;
                uint32_t const thresh = (scs_ptr->static_config.frame_rate >> 16) < 50 ? 25 : 30;

                if ((ref_obj_l0->tmp_layer_idx == 2 && ref_obj_l0->intra_coded_area > thresh) ||
                    (ref_obj_l1->tmp_layer_idx == 2 && ref_obj_l1->intra_coded_area > thresh))
                    high_intra_ref = EB_TRUE;
            }
            if (high_intra_ref == EB_FALSE) check_zero_luma_cbf = EB_TRUE;
    }
    context_ptr->intra_coded_area_sb[sb_addr] = 0;
    context_ptr->coded_area_sb                = 0;
    context_ptr->coded_area_sb_uv             = 0;

#if TILES_PARALLEL
    if (dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode == 1 && total_tile_cnt == 1) {
#else
    if (dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode == 1) {
#endif
        if (sb_addr == 0) {
            eb_av1_loop_filter_init(pcs_ptr);

            eb_av1_pick_filter_level(
                0,
                (EbPictureBufferDesc *)pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                pcs_ptr,
                LPF_PICK_FROM_Q);

            eb_av1_loop_filter_frame_init(
                &pcs_ptr->parent_pcs_ptr->frm_hdr, &pcs_ptr->parent_pcs_ptr->lf_info, 0, 3);
        }
    }

    uint8_t  allow_update_cdf = pcs_ptr->update_cdf;
    uint32_t final_blk_itr    = 0;
    // CU Loop
    uint32_t blk_it = 0;
    while (blk_it < scs_ptr->max_block_cnt) {
        BlkStruct *blk_ptr = context_ptr->blk_ptr =
            &context_ptr->md_context->md_blk_arr_nsq[blk_it];
        PartitionType part = blk_ptr->part;

        const BlockGeom *blk_geom = context_ptr->blk_geom = get_blk_geom_mds(blk_it);
        UNUSED(blk_geom);
        sb_ptr->cu_partition_array[blk_it] = context_ptr->md_context->md_blk_arr_nsq[blk_it].part;
        if (pcs_ptr->update_cdf) {
            blk_ptr->av1xd->tile_ctx = &pcs_ptr->ec_ctx_array[sb_addr];
            // Update the partition stats
            update_part_stats(pcs_ptr,
                              blk_ptr,
#if TILES_PARALLEL
                              tile_idx,
#endif
                              (sb_origin_y + blk_geom->origin_y) >> MI_SIZE_LOG2,
                              (sb_origin_x + blk_geom->origin_x) >> MI_SIZE_LOG2);
        }
        if (part != PARTITION_SPLIT && pcs_ptr->parent_pcs_ptr->sb_geom[sb_addr].block_is_allowed[blk_it]) {
            int32_t offset_d1 = ns_blk_offset[(int32_t)part]; //blk_ptr->best_d1_blk; // TOCKECK
            int32_t num_d1_block =
                ns_blk_num[(int32_t)part]; // context_ptr->blk_geom->totns; // TOCKECK

            // for (int32_t d1_itr = blk_it; d1_itr < blk_it + num_d1_block; d1_itr++) {
            for (int32_t d1_itr = (int32_t)blk_it + offset_d1;
                 d1_itr < (int32_t)blk_it + offset_d1 + num_d1_block;
                 d1_itr++) {
                const BlockGeom *blk_geom = context_ptr->blk_geom = get_blk_geom_mds(d1_itr);

                // PU Stack variables
                PredictionUnit *     pu_ptr           = (PredictionUnit *)EB_NULL; //  done
                EbPictureBufferDesc *residual_buffer  = context_ptr->residual_buffer;
                EbPictureBufferDesc *transform_buffer = context_ptr->transform_buffer;

                EbPictureBufferDesc *inverse_quant_buffer = context_ptr->inverse_quant_buffer;

                int16_t *transform_inner_array_ptr = context_ptr->transform_inner_array_ptr;

                BlkStruct *blk_ptr = context_ptr->blk_ptr =
                    &context_ptr->md_context->md_blk_arr_nsq[d1_itr];

                context_ptr->blk_origin_x = (uint16_t)(sb_origin_x + blk_geom->origin_x);
                context_ptr->blk_origin_y = (uint16_t)(sb_origin_y + blk_geom->origin_y);
                blk_ptr->delta_qp         = 0;
                context_ptr->md_skip_blk =
                    context_ptr->md_context->blk_skip_decision
                        ? ((blk_ptr->prediction_mode_flag == INTRA_MODE || blk_ptr->block_has_coeff)
                               ? 0
                               : 1)
                        : 0;
                blk_ptr->block_has_coeff = 0;

                // if(pcs_ptr->picture_number==4 && context_ptr->blk_origin_x==0 && context_ptr->blk_origin_y==0)
                //     SVT_LOG("CHEDD");
                uint32_t coded_area_org    = context_ptr->coded_area_sb;
                uint32_t coded_area_org_uv = context_ptr->coded_area_sb_uv;

                // Derive disable_cfl_flag as evaluate_cfl_ep = f(disable_cfl_flag)
                EbBool disable_cfl_flag =
                    (context_ptr->blk_geom->sq_size > 32 || context_ptr->blk_geom->bwidth == 4 ||
                     context_ptr->blk_geom->bheight == 4)
                        ? EB_TRUE
                        : EB_FALSE;
                // Evaluate cfl @ EP if applicable, and not done @ MD
                context_ptr->evaluate_cfl_ep =
                    (disable_cfl_flag == EB_FALSE &&
                     context_ptr->md_context->chroma_level == CHROMA_MODE_2);
                // for now, segmentation independent of sharpness/delta QP.
                if (pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled) {
                    apply_segmentation_based_quantization(blk_geom, pcs_ptr, sb_ptr, blk_ptr);

                    sb_ptr->qp = blk_ptr->qp;
                } else {
                    blk_ptr->qp       = sb_ptr->qp;
                    blk_ptr->delta_qp = sb_ptr->delta_qp;
                }

                if (blk_ptr->prediction_mode_flag == INTRA_MODE) {
                    context_ptr->is_inter = blk_ptr->av1xd->use_intrabc;
                    context_ptr->tot_intra_coded_area += blk_geom->bwidth * blk_geom->bheight;
                    if (pcs_ptr->slice_type != I_SLICE)
                        context_ptr->intra_coded_area_sb[sb_addr] +=
                            blk_geom->bwidth * blk_geom->bheight;

                    if (scs_ptr->static_config.encoder_bit_depth > EB_8BIT &&
                        pcs_ptr->hbd_mode_decision == 0 &&
                        blk_ptr->palette_info.pmi.palette_size[0] > 0) {
                        //MD was done on 8bit, scale  palette colors to 10bit
                        for (uint8_t col = 0; col < blk_ptr->palette_info.pmi.palette_size[0];
                             col++)
                            blk_ptr->palette_info.pmi.palette_colors[col] *= 4;
                    }
                    // *Note - Transforms are the same size as predictions
                    // Partition Loop
                    context_ptr->txb_itr = 0;
                    // Transform partitioning path (INTRA Luma/Chroma)
                    if (blk_ptr->av1xd->use_intrabc == 0) {
                        // Set the PU Loop Variables
                        pu_ptr = blk_ptr->prediction_unit_array;
                        // Generate Intra Luma Neighbor Modes
                        generate_pu_intra_luma_neighbor_nodes(blk_ptr,
                                                              context_ptr->blk_origin_x,
                                                              context_ptr->blk_origin_y,
                                                              BLOCK_SIZE_64,
                                                              ep_intra_luma_mode_neighbor_array,
                                                              ep_intra_chroma_mode_neighbor_array,
                                                              ep_mode_type_neighbor_array);

                        perform_intra_coding_loop(
                            pcs_ptr, sb_ptr, sb_addr, blk_ptr, pu_ptr, context_ptr);

                        // Update the Intra-specific Neighbor Arrays
                        encode_pass_update_intra_mode_neighbor_arrays(
                            ep_mode_type_neighbor_array,
                            ep_intra_luma_mode_neighbor_array,
                            ep_intra_chroma_mode_neighbor_array,
                            (uint8_t)blk_ptr->pred_mode,
                            (uint8_t)pu_ptr->intra_chroma_mode,
                            context_ptr->blk_origin_x,
                            context_ptr->blk_origin_y,
                            context_ptr->blk_geom->bwidth,
                            context_ptr->blk_geom->bheight,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv,
                            blk_geom->has_uv ? PICTURE_BUFFER_DESC_FULL_MASK
                                             : PICTURE_BUFFER_DESC_LUMA_MASK);

                    }
                    // Transform partitioning free patch (except the 128x128 case)
                    else {
                        // Set the PU Loop Variables
                        pu_ptr = blk_ptr->prediction_unit_array;
                        // Generate Intra Luma Neighbor Modes
                        generate_pu_intra_luma_neighbor_nodes( // HT done
                            blk_ptr,
                            context_ptr->blk_origin_x,
                            context_ptr->blk_origin_y,
                            BLOCK_SIZE_64,
                            ep_intra_luma_mode_neighbor_array,
                            ep_intra_chroma_mode_neighbor_array,
                            ep_mode_type_neighbor_array);

                        {
                            uint32_t blk_originy_uv = (context_ptr->blk_origin_y >> 3 << 3) >> 1;
                            uint32_t blk_originx_uv = (context_ptr->blk_origin_x >> 3 << 3) >> 1;

                            context_ptr->md_context->luma_txb_skip_context = 0;
                            context_ptr->md_context->luma_dc_sign_context  = 0;
                            get_txb_ctx(pcs_ptr,
                                        COMPONENT_LUMA,
#if TILES_PARALLEL
                                        pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                        pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                                        context_ptr->blk_origin_x,
                                        context_ptr->blk_origin_y,
                                        context_ptr->blk_geom->bsize,
                                        context_ptr->blk_geom->txsize[0][0],
                                        &context_ptr->md_context->luma_txb_skip_context,
                                        &context_ptr->md_context->luma_dc_sign_context);

                            if (context_ptr->blk_geom->has_uv) {
                                context_ptr->md_context->cb_txb_skip_context = 0;
                                context_ptr->md_context->cb_dc_sign_context  = 0;
                                get_txb_ctx(pcs_ptr,
                                            COMPONENT_CHROMA,
#if TILES_PARALLEL
                                            pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                            pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                                            blk_originx_uv,
                                            blk_originy_uv,
                                            context_ptr->blk_geom->bsize_uv,
                                            context_ptr->blk_geom->txsize_uv[0][0],
                                            &context_ptr->md_context->cb_txb_skip_context,
                                            &context_ptr->md_context->cb_dc_sign_context);

                                context_ptr->md_context->cr_txb_skip_context = 0;
                                context_ptr->md_context->cr_dc_sign_context  = 0;
                                get_txb_ctx(pcs_ptr,
                                            COMPONENT_CHROMA,
#if TILES_PARALLEL
                                            pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                            pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                                            blk_originx_uv,
                                            blk_originy_uv,
                                            context_ptr->blk_geom->bsize_uv,
                                            context_ptr->blk_geom->txsize_uv[0][0],
                                            &context_ptr->md_context->cr_txb_skip_context,
                                            &context_ptr->md_context->cr_dc_sign_context);
                            }
                            {
                                MvReferenceFrame ref_frame = INTRA_FRAME;
                                generate_av1_mvp_table(&sb_ptr->tile_info,
                                                       context_ptr->md_context,
                                                       blk_ptr,
                                                       context_ptr->blk_geom,
                                                       context_ptr->blk_origin_x,
                                                       context_ptr->blk_origin_y,
                                                       &ref_frame,
                                                       1,
                                                       pcs_ptr);

                                IntMv nearestmv, nearmv;
                                eb_av1_find_best_ref_mvs_from_stack(
                                    0,
                                    context_ptr->md_context->md_local_blk_unit[blk_geom->blkidx_mds]
                                        .ed_ref_mv_stack,
                                    blk_ptr->av1xd,
                                    ref_frame,
                                    &nearestmv,
                                    &nearmv,
                                    0);

                                if (nearestmv.as_int == INVALID_MV) nearestmv.as_int = 0;
                                if (nearmv.as_int == INVALID_MV) nearmv.as_int = 0;
                                IntMv dv_ref = nearestmv.as_int == 0 ? nearmv : nearestmv;
                                if (dv_ref.as_int == 0)
                                    av1_find_ref_dv(&dv_ref,
                                                    &blk_ptr->av1xd->tile,
                                                    scs_ptr->seq_header.sb_mi_size,
                                                    context_ptr->blk_origin_y >> MI_SIZE_LOG2,
                                                    context_ptr->blk_origin_x >> MI_SIZE_LOG2);
                                // Ref DV should not have sub-pel.
                                assert((dv_ref.as_mv.col & 7) == 0);
                                assert((dv_ref.as_mv.row & 7) == 0);
                                context_ptr->md_context->md_local_blk_unit[blk_geom->blkidx_mds]
                                    .ed_ref_mv_stack[INTRA_FRAME][0]
                                    .this_mv       = dv_ref;
                                blk_ptr->predmv[0] = dv_ref;

                                //keep final usefull mvp for entropy
                                memcpy(blk_ptr->av1xd->final_ref_mv_stack,
                                       context_ptr->md_context
                                           ->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                                           .ed_ref_mv_stack[blk_ptr->prediction_unit_array[0]
                                                                .ref_frame_type],
                                       sizeof(CandidateMv) * MAX_REF_MV_STACK_SIZE);

                                pu_ptr = blk_ptr->prediction_unit_array;
                                // Set MvUnit
                                context_ptr->mv_unit.pred_direction =
                                    (uint8_t)pu_ptr->inter_pred_direction_index;
                                context_ptr->mv_unit.mv[REF_LIST_0].mv_union =
                                    pu_ptr->mv[REF_LIST_0].mv_union;
                                context_ptr->mv_unit.mv[REF_LIST_1].mv_union =
                                    pu_ptr->mv[REF_LIST_1].mv_union;

                                EbPictureBufferDesc *ref_pic_list0 =
                                    ((EbReferenceObject *)pcs_ptr->parent_pcs_ptr
                                         ->reference_picture_wrapper_ptr->object_ptr)
                                        ->reference_picture;

                                if (is_16bit)
                                    ref_pic_list0 =
                                        ((EbReferenceObject *)pcs_ptr->parent_pcs_ptr
                                             ->reference_picture_wrapper_ptr->object_ptr)
                                            ->reference_picture16bit;

                                av1_inter_prediction_function_table[is_16bit](
                                    pcs_ptr,
                                    blk_ptr->interp_filters,
                                    blk_ptr,
                                    blk_ptr->prediction_unit_array->ref_frame_type,
                                    &context_ptr->mv_unit,
                                    1, // use_intrabc,
                                    SIMPLE_TRANSLATION,
                                    0,
                                    0,
                                    1,
                                    &blk_ptr->interinter_comp,
                                    &sb_ptr->tile_info,
                                    ep_luma_recon_neighbor_array,
                                    ep_cb_recon_neighbor_array,
                                    ep_cr_recon_neighbor_array,
                                    blk_ptr->is_interintra_used,
                                    blk_ptr->interintra_mode,
                                    blk_ptr->use_wedge_interintra,
                                    blk_ptr->interintra_wedge_index,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    blk_geom->bwidth,
                                    blk_geom->bheight,
                                    ref_pic_list0,
                                    0,
                                    recon_buffer,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    EB_TRUE,
                                    (uint8_t)scs_ptr->static_config.encoder_bit_depth);
                            }
                            // Encode Transform Unit -INTRA-
                            {
                                uint16_t cb_qp = blk_ptr->qp;

                                av1_encode_loop_func_table[is_16bit](
                                    pcs_ptr,
                                    context_ptr,
                                    sb_ptr,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    cb_qp,
                                    recon_buffer,
                                    coeff_buffer_sb,
                                    residual_buffer,
                                    transform_buffer,
                                    inverse_quant_buffer,
                                    transform_inner_array_ptr,
                                    count_non_zero_coeffs,
                                    blk_geom->has_uv ? PICTURE_BUFFER_DESC_FULL_MASK
                                                     : PICTURE_BUFFER_DESC_LUMA_MASK,
                                    eobs[context_ptr->txb_itr],
                                    blk_plane);

                                if (allow_update_cdf) {
                                    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
                                        context_ptr->md_context->candidate_buffer_ptr_array;
                                    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
                                        &(candidate_buffer_ptr_array_base[0]);
                                    ModeDecisionCandidateBuffer *candidate_buffer;

                                    // Set the Candidate Buffer
                                    candidate_buffer = candidate_buffer_ptr_array[0];
                                    // Rate estimation function uses the values from CandidatePtr. The right values are copied from blk_ptr to CandidatePtr
                                    candidate_buffer->candidate_ptr->type =
                                        blk_ptr->prediction_mode_flag;
                                    candidate_buffer->candidate_ptr->pred_mode = blk_ptr->pred_mode;
                                    candidate_buffer->candidate_ptr->filter_intra_mode =
                                        blk_ptr->filter_intra_mode;
                                    const uint32_t coeff1d_offset = context_ptr->coded_area_sb;

                                    av1_txb_estimate_coeff_bits(
                                        context_ptr->md_context,
                                        1, //allow_update_cdf,
                                        &pcs_ptr->ec_ctx_array[sb_addr],
                                        pcs_ptr,
                                        candidate_buffer,
                                        coeff1d_offset,
                                        context_ptr->coded_area_sb_uv,
                                        coeff_est_entropy_coder_ptr,
                                        coeff_buffer_sb,
                                        eobs[context_ptr->txb_itr][0],
                                        eobs[context_ptr->txb_itr][1],
                                        eobs[context_ptr->txb_itr][2],
                                        &y_txb_coeff_bits,
                                        &cb_txb_coeff_bits,
                                        &cr_txb_coeff_bits,
                                        context_ptr->blk_geom
                                            ->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        context_ptr->blk_geom
                                            ->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        blk_ptr->txb_array[context_ptr->txb_itr]
                                            .transform_type[PLANE_TYPE_Y],
                                        blk_ptr->txb_array[context_ptr->txb_itr]
                                            .transform_type[PLANE_TYPE_UV],
                                        context_ptr->blk_geom->has_uv ? COMPONENT_ALL
                                                                      : COMPONENT_LUMA);
                                }
                                //intra mode
                                av1_enc_gen_recon_func_ptr[is_16bit](
                                    context_ptr,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    recon_buffer,
                                    inverse_quant_buffer,
                                    transform_inner_array_ptr,
                                    blk_geom->has_uv ? PICTURE_BUFFER_DESC_FULL_MASK
                                                     : PICTURE_BUFFER_DESC_LUMA_MASK,
                                    eobs[context_ptr->txb_itr]);
                            }

                            // Update the Intra-specific Neighbor Arrays
                            encode_pass_update_intra_mode_neighbor_arrays(
                                ep_mode_type_neighbor_array,
                                ep_intra_luma_mode_neighbor_array,
                                ep_intra_chroma_mode_neighbor_array,
                                (uint8_t)blk_ptr->pred_mode,
                                (uint8_t)pu_ptr->intra_chroma_mode,
                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                context_ptr->blk_geom->bwidth,
                                context_ptr->blk_geom->bheight,
                                context_ptr->blk_geom->bwidth_uv,
                                context_ptr->blk_geom->bheight_uv,
                                blk_geom->has_uv ? PICTURE_BUFFER_DESC_FULL_MASK
                                                 : PICTURE_BUFFER_DESC_LUMA_MASK);

                            // Update Recon Samples-INTRA-
                            encode_pass_update_recon_sample_neighbour_arrays(
                                ep_luma_recon_neighbor_array,
                                ep_cb_recon_neighbor_array,
                                ep_cr_recon_neighbor_array,
                                recon_buffer,
                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                context_ptr->blk_geom->bwidth,
                                context_ptr->blk_geom->bheight,
                                context_ptr->blk_geom->bwidth_uv,
                                context_ptr->blk_geom->bheight_uv,
                                blk_geom->has_uv ? PICTURE_BUFFER_DESC_FULL_MASK
                                                 : PICTURE_BUFFER_DESC_LUMA_MASK,
                                is_16bit);

                            // Update the luma Dc Sign Level Coeff Neighbor Array
                            {
                                uint8_t dc_sign_level_coeff =
                                    (uint8_t)blk_ptr->quantized_dc[0][context_ptr->txb_itr];
                                neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                    pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                                    (uint8_t *)&dc_sign_level_coeff,
                                    context_ptr->blk_origin_x,
                                    context_ptr->blk_origin_y,
                                    context_ptr->blk_geom->bwidth,
                                    context_ptr->blk_geom->bheight,
                                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                            }

                            // Update the cb Dc Sign Level Coeff Neighbor Array
                            if (context_ptr->blk_geom->has_uv) {
                                uint8_t dc_sign_level_coeff =
                                    (uint8_t)blk_ptr->quantized_dc[1][context_ptr->txb_itr];
                                neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                                    (uint8_t *)&dc_sign_level_coeff,
                                    ROUND_UV(context_ptr->blk_origin_x) >> 1,
                                    ROUND_UV(context_ptr->blk_origin_y) >> 1,
                                    context_ptr->blk_geom->bwidth_uv,
                                    context_ptr->blk_geom->bheight_uv,
                                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                            }

                            // Update the cr DC Sign Level Coeff Neighbor Array
                            if (context_ptr->blk_geom->has_uv) {
                                uint8_t dc_sign_level_coeff =
                                    (uint8_t)blk_ptr->quantized_dc[2][context_ptr->txb_itr];
                                neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                    pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                                    (uint8_t *)&dc_sign_level_coeff,
                                    ROUND_UV(context_ptr->blk_origin_x) >> 1,
                                    ROUND_UV(context_ptr->blk_origin_y) >> 1,
                                    context_ptr->blk_geom->bwidth_uv,
                                    context_ptr->blk_geom->bheight_uv,
                                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                            }

                            if (context_ptr->blk_geom->has_uv) {
                                blk_ptr->block_has_coeff =
                                    blk_ptr->block_has_coeff |
                                    blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff |
                                    blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff |
                                    blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff;

                                if (blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff)
                                    blk_ptr->txb_array[0].u_has_coeff = EB_TRUE;
                                if (blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff)
                                    blk_ptr->txb_array[0].v_has_coeff = EB_TRUE;
                            } else {
                                blk_ptr->block_has_coeff =
                                    blk_ptr->block_has_coeff |
                                    blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff;
                            }
                        } // Transform Loop
                        context_ptr->coded_area_sb += blk_geom->bwidth * blk_geom->bheight;
                        if (blk_geom->has_uv)
                            context_ptr->coded_area_sb_uv +=
                                blk_geom->bwidth_uv * blk_geom->bheight_uv;
                    }
                }

                // Inter
                else if (blk_ptr->prediction_mode_flag == INTER_MODE) {
                    context_ptr->is_inter = 1;
                    int8_t ref_idx_l0 = (&blk_ptr->prediction_unit_array[0])->ref_frame_index_l0;
                    int8_t ref_idx_l1 = (&blk_ptr->prediction_unit_array[0])->ref_frame_index_l1;
                    MvReferenceFrame rf[2];
                    av1_set_ref_frame(rf, (&blk_ptr->prediction_unit_array[0])->ref_frame_type);
                    uint8_t list_idx0, list_idx1;
                    list_idx0 = get_list_idx(rf[0]);
                    if (rf[1] == NONE_FRAME)
                        list_idx1 = get_list_idx(rf[0]);
                    else
                        list_idx1 = get_list_idx(rf[1]);
                    EbReferenceObject *ref_obj_0 =
                        ref_idx_l0 >= 0
                            ? (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx0][ref_idx_l0]
                                  ->object_ptr
                            : (EbReferenceObject *)EB_NULL;
                    EbReferenceObject *ref_obj_1 =
                        ref_idx_l1 >= 0
                            ? (EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[list_idx1][ref_idx_l1]
                                  ->object_ptr
                            : (EbReferenceObject *)EB_NULL;
                    uint16_t txb_origin_x;
                    uint16_t txb_origin_y;
                    EbBool   is_blk_skip = EB_FALSE;

                    //********************************
                    //        INTER
                    //********************************

                    EbBool zero_luma_cbf_md = EB_FALSE;
                    //EbBool doLumaMC = EB_TRUE;
                    EbBool do_mv_pred = EB_TRUE;
                    //if QP M and Segments are used, First Cu in SB row should have at least one coeff.
                    EbBool is_first_blk_in_row = EB_FALSE;
                    zero_luma_cbf_md           = (EbBool)(
                        check_zero_luma_cbf &&
                        ((&blk_ptr->prediction_unit_array[0])->merge_flag == EB_FALSE &&
                         blk_ptr->block_has_coeff == 0 && is_first_blk_in_row == EB_FALSE));
                    zero_luma_cbf_md = EB_FALSE;

                    //Motion Compensation could be avoided in the case below
                    EbBool do_mc = EB_TRUE;

                    // Perform Merge/Skip Decision if the mode coming from MD is merge. for the First CU in Row merge will remain as is.
                    if (blk_ptr->prediction_unit_array[0].merge_flag == EB_TRUE) {
                        if (is_first_blk_in_row == EB_FALSE)
                            is_blk_skip =
                                md_context_ptr->md_ep_pipe_sb[blk_ptr->mds_idx].skip_cost <=
                                        md_context_ptr->md_ep_pipe_sb[blk_ptr->mds_idx].merge_cost
                                    ? 1
                                    : 0;
                    }

                    //MC could be avoided in some cases below
                    if (is_first_blk_in_row == EB_FALSE) {
                        if (pcs_ptr->limit_intra && is_intra_sb == EB_FALSE) {
                            if (is_blk_skip) {
                                do_mc      = EB_FALSE;
                                do_mv_pred = EB_FALSE;
                            }
                        }
                    }

                    do_mc = (EbBool)(do_recon | do_mc);

                    do_mv_pred = (EbBool)(do_recon | do_mv_pred);

                    //IntMv  predmv[2];
                    enc_pass_av1_mv_pred(&sb_ptr->tile_info,
                                         context_ptr->md_context,
                                         blk_ptr,
                                         blk_geom,
                                         context_ptr->blk_origin_x,
                                         context_ptr->blk_origin_y,
                                         pcs_ptr,
                                         blk_ptr->prediction_unit_array[0].ref_frame_type,
                                         blk_ptr->prediction_unit_array[0].is_compound,
                                         blk_ptr->pred_mode,
                                         blk_ptr->predmv);
                    //out1:  predmv
                    //out2:   blk_ptr->inter_mode_ctx[ blk_ptr->prediction_unit_array[0].ref_frame_type ]

                    //keep final usefull mvp for entropy
                    memcpy(blk_ptr->av1xd->final_ref_mv_stack,
                           context_ptr->md_context
                               ->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                               .ed_ref_mv_stack[blk_ptr->prediction_unit_array[0].ref_frame_type],
                           sizeof(CandidateMv) * MAX_REF_MV_STACK_SIZE);

                    {
                        // 1st Partition Loop
                        pu_ptr = blk_ptr->prediction_unit_array;

                        // Set MvUnit
                        context_ptr->mv_unit.pred_direction =
                            (uint8_t)pu_ptr->inter_pred_direction_index;
                        context_ptr->mv_unit.mv[REF_LIST_0].mv_union =
                            pu_ptr->mv[REF_LIST_0].mv_union;
                        context_ptr->mv_unit.mv[REF_LIST_1].mv_union =
                            pu_ptr->mv[REF_LIST_1].mv_union;

                        // Inter Prediction
                        if (do_mc && pu_ptr->motion_mode == WARPED_CAUSAL) {
                            warped_motion_prediction(
                                pcs_ptr,
                                &context_ptr->mv_unit,
                                blk_ptr->prediction_unit_array[0].ref_frame_type,
                                blk_ptr->prediction_unit_array[0].compound_idx,
                                &blk_ptr->prediction_unit_array[0].interinter_comp,
                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                blk_ptr,
                                blk_geom,
                                is_16bit ? ref_obj_0->reference_picture16bit
                                         : ref_obj_0->reference_picture,
                                ref_idx_l1 >= 0 ? is_16bit ? ref_obj_1->reference_picture16bit
                                                           : ref_obj_1->reference_picture
                                                : NULL,
                                recon_buffer,
                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                &blk_ptr->prediction_unit_array[0].wm_params_l0,
                                &blk_ptr->prediction_unit_array[0].wm_params_l1,
                                (uint8_t)scs_ptr->static_config.encoder_bit_depth,
                                EB_TRUE);
                        }

                        if (do_mc && pu_ptr->motion_mode != WARPED_CAUSAL) {
                            EbPictureBufferDesc *ref_pic_list0;
                            EbPictureBufferDesc *ref_pic_list1;

                            if (!is_16bit) {
                                ref_pic_list0 =
                                    blk_ptr->prediction_unit_array->ref_frame_index_l0 >= 0
                                        ? ref_obj_0->reference_picture
                                        : (EbPictureBufferDesc *)EB_NULL;
                                ref_pic_list1 =
                                    blk_ptr->prediction_unit_array->ref_frame_index_l1 >= 0
                                        ? ref_obj_1->reference_picture
                                        : (EbPictureBufferDesc *)EB_NULL;
                            } else {
                                ref_pic_list0 =
                                    blk_ptr->prediction_unit_array->ref_frame_index_l0 >= 0
                                        ? ref_obj_0->reference_picture16bit
                                        : (EbPictureBufferDesc *)EB_NULL;
                                ref_pic_list1 =
                                    blk_ptr->prediction_unit_array->ref_frame_index_l1 >= 0
                                        ? ref_obj_1->reference_picture16bit
                                        : (EbPictureBufferDesc *)EB_NULL;
                            }

                            av1_inter_prediction_function_table[is_16bit](
                                pcs_ptr,
                                blk_ptr->interp_filters,
                                blk_ptr,
                                blk_ptr->prediction_unit_array->ref_frame_type,
                                &context_ptr->mv_unit,
                                0, //use_intrabc,
                                blk_ptr->prediction_unit_array->motion_mode,
                                0, //use_precomputed_obmc,
                                0,
                                blk_ptr->compound_idx,
                                &blk_ptr->interinter_comp,
                                &sb_ptr->tile_info,
                                ep_luma_recon_neighbor_array,
                                ep_cb_recon_neighbor_array,
                                ep_cr_recon_neighbor_array,
                                blk_ptr->is_interintra_used,
                                blk_ptr->interintra_mode,
                                blk_ptr->use_wedge_interintra,
                                blk_ptr->interintra_wedge_index,

                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                blk_geom->bwidth,
                                blk_geom->bheight,
                                ref_pic_list0,
                                ref_pic_list1,
                                recon_buffer,
                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                EB_TRUE,
                                (uint8_t)scs_ptr->static_config.encoder_bit_depth);
                        }
                    }

                    context_ptr->txb_itr = 0;
                    // Transform Loop
                    blk_ptr->txb_array[0].y_has_coeff = EB_FALSE;
                    blk_ptr->txb_array[0].u_has_coeff = EB_FALSE;
                    blk_ptr->txb_array[0].v_has_coeff = EB_FALSE;

                    // initialize TU Split
                    y_full_distortion[DIST_CALC_RESIDUAL]   = 0;
                    y_full_distortion[DIST_CALC_PREDICTION] = 0;

                    y_coeff_bits  = 0;
                    cb_coeff_bits = 0;
                    cr_coeff_bits = 0;

                    uint32_t tot_tu = context_ptr->blk_geom->txb_count[blk_ptr->tx_depth];
                    uint8_t  tu_it;
                    uint16_t cb_qp          = blk_ptr->qp;
                    uint32_t component_mask = context_ptr->blk_geom->has_uv
                                                  ? PICTURE_BUFFER_DESC_FULL_MASK
                                                  : PICTURE_BUFFER_DESC_LUMA_MASK;

                    if (blk_ptr->prediction_unit_array[0].merge_flag == EB_FALSE) {
                        for (uint8_t tu_it = 0; tu_it < tot_tu; tu_it++) {
                            context_ptr->txb_itr = tu_it;
                            uint8_t uv_pass =
                                blk_ptr->tx_depth && tu_it ? 0 : 1; //NM: 128x128 exeption
#if TX_ORG_INTERINTRA
                            txb_origin_x =
                                context_ptr->blk_origin_x +
                                context_ptr->blk_geom->tx_org_x[1][blk_ptr->tx_depth][tu_it] -
                                context_ptr->blk_geom->origin_x;
                            txb_origin_y =
                                context_ptr->blk_origin_y +
                                context_ptr->blk_geom->tx_org_y[1][blk_ptr->tx_depth][tu_it] -
                                context_ptr->blk_geom->origin_y;
#else
                            txb_origin_x =
                                context_ptr->blk_origin_x +
                                context_ptr->blk_geom->tx_boff_x[blk_ptr->tx_depth][tu_it];
                            txb_origin_y =
                                context_ptr->blk_origin_y +
                                context_ptr->blk_geom->tx_boff_y[blk_ptr->tx_depth][tu_it];
#endif

                            context_ptr->md_context->luma_txb_skip_context = 0;
                            context_ptr->md_context->luma_dc_sign_context  = 0;
                            get_txb_ctx(pcs_ptr,
                                        COMPONENT_LUMA,
#if TILES_PARALLEL
                                        pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                        pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                                        txb_origin_x,
                                        txb_origin_y,
                                        context_ptr->blk_geom->bsize,
                                        context_ptr->blk_geom
                                            ->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        &context_ptr->md_context->luma_txb_skip_context,
                                        &context_ptr->md_context->luma_dc_sign_context);

                            if (context_ptr->blk_geom->has_uv && uv_pass) {
                                context_ptr->md_context->cb_txb_skip_context = 0;
                                context_ptr->md_context->cb_dc_sign_context  = 0;
                                get_txb_ctx(
                                    pcs_ptr,
                                    COMPONENT_CHROMA,
#if TILES_PARALLEL
                                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                                    ROUND_UV(txb_origin_x) >> 1,
                                    ROUND_UV(txb_origin_y) >> 1,
                                    context_ptr->blk_geom->bsize_uv,
                                    context_ptr->blk_geom->txsize_uv[context_ptr->blk_ptr->tx_depth]
                                                                    [context_ptr->txb_itr],
                                    &context_ptr->md_context->cb_txb_skip_context,
                                    &context_ptr->md_context->cb_dc_sign_context);

                                context_ptr->md_context->cr_txb_skip_context = 0;
                                context_ptr->md_context->cr_dc_sign_context  = 0;
                                get_txb_ctx(pcs_ptr,
                                            COMPONENT_CHROMA,
#if TILES_PARALLEL
                                            pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                            pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                                            ROUND_UV(txb_origin_x) >> 1,
                                            ROUND_UV(txb_origin_y) >> 1,
                                            context_ptr->blk_geom->bsize_uv,
                                            context_ptr->blk_geom->txsize_uv[blk_ptr->tx_depth]
                                                                            [context_ptr->txb_itr],
                                            &context_ptr->md_context->cr_txb_skip_context,
                                            &context_ptr->md_context->cr_dc_sign_context);
                            }
                            if (!zero_luma_cbf_md)
                                //inter mode  1
                                av1_encode_loop_func_table[is_16bit](
                                    pcs_ptr,
                                    context_ptr,
                                    sb_ptr,
                                    txb_origin_x, //pic org
                                    txb_origin_y,
                                    cb_qp,
                                    recon_buffer,
                                    coeff_buffer_sb,
                                    residual_buffer,
                                    transform_buffer,
                                    inverse_quant_buffer,
                                    transform_inner_array_ptr,
                                    count_non_zero_coeffs,
                                    context_ptr->blk_geom->has_uv && uv_pass
                                        ? PICTURE_BUFFER_DESC_FULL_MASK
                                        : PICTURE_BUFFER_DESC_LUMA_MASK,
                                    eobs[context_ptr->txb_itr],
                                    blk_plane);

                            // SKIP the CBF zero mode for DC path. There are problems with cost calculations
                            {
                                // Compute Tu distortion
                                if (!zero_luma_cbf_md)

                                    // LUMA DISTORTION
                                    picture_full_distortion32_bits(
                                        transform_buffer,
                                        context_ptr->coded_area_sb,
                                        0,
                                        inverse_quant_buffer,
                                        context_ptr->coded_area_sb,
                                        0,
                                        blk_geom->tx_width[blk_ptr->tx_depth][tu_it],
                                        blk_geom->tx_height[blk_ptr->tx_depth][tu_it],
                                        context_ptr->blk_geom->bwidth_uv,
                                        context_ptr->blk_geom->bheight_uv,
                                        y_tu_full_distortion,
                                        y_tu_full_distortion,
                                        y_tu_full_distortion,
                                        eobs[context_ptr->txb_itr][0],
                                        0,
                                        0,
                                        COMPONENT_LUMA);
                                TxSize tx_size =
                                    blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr];
                                int32_t shift = (MAX_TX_SCALE - av1_get_tx_scale(tx_size)) * 2;
                                y_tu_full_distortion[DIST_CALC_RESIDUAL] = RIGHT_SIGNED_SHIFT(
                                    y_tu_full_distortion[DIST_CALC_RESIDUAL], shift);
                                y_tu_full_distortion[DIST_CALC_PREDICTION] = RIGHT_SIGNED_SHIFT(
                                    y_tu_full_distortion[DIST_CALC_PREDICTION], shift);

                                y_txb_coeff_bits  = 0;
                                cb_txb_coeff_bits = 0;
                                cr_txb_coeff_bits = 0;

                                if (!zero_luma_cbf_md) {
                                    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
                                        context_ptr->md_context->candidate_buffer_ptr_array;
                                    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
                                        &(candidate_buffer_ptr_array_base[0]);
                                    ModeDecisionCandidateBuffer *candidate_buffer;

                                    // Set the Candidate Buffer
                                    candidate_buffer = candidate_buffer_ptr_array[0];
                                    // Rate estimation function uses the values from CandidatePtr. The right values are copied from blk_ptr to CandidatePtr
                                    candidate_buffer->candidate_ptr->type =
                                        blk_ptr->prediction_mode_flag;

                                    const uint32_t coeff1d_offset = context_ptr->coded_area_sb;

                                    av1_txb_estimate_coeff_bits(
                                        context_ptr->md_context,
                                        0, //allow_update_cdf,
                                        NULL,
                                        pcs_ptr,
                                        candidate_buffer,
                                        coeff1d_offset,
                                        context_ptr->coded_area_sb_uv,
                                        coeff_est_entropy_coder_ptr,
                                        coeff_buffer_sb,
                                        eobs[context_ptr->txb_itr][0],
                                        eobs[context_ptr->txb_itr][1],
                                        eobs[context_ptr->txb_itr][2],
                                        &y_txb_coeff_bits,
                                        &cb_txb_coeff_bits,
                                        &cr_txb_coeff_bits,
                                        context_ptr->blk_geom
                                            ->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        context_ptr->blk_geom
                                            ->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        blk_ptr->txb_array[context_ptr->txb_itr]
                                            .transform_type[PLANE_TYPE_Y],
                                        blk_ptr->txb_array[context_ptr->txb_itr]
                                            .transform_type[PLANE_TYPE_UV],
                                        context_ptr->blk_geom->has_uv && uv_pass ? COMPONENT_ALL
                                                                                 : COMPONENT_LUMA);
                                }

                                // CBF Tu decision
                                if (zero_luma_cbf_md == EB_FALSE)

                                    av1_encode_txb_calc_cost(context_ptr,
                                                             count_non_zero_coeffs,
                                                             y_tu_full_distortion,
                                                             &y_txb_coeff_bits,
                                                             component_mask);

                                else {
                                    blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff = 0;
                                    blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff = 0;
                                    blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff = 0;
                                }
                                // Update count_non_zero_coeffs after CBF decision
                                if (blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff ==
                                    EB_FALSE)
                                    count_non_zero_coeffs[0] = 0;
                                if (context_ptr->blk_geom->has_uv && uv_pass) {
                                    if (blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff ==
                                        EB_FALSE)
                                        count_non_zero_coeffs[1] = 0;
                                    if (blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff ==
                                        EB_FALSE)
                                        count_non_zero_coeffs[2] = 0;
                                }

                                // Update TU count_non_zero_coeffs
                                blk_ptr->txb_array[context_ptr->txb_itr].nz_coef_count[0] =
                                    (uint16_t)count_non_zero_coeffs[0];
                                blk_ptr->txb_array[context_ptr->txb_itr].nz_coef_count[1] =
                                    (uint16_t)count_non_zero_coeffs[1];
                                blk_ptr->txb_array[context_ptr->txb_itr].nz_coef_count[2] =
                                    (uint16_t)count_non_zero_coeffs[2];

                                y_coeff_bits += y_txb_coeff_bits;
                                if (context_ptr->blk_geom->has_uv && uv_pass) {
                                    cb_coeff_bits += cb_txb_coeff_bits;
                                    cr_coeff_bits += cr_txb_coeff_bits;
                                }

                                y_full_distortion[DIST_CALC_RESIDUAL] +=
                                    y_tu_full_distortion[DIST_CALC_RESIDUAL];
                                y_full_distortion[DIST_CALC_PREDICTION] +=
                                    y_tu_full_distortion[DIST_CALC_PREDICTION];

                                if (allow_update_cdf) {
                                    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
                                        context_ptr->md_context->candidate_buffer_ptr_array;
                                    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
                                        &(candidate_buffer_ptr_array_base[0]);
                                    ModeDecisionCandidateBuffer *candidate_buffer;

                                    // Set the Candidate Buffer
                                    candidate_buffer = candidate_buffer_ptr_array[0];
                                    // Rate estimation function uses the values from CandidatePtr. The right values are copied from blk_ptr to CandidatePtr
                                    candidate_buffer->candidate_ptr->type =
                                        blk_ptr->prediction_mode_flag;
                                    candidate_buffer->candidate_ptr->pred_mode = blk_ptr->pred_mode;
                                    candidate_buffer->candidate_ptr->filter_intra_mode =
                                        blk_ptr->filter_intra_mode;
                                    const uint32_t coeff1d_offset = context_ptr->coded_area_sb;

                                    //CHKN add updating eobs[] after CBF decision
                                    if (blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff ==
                                        EB_FALSE)
                                        eobs[context_ptr->txb_itr][0] = 0;
                                    if (context_ptr->blk_geom->has_uv && uv_pass) {
                                        if (blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff ==
                                            EB_FALSE)
                                            eobs[context_ptr->txb_itr][1] = 0;
                                        if (blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff ==
                                            EB_FALSE)
                                            eobs[context_ptr->txb_itr][2] = 0;
                                    }

                                    av1_txb_estimate_coeff_bits(
                                        context_ptr->md_context,
                                        1, //allow_update_cdf,
                                        &pcs_ptr->ec_ctx_array[sb_addr],
                                        pcs_ptr,
                                        candidate_buffer,
                                        coeff1d_offset,
                                        context_ptr->coded_area_sb_uv,
                                        coeff_est_entropy_coder_ptr,
                                        coeff_buffer_sb,
                                        eobs[context_ptr->txb_itr][0],
                                        eobs[context_ptr->txb_itr][1],
                                        eobs[context_ptr->txb_itr][2],
                                        &y_txb_coeff_bits,
                                        &cb_txb_coeff_bits,
                                        &cr_txb_coeff_bits,
                                        context_ptr->blk_geom
                                            ->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        context_ptr->blk_geom
                                            ->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        blk_ptr->txb_array[context_ptr->txb_itr]
                                            .transform_type[PLANE_TYPE_Y],
                                        blk_ptr->txb_array[context_ptr->txb_itr]
                                            .transform_type[PLANE_TYPE_UV],
                                        context_ptr->blk_geom->has_uv && uv_pass ? COMPONENT_ALL
                                                                                 : COMPONENT_LUMA);
                                }
                            }
                            context_ptr->coded_area_sb +=
                                blk_geom->tx_width[blk_ptr->tx_depth][tu_it] *
                                blk_geom->tx_height[blk_ptr->tx_depth][tu_it];
                            if (context_ptr->blk_geom->has_uv && uv_pass)
                                context_ptr->coded_area_sb_uv +=
                                    blk_geom->tx_width_uv[blk_ptr->tx_depth][tu_it] *
                                    blk_geom->tx_height_uv[blk_ptr->tx_depth][tu_it];

                            // Update the luma Dc Sign Level Coeff Neighbor Array
                            {
                                uint8_t dc_sign_level_coeff =
                                    (uint8_t)blk_ptr->quantized_dc[0][context_ptr->txb_itr];

                                neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                    pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                                    (uint8_t *)&dc_sign_level_coeff,
                                    txb_origin_x,
                                    txb_origin_y,
                                    context_ptr->blk_geom
                                        ->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    context_ptr->blk_geom
                                        ->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                            }

                            // Update the cb Dc Sign Level Coeff Neighbor Array
                            if (context_ptr->blk_geom->has_uv && uv_pass) {
                                uint8_t dc_sign_level_coeff =
                                    (uint8_t)blk_ptr->quantized_dc[1][context_ptr->txb_itr];
                                neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                                    (uint8_t *)&dc_sign_level_coeff,
                                    ROUND_UV(txb_origin_x) >> 1,
                                    ROUND_UV(txb_origin_y) >> 1,
                                    context_ptr->blk_geom
                                        ->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    context_ptr->blk_geom
                                        ->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                            }

                            // Update the cr DC Sign Level Coeff Neighbor Array
                            if (context_ptr->blk_geom->has_uv && uv_pass) {
                                uint8_t dc_sign_level_coeff =
                                    (uint8_t)blk_ptr->quantized_dc[2][context_ptr->txb_itr];
                                neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                    pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                    pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                                    (uint8_t *)&dc_sign_level_coeff,
                                    ROUND_UV(txb_origin_x) >> 1,
                                    ROUND_UV(txb_origin_y) >> 1,
                                    context_ptr->blk_geom
                                        ->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    context_ptr->blk_geom
                                        ->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                            }

                        } // Transform Loop
                    }

                    //Set Final CU data flags after skip/Merge decision.
                    if (is_first_blk_in_row == EB_FALSE) {
                        if (blk_ptr->prediction_unit_array[0].merge_flag == EB_TRUE) {
                            blk_ptr->skip_flag = (is_blk_skip) ? EB_TRUE : EB_FALSE;
                            blk_ptr->prediction_unit_array[0].merge_flag =
                                (is_blk_skip) ? EB_FALSE : EB_TRUE;
                        }
                    }

                    // Initialize the Transform Loop

                    context_ptr->txb_itr = 0;
                    y_has_coeff          = 0;
                    u_has_coeff          = 0;
                    v_has_coeff          = 0;
                    tot_tu               = context_ptr->blk_geom->txb_count[blk_ptr->tx_depth];

                    //reset coeff buffer offsets at the start of a new Tx loop
                    context_ptr->coded_area_sb    = coded_area_org;
                    context_ptr->coded_area_sb_uv = coded_area_org_uv;
                    for (tu_it = 0; tu_it < tot_tu; tu_it++) {
                        uint8_t uv_pass = blk_ptr->tx_depth && tu_it ? 0 : 1; //NM: 128x128 exeption
                        context_ptr->txb_itr = tu_it;
#if TX_ORG_INTERINTRA
                        txb_origin_x = context_ptr->blk_origin_x +
                                       (context_ptr->blk_geom
                                            ->tx_org_x[1][blk_ptr->tx_depth][context_ptr->txb_itr] -
                                        context_ptr->blk_geom->origin_x);
                        txb_origin_y = context_ptr->blk_origin_y +
                                       (context_ptr->blk_geom
                                            ->tx_org_y[1][blk_ptr->tx_depth][context_ptr->txb_itr] -
                                        context_ptr->blk_geom->origin_y);
#else
                        txb_origin_x = context_ptr->blk_origin_x +
                                       context_ptr->blk_geom->tx_boff_x[blk_ptr->tx_depth][tu_it];
                        txb_origin_y = context_ptr->blk_origin_y +
                                       context_ptr->blk_geom->tx_boff_y[blk_ptr->tx_depth][tu_it];
#endif
                        context_ptr->md_context->luma_txb_skip_context = 0;
                        context_ptr->md_context->luma_dc_sign_context  = 0;
                        get_txb_ctx(
                            pcs_ptr,
                            COMPONENT_LUMA,
#if TILES_PARALLEL
                            pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                            pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                            txb_origin_x,
                            txb_origin_y,
                            context_ptr->blk_geom->bsize,
                            context_ptr->blk_geom->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                            &context_ptr->md_context->luma_txb_skip_context,
                            &context_ptr->md_context->luma_dc_sign_context);

                        if (context_ptr->blk_geom->has_uv && uv_pass) {
                            context_ptr->md_context->cb_txb_skip_context = 0;
                            context_ptr->md_context->cb_dc_sign_context  = 0;
                            get_txb_ctx(
                                pcs_ptr,
                                COMPONENT_CHROMA,
#if TILES_PARALLEL
                                pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                                ROUND_UV(txb_origin_x) >> 1,
                                ROUND_UV(txb_origin_y) >> 1,
                                context_ptr->blk_geom->bsize_uv,
                                context_ptr->blk_geom->txsize_uv[context_ptr->blk_ptr->tx_depth]
                                                                [context_ptr->txb_itr],
                                &context_ptr->md_context->cb_txb_skip_context,
                                &context_ptr->md_context->cb_dc_sign_context);

                            context_ptr->md_context->cr_txb_skip_context = 0;
                            context_ptr->md_context->cr_dc_sign_context  = 0;
                            get_txb_ctx(pcs_ptr,
                                        COMPONENT_CHROMA,
#if TILES_PARALLEL
                                        pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                        pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                                        ROUND_UV(txb_origin_x) >> 1,
                                        ROUND_UV(txb_origin_y) >> 1,
                                        context_ptr->blk_geom->bsize_uv,
                                        context_ptr->blk_geom
                                            ->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                        &context_ptr->md_context->cr_txb_skip_context,
                                        &context_ptr->md_context->cr_dc_sign_context);
                        }
                        if (blk_ptr->skip_flag == EB_TRUE) {
                            blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff = EB_FALSE;
                            blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff = EB_FALSE;
                            blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff = EB_FALSE;

                            context_ptr->blk_ptr->quantized_dc[0][context_ptr->txb_itr] = 0;
                            context_ptr->blk_ptr->quantized_dc[1][context_ptr->txb_itr] = 0;
                            context_ptr->blk_ptr->quantized_dc[2][context_ptr->txb_itr] = 0;
                        } else if ((&blk_ptr->prediction_unit_array[0])->merge_flag == EB_TRUE) {
                            //inter mode  2

                            av1_encode_loop_func_table[is_16bit](
                                pcs_ptr,
                                context_ptr,
                                sb_ptr,
                                txb_origin_x, //pic offset
                                txb_origin_y,
                                cb_qp,
                                recon_buffer,
                                coeff_buffer_sb,
                                residual_buffer,
                                transform_buffer,
                                inverse_quant_buffer,
                                transform_inner_array_ptr,
                                count_non_zero_coeffs,
                                context_ptr->blk_geom->has_uv && uv_pass
                                    ? PICTURE_BUFFER_DESC_FULL_MASK
                                    : PICTURE_BUFFER_DESC_LUMA_MASK,
                                eobs[context_ptr->txb_itr],
                                blk_plane);

                            if (allow_update_cdf) {
                                ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
                                    context_ptr->md_context->candidate_buffer_ptr_array;
                                ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
                                    &(candidate_buffer_ptr_array_base[0]);
                                ModeDecisionCandidateBuffer *candidate_buffer;

                                // Set the Candidate Buffer
                                candidate_buffer = candidate_buffer_ptr_array[0];
                                // Rate estimation function uses the values from CandidatePtr. The right values are copied from blk_ptr to CandidatePtr
                                candidate_buffer->candidate_ptr->type =
                                    blk_ptr->prediction_mode_flag;
                                candidate_buffer->candidate_ptr->pred_mode = blk_ptr->pred_mode;
                                candidate_buffer->candidate_ptr->filter_intra_mode =
                                    blk_ptr->filter_intra_mode;
                                const uint32_t coeff1d_offset = context_ptr->coded_area_sb;

                                av1_txb_estimate_coeff_bits(
                                    context_ptr->md_context,
                                    1, //allow_update_cdf,
                                    &pcs_ptr->ec_ctx_array[sb_addr],
                                    pcs_ptr,
                                    candidate_buffer,
                                    coeff1d_offset,
                                    context_ptr->coded_area_sb_uv,
                                    coeff_est_entropy_coder_ptr,
                                    coeff_buffer_sb,
                                    eobs[context_ptr->txb_itr][0],
                                    eobs[context_ptr->txb_itr][1],
                                    eobs[context_ptr->txb_itr][2],
                                    &y_txb_coeff_bits,
                                    &cb_txb_coeff_bits,
                                    &cr_txb_coeff_bits,
                                    context_ptr->blk_geom
                                        ->txsize[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    context_ptr->blk_geom
                                        ->txsize_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                    blk_ptr->txb_array[context_ptr->txb_itr]
                                        .transform_type[PLANE_TYPE_Y],
                                    blk_ptr->txb_array[context_ptr->txb_itr]
                                        .transform_type[PLANE_TYPE_UV],
                                    context_ptr->blk_geom->has_uv && uv_pass ? COMPONENT_ALL
                                                                             : COMPONENT_LUMA);
                            }
                        }
                        if (context_ptr->blk_geom->has_uv && uv_pass) {
                            blk_ptr->block_has_coeff =
                                blk_ptr->block_has_coeff |
                                blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff |
                                blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff |
                                blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff;
                        } else {
                            blk_ptr->block_has_coeff =
                                blk_ptr->block_has_coeff |
                                blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff;
                        }

                        //inter mode
                        if (do_recon)

                            av1_enc_gen_recon_func_ptr[is_16bit](
                                context_ptr,
                                txb_origin_x, //pic offset
                                txb_origin_y,
                                recon_buffer,
                                inverse_quant_buffer,
                                transform_inner_array_ptr,
                                context_ptr->blk_geom->has_uv && uv_pass
                                    ? PICTURE_BUFFER_DESC_FULL_MASK
                                    : PICTURE_BUFFER_DESC_LUMA_MASK,
                                eobs[context_ptr->txb_itr]);

                        if (context_ptr->blk_geom->has_uv && uv_pass) {
                            y_has_coeff |= blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff;
                            u_has_coeff |= blk_ptr->txb_array[context_ptr->txb_itr].u_has_coeff;
                            v_has_coeff |= blk_ptr->txb_array[context_ptr->txb_itr].v_has_coeff;
                        } else
                            y_has_coeff |= blk_ptr->txb_array[context_ptr->txb_itr].y_has_coeff;
                        context_ptr->coded_area_sb += blk_geom->tx_width[blk_ptr->tx_depth][tu_it] *
                                                      blk_geom->tx_height[blk_ptr->tx_depth][tu_it];

                        if (context_ptr->blk_geom->has_uv && uv_pass)
                            context_ptr->coded_area_sb_uv +=
                                blk_geom->tx_width_uv[blk_ptr->tx_depth][tu_it] *
                                blk_geom->tx_height_uv[blk_ptr->tx_depth][tu_it];

                        // Update the luma Dc Sign Level Coeff Neighbor Array
                        {
                            uint8_t dc_sign_level_coeff =
                                (uint8_t)blk_ptr->quantized_dc[0][context_ptr->txb_itr];

                            neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                pcs_ptr->ep_luma_dc_sign_level_coeff_neighbor_array,
#endif
                                (uint8_t *)&dc_sign_level_coeff,
                                txb_origin_x,
                                txb_origin_y,
                                context_ptr->blk_geom
                                    ->tx_width[blk_ptr->tx_depth][context_ptr->txb_itr],
                                context_ptr->blk_geom
                                    ->tx_height[blk_ptr->tx_depth][context_ptr->txb_itr],
                                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                        }

                        // Update the cb Dc Sign Level Coeff Neighbor Array
                        if (context_ptr->blk_geom->has_uv && uv_pass) {
                            uint8_t dc_sign_level_coeff =
                                (uint8_t)blk_ptr->quantized_dc[1][context_ptr->txb_itr];
                            neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                pcs_ptr->ep_cb_dc_sign_level_coeff_neighbor_array,
#endif
                                (uint8_t *)&dc_sign_level_coeff,
                                ROUND_UV(txb_origin_x) >> 1,
                                ROUND_UV(txb_origin_y) >> 1,
                                context_ptr->blk_geom
                                    ->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                context_ptr->blk_geom
                                    ->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                        }

                        // Update the cr DC Sign Level Coeff Neighbor Array
                        if (context_ptr->blk_geom->has_uv && uv_pass) {
                            uint8_t dc_sign_level_coeff =
                                (uint8_t)blk_ptr->quantized_dc[2][context_ptr->txb_itr];
                            neighbor_array_unit_mode_write(
#if TILES_PARALLEL
                                pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array[tile_idx],
#else
                                pcs_ptr->ep_cr_dc_sign_level_coeff_neighbor_array,
#endif
                                (uint8_t *)&dc_sign_level_coeff,
                                ROUND_UV(txb_origin_x) >> 1,
                                ROUND_UV(txb_origin_y) >> 1,
                                context_ptr->blk_geom
                                    ->tx_width_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                context_ptr->blk_geom
                                    ->tx_height_uv[blk_ptr->tx_depth][context_ptr->txb_itr],
                                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
                        }

                    } // Transform Loop

                    // Calculate Root CBF
                    if (context_ptr->blk_geom->has_uv)
                        blk_ptr->block_has_coeff =
                            (y_has_coeff | u_has_coeff | v_has_coeff) ? EB_TRUE : EB_FALSE;
                    else
                        blk_ptr->block_has_coeff = (y_has_coeff) ? EB_TRUE : EB_FALSE;

                    // Force Skip if MergeFlag == TRUE && RootCbf == 0

                    if (blk_ptr->skip_flag == EB_FALSE &&
                        blk_ptr->prediction_unit_array[0].merge_flag == EB_TRUE &&
                        blk_ptr->block_has_coeff == EB_FALSE) {
                        blk_ptr->skip_flag = EB_TRUE;
                    }

                    {
                        // Set the PU Loop Variables
                        pu_ptr = blk_ptr->prediction_unit_array;

                        // Set MvUnit
                        context_ptr->mv_unit.pred_direction =
                            (uint8_t)pu_ptr->inter_pred_direction_index;
                        context_ptr->mv_unit.mv[REF_LIST_0].mv_union =
                            pu_ptr->mv[REF_LIST_0].mv_union;
                        context_ptr->mv_unit.mv[REF_LIST_1].mv_union =
                            pu_ptr->mv[REF_LIST_1].mv_union;

                        // Update Neighbor Arrays (Mode Type, mvs, SKIP)
                        {
                            uint8_t skip_flag = (uint8_t)blk_ptr->skip_flag;
                            encode_pass_update_inter_mode_neighbor_arrays(
                                ep_mode_type_neighbor_array,
                                ep_mv_neighbor_array,
                                ep_skip_flag_neighbor_array,
                                &context_ptr->mv_unit,
                                &skip_flag,
                                context_ptr->blk_origin_x,
                                context_ptr->blk_origin_y,
                                blk_geom->bwidth,
                                blk_geom->bheight);
                        }
                    } // 2nd Partition Loop

                    // Update Recon Samples Neighbor Arrays -INTER-

                    if (do_recon)
                        encode_pass_update_recon_sample_neighbour_arrays(
                            ep_luma_recon_neighbor_array,
                            ep_cb_recon_neighbor_array,
                            ep_cr_recon_neighbor_array,
                            recon_buffer,
                            context_ptr->blk_origin_x,
                            context_ptr->blk_origin_y,
                            context_ptr->blk_geom->bwidth,
                            context_ptr->blk_geom->bheight,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv,
                            context_ptr->blk_geom->has_uv ? PICTURE_BUFFER_DESC_FULL_MASK
                                                          : PICTURE_BUFFER_DESC_LUMA_MASK,
                            is_16bit);
                    // Collect the referenced area per 64x64
                    if (scs_ptr->use_output_stat_file) {
                        if (blk_ptr->prediction_unit_array->ref_frame_index_l0 >= 0) {
                            eb_block_on_mutex(ref_obj_0->referenced_area_mutex);
                            {
                                if (context_ptr->mv_unit.pred_direction == UNI_PRED_LIST_0 ||
                                    context_ptr->mv_unit.pred_direction == BI_PRED) {
                                    //List0-Y
                                    uint16_t origin_x =
                                        MAX(0,
                                            (int16_t)context_ptr->blk_origin_x +
                                                (context_ptr->mv_unit.mv[REF_LIST_0].x >> 3));
                                    uint16_t origin_y =
                                        MAX(0,
                                            (int16_t)context_ptr->blk_origin_y +
                                                (context_ptr->mv_unit.mv[REF_LIST_0].y >> 3));
                                    origin_x =
                                        MIN(origin_x,
                                            pcs_ptr->parent_pcs_ptr->aligned_width - blk_geom->bwidth);
                                    origin_y = MIN(
                                        origin_y,
                                        pcs_ptr->parent_pcs_ptr->aligned_height - blk_geom->bheight);
                                    uint16_t sb_origin_x =
                                        origin_x / context_ptr->sb_sz * context_ptr->sb_sz;
                                    uint16_t sb_origin_y =
                                        origin_y / context_ptr->sb_sz * context_ptr->sb_sz;
                                    uint32_t pic_width_in_sb =
                                        (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) /
                                        scs_ptr->sb_sz;
                                    uint16_t sb_index =
                                        sb_origin_x / context_ptr->sb_sz +
                                        pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                    uint16_t width, height, weight;
                                    weight = 1
                                             << (4 - pcs_ptr->parent_pcs_ptr->temporal_layer_index);

                                    width = MIN(sb_origin_x + context_ptr->sb_sz,
                                                origin_x + blk_geom->bwidth) -
                                            origin_x;
                                    height = MIN(sb_origin_y + context_ptr->sb_sz,
                                                 origin_y + blk_geom->bheight) -
                                             origin_y;
                                    ref_obj_0->stat_struct.referenced_area[sb_index] +=
                                        width * height * weight;

                                    if (origin_x + blk_geom->bwidth >
                                        sb_origin_x + context_ptr->sb_sz) {
                                        sb_origin_x = (origin_x / context_ptr->sb_sz + 1) *
                                                      context_ptr->sb_sz;
                                        sb_origin_y =
                                            origin_y / context_ptr->sb_sz * context_ptr->sb_sz;
                                        sb_index =
                                            sb_origin_x / context_ptr->sb_sz +
                                            pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                        width = origin_x + blk_geom->bwidth -
                                                MAX(sb_origin_x, origin_x);
                                        height = MIN(sb_origin_y + context_ptr->sb_sz,
                                                     origin_y + blk_geom->bheight) -
                                                 origin_y;
                                        ref_obj_0->stat_struct.referenced_area[sb_index] +=
                                            width * height * weight;
                                    }
                                    if (origin_y + blk_geom->bheight >
                                        sb_origin_y + context_ptr->sb_sz) {
                                        sb_origin_x =
                                            (origin_x / context_ptr->sb_sz) * context_ptr->sb_sz;
                                        sb_origin_y = (origin_y / context_ptr->sb_sz + 1) *
                                                      context_ptr->sb_sz;
                                        sb_index =
                                            sb_origin_x / context_ptr->sb_sz +
                                            pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                        width = MIN(sb_origin_x + context_ptr->sb_sz,
                                                    origin_x + blk_geom->bwidth) -
                                                origin_x;
                                        height = origin_y + blk_geom->bheight -
                                                 MAX(sb_origin_y, origin_y);
                                        ref_obj_0->stat_struct.referenced_area[sb_index] +=
                                            width * height * weight;
                                    }
                                    if (origin_x + blk_geom->bwidth >
                                            sb_origin_x + context_ptr->sb_sz &&
                                        origin_y + blk_geom->bheight >
                                            sb_origin_y + context_ptr->sb_sz) {
                                        sb_origin_x = (origin_x / context_ptr->sb_sz + 1) *
                                                      context_ptr->sb_sz;
                                        sb_origin_y = (origin_y / context_ptr->sb_sz + 1) *
                                                      context_ptr->sb_sz;
                                        sb_index =
                                            sb_origin_x / context_ptr->sb_sz +
                                            pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                        width = origin_x + blk_geom->bwidth -
                                                MAX(sb_origin_x, origin_x);
                                        height = origin_y + blk_geom->bheight -
                                                 MAX(sb_origin_y, origin_y);
                                        ref_obj_0->stat_struct.referenced_area[sb_index] +=
                                            width * height * weight;
                                    }
                                }
                            }
                            eb_release_mutex(ref_obj_0->referenced_area_mutex);
                        }

                        if (blk_ptr->prediction_unit_array->ref_frame_index_l1 >= 0) {
                            eb_block_on_mutex(ref_obj_1->referenced_area_mutex);
                            if (context_ptr->mv_unit.pred_direction == UNI_PRED_LIST_1 ||
                                context_ptr->mv_unit.pred_direction == BI_PRED) {
                                //List1-Y
                                uint16_t origin_x =
                                    MAX(0,
                                        (int16_t)context_ptr->blk_origin_x +
                                            (context_ptr->mv_unit.mv[REF_LIST_1].x >> 3));
                                uint16_t origin_y =
                                    MAX(0,
                                        (int16_t)context_ptr->blk_origin_y +
                                            (context_ptr->mv_unit.mv[REF_LIST_1].y >> 3));
                                origin_x =
                                    MIN(origin_x,
                                        pcs_ptr->parent_pcs_ptr->aligned_width - blk_geom->bwidth);
                                origin_y =
                                    MIN(origin_y,
                                        pcs_ptr->parent_pcs_ptr->aligned_height - blk_geom->bheight);
                                uint16_t sb_origin_x =
                                    origin_x / context_ptr->sb_sz * context_ptr->sb_sz;
                                uint16_t sb_origin_y =
                                    origin_y / context_ptr->sb_sz * context_ptr->sb_sz;
                                uint32_t pic_width_in_sb =
                                    (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) /
                                    scs_ptr->sb_sz;
                                uint16_t sb_index =
                                    sb_origin_x / context_ptr->sb_sz +
                                    pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                uint16_t width, height, weight;
                                weight = 1 << (4 - pcs_ptr->parent_pcs_ptr->temporal_layer_index);

                                width = MIN(sb_origin_x + context_ptr->sb_sz,
                                            origin_x + blk_geom->bwidth) -
                                        origin_x;
                                height = MIN(sb_origin_y + context_ptr->sb_sz,
                                             origin_y + blk_geom->bheight) -
                                         origin_y;
                                ref_obj_1->stat_struct.referenced_area[sb_index] +=
                                    width * height * weight;
                                if (origin_x + blk_geom->bwidth >
                                    sb_origin_x + context_ptr->sb_sz) {
                                    sb_origin_x =
                                        (origin_x / context_ptr->sb_sz + 1) * context_ptr->sb_sz;
                                    sb_origin_y =
                                        origin_y / context_ptr->sb_sz * context_ptr->sb_sz;
                                    sb_index = sb_origin_x / context_ptr->sb_sz +
                                               pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                    width =
                                        origin_x + blk_geom->bwidth - MAX(sb_origin_x, origin_x);
                                    height = MIN(sb_origin_y + context_ptr->sb_sz,
                                                 origin_y + blk_geom->bheight) -
                                             origin_y;
                                    ref_obj_1->stat_struct.referenced_area[sb_index] +=
                                        width * height * weight;
                                }
                                if (origin_y + blk_geom->bheight >
                                    sb_origin_y + context_ptr->sb_sz) {
                                    sb_origin_x =
                                        (origin_x / context_ptr->sb_sz) * context_ptr->sb_sz;
                                    sb_origin_y =
                                        (origin_y / context_ptr->sb_sz + 1) * context_ptr->sb_sz;
                                    sb_index = sb_origin_x / context_ptr->sb_sz +
                                               pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                    width = MIN(sb_origin_x + context_ptr->sb_sz,
                                                origin_x + blk_geom->bwidth) -
                                            origin_x;
                                    height =
                                        origin_y + blk_geom->bheight - MAX(sb_origin_y, origin_y);
                                    ref_obj_1->stat_struct.referenced_area[sb_index] +=
                                        width * height * weight;
                                }
                                if (origin_x + blk_geom->bwidth >
                                        sb_origin_x + context_ptr->sb_sz &&
                                    origin_y + blk_geom->bheight >
                                        sb_origin_y + context_ptr->sb_sz) {
                                    sb_origin_x =
                                        (origin_x / context_ptr->sb_sz + 1) * context_ptr->sb_sz;
                                    sb_origin_y =
                                        (origin_y / context_ptr->sb_sz + 1) * context_ptr->sb_sz;
                                    sb_index = sb_origin_x / context_ptr->sb_sz +
                                               pic_width_in_sb * (sb_origin_y / context_ptr->sb_sz);
                                    width =
                                        origin_x + blk_geom->bwidth - MAX(sb_origin_x, origin_x);
                                    height =
                                        origin_y + blk_geom->bheight - MAX(sb_origin_y, origin_y);
                                    ref_obj_1->stat_struct.referenced_area[sb_index] +=
                                        width * height * weight;
                                }
                            }
                            eb_release_mutex(ref_obj_1->referenced_area_mutex);
                        }
                    }

                } else {
                    CHECK_REPORT_ERROR_NC(encode_context_ptr->app_callback_ptr, EB_ENC_CL_ERROR2);
                }

                update_av1_mi_map(blk_ptr,
                                  context_ptr->blk_origin_x,
                                  context_ptr->blk_origin_y,
                                  blk_geom,
                                  pcs_ptr);
                if (pcs_ptr->update_cdf) {
                    // Update the partition Neighbor Array
                    PartitionContext partition;
                    partition.above = partition_context_lookup[blk_geom->bsize].above;
                    partition.left  = partition_context_lookup[blk_geom->bsize].left;

#if TILES_PARALLEL
                    neighbor_array_unit_mode_write(pcs_ptr->ep_partition_context_neighbor_array[tile_idx],
#else
                    neighbor_array_unit_mode_write(pcs_ptr->ep_partition_context_neighbor_array,
#endif
                                                   (uint8_t *)&partition,
                                                   context_ptr->blk_origin_x,
                                                   context_ptr->blk_origin_y,
                                                   blk_geom->bwidth,
                                                   blk_geom->bheight,
                                                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

                    // Update the CDFs based on the current block
                    blk_ptr->av1xd->tile_ctx = &pcs_ptr->ec_ctx_array[sb_addr];
                    update_stats(pcs_ptr,
                                 blk_ptr,
                                 context_ptr->blk_origin_y >> MI_SIZE_LOG2,
                                 context_ptr->blk_origin_x >> MI_SIZE_LOG2);
                }

                if (dlf_enable_flag) {}

                {{// Set the PU Loop Variables
                  pu_ptr = blk_ptr->prediction_unit_array;
                // Set MvUnit
                context_ptr->mv_unit.pred_direction = (uint8_t)pu_ptr->inter_pred_direction_index;
                context_ptr->mv_unit.mv[REF_LIST_0].mv_union = pu_ptr->mv[REF_LIST_0].mv_union;
                context_ptr->mv_unit.mv[REF_LIST_1].mv_union = pu_ptr->mv[REF_LIST_1].mv_union;
            }
        }

        {
            BlkStruct *src_cu = &context_ptr->md_context->md_blk_arr_nsq[d1_itr];
            BlkStruct *dst_cu = &sb_ptr->final_blk_arr[final_blk_itr++];
            move_blk_data(pcs_ptr, context_ptr, src_cu, dst_cu);
        }
        if (scs_ptr->mfmv_enabled && pcs_ptr->slice_type != I_SLICE &&
            pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) {
            uint32_t           mi_stride = pcs_ptr->mi_stride;
            int32_t            mi_row    = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
            int32_t            mi_col    = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
            const int32_t      offset    = mi_row * mi_stride + mi_col;
            ModeInfo *         mi_ptr    = *(pcs_ptr->mi_grid_base + offset);
            const int          x_mis     = AOMMIN(context_ptr->blk_geom->bwidth >> MI_SIZE_LOG2,
                                     pcs_ptr->parent_pcs_ptr->av1_cm->mi_cols - mi_col);
            const int          y_mis     = AOMMIN(context_ptr->blk_geom->bheight >> MI_SIZE_LOG2,
                                     pcs_ptr->parent_pcs_ptr->av1_cm->mi_rows - mi_row);
            EbReferenceObject *obj_l0 =
                (EbReferenceObject *)
                    pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr;

            av1_copy_frame_mvs(pcs_ptr,
                               pcs_ptr->parent_pcs_ptr->av1_cm,
                               mi_ptr->mbmi,
                               mi_row,
                               mi_col,
                               x_mis,
                               y_mis,
                               obj_l0);
        }
    }
    blk_it +=
        ns_depth_offset[scs_ptr->seq_header.sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
}
else blk_it +=
    d1_depth_offset[scs_ptr->seq_header.sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
} // CU Loop
// First Pass Deblocking
#if TILES_PARALLEL
    if (dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode == 1 && total_tile_cnt == 1) {
        //Jing: Don't work for tile_parallel since the SB of bottom tile comes early than the bottom SB of top tile
#else
if (dlf_enable_flag && pcs_ptr->parent_pcs_ptr->loop_filter_mode == 1) {
#endif
    if (pcs_ptr->parent_pcs_ptr->frm_hdr.loop_filter_params.filter_level[0] ||
        pcs_ptr->parent_pcs_ptr->frm_hdr.loop_filter_params.filter_level[1]) {
        uint8_t last_col =
            ((sb_origin_x) + sb_width == pcs_ptr->parent_pcs_ptr->aligned_width) ? 1 : 0;
        loop_filter_sb(
            recon_buffer, pcs_ptr, NULL, sb_origin_y >> 2, sb_origin_x >> 2, 0, 3, last_col);
    }
}

return;
}

#if NO_ENCDEC
EB_EXTERN void no_enc_dec_pass(SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                               SuperBlock *sb_ptr, uint32_t sb_addr, uint32_t sb_origin_x,
                               uint32_t sb_origin_y, uint32_t sb_qp, EncDecContext *context_ptr) {
    context_ptr->coded_area_sb    = 0;
    context_ptr->coded_area_sb_uv = 0;

    uint32_t final_blk_itr = 0;

    uint32_t blk_it = 0;

    while (blk_it < scs_ptr->max_block_cnt) {
        BlkStruct *blk_ptr = context_ptr->blk_ptr =
            &context_ptr->md_context->md_blk_arr_nsq[blk_it];
        PartitionType    part     = blk_ptr->part;
        const BlockGeom *blk_geom = context_ptr->blk_geom = get_blk_geom_mds(blk_it);

        sb_ptr->cu_partition_array[blk_it] = context_ptr->md_context->md_blk_arr_nsq[blk_it].part;

        if (part != PARTITION_SPLIT) {
            int32_t offset_d1 = ns_blk_offset[(int32_t)part]; //blk_ptr->best_d1_blk; // TOCKECK
            int32_t num_d1_block =
                ns_blk_num[(int32_t)part]; // context_ptr->blk_geom->totns; // TOCKECK

            for (int32_t d1_itr = blk_it + offset_d1; d1_itr < blk_it + offset_d1 + num_d1_block;
                 d1_itr++) {
                const BlockGeom *blk_geom = context_ptr->blk_geom = get_blk_geom_mds(d1_itr);
                BlkStruct *     blk_ptr                          = context_ptr->blk_ptr =
                    &context_ptr->md_context->md_blk_arr_nsq[d1_itr];

                blk_ptr->delta_qp     = 0;
                blk_ptr->qp           = pcs_ptr->picture_qp;
                sb_ptr->qp            = pcs_ptr->picture_qp;

                {
                    BlkStruct *src_cu = &context_ptr->md_context->md_blk_arr_nsq[d1_itr];
                    BlkStruct *dst_cu = &sb_ptr->final_blk_arr[final_blk_itr++];

                    move_blk_data(src_cu, dst_cu);
                }

                //copy coeff
                int32_t txb_1d_offset = 0, txb_1d_offset_uv = 0;

                int32_t txb_itr = 0;
                do {
                    uint32_t bwidth = context_ptr->blk_geom->tx_width[txb_itr] < 64
                                          ? context_ptr->blk_geom->tx_width[txb_itr]
                                          : 32;
                    uint32_t bheight = context_ptr->blk_geom->tx_height[txb_itr] < 64
                                           ? context_ptr->blk_geom->tx_height[txb_itr]
                                           : 32;

                    int32_t *src_ptr =
                        &(((int32_t *)context_ptr->blk_ptr->coeff_tmp->buffer_y)[txb_1d_offset]);
                    int32_t *dst_ptr = &(
                        ((int32_t *)sb_ptr->quantized_coeff->buffer_y)[context_ptr->coded_area_sb]);

                    uint32_t j;
                    for (j = 0; j < bheight; j++)
                        memcpy(
                            dst_ptr + j * bwidth, src_ptr + j * bwidth, bwidth * sizeof(int32_t));
                    if (context_ptr->blk_geom->has_uv) {
                        // Cb
                        bwidth  = context_ptr->blk_geom->tx_width_uv[txb_itr];
                        bheight = context_ptr->blk_geom->tx_height_uv[txb_itr];

                        src_ptr =
                            &(((int32_t *)
                                   context_ptr->blk_ptr->coeff_tmp->buffer_cb)[txb_1d_offset_uv]);
                        dst_ptr = &(((int32_t *)sb_ptr->quantized_coeff
                                         ->buffer_cb)[context_ptr->coded_area_sb_uv]);

                        for (j = 0; j < bheight; j++)
                            memcpy(dst_ptr + j * bwidth,
                                   src_ptr + j * bwidth,
                                   bwidth * sizeof(int32_t));
                        //Cr
                        src_ptr =
                            &(((int32_t *)
                                   context_ptr->blk_ptr->coeff_tmp->buffer_cr)[txb_1d_offset_uv]);
                        dst_ptr = &(((int32_t *)sb_ptr->quantized_coeff
                                         ->buffer_cr)[context_ptr->coded_area_sb_uv]);

                        for (j = 0; j < bheight; j++)
                            memcpy(dst_ptr + j * bwidth,
                                   src_ptr + j * bwidth,
                                   bwidth * sizeof(int32_t));
                    }

                    context_ptr->coded_area_sb += context_ptr->blk_geom->tx_width[txb_itr] *
                                                  context_ptr->blk_geom->tx_height[txb_itr];
                    if (context_ptr->blk_geom->has_uv)
                        context_ptr->coded_area_sb_uv +=
                            context_ptr->blk_geom->tx_width_uv[txb_itr] *
                            context_ptr->blk_geom->tx_height_uv[txb_itr];

                    txb_1d_offset += context_ptr->blk_geom->tx_width[txb_itr] *
                                     context_ptr->blk_geom->tx_height[txb_itr];
                    if (context_ptr->blk_geom->has_uv)
                        txb_1d_offset_uv += context_ptr->blk_geom->tx_width_uv[txb_itr] *
                                            context_ptr->blk_geom->tx_height_uv[txb_itr];

                    txb_itr++;
                } while (txb_itr < context_ptr->blk_geom->txb_count);

                //copy recon
                {
                    EbPictureBufferDesc *ref_pic;
                    if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) {
                        EbReferenceObject *ref_obj =
                            (EbReferenceObject *)
                                pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr;
                        ref_pic = ref_obj->reference_picture;
                    } else
                        ref_pic = pcs_ptr->recon_picture_ptr;
                    context_ptr->blk_origin_x = sb_origin_x + context_ptr->blk_geom->origin_x;
                    context_ptr->blk_origin_y = sb_origin_y + context_ptr->blk_geom->origin_y;

                    uint32_t bwidth  = context_ptr->blk_geom->bwidth;
                    uint32_t bheight = context_ptr->blk_geom->bheight;

                    uint8_t *src_ptr = &(((uint8_t *)context_ptr->blk_ptr->recon_tmp->buffer_y)[0]);
                    uint8_t *dst_ptr =
                        ref_pic->buffer_y + ref_pic->origin_x + context_ptr->blk_origin_x +
                        (ref_pic->origin_y + context_ptr->blk_origin_y) * ref_pic->stride_y;

                    uint32_t j;
                    for (j = 0; j < bheight; j++)
                        memcpy(dst_ptr + j * ref_pic->stride_y,
                               src_ptr + j * 128,
                               bwidth * sizeof(uint8_t));
                    if (context_ptr->blk_geom->has_uv) {
                        bwidth  = context_ptr->blk_geom->bwidth_uv;
                        bheight = context_ptr->blk_geom->bheight_uv;

                        src_ptr = &(((uint8_t *)context_ptr->blk_ptr->recon_tmp->buffer_cb)[0]);

                        dst_ptr =
                            ref_pic->buffer_cb + ref_pic->origin_x / 2 +
                            ((context_ptr->blk_origin_x >> 3) << 3) / 2 +
                            (ref_pic->origin_y / 2 + ((context_ptr->blk_origin_y >> 3) << 3) / 2) *
                                ref_pic->stride_cb;

                        for (j = 0; j < bheight; j++)
                            memcpy(dst_ptr + j * ref_pic->stride_cb,
                                   src_ptr + j * 64,
                                   bwidth * sizeof(uint8_t));
                        src_ptr = &(((uint8_t *)context_ptr->blk_ptr->recon_tmp->buffer_cr)[0]);

                        dst_ptr =
                            ref_pic->buffer_cr + ref_pic->origin_x / 2 +
                            ((context_ptr->blk_origin_x >> 3) << 3) / 2 +
                            (ref_pic->origin_y / 2 + ((context_ptr->blk_origin_y >> 3) << 3) / 2) *
                                ref_pic->stride_cr;

                        for (j = 0; j < bheight; j++)
                            memcpy(dst_ptr + j * ref_pic->stride_cr,
                                   src_ptr + j * 64,
                                   bwidth * sizeof(uint8_t));
                    }
                }
            }
            blk_it +=
                ns_depth_offset[scs_ptr->sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
        } else
            blk_it +=
                d1_depth_offset[scs_ptr->sb_size == BLOCK_128X128][context_ptr->blk_geom->depth];
    } // CU Loop

    return;
}
#endif
