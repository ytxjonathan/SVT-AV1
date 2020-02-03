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

#include <stdlib.h>

#include "EbDefinitions.h"
#include "EbUtility.h"
#include "EbTransformUnit.h"
#include "EbRateDistortionCost.h"
#include "EbFullLoop.h"
#include "EbPictureOperators.h"
#include "EbModeDecisionProcess.h"
#include "EbTransforms.h"
#include "EbMotionEstimation.h"
#include "aom_dsp_rtcd.h"
#include "EbCodingLoop.h"
#include "partition_model_weights.h"
#include "ml.h"
#include "EbLog.h"
#include "EbCommonUtils.h"

EbErrorType generate_md_stage_0_cand(SuperBlock *sb_ptr, ModeDecisionContext *context_ptr,
                                     uint32_t *         fast_candidate_total_count,
                                     PictureControlSet *pcs_ptr);

int16_t eb_av1_dc_quant_qtx(int32_t qindex, int32_t delta, AomBitDepth bit_depth);

static INLINE int is_interintra_allowed_bsize(const BlockSize bsize) {
    return (bsize >= BLOCK_8X8) && (bsize <= BLOCK_32X32);
}
void precompute_intra_pred_for_inter_intra(PictureControlSet *  pcs_ptr,
                                           ModeDecisionContext *context_ptr);

int svt_av1_allow_palette(int allow_palette, BlockSize sb_type);

/*******************************************
* set Penalize Skip Flag
*
* Summary: Set the penalize_skipflag to true
* When there is luminance/chrominance change
* or in noisy clip with low motion at meduim
* varince area
*
*******************************************/

const EbPredictionFunc product_prediction_fun_table[3] = {
    NULL, inter_pu_prediction_av1, eb_av1_intra_prediction_cl};

const EbFastCostFunc av1_product_fast_cost_func_table[3] = {
    NULL,
    av1_inter_fast_cost, /*INTER */
    av1_intra_fast_cost /*INTRA */
};

const EbAv1FullCostFunc av1_product_full_cost_func_table[3] = {
    NULL,
    av1_inter_full_cost, /*INTER */
    av1_intra_full_cost /*INTRA */
};

/***************************************************
* Update Recon Samples Neighbor Arrays
***************************************************/
void mode_decision_update_neighbor_arrays(PictureControlSet *  pcs_ptr,
                                          ModeDecisionContext *context_ptr, uint32_t index_mds,
                                          EbBool intraMdOpenLoop, EbBool intra4x4Selected) {
    uint32_t bwdith  = context_ptr->blk_geom->bwidth;
    uint32_t bheight = context_ptr->blk_geom->bheight;

    uint32_t origin_x = context_ptr->blk_origin_x;
    uint32_t origin_y = context_ptr->blk_origin_y;
    (void)intra4x4Selected;

    uint32_t blk_origin_x_uv = context_ptr->round_origin_x >> 1;
    uint32_t blk_origin_y_uv = context_ptr->round_origin_y >> 1;
    uint32_t bwdith_uv      = context_ptr->blk_geom->bwidth_uv;
    uint32_t bwheight_uv    = context_ptr->blk_geom->bheight_uv;

    uint8_t mode_type       = context_ptr->blk_ptr->prediction_mode_flag;
    uint8_t intra_luma_mode = (uint8_t)context_ptr->blk_ptr->pred_mode;
    uint8_t chroma_mode = (uint8_t)context_ptr->blk_ptr->prediction_unit_array->intra_chroma_mode;
    uint8_t skip_flag   = (uint8_t)context_ptr->blk_ptr->skip_flag;

    context_ptr->mv_unit.pred_direction = (uint8_t)(
        context_ptr->md_blk_arr_nsq[index_mds].prediction_unit_array[0].inter_pred_direction_index);
    context_ptr->mv_unit.mv[REF_LIST_0].mv_union =
        context_ptr->md_blk_arr_nsq[index_mds].prediction_unit_array[0].mv[REF_LIST_0].mv_union;
    context_ptr->mv_unit.mv[REF_LIST_1].mv_union =
        context_ptr->md_blk_arr_nsq[index_mds].prediction_unit_array[0].mv[REF_LIST_1].mv_union;
    uint8_t inter_pred_direction_index =
        (uint8_t)context_ptr->blk_ptr->prediction_unit_array->inter_pred_direction_index;
    uint8_t ref_frame_type = (uint8_t)context_ptr->blk_ptr->prediction_unit_array[0].ref_frame_type;
#if TX_ORG_INTERINTRA
    int32_t is_inter = (context_ptr->blk_ptr->prediction_mode_flag == INTER_MODE ||
                        context_ptr->blk_ptr->av1xd->use_intrabc)
                           ? EB_TRUE
                           : EB_FALSE;
#endif

#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
#endif

    if (context_ptr->interpolation_search_level != IT_SEARCH_OFF)
        neighbor_array_unit_mode_write32(context_ptr->interpolation_type_neighbor_array,
                                         context_ptr->blk_ptr->interp_filters,
                                         origin_x,
                                         origin_y,
                                         bwdith,
                                         bheight,
                                         NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    {
        struct PartitionContext partition;
        partition.above = partition_context_lookup[context_ptr->blk_geom->bsize].above;
        partition.left  = partition_context_lookup[context_ptr->blk_geom->bsize].left;

        neighbor_array_unit_mode_write(context_ptr->leaf_partition_neighbor_array,
                                       (uint8_t *)(&partition), // NaderM
                                       origin_x,
                                       origin_y,
                                       bwdith,
                                       bheight,
                                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

        // Mode Type Update
        neighbor_array_unit_mode_write(context_ptr->mode_type_neighbor_array,
                                       &mode_type,
                                       origin_x,
                                       origin_y,
                                       bwdith,
                                       bheight,
                                       NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        // Intra Luma Mode Update
        neighbor_array_unit_mode_write(context_ptr->intra_luma_mode_neighbor_array,
                                       &intra_luma_mode, //(uint8_t*)luma_mode,
                                       origin_x,
                                       origin_y,
                                       bwdith,
                                       bheight,
                                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

        uint16_t txb_count = context_ptr->blk_geom->txb_count[context_ptr->blk_ptr->tx_depth];
        for (uint8_t txb_itr = 0; txb_itr < txb_count; txb_itr++) {
            uint8_t dc_sign_level_coeff = (int32_t)context_ptr->blk_ptr->quantized_dc[0][txb_itr];

            neighbor_array_unit_mode_write(
                context_ptr->luma_dc_sign_level_coeff_neighbor_array,
                (uint8_t *)&dc_sign_level_coeff,
#if TX_ORG_INTERINTRA
                context_ptr->sb_origin_x +
                    context_ptr->blk_geom
                        ->tx_org_x[is_inter][context_ptr->blk_ptr->tx_depth][txb_itr],
                context_ptr->sb_origin_y +
                    context_ptr->blk_geom
                        ->tx_org_y[is_inter][context_ptr->blk_ptr->tx_depth][txb_itr],
#else
                context_ptr->sb_origin_x +
                    context_ptr->blk_geom->tx_org_x[context_ptr->blk_ptr->tx_depth][txb_itr],
                context_ptr->sb_origin_y +
                    context_ptr->blk_geom->tx_org_y[context_ptr->blk_ptr->tx_depth][txb_itr],
#endif
                context_ptr->blk_geom->tx_width[context_ptr->blk_ptr->tx_depth][txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->blk_ptr->tx_depth][txb_itr],
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

            neighbor_array_unit_mode_write(
                pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array
#if TILES_PARALLEL
                    [MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                    [MD_NEIGHBOR_ARRAY_INDEX],
#endif
                (uint8_t *)&dc_sign_level_coeff,
#if TX_ORG_INTERINTRA
                context_ptr->sb_origin_x +
                    context_ptr->blk_geom
                        ->tx_org_x[is_inter][context_ptr->blk_ptr->tx_depth][txb_itr],
                context_ptr->sb_origin_y +
                    context_ptr->blk_geom
                        ->tx_org_y[is_inter][context_ptr->blk_ptr->tx_depth][txb_itr],
#else
                context_ptr->sb_origin_x +
                    context_ptr->blk_geom->tx_org_x[context_ptr->blk_ptr->tx_depth][txb_itr],
                context_ptr->sb_origin_y +
                    context_ptr->blk_geom->tx_org_y[context_ptr->blk_ptr->tx_depth][txb_itr],
#endif
                context_ptr->blk_geom->tx_width[context_ptr->blk_ptr->tx_depth][txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->blk_ptr->tx_depth][txb_itr],
                NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }
    }

    // Hsan: chroma mode rate estimation is kept even for chroma blind
    if (context_ptr->blk_geom->has_uv) {
        // Intra Chroma Mode Update
        neighbor_array_unit_mode_write(context_ptr->intra_chroma_mode_neighbor_array,
                                       &chroma_mode,
                                       blk_origin_x_uv,
                                       blk_origin_y_uv,
                                       bwdith_uv,
                                       bwheight_uv,
                                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }

    neighbor_array_unit_mode_write(context_ptr->skip_flag_neighbor_array,
                                   &skip_flag,
                                   origin_x,
                                   origin_y,
                                   bwdith,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
        //  Update chroma CB cbf and Dc context
        {
            uint8_t dc_sign_level_coeff = (int32_t)context_ptr->blk_ptr->quantized_dc[1][0];
            neighbor_array_unit_mode_write(context_ptr->cb_dc_sign_level_coeff_neighbor_array,
                                           (uint8_t *)&dc_sign_level_coeff,
                                           blk_origin_x_uv,
                                           blk_origin_y_uv,
                                           bwdith_uv,
                                           bwheight_uv,
                                           NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }

        //  Update chroma CR cbf and Dc context
        {
            uint8_t dc_sign_level_coeff = (int32_t)context_ptr->blk_ptr->quantized_dc[2][0];
            neighbor_array_unit_mode_write(context_ptr->cr_dc_sign_level_coeff_neighbor_array,
                                           (uint8_t *)&dc_sign_level_coeff,
                                           blk_origin_x_uv,
                                           blk_origin_y_uv,
                                           bwdith_uv,
                                           bwheight_uv,
                                           NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        }
    }
    uint8_t tx_size =
        tx_depth_to_tx_size[context_ptr->blk_ptr->tx_depth][context_ptr->blk_geom->bsize];
    uint8_t bw = tx_size_wide[tx_size];
    uint8_t bh = tx_size_high[tx_size];

    neighbor_array_unit_mode_write(context_ptr->txfm_context_array,
                                   &bw,
                                   origin_x,
                                   origin_y,
                                   bwdith,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_TOP_MASK);

    neighbor_array_unit_mode_write(context_ptr->txfm_context_array,
                                   &bh,
                                   origin_x,
                                   origin_y,
                                   bwdith,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_LEFT_MASK);

    // Update the Inter Pred Type Neighbor Array

    neighbor_array_unit_mode_write(context_ptr->inter_pred_dir_neighbor_array,
                                   &inter_pred_direction_index,
                                   origin_x,
                                   origin_y,
                                   bwdith,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    // Update the refFrame Type Neighbor Array
    neighbor_array_unit_mode_write(context_ptr->ref_frame_type_neighbor_array,
                                   &ref_frame_type,
                                   origin_x,
                                   origin_y,
                                   bwdith,
                                   bheight,
                                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    if (!context_ptr->hbd_mode_decision) {
        if (intraMdOpenLoop == EB_FALSE) {
            update_recon_neighbor_array(context_ptr->luma_recon_neighbor_array,
                                        context_ptr->blk_ptr->neigh_top_recon[0],
                                        context_ptr->blk_ptr->neigh_left_recon[0],
                                        origin_x,
                                        origin_y,
                                        context_ptr->blk_geom->bwidth,
                                        context_ptr->blk_geom->bheight);
            if (context_ptr->md_tx_size_search_mode) {
                update_recon_neighbor_array(
#if TILES_PARALLEL
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
#endif
                    context_ptr->blk_ptr->neigh_top_recon[0],
                    context_ptr->blk_ptr->neigh_left_recon[0],
                    origin_x,
                    origin_y,
                    context_ptr->blk_geom->bwidth,
                    context_ptr->blk_geom->bheight);
            }
        }

        if (intraMdOpenLoop == EB_FALSE) {
            if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
                update_recon_neighbor_array(context_ptr->cb_recon_neighbor_array,
                                            context_ptr->blk_ptr->neigh_top_recon[1],
                                            context_ptr->blk_ptr->neigh_left_recon[1],
                                            blk_origin_x_uv,
                                            blk_origin_y_uv,
                                            bwdith_uv,
                                            bwheight_uv);
                update_recon_neighbor_array(context_ptr->cr_recon_neighbor_array,
                                            context_ptr->blk_ptr->neigh_top_recon[2],
                                            context_ptr->blk_ptr->neigh_left_recon[2],
                                            blk_origin_x_uv,
                                            blk_origin_y_uv,
                                            bwdith_uv,
                                            bwheight_uv);
            }
        }
    } else {
        if (intraMdOpenLoop == EB_FALSE)
            update_recon_neighbor_array16bit(context_ptr->luma_recon_neighbor_array16bit,
                                             context_ptr->blk_ptr->neigh_top_recon_16bit[0],
                                             context_ptr->blk_ptr->neigh_left_recon_16bit[0],
                                             origin_x,
                                             origin_y,
                                             context_ptr->blk_geom->bwidth,
                                             context_ptr->blk_geom->bheight);
        if (context_ptr->md_tx_size_search_mode) {
            update_recon_neighbor_array16bit(
#if TILES_PARALLEL
                pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX],
#endif
                context_ptr->blk_ptr->neigh_top_recon_16bit[0],
                context_ptr->blk_ptr->neigh_left_recon_16bit[0],
                origin_x,
                origin_y,
                context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight);
        }

        if (intraMdOpenLoop == EB_FALSE && context_ptr->blk_geom->has_uv &&
            context_ptr->chroma_level <= CHROMA_MODE_1) {
            update_recon_neighbor_array16bit(context_ptr->cb_recon_neighbor_array16bit,
                                             context_ptr->blk_ptr->neigh_top_recon_16bit[1],
                                             context_ptr->blk_ptr->neigh_left_recon_16bit[1],
                                             blk_origin_x_uv,
                                             blk_origin_y_uv,
                                             bwdith_uv,
                                             bwheight_uv);
            update_recon_neighbor_array16bit(context_ptr->cr_recon_neighbor_array16bit,
                                             context_ptr->blk_ptr->neigh_top_recon_16bit[2],
                                             context_ptr->blk_ptr->neigh_left_recon_16bit[2],
                                             blk_origin_x_uv,
                                             blk_origin_y_uv,
                                             bwdith_uv,
                                             bwheight_uv);
        }
    }

    return;
}

void copy_neighbour_arrays(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                           uint32_t src_idx, uint32_t dst_idx, uint32_t blk_mds, uint32_t sb_org_x,
                           uint32_t sb_org_y) {
#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
#else
    (void)*context_ptr;
#endif

    const BlockGeom *blk_geom = get_blk_geom_mds(blk_mds);

    uint32_t blk_org_x    = sb_org_x + blk_geom->origin_x;
    uint32_t blk_org_y    = sb_org_y + blk_geom->origin_y;
    uint32_t blk_org_x_uv = (blk_org_x >> 3 << 3) >> 1;
    uint32_t blk_org_y_uv = (blk_org_y >> 3 << 3) >> 1;
    uint32_t bwidth_uv    = blk_geom->bwidth_uv;
    uint32_t bheight_uv   = blk_geom->bheight_uv;

#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_intra_luma_mode_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_intra_luma_mode_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_intra_luma_mode_neighbor_array[src_idx],
                   pcs_ptr->md_intra_luma_mode_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(pcs_ptr->md_intra_chroma_mode_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_intra_chroma_mode_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_intra_chroma_mode_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_intra_chroma_mode_neighbor_array[src_idx],
                   pcs_ptr->md_intra_chroma_mode_neighbor_array[dst_idx],
#endif
                   blk_org_x_uv,
                   blk_org_y_uv,
                   bwidth_uv,
                   bheight_uv,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(pcs_ptr->md_skip_flag_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_skip_flag_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_skip_flag_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_skip_flag_neighbor_array[src_idx],
                   pcs_ptr->md_skip_flag_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    //neighbor_array_unit_reset(pcs_ptr->md_mode_type_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_mode_type_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_mode_type_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_mode_type_neighbor_array[src_idx],
                   pcs_ptr->md_mode_type_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_FULL_MASK);

    //neighbor_array_unit_reset(pcs_ptr->md_leaf_depth_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_leaf_depth_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_leaf_depth_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_leaf_depth_neighbor_array[src_idx],
                   pcs_ptr->md_leaf_depth_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->mdleaf_partition_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->mdleaf_partition_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->mdleaf_partition_neighbor_array[src_idx],
                   pcs_ptr->mdleaf_partition_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    if (!context_ptr->hbd_mode_decision) {
#if TILES_PARALLEL
        copy_neigh_arr(pcs_ptr->md_luma_recon_neighbor_array[src_idx][tile_idx],
                       pcs_ptr->md_luma_recon_neighbor_array[dst_idx][tile_idx],
#else
        copy_neigh_arr(pcs_ptr->md_luma_recon_neighbor_array[src_idx],
                       pcs_ptr->md_luma_recon_neighbor_array[dst_idx],
#endif
                       blk_org_x,
                       blk_org_y,
                       blk_geom->bwidth,
                       blk_geom->bheight,
                       NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        if (context_ptr->md_tx_size_search_mode) {
#if TILES_PARALLEL
            copy_neigh_arr(pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[src_idx][tile_idx],
                           pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[dst_idx][tile_idx],
#else
            copy_neigh_arr(pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[src_idx],
                           pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[dst_idx],
#endif
                           blk_org_x,
                           blk_org_y,
                           blk_geom->bwidth,
                           blk_geom->bheight,
                           NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }
        if (blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
#if TILES_PARALLEL
            copy_neigh_arr(pcs_ptr->md_cb_recon_neighbor_array[src_idx][tile_idx],
                           pcs_ptr->md_cb_recon_neighbor_array[dst_idx][tile_idx],
#else
            copy_neigh_arr(pcs_ptr->md_cb_recon_neighbor_array[src_idx],
                           pcs_ptr->md_cb_recon_neighbor_array[dst_idx],
#endif
                           blk_org_x_uv,
                           blk_org_y_uv,
                           bwidth_uv,
                           bheight_uv,
                           NEIGHBOR_ARRAY_UNIT_FULL_MASK);

#if TILES_PARALLEL
            copy_neigh_arr(pcs_ptr->md_cr_recon_neighbor_array[src_idx][tile_idx],
                           pcs_ptr->md_cr_recon_neighbor_array[dst_idx][tile_idx],
#else
            copy_neigh_arr(pcs_ptr->md_cr_recon_neighbor_array[src_idx],
                           pcs_ptr->md_cr_recon_neighbor_array[dst_idx],
#endif
                           blk_org_x_uv,
                           blk_org_y_uv,
                           bwidth_uv,
                           bheight_uv,
                           NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }
    } else {
#if TILES_PARALLEL
        copy_neigh_arr(pcs_ptr->md_luma_recon_neighbor_array16bit[src_idx][tile_idx],
                       pcs_ptr->md_luma_recon_neighbor_array16bit[dst_idx][tile_idx],
#else
        copy_neigh_arr(pcs_ptr->md_luma_recon_neighbor_array16bit[src_idx],
                       pcs_ptr->md_luma_recon_neighbor_array16bit[dst_idx],
#endif
                       blk_org_x,
                       blk_org_y,
                       blk_geom->bwidth,
                       blk_geom->bheight,
                       NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        if (context_ptr->md_tx_size_search_mode) {
#if TILES_PARALLEL
            copy_neigh_arr(pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[src_idx][tile_idx],
                           pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[dst_idx][tile_idx],
#else
            copy_neigh_arr(pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[src_idx],
                           pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[dst_idx],
#endif
                           blk_org_x,
                           blk_org_y,
                           blk_geom->bwidth,
                           blk_geom->bheight,
                           NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }

        if (blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
#if TILES_PARALLEL
            copy_neigh_arr(pcs_ptr->md_cb_recon_neighbor_array16bit[src_idx][tile_idx],
                           pcs_ptr->md_cb_recon_neighbor_array16bit[dst_idx][tile_idx],
#else
            copy_neigh_arr(pcs_ptr->md_cb_recon_neighbor_array16bit[src_idx],
                           pcs_ptr->md_cb_recon_neighbor_array16bit[dst_idx],
#endif
                           blk_org_x_uv,
                           blk_org_y_uv,
                           bwidth_uv,
                           bheight_uv,
                           NEIGHBOR_ARRAY_UNIT_FULL_MASK);

#if TILES_PARALLEL
            copy_neigh_arr(pcs_ptr->md_cr_recon_neighbor_array16bit[src_idx][tile_idx],
                           pcs_ptr->md_cr_recon_neighbor_array16bit[dst_idx][tile_idx],
#else
            copy_neigh_arr(pcs_ptr->md_cr_recon_neighbor_array16bit[src_idx],
                           pcs_ptr->md_cr_recon_neighbor_array16bit[dst_idx],
#endif
                           blk_org_x_uv,
                           blk_org_y_uv,
                           bwidth_uv,
                           bheight_uv,
                           NEIGHBOR_ARRAY_UNIT_FULL_MASK);
        }
    }

    //neighbor_array_unit_reset(pcs_ptr->md_skip_coeff_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_skip_coeff_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_skip_coeff_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_skip_coeff_neighbor_array[src_idx],
                   pcs_ptr->md_skip_coeff_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    //neighbor_array_unit_reset(pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[src_idx],
                   pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[src_idx],
                   pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

    if (blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
#if TILES_PARALLEL
        copy_neigh_arr(pcs_ptr->md_cb_dc_sign_level_coeff_neighbor_array[src_idx][tile_idx],
                       pcs_ptr->md_cb_dc_sign_level_coeff_neighbor_array[dst_idx][tile_idx],
#else
        copy_neigh_arr(pcs_ptr->md_cb_dc_sign_level_coeff_neighbor_array[src_idx],
                       pcs_ptr->md_cb_dc_sign_level_coeff_neighbor_array[dst_idx],
#endif
                       blk_org_x_uv,
                       blk_org_y_uv,
                       bwidth_uv,
                       bheight_uv,
                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
        //neighbor_array_unit_reset(pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[depth]);

#if TILES_PARALLEL
        copy_neigh_arr(pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[src_idx][tile_idx],
                       pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[dst_idx][tile_idx],
#else
        copy_neigh_arr(pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[src_idx],
                       pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[dst_idx],
#endif
                       blk_org_x_uv,
                       blk_org_y_uv,
                       bwidth_uv,
                       bheight_uv,
                       NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }

    //neighbor_array_unit_reset(pcs_ptr->md_txfm_context_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_txfm_context_array[src_idx][tile_idx],
                   pcs_ptr->md_txfm_context_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_txfm_context_array[src_idx],
                   pcs_ptr->md_txfm_context_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    //neighbor_array_unit_reset(pcs_ptr->md_inter_pred_dir_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_inter_pred_dir_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_inter_pred_dir_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_inter_pred_dir_neighbor_array[src_idx],
                   pcs_ptr->md_inter_pred_dir_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    //neighbor_array_unit_reset(pcs_ptr->md_ref_frame_type_neighbor_array[depth]);
#if TILES_PARALLEL
    copy_neigh_arr(pcs_ptr->md_ref_frame_type_neighbor_array[src_idx][tile_idx],
                   pcs_ptr->md_ref_frame_type_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr(pcs_ptr->md_ref_frame_type_neighbor_array[src_idx],
                   pcs_ptr->md_ref_frame_type_neighbor_array[dst_idx],
#endif
                   blk_org_x,
                   blk_org_y,
                   blk_geom->bwidth,
                   blk_geom->bheight,
                   NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);

#if TILES_PARALLEL
    copy_neigh_arr_32(pcs_ptr->md_interpolation_type_neighbor_array[src_idx][tile_idx],
                      pcs_ptr->md_interpolation_type_neighbor_array[dst_idx][tile_idx],
#else
    copy_neigh_arr_32(pcs_ptr->md_interpolation_type_neighbor_array[src_idx],
                      pcs_ptr->md_interpolation_type_neighbor_array[dst_idx],
#endif
                      blk_org_x,
                      blk_org_y,
                      blk_geom->bwidth,
                      blk_geom->bheight,
                      NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
}

void md_update_all_neighbour_arrays(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                    uint32_t last_blk_index_mds, uint32_t sb_origin_x,
                                    uint32_t sb_origin_y) {
    context_ptr->blk_geom       = get_blk_geom_mds(last_blk_index_mds);
    context_ptr->blk_origin_x   = sb_origin_x + context_ptr->blk_geom->origin_x;
    context_ptr->blk_origin_y   = sb_origin_y + context_ptr->blk_geom->origin_y;
    context_ptr->round_origin_x = ((context_ptr->blk_origin_x >> 3) << 3);
    context_ptr->round_origin_y = ((context_ptr->blk_origin_y >> 3) << 3);

    context_ptr->blk_ptr = &context_ptr->md_blk_arr_nsq[last_blk_index_mds];

    mode_decision_update_neighbor_arrays(
        pcs_ptr, context_ptr, last_blk_index_mds, pcs_ptr->intra_md_open_loop_flag, EB_FALSE);

    update_mi_map(context_ptr,
                  context_ptr->blk_ptr,
                  context_ptr->blk_origin_x,
                  context_ptr->blk_origin_y,
                  context_ptr->blk_geom,
                  pcs_ptr);
}

void md_update_all_neighbour_arrays_multiple(PictureControlSet *  pcs_ptr,
                                             ModeDecisionContext *context_ptr, uint32_t blk_mds,
                                             uint32_t sb_origin_x, uint32_t sb_origin_y) {
    context_ptr->blk_geom = get_blk_geom_mds(blk_mds);

    uint32_t blk_it;
    for (blk_it = 0; blk_it < context_ptr->blk_geom->totns; blk_it++) {
        md_update_all_neighbour_arrays(
            pcs_ptr, context_ptr, blk_mds + blk_it, sb_origin_x, sb_origin_y);
    }
}

#define TOTAL_SQ_BLOCK_COUNT 341
int sq_block_index[TOTAL_SQ_BLOCK_COUNT] = {
    0,    25,   50,   75,   80,   81,   82,   83,   84,   89,   90,   91,   92,   93,   98,   99,
    100,  101,  102,  107,  108,  109,  110,  111,  136,  141,  142,  143,  144,  145,  150,  151,
    152,  153,  154,  159,  160,  161,  162,  163,  168,  169,  170,  171,  172,  197,  202,  203,
    204,  205,  206,  211,  212,  213,  214,  215,  220,  221,  222,  223,  224,  229,  230,  231,
    232,  233,  258,  263,  264,  265,  266,  267,  272,  273,  274,  275,  276,  281,  282,  283,
    284,  285,  290,  291,  292,  293,  294,  319,  344,  349,  350,  351,  352,  353,  358,  359,
    360,  361,  362,  367,  368,  369,  370,  371,  376,  377,  378,  379,  380,  405,  410,  411,
    412,  413,  414,  419,  420,  421,  422,  423,  428,  429,  430,  431,  432,  437,  438,  439,
    440,  441,  466,  471,  472,  473,  474,  475,  480,  481,  482,  483,  484,  489,  490,  491,
    492,  493,  498,  499,  500,  501,  502,  527,  532,  533,  534,  535,  536,  541,  542,  543,
    544,  545,  550,  551,  552,  553,  554,  559,  560,  561,  562,  563,  588,  613,  618,  619,
    620,  621,  622,  627,  628,  629,  630,  631,  636,  637,  638,  639,  640,  645,  646,  647,
    648,  649,  674,  679,  680,  681,  682,  683,  688,  689,  690,  691,  692,  697,  698,  699,
    700,  701,  706,  707,  708,  709,  710,  735,  740,  741,  742,  743,  744,  749,  750,  751,
    752,  753,  758,  759,  760,  761,  762,  767,  768,  769,  770,  771,  796,  801,  802,  803,
    804,  805,  810,  811,  812,  813,  814,  819,  820,  821,  822,  823,  828,  829,  830,  831,
    832,  857,  882,  887,  888,  889,  890,  891,  896,  897,  898,  899,  900,  905,  906,  907,
    908,  909,  914,  915,  916,  917,  918,  943,  948,  949,  950,  951,  952,  957,  958,  959,
    960,  961,  966,  967,  968,  969,  970,  975,  976,  977,  978,  979,  1004, 1009, 1010, 1011,
    1012, 1013, 1018, 1019, 1020, 1021, 1022, 1027, 1028, 1029, 1030, 1031, 1036, 1037, 1038, 1039,
    1040, 1065, 1070, 1071, 1072, 1073, 1074, 1079, 1080, 1081, 1082, 1083, 1088, 1089, 1090, 1091,
    1092, 1097, 1098, 1099, 1100};
void init_sq_nsq_block(SequenceControlSet *scs_ptr, ModeDecisionContext *context_ptr) {
    uint32_t blk_idx = 0;
    do {
        const BlockGeom *blk_geom                              = get_blk_geom_mds(blk_idx);
        context_ptr->md_local_blk_unit[blk_idx].avail_blk_flag = EB_FALSE;
        context_ptr->md_local_blk_unit[blk_idx].left_neighbor_partition = INVALID_NEIGHBOR_DATA;
        context_ptr->md_local_blk_unit[blk_idx].above_neighbor_partition = INVALID_NEIGHBOR_DATA;
        if (blk_geom->shape == PART_N) {
            context_ptr->md_blk_arr_nsq[blk_idx].split_flag         = EB_TRUE;
            context_ptr->md_blk_arr_nsq[blk_idx].part               = PARTITION_SPLIT;
            context_ptr->md_local_blk_unit[blk_idx].tested_blk_flag = EB_FALSE;
        }
        ++blk_idx;
    } while (blk_idx < scs_ptr->max_block_cnt);
}
void init_sq_non4_block(SequenceControlSet *scs_ptr, ModeDecisionContext *context_ptr) {
    for (uint32_t blk_idx = 0; blk_idx < TOTAL_SQ_BLOCK_COUNT; blk_idx++) {
        context_ptr->md_blk_arr_nsq[sq_block_index[blk_idx]].part               = PARTITION_SPLIT;
        context_ptr->md_local_blk_unit[sq_block_index[blk_idx]].tested_blk_flag = EB_FALSE;
    }
    for (uint32_t blk_idx = 0; blk_idx < scs_ptr->max_block_cnt; ++blk_idx) {
        context_ptr->md_local_blk_unit[blk_idx].avail_blk_flag = EB_FALSE;
        context_ptr->md_local_blk_unit[blk_idx].left_neighbor_partition = INVALID_NEIGHBOR_DATA;
        context_ptr->md_local_blk_unit[blk_idx].above_neighbor_partition = INVALID_NEIGHBOR_DATA;
    }
}
static INLINE TranHigh check_range(TranHigh input, int32_t bd) {
    // AV1 TX case
    // - 8 bit: signed 16 bit integer
    // - 10 bit: signed 18 bit integer
    // - 12 bit: signed 20 bit integer
    // - max quantization error = 1828 << (bd - 8)
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

/*********************************
* Picture Single Channel Kernel
*********************************/
void picture_addition_kernel(uint8_t *pred_ptr, uint32_t pred_stride, int32_t *residual_ptr,
                             uint32_t residual_stride, uint8_t *recon_ptr, uint32_t recon_stride,
                             uint32_t width, uint32_t height, int32_t bd) {
    uint32_t column_index;
    uint32_t row_index = 0;
    //    const int32_t    maxValue = 0xFF;

    //SVT_LOG("\n");
    //SVT_LOG("Reconstruction---------------------------------------------------\n");

    while (row_index < height) {
        column_index = 0;
        while (column_index < width) {
            //recon_ptr[column_index] = (uint8_t)CLIP3(0, maxValue, ((int32_t)residual_ptr[column_index]) + ((int32_t)pred_ptr[column_index]));
            uint16_t rec = (uint16_t)pred_ptr[column_index];
            recon_ptr[column_index] =
                (uint8_t)highbd_clip_pixel_add(rec, (TranLow)residual_ptr[column_index], bd);

            //SVT_LOG("%d\t", recon_ptr[column_index]);
            ++column_index;
        }

        //SVT_LOG("\n");
        residual_ptr += residual_stride;
        pred_ptr += pred_stride;
        recon_ptr += recon_stride;
        ++row_index;
    }
    //SVT_LOG("-----------------------------------------------------------------\n");
    //SVT_LOG("\n");
    //SVT_LOG("\n");
    return;
}

void picture_addition_kernel16_bit(uint16_t *pred_ptr, uint32_t pred_stride, int32_t *residual_ptr,
                                   uint32_t residual_stride, uint16_t *recon_ptr,
                                   uint32_t recon_stride, uint32_t width, uint32_t height,
                                   int32_t bd) {
    uint32_t column_index;
    uint32_t row_index = 0;
    //    const int32_t    maxValue = 0xFF;

    //SVT_LOG("\n");
    //SVT_LOG("Reconstruction---------------------------------------------------\n");

    while (row_index < height) {
        column_index = 0;
        while (column_index < width) {
            //recon_ptr[column_index] = (uint8_t)CLIP3(0, maxValue, ((int32_t)residual_ptr[column_index]) + ((int32_t)pred_ptr[column_index]));
            uint16_t rec = (uint16_t)pred_ptr[column_index];
            recon_ptr[column_index] =
                highbd_clip_pixel_add(rec, (TranLow)residual_ptr[column_index], bd);

            //SVT_LOG("%d\t", recon_ptr[column_index]);
            ++column_index;
        }

        //SVT_LOG("\n");
        residual_ptr += residual_stride;
        pred_ptr += pred_stride;
        recon_ptr += recon_stride;
        ++row_index;
    }
    //    SVT_LOG("-----------------------------------------------------------------\n");
    //    SVT_LOG("\n");
    //    SVT_LOG("\n");
    return;
}

void av1_perform_inverse_transform_recon_luma(PictureControlSet *          pcs_ptr,
                                              ModeDecisionContext *        context_ptr,
                                              ModeDecisionCandidateBuffer *candidate_buffer) {
    uint32_t txb_width;
    uint32_t txb_height;
    uint32_t txb_origin_x;
    uint32_t txb_origin_y;
    uint32_t txb_origin_index;
    uint32_t tu_total_count;
    uint32_t txb_itr;

    if (pcs_ptr->intra_md_open_loop_flag == EB_FALSE) {
        uint8_t tx_depth       = candidate_buffer->candidate_ptr->tx_depth;
        tu_total_count         = context_ptr->blk_geom->txb_count[tx_depth];
        txb_itr                = 0;
        uint32_t txb_1d_offset = 0;
#if TX_ORG_INTERINTRA
        int32_t is_inter = (candidate_buffer->candidate_ptr->type == INTER_MODE ||
                            candidate_buffer->candidate_ptr->use_intrabc)
                               ? EB_TRUE
                               : EB_FALSE;
#endif
        do {
#if TX_ORG_INTERINTRA
            txb_origin_x = context_ptr->blk_geom->tx_org_x[is_inter][tx_depth][txb_itr];
            txb_origin_y = context_ptr->blk_geom->tx_org_y[is_inter][tx_depth][txb_itr];
#else
            txb_origin_x = context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr];
            txb_origin_y = context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr];
#endif
            txb_width  = context_ptr->blk_geom->tx_width[tx_depth][txb_itr];
            txb_height = context_ptr->blk_geom->tx_height[tx_depth][txb_itr];
            txb_origin_index =
                txb_origin_x + txb_origin_y * candidate_buffer->prediction_ptr->stride_y;
            uint32_t rec_luma_offset =
                txb_origin_x + txb_origin_y * candidate_buffer->recon_ptr->stride_y;
            uint32_t y_has_coeff =
                (candidate_buffer->candidate_ptr->y_has_coeff & (1 << txb_itr)) > 0;

            if (y_has_coeff)
                inv_transform_recon_wrapper(
                    candidate_buffer->prediction_ptr->buffer_y,
                    txb_origin_index,
                    candidate_buffer->prediction_ptr->stride_y,
                    context_ptr->hbd_mode_decision
                        ? (uint8_t *)context_ptr->cfl_temp_luma_recon16bit
                        : context_ptr->cfl_temp_luma_recon,
                    rec_luma_offset,
                    candidate_buffer->recon_ptr->stride_y,
                    (int32_t *)candidate_buffer->recon_coeff_ptr->buffer_y,
                    txb_1d_offset,
                    context_ptr->hbd_mode_decision,
                    context_ptr->blk_geom->txsize[tx_depth][txb_itr],
                    candidate_buffer->candidate_ptr->transform_type[txb_itr],
                    PLANE_TYPE_Y,
                    (uint32_t)candidate_buffer->candidate_ptr->eob[0][txb_itr]);
            else {
                if (context_ptr->hbd_mode_decision) {
                    pic_copy_kernel_16bit(
                        ((uint16_t *)candidate_buffer->prediction_ptr->buffer_y) + txb_origin_index,
                        candidate_buffer->prediction_ptr->stride_y,
                        context_ptr->cfl_temp_luma_recon16bit + rec_luma_offset,
                        candidate_buffer->recon_ptr->stride_y,
                        txb_width,
                        txb_height);
                } else {
                    pic_copy_kernel_8bit(
                        &(candidate_buffer->prediction_ptr->buffer_y[txb_origin_index]),
                        candidate_buffer->prediction_ptr->stride_y,
                        &(context_ptr->cfl_temp_luma_recon[rec_luma_offset]),
                        candidate_buffer->recon_ptr->stride_y,
                        txb_width,
                        txb_height);
                }
            }
            txb_1d_offset += context_ptr->blk_geom->tx_width[tx_depth][txb_itr] *
                             context_ptr->blk_geom->tx_height[tx_depth][txb_itr];
            ++txb_itr;
        } while (txb_itr < tu_total_count);
    }
}
void av1_perform_inverse_transform_recon(PictureControlSet *          pcs_ptr,
                                         ModeDecisionContext *        context_ptr,
                                         ModeDecisionCandidateBuffer *candidate_buffer,
                                         BlkStruct *blk_ptr, const BlockGeom *blk_geom) {
    uint32_t       txb_width;
    uint32_t       txb_height;
    uint32_t       txb_origin_x;
    uint32_t       txb_origin_y;
    uint32_t       txb_origin_index;
    uint32_t       tu_total_count;
    uint32_t       txb_index;
    uint32_t       txb_itr;
    TransformUnit *txb_ptr;

    UNUSED(blk_geom);

    if (pcs_ptr->intra_md_open_loop_flag == EB_FALSE) {
        uint8_t tx_depth       = candidate_buffer->candidate_ptr->tx_depth;
        tu_total_count         = context_ptr->blk_geom->txb_count[tx_depth];
        txb_index              = 0;
        txb_itr                = 0;
        uint32_t txb_1d_offset = 0, txb_1d_offset_uv = 0;
        uint32_t rec_luma_offset, rec_cb_offset, rec_cr_offset;
#if TX_ORG_INTERINTRA
        int32_t is_inter = (candidate_buffer->candidate_ptr->type == INTER_MODE ||
                            candidate_buffer->candidate_ptr->use_intrabc)
                               ? EB_TRUE
                               : EB_FALSE;
#endif

        do {
#if TX_ORG_INTERINTRA
            txb_origin_x = context_ptr->blk_geom->tx_org_x[is_inter][tx_depth][txb_itr];
            txb_origin_y = context_ptr->blk_geom->tx_org_y[is_inter][tx_depth][txb_itr];
#else
            txb_origin_x = context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr];
            txb_origin_y = context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr];
#endif
            txb_width  = context_ptr->blk_geom->tx_width[tx_depth][txb_itr];
            txb_height = context_ptr->blk_geom->tx_height[tx_depth][txb_itr];
            txb_ptr    = &blk_ptr->txb_array[txb_index];
#if TX_ORG_INTERINTRA
            rec_luma_offset = txb_origin_x + txb_origin_y * candidate_buffer->recon_ptr->stride_y;
            rec_cb_offset =
                ((((txb_origin_x >> 3) << 3) +
                  ((txb_origin_y >> 3) << 3) * candidate_buffer->recon_ptr->stride_cb) >>
                 1);
            rec_cr_offset =
                ((((txb_origin_x >> 3) << 3) +
                  ((txb_origin_y >> 3) << 3) * candidate_buffer->recon_ptr->stride_cr) >>
                 1);
#else
            rec_luma_offset = context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr] +
                              context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr] *
                                  candidate_buffer->recon_ptr->stride_y;
            rec_cb_offset = ((((context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr] >> 3) << 3) +
                              ((context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr] >> 3) << 3) *
                                  candidate_buffer->recon_ptr->stride_cb) >>
                             1);
            rec_cr_offset = ((((context_ptr->blk_geom->tx_org_x[tx_depth][txb_itr] >> 3) << 3) +
                              ((context_ptr->blk_geom->tx_org_y[tx_depth][txb_itr] >> 3) << 3) *
                                  candidate_buffer->recon_ptr->stride_cr) >>
                             1);
#endif
            txb_origin_index =
                txb_origin_x + txb_origin_y * candidate_buffer->prediction_ptr->stride_y;
            if (txb_ptr->y_has_coeff)
                inv_transform_recon_wrapper(
                    candidate_buffer->prediction_ptr->buffer_y,
                    txb_origin_index,
                    candidate_buffer->prediction_ptr->stride_y,
                    candidate_buffer->recon_ptr->buffer_y,
                    rec_luma_offset,
                    candidate_buffer->recon_ptr->stride_y,
                    (int32_t *)candidate_buffer->recon_coeff_ptr->buffer_y,
                    txb_1d_offset,
                    context_ptr->hbd_mode_decision,
                    context_ptr->blk_geom->txsize[tx_depth][txb_itr],
                    candidate_buffer->candidate_ptr->transform_type[txb_itr],
                    PLANE_TYPE_Y,
                    (uint32_t)candidate_buffer->candidate_ptr->eob[0][txb_itr]);
            else
                picture_copy(candidate_buffer->prediction_ptr,
                             txb_origin_index,
                             0, //txb_chroma_origin_index,
                             candidate_buffer->recon_ptr,
                             rec_luma_offset,
                             0, //txb_chroma_origin_index,
                             txb_width,
                             txb_height,
                             0, //chromaTuSize,
                             0, //chromaTuSize,
                             PICTURE_BUFFER_DESC_Y_FLAG,
                             context_ptr->hbd_mode_decision);

            //CHROMA
            uint8_t tx_depth = candidate_buffer->candidate_ptr->tx_depth;
            if (tx_depth == 0 || txb_itr == 0) {
                if (context_ptr->chroma_level <= CHROMA_MODE_1) {
                    uint32_t chroma_txb_width =
                        tx_size_wide[context_ptr->blk_geom->txsize_uv[tx_depth][txb_itr]];
                    uint32_t chroma_txb_height =
                        tx_size_high[context_ptr->blk_geom->txsize_uv[tx_depth][txb_itr]];
                    uint32_t cb_tu_chroma_origin_index =
                        ((((txb_origin_x >> 3) << 3) +
                          ((txb_origin_y >> 3) << 3) *
                              candidate_buffer->recon_coeff_ptr->stride_cb) >>
                         1);
                    uint32_t cr_tu_chroma_origin_index =
                        ((((txb_origin_x >> 3) << 3) +
                          ((txb_origin_y >> 3) << 3) *
                              candidate_buffer->recon_coeff_ptr->stride_cr) >>
                         1);

                    if (context_ptr->blk_geom->has_uv && txb_ptr->u_has_coeff)
                        inv_transform_recon_wrapper(
                            candidate_buffer->prediction_ptr->buffer_cb,
                            cb_tu_chroma_origin_index,
                            candidate_buffer->prediction_ptr->stride_cb,
                            candidate_buffer->recon_ptr->buffer_cb,
                            rec_cb_offset,
                            candidate_buffer->recon_ptr->stride_cb,
                            (int32_t *)candidate_buffer->recon_coeff_ptr->buffer_cb,
                            txb_1d_offset_uv,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->txsize_uv[tx_depth][txb_itr],
                            candidate_buffer->candidate_ptr->transform_type_uv,
                            PLANE_TYPE_UV,
                            (uint32_t)candidate_buffer->candidate_ptr->eob[1][txb_itr]);
                    else
                        picture_copy(candidate_buffer->prediction_ptr,
                                     0,
                                     cb_tu_chroma_origin_index,
                                     candidate_buffer->recon_ptr,
                                     0,
                                     rec_cb_offset,
                                     0,
                                     0,
                                     chroma_txb_width,
                                     chroma_txb_height,
                                     PICTURE_BUFFER_DESC_Cb_FLAG,
                                     context_ptr->hbd_mode_decision);

                    if (context_ptr->blk_geom->has_uv && txb_ptr->v_has_coeff)
                        inv_transform_recon_wrapper(
                            candidate_buffer->prediction_ptr->buffer_cr,
                            cr_tu_chroma_origin_index,
                            candidate_buffer->prediction_ptr->stride_cr,
                            candidate_buffer->recon_ptr->buffer_cr,
                            rec_cr_offset,
                            candidate_buffer->recon_ptr->stride_cr,
                            (int32_t *)candidate_buffer->recon_coeff_ptr->buffer_cr,
                            txb_1d_offset_uv,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->txsize_uv[tx_depth][txb_itr],
                            candidate_buffer->candidate_ptr->transform_type_uv,
                            PLANE_TYPE_UV,
                            (uint32_t)candidate_buffer->candidate_ptr->eob[2][txb_itr]);
                    else
                        picture_copy(candidate_buffer->prediction_ptr,
                                     0,
                                     cr_tu_chroma_origin_index,
                                     candidate_buffer->recon_ptr,
                                     0,
                                     rec_cr_offset,
                                     0,
                                     0,
                                     chroma_txb_width,
                                     chroma_txb_height,
                                     PICTURE_BUFFER_DESC_Cr_FLAG,
                                     context_ptr->hbd_mode_decision);

                    if (context_ptr->blk_geom->has_uv)
                        txb_1d_offset_uv += context_ptr->blk_geom->tx_width_uv[tx_depth][txb_itr] *
                                            context_ptr->blk_geom->tx_height_uv[tx_depth][txb_itr];
                }
            }
            txb_1d_offset += context_ptr->blk_geom->tx_width[tx_depth][txb_itr] *
                             context_ptr->blk_geom->tx_height[tx_depth][txb_itr];
            ++txb_index;
            ++txb_itr;
        } while (txb_itr < tu_total_count);
    }
}

/*******************************************
* Coding Loop - Fast Loop Initialization
*******************************************/
void product_coding_loop_init_fast_loop(ModeDecisionContext *context_ptr,
                                        NeighborArrayUnit *  skip_coeff_neighbor_array,
                                        NeighborArrayUnit *  inter_pred_dir_neighbor_array,
                                        NeighborArrayUnit *  ref_frame_type_neighbor_array,
                                        NeighborArrayUnit *  intra_luma_mode_neighbor_array,
                                        NeighborArrayUnit *  skip_flag_neighbor_array,
                                        NeighborArrayUnit *  mode_type_neighbor_array,
                                        NeighborArrayUnit *  leaf_depth_neighbor_array,
                                        NeighborArrayUnit *  leaf_partition_neighbor_array) {
    context_ptr->tx_depth = context_ptr->blk_ptr->tx_depth = 0;
    // Generate Split, Skip and intra mode contexts for the rate estimation
    coding_loop_context_generation(context_ptr,
                                   context_ptr->blk_ptr,
                                   context_ptr->blk_origin_x,
                                   context_ptr->blk_origin_y,
                                   BLOCK_SIZE_64,
                                   skip_coeff_neighbor_array,
                                   inter_pred_dir_neighbor_array,
                                   ref_frame_type_neighbor_array,
                                   intra_luma_mode_neighbor_array,
                                   skip_flag_neighbor_array,
                                   mode_type_neighbor_array,
                                   leaf_depth_neighbor_array,
                                   leaf_partition_neighbor_array);
    for (uint32_t index = 0; index < MAX_NFL_BUFF; ++index)
        context_ptr->fast_cost_array[index] = MAX_CU_COST;
    return;
}

void fast_loop_core(ModeDecisionCandidateBuffer *candidate_buffer, PictureControlSet *pcs_ptr,
                    ModeDecisionContext *context_ptr, EbPictureBufferDesc *input_picture_ptr,
                    uint32_t input_origin_index, uint32_t input_cb_origin_in_index,
                    uint32_t input_cr_origin_in_index, BlkStruct *blk_ptr,
                    uint32_t cu_origin_index, uint32_t cu_chroma_origin_index, EbBool use_ssd) {
    uint64_t luma_fast_distortion;
    uint64_t chroma_fast_distortion;

    ModeDecisionCandidate *candidate_ptr  = candidate_buffer->candidate_ptr;
    EbPictureBufferDesc *  prediction_ptr = candidate_buffer->prediction_ptr;
    context_ptr->pu_itr                   = 0;
    // Prediction
    // Set default interp_filters
    candidate_buffer->candidate_ptr->interp_filters =
        (context_ptr->md_staging_use_bilinear) ? av1_make_interp_filters(BILINEAR, BILINEAR) : 0;
    product_prediction_fun_table[candidate_buffer->candidate_ptr->use_intrabc
                                     ? INTER_MODE
                                     : candidate_ptr->type](
        context_ptr->hbd_mode_decision, context_ptr, pcs_ptr, candidate_buffer);

    // Distortion
    // Y
    if (use_ssd) {
        EbSpatialFullDistType spatial_full_dist_type_fun = context_ptr->hbd_mode_decision
                                                               ? full_distortion_kernel16_bits
                                                               : spatial_full_distortion_kernel;

        candidate_buffer->candidate_ptr->luma_fast_distortion = (uint32_t)(
            luma_fast_distortion = spatial_full_dist_type_fun(input_picture_ptr->buffer_y,
                                                              input_origin_index,
                                                              input_picture_ptr->stride_y,
                                                              prediction_ptr->buffer_y,
                                                              cu_origin_index,
                                                              prediction_ptr->stride_y,
                                                              context_ptr->blk_geom->bwidth,
                                                              context_ptr->blk_geom->bheight));
    } else {
        assert((context_ptr->blk_geom->bwidth >> 3) < 17);
        if (!context_ptr->hbd_mode_decision) {
            candidate_buffer->candidate_ptr->luma_fast_distortion =
                (uint32_t)(luma_fast_distortion = nxm_sad_kernel_sub_sampled(
                               input_picture_ptr->buffer_y + input_origin_index,
                               input_picture_ptr->stride_y,
                               prediction_ptr->buffer_y + cu_origin_index,
                               prediction_ptr->stride_y,
                               context_ptr->blk_geom->bheight,
                               context_ptr->blk_geom->bwidth));
        } else {
            candidate_buffer->candidate_ptr->luma_fast_distortion =
                (uint32_t)(luma_fast_distortion = sad_16b_kernel(
                               ((uint16_t *)input_picture_ptr->buffer_y) + input_origin_index,
                               input_picture_ptr->stride_y,
                               ((uint16_t *)prediction_ptr->buffer_y) + cu_origin_index,
                               prediction_ptr->stride_y,
                               context_ptr->blk_geom->bheight,
                               context_ptr->blk_geom->bwidth));
        }
    }

    if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1 &&
        context_ptr->md_staging_skip_inter_chroma_pred == EB_FALSE) {
        if (use_ssd) {
            EbSpatialFullDistType spatial_full_dist_type_fun = context_ptr->hbd_mode_decision
                                                                   ? full_distortion_kernel16_bits
                                                                   : spatial_full_distortion_kernel;

            chroma_fast_distortion =
                spatial_full_dist_type_fun(input_picture_ptr->buffer_cb,
                                           input_cb_origin_in_index,
                                           input_picture_ptr->stride_cb,
                                           candidate_buffer->prediction_ptr->buffer_cb,
                                           cu_chroma_origin_index,
                                           prediction_ptr->stride_cb,
                                           context_ptr->blk_geom->bwidth_uv,
                                           context_ptr->blk_geom->bheight_uv);

            chroma_fast_distortion +=
                spatial_full_dist_type_fun(input_picture_ptr->buffer_cr,
                                           input_cr_origin_in_index,
                                           input_picture_ptr->stride_cb,
                                           candidate_buffer->prediction_ptr->buffer_cr,
                                           cu_chroma_origin_index,
                                           prediction_ptr->stride_cr,
                                           context_ptr->blk_geom->bwidth_uv,
                                           context_ptr->blk_geom->bheight_uv);
        } else {
            assert((context_ptr->blk_geom->bwidth_uv >> 3) < 17);

            if (!context_ptr->hbd_mode_decision) {
                chroma_fast_distortion = nxm_sad_kernel_sub_sampled(
                    input_picture_ptr->buffer_cb + input_cb_origin_in_index,
                    input_picture_ptr->stride_cb,
                    candidate_buffer->prediction_ptr->buffer_cb + cu_chroma_origin_index,
                    prediction_ptr->stride_cb,
                    context_ptr->blk_geom->bheight_uv,
                    context_ptr->blk_geom->bwidth_uv);

                chroma_fast_distortion += nxm_sad_kernel_sub_sampled(
                    input_picture_ptr->buffer_cr + input_cr_origin_in_index,
                    input_picture_ptr->stride_cr,
                    candidate_buffer->prediction_ptr->buffer_cr + cu_chroma_origin_index,
                    prediction_ptr->stride_cr,
                    context_ptr->blk_geom->bheight_uv,
                    context_ptr->blk_geom->bwidth_uv);
            } else {
                chroma_fast_distortion = sad_16b_kernel(
                    ((uint16_t *)input_picture_ptr->buffer_cb) + input_cb_origin_in_index,
                    input_picture_ptr->stride_cb,
                    ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cb) +
                        cu_chroma_origin_index,
                    prediction_ptr->stride_cb,
                    context_ptr->blk_geom->bheight_uv,
                    context_ptr->blk_geom->bwidth_uv);

                chroma_fast_distortion += sad_16b_kernel(
                    ((uint16_t *)input_picture_ptr->buffer_cr) + input_cr_origin_in_index,
                    input_picture_ptr->stride_cr,
                    ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cr) +
                        cu_chroma_origin_index,
                    prediction_ptr->stride_cr,
                    context_ptr->blk_geom->bheight_uv,
                    context_ptr->blk_geom->bwidth_uv);
            }
        }
    } else
        chroma_fast_distortion = 0;
    // Fast Cost
    *(candidate_buffer->fast_cost_ptr) = av1_product_fast_cost_func_table[candidate_ptr->type](
        blk_ptr,
        candidate_buffer->candidate_ptr,
        blk_ptr->qp,
        luma_fast_distortion,
        chroma_fast_distortion,
        use_ssd ? context_ptr->full_lambda : context_ptr->fast_lambda,
        use_ssd,
        pcs_ptr,
        &(context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
              .ed_ref_mv_stack[candidate_ptr->ref_frame_type][0]),
        context_ptr->blk_geom,
        context_ptr->blk_origin_y >> MI_SIZE_LOG2,
        context_ptr->blk_origin_x >> MI_SIZE_LOG2,
        context_ptr->md_enable_inter_intra,
        context_ptr->full_cost_shut_fast_rate_flag,
        1,
        context_ptr->intra_luma_left_mode,
        context_ptr->intra_luma_top_mode);
}
void set_md_stage_counts(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                         uint32_t fastCandidateTotalCount) {
    SequenceControlSet *scs = (SequenceControlSet *)(pcs_ptr->scs_wrapper_ptr->object_ptr);

    // Step 1: derive bypass_stage1 flags
    if (context_ptr->md_staging_mode == MD_STAGING_MODE_1)
        memset(context_ptr->bypass_md_stage_1, EB_FALSE, CAND_CLASS_TOTAL);
    else
        memset(context_ptr->bypass_md_stage_1, EB_TRUE, CAND_CLASS_TOTAL);
    if (context_ptr->md_staging_count_level == 0) {
        // Stage 1 Cand Count
        context_ptr->md_stage_1_count[CAND_CLASS_0] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_1] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_2] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_3] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_4] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_5] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_6] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_7] = 1;
        context_ptr->md_stage_1_count[CAND_CLASS_8] = 1;

        // Stage 2 Cand Count
        context_ptr->md_stage_2_count[CAND_CLASS_0] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_1] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_2] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_3] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_4] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_5] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_6] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_7] = 1;
        context_ptr->md_stage_2_count[CAND_CLASS_8] = 1;
    } else if (context_ptr->md_staging_count_level == 1) {
        uint8_t is_ref   = pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag;
        uint8_t is_base  = (pcs_ptr->temporal_layer_index == 0) ? 1 : 0;
        uint8_t is_intra = (pcs_ptr->slice_type == I_SLICE) ? 1 : 0;

        // Stage 1 Cand Count
        context_ptr->md_stage_1_count[CAND_CLASS_0] =
            is_intra ? fastCandidateTotalCount : is_ref ? 16 : 8;
        context_ptr->md_stage_1_count[CAND_CLASS_1] = is_intra ? 0 : is_ref ? 16 : 8;
        context_ptr->md_stage_1_count[CAND_CLASS_2] = is_intra ? 0 : is_ref ? 16 : 8;
        context_ptr->md_stage_1_count[CAND_CLASS_3] = is_intra ? 0 : is_ref ? 16 : 8;
        context_ptr->md_stage_1_count[CAND_CLASS_4] = is_intra ? 0 : is_ref ? 14 : 6;
        context_ptr->md_stage_1_count[CAND_CLASS_5] = 16;
        context_ptr->md_stage_1_count[CAND_CLASS_6] = is_base ? 10 : 5;
        context_ptr->md_stage_1_count[CAND_CLASS_7] = 14;
        context_ptr->md_stage_1_count[CAND_CLASS_8] = is_base ? 4 : is_ref ? 3 : 2;

        // Stage 2 Cand Count
        context_ptr->md_stage_2_count[CAND_CLASS_0] = is_intra ? 10 : is_ref ? 10 : 4;
        context_ptr->md_stage_2_count[CAND_CLASS_1] = is_intra ? 0 : is_ref ? 6 : 3;
        context_ptr->md_stage_2_count[CAND_CLASS_2] = is_intra ? 0 : is_ref ? 6 : 3;
        context_ptr->md_stage_2_count[CAND_CLASS_3] = is_intra ? 0 : is_ref ? 6 : 3;
        context_ptr->md_stage_2_count[CAND_CLASS_4] = is_intra ? 0 : is_ref ? 12 : 4;
        context_ptr->md_stage_2_count[CAND_CLASS_5] = is_base ? 12 : is_ref ? 8 : 4;
        context_ptr->md_stage_2_count[CAND_CLASS_6] = is_base ? 5 : is_ref ? 3 : 2;
        context_ptr->md_stage_2_count[CAND_CLASS_7] = 7;
        context_ptr->md_stage_2_count[CAND_CLASS_8] = 1;
    } else {
        // Step 2: set md_stage count
        context_ptr->md_stage_1_count[CAND_CLASS_0] =
            (pcs_ptr->slice_type == I_SLICE)
                ? fastCandidateTotalCount
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? INTRA_NFL
                                                                       : (INTRA_NFL >> 1);
        context_ptr->md_stage_1_count[CAND_CLASS_1] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? INTER_NEW_NFL
                                                                       : (INTER_NEW_NFL >> 1);
        context_ptr->md_stage_1_count[CAND_CLASS_2] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? INTER_PRED_NFL
                                                                       : (INTER_PRED_NFL >> 1);
        context_ptr->md_stage_1_count[CAND_CLASS_3] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? INTER_PRED_NFL
                                                                       : (INTER_PRED_NFL >> 1);
        context_ptr->md_stage_1_count[CAND_CLASS_4] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 14 : 6;
        context_ptr->md_stage_1_count[CAND_CLASS_5] = 16;
        context_ptr->md_stage_1_count[CAND_CLASS_6] = (pcs_ptr->temporal_layer_index == 0) ? 10 : 5;
        context_ptr->md_stage_1_count[CAND_CLASS_7] = 12;
        context_ptr->md_stage_1_count[CAND_CLASS_8] = (pcs_ptr->temporal_layer_index == 0) ? 5 : 4;
        if (context_ptr->combine_class12) {
            context_ptr->md_stage_1_count[CAND_CLASS_1] =
                context_ptr->md_stage_1_count[CAND_CLASS_1] * 2;
        }
        if (pcs_ptr->enc_mode >= ENC_M3) {
            context_ptr->md_stage_1_count[CAND_CLASS_1] =
                context_ptr->md_stage_1_count[CAND_CLASS_1] / 2;
            context_ptr->md_stage_1_count[CAND_CLASS_2] =
                context_ptr->md_stage_1_count[CAND_CLASS_2] / 2;
            context_ptr->md_stage_1_count[CAND_CLASS_3] =
                context_ptr->md_stage_1_count[CAND_CLASS_3] / 2;
        }

        context_ptr->md_stage_2_count[CAND_CLASS_0] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 10
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag)
                      ? ((scs->input_resolution >= INPUT_SIZE_1080i_RANGE) ? 7 : 10)
                      : 4;
        context_ptr->md_stage_2_count[CAND_CLASS_6] =
            (pcs_ptr->temporal_layer_index == 0)
                ? 5
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 3 : 2;
        if (pcs_ptr->parent_pcs_ptr->palette_mode == 1)
            context_ptr->md_stage_2_count[CAND_CLASS_7] =
                (pcs_ptr->temporal_layer_index == 0)
                    ? 7
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 4 : 4;
        else if (pcs_ptr->parent_pcs_ptr->palette_mode == 2 ||
                 pcs_ptr->parent_pcs_ptr->palette_mode == 3)
            context_ptr->md_stage_2_count[CAND_CLASS_7] =
                (pcs_ptr->temporal_layer_index == 0)
                    ? 7
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 2;
        else if (pcs_ptr->parent_pcs_ptr->palette_mode == 4 ||
                 pcs_ptr->parent_pcs_ptr->palette_mode == 5)
            context_ptr->md_stage_2_count[CAND_CLASS_7] =
                (pcs_ptr->temporal_layer_index == 0)
                    ? 4
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
        else
            context_ptr->md_stage_2_count[CAND_CLASS_7] =
                (pcs_ptr->temporal_layer_index == 0)
                    ? 2
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 1 : 1;
        context_ptr->md_stage_2_count[CAND_CLASS_8] =
            (pcs_ptr->temporal_layer_index == 0)
                ? 4
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 3 : 2;
        context_ptr->md_stage_2_count[CAND_CLASS_1] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 6 : 3;
        context_ptr->md_stage_2_count[CAND_CLASS_2] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 6 : 3;
        context_ptr->md_stage_2_count[CAND_CLASS_3] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 6 : 3;
        context_ptr->md_stage_2_count[CAND_CLASS_4] =
            (pcs_ptr->slice_type == I_SLICE)
                ? 0
                : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 12 : 4; // 14 : 4;
        if (pcs_ptr->parent_pcs_ptr->pic_obmc_mode == 1)
            context_ptr->md_stage_2_count[CAND_CLASS_5] = 14;
        else if (pcs_ptr->parent_pcs_ptr->pic_obmc_mode <= 3)
            context_ptr->md_stage_2_count[CAND_CLASS_5] =
                (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 12 : 4;
        else
            context_ptr->md_stage_2_count[CAND_CLASS_5] =
                (pcs_ptr->temporal_layer_index == 0)
                    ? 12
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 8 : 4;

        if (context_ptr->combine_class12) {
            context_ptr->md_stage_2_count[CAND_CLASS_1] =
                context_ptr->md_stage_2_count[CAND_CLASS_1] * 2;
        }

        if (!context_ptr->combine_class12 && pcs_ptr->parent_pcs_ptr->sc_content_detected &&
            pcs_ptr->enc_mode == ENC_M0) {
            context_ptr->md_stage_2_count[CAND_CLASS_0] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 10
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 8 : 4;
            context_ptr->md_stage_2_count[CAND_CLASS_1] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 0
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 12 : 6;
            context_ptr->md_stage_2_count[CAND_CLASS_2] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 0
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 12 : 6;
            context_ptr->md_stage_2_count[CAND_CLASS_3] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 0
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 12 : 6;
        }

        if (pcs_ptr->enc_mode >= ENC_M8)
            context_ptr->md_stage_2_count[CAND_CLASS_0] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 10
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 4 : 1;
        if (pcs_ptr->enc_mode >= ENC_M3 && pcs_ptr->enc_mode <= ENC_M4) {
            context_ptr->md_stage_2_count[CAND_CLASS_1] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 0
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 4 : 2;
            context_ptr->md_stage_2_count[CAND_CLASS_2] =
                (pcs_ptr->slice_type == I_SLICE)
                    ? 0
                    : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
            if (!context_ptr->combine_class12) {
                context_ptr->md_stage_2_count[CAND_CLASS_1] =
                    context_ptr->md_stage_2_count[CAND_CLASS_1] / 2;
                context_ptr->md_stage_2_count[CAND_CLASS_3] =
                    (pcs_ptr->slice_type == I_SLICE)
                        ? 0
                        : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
            }
        } else if (pcs_ptr->enc_mode >= ENC_M5) {
            if (pcs_ptr->enc_mode <= ENC_M6) {
                context_ptr->md_stage_1_count[CAND_CLASS_0] =
                    (pcs_ptr->slice_type == I_SLICE)
                        ? 8
                        : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 3 : 1;

                if (context_ptr->combine_class12) {
                    context_ptr->md_stage_1_count[CAND_CLASS_1] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 5 : 3;
                    context_ptr->md_stage_1_count[CAND_CLASS_2] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 1 : 1;

                } else {
                    context_ptr->md_stage_1_count[CAND_CLASS_1] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
                    context_ptr->md_stage_1_count[CAND_CLASS_2] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
                    context_ptr->md_stage_1_count[CAND_CLASS_3] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 1 : 1;
                }

                context_ptr->md_stage_2_count[CAND_CLASS_0] =
                    context_ptr->md_stage_1_count[CAND_CLASS_0];
                context_ptr->md_stage_2_count[CAND_CLASS_1] =
                    context_ptr->md_stage_1_count[CAND_CLASS_1];
                context_ptr->md_stage_2_count[CAND_CLASS_2] =
                    context_ptr->md_stage_1_count[CAND_CLASS_2];
                if (!context_ptr->combine_class12)
                    context_ptr->md_stage_2_count[CAND_CLASS_3] =
                        context_ptr->md_stage_1_count[CAND_CLASS_3];
            } else {
                context_ptr->md_stage_1_count[CAND_CLASS_0] =
                    (pcs_ptr->slice_type == I_SLICE)
                        ? 6
                        : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
                if (context_ptr->combine_class12) {
                    context_ptr->md_stage_1_count[CAND_CLASS_1] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 4 : 2;
                    context_ptr->md_stage_1_count[CAND_CLASS_2] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 1 : 1;

                } else {
                    context_ptr->md_stage_1_count[CAND_CLASS_1] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
                    context_ptr->md_stage_1_count[CAND_CLASS_2] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 2 : 1;
                    context_ptr->md_stage_1_count[CAND_CLASS_3] =
                        (pcs_ptr->slice_type == I_SLICE)
                            ? 0
                            : (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) ? 1 : 1;
                }
                context_ptr->md_stage_2_count[CAND_CLASS_0] =
                    context_ptr->md_stage_1_count[CAND_CLASS_0];
                context_ptr->md_stage_2_count[CAND_CLASS_1] =
                    context_ptr->md_stage_1_count[CAND_CLASS_1];
                context_ptr->md_stage_2_count[CAND_CLASS_2] =
                    context_ptr->md_stage_1_count[CAND_CLASS_2];
                if (!context_ptr->combine_class12)
                    context_ptr->md_stage_2_count[CAND_CLASS_3] =
                        context_ptr->md_stage_1_count[CAND_CLASS_3];
            }
        }
    }
    // Step 3: update count for md_stage_1 and d_stage_2 if bypassed (no NIC setting should be done beyond this point)
    context_ptr->md_stage_2_count[CAND_CLASS_0] = context_ptr->bypass_md_stage_1[CAND_CLASS_0]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_0]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_0];
    context_ptr->md_stage_2_count[CAND_CLASS_1] = context_ptr->bypass_md_stage_1[CAND_CLASS_1]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_1]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_1];
    context_ptr->md_stage_2_count[CAND_CLASS_2] = context_ptr->bypass_md_stage_1[CAND_CLASS_2]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_2]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_2];
    context_ptr->md_stage_2_count[CAND_CLASS_3] = context_ptr->bypass_md_stage_1[CAND_CLASS_3]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_3]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_3];
    context_ptr->md_stage_2_count[CAND_CLASS_4] = context_ptr->bypass_md_stage_1[CAND_CLASS_4]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_4]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_4];
    context_ptr->md_stage_2_count[CAND_CLASS_5] = context_ptr->bypass_md_stage_1[CAND_CLASS_5]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_5]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_5];
    context_ptr->md_stage_2_count[CAND_CLASS_6] = context_ptr->bypass_md_stage_1[CAND_CLASS_6]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_6]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_6];
    context_ptr->md_stage_2_count[CAND_CLASS_8] = context_ptr->bypass_md_stage_1[CAND_CLASS_8]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_8]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_8];

    //TODO: use actual number of stages on the setting section and update using the following logic.
    // stage1_cand_count[CAND_CLASS_i] = bypass_stage1 ? stage2_cand_count[CAND_CLASS_i] : stage1_cand_count[CAND_CLASS_i];
    context_ptr->md_stage_2_count[CAND_CLASS_7] = context_ptr->bypass_md_stage_1[CAND_CLASS_7]
                                                      ? context_ptr->md_stage_1_count[CAND_CLASS_7]
                                                      : context_ptr->md_stage_2_count[CAND_CLASS_7];

    // Step 4: zero-out count for CAND_CLASS_3 if CAND_CLASS_1 and CAND_CLASS_2 are merged (i.e. shift to the left)
    if (context_ptr->combine_class12)
        context_ptr->md_stage_1_count[CAND_CLASS_3] = context_ptr->md_stage_2_count[CAND_CLASS_3] =
            0;
}
void sort_fast_candidates(
    struct ModeDecisionContext *context_ptr, uint32_t input_buffer_start_idx,
    uint32_t
              input_buffer_count, //how many cand buffers to sort. one of the buffer can have max cost.
    uint32_t *cand_buff_indices) {
    ModeDecisionCandidateBuffer **buffer_ptr_array = context_ptr->candidate_buffer_ptr_array;
    uint32_t input_buffer_end_idx = input_buffer_start_idx + input_buffer_count - 1;
    uint32_t buffer_index, i, j;
    uint32_t k = 0;
    for (buffer_index = input_buffer_start_idx; buffer_index <= input_buffer_end_idx;
         buffer_index++, k++) {
        cand_buff_indices[k] = buffer_index;
    }
    for (i = 0; i < input_buffer_count - 1; ++i) {
        for (j = i + 1; j < input_buffer_count; ++j) {
            if (*(buffer_ptr_array[cand_buff_indices[j]]->fast_cost_ptr) <
                *(buffer_ptr_array[cand_buff_indices[i]]->fast_cost_ptr)) {
                buffer_index         = cand_buff_indices[i];
                cand_buff_indices[i] = (uint32_t)cand_buff_indices[j];
                cand_buff_indices[j] = (uint32_t)buffer_index;
            }
        }
    }
}

static INLINE void heap_sort_stage_max_node_fast_cost_ptr(ModeDecisionCandidateBuffer **buffer_ptr,
                                                          uint32_t *sort_index, uint32_t i,
                                                          uint32_t num) {
    uint32_t left, right, max;

    /* Loop for removing recursion. */
    while (1) {
        left  = 2 * i;
        right = 2 * i + 1;
        max   = i;

        if (left <= num && *(buffer_ptr[sort_index[left]]->fast_cost_ptr) >
                               *(buffer_ptr[sort_index[i]]->fast_cost_ptr)) {
            max = left;
        }

        if (right <= num && *(buffer_ptr[sort_index[right]]->fast_cost_ptr) >
                                *(buffer_ptr[sort_index[max]]->fast_cost_ptr)) {
            max = right;
        }

        if (max == i) { break; }

        uint32_t swap   = sort_index[i];
        sort_index[i]   = sort_index[max];
        sort_index[max] = swap;
        i               = max;
    }
}

static void qsort_stage_max_node_fast_cost_ptr(ModeDecisionCandidateBuffer **buffer_ptr_array,
                                               uint32_t *dst, uint32_t *a, uint32_t *b, int num) {
    if (num < 4) {
        if (num < 2) {
            if (num) {
                //num = 1
                dst[0] = a[0];
            }
            return;
        }
        if (num > 2) {
            //num = 3
            uint32_t tmp_a = a[0];
            uint32_t tmp_b = a[1];
            uint32_t tmp_c = a[2];
            uint64_t val_a = *(buffer_ptr_array[tmp_a]->fast_cost_ptr);
            uint64_t val_b = *(buffer_ptr_array[tmp_b]->fast_cost_ptr);
            uint64_t val_c = *(buffer_ptr_array[tmp_c]->fast_cost_ptr);

            if (val_a < val_b) {
                if (val_b < val_c) {
                    //Sorted abc
                    dst[0] = tmp_a;
                    dst[1] = tmp_b;
                    dst[2] = tmp_c;
                } else {
                    //xcx
                    if (val_a < val_c) {
                        //Sorted 132
                        dst[0] = tmp_a;
                        dst[1] = tmp_c;
                        dst[2] = tmp_b;
                    } else {
                        //Sorted 231
                        dst[0] = tmp_c;
                        dst[1] = tmp_a;
                        dst[2] = tmp_b;
                    }
                }
            } else {
                //a>b
                if (val_b > val_c) {
                    //Sorted cba
                    dst[0] = tmp_c;
                    dst[1] = tmp_b;
                    dst[2] = tmp_a;
                } else {
                    //bxx
                    if (val_a < val_c) {
                        //Sorted bac
                        dst[0] = tmp_b;
                        dst[1] = tmp_a;
                        dst[2] = tmp_c;
                    } else {
                        //Sorted bca
                        dst[0] = tmp_b;
                        dst[1] = tmp_c;
                        dst[2] = tmp_a;
                    }
                }
            }
            return;
        }

        /* bacuse a and dst can point on this same array, copy temporary values*/
        uint32_t tmp_a = a[0];
        uint32_t tmp_b = a[1];
        if (*(buffer_ptr_array[tmp_a]->fast_cost_ptr) < *(buffer_ptr_array[tmp_b]->fast_cost_ptr)) {
            dst[0] = tmp_a;
            dst[1] = tmp_b;
        } else {
            dst[0] = tmp_b;
            dst[1] = tmp_a;
        }
        return;
    }

    int sorted_down = 0;
    int sorted_up   = num - 1;

    uint64_t pivot_val = *(buffer_ptr_array[a[0]]->fast_cost_ptr);
    for (int i = 1; i < num; ++i) {
        if (pivot_val < *(buffer_ptr_array[a[i]]->fast_cost_ptr)) {
            b[sorted_up] = a[i];
            sorted_up--;
        } else {
            b[sorted_down] = a[i];
            sorted_down++;
        }
    }

    dst[sorted_down] = a[0];

    qsort_stage_max_node_fast_cost_ptr(buffer_ptr_array, dst, b, a, sorted_down);

    qsort_stage_max_node_fast_cost_ptr(buffer_ptr_array,
                                       dst + (sorted_down + 1),
                                       b + (sorted_down + 1),
                                       a + (sorted_down + 1),
                                       num - (sorted_down)-1);
}

static INLINE void sort_array_index_fast_cost_ptr(ModeDecisionCandidateBuffer **buffer_ptr,
                                                  uint32_t *sort_index, uint32_t num) {
    if (num <= 60) {
        //For small array uses 'quick sort', work much faster for small array,
        //but required alloc temporary memory.
        uint32_t sorted_tmp[60];
        qsort_stage_max_node_fast_cost_ptr(buffer_ptr, sort_index, sort_index, sorted_tmp, num);
        return;
    }

    //For big arrays uses 'heap sort', not need allocate memory
    //For small array less that 40 elements heap sort work slower than 'insertion sort'
    uint32_t i;
    for (i = (num - 1) / 2; i > 0; i--) {
        heap_sort_stage_max_node_fast_cost_ptr(buffer_ptr, sort_index, i, num - 1);
    }

    heap_sort_stage_max_node_fast_cost_ptr(buffer_ptr, sort_index, 0, num - 1);

    for (i = num - 1; i > 0; i--) {
        uint32_t swap = sort_index[i];
        sort_index[i] = sort_index[0];
        sort_index[0] = swap;
        heap_sort_stage_max_node_fast_cost_ptr(buffer_ptr, sort_index, 0, i - 1);
    }
}
void sort_stage1_candidates(struct ModeDecisionContext *context_ptr, uint32_t num_of_cand_to_sort,
                            uint32_t *cand_buff_indices) {
    uint32_t                      i, j, index;
    ModeDecisionCandidateBuffer **buffer_ptr_array = context_ptr->candidate_buffer_ptr_array;
    for (i = 0; i < num_of_cand_to_sort - 1; ++i) {
        for (j = i + 1; j < num_of_cand_to_sort; ++j) {
            if (*(buffer_ptr_array[cand_buff_indices[j]]->full_cost_ptr) <
                *(buffer_ptr_array[cand_buff_indices[i]]->full_cost_ptr)) {
                index                = cand_buff_indices[i];
                cand_buff_indices[i] = (uint32_t)cand_buff_indices[j];
                cand_buff_indices[j] = (uint32_t)index;
            }
        }
    }
}
void construct_best_sorted_arrays_md_stage_1(struct ModeDecisionContext *  context_ptr,
                                             ModeDecisionCandidateBuffer **buffer_ptr_array,
                                             uint32_t *best_candidate_index_array,
                                             uint32_t *sorted_candidate_index_array,
                                             uint64_t *ref_fast_cost) {
    //best = union from all classes
    uint32_t best_candi = 0;
    for (CandClass class_i = CAND_CLASS_0; class_i < CAND_CLASS_TOTAL; class_i++)
        for (uint32_t candi = 0; candi < context_ptr->md_stage_1_count[class_i]; candi++)
            sorted_candidate_index_array[best_candi++] =
                context_ptr->cand_buff_indices[class_i][candi];

    assert(best_candi == context_ptr->md_stage_1_total_count);
    uint32_t full_recon_candidate_count = context_ptr->md_stage_1_total_count;

    //sort best: inter, then intra
    uint32_t i, id;
    uint32_t id_inter = 0;
    uint32_t id_intra = full_recon_candidate_count - 1;
    for (i = 0; i < full_recon_candidate_count; ++i) {
        id = sorted_candidate_index_array[i];
        if (buffer_ptr_array[id]->candidate_ptr->type == INTER_MODE) {
            best_candidate_index_array[id_inter++] = id;
        } else {
            assert(buffer_ptr_array[id]->candidate_ptr->type == INTRA_MODE);
            best_candidate_index_array[id_intra--] = id;
        }
    }

    //sorted best: *(buffer_ptr_array[sorted_candidate_index_array[?]]->fast_cost_ptr)
    sort_array_index_fast_cost_ptr(
        buffer_ptr_array, sorted_candidate_index_array, full_recon_candidate_count);

    // tx search
    *ref_fast_cost = *(buffer_ptr_array[sorted_candidate_index_array[0]]->fast_cost_ptr);
}

void construct_best_sorted_arrays_md_stage_2(struct ModeDecisionContext *  context_ptr,
                                             ModeDecisionCandidateBuffer **buffer_ptr_array,
                                             uint32_t *best_candidate_index_array,
                                             uint32_t *sorted_candidate_index_array) {
    //best = union from all classes
    uint32_t best_candi = 0;
    for (CandClass class_i = CAND_CLASS_0; class_i < CAND_CLASS_TOTAL; class_i++)
        for (uint32_t candi = 0; candi < context_ptr->md_stage_2_count[class_i]; candi++)
            sorted_candidate_index_array[best_candi++] =
                context_ptr->cand_buff_indices[class_i][candi];

    assert(best_candi == context_ptr->md_stage_2_total_count);
    uint32_t full_recon_candidate_count = context_ptr->md_stage_2_total_count;
    //sort best: inter, then intra
    uint32_t i, id;
    uint32_t id_inter = 0;
    uint32_t id_intra = full_recon_candidate_count - 1;
    for (i = 0; i < full_recon_candidate_count; ++i) {
        id = sorted_candidate_index_array[i];
        if (buffer_ptr_array[id]->candidate_ptr->type == INTER_MODE) {
            best_candidate_index_array[id_inter++] = id;
        } else {
            assert(buffer_ptr_array[id]->candidate_ptr->type == INTRA_MODE);
            best_candidate_index_array[id_intra--] = id;
        }
    }

    //sorted best: *(buffer_ptr_array[sorted_candidate_index_array[?]]->fast_cost_ptr)
    sort_array_index_fast_cost_ptr(
        buffer_ptr_array, sorted_candidate_index_array, full_recon_candidate_count);
}

void md_stage_0(

    PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base,
    ModeDecisionCandidate *fast_candidate_array, int32_t fast_candidate_start_index,
    int32_t fast_candidate_end_index, EbPictureBufferDesc *input_picture_ptr,
    uint32_t input_origin_index, uint32_t input_cb_origin_in_index,
    uint32_t input_cr_origin_in_index, BlkStruct *blk_ptr, uint32_t blk_origin_index,
    uint32_t blk_chroma_origin_index, uint32_t candidate_buffer_start_index, uint32_t max_buffers,
    EbBool scratch_buffer_pesent_flag) {
    int32_t  fast_loop_cand_index;
    uint64_t luma_fast_distortion;
    uint32_t highest_cost_index;
    uint64_t highest_cost;
    uint64_t best_first_fast_cost_search_candidate_cost  = MAX_CU_COST;
    int32_t  best_first_fast_cost_search_candidate_index = INVALID_FAST_CANDIDATE_INDEX;
    EbBool   use_ssd = EB_FALSE;
    // Set MD Staging fast_loop_core settings
    context_ptr->md_staging_skip_interpolation_search =
        (context_ptr->md_staging_mode == MD_STAGING_MODE_1)
            ? EB_TRUE
            : context_ptr->interpolation_search_level >= IT_SEARCH_FAST_LOOP_UV_BLIND ? EB_FALSE
                                                                                      : EB_TRUE;

    context_ptr->md_staging_skip_inter_chroma_pred =
        (context_ptr->md_staging_mode == MD_STAGING_MODE_1 &&
         context_ptr->target_class != CAND_CLASS_0 && context_ptr->target_class != CAND_CLASS_6 &&
         context_ptr->target_class != CAND_CLASS_7)
            ? EB_TRUE
            : EB_FALSE;

    context_ptr->md_staging_use_bilinear =
        (context_ptr->md_staging_mode == MD_STAGING_MODE_1) ? EB_TRUE : EB_FALSE;
    // 1st fast loop: src-to-src
    fast_loop_cand_index = fast_candidate_end_index;
    while (fast_loop_cand_index >= fast_candidate_start_index) {
        if (fast_candidate_array[fast_loop_cand_index].cand_class == context_ptr->target_class) {
            // Set the Candidate Buffer
            ModeDecisionCandidateBuffer *candidate_buffer =
                candidate_buffer_ptr_array_base[candidate_buffer_start_index];
            ModeDecisionCandidate *candidate_ptr = candidate_buffer->candidate_ptr =
                &fast_candidate_array[fast_loop_cand_index];
            // Initialize tx_depth
            candidate_buffer->candidate_ptr->tx_depth = 0;
            // Only check (src - src) candidates (Tier0 candidates)
            if (candidate_ptr->distortion_ready) {
                // Distortion
                luma_fast_distortion = candidate_ptr->me_distortion;

                // Fast Cost
                *(candidate_buffer->fast_cost_ptr) =
                    av1_product_fast_cost_func_table[candidate_ptr->type](
                        blk_ptr,
                        candidate_buffer->candidate_ptr,
                        blk_ptr->qp,
                        luma_fast_distortion,
                        0,
                        context_ptr->fast_lambda,
                        0,
                        pcs_ptr,
                        &(context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                              .ed_ref_mv_stack[candidate_ptr->ref_frame_type][0]),
                        context_ptr->blk_geom,
                        context_ptr->blk_origin_y >> MI_SIZE_LOG2,
                        context_ptr->blk_origin_x >> MI_SIZE_LOG2,
                        context_ptr->md_enable_inter_intra,
                        context_ptr->full_cost_shut_fast_rate_flag,
                        1,
                        context_ptr->intra_luma_left_mode,
                        context_ptr->intra_luma_top_mode);

                // Keep track of the candidate index of the best  (src - src) candidate
                if (*(candidate_buffer->fast_cost_ptr) <=
                    best_first_fast_cost_search_candidate_cost) {
                    best_first_fast_cost_search_candidate_index = fast_loop_cand_index;
                    best_first_fast_cost_search_candidate_cost = *(candidate_buffer->fast_cost_ptr);
                }

                // Initialize Fast Cost - to do not interact with the second Fast-Cost Search
                *(candidate_buffer->fast_cost_ptr) = MAX_CU_COST;
            }
        }
        --fast_loop_cand_index;
    }

    // 2nd fast loop: src-to-recon
    highest_cost_index   = candidate_buffer_start_index;
    fast_loop_cand_index = fast_candidate_end_index;
    while (fast_loop_cand_index >= fast_candidate_start_index) {
        if (fast_candidate_array[fast_loop_cand_index].cand_class == context_ptr->target_class) {
            ModeDecisionCandidateBuffer *candidate_buffer =
                candidate_buffer_ptr_array_base[highest_cost_index];
            ModeDecisionCandidate *candidate_ptr = candidate_buffer->candidate_ptr =
                &fast_candidate_array[fast_loop_cand_index];
            // Initialize tx_depth
            candidate_buffer->candidate_ptr->tx_depth = 0;
            if (!candidate_ptr->distortion_ready ||
                fast_loop_cand_index == best_first_fast_cost_search_candidate_index) {
                // Prediction
                fast_loop_core(candidate_buffer,
                               pcs_ptr,
                               context_ptr,
                               input_picture_ptr,
                               input_origin_index,
                               input_cb_origin_in_index,
                               input_cr_origin_in_index,
                               blk_ptr,
                               blk_origin_index,
                               blk_chroma_origin_index,
                               use_ssd);
            }

            // Find the buffer with the highest cost
            if (fast_loop_cand_index || scratch_buffer_pesent_flag) {
                // max_cost is volatile to prevent the compiler from loading 0xFFFFFFFFFFFFFF
                //   as a const at the early-out. Loading a large constant on intel x64 processors
                //   clogs the i-cache/intstruction decode. This still reloads the variable from
                //   the stack each pass, so a better solution would be to register the variable,
                //   but this might require asm.
                volatile uint64_t max_cost           = MAX_CU_COST;
                const uint64_t *  fast_cost_array    = context_ptr->fast_cost_array;
                const uint32_t    buffer_index_start = candidate_buffer_start_index;
                const uint32_t    buffer_index_end   = buffer_index_start + max_buffers;
                uint32_t          buffer_index;

                highest_cost_index = buffer_index_start;
                buffer_index       = buffer_index_start + 1;

                do {
                    highest_cost = fast_cost_array[highest_cost_index];
                    if (highest_cost == max_cost) break;

                    if (fast_cost_array[buffer_index] > highest_cost)
                        highest_cost_index = buffer_index;
                } while (++buffer_index < buffer_index_end);
            }
        }
        --fast_loop_cand_index;
    }

    // Set the cost of the scratch canidate to max to get discarded @ the sorting phase
    *(candidate_buffer_ptr_array_base[highest_cost_index]->fast_cost_ptr) =
        (scratch_buffer_pesent_flag)
            ? MAX_CU_COST
            : *(candidate_buffer_ptr_array_base[highest_cost_index]->fast_cost_ptr);
}
void predictive_me_full_pel_search(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                   EbPictureBufferDesc *input_picture_ptr,
                                   uint32_t input_origin_index, EbBool use_ssd, uint8_t list_idx,
                                   int8_t ref_idx, int16_t mvx, int16_t mvy,
                                   int16_t search_position_start_x, int16_t search_position_end_x,
                                   int16_t search_position_start_y, int16_t search_position_end_y,
                                   int16_t search_step, int16_t *best_mvx, int16_t *best_mvy,
                                   uint32_t *best_distortion) {
    uint8_t hbd_mode_decision = context_ptr->hbd_mode_decision == EB_DUAL_BIT_MD
                                    ? EB_8_BIT_MD
                                    : context_ptr->hbd_mode_decision;
    uint32_t                     distortion;
    ModeDecisionCandidateBuffer *candidate_buffer =
        &(context_ptr->candidate_buffer_ptr_array[0][0]);
    candidate_buffer->candidate_ptr = &(context_ptr->fast_candidate_array[0]);

    EbReferenceObject *  ref_obj = pcs_ptr->ref_pic_ptr_array[list_idx][ref_idx]->object_ptr;
    EbPictureBufferDesc *ref_pic =
        hbd_mode_decision ? ref_obj->reference_picture16bit : ref_obj->reference_picture;
    for (int32_t refinement_pos_x = search_position_start_x;
         refinement_pos_x <= search_position_end_x;
         ++refinement_pos_x) {
        for (int32_t refinement_pos_y = search_position_start_y;
             refinement_pos_y <= search_position_end_y;
             ++refinement_pos_y) {
            uint32_t ref_origin_index =
                ref_pic->origin_x + (context_ptr->blk_origin_x + (mvx >> 3) + refinement_pos_x) +
                (context_ptr->blk_origin_y + (mvy >> 3) + ref_pic->origin_y + refinement_pos_y) *
                    ref_pic->stride_y;
            if (use_ssd) {
                EbSpatialFullDistType spatial_full_dist_type_fun =
                    hbd_mode_decision ? full_distortion_kernel16_bits
                                      : spatial_full_distortion_kernel;

                distortion = (uint32_t)spatial_full_dist_type_fun(input_picture_ptr->buffer_y,
                                                                  input_origin_index,
                                                                  input_picture_ptr->stride_y,
                                                                  ref_pic->buffer_y,
                                                                  ref_origin_index,
                                                                  ref_pic->stride_y,
                                                                  context_ptr->blk_geom->bwidth,
                                                                  context_ptr->blk_geom->bheight);
            } else {
                assert((context_ptr->blk_geom->bwidth >> 3) < 17);

                if (hbd_mode_decision) {
                    distortion = sad_16b_kernel(
                        ((uint16_t *)input_picture_ptr->buffer_y) + input_origin_index,
                        input_picture_ptr->stride_y,
                        ((uint16_t *)ref_pic->buffer_y) + ref_origin_index,
                        ref_pic->stride_y,
                        context_ptr->blk_geom->bheight,
                        context_ptr->blk_geom->bwidth);
                } else {
                    distortion =
                        nxm_sad_kernel_sub_sampled(input_picture_ptr->buffer_y + input_origin_index,
                                                   input_picture_ptr->stride_y,
                                                   ref_pic->buffer_y + ref_origin_index,
                                                   ref_pic->stride_y,
                                                   context_ptr->blk_geom->bheight,
                                                   context_ptr->blk_geom->bwidth);
                }
            }

            if (distortion < *best_distortion) {
                *best_mvx        = mvx + (refinement_pos_x * search_step);
                *best_mvy        = mvy + (refinement_pos_y * search_step);
                *best_distortion = distortion;
            }
        }
    }
}

void predictive_me_sub_pel_search(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                  EbPictureBufferDesc *input_picture_ptr,
                                  uint32_t input_origin_index, uint32_t blk_origin_index,
                                  EbBool use_ssd, uint8_t list_idx, int8_t ref_idx, int16_t mvx,
                                  int16_t mvy, int16_t search_position_start_x,
                                  int16_t search_position_end_x, int16_t search_position_start_y,
                                  int16_t search_position_end_y, int16_t search_step,
                                  int16_t *best_mvx, int16_t *best_mvy, uint32_t *best_distortion,
                                  uint8_t search_pattern) {
    uint8_t hbd_mode_decision = context_ptr->hbd_mode_decision == EB_DUAL_BIT_MD
                                    ? EB_8_BIT_MD
                                    : context_ptr->hbd_mode_decision;
    uint32_t                     distortion;
    ModeDecisionCandidateBuffer *candidate_buffer =
        &(context_ptr->candidate_buffer_ptr_array[0][0]);
    candidate_buffer->candidate_ptr = &(context_ptr->fast_candidate_array[0]);

    for (int32_t refinement_pos_x = search_position_start_x;
         refinement_pos_x <= search_position_end_x;
         ++refinement_pos_x) {
        for (int32_t refinement_pos_y = search_position_start_y;
             refinement_pos_y <= search_position_end_y;
             ++refinement_pos_y) {
            if (refinement_pos_x == 0 && refinement_pos_y == 0) continue;

            if (search_pattern == 1 && refinement_pos_x != 0 && refinement_pos_y != 0) continue;

            if (search_pattern == 2 && refinement_pos_y != 0) continue;

            if (search_pattern == 3 && refinement_pos_x != 0) continue;

            ModeDecisionCandidate *candidate_ptr  = candidate_buffer->candidate_ptr;
            EbPictureBufferDesc *  prediction_ptr = candidate_buffer->prediction_ptr;

            candidate_ptr->type                         = INTER_MODE;
            candidate_ptr->distortion_ready             = 0;
            candidate_ptr->use_intrabc                  = 0;
            candidate_ptr->merge_flag                   = EB_FALSE;
            candidate_ptr->prediction_direction[0]      = (EbPredDirection)list_idx;
            candidate_ptr->inter_mode                   = NEWMV;
            candidate_ptr->pred_mode                    = NEWMV;
            candidate_ptr->motion_mode                  = SIMPLE_TRANSLATION;
            candidate_ptr->is_interintra_used           = 0;
            candidate_ptr->is_compound                  = 0;
            candidate_ptr->is_new_mv                    = 1;
            candidate_ptr->is_zero_mv                   = 0;
            candidate_ptr->drl_index                    = 0;
            candidate_ptr->ref_mv_index                 = 0;
            candidate_ptr->pred_mv_weight               = 0;
            candidate_ptr->ref_frame_type               = svt_get_ref_frame_type(list_idx, ref_idx);
            candidate_ptr->transform_type[PLANE_TYPE_Y] = DCT_DCT;
            candidate_ptr->transform_type[PLANE_TYPE_UV] = DCT_DCT;
            candidate_ptr->motion_vector_xl0 =
                list_idx == 0 ? mvx + (refinement_pos_x * search_step) : 0;
            candidate_ptr->motion_vector_yl0 =
                list_idx == 0 ? mvy + (refinement_pos_y * search_step) : 0;
            candidate_ptr->motion_vector_xl1 =
                list_idx == 1 ? mvx + (refinement_pos_x * search_step) : 0;
            candidate_ptr->motion_vector_yl1 =
                list_idx == 1 ? mvy + (refinement_pos_y * search_step) : 0;
            candidate_ptr->ref_frame_index_l0 = list_idx == 0 ? ref_idx : -1;
            candidate_ptr->ref_frame_index_l1 = list_idx == 1 ? ref_idx : -1;
            candidate_ptr->interp_filters     = 0;

            // Prediction
            context_ptr->md_staging_skip_interpolation_search = EB_TRUE;
            context_ptr->md_staging_skip_inter_chroma_pred    = EB_TRUE;
            product_prediction_fun_table[INTER_MODE](
                hbd_mode_decision, context_ptr, pcs_ptr, candidate_buffer);

            // Distortion
            if (use_ssd) {
                EbSpatialFullDistType spatial_full_dist_type_fun =
                    hbd_mode_decision ? full_distortion_kernel16_bits
                                      : spatial_full_distortion_kernel;

                distortion = (uint32_t)spatial_full_dist_type_fun(input_picture_ptr->buffer_y,
                                                                  input_origin_index,
                                                                  input_picture_ptr->stride_y,
                                                                  prediction_ptr->buffer_y,
                                                                  blk_origin_index,
                                                                  prediction_ptr->stride_y,
                                                                  context_ptr->blk_geom->bwidth,
                                                                  context_ptr->blk_geom->bheight);
            } else {
                assert((context_ptr->blk_geom->bwidth >> 3) < 17);

                if (hbd_mode_decision) {
                    distortion = sad_16b_kernel(
                        ((uint16_t *)input_picture_ptr->buffer_y) + input_origin_index,
                        input_picture_ptr->stride_y,
                        ((uint16_t *)prediction_ptr->buffer_y) + blk_origin_index,
                        prediction_ptr->stride_y,
                        context_ptr->blk_geom->bheight,
                        context_ptr->blk_geom->bwidth);
                } else {
                    distortion =
                        nxm_sad_kernel_sub_sampled(input_picture_ptr->buffer_y + input_origin_index,
                                                   input_picture_ptr->stride_y,
                                                   prediction_ptr->buffer_y + blk_origin_index,
                                                   prediction_ptr->stride_y,
                                                   context_ptr->blk_geom->bheight,
                                                   context_ptr->blk_geom->bwidth);
                }
            }
            if (distortion < *best_distortion) {
                *best_mvx        = mvx + (refinement_pos_x * search_step);
                *best_mvy        = mvy + (refinement_pos_y * search_step);
                *best_distortion = distortion;
            }
        }
    }
}

void    av1_set_ref_frame(MvReferenceFrame *rf, int8_t ref_frame_type);
uint8_t get_max_drl_index(uint8_t refmvCnt, PredictionMode mode);
void    predictive_me_search(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                             EbPictureBufferDesc *input_picture_ptr, uint32_t input_origin_index,
                             uint32_t blk_origin_index) {
    const SequenceControlSet *scs_ptr = (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;
    EbBool                    use_ssd           = EB_TRUE;
    uint8_t                   hbd_mode_decision = context_ptr->hbd_mode_decision == EB_DUAL_BIT_MD
                                    ? EB_8_BIT_MD
                                    : context_ptr->hbd_mode_decision;
    input_picture_ptr = hbd_mode_decision ? pcs_ptr->input_frame16bit
                                          : pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    //Update input origin
    input_origin_index =
        (context_ptr->blk_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y +
        (context_ptr->blk_origin_x + input_picture_ptr->origin_x);
    // Reset valid_refined_mv
    memset(context_ptr->valid_refined_mv, 0, 8); // [2][4]

    for (uint32_t ref_it = 0; ref_it < pcs_ptr->parent_pcs_ptr->tot_ref_frame_types; ++ref_it) {
        MvReferenceFrame ref_pair = pcs_ptr->parent_pcs_ptr->ref_frame_type_arr[ref_it];

        MacroBlockD *xd = context_ptr->blk_ptr->av1xd;
        uint8_t      drli, max_drl_index;
        IntMv        nearestmv[2], nearmv[2], ref_mv[2];

        MvReferenceFrame rf[2];
        av1_set_ref_frame(rf, ref_pair);

        // Reset search variable(s)
        uint32_t best_mvp_distortion = (int32_t)~0;
        uint32_t mvp_distortion;

        int16_t  best_search_mvx        = (int16_t)~0;
        int16_t  best_search_mvy        = (int16_t)~0;
        uint32_t best_search_distortion = (int32_t)~0;

        // Step 0: derive the MVP list; 1 nearest and up to 3 near
        int16_t mvp_x_array[PREDICTIVE_ME_MAX_MVP_CANIDATES];
        int16_t mvp_y_array[PREDICTIVE_ME_MAX_MVP_CANIDATES];
        int8_t  mvp_count = 0;
        if (rf[1] == NONE_FRAME) {
            MvReferenceFrame frame_type = rf[0];
            uint8_t          list_idx   = get_list_idx(rf[0]);
            uint8_t          ref_idx    = get_ref_frame_idx(rf[0]);
            if (ref_idx > context_ptr->md_max_ref_count - 1) continue;
            // Get the ME MV
            const MeSbResults *me_results =
                pcs_ptr->parent_pcs_ptr->me_results[context_ptr->me_sb_addr];
            int16_t me_mv_x;
            int16_t me_mv_y;
            if (list_idx == 0) {
                me_mv_x = (me_results->me_mv_array[context_ptr->me_block_offset][ref_idx].x_mv)
                          << 1;
                me_mv_y = (me_results->me_mv_array[context_ptr->me_block_offset][ref_idx].y_mv)
                          << 1;
            } else {
                me_mv_x = (me_results
                               ->me_mv_array[context_ptr->me_block_offset]
                                            [((scs_ptr->mrp_mode == 0) ? 4 : 2) + ref_idx]
                               .x_mv)
                          << 1;
                me_mv_y = (me_results
                               ->me_mv_array[context_ptr->me_block_offset]
                                            [((scs_ptr->mrp_mode == 0) ? 4 : 2) + ref_idx]
                               .y_mv)
                          << 1;
            }
            // Round-up to the closest integer the ME MV
            me_mv_x = (me_mv_x + 4) & ~0x07;
            me_mv_y = (me_mv_y + 4) & ~0x07;

            uint32_t           pa_me_distortion;
            EbReferenceObject *ref_obj = pcs_ptr->ref_pic_ptr_array[list_idx][ref_idx]->object_ptr;
            EbPictureBufferDesc *ref_pic =
                hbd_mode_decision ? ref_obj->reference_picture16bit : ref_obj->reference_picture;

            uint32_t ref_origin_index =
                ref_pic->origin_x + (context_ptr->blk_origin_x + (me_mv_x >> 3)) +
                (context_ptr->blk_origin_y + (me_mv_y >> 3) + ref_pic->origin_y) *
                    ref_pic->stride_y;
            if (use_ssd) {
                EbSpatialFullDistType spatial_full_dist_type_fun =
                    hbd_mode_decision ? full_distortion_kernel16_bits
                                      : spatial_full_distortion_kernel;

                pa_me_distortion =
                    (uint32_t)spatial_full_dist_type_fun(input_picture_ptr->buffer_y,
                                                         input_origin_index,
                                                         input_picture_ptr->stride_y,
                                                         ref_pic->buffer_y,
                                                         ref_origin_index,
                                                         ref_pic->stride_y,
                                                         context_ptr->blk_geom->bwidth,
                                                         context_ptr->blk_geom->bheight);
            } else {
                assert((context_ptr->blk_geom->bwidth >> 3) < 17);

                if (hbd_mode_decision) {
                    pa_me_distortion = sad_16b_kernel(
                        ((uint16_t *)input_picture_ptr->buffer_y) + input_origin_index,
                        input_picture_ptr->stride_y,
                        ((uint16_t *)ref_pic->buffer_y) + ref_origin_index,
                        ref_pic->stride_y,
                        context_ptr->blk_geom->bheight,
                        context_ptr->blk_geom->bwidth);
                } else {
                    pa_me_distortion =
                        nxm_sad_kernel_sub_sampled(input_picture_ptr->buffer_y + input_origin_index,
                                                   input_picture_ptr->stride_y,
                                                   ref_pic->buffer_y + ref_origin_index,
                                                   ref_pic->stride_y,
                                                   context_ptr->blk_geom->bheight,
                                                   context_ptr->blk_geom->bwidth);
                }
            }

            if (pa_me_distortion != 0 || context_ptr->predictive_me_level >= 5) {
                //NEAREST
                mvp_x_array[mvp_count] =
                    (context_ptr->blk_ptr->ref_mvs[frame_type][0].as_mv.col + 4) & ~0x07;
                mvp_y_array[mvp_count] =
                    (context_ptr->blk_ptr->ref_mvs[frame_type][0].as_mv.row + 4) & ~0x07;

                mvp_count++;

                //NEAR
                max_drl_index = get_max_drl_index(xd->ref_mv_count[frame_type], NEARMV);

                for (drli = 0; drli < max_drl_index; drli++) {
                    get_av1_mv_pred_drl(context_ptr,
                                        context_ptr->blk_ptr,
                                        frame_type,
                                        0,
                                        NEARMV,
                                        drli,
                                        nearestmv,
                                        nearmv,
                                        ref_mv);

                    if (((nearmv[0].as_mv.col + 4) & ~0x07) != mvp_x_array[0] &&
                        ((nearmv[0].as_mv.row + 4) & ~0x07) != mvp_y_array[0]) {
                        mvp_x_array[mvp_count] = (nearmv[0].as_mv.col + 4) & ~0x07;
                        mvp_y_array[mvp_count] = (nearmv[0].as_mv.row + 4) & ~0x07;
                        mvp_count++;
                    }
                }
                // Step 1: derive the best MVP in term of distortion
                int16_t best_mvp_x = 0;
                int16_t best_mvp_y = 0;

                for (int8_t mvp_index = 0; mvp_index < mvp_count; mvp_index++) {
                    // MVP Distortion
                    EbReferenceObject *ref_obj =
                        pcs_ptr->ref_pic_ptr_array[list_idx][ref_idx]->object_ptr;
                    EbPictureBufferDesc *ref_pic = hbd_mode_decision
                                                       ? ref_obj->reference_picture16bit
                                                       : ref_obj->reference_picture;

                    uint32_t ref_origin_index =
                        ref_pic->origin_x +
                        (context_ptr->blk_origin_x + (mvp_x_array[mvp_index] >> 3)) +
                        (context_ptr->blk_origin_y + (mvp_y_array[mvp_index] >> 3) +
                         ref_pic->origin_y) *
                            ref_pic->stride_y;
                    if (use_ssd) {
                        EbSpatialFullDistType spatial_full_dist_type_fun =
                            hbd_mode_decision ? full_distortion_kernel16_bits
                                              : spatial_full_distortion_kernel;

                        mvp_distortion =
                            (uint32_t)spatial_full_dist_type_fun(input_picture_ptr->buffer_y,
                                                                 input_origin_index,
                                                                 input_picture_ptr->stride_y,
                                                                 ref_pic->buffer_y,
                                                                 ref_origin_index,
                                                                 ref_pic->stride_y,
                                                                 context_ptr->blk_geom->bwidth,
                                                                 context_ptr->blk_geom->bheight);
                    } else {
                        assert((context_ptr->blk_geom->bwidth >> 3) < 17);

                        if (hbd_mode_decision) {
                            mvp_distortion = sad_16b_kernel(
                                ((uint16_t *)input_picture_ptr->buffer_y) + input_origin_index,
                                input_picture_ptr->stride_y,
                                ((uint16_t *)ref_pic->buffer_y) + ref_origin_index,
                                ref_pic->stride_y,
                                context_ptr->blk_geom->bheight,
                                context_ptr->blk_geom->bwidth);
                        } else {
                            mvp_distortion = nxm_sad_kernel_sub_sampled(
                                input_picture_ptr->buffer_y + input_origin_index,
                                input_picture_ptr->stride_y,
                                ref_pic->buffer_y + ref_origin_index,
                                ref_pic->stride_y,
                                context_ptr->blk_geom->bheight,
                                context_ptr->blk_geom->bwidth);
                        }
                    }

                    if (mvp_distortion < best_mvp_distortion) {
                        best_mvp_distortion = mvp_distortion;
                        best_mvp_x          = mvp_x_array[mvp_index];
                        best_mvp_y          = mvp_y_array[mvp_index];
                    }
                }

                // Step 2: perform full pel search around the best MVP
                best_mvp_x = (best_mvp_x + 4) & ~0x07;
                best_mvp_y = (best_mvp_y + 4) & ~0x07;

                predictive_me_full_pel_search(pcs_ptr,
                                              context_ptr,
                                              input_picture_ptr,
                                              input_origin_index,
                                              use_ssd,
                                              list_idx,
                                              ref_idx,
                                              best_mvp_x,
                                              best_mvp_y,
                                              -(context_ptr->full_pel_ref_window_width_th >> 1),
                                              +(context_ptr->full_pel_ref_window_width_th >> 1),
                                              -(context_ptr->full_pel_ref_window_height_th >> 1),
                                              +(context_ptr->full_pel_ref_window_height_th >> 1),
                                              8,
                                              &best_search_mvx,
                                              &best_search_mvy,
                                              &best_search_distortion);

                EbBool exit_predictive_me_sub_pel;

                if (pa_me_distortion == 0)
                    exit_predictive_me_sub_pel = EB_TRUE;
                else if (best_search_distortion <= pa_me_distortion)
                    exit_predictive_me_sub_pel = EB_FALSE;
                else {
                    exit_predictive_me_sub_pel =
                        ((((best_search_distortion - pa_me_distortion) * 100) / pa_me_distortion) <
                         PREDICTIVE_ME_DEVIATION_TH)
                            ? EB_FALSE
                            : EB_TRUE;
                }

                if (exit_predictive_me_sub_pel == EB_FALSE ||
                    context_ptr->predictive_me_level >= 5) {
                    if (context_ptr->predictive_me_level >= 2) {
                        uint8_t search_pattern;
                        // 0: all possible position(s): horizontal, vertical, diagonal
                        // 1: horizontal, vertical
                        // 2: horizontal only
                        // 3: vertical only

                        // Step 3: perform half pel search around the best full pel position
                        search_pattern = (context_ptr->predictive_me_level >= 4) ? 0 : 1;

                        predictive_me_sub_pel_search(pcs_ptr,
                                                     context_ptr,
                                                     input_picture_ptr,
                                                     input_origin_index,
                                                     blk_origin_index,
                                                     use_ssd,
                                                     list_idx,
                                                     ref_idx,
                                                     best_search_mvx,
                                                     best_search_mvy,
                                                     -(HALF_PEL_REF_WINDOW >> 1),
                                                     +(HALF_PEL_REF_WINDOW >> 1),
                                                     -(HALF_PEL_REF_WINDOW >> 1),
                                                     +(HALF_PEL_REF_WINDOW >> 1),
                                                     4,
                                                     &best_search_mvx,
                                                     &best_search_mvy,
                                                     &best_search_distortion,
                                                     search_pattern);

                        if (context_ptr->predictive_me_level == 3) {
                            if ((best_search_mvx & 0x07) != 0 || (best_search_mvy & 0x07) != 0) {
                                if ((best_search_mvx & 0x07) == 0)
                                    search_pattern = 2;
                                else // if(best_search_mvy & 0x07 == 0)
                                    search_pattern = 3;

                                predictive_me_sub_pel_search(pcs_ptr,
                                                             context_ptr,
                                                             input_picture_ptr,
                                                             input_origin_index,
                                                             blk_origin_index,
                                                             use_ssd,
                                                             list_idx,
                                                             ref_idx,
                                                             best_search_mvx,
                                                             best_search_mvy,
                                                             -(HALF_PEL_REF_WINDOW >> 1),
                                                             +(HALF_PEL_REF_WINDOW >> 1),
                                                             -(HALF_PEL_REF_WINDOW >> 1),
                                                             +(HALF_PEL_REF_WINDOW >> 1),
                                                             4,
                                                             &best_search_mvx,
                                                             &best_search_mvy,
                                                             &best_search_distortion,
                                                             search_pattern);
                            }
                        }

                        // Step 4: perform quarter pel search around the best half pel position
                        search_pattern = (context_ptr->predictive_me_level >= 4) ? 0 : 1;
                        predictive_me_sub_pel_search(pcs_ptr,
                                                     context_ptr,
                                                     input_picture_ptr,
                                                     input_origin_index,
                                                     blk_origin_index,
                                                     use_ssd,
                                                     list_idx,
                                                     ref_idx,
                                                     best_search_mvx,
                                                     best_search_mvy,
                                                     -(QUARTER_PEL_REF_WINDOW >> 1),
                                                     +(QUARTER_PEL_REF_WINDOW >> 1),
                                                     -(QUARTER_PEL_REF_WINDOW >> 1),
                                                     +(QUARTER_PEL_REF_WINDOW >> 1),
                                                     2,
                                                     &best_search_mvx,
                                                     &best_search_mvy,
                                                     &best_search_distortion,
                                                     search_pattern);

                        if (context_ptr->predictive_me_level == 3) {
                            if ((best_search_mvx & 0x03) != 0 || (best_search_mvy & 0x03) != 0) {
                                if ((best_search_mvx & 0x03) == 0)
                                    search_pattern = 2;
                                else // if(best_search_mvy & 0x03 == 0)
                                    search_pattern = 3;

                                predictive_me_sub_pel_search(pcs_ptr,
                                                             context_ptr,
                                                             input_picture_ptr,
                                                             input_origin_index,
                                                             blk_origin_index,
                                                             use_ssd,
                                                             list_idx,
                                                             ref_idx,
                                                             best_search_mvx,
                                                             best_search_mvy,
                                                             -(QUARTER_PEL_REF_WINDOW >> 1),
                                                             +(QUARTER_PEL_REF_WINDOW >> 1),
                                                             -(QUARTER_PEL_REF_WINDOW >> 1),
                                                             +(QUARTER_PEL_REF_WINDOW >> 1),
                                                             2,
                                                             &best_search_mvx,
                                                             &best_search_mvy,
                                                             &best_search_distortion,
                                                             search_pattern);
                            }
                        }
                    }
                    // Step 5: perform eigh pel search around the best quarter pel position
                    if (pcs_ptr->parent_pcs_ptr->frm_hdr.allow_high_precision_mv) {
                        uint8_t search_pattern = 0;
                        predictive_me_sub_pel_search(pcs_ptr,
                                                     context_ptr,
                                                     input_picture_ptr,
                                                     input_origin_index,
                                                     blk_origin_index,
                                                     use_ssd,
                                                     list_idx,
                                                     ref_idx,
                                                     best_search_mvx,
                                                     best_search_mvy,
                                                     -(QUARTER_PEL_REF_WINDOW >> 1),
                                                     +(QUARTER_PEL_REF_WINDOW >> 1),
                                                     -(QUARTER_PEL_REF_WINDOW >> 1),
                                                     +(QUARTER_PEL_REF_WINDOW >> 1),
                                                     1,
                                                     &best_search_mvx,
                                                     &best_search_mvy,
                                                     &best_search_distortion,
                                                     search_pattern);
                    }
                    context_ptr->best_spatial_pred_mv[list_idx][ref_idx][0] = best_search_mvx;
                    context_ptr->best_spatial_pred_mv[list_idx][ref_idx][1] = best_search_mvy;
                    context_ptr->valid_refined_mv[list_idx][ref_idx]        = 1;
                }
            }
        }
    }
}
void av1_cost_calc_cfl(PictureControlSet *pcs_ptr, ModeDecisionCandidateBuffer *candidate_buffer,
                       SuperBlock *sb_ptr, ModeDecisionContext *context_ptr,
                       uint32_t component_mask, EbPictureBufferDesc *input_picture_ptr,
                       uint32_t input_cb_origin_in_index, uint32_t blk_chroma_origin_index,
                       uint64_t full_distortion[DIST_CALC_TOTAL], uint64_t *coeff_bits,
                       EbBool check_dc) {
    ModeDecisionCandidate *candidate_ptr = candidate_buffer->candidate_ptr;
    uint32_t               count_non_zero_coeffs[3][MAX_NUM_OF_TU_PER_CU];
    uint64_t               cb_full_distortion[DIST_CALC_TOTAL];
    uint64_t               cr_full_distortion[DIST_CALC_TOTAL];
    uint64_t               cb_coeff_bits = 0;
    uint64_t               cr_coeff_bits = 0;
    uint32_t               chroma_width  = context_ptr->blk_geom->bwidth_uv;
    uint32_t               chroma_height = context_ptr->blk_geom->bheight_uv;
    // FullLoop and TU search
    int32_t  alpha_q3;
    uint16_t cb_qp = context_ptr->qp;
    uint16_t cr_qp = context_ptr->qp;

    full_distortion[DIST_CALC_RESIDUAL]   = 0;
    full_distortion[DIST_CALC_PREDICTION] = 0;
    *coeff_bits                           = 0;

    // Loop over alphas and find the best
    if (component_mask == COMPONENT_CHROMA_CB || component_mask == COMPONENT_CHROMA ||
        component_mask == COMPONENT_ALL) {
        cb_full_distortion[DIST_CALC_RESIDUAL]   = 0;
        cr_full_distortion[DIST_CALC_RESIDUAL]   = 0;
        cb_full_distortion[DIST_CALC_PREDICTION] = 0;
        cr_full_distortion[DIST_CALC_PREDICTION] = 0;
        cb_coeff_bits                            = 0;
        cr_coeff_bits                            = 0;
        alpha_q3                                 = (check_dc) ? 0
                              : cfl_idx_to_alpha(candidate_ptr->cfl_alpha_idx,
                                                 candidate_ptr->cfl_alpha_signs,
                                                 CFL_PRED_U); // once for U, once for V
        assert(chroma_width * CFL_BUF_LINE + chroma_height <= CFL_BUF_SQUARE);

        if (!context_ptr->hbd_mode_decision) {
            eb_cfl_predict_lbd(
                context_ptr->pred_buf_q3,
                &(candidate_buffer->prediction_ptr->buffer_cb[blk_chroma_origin_index]),
                candidate_buffer->prediction_ptr->stride_cb,
                &(candidate_buffer->cfl_temp_prediction_ptr->buffer_cb[blk_chroma_origin_index]),
                candidate_buffer->cfl_temp_prediction_ptr->stride_cb,
                alpha_q3,
                8,
                chroma_width,
                chroma_height);
        } else {
            eb_cfl_predict_hbd(
                context_ptr->pred_buf_q3,
                ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cb) + blk_chroma_origin_index,
                candidate_buffer->prediction_ptr->stride_cb,
                ((uint16_t *)candidate_buffer->cfl_temp_prediction_ptr->buffer_cb) +
                    blk_chroma_origin_index,
                candidate_buffer->cfl_temp_prediction_ptr->stride_cb,
                alpha_q3,
                10,
                chroma_width,
                chroma_height);
        }

        // Cb Residual
        residual_kernel(input_picture_ptr->buffer_cb,
                        input_cb_origin_in_index,
                        input_picture_ptr->stride_cb,
                        candidate_buffer->cfl_temp_prediction_ptr->buffer_cb,
                        blk_chroma_origin_index,
                        candidate_buffer->cfl_temp_prediction_ptr->stride_cb,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_cb,
                        blk_chroma_origin_index,
                        candidate_buffer->residual_ptr->stride_cb,
                        context_ptr->hbd_mode_decision,
                        chroma_width,
                        chroma_height);

        full_loop_r(sb_ptr,
                    candidate_buffer,
                    context_ptr,
                    input_picture_ptr,
                    pcs_ptr,
                    PICTURE_BUFFER_DESC_Cb_FLAG,
                    cb_qp,
                    cr_qp,
                    &(*count_non_zero_coeffs[1]),
                    &(*count_non_zero_coeffs[2]));

        // Create new function
        cu_full_distortion_fast_txb_mode_r(sb_ptr,
                                           candidate_buffer,
                                           context_ptr,
                                           candidate_ptr,
                                           pcs_ptr,
                                           input_picture_ptr,
                                           cb_full_distortion,
                                           cr_full_distortion,
                                           count_non_zero_coeffs,
                                           COMPONENT_CHROMA_CB,
                                           &cb_coeff_bits,
                                           &cr_coeff_bits,
                                           0);

        full_distortion[DIST_CALC_RESIDUAL] += cb_full_distortion[DIST_CALC_RESIDUAL];
        full_distortion[DIST_CALC_PREDICTION] += cb_full_distortion[DIST_CALC_PREDICTION];
        *coeff_bits += cb_coeff_bits;
    }
    if (component_mask == COMPONENT_CHROMA_CR || component_mask == COMPONENT_CHROMA ||
        component_mask == COMPONENT_ALL) {
        cb_full_distortion[DIST_CALC_RESIDUAL]   = 0;
        cr_full_distortion[DIST_CALC_RESIDUAL]   = 0;
        cb_full_distortion[DIST_CALC_PREDICTION] = 0;
        cr_full_distortion[DIST_CALC_PREDICTION] = 0;

        cb_coeff_bits = 0;
        cr_coeff_bits = 0;
        alpha_q3      = (check_dc) ? 0
                              : cfl_idx_to_alpha(candidate_ptr->cfl_alpha_idx,
                                                 candidate_ptr->cfl_alpha_signs,
                                                 CFL_PRED_V); // once for U, once for V
        assert(chroma_width * CFL_BUF_LINE + chroma_height <= CFL_BUF_SQUARE);

        if (!context_ptr->hbd_mode_decision) {
            eb_cfl_predict_lbd(
                context_ptr->pred_buf_q3,
                &(candidate_buffer->prediction_ptr->buffer_cr[blk_chroma_origin_index]),
                candidate_buffer->prediction_ptr->stride_cr,
                &(candidate_buffer->cfl_temp_prediction_ptr->buffer_cr[blk_chroma_origin_index]),
                candidate_buffer->cfl_temp_prediction_ptr->stride_cr,
                alpha_q3,
                8,
                chroma_width,
                chroma_height);
        } else {
            eb_cfl_predict_hbd(
                context_ptr->pred_buf_q3,
                ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cr) + blk_chroma_origin_index,
                candidate_buffer->prediction_ptr->stride_cr,
                ((uint16_t *)candidate_buffer->cfl_temp_prediction_ptr->buffer_cr) +
                    blk_chroma_origin_index,
                candidate_buffer->cfl_temp_prediction_ptr->stride_cr,
                alpha_q3,
                10,
                chroma_width,
                chroma_height);
        }

        // Cr Residual
        residual_kernel(input_picture_ptr->buffer_cr,
                        input_cb_origin_in_index,
                        input_picture_ptr->stride_cr,
                        candidate_buffer->cfl_temp_prediction_ptr->buffer_cr,
                        blk_chroma_origin_index,
                        candidate_buffer->cfl_temp_prediction_ptr->stride_cr,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_cr,
                        blk_chroma_origin_index,
                        candidate_buffer->residual_ptr->stride_cr,
                        context_ptr->hbd_mode_decision,
                        chroma_width,
                        chroma_height);

        full_loop_r(sb_ptr,
                    candidate_buffer,
                    context_ptr,
                    input_picture_ptr,
                    pcs_ptr,
                    PICTURE_BUFFER_DESC_Cr_FLAG,
                    cb_qp,
                    cr_qp,
                    &(*count_non_zero_coeffs[1]),
                    &(*count_non_zero_coeffs[2]));
        candidate_ptr->v_has_coeff = *count_non_zero_coeffs[2] ? EB_TRUE : EB_FALSE;

        // Create new function
        cu_full_distortion_fast_txb_mode_r(sb_ptr,
                                           candidate_buffer,
                                           context_ptr,
                                           candidate_ptr,
                                           pcs_ptr,
                                           input_picture_ptr,
                                           cb_full_distortion,
                                           cr_full_distortion,
                                           count_non_zero_coeffs,
                                           COMPONENT_CHROMA_CR,
                                           &cb_coeff_bits,
                                           &cr_coeff_bits,
                                           0);

        full_distortion[DIST_CALC_RESIDUAL] += cr_full_distortion[DIST_CALC_RESIDUAL];
        full_distortion[DIST_CALC_PREDICTION] += cr_full_distortion[DIST_CALC_PREDICTION];
        *coeff_bits += cr_coeff_bits;
    }
}

#define PLANE_SIGN_TO_JOINT_SIGN(plane, a, b) \
    (plane == CFL_PRED_U ? a * CFL_SIGNS + b - 1 : b * CFL_SIGNS + a - 1)
/*************************Pick the best alpha for cfl mode  or Choose DC******************************************************/
void cfl_rd_pick_alpha(PictureControlSet *pcs_ptr, ModeDecisionCandidateBuffer *candidate_buffer,
                       SuperBlock *sb_ptr, ModeDecisionContext *context_ptr,
                       EbPictureBufferDesc *input_picture_ptr, uint32_t input_cb_origin_in_index,
                       uint32_t blk_chroma_origin_index) {
    int64_t  best_rd = INT64_MAX;
    uint64_t full_distortion[DIST_CALC_TOTAL];
    uint64_t coeff_bits;

    const int64_t mode_rd = RDCOST(
        context_ptr->full_lambda,
        (uint64_t)candidate_buffer->candidate_ptr->md_rate_estimation_ptr
            ->intra_uv_mode_fac_bits[CFL_ALLOWED][candidate_buffer->candidate_ptr->intra_luma_mode]
                                    [UV_CFL_PRED],
        0);

    int64_t best_rd_uv[CFL_JOINT_SIGNS][CFL_PRED_PLANES];
    int32_t best_c[CFL_JOINT_SIGNS][CFL_PRED_PLANES];

    for (int32_t plane = 0; plane < CFL_PRED_PLANES; plane++) {
        coeff_bits                          = 0;
        full_distortion[DIST_CALC_RESIDUAL] = 0;
        for (int32_t joint_sign = 0; joint_sign < CFL_JOINT_SIGNS; joint_sign++) {
            best_rd_uv[joint_sign][plane] = INT64_MAX;
            best_c[joint_sign][plane]     = 0;
        }
        // Collect RD stats for an alpha value of zero in this plane.
        // Skip i == CFL_SIGN_ZERO as (0, 0) is invalid.
        for (int32_t i = CFL_SIGN_NEG; i < CFL_SIGNS; i++) {
            const int32_t joint_sign = PLANE_SIGN_TO_JOINT_SIGN(plane, CFL_SIGN_ZERO, i);
            if (i == CFL_SIGN_NEG) {
                candidate_buffer->candidate_ptr->cfl_alpha_idx   = 0;
                candidate_buffer->candidate_ptr->cfl_alpha_signs = joint_sign;

                av1_cost_calc_cfl(pcs_ptr,
                                  candidate_buffer,
                                  sb_ptr,
                                  context_ptr,
                                  (plane == 0) ? COMPONENT_CHROMA_CB : COMPONENT_CHROMA_CR,
                                  input_picture_ptr,
                                  input_cb_origin_in_index,
                                  blk_chroma_origin_index,
                                  full_distortion,
                                  &coeff_bits,
                                  0);

                if (coeff_bits == INT64_MAX) break;
            }

            const int32_t alpha_rate = candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                                           ->cfl_alpha_fac_bits[joint_sign][plane][0];

            best_rd_uv[joint_sign][plane] = RDCOST(context_ptr->full_lambda,
                                                   coeff_bits + alpha_rate,
                                                   full_distortion[DIST_CALC_RESIDUAL]);
        }
    }

    int32_t best_joint_sign = -1;

    for (int32_t plane = 0; plane < CFL_PRED_PLANES; plane++) {
        for (int32_t pn_sign = CFL_SIGN_NEG; pn_sign < CFL_SIGNS; pn_sign++) {
            int32_t progress = 0;
            for (int32_t c = 0; c < CFL_ALPHABET_SIZE; c++) {
                int32_t flag = 0;
                if (c > 2 && progress < c) break;
                coeff_bits                          = 0;
                full_distortion[DIST_CALC_RESIDUAL] = 0;
                for (int32_t i = 0; i < CFL_SIGNS; i++) {
                    const int32_t joint_sign = PLANE_SIGN_TO_JOINT_SIGN(plane, pn_sign, i);
                    if (i == 0) {
                        candidate_buffer->candidate_ptr->cfl_alpha_idx =
                            (c << CFL_ALPHABET_SIZE_LOG2) + c;
                        candidate_buffer->candidate_ptr->cfl_alpha_signs = joint_sign;

                        av1_cost_calc_cfl(pcs_ptr,
                                          candidate_buffer,
                                          sb_ptr,
                                          context_ptr,
                                          (plane == 0) ? COMPONENT_CHROMA_CB : COMPONENT_CHROMA_CR,
                                          input_picture_ptr,
                                          input_cb_origin_in_index,
                                          blk_chroma_origin_index,
                                          full_distortion,
                                          &coeff_bits,
                                          0);

                        if (coeff_bits == INT64_MAX) break;
                    }

                    const int32_t alpha_rate =
                        candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                            ->cfl_alpha_fac_bits[joint_sign][plane][c];

                    int64_t this_rd = RDCOST(context_ptr->full_lambda,
                                             coeff_bits + alpha_rate,
                                             full_distortion[DIST_CALC_RESIDUAL]);
                    if (this_rd >= best_rd_uv[joint_sign][plane]) continue;
                    best_rd_uv[joint_sign][plane] = this_rd;
                    best_c[joint_sign][plane]     = c;

                    flag = 2;
                    if (best_rd_uv[joint_sign][!plane] == INT64_MAX) continue;
                    this_rd += mode_rd + best_rd_uv[joint_sign][!plane];
                    if (this_rd >= best_rd) continue;
                    best_rd         = this_rd;
                    best_joint_sign = joint_sign;
                }
                progress += flag;
            }
        }
    }

    // Compare with DC Chroma
    coeff_bits                          = 0;
    full_distortion[DIST_CALC_RESIDUAL] = 0;

    candidate_buffer->candidate_ptr->cfl_alpha_idx   = 0;
    candidate_buffer->candidate_ptr->cfl_alpha_signs = 0;

    const int64_t dc_mode_rd = RDCOST(
        context_ptr->full_lambda,
        candidate_buffer->candidate_ptr->md_rate_estimation_ptr
            ->intra_uv_mode_fac_bits[CFL_ALLOWED][candidate_buffer->candidate_ptr->intra_luma_mode]
                                    [UV_DC_PRED],
        0);

    av1_cost_calc_cfl(pcs_ptr,
                      candidate_buffer,
                      sb_ptr,
                      context_ptr,
                      COMPONENT_CHROMA,
                      input_picture_ptr,
                      input_cb_origin_in_index,
                      blk_chroma_origin_index,
                      full_distortion,
                      &coeff_bits,
                      1);

    int64_t dc_rd =
        RDCOST(context_ptr->full_lambda, coeff_bits, full_distortion[DIST_CALC_RESIDUAL]);

    dc_rd += dc_mode_rd;
    if (dc_rd <= best_rd) {
        candidate_buffer->candidate_ptr->intra_chroma_mode = UV_DC_PRED;
        candidate_buffer->candidate_ptr->cfl_alpha_idx     = 0;
        candidate_buffer->candidate_ptr->cfl_alpha_signs   = 0;
    } else {
        candidate_buffer->candidate_ptr->intra_chroma_mode = UV_CFL_PRED;
        int32_t ind                                        = 0;
        if (best_joint_sign >= 0) {
            const int32_t u = best_c[best_joint_sign][CFL_PRED_U];
            const int32_t v = best_c[best_joint_sign][CFL_PRED_V];
            ind             = (u << CFL_ALPHABET_SIZE_LOG2) + v;
        } else
            best_joint_sign = 0;
        candidate_buffer->candidate_ptr->cfl_alpha_idx   = ind;
        candidate_buffer->candidate_ptr->cfl_alpha_signs = best_joint_sign;
    }
}

// If mode is CFL:
// 1: recon the Luma
// 2: Form the pred_buf_q3
// 3: Loop over alphas and find the best or choose DC
// 4: Recalculate the residual for chroma
static void cfl_prediction(PictureControlSet *          pcs_ptr,
                           ModeDecisionCandidateBuffer *candidate_buffer, SuperBlock *sb_ptr,
                           ModeDecisionContext *context_ptr, EbPictureBufferDesc *input_picture_ptr,
                           uint32_t input_cb_origin_in_index, uint32_t blk_chroma_origin_index) {
    if (context_ptr->blk_geom->has_uv) {
        // 1: recon the Luma
        av1_perform_inverse_transform_recon_luma(pcs_ptr, context_ptr, candidate_buffer);

        uint32_t rec_luma_offset =
            ((context_ptr->blk_geom->origin_y >> 3) << 3) * candidate_buffer->recon_ptr->stride_y +
            ((context_ptr->blk_geom->origin_x >> 3) << 3);
        // 2: Form the pred_buf_q3
        uint32_t chroma_width  = context_ptr->blk_geom->bwidth_uv;
        uint32_t chroma_height = context_ptr->blk_geom->bheight_uv;

        // Down sample Luma
        if (!context_ptr->hbd_mode_decision) {
            cfl_luma_subsampling_420_lbd_c(
                &(context_ptr->cfl_temp_luma_recon[rec_luma_offset]),
                candidate_buffer->recon_ptr->stride_y,
                context_ptr->pred_buf_q3,
                context_ptr->blk_geom->bwidth_uv == context_ptr->blk_geom->bwidth
                    ? (context_ptr->blk_geom->bwidth_uv << 1)
                    : context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight_uv == context_ptr->blk_geom->bheight
                    ? (context_ptr->blk_geom->bheight_uv << 1)
                    : context_ptr->blk_geom->bheight);
        } else {
            cfl_luma_subsampling_420_hbd_c(
                context_ptr->cfl_temp_luma_recon16bit + rec_luma_offset,
                candidate_buffer->recon_ptr->stride_y,
                context_ptr->pred_buf_q3,
                context_ptr->blk_geom->bwidth_uv == context_ptr->blk_geom->bwidth
                    ? (context_ptr->blk_geom->bwidth_uv << 1)
                    : context_ptr->blk_geom->bwidth,
                context_ptr->blk_geom->bheight_uv == context_ptr->blk_geom->bheight
                    ? (context_ptr->blk_geom->bheight_uv << 1)
                    : context_ptr->blk_geom->bheight);
        }
        int32_t round_offset = chroma_width * chroma_height / 2;

        eb_subtract_average(context_ptr->pred_buf_q3,
                            chroma_width,
                            chroma_height,
                            round_offset,
                            LOG2F(chroma_width) + LOG2F(chroma_height));

        // 3: Loop over alphas and find the best or choose DC
        cfl_rd_pick_alpha(pcs_ptr,
                          candidate_buffer,
                          sb_ptr,
                          context_ptr,
                          input_picture_ptr,
                          input_cb_origin_in_index,
                          blk_chroma_origin_index);

        if (candidate_buffer->candidate_ptr->intra_chroma_mode == UV_CFL_PRED) {
            // 4: Recalculate the prediction and the residual
            int32_t alpha_q3_cb = cfl_idx_to_alpha(candidate_buffer->candidate_ptr->cfl_alpha_idx,
                                                   candidate_buffer->candidate_ptr->cfl_alpha_signs,
                                                   CFL_PRED_U);
            int32_t alpha_q3_cr = cfl_idx_to_alpha(candidate_buffer->candidate_ptr->cfl_alpha_idx,
                                                   candidate_buffer->candidate_ptr->cfl_alpha_signs,
                                                   CFL_PRED_V);

            assert(chroma_height * CFL_BUF_LINE + chroma_width <= CFL_BUF_SQUARE);

            if (!context_ptr->hbd_mode_decision) {
                eb_cfl_predict_lbd(
                    context_ptr->pred_buf_q3,
                    &(candidate_buffer->prediction_ptr->buffer_cb[blk_chroma_origin_index]),
                    candidate_buffer->prediction_ptr->stride_cb,
                    &(candidate_buffer->prediction_ptr->buffer_cb[blk_chroma_origin_index]),
                    candidate_buffer->prediction_ptr->stride_cb,
                    alpha_q3_cb,
                    8,
                    chroma_width,
                    chroma_height);

                eb_cfl_predict_lbd(
                    context_ptr->pred_buf_q3,
                    &(candidate_buffer->prediction_ptr->buffer_cr[blk_chroma_origin_index]),
                    candidate_buffer->prediction_ptr->stride_cr,
                    &(candidate_buffer->prediction_ptr->buffer_cr[blk_chroma_origin_index]),
                    candidate_buffer->prediction_ptr->stride_cr,
                    alpha_q3_cr,
                    8,
                    chroma_width,
                    chroma_height);
            } else {
                eb_cfl_predict_hbd(context_ptr->pred_buf_q3,
                                   ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cb) +
                                       blk_chroma_origin_index,
                                   candidate_buffer->prediction_ptr->stride_cb,
                                   ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cb) +
                                       blk_chroma_origin_index,
                                   candidate_buffer->prediction_ptr->stride_cb,
                                   alpha_q3_cb,
                                   10,
                                   chroma_width,
                                   chroma_height);

                eb_cfl_predict_hbd(context_ptr->pred_buf_q3,
                                   ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cr) +
                                       blk_chroma_origin_index,
                                   candidate_buffer->prediction_ptr->stride_cr,
                                   ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cr) +
                                       blk_chroma_origin_index,
                                   candidate_buffer->prediction_ptr->stride_cr,
                                   alpha_q3_cr,
                                   10,
                                   chroma_width,
                                   chroma_height);
            }

            // Cb Residual
            residual_kernel(input_picture_ptr->buffer_cb,
                            input_cb_origin_in_index,
                            input_picture_ptr->stride_cb,
                            candidate_buffer->prediction_ptr->buffer_cb,
                            blk_chroma_origin_index,
                            candidate_buffer->prediction_ptr->stride_cb,
                            (int16_t *)candidate_buffer->residual_ptr->buffer_cb,
                            blk_chroma_origin_index,
                            candidate_buffer->residual_ptr->stride_cb,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv);

            // Cr Residual
            residual_kernel(input_picture_ptr->buffer_cr,
                            input_cb_origin_in_index,
                            input_picture_ptr->stride_cr,
                            candidate_buffer->prediction_ptr->buffer_cr,
                            blk_chroma_origin_index,
                            candidate_buffer->prediction_ptr->stride_cr,
                            (int16_t *)candidate_buffer->residual_ptr->buffer_cr,
                            blk_chroma_origin_index,
                            candidate_buffer->residual_ptr->stride_cr,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv);
        } else {
            // Alphas = 0, Preds are the same as DC. Switch to DC mode
            candidate_buffer->candidate_ptr->intra_chroma_mode = UV_DC_PRED;
        }
    }
}
uint8_t get_skip_tx_search_flag(int32_t sq_size, uint64_t ref_fast_cost, uint64_t cu_cost,
                                uint64_t weight) {
    //NM: Skip tx search when the fast cost of the current mode candidate is substansially
    // Larger than the best fast_cost (
    uint8_t tx_search_skip_flag = cu_cost >= ((ref_fast_cost * weight) / 100) ? 1 : 0;
    tx_search_skip_flag         = sq_size >= 128 ? 1 : tx_search_skip_flag;
    return tx_search_skip_flag;
}

static INLINE TxType av1_get_tx_type(BlockSize sb_type, int32_t is_inter, PredictionMode pred_mode,
                                     UvPredictionMode pred_mode_uv, PlaneType plane_type,
                                     const MacroBlockD *xd, int32_t blk_row, int32_t blk_col,
                                     TxSize tx_size, int32_t reduced_tx_set) {
    UNUSED(sb_type);
    UNUSED(*xd);
    UNUSED(blk_row);
    UNUSED(blk_col);

    // block_size  sb_type = BLOCK_8X8;

    MbModeInfo mbmi;
    mbmi.block_mi.mode    = pred_mode;
    mbmi.block_mi.uv_mode = pred_mode_uv;

    // const MbModeInfo *const mbmi = xd->mi[0];
    // const struct MacroblockdPlane *const pd = &xd->plane[plane_type];
    const TxSetType tx_set_type =
        /*av1_*/ get_ext_tx_set_type(tx_size, is_inter, reduced_tx_set);

    TxType tx_type = DCT_DCT;
    if (/*xd->lossless[mbmi->segment_id] ||*/ txsize_sqr_up_map[tx_size] > TX_32X32)
        tx_type = DCT_DCT;
    else {
        if (plane_type == PLANE_TYPE_Y) {
            //const int32_t txk_type_idx =
            //    av1_get_txk_type_index(/*mbmi->*/sb_type, blk_row, blk_col);
            //tx_type = mbmi->txk_type[txk_type_idx];
        } else if (is_inter /*is_inter_block(mbmi)*/) {
            // scale back to y plane's coordinate
            //blk_row <<= pd->subsampling_y;
            //blk_col <<= pd->subsampling_x;
            //const int32_t txk_type_idx =
            //    av1_get_txk_type_index(mbmi->sb_type, blk_row, blk_col);
            //tx_type = mbmi->txk_type[txk_type_idx];
        } else {
            // In intra mode, uv planes don't share the same prediction mode as y
            // plane, so the tx_type should not be shared
            tx_type = intra_mode_to_tx_type(&mbmi.block_mi, PLANE_TYPE_UV);
        }
    }
    ASSERT(tx_type < TX_TYPES);
    if (!av1_ext_tx_used[tx_set_type][tx_type]) return DCT_DCT;
    return tx_type;
}

void check_best_indepedant_cfl(PictureControlSet *pcs_ptr, EbPictureBufferDesc *input_picture_ptr,
                               ModeDecisionContext *context_ptr, uint32_t input_cb_origin_in_index,
                               uint32_t                     blk_chroma_origin_index,
                               ModeDecisionCandidateBuffer *candidate_buffer, uint8_t cb_qp,
                               uint8_t cr_qp, uint64_t *cb_full_distortion,
                               uint64_t *cr_full_distortion, uint64_t *cb_coeff_bits,
                               uint64_t *cr_coeff_bits) {
    if (candidate_buffer->candidate_ptr->filter_intra_mode != FILTER_INTRA_MODES)
        assert(candidate_buffer->candidate_ptr->intra_luma_mode == DC_PRED);
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;
    // cfl cost
    uint64_t chroma_rate = 0;
    if (candidate_buffer->candidate_ptr->intra_chroma_mode == UV_CFL_PRED) {
        chroma_rate +=
            candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                ->cfl_alpha_fac_bits[candidate_buffer->candidate_ptr->cfl_alpha_signs][CFL_PRED_U]
                                    [CFL_IDX_U(candidate_buffer->candidate_ptr->cfl_alpha_idx)] +
            candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                ->cfl_alpha_fac_bits[candidate_buffer->candidate_ptr->cfl_alpha_signs][CFL_PRED_V]
                                    [CFL_IDX_V(candidate_buffer->candidate_ptr->cfl_alpha_idx)];

        chroma_rate +=
            (uint64_t)candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                ->intra_uv_mode_fac_bits[CFL_ALLOWED][candidate_buffer->candidate_ptr
                                                          ->intra_luma_mode][UV_CFL_PRED];
        chroma_rate -= (uint64_t)candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                           ->intra_uv_mode_fac_bits[CFL_ALLOWED][candidate_buffer->candidate_ptr
                                                                     ->intra_luma_mode][UV_DC_PRED];
    } else
        chroma_rate = (uint64_t)candidate_buffer->candidate_ptr->md_rate_estimation_ptr
                          ->intra_uv_mode_fac_bits[CFL_ALLOWED][candidate_buffer->candidate_ptr
                                                                    ->intra_luma_mode][UV_DC_PRED];
    int coeff_rate = (int)(*cb_coeff_bits + *cr_coeff_bits);
    int distortion =
        (int)(cb_full_distortion[DIST_CALC_RESIDUAL] + cr_full_distortion[DIST_CALC_RESIDUAL]);
    int rate = (int)(coeff_rate + chroma_rate + candidate_buffer->candidate_ptr->fast_luma_rate);
    uint64_t cfl_uv_cost = RDCOST(context_ptr->full_lambda, rate, distortion);

    // cfl vs. best independant
    if (context_ptr->best_uv_cost[candidate_buffer->candidate_ptr->intra_luma_mode]
                                 [3 + candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_Y]] <
        cfl_uv_cost) {
        // Update the current candidate
        candidate_buffer->candidate_ptr->intra_chroma_mode =
            context_ptr->best_uv_mode[candidate_buffer->candidate_ptr->intra_luma_mode]
                                     [MAX_ANGLE_DELTA +
                                      candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_Y]];
        candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_UV] =
            context_ptr->best_uv_angle[candidate_buffer->candidate_ptr->intra_luma_mode]
                                      [MAX_ANGLE_DELTA +
                                       candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_Y]];
        candidate_buffer->candidate_ptr->is_directional_chroma_mode_flag =
            (uint8_t)av1_is_directional_mode((PredictionMode)(
                context_ptr->best_uv_mode[candidate_buffer->candidate_ptr->intra_luma_mode]
                                         [MAX_ANGLE_DELTA + candidate_buffer->candidate_ptr
                                                                ->angle_delta[PLANE_TYPE_Y]]));

        // check if candidate_buffer->candidate_ptr->fast_luma_rate = context_ptr->fast_luma_rate[candidate_buffer->candidate_ptr->intra_luma_mode];
        candidate_buffer->candidate_ptr->fast_chroma_rate =
            context_ptr->fast_chroma_rate[candidate_buffer->candidate_ptr->intra_luma_mode]
                                         [MAX_ANGLE_DELTA + candidate_buffer->candidate_ptr
                                                                ->angle_delta[PLANE_TYPE_Y]];

        candidate_buffer->candidate_ptr->transform_type_uv = av1_get_tx_type(
            context_ptr->blk_geom->bsize,
            0,
            (PredictionMode)NULL,
            (UvPredictionMode)context_ptr
                ->best_uv_mode[candidate_buffer->candidate_ptr->intra_luma_mode]
                              [3 + candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_Y]],
            PLANE_TYPE_UV,
            0,
            0,
            0,
            context_ptr->blk_geom->txsize_uv[0][0],
            frm_hdr->reduced_tx_set);

        // Start uv search path
        context_ptr->uv_search_path = EB_TRUE;

        memset(candidate_buffer->candidate_ptr->eob[1], 0, sizeof(uint16_t));
        memset(candidate_buffer->candidate_ptr->eob[2], 0, sizeof(uint16_t));
        candidate_buffer->candidate_ptr->u_has_coeff = 0;
        candidate_buffer->candidate_ptr->v_has_coeff = 0;
        cb_full_distortion[DIST_CALC_RESIDUAL]       = 0;
        cr_full_distortion[DIST_CALC_RESIDUAL]       = 0;
        cb_full_distortion[DIST_CALC_PREDICTION]     = 0;
        cr_full_distortion[DIST_CALC_PREDICTION]     = 0;

        *cb_coeff_bits = 0;
        *cr_coeff_bits = 0;

        uint32_t count_non_zero_coeffs[3][MAX_NUM_OF_TU_PER_CU];
        context_ptr->md_staging_skip_inter_chroma_pred = EB_FALSE;
        product_prediction_fun_table[candidate_buffer->candidate_ptr->type](
            context_ptr->hbd_mode_decision, context_ptr, pcs_ptr, candidate_buffer);

        // Cb Residual
        residual_kernel(input_picture_ptr->buffer_cb,
                        input_cb_origin_in_index,
                        input_picture_ptr->stride_cb,
                        candidate_buffer->prediction_ptr->buffer_cb,
                        blk_chroma_origin_index,
                        candidate_buffer->prediction_ptr->stride_cb,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_cb,
                        blk_chroma_origin_index,
                        candidate_buffer->residual_ptr->stride_cb,
                        context_ptr->hbd_mode_decision,
                        context_ptr->blk_geom->bwidth_uv,
                        context_ptr->blk_geom->bheight_uv);

        // Cr Residual
        residual_kernel(input_picture_ptr->buffer_cr,
                        input_cb_origin_in_index,
                        input_picture_ptr->stride_cr,
                        candidate_buffer->prediction_ptr->buffer_cr,
                        blk_chroma_origin_index,
                        candidate_buffer->prediction_ptr->stride_cr,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_cr,
                        blk_chroma_origin_index,
                        candidate_buffer->residual_ptr->stride_cr,
                        context_ptr->hbd_mode_decision,
                        context_ptr->blk_geom->bwidth_uv,
                        context_ptr->blk_geom->bheight_uv);

        full_loop_r(context_ptr->sb_ptr,
                    candidate_buffer,
                    context_ptr,
                    input_picture_ptr,
                    pcs_ptr,
                    PICTURE_BUFFER_DESC_CHROMA_MASK,
                    cb_qp,
                    cr_qp,
                    &(*count_non_zero_coeffs[1]),
                    &(*count_non_zero_coeffs[2]));

        cu_full_distortion_fast_txb_mode_r(context_ptr->sb_ptr,
                                           candidate_buffer,
                                           context_ptr,
                                           candidate_buffer->candidate_ptr,
                                           pcs_ptr,
                                           input_picture_ptr,
                                           cb_full_distortion,
                                           cr_full_distortion,
                                           count_non_zero_coeffs,
                                           COMPONENT_CHROMA,
                                           cb_coeff_bits,
                                           cr_coeff_bits,
                                           1);

        // End uv search path
        context_ptr->uv_search_path = EB_FALSE;
    }
}

// double check the usage of tx_search_luma_recon_neighbor_array16bit
EbErrorType av1_intra_luma_prediction(ModeDecisionContext *        md_context_ptr,
                                      PictureControlSet *          pcs_ptr,
                                      ModeDecisionCandidateBuffer *candidate_buffer_ptr) {
    EbErrorType return_error = EB_ErrorNone;

#if TX_ORG_INTERINTRA
    uint16_t txb_origin_x =
        md_context_ptr->blk_origin_x +
        md_context_ptr->blk_geom->tx_org_x[0][md_context_ptr->tx_depth][md_context_ptr->txb_itr] -
        md_context_ptr->blk_geom->origin_x;
    uint16_t txb_origin_y =
        md_context_ptr->blk_origin_y +
        md_context_ptr->blk_geom->tx_org_y[0][md_context_ptr->tx_depth][md_context_ptr->txb_itr] -
        md_context_ptr->blk_geom->origin_y;
#else
    uint16_t txb_origin_x =
        md_context_ptr->blk_origin_x +
        md_context_ptr->blk_geom->tx_boff_x[md_context_ptr->tx_depth][md_context_ptr->txb_itr];
    uint16_t txb_origin_y =
        md_context_ptr->blk_origin_y +
        md_context_ptr->blk_geom->tx_boff_y[md_context_ptr->tx_depth][md_context_ptr->txb_itr];
#endif
    uint8_t tx_width =
        md_context_ptr->blk_geom->tx_width[md_context_ptr->tx_depth][md_context_ptr->txb_itr];
    uint8_t tx_height =
        md_context_ptr->blk_geom->tx_height[md_context_ptr->tx_depth][md_context_ptr->txb_itr];

    uint32_t mode_type_left_neighbor_index =
        get_neighbor_array_unit_left_index(md_context_ptr->mode_type_neighbor_array, txb_origin_y);
    uint32_t mode_type_top_neighbor_index =
        get_neighbor_array_unit_top_index(md_context_ptr->mode_type_neighbor_array, txb_origin_x);
    uint32_t intra_luma_mode_left_neighbor_index = get_neighbor_array_unit_left_index(
        md_context_ptr->intra_luma_mode_neighbor_array, txb_origin_y);
    uint32_t intra_luma_mode_top_neighbor_index = get_neighbor_array_unit_top_index(
        md_context_ptr->intra_luma_mode_neighbor_array, txb_origin_x);

    md_context_ptr->intra_luma_left_mode = (uint32_t)(
        (md_context_ptr->mode_type_neighbor_array->left_array[mode_type_left_neighbor_index] !=
         INTRA_MODE)
            ? DC_PRED /*EB_INTRA_DC*/
            : (uint32_t)md_context_ptr->intra_luma_mode_neighbor_array
                  ->left_array[intra_luma_mode_left_neighbor_index]);

    md_context_ptr->intra_luma_top_mode = (uint32_t)(
        (md_context_ptr->mode_type_neighbor_array->top_array[mode_type_top_neighbor_index] !=
         INTRA_MODE)
            ? DC_PRED /*EB_INTRA_DC*/
            : (uint32_t)md_context_ptr->intra_luma_mode_neighbor_array->top_array
                  [intra_luma_mode_top_neighbor_index]); //   use DC. This seems like we could use a SB-width

    TxSize tx_size =
        md_context_ptr->blk_geom->txsize[md_context_ptr->tx_depth][md_context_ptr->txb_itr];

    PredictionMode mode;
    if (!md_context_ptr->hbd_mode_decision) {
        uint8_t top_neigh_array[64 * 2 + 1];
        uint8_t left_neigh_array[64 * 2 + 1];

        if (txb_origin_y != 0)
            memcpy(top_neigh_array + 1,
                   md_context_ptr->tx_search_luma_recon_neighbor_array->top_array + txb_origin_x,
                   tx_width * 2);
        if (txb_origin_x != 0)
            memcpy(left_neigh_array + 1,
                   md_context_ptr->tx_search_luma_recon_neighbor_array->left_array + txb_origin_y,
                   tx_height * 2);
        if (txb_origin_y != 0 && txb_origin_x != 0)
            top_neigh_array[0] = left_neigh_array[0] =
                md_context_ptr->tx_search_luma_recon_neighbor_array
                    ->top_left_array[MAX_PICTURE_HEIGHT_SIZE + txb_origin_x - txb_origin_y];

        mode = candidate_buffer_ptr->candidate_ptr->pred_mode;
        eb_av1_predict_intra_block(
            &md_context_ptr->sb_ptr->tile_info,
            !ED_STAGE,
            md_context_ptr->blk_geom,
            pcs_ptr->parent_pcs_ptr->av1_cm, //const Av1Common *cm,
            md_context_ptr->blk_geom->bwidth,
            md_context_ptr->blk_geom->bheight,
            tx_size,
            mode, //PredictionMode mode,
            candidate_buffer_ptr->candidate_ptr->angle_delta[PLANE_TYPE_Y],
            candidate_buffer_ptr->candidate_ptr->palette_info.pmi.palette_size[0] > 0,
            &candidate_buffer_ptr->candidate_ptr->palette_info, //ATB MD
            candidate_buffer_ptr->candidate_ptr->filter_intra_mode,
            top_neigh_array + 1,
            left_neigh_array + 1,
            candidate_buffer_ptr->prediction_ptr, //uint8_t *dst,
#if TX_ORG_INTERINTRA
            (md_context_ptr->blk_geom
                 ->tx_org_x[0][md_context_ptr->tx_depth][md_context_ptr->txb_itr] -
             md_context_ptr->blk_geom->origin_x) >>
                2,
            (md_context_ptr->blk_geom
                 ->tx_org_y[0][md_context_ptr->tx_depth][md_context_ptr->txb_itr] -
             md_context_ptr->blk_geom->origin_y) >>
                2,
#else
            md_context_ptr->blk_geom
                    ->tx_boff_x[md_context_ptr->tx_depth][md_context_ptr->txb_itr] >>
                2, //int32_t col_off,
            md_context_ptr->blk_geom
                    ->tx_boff_y[md_context_ptr->tx_depth][md_context_ptr->txb_itr] >>
                2, //int32_t row_off,
#endif
            PLANE_TYPE_Y, //int32_t plane,
            md_context_ptr->blk_geom->bsize,
            md_context_ptr->blk_origin_x,
            md_context_ptr->blk_origin_y,
            md_context_ptr->blk_origin_x,
            md_context_ptr->blk_origin_y,
#if TX_ORG_INTERINTRA
            md_context_ptr->blk_geom
                ->tx_org_x[0][md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr], //uint32_t cuOrgX used only for prediction Ptr
            md_context_ptr->blk_geom
                ->tx_org_y[0][md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr] //uint32_t cuOrgY used only for prediction Ptr
#else
            md_context_ptr->blk_geom
                ->tx_org_x[md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr], //uint32_t cuOrgX used only for prediction Ptr
            md_context_ptr->blk_geom
                ->tx_org_y[md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr] //uint32_t cuOrgY used only for prediction Ptr
#endif
        );
    } else {
        uint16_t top_neigh_array[64 * 2 + 1];
        uint16_t left_neigh_array[64 * 2 + 1];

        if (txb_origin_y != 0)
            memcpy(
                top_neigh_array + 1,
                (uint16_t *)(md_context_ptr->tx_search_luma_recon_neighbor_array16bit->top_array) +
                    txb_origin_x,
                sizeof(uint16_t) * tx_width * 2);
        if (txb_origin_x != 0)
            memcpy(
                left_neigh_array + 1,
                (uint16_t *)(md_context_ptr->tx_search_luma_recon_neighbor_array16bit->left_array) +
                    txb_origin_y,
                sizeof(uint16_t) * tx_height * 2);
        if (txb_origin_y != 0 && txb_origin_x != 0)
            top_neigh_array[0] = left_neigh_array[0] =
                ((uint16_t *)(md_context_ptr->tx_search_luma_recon_neighbor_array16bit
                                  ->top_left_array) +
                 MAX_PICTURE_HEIGHT_SIZE + txb_origin_x - txb_origin_y)[0];

        mode = candidate_buffer_ptr->candidate_ptr->pred_mode;
        eb_av1_predict_intra_block_16bit(
            &md_context_ptr->sb_ptr->tile_info,
            !ED_STAGE,
            md_context_ptr->blk_geom,
            pcs_ptr->parent_pcs_ptr->av1_cm,
            md_context_ptr->blk_geom->bwidth,
            md_context_ptr->blk_geom->bheight,
            tx_size,
            mode,
            candidate_buffer_ptr->candidate_ptr->angle_delta[PLANE_TYPE_Y],
            candidate_buffer_ptr->candidate_ptr->palette_info.pmi.palette_size[0] > 0,
            &candidate_buffer_ptr->candidate_ptr->palette_info, //ATB MD
            candidate_buffer_ptr->candidate_ptr->filter_intra_mode,
            top_neigh_array + 1,
            left_neigh_array + 1,
            candidate_buffer_ptr->prediction_ptr,
#if TX_ORG_INTERINTRA
            (md_context_ptr->blk_geom
                 ->tx_org_x[0][md_context_ptr->tx_depth][md_context_ptr->txb_itr] -
             md_context_ptr->blk_geom->origin_x) >>
                2,
            (md_context_ptr->blk_geom
                 ->tx_org_y[0][md_context_ptr->tx_depth][md_context_ptr->txb_itr] -
             md_context_ptr->blk_geom->origin_y) >>
                2,
#else
            md_context_ptr->blk_geom
                    ->tx_boff_x[md_context_ptr->tx_depth][md_context_ptr->txb_itr] >>
                2,
            md_context_ptr->blk_geom
                    ->tx_boff_y[md_context_ptr->tx_depth][md_context_ptr->txb_itr] >>
                2, //int32_t row_off,
#endif
            PLANE_TYPE_Y,
            md_context_ptr->blk_geom->bsize,
            md_context_ptr->blk_origin_x,
            md_context_ptr->blk_origin_y,
            md_context_ptr->blk_origin_x,
            md_context_ptr->blk_origin_y,
#if TX_ORG_INTERINTRA
            md_context_ptr->blk_geom
                ->tx_org_x[0][md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr], //uint32_t cuOrgX used only for prediction Ptr
            md_context_ptr->blk_geom
                ->tx_org_y[0][md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr] //uint32_t cuOrgY used only for prediction Ptr
#else
            md_context_ptr->blk_geom
                ->tx_org_x[md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr], //uint32_t cuOrgX used only for prediction Ptr
            md_context_ptr->blk_geom
                ->tx_org_y[md_context_ptr->tx_depth]
                          [md_context_ptr->txb_itr] //uint32_t cuOrgY used only for prediction Ptr
#endif
        );
    }

    return return_error;
}

static void tx_search_update_recon_sample_neighbor_array(
    NeighborArrayUnit *lumaReconSampleNeighborArray, EbPictureBufferDesc *recon_buffer,
    uint32_t txb_origin_x, uint32_t txb_origin_y, uint32_t input_origin_x, uint32_t input_origin_y,
    uint32_t width, uint32_t height, EbBool hbd) {
    if (hbd) {
        neighbor_array_unit16bit_sample_write(lumaReconSampleNeighborArray,
                                              (uint16_t *)recon_buffer->buffer_y,
                                              recon_buffer->stride_y,
                                              recon_buffer->origin_x + txb_origin_x,
                                              recon_buffer->origin_y + txb_origin_y,
                                              input_origin_x,
                                              input_origin_y,
                                              width,
                                              height,
                                              NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    } else {
        neighbor_array_unit_sample_write(lumaReconSampleNeighborArray,
                                         recon_buffer->buffer_y,
                                         recon_buffer->stride_y,
                                         recon_buffer->origin_x + txb_origin_x,
                                         recon_buffer->origin_y + txb_origin_y,
                                         input_origin_x,
                                         input_origin_y,
                                         width,
                                         height,
                                         NEIGHBOR_ARRAY_UNIT_FULL_MASK);
    }

    return;
}

uint8_t get_end_tx_depth(BlockSize bsize, uint8_t btype) {
    uint8_t tx_depth = 0;
    if (bsize == BLOCK_64X64 || bsize == BLOCK_32X32 || bsize == BLOCK_16X16 ||
        bsize == BLOCK_64X32 || bsize == BLOCK_32X64 || bsize == BLOCK_16X32 ||
        bsize == BLOCK_32X16 || bsize == BLOCK_16X8 || bsize == BLOCK_8X16)
        tx_depth = (btype == INTRA_MODE) ? 1 : 1;
    else if (bsize == BLOCK_8X8 || bsize == BLOCK_64X16 || bsize == BLOCK_16X64 ||
             bsize == BLOCK_32X8 || bsize == BLOCK_8X32 || bsize == BLOCK_16X4 ||
             bsize == BLOCK_4X16)
        tx_depth = (btype == INTRA_MODE) ? 1 : 1;

    return tx_depth;
}

uint8_t allowed_tx_set_a[TX_SIZES_ALL][TX_TYPES];

void tx_initialize_neighbor_arrays(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                                   EbBool is_inter) {
#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
#endif
    // Set recon neighbor array to be used @ intra compensation
    if (!is_inter) {
        if (context_ptr->hbd_mode_decision)
            context_ptr->tx_search_luma_recon_neighbor_array16bit =
                (context_ptr->tx_depth)
#if TILES_PARALLEL
                    ? pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx]
                    : pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
#else
                    ? pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX]
                    : pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX];
#endif
        else
            context_ptr->tx_search_luma_recon_neighbor_array =
                (context_ptr->tx_depth)
#if TILES_PARALLEL
                    ? pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx]
                    : pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
#else
                    ? pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX]
                    : pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
#endif
    }
    // Set luma dc sign level coeff
    context_ptr->full_loop_luma_dc_sign_level_coeff_neighbor_array =
        (context_ptr->tx_depth == 1)
#if TILES_PARALLEL
            ? pcs_ptr
                  ->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx]
            : pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
#else
            ? pcs_ptr
                  ->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX]
            : pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
#endif
}

void tx_update_neighbor_arrays(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                               ModeDecisionCandidateBuffer *candidate_buffer, EbBool is_inter) {
#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
#endif
    if (context_ptr->tx_depth) {
        if (!is_inter)
            tx_search_update_recon_sample_neighbor_array(
                context_ptr->hbd_mode_decision
                    ? context_ptr->tx_search_luma_recon_neighbor_array16bit
                    : context_ptr->tx_search_luma_recon_neighbor_array,
                candidate_buffer->recon_ptr,
#if TX_ORG_INTERINTRA
                context_ptr->blk_geom
                    ->tx_org_x[is_inter][context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom
                    ->tx_org_y[is_inter][context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->sb_origin_x +
                    context_ptr->blk_geom
                        ->tx_org_x[is_inter][context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->sb_origin_y +
                    context_ptr->blk_geom
                        ->tx_org_y[is_inter][context_ptr->tx_depth][context_ptr->txb_itr],
#else
                context_ptr->blk_geom->tx_org_x[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_org_y[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->sb_origin_x +
                    context_ptr->blk_geom->tx_org_x[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->sb_origin_y +
                    context_ptr->blk_geom->tx_org_y[context_ptr->tx_depth][context_ptr->txb_itr],
#endif
                context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->hbd_mode_decision);

        int8_t dc_sign_level_coeff =
            candidate_buffer->candidate_ptr->quantized_dc[0][context_ptr->txb_itr];
        neighbor_array_unit_mode_write(
#if TILES_PARALLEL
            pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
            pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
#endif
            (uint8_t *)&dc_sign_level_coeff,
#if TX_ORG_INTERINTRA
            context_ptr->sb_origin_x +
                context_ptr->blk_geom
                    ->tx_org_x[is_inter][context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->sb_origin_y +
                context_ptr->blk_geom
                    ->tx_org_y[is_inter][context_ptr->tx_depth][context_ptr->txb_itr],
#else
            context_ptr->sb_origin_x +
                context_ptr->blk_geom->tx_org_x[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->sb_origin_y +
                context_ptr->blk_geom->tx_org_y[context_ptr->tx_depth][context_ptr->txb_itr],
#endif
            context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr],
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }
}

void tx_reset_neighbor_arrays(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                              EbBool is_inter, uint8_t end_tx_depth) {
#if TILES_PARALLEL
    uint16_t tile_idx = context_ptr->tile_index;
#endif
    if (end_tx_depth) {
        if (!is_inter) {
            if (context_ptr->hbd_mode_decision) {
                copy_neigh_arr(
#if TILES_PARALLEL
                    pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                    pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX],
#endif
                    context_ptr->sb_origin_x + context_ptr->blk_geom->origin_x,
                    context_ptr->sb_origin_y + context_ptr->blk_geom->origin_y,
                    context_ptr->blk_geom->bwidth,
                    context_ptr->blk_geom->bheight,
                    NEIGHBOR_ARRAY_UNIT_TOPLEFT_MASK);
                copy_neigh_arr(
#if TILES_PARALLEL
                    pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                    pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX],
#endif
                    context_ptr->sb_origin_x + context_ptr->blk_geom->origin_x,
                    context_ptr->sb_origin_y + context_ptr->blk_geom->origin_y,
                    context_ptr->blk_geom->bwidth * 2,
                    context_ptr->blk_geom->bheight * 2,
                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
            }

            else {
                copy_neigh_arr(
#if TILES_PARALLEL
                    pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                    pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
#endif
                    context_ptr->sb_origin_x + context_ptr->blk_geom->origin_x,
                    context_ptr->sb_origin_y + context_ptr->blk_geom->origin_y,
                    context_ptr->blk_geom->bwidth,
                    context_ptr->blk_geom->bheight,
                    NEIGHBOR_ARRAY_UNIT_TOPLEFT_MASK);

                copy_neigh_arr(
#if TILES_PARALLEL
                    pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
                    pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
                    pcs_ptr->md_tx_depth_1_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
#endif
                    context_ptr->sb_origin_x + context_ptr->blk_geom->origin_x,
                    context_ptr->sb_origin_y + context_ptr->blk_geom->origin_y,
                    context_ptr->blk_geom->bwidth * 2,
                    context_ptr->blk_geom->bheight * 2,
                    NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
            }
        }

        copy_neigh_arr(
#if TILES_PARALLEL
            pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
            pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx],
#else
            pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
            pcs_ptr->md_tx_depth_1_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX],
#endif
            context_ptr->sb_origin_x + context_ptr->blk_geom->origin_x,
            context_ptr->sb_origin_y + context_ptr->blk_geom->origin_y,
            context_ptr->blk_geom->bwidth,
            context_ptr->blk_geom->bheight,
            NEIGHBOR_ARRAY_UNIT_TOP_AND_LEFT_ONLY_MASK);
    }
}

void tx_type_search(PictureControlSet *pcs_ptr,
                    ModeDecisionContext *context_ptr, ModeDecisionCandidateBuffer *candidate_buffer,
                    uint32_t qp) {
    EbPictureBufferDesc *input_picture_ptr = context_ptr->hbd_mode_decision
                                                 ? pcs_ptr->input_frame16bit
                                                 : pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    int32_t seg_qp = pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params.segmentation_enabled
                         ? pcs_ptr->parent_pcs_ptr->frm_hdr.segmentation_params
                               .feature_data[context_ptr->blk_ptr->segment_id][SEG_LVL_ALT_Q]
                         : 0;

    TxType   txk_start           = DCT_DCT;
    TxType   txk_end             = TX_TYPES;
    uint64_t best_cost_tx_search = (uint64_t)~0;
    int32_t  tx_type;
    TxSize   tx_size  = context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr];
    int32_t  is_inter = (candidate_buffer->candidate_ptr->type == INTER_MODE ||
                        candidate_buffer->candidate_ptr->use_intrabc)
                           ? EB_TRUE
                           : EB_FALSE;
    const TxSetType tx_set_type =
        get_ext_tx_set_type(tx_size, is_inter, context_ptr->tx_search_reduced_set);

#if TX_ORG_INTERINTRA
    uint8_t txb_origin_x =
        (uint8_t)
            context_ptr->blk_geom->tx_org_x[is_inter][context_ptr->tx_depth][context_ptr->txb_itr];
    uint8_t txb_origin_y =
        (uint8_t)
            context_ptr->blk_geom->tx_org_y[is_inter][context_ptr->tx_depth][context_ptr->txb_itr];
#else
    uint8_t txb_origin_x =
        (uint8_t)context_ptr->blk_geom->tx_org_x[context_ptr->tx_depth][context_ptr->txb_itr];
    uint8_t txb_origin_y =
        (uint8_t)context_ptr->blk_geom->tx_org_y[context_ptr->tx_depth][context_ptr->txb_itr];
#endif
    uint32_t txb_origin_index =
        txb_origin_x + (txb_origin_y * candidate_buffer->residual_ptr->stride_y);
    uint32_t input_txb_origin_index =
        (context_ptr->sb_origin_x + txb_origin_x + input_picture_ptr->origin_x) +
        ((context_ptr->sb_origin_y + txb_origin_y + input_picture_ptr->origin_y) *
         input_picture_ptr->stride_y);

    context_ptr->luma_txb_skip_context = 0;
    context_ptr->luma_dc_sign_context  = 0;
    get_txb_ctx(pcs_ptr,
                COMPONENT_LUMA,
                context_ptr->full_loop_luma_dc_sign_level_coeff_neighbor_array,
                context_ptr->sb_origin_x + txb_origin_x,
                context_ptr->sb_origin_y + txb_origin_y,
                context_ptr->blk_geom->bsize,
                context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
                &context_ptr->luma_txb_skip_context,
                &context_ptr->luma_dc_sign_context);
    if (context_ptr->tx_search_reduced_set == 2) txk_end = 2;
#if TX_SEARCH_REDUCED
    // part of 2
    int32_t allowed_tx_mask[TX_TYPES] = {0}; // 1: allow; 0: skip.
    int32_t allowed_tx_num            = 0;
    int32_t plane                     = 0;
    TxType  uv_tx_type                = DCT_DCT;
    for (int32_t tx_type_index = txk_start; tx_type_index < txk_end; ++tx_type_index) {
        if (context_ptr->tx_search_reduced_set == 2)
            tx_type_index = (tx_type_index == 1) ? IDTX : tx_type_index;
        tx_type                  = (TxType)tx_type_index;
        allowed_tx_mask[tx_type] = 1;
        if (plane == 0) {
            if (allowed_tx_mask[tx_type]) {
                const TxType ref_tx_type = ((!av1_ext_tx_used[tx_set_type][tx_type]) ||
                                            txsize_sqr_up_map[tx_size] > TX_32X32)
                                               ? DCT_DCT
                                               : tx_type;
                if (tx_type != ref_tx_type) allowed_tx_mask[tx_type] = 0;
            }
        }

        allowed_tx_num += allowed_tx_mask[tx_type];
    }
    // Need to have at least one transform type allowed.
    if (allowed_tx_num == 0) allowed_tx_mask[plane ? uv_tx_type : DCT_DCT] = 1;
    //if (allowed_tx_mask[tx_type] &&
    //    !(context_ptr->tx_search_reduced_set && !allowed_tx_set_a[tx_size][tx_type])) {
    //    context_ptr->luma_txb_skip_context = 0;
    //    context_ptr->luma_dc_sign_context  = 0;
    //    get_txb_ctx(scs_ptr,
    //                COMPONENT_LUMA,
    //                context_ptr->full_loop_luma_dc_sign_level_coeff_neighbor_array,
    //                context_ptr->sb_origin_x + txb_origin_x,
    //                context_ptr->sb_origin_y + txb_origin_y,
    //                context_ptr->blk_geom->bsize,
    //                context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
    //                &context_ptr->luma_txb_skip_context,
    //                &context_ptr->luma_dc_sign_context);
    //    context_ptr->three_quad_energy = 0;
    //}
#endif
    TxType best_tx_type = DCT_DCT;
    for (tx_type = txk_start; tx_type < txk_end; ++tx_type) {
        uint64_t txb_full_distortion[3][DIST_CALC_TOTAL];
        uint64_t y_txb_coeff_bits = 0;
        uint32_t y_count_non_zero_coeffs;

#if TX_SEARCH_REDUCED
        if (context_ptr->tx_search_reduced_set == 2) tx_type = (tx_type == 1) ? IDTX : tx_type;
        // below is part of 2
        if (!allowed_tx_mask[tx_type]) continue;
        //if (context_ptr->tx_search_reduced_set) // done below
        //    if (!allowed_tx_set_a[tx_size][tx_type]) continue;
#endif
#if FREQUENCY_SPATIAL_DOMAIN
        context_ptr->three_quad_energy = 0;
#else
        //context_ptr->three_quad_energy = 0;
#endif
        if (tx_type != DCT_DCT) {
            if (is_inter) {
                TxSize          max_tx_size = context_ptr->blk_geom->txsize[0][0];
                const TxSetType tx_set_type_inter =
                    get_ext_tx_set_type(max_tx_size, is_inter, context_ptr->tx_search_reduced_set);
                int32_t eset =
                    get_ext_tx_set(max_tx_size, is_inter, context_ptr->tx_search_reduced_set);
                // eset == 0 should correspond to a set with only DCT_DCT and there
                // is no need to send the tx_type
                if (eset <= 0)
                    continue;
                else if (av1_ext_tx_used[tx_set_type_inter][tx_type] == 0)
                    continue;
                else if (context_ptr->blk_geom
                                 ->tx_height[context_ptr->tx_depth][context_ptr->txb_itr] > 32 ||
                         context_ptr->blk_geom
                                 ->tx_width[context_ptr->tx_depth][context_ptr->txb_itr] > 32)
                    continue;
            }
            int32_t eset = get_ext_tx_set(
                context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
                is_inter,
                context_ptr->tx_search_reduced_set);
            // eset == 0 should correspond to a set with only DCT_DCT and there
            // is no need to send the tx_type
            if (eset <= 0)
                continue;
            else if (av1_ext_tx_used[tx_set_type][tx_type] == 0)
                continue;
            else if (context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr] >
                         32 ||
                     context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr] >
                         32)
                continue;
        }
        if (context_ptr->tx_search_reduced_set)
            if (!allowed_tx_set_a[context_ptr->blk_geom->txsize[context_ptr->tx_depth]
                                                               [context_ptr->txb_itr]][tx_type])
                continue;

        // For Inter blocks, transform type of chroma follows luma transfrom type
        if (is_inter)
            candidate_buffer->candidate_ptr->transform_type_uv =
                (context_ptr->txb_itr == 0)
                    ? candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr]
                    : candidate_buffer->candidate_ptr->transform_type_uv;

        // Y: T Q i_q
        av1_estimate_transform(
            &(((int16_t *)candidate_buffer->residual_ptr->buffer_y)[txb_origin_index]),
            candidate_buffer->residual_ptr->stride_y,
            &(((int32_t *)context_ptr->trans_quant_buffers_ptr->txb_trans_coeff2_nx2_n_ptr
                   ->buffer_y)[context_ptr->txb_1d_offset]),
            NOT_USED_VALUE,
            context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
            &context_ptr->three_quad_energy,
            context_ptr->transform_inner_array_ptr,
            context_ptr->hbd_mode_decision ? BIT_INCREMENT_10BIT : BIT_INCREMENT_8BIT,
            tx_type,
            PLANE_TYPE_Y,
            DEFAULT_SHAPE);

        av1_quantize_inv_quantize(
            pcs_ptr,
            context_ptr,
            &(((int32_t *)context_ptr->trans_quant_buffers_ptr->txb_trans_coeff2_nx2_n_ptr
                   ->buffer_y)[context_ptr->txb_1d_offset]),
            NOT_USED_VALUE,
            &(((int32_t *)candidate_buffer->residual_quant_coeff_ptr
                   ->buffer_y)[context_ptr->txb_1d_offset]),
            &(((int32_t *)candidate_buffer->recon_coeff_ptr->buffer_y)[context_ptr->txb_1d_offset]),
            qp,
            seg_qp,
            context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
            &candidate_buffer->candidate_ptr->eob[0][context_ptr->txb_itr],
            &y_count_non_zero_coeffs,
            COMPONENT_LUMA,
            context_ptr->hbd_mode_decision ? BIT_INCREMENT_10BIT : BIT_INCREMENT_8BIT,
            tx_type,
            candidate_buffer,
            context_ptr->luma_txb_skip_context,
            context_ptr->luma_dc_sign_context,
            candidate_buffer->candidate_ptr->pred_mode,
#if LOSSLESS_TX_TYPE_OPT
            candidate_buffer->candidate_ptr->use_intrabc,
#else
            EB_FALSE,
#endif
            EB_FALSE);

        candidate_buffer->candidate_ptr->quantized_dc[0][context_ptr->txb_itr] =
            (((int32_t *)candidate_buffer->residual_quant_coeff_ptr
                  ->buffer_y)[context_ptr->txb_1d_offset]);
        uint32_t y_has_coeff = y_count_non_zero_coeffs > 0;

        // tx_type not equal to DCT_DCT and no coeff is not an acceptable option in AV1.
        if (y_has_coeff == 0 && tx_type != DCT_DCT) continue;
#if FREQUENCY_SPATIAL_DOMAIN
        if (context_ptr->md_staging_spatial_sse_full_loop) {
#endif
        if (y_has_coeff)
            inv_transform_recon_wrapper(
                candidate_buffer->prediction_ptr->buffer_y,
                txb_origin_index,
                candidate_buffer->prediction_ptr->stride_y,
                candidate_buffer->recon_ptr->buffer_y,
                txb_origin_index,
                candidate_buffer->recon_ptr->stride_y,
                (int32_t *)candidate_buffer->recon_coeff_ptr->buffer_y,
                context_ptr->txb_1d_offset,
                context_ptr->hbd_mode_decision,
                context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
                tx_type,
                PLANE_TYPE_Y,
                (uint16_t)candidate_buffer->candidate_ptr->eob[0][context_ptr->txb_itr]);
        else
            picture_copy(
                candidate_buffer->prediction_ptr,
                txb_origin_index,
                0,
                candidate_buffer->recon_ptr,
                txb_origin_index,
                0,
                context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr],
                0,
                0,
                PICTURE_BUFFER_DESC_Y_FLAG,
                context_ptr->hbd_mode_decision);

        EbSpatialFullDistType spatial_full_dist_type_fun = context_ptr->hbd_mode_decision
                                                               ? full_distortion_kernel16_bits
                                                               : spatial_full_distortion_kernel;

        txb_full_distortion[0][DIST_CALC_PREDICTION] = spatial_full_dist_type_fun(
            input_picture_ptr->buffer_y,
            input_txb_origin_index,
            input_picture_ptr->stride_y,
            candidate_buffer->prediction_ptr->buffer_y,
            txb_origin_index,
            candidate_buffer->prediction_ptr->stride_y,
            context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr]);

        txb_full_distortion[0][DIST_CALC_RESIDUAL] = spatial_full_dist_type_fun(
            input_picture_ptr->buffer_y,
            input_txb_origin_index,
            input_picture_ptr->stride_y,
            candidate_buffer->recon_ptr->buffer_y,
            txb_origin_index,
            candidate_buffer->recon_ptr->stride_y,
            context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr]);

        txb_full_distortion[0][DIST_CALC_PREDICTION] <<= 4;
        txb_full_distortion[0][DIST_CALC_RESIDUAL] <<= 4;
#if FREQUENCY_SPATIAL_DOMAIN
        } else {
            // LUMA DISTORTION
            picture_full_distortion32_bits(
                context_ptr->trans_quant_buffers_ptr->txb_trans_coeff2_nx2_n_ptr,
                context_ptr->txb_1d_offset,
                0,
                candidate_buffer->recon_coeff_ptr,
                context_ptr->txb_1d_offset,
                0,
                context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
                context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr],
                NOT_USED_VALUE,
                NOT_USED_VALUE,
                txb_full_distortion[0],
                NOT_USED_VALUE,
                NOT_USED_VALUE,
                y_count_non_zero_coeffs,
                0,
                0,
                COMPONENT_LUMA);

            txb_full_distortion[0][DIST_CALC_RESIDUAL] += context_ptr->three_quad_energy;
            txb_full_distortion[0][DIST_CALC_PREDICTION] += context_ptr->three_quad_energy;
            //assert(context_ptr->three_quad_energy == 0 && context_ptr->cu_stats->size < 64);
            TxSize tx_size =
                context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr];
            int32_t shift = (MAX_TX_SCALE - av1_get_tx_scale(tx_size)) * 2;
            txb_full_distortion[0][DIST_CALC_RESIDUAL] =
                RIGHT_SIGNED_SHIFT(txb_full_distortion[0][DIST_CALC_RESIDUAL], shift);
            txb_full_distortion[0][DIST_CALC_PREDICTION] =
                RIGHT_SIGNED_SHIFT(txb_full_distortion[0][DIST_CALC_PREDICTION], shift);
        }
#endif
        //LUMA-ONLY
        av1_txb_estimate_coeff_bits(
            context_ptr,
            0, //allow_update_cdf,
            NULL, //FRAME_CONTEXT *ec_ctx,
            pcs_ptr,
            candidate_buffer,
            context_ptr->txb_1d_offset,
            0,
            context_ptr->coeff_est_entropy_coder_ptr,
            candidate_buffer->residual_quant_coeff_ptr,
            y_count_non_zero_coeffs,
            0,
            0,
            &y_txb_coeff_bits,
            &y_txb_coeff_bits,
            &y_txb_coeff_bits,
            context_ptr->blk_geom->txsize[context_ptr->tx_depth][context_ptr->txb_itr],
            context_ptr->blk_geom->txsize_uv[context_ptr->tx_depth][context_ptr->txb_itr],
            tx_type,
            candidate_buffer->candidate_ptr->transform_type_uv,
            COMPONENT_LUMA);

        uint64_t cost = RDCOST(
            context_ptr->full_lambda, y_txb_coeff_bits, txb_full_distortion[0][DIST_CALC_RESIDUAL]);
        if (cost < best_cost_tx_search) {
            best_cost_tx_search = cost;
            best_tx_type        = tx_type;
        }
    }

    //  Best Tx Type Pass
    candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr] = best_tx_type;

    // For Inter blocks, transform type of chroma follows luma transfrom type
    if (is_inter)
        candidate_buffer->candidate_ptr->transform_type_uv =
            (context_ptr->txb_itr == 0)
                ? candidate_buffer->candidate_ptr->transform_type[context_ptr->txb_itr]
                : candidate_buffer->candidate_ptr->transform_type_uv;
}

static INLINE int block_signals_txsize(BlockSize bsize) { return bsize > BLOCK_4X4; }

static INLINE int is_inter_block(const BlockModeInfo *bloc_mi) {
    return is_intrabc_block(bloc_mi) || bloc_mi->ref_frame[0] > INTRA_FRAME;
}

static INLINE int get_vartx_max_txsize(/*const MbModeInfo *xd,*/ BlockSize bsize, int plane) {
    /* if (xd->lossless[xd->mi[0]->segment_id]) return TX_4X4;*/
    const TxSize max_txsize = max_txsize_rect_lookup[bsize];
    if (plane == 0) return max_txsize; // luma
    return av1_get_adjusted_tx_size(max_txsize); // chroma
}

static INLINE int max_block_wide(const MacroBlockD *xd, BlockSize bsize, int plane) {
    int                                   max_blocks_wide = block_size_wide[bsize];
    const struct macroblockd_plane *const pd              = &xd->plane[plane];

    if (xd->mb_to_right_edge < 0)
        max_blocks_wide += xd->mb_to_right_edge >> (3 + pd->subsampling_x);

    // Scale the width in the transform block unit.
    return max_blocks_wide >> tx_size_wide_log2[0];
}

static INLINE int max_block_high(const MacroBlockD *xd, BlockSize bsize, int plane) {
    int                                   max_blocks_high = block_size_high[bsize];
    const struct macroblockd_plane *const pd              = &xd->plane[plane];

    if (xd->mb_to_bottom_edge < 0)
        max_blocks_high += xd->mb_to_bottom_edge >> (3 + pd->subsampling_y);

    // Scale the height in the transform block unit.
    return max_blocks_high >> tx_size_high_log2[0];
}

static INLINE void txfm_partition_update(TXFM_CONTEXT *above_ctx, TXFM_CONTEXT *left_ctx,
                                         TxSize tx_size, TxSize txb_size) {
    BlockSize bsize = txsize_to_bsize[txb_size];
    int       bh    = mi_size_high[bsize];
    int       bw    = mi_size_wide[bsize];
    uint8_t   txw   = tx_size_wide[tx_size];
    uint8_t   txh   = tx_size_high[tx_size];
    int       i;
    for (i = 0; i < bh; ++i) left_ctx[i] = txh;
    for (i = 0; i < bw; ++i) above_ctx[i] = txw;
}

static INLINE TxSize get_sqr_tx_size(int tx_dim) {
    switch (tx_dim) {
    case 128:
    case 64: return TX_64X64; break;
    case 32: return TX_32X32; break;
    case 16: return TX_16X16; break;
    case 8: return TX_8X8; break;
    default: return TX_4X4;
    }
}
static INLINE int txfm_partition_context(TXFM_CONTEXT *above_ctx, TXFM_CONTEXT *left_ctx,
                                         BlockSize bsize, TxSize tx_size) {
    const uint8_t txw      = tx_size_wide[tx_size];
    const uint8_t txh      = tx_size_high[tx_size];
    const int     above    = *above_ctx < txw;
    const int     left     = *left_ctx < txh;
    int           category = TXFM_PARTITION_CONTEXTS;

    // dummy return, not used by others.
    if (tx_size <= TX_4X4) return 0;

    TxSize max_tx_size = get_sqr_tx_size(AOMMAX(block_size_wide[bsize], block_size_high[bsize]));

    if (max_tx_size >= TX_8X8) {
        category = (txsize_sqr_up_map[tx_size] != max_tx_size && max_tx_size > TX_8X8) +
                   (TX_SIZES - 1 - max_tx_size) * 2;
    }
    assert(category != TXFM_PARTITION_CONTEXTS);
    return category * 3 + above + left;
}

static uint64_t cost_tx_size_vartx(MacroBlockD *xd, const MbModeInfo *mbmi, TxSize tx_size,
                                   int depth, int blk_row, int blk_col,
                                   MdRateEstimationContext *md_rate_estimation_ptr) {
    uint64_t  bits            = 0;
    const int max_blocks_high = max_block_high(xd, mbmi->block_mi.sb_type, 0);
    const int max_blocks_wide = max_block_wide(xd, mbmi->block_mi.sb_type, 0);

    if (blk_row >= max_blocks_high || blk_col >= max_blocks_wide) return bits;

    if (depth == MAX_VARTX_DEPTH) {
        txfm_partition_update(
            xd->above_txfm_context + blk_col, xd->left_txfm_context + blk_row, tx_size, tx_size);

        return bits;
    }

    const int ctx = txfm_partition_context(xd->above_txfm_context + blk_col,
                                           xd->left_txfm_context + blk_row,
                                           mbmi->block_mi.sb_type,
                                           tx_size);

    const int write_txfm_partition =
        (tx_size == tx_depth_to_tx_size[mbmi->tx_depth][mbmi->block_mi.sb_type]);

    if (write_txfm_partition) {
        bits += md_rate_estimation_ptr->txfm_partition_fac_bits[ctx][0];

        txfm_partition_update(
            xd->above_txfm_context + blk_col, xd->left_txfm_context + blk_row, tx_size, tx_size);

    } else {
        const TxSize sub_txs = sub_tx_size_map[tx_size];
        const int    bsw     = tx_size_wide_unit[sub_txs];
        const int    bsh     = tx_size_high_unit[sub_txs];

        bits += md_rate_estimation_ptr->txfm_partition_fac_bits[ctx][1];
        if (sub_txs == TX_4X4) {
            txfm_partition_update(xd->above_txfm_context + blk_col,
                                  xd->left_txfm_context + blk_row,
                                  sub_txs,
                                  tx_size);

            return bits;
        }

        assert(bsw > 0 && bsh > 0);
        for (int row = 0; row < tx_size_high_unit[tx_size]; row += bsh)
            for (int col = 0; col < tx_size_wide_unit[tx_size]; col += bsw) {
                int offsetr = blk_row + row;
                int offsetc = blk_col + col;
                bits += cost_tx_size_vartx(
                    xd, mbmi, sub_txs, depth + 1, offsetr, offsetc, md_rate_estimation_ptr);
            }
    }
    return bits;
}

static INLINE void set_txfm_ctx(TXFM_CONTEXT *txfm_ctx, uint8_t txs, int len) {
    int i;
    for (i = 0; i < len; ++i) txfm_ctx[i] = txs;
}

static INLINE void set_txfm_ctxs(TxSize tx_size, int n8_w, int n8_h, int skip,
                                 const MacroBlockD *xd) {
    uint8_t bw = tx_size_wide[tx_size];
    uint8_t bh = tx_size_high[tx_size];

    if (skip) {
        bw = n8_w * MI_SIZE;
        bh = n8_h * MI_SIZE;
    }

    set_txfm_ctx(xd->above_txfm_context, bw, n8_w);
    set_txfm_ctx(xd->left_txfm_context, bh, n8_h);
}

static INLINE int tx_size_to_depth(TxSize tx_size, BlockSize bsize) {
    TxSize ctx_size = max_txsize_rect_lookup[bsize];
    int    depth    = 0;
    while (tx_size != ctx_size) {
        depth++;
        ctx_size = sub_tx_size_map[ctx_size];
        assert(depth <= MAX_TX_DEPTH);
    }
    return depth;
}

#define BLOCK_SIZES_ALL 22

// Returns a context number for the given MB prediction signal
// The mode info data structure has a one element border above and to the
// left of the entries corresponding to real blocks.
// The prediction flags in these dummy entries are initialized to 0.
static INLINE int get_tx_size_context(const MacroBlockD *xd) {
    const ModeInfo *        mi          = xd->mi[0];
    const MbModeInfo *      mbmi        = &mi->mbmi;
    const MbModeInfo *const above_mbmi  = xd->above_mbmi;
    const MbModeInfo *const left_mbmi   = xd->left_mbmi;
    const TxSize            max_tx_size = max_txsize_rect_lookup[mbmi->block_mi.sb_type];
    const int               max_tx_wide = tx_size_wide[max_tx_size];
    const int               max_tx_high = tx_size_high[max_tx_size];
    const int               has_above   = xd->up_available;
    const int               has_left    = xd->left_available;

    int above = xd->above_txfm_context[0] >= max_tx_wide;
    int left  = xd->left_txfm_context[0] >= max_tx_high;

    if (has_above)
        if (is_inter_block(&above_mbmi->block_mi))
            above = block_size_wide[above_mbmi->block_mi.sb_type] >= max_tx_wide;

    if (has_left)
        if (is_inter_block(&left_mbmi->block_mi))
            left = block_size_high[left_mbmi->block_mi.sb_type] >= max_tx_high;

    if (has_above && has_left)
        return (above + left);
    else if (has_above)
        return above;
    else if (has_left)
        return left;
    else
        return 0;
}

static uint64_t cost_selected_tx_size(const MacroBlockD *      xd,
                                      MdRateEstimationContext *md_rate_estimation_ptr) {
    const ModeInfo *const   mi    = xd->mi[0];
    const MbModeInfo *const mbmi  = &mi->mbmi;
    const BlockSize         bsize = mbmi->block_mi.sb_type;
    uint64_t                bits  = 0;

    if (block_signals_txsize(bsize)) {
        const TxSize  tx_size     = mbmi->tx_size;
        const int     tx_size_ctx = get_tx_size_context(xd);
        const int     depth       = tx_size_to_depth(tx_size, bsize);
        const int32_t tx_size_cat = bsize_to_tx_size_cat(bsize);
        bits += md_rate_estimation_ptr->tx_size_fac_bits[tx_size_cat][tx_size_ctx][depth];
    }

    return bits;
}

static uint64_t tx_size_bits(MdRateEstimationContext *md_rate_estimation_ptr, MacroBlockD *xd,
                             const MbModeInfo *mbmi, TxMode tx_mode, BlockSize bsize,
                             uint8_t skip) {
    uint64_t bits = 0;

    int is_inter_tx = is_inter_block(&mbmi->block_mi) || is_intrabc_block(&mbmi->block_mi);
    if (tx_mode == TX_MODE_SELECT && block_signals_txsize(bsize) &&
        !(is_inter_tx && skip) /*&& !xd->lossless[segment_id]*/) {
        if (is_inter_tx) { // This implies skip flag is 0.
            const TxSize max_tx_size = get_vartx_max_txsize(/*xd,*/ bsize, 0);
            const int    txbh        = tx_size_high_unit[max_tx_size];
            const int    txbw        = tx_size_wide_unit[max_tx_size];
            const int    width       = block_size_wide[bsize] >> tx_size_wide_log2[0];
            const int    height      = block_size_high[bsize] >> tx_size_high_log2[0];
            int          idx, idy;
            for (idy = 0; idy < height; idy += txbh)
                for (idx = 0; idx < width; idx += txbw)
                    bits += cost_tx_size_vartx(
                        xd, mbmi, max_tx_size, 0, idy, idx, md_rate_estimation_ptr);
        } else {
            bits += cost_selected_tx_size(xd, md_rate_estimation_ptr);
            set_txfm_ctxs(mbmi->tx_size, xd->n8_w, xd->n8_h, 0, xd);
        }
    } else {
        set_txfm_ctxs(
            mbmi->tx_size, xd->n8_w, xd->n8_h, skip && is_inter_block(&mbmi->block_mi), xd);
    }
    return bits;
}

void set_mi_row_col(PictureControlSet *pcs_ptr, MacroBlockD *xd, TileInfo *tile, int mi_row, int bh,
                    int mi_col, int bw, uint32_t mi_stride, int mi_rows, int mi_cols);

uint64_t estimate_tx_size_bits(PictureControlSet *pcsPtr, ModeDecisionContext *context_ptr,
                               ModeDecisionCandidate *candidate_ptr, EbBool skip_flag,
                               uint32_t blk_origin_x, uint32_t blk_origin_y, BlkStruct *blk_ptr,
                               const BlockGeom *blk_geom, NeighborArrayUnit *txfm_context_array,
                               uint8_t tx_depth, MdRateEstimationContext *md_rate_estimation_ptr) {
    uint32_t txfm_context_left_index =
        get_neighbor_array_unit_left_index(txfm_context_array, blk_origin_y);
    uint32_t txfm_context_above_index =
        get_neighbor_array_unit_top_index(txfm_context_array, blk_origin_x);

    TxMode        tx_mode   = pcsPtr->parent_pcs_ptr->frm_hdr.tx_mode;
    Av1Common *   cm        = pcsPtr->parent_pcs_ptr->av1_cm;
    MacroBlockD * xd        = blk_ptr->av1xd;
    TileInfo *    tile      = &xd->tile;
    int32_t       mi_row    = blk_origin_y >> MI_SIZE_LOG2;
    int32_t       mi_col    = blk_origin_x >> MI_SIZE_LOG2;
    BlockSize     bsize     = blk_geom->bsize;
    const int32_t bw        = mi_size_wide[bsize];
    const int32_t bh        = mi_size_high[bsize];
    uint32_t      mi_stride = pcsPtr->mi_stride;

    set_mi_row_col(pcsPtr, xd, tile, mi_row, bh, mi_col, bw, mi_stride, cm->mi_rows, cm->mi_cols);

    MbModeInfo *mbmi = &xd->mi[0]->mbmi;

    memcpy(context_ptr->above_txfm_context,
           &(txfm_context_array->top_array[txfm_context_above_index]),
           (blk_geom->bwidth >> MI_SIZE_LOG2) * sizeof(TXFM_CONTEXT));
    memcpy(context_ptr->left_txfm_context,
           &(txfm_context_array->left_array[txfm_context_left_index]),
           (blk_geom->bheight >> MI_SIZE_LOG2) * sizeof(TXFM_CONTEXT));

    xd->above_txfm_context = context_ptr->above_txfm_context;
    xd->left_txfm_context  = context_ptr->left_txfm_context;

    mbmi->tx_size               = blk_geom->txsize[tx_depth][0];
    mbmi->block_mi.sb_type      = blk_geom->bsize;
    mbmi->block_mi.use_intrabc  = candidate_ptr->use_intrabc;
    mbmi->block_mi.ref_frame[0] = candidate_ptr->ref_frame_type;
    mbmi->tx_depth              = tx_depth;

    uint64_t bits = tx_size_bits(md_rate_estimation_ptr, xd, mbmi, tx_mode, bsize, skip_flag);

    return bits;
}

uint64_t get_tx_size_bits(ModeDecisionCandidateBuffer *candidateBuffer,
                          ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr,
                          uint8_t tx_depth, EbBool block_has_coeff) {
    uint64_t tx_size_bits = 0;

    tx_size_bits = estimate_tx_size_bits(pcs_ptr,
                                         context_ptr,
                                         candidateBuffer->candidate_ptr,
                                         block_has_coeff ? 0 : 1,
                                         context_ptr->blk_origin_x,
                                         context_ptr->blk_origin_y,
                                         context_ptr->blk_ptr,
                                         context_ptr->blk_geom,
                                         context_ptr->txfm_context_array,
                                         tx_depth,
                                         context_ptr->md_rate_estimation_ptr);

    return tx_size_bits;
}
#if LOSSLESS_TX_TYPE_OPT
void perform_tx_partitioning(ModeDecisionCandidateBuffer *candidate_buffer,
#else
void tx_partitioning_path(ModeDecisionCandidateBuffer *candidate_buffer,
#endif
                             ModeDecisionContext *context_ptr, PictureControlSet *pcs_ptr,
                             uint64_t ref_fast_cost, uint8_t end_tx_depth, uint32_t qp,
                             uint32_t *y_count_non_zero_coeffs, uint64_t *y_coeff_bits,
                             uint64_t *y_full_distortion) {
    EbPictureBufferDesc *input_picture_ptr = context_ptr->hbd_mode_decision
                                                 ? pcs_ptr->input_frame16bit
                                                 : pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    int32_t             is_inter = (candidate_buffer->candidate_ptr->type == INTER_MODE ||
                        candidate_buffer->candidate_ptr->use_intrabc)
                           ? EB_TRUE
                           : EB_FALSE;

    uint8_t  best_tx_depth     = 0;
    uint64_t best_cost_search  = (uint64_t)~0;
    uint8_t  is_best_has_coeff = 1;
    // Fill the scratch buffer
#if LOSSLESS_TX_TYPE_OPT
    if (end_tx_depth) {
#endif
        memcpy(context_ptr->scratch_candidate_buffer->candidate_ptr,
               candidate_buffer->candidate_ptr,
               sizeof(ModeDecisionCandidate));
#if LOSSLESS_TX_TYPE_OPT
    }
#endif
    if (is_inter) {
        uint32_t block_index =
            context_ptr->blk_geom->origin_x + (context_ptr->blk_geom->origin_y * MAX_SB_SIZE);
        if (context_ptr->hbd_mode_decision) {
            // Copy pred
            {
                uint16_t *src =
                    &(((uint16_t *)candidate_buffer->prediction_ptr->buffer_y)[block_index]);
                uint16_t *dst = &(((uint16_t *)context_ptr->scratch_candidate_buffer->prediction_ptr
                                       ->buffer_y)[block_index]);

                for (int i = 0; i < context_ptr->blk_geom->bheight; i++) {
                    memcpy(dst, src, context_ptr->blk_geom->bwidth * sizeof(uint16_t));
                    src += candidate_buffer->prediction_ptr->stride_y;
                    dst += context_ptr->scratch_candidate_buffer->prediction_ptr->stride_y;
                }
            }

            // Copy residual
            {
                int16_t *src =
                    &(((int16_t *)candidate_buffer->residual_ptr->buffer_y)[block_index]);
                int16_t *dst = &(((int16_t *)context_ptr->scratch_candidate_buffer->residual_ptr
                                      ->buffer_y)[block_index]);

                for (int i = 0; i < context_ptr->blk_geom->bheight; i++) {
                    memcpy(dst, src, context_ptr->blk_geom->bwidth << 1);
                    src += candidate_buffer->residual_ptr->stride_y;
                    dst += context_ptr->scratch_candidate_buffer->residual_ptr->stride_y;
                }
            }
        } else

        {
            // Copy pred
            {
                EbByte src = &(candidate_buffer->prediction_ptr->buffer_y[block_index]);
                EbByte dst =
                    &(context_ptr->scratch_candidate_buffer->prediction_ptr->buffer_y[block_index]);
                for (int i = 0; i < context_ptr->blk_geom->bheight; i++) {
                    memcpy(dst, src, context_ptr->blk_geom->bwidth);
                    src += candidate_buffer->prediction_ptr->stride_y;
                    dst += context_ptr->scratch_candidate_buffer->prediction_ptr->stride_y;
                }
            }

            // Copy residual
            {
                int16_t *src =
                    &(((int16_t *)candidate_buffer->residual_ptr->buffer_y)[block_index]);
                int16_t *dst = &(((int16_t *)context_ptr->scratch_candidate_buffer->residual_ptr
                                      ->buffer_y)[block_index]);

                for (int i = 0; i < context_ptr->blk_geom->bheight; i++) {
                    memcpy(dst, src, context_ptr->blk_geom->bwidth << 1);
                    src += candidate_buffer->residual_ptr->stride_y;
                    dst += context_ptr->scratch_candidate_buffer->residual_ptr->stride_y;
                }
            }
        }
    }

    uint8_t tx_search_skip_flag;
    if (context_ptr->md_staging_tx_search == 0)
        tx_search_skip_flag = EB_TRUE;
    else if (context_ptr->md_staging_tx_search == 1)
        tx_search_skip_flag = context_ptr->tx_search_level == TX_SEARCH_FULL_LOOP
                                  ? get_skip_tx_search_flag(context_ptr->blk_geom->sq_size,
                                                            ref_fast_cost,
                                                            *candidate_buffer->fast_cost_ptr,
                                                            context_ptr->tx_weight)
                                  : EB_TRUE;
    else
        tx_search_skip_flag =
            context_ptr->tx_search_level == TX_SEARCH_FULL_LOOP ? EB_FALSE : EB_TRUE;
    tx_reset_neighbor_arrays(pcs_ptr, context_ptr, is_inter, end_tx_depth);

    // Transform Depth Loop
    for (context_ptr->tx_depth = 0; context_ptr->tx_depth <= end_tx_depth;
         context_ptr->tx_depth++) {
        if (pcs_ptr->parent_pcs_ptr->tx_size_early_exit) {
            if (best_tx_depth != 1 && context_ptr->tx_depth == 2) continue;
            if (!is_best_has_coeff) continue;
        }
        ModeDecisionCandidateBuffer *tx_candidate_buffer =
            (context_ptr->tx_depth == 0) ? candidate_buffer : context_ptr->scratch_candidate_buffer;

        tx_candidate_buffer->candidate_ptr->tx_depth = context_ptr->tx_depth;

        tx_initialize_neighbor_arrays(pcs_ptr, context_ptr, is_inter);

        // Initialize TU Split
        uint32_t tx_y_count_non_zero_coeffs[MAX_NUM_OF_TU_PER_CU];
        uint64_t tx_y_coeff_bits                       = 0;
        uint64_t tx_y_full_distortion[DIST_CALC_TOTAL] = {0};

#if LOSSLESS_TX_SIZE_OPT
        // Current tx_cost while performing tx loop
        uint64_t current_tx_cost = 0;
#endif

        context_ptr->txb_1d_offset                      = 0;
        context_ptr->three_quad_energy                  = 0;
        tx_candidate_buffer->candidate_ptr->y_has_coeff = 0;

        uint16_t txb_count = context_ptr->blk_geom->txb_count[context_ptr->tx_depth];

        uint32_t block_has_coeff = EB_FALSE;
        for (context_ptr->txb_itr = 0; context_ptr->txb_itr < txb_count; context_ptr->txb_itr++) {
#if TX_ORG_INTERINTRA
            uint16_t tx_org_x =
                context_ptr->blk_geom
                    ->tx_org_x[is_inter][context_ptr->tx_depth][context_ptr->txb_itr];
            uint16_t tx_org_y =
                context_ptr->blk_geom
                    ->tx_org_y[is_inter][context_ptr->tx_depth][context_ptr->txb_itr];
#else
            uint16_t tx_org_x =
                context_ptr->blk_geom->tx_org_x[context_ptr->tx_depth][context_ptr->txb_itr];
            uint16_t tx_org_y =
                context_ptr->blk_geom->tx_org_y[context_ptr->tx_depth][context_ptr->txb_itr];
#endif
            uint32_t txb_origin_index =
                tx_org_x + (tx_org_y * tx_candidate_buffer->residual_ptr->stride_y);
            uint32_t input_txb_origin_index =
                (context_ptr->sb_origin_x + tx_org_x + input_picture_ptr->origin_x) +
                ((context_ptr->sb_origin_y + tx_org_y + input_picture_ptr->origin_y) *
                 input_picture_ptr->stride_y);

            // Y Prediction

            if (!is_inter) {
#if LOSSLESS_TX_TYPE_OPT
                if (context_ptr->tx_depth)
#endif
                    av1_intra_luma_prediction(context_ptr, pcs_ptr, tx_candidate_buffer);

                // Y Residual
                residual_kernel(
                    input_picture_ptr->buffer_y,
                    input_txb_origin_index,
                    input_picture_ptr->stride_y,
                    tx_candidate_buffer->prediction_ptr->buffer_y,
                    txb_origin_index,
                    tx_candidate_buffer->prediction_ptr->stride_y,
                    (int16_t *)tx_candidate_buffer->residual_ptr->buffer_y,
                    txb_origin_index,
                    tx_candidate_buffer->residual_ptr->stride_y,
                    context_ptr->hbd_mode_decision,
                    context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr],
                    context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr]);
            }

#if LOSSLESS_TX_TYPE_OPT
            if (context_ptr->blk_geom->tx_width[context_ptr->tx_depth][context_ptr->txb_itr] <=
                    32 &&
                context_ptr->blk_geom->tx_height[context_ptr->tx_depth][context_ptr->txb_itr] <= 32)
#endif
                if (!tx_search_skip_flag) {
                    tx_type_search(pcs_ptr, context_ptr, tx_candidate_buffer, qp);
                }

            product_full_loop(tx_candidate_buffer,
                              context_ptr,
                              pcs_ptr,
                              input_picture_ptr,
                              context_ptr->blk_ptr->qp,
                              &(tx_y_count_non_zero_coeffs[0]),
                              &tx_y_coeff_bits,
                              &tx_y_full_distortion[0]);

            uint32_t y_has_coeff = tx_y_count_non_zero_coeffs[context_ptr->txb_itr] > 0;

            tx_update_neighbor_arrays(pcs_ptr, context_ptr, tx_candidate_buffer, is_inter);

            if (y_has_coeff) block_has_coeff = EB_TRUE;

#if LOSSLESS_TX_SIZE_OPT
            current_tx_cost = RDCOST(context_ptr->full_lambda,
                                     tx_y_coeff_bits,
                                     tx_y_full_distortion[DIST_CALC_RESIDUAL]);
            if (current_tx_cost > best_cost_search) break;
#endif

        } // Transform Loop

#if LOSSLESS_TX_TYPE_OPT
        if (end_tx_depth) {
#endif
            uint64_t tx_size_bits = 0;
            if (pcs_ptr->parent_pcs_ptr->frm_hdr.tx_mode == TX_MODE_SELECT)
                tx_size_bits = get_tx_size_bits(tx_candidate_buffer,
                                                context_ptr,
                                                pcs_ptr,
                                                context_ptr->tx_depth,
                                                block_has_coeff);

        uint64_t cost = RDCOST(context_ptr->full_lambda,
                               (tx_y_coeff_bits + tx_size_bits),
                               tx_y_full_distortion[DIST_CALC_RESIDUAL]);

            if (cost < best_cost_search) {
                best_cost_search                      = cost;
                best_tx_depth                         = context_ptr->tx_depth;
                is_best_has_coeff                     = block_has_coeff;
                y_full_distortion[DIST_CALC_RESIDUAL] = tx_y_full_distortion[DIST_CALC_RESIDUAL];
                y_full_distortion[DIST_CALC_PREDICTION] =
                    tx_y_full_distortion[DIST_CALC_PREDICTION];
                *y_coeff_bits = tx_y_coeff_bits;
                for (context_ptr->txb_itr = 0; context_ptr->txb_itr < txb_count;
                     context_ptr->txb_itr++) {
                    y_count_non_zero_coeffs[context_ptr->txb_itr] =
                        tx_y_count_non_zero_coeffs[context_ptr->txb_itr];
                }
            }
#if LOSSLESS_TX_TYPE_OPT
        } else {
            y_full_distortion[DIST_CALC_RESIDUAL]   = tx_y_full_distortion[DIST_CALC_RESIDUAL];
            y_full_distortion[DIST_CALC_PREDICTION] = tx_y_full_distortion[DIST_CALC_PREDICTION];
            *y_coeff_bits                           = tx_y_coeff_bits;
            for (context_ptr->txb_itr = 0; context_ptr->txb_itr < txb_count;
                 context_ptr->txb_itr++) {
                y_count_non_zero_coeffs[context_ptr->txb_itr] =
                    tx_y_count_non_zero_coeffs[context_ptr->txb_itr];
            }
        }
#endif
    } // Transform Depth Loop

    // ATB Recon
    if (best_tx_depth == 1) {
        // Copy depth 1 mode/type/eob ..
        memcpy(candidate_buffer->candidate_ptr,
               context_ptr->scratch_candidate_buffer->candidate_ptr,
               sizeof(ModeDecisionCandidate));

        // Copy depth 1 pred
        uint32_t block_index =
            context_ptr->blk_geom->origin_x + (context_ptr->blk_geom->origin_y * MAX_SB_SIZE);
        if (context_ptr->hbd_mode_decision) {
            // Copy pred
            // &(((int32_t*)context_ptr->trans_quant_buffers_ptr->txb_trans_coeff2_nx2_n_ptr->buffer_y)[txb_1d_offset]),
            uint16_t *src = &(((uint16_t *)context_ptr->scratch_candidate_buffer->prediction_ptr
                                   ->buffer_y)[block_index]);
            uint16_t *dst =
                &(((uint16_t *)candidate_buffer->prediction_ptr->buffer_y)[block_index]);
            for (int i = 0; i < context_ptr->blk_geom->bheight; i++) {
                memcpy(dst, src, context_ptr->blk_geom->bwidth * sizeof(uint16_t));
                src += context_ptr->scratch_candidate_buffer->prediction_ptr->stride_y;
                dst += candidate_buffer->prediction_ptr->stride_y;
            }
        } else

        {
            EbByte src =
                &(context_ptr->scratch_candidate_buffer->prediction_ptr->buffer_y[block_index]);
            EbByte dst = &(candidate_buffer->prediction_ptr->buffer_y[block_index]);
            for (int i = 0; i < context_ptr->blk_geom->bheight; i++) {
                memcpy(dst, src, context_ptr->blk_geom->bwidth);
                src += context_ptr->scratch_candidate_buffer->prediction_ptr->stride_y;
                dst += candidate_buffer->prediction_ptr->stride_y;
            }
        }

        // Copy depth 1 recon coeff
        memcpy(candidate_buffer->recon_coeff_ptr->buffer_y,
               context_ptr->scratch_candidate_buffer->recon_coeff_ptr->buffer_y,
               (context_ptr->blk_geom->bwidth * context_ptr->blk_geom->bheight << 2));
    }
}

void full_loop_core(PictureControlSet *pcs_ptr, SuperBlock *sb_ptr, BlkStruct *blk_ptr,
                    ModeDecisionContext *context_ptr, ModeDecisionCandidateBuffer *candidate_buffer,
                    ModeDecisionCandidate *candidate_ptr, EbPictureBufferDesc *input_picture_ptr,
                    uint32_t input_origin_index, uint32_t input_cb_origin_in_index,
                    uint32_t blk_origin_index, uint32_t blk_chroma_origin_index,
                    uint64_t ref_fast_cost) {
    uint64_t y_full_distortion[DIST_CALC_TOTAL];
    uint32_t count_non_zero_coeffs[3][MAX_NUM_OF_TU_PER_CU];

    uint64_t cb_full_distortion[DIST_CALC_TOTAL];
    uint64_t cr_full_distortion[DIST_CALC_TOTAL];

    uint64_t y_coeff_bits;
    uint64_t cb_coeff_bits = 0;
    uint64_t cr_coeff_bits = 0;

    // initialize TU Split
    y_full_distortion[DIST_CALC_RESIDUAL]   = 0;
    y_full_distortion[DIST_CALC_PREDICTION] = 0;
    y_coeff_bits                            = 0;

    candidate_ptr->full_distortion = 0;

    memset(candidate_ptr->eob[0], 0, sizeof(uint16_t));
    memset(candidate_ptr->eob[1], 0, sizeof(uint16_t));
    memset(candidate_ptr->eob[2], 0, sizeof(uint16_t));

    candidate_ptr->chroma_distortion             = 0;
    candidate_ptr->chroma_distortion_inter_depth = 0;
    // Set Skip Flag
    candidate_ptr->skip_flag = EB_FALSE;

    if (candidate_ptr->type != INTRA_MODE) {
        if (context_ptr->md_staging_skip_full_pred == EB_FALSE) {
            product_prediction_fun_table[candidate_ptr->type](
                context_ptr->hbd_mode_decision, context_ptr, pcs_ptr, candidate_buffer);
        }
    }

    // Initialize luma CBF
    candidate_ptr->y_has_coeff = 0;
    candidate_ptr->u_has_coeff = 0;
    candidate_ptr->v_has_coeff = 0;

    // Initialize tx type
    candidate_ptr->transform_type[0] = DCT_DCT;
    candidate_ptr->transform_type[1] = DCT_DCT;
    candidate_ptr->transform_type[2] = DCT_DCT;
    candidate_ptr->transform_type[3] = DCT_DCT;

#if LOSSLESS_TX_TYPE_OPT
    uint8_t start_tx_depth = 0;
#endif
    uint8_t end_tx_depth = 0;

#if LOSSLESS_TX_TYPE_OPT
    if (context_ptr->md_tx_size_search_mode == 0 || candidate_buffer->candidate_ptr->use_intrabc) {
        start_tx_depth = end_tx_depth = 0;
    } else if (context_ptr->md_staging_tx_size_mode == 0) {
        start_tx_depth = end_tx_depth = candidate_buffer->candidate_ptr->tx_depth;
    } else {
#endif
        // end_tx_depth set to zero for blocks which go beyond the picture boundaries
        if ((context_ptr->sb_origin_x + context_ptr->blk_geom->origin_x +
                     context_ptr->blk_geom->bwidth <
                 pcs_ptr->parent_pcs_ptr->scs_ptr->seq_header.max_frame_width &&
             context_ptr->sb_origin_y + context_ptr->blk_geom->origin_y +
                     context_ptr->blk_geom->bheight <
                 pcs_ptr->parent_pcs_ptr->scs_ptr->seq_header.max_frame_height))
            end_tx_depth = get_end_tx_depth(context_ptr->blk_geom->bsize,
                                            candidate_buffer->candidate_ptr->type);
        else
            end_tx_depth = 0;
#if LOSSLESS_TX_TYPE_OPT
    }
#endif
    // Transform partitioning path (INTRA Luma)
#if !LOSSLESS_TX_TYPE_OPT
    if (context_ptr->md_tx_size_search_mode && context_ptr->md_staging_skip_atb == EB_FALSE &&
        end_tx_depth && candidate_buffer->candidate_ptr->use_intrabc == 0) {
#endif
        int32_t is_inter = (candidate_buffer->candidate_ptr->type == INTER_MODE ||
                            candidate_buffer->candidate_ptr->use_intrabc)
                               ? EB_TRUE
                               : EB_FALSE;

        //Y Residual: residual for INTRA is computed inside the TU loop
        if (is_inter)
            //Y Residual
            residual_kernel(input_picture_ptr->buffer_y,
                            input_origin_index,
                            input_picture_ptr->stride_y,
                            candidate_buffer->prediction_ptr->buffer_y,
                            blk_origin_index,
                            candidate_buffer->prediction_ptr->stride_y,
                            (int16_t *)candidate_buffer->residual_ptr->buffer_y,
                            blk_origin_index,
                            candidate_buffer->residual_ptr->stride_y,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->bwidth,
                            context_ptr->blk_geom->bheight);

#if LOSSLESS_TX_TYPE_OPT
        perform_tx_partitioning(candidate_buffer,
#else
    tx_partitioning_path(candidate_buffer,
#endif
                                context_ptr,
                                pcs_ptr,
                                ref_fast_cost,
                                end_tx_depth,
                                context_ptr->blk_ptr->qp,
                                &(*count_non_zero_coeffs[0]),
                                &y_coeff_bits,
                                &y_full_distortion[0]);
#if !LOSSLESS_TX_TYPE_OPT
    } else {
        // Transform partitioning free patch (except the 128x128 case)

        //Y Residual
        residual_kernel(input_picture_ptr->buffer_y,
                        input_origin_index,
                        input_picture_ptr->stride_y,
                        candidate_buffer->prediction_ptr->buffer_y,
                        blk_origin_index,
                        candidate_buffer->prediction_ptr->stride_y,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_y,
                        blk_origin_index,
                        candidate_buffer->residual_ptr->stride_y,
                        context_ptr->hbd_mode_decision,
                        context_ptr->blk_geom->bwidth,
                        context_ptr->blk_geom->bheight);

        // Transform partitioning free path
        uint8_t tx_search_skip_flag;
        if (context_ptr->md_staging_tx_search == 0)
            tx_search_skip_flag = EB_TRUE;
        else if (context_ptr->md_staging_tx_search == 1)
            tx_search_skip_flag = context_ptr->tx_search_level == TX_SEARCH_FULL_LOOP
                                      ? get_skip_tx_search_flag(context_ptr->blk_geom->sq_size,
                                                                ref_fast_cost,
                                                                *candidate_buffer->fast_cost_ptr,
                                                                context_ptr->tx_weight)
                                      : EB_TRUE;
        else
            tx_search_skip_flag =
                context_ptr->tx_search_level == TX_SEARCH_FULL_LOOP ? EB_FALSE : EB_TRUE;
        if (!tx_search_skip_flag) {
            product_full_loop_tx_search(candidate_buffer, context_ptr, pcs_ptr);
            candidate_ptr->full_distortion = 0;

            memset(candidate_ptr->eob[0], 0, sizeof(uint16_t));

            //re-init
            candidate_ptr->y_has_coeff = 0;
        }
        context_ptr->tx_depth = candidate_buffer->candidate_ptr->tx_depth;
        context_ptr->full_loop_luma_dc_sign_level_coeff_neighbor_array =
            context_ptr->luma_dc_sign_level_coeff_neighbor_array;
        context_ptr->txb_1d_offset = 0;
        for (context_ptr->txb_itr = 0;
             context_ptr->txb_itr < context_ptr->blk_geom->txb_count[context_ptr->tx_depth];
             context_ptr->txb_itr++)
            product_full_loop(candidate_buffer,
                              context_ptr,
                              pcs_ptr,
                              input_picture_ptr,
                              context_ptr->blk_ptr->qp,
                              &(*count_non_zero_coeffs[0]),
                              &y_coeff_bits,
                              &y_full_distortion[0]);
    }
#endif

    candidate_ptr->chroma_distortion_inter_depth = 0;
    candidate_ptr->chroma_distortion             = 0;

    //CHROMA

    cb_full_distortion[DIST_CALC_RESIDUAL]   = 0;
    cr_full_distortion[DIST_CALC_RESIDUAL]   = 0;
    cb_full_distortion[DIST_CALC_PREDICTION] = 0;
    cr_full_distortion[DIST_CALC_PREDICTION] = 0;

    cb_coeff_bits = 0;
    cr_coeff_bits = 0;

    // FullLoop and TU search
    uint16_t cb_qp = context_ptr->qp;
    uint16_t cr_qp = context_ptr->qp;
    if (context_ptr->md_staging_skip_full_chroma == EB_FALSE) {
        if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
            //Cb Residual
            residual_kernel(input_picture_ptr->buffer_cb,
                            input_cb_origin_in_index,
                            input_picture_ptr->stride_cb,
                            candidate_buffer->prediction_ptr->buffer_cb,
                            blk_chroma_origin_index,
                            candidate_buffer->prediction_ptr->stride_cb,
                            (int16_t *)candidate_buffer->residual_ptr->buffer_cb,
                            blk_chroma_origin_index,
                            candidate_buffer->residual_ptr->stride_cb,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv);

            //Cr Residual
            residual_kernel(input_picture_ptr->buffer_cr,
                            input_cb_origin_in_index,
                            input_picture_ptr->stride_cr,
                            candidate_buffer->prediction_ptr->buffer_cr,
                            blk_chroma_origin_index,
                            candidate_buffer->prediction_ptr->stride_cr,
                            (int16_t *)candidate_buffer->residual_ptr->buffer_cr,
                            blk_chroma_origin_index,
                            candidate_buffer->residual_ptr->stride_cr,
                            context_ptr->hbd_mode_decision,
                            context_ptr->blk_geom->bwidth_uv,
                            context_ptr->blk_geom->bheight_uv);
        }

        if (candidate_ptr->type == INTRA_MODE &&
            candidate_buffer->candidate_ptr->intra_chroma_mode == UV_CFL_PRED) {
            // If mode is CFL:
            // 1: recon the Luma
            // 2: Form the pred_buf_q3
            // 3: Loop over alphas and find the best or choose DC
            // 4: Recalculate the residual for chroma
            cfl_prediction(pcs_ptr,
                           candidate_buffer,
                           sb_ptr,
                           context_ptr,
                           input_picture_ptr,
                           input_cb_origin_in_index,
                           blk_chroma_origin_index);
        }
        if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
            full_loop_r(sb_ptr,
                        candidate_buffer,
                        context_ptr,
                        input_picture_ptr,
                        pcs_ptr,
                        PICTURE_BUFFER_DESC_CHROMA_MASK,
                        cb_qp,
                        cr_qp,
                        &(*count_non_zero_coeffs[1]),
                        &(*count_non_zero_coeffs[2]));

            cu_full_distortion_fast_txb_mode_r(sb_ptr,
                                               candidate_buffer,
                                               context_ptr,
                                               candidate_ptr,
                                               pcs_ptr,
                                               input_picture_ptr,
                                               cb_full_distortion,
                                               cr_full_distortion,
                                               count_non_zero_coeffs,
                                               COMPONENT_CHROMA,
                                               &cb_coeff_bits,
                                               &cr_coeff_bits,
                                               1);
        }

        // Check independant chroma vs. cfl
        if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level == CHROMA_MODE_0) {
            if (candidate_buffer->candidate_ptr->type == INTRA_MODE &&
                (candidate_buffer->candidate_ptr->intra_chroma_mode == UV_CFL_PRED ||
                 candidate_buffer->candidate_ptr->intra_chroma_mode == UV_DC_PRED)) {
                check_best_indepedant_cfl(pcs_ptr,
                                          input_picture_ptr,
                                          context_ptr,
                                          input_cb_origin_in_index,
                                          blk_chroma_origin_index,
                                          candidate_buffer,
                                          (uint8_t)cb_qp,
                                          (uint8_t)cr_qp,
                                          cb_full_distortion,
                                          cr_full_distortion,
                                          &cb_coeff_bits,
                                          &cr_coeff_bits);
            }
        }
    }

    candidate_ptr->block_has_coeff =
        (candidate_ptr->y_has_coeff | candidate_ptr->u_has_coeff | candidate_ptr->v_has_coeff)
            ? EB_TRUE
            : EB_FALSE;

    //ALL PLANE
    av1_product_full_cost_func_table[candidate_ptr->type](pcs_ptr,
                                                          context_ptr,
                                                          candidate_buffer,
                                                          blk_ptr,
                                                          y_full_distortion,
                                                          cb_full_distortion,
                                                          cr_full_distortion,
                                                          context_ptr->full_lambda,
                                                          &y_coeff_bits,
                                                          &cb_coeff_bits,
                                                          &cr_coeff_bits,
                                                          context_ptr->blk_geom->bsize);
}
void md_stage_1(PictureControlSet *pcs_ptr, SuperBlock *sb_ptr, BlkStruct *blk_ptr,
                ModeDecisionContext *context_ptr, EbPictureBufferDesc *input_picture_ptr,
                uint32_t input_origin_index, uint32_t input_cb_origin_in_index,
                uint32_t blk_origin_index, uint32_t blk_chroma_origin_index,
                uint64_t ref_fast_cost) {
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
        context_ptr->candidate_buffer_ptr_array;
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
        &(candidate_buffer_ptr_array_base[0]);
    ModeDecisionCandidateBuffer *candidate_buffer;
    ModeDecisionCandidate *      candidate_ptr;

    uint32_t full_loop_candidate_index;
    uint32_t cand_index;

    // Set MD Staging full_loop_core settings
#if LOSSLESS_TX_TYPE_OPT
    context_ptr->md_staging_tx_size_mode = 0;
#else
    context_ptr->md_staging_skip_atb = EB_TRUE;
#endif
    context_ptr->md_staging_tx_search        = 0;
    context_ptr->md_staging_skip_full_chroma = EB_TRUE;
    context_ptr->md_staging_skip_rdoq        = EB_TRUE;
#if FREQUENCY_SPATIAL_DOMAIN
    context_ptr->md_staging_spatial_sse_full_loop = context_ptr->spatial_sse_full_loop;
#endif
    for (full_loop_candidate_index = 0;
         full_loop_candidate_index < context_ptr->md_stage_1_count[context_ptr->target_class];
         ++full_loop_candidate_index) {
        cand_index =
            context_ptr->cand_buff_indices[context_ptr->target_class][full_loop_candidate_index];
        candidate_buffer = candidate_buffer_ptr_array[cand_index];
        candidate_ptr    = candidate_buffer->candidate_ptr;

        context_ptr->md_staging_skip_full_pred            = EB_FALSE;
        context_ptr->md_staging_skip_interpolation_search = EB_FALSE;
        context_ptr->md_staging_skip_inter_chroma_pred    = EB_TRUE;
        candidate_buffer->candidate_ptr->interp_filters   = 0;
        full_loop_core(pcs_ptr,
                       sb_ptr,
                       blk_ptr,
                       context_ptr,
                       candidate_buffer,
                       candidate_ptr,
                       input_picture_ptr,
                       input_origin_index,
                       input_cb_origin_in_index,
                       blk_origin_index,
                       blk_chroma_origin_index,
                       ref_fast_cost);
    }
}
void md_stage_2(PictureControlSet *pcs_ptr, SuperBlock *sb_ptr, BlkStruct *blk_ptr,
                ModeDecisionContext *context_ptr, EbPictureBufferDesc *input_picture_ptr,
                uint32_t input_origin_index, uint32_t input_cb_origin_in_index,
                uint32_t blk_origin_index, uint32_t blk_chroma_origin_index,
                uint32_t fullCandidateTotalCount, uint64_t ref_fast_cost) {
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
        context_ptr->candidate_buffer_ptr_array;
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array =
        &(candidate_buffer_ptr_array_base[0]);
    ModeDecisionCandidateBuffer *candidate_buffer;
    ModeDecisionCandidate *      candidate_ptr;

    uint32_t best_inter_luma_zero_coeff = 1;
    uint64_t best_full_cost             = 0xFFFFFFFFull;
    uint32_t full_loop_candidate_index;
    uint32_t cand_index;

    for (full_loop_candidate_index = 0; full_loop_candidate_index < fullCandidateTotalCount;
         ++full_loop_candidate_index) {
        cand_index = (context_ptr->full_loop_escape == 2)
                         ? context_ptr->sorted_candidate_index_array[full_loop_candidate_index]
                         : context_ptr->best_candidate_index_array[full_loop_candidate_index];
        candidate_buffer = candidate_buffer_ptr_array[cand_index];
        candidate_ptr    = candidate_buffer->candidate_ptr;

        // Set MD Staging full_loop_core settings
        context_ptr->md_staging_skip_full_pred = context_ptr->md_staging_mode == MD_STAGING_MODE_0;
        context_ptr->md_staging_skip_interpolation_search =
            context_ptr->md_staging_mode == MD_STAGING_MODE_1;
        context_ptr->md_staging_skip_inter_chroma_pred = EB_FALSE;
#if LOSSLESS_TX_TYPE_OPT
#if REMOVE_COEFF_BASED_SKIP_ATB
        context_ptr->md_staging_tx_size_mode = EB_TRUE;
#else
        context_ptr->md_staging_tx_size_mode = !context_ptr->coeff_based_skip_atb;
#endif
#else
#if REMOVE_COEFF_BASED_SKIP_ATB
        context_ptr->md_staging_skip_atb = 0;
#else
        context_ptr->md_staging_skip_atb = context_ptr->coeff_based_skip_atb;
#endif
#endif
        context_ptr->md_staging_tx_search =
            (candidate_ptr->cand_class == CAND_CLASS_0 ||
             candidate_ptr->cand_class == CAND_CLASS_6 || candidate_ptr->cand_class == CAND_CLASS_7)
                ? 2
                : 1;
        context_ptr->md_staging_skip_full_chroma = EB_FALSE;

        context_ptr->md_staging_skip_rdoq = EB_FALSE;
#if FREQUENCY_SPATIAL_DOMAIN
        context_ptr->md_staging_spatial_sse_full_loop = context_ptr->spatial_sse_full_loop;
#endif

        if (pcs_ptr->slice_type != I_SLICE) {
            if ((candidate_ptr->type == INTRA_MODE || context_ptr->full_loop_escape == 2) &&
                best_inter_luma_zero_coeff == 0) {
                context_ptr->md_stage_2_total_count = full_loop_candidate_index;
                return;
            }
        }

        full_loop_core(pcs_ptr,
                       sb_ptr,
                       blk_ptr,
                       context_ptr,
                       candidate_buffer,
                       candidate_ptr,
                       input_picture_ptr,
                       input_origin_index,
                       input_cb_origin_in_index,
                       blk_origin_index,
                       blk_chroma_origin_index,
                       ref_fast_cost);

        if (context_ptr->full_loop_escape) {
            if (pcs_ptr->slice_type != I_SLICE) {
                if (candidate_ptr->type == INTER_MODE) {
                    if (*candidate_buffer->full_cost_ptr < best_full_cost) {
                        best_inter_luma_zero_coeff = candidate_ptr->y_has_coeff;
                        best_full_cost             = *candidate_buffer->full_cost_ptr;
                    }
                }
            }
        }
    }
}

void move_blk_data(PictureControlSet *pcs, EncDecContext *context_ptr, BlkStruct *src_cu,
                   BlkStruct *dst_cu) {
    memcpy(&dst_cu->palette_info.pmi, &src_cu->palette_info.pmi, sizeof(PaletteModeInfo));
    if (svt_av1_allow_palette(pcs->parent_pcs_ptr->palette_mode, context_ptr->blk_geom->bsize)) {
        dst_cu->palette_info.color_idx_map = (uint8_t *)malloc(MAX_PALETTE_SQUARE);
        assert(dst_cu->palette_info.color_idx_map != NULL && "palette:Not-Enough-Memory");
        if (dst_cu->palette_info.color_idx_map != NULL)
            memcpy(dst_cu->palette_info.color_idx_map,
                   src_cu->palette_info.color_idx_map,
                   MAX_PALETTE_SQUARE);
        else
            SVT_LOG("ERROR palette:Not-Enough-Memory\n");
    }
    dst_cu->interp_filters              = src_cu->interp_filters;
    dst_cu->interinter_comp.type        = src_cu->interinter_comp.type;
    dst_cu->interinter_comp.mask_type   = src_cu->interinter_comp.mask_type;
    dst_cu->interinter_comp.wedge_index = src_cu->interinter_comp.wedge_index;
    dst_cu->interinter_comp.wedge_sign  = src_cu->interinter_comp.wedge_sign;
    dst_cu->compound_idx                = src_cu->compound_idx;
    dst_cu->comp_group_idx              = src_cu->comp_group_idx;

    dst_cu->is_interintra_used     = src_cu->is_interintra_used;
    dst_cu->interintra_mode        = src_cu->interintra_mode;
    dst_cu->use_wedge_interintra   = src_cu->use_wedge_interintra;
    dst_cu->interintra_wedge_index = src_cu->interintra_wedge_index; //inter_intra wedge index
    dst_cu->ii_wedge_sign          = src_cu->ii_wedge_sign; //inter_intra wedge sign=-1
    //CHKN TransformUnit             txb_array[TRANSFORM_UNIT_MAX_COUNT]; // 2-bytes * 21 = 42-bytes
    memcpy(dst_cu->txb_array, src_cu->txb_array, TRANSFORM_UNIT_MAX_COUNT * sizeof(TransformUnit));

    //CHKN PredictionUnit            prediction_unit_array[MAX_NUM_OF_PU_PER_CU];    // 35-bytes * 4 = 140 bytes
    memcpy(dst_cu->prediction_unit_array,
           src_cu->prediction_unit_array,
           MAX_NUM_OF_PU_PER_CU * sizeof(PredictionUnit));

    dst_cu->skip_flag_context    = src_cu->skip_flag_context;
    dst_cu->prediction_mode_flag = src_cu->prediction_mode_flag;
    dst_cu->block_has_coeff      = src_cu->block_has_coeff;
    dst_cu->split_flag_context   = src_cu->split_flag_context;
    dst_cu->qp                   = src_cu->qp;
    dst_cu->delta_qp             = src_cu->delta_qp;
    dst_cu->tx_depth             = src_cu->tx_depth;
    dst_cu->leaf_index           = src_cu->leaf_index;
    dst_cu->split_flag           = src_cu->split_flag;
    dst_cu->skip_flag            = src_cu->skip_flag;

    //CHKN    MacroBlockD*  av1xd;
    memcpy(dst_cu->av1xd, src_cu->av1xd, sizeof(MacroBlockD));

    // uint8_t ref_mv_count[MODE_CTX_REF_FRAMES];

    //CHKN int16_t inter_mode_ctx[MODE_CTX_REF_FRAMES];
    memcpy(dst_cu->inter_mode_ctx, src_cu->inter_mode_ctx, MODE_CTX_REF_FRAMES * sizeof(int16_t));

    //CHKN IntMv ref_mvs[MODE_CTX_REF_FRAMES][MAX_MV_REF_CANDIDATES]; //used only for nonCompound modes.
    memcpy(dst_cu->ref_mvs,
           src_cu->ref_mvs,
           MODE_CTX_REF_FRAMES * MAX_MV_REF_CANDIDATES * sizeof(IntMv));

    //CHKN uint8_t  drl_index;
    //CHKN PredictionMode               pred_mode;
    dst_cu->drl_index = src_cu->drl_index;
    dst_cu->pred_mode = src_cu->pred_mode;

    //CHKN IntMv  predmv[2];

    memcpy(dst_cu->predmv, src_cu->predmv, 2 * sizeof(IntMv));
    //CHKN uint8_t                         skip_coeff_context;
    //CHKN int16_t                        luma_txb_skip_context;
    //CHKN int16_t                        luma_dc_sign_context;
    //CHKN int16_t                        cb_txb_skip_context;
    //CHKN int16_t                        cb_dc_sign_context;
    //CHKN int16_t                        cr_txb_skip_context;
    //CHKN int16_t                        cr_dc_sign_context;
    //CHKN uint8_t                         reference_mode_context;
    //CHKN uint8_t                         compoud_reference_type_context;
    //CHKN uint32_t                        partitionContext;

    dst_cu->skip_coeff_context             = src_cu->skip_coeff_context;
    dst_cu->reference_mode_context         = src_cu->reference_mode_context;
    dst_cu->compoud_reference_type_context = src_cu->compoud_reference_type_context;
    dst_cu->segment_id                     = src_cu->segment_id;

    memcpy(dst_cu->quantized_dc, src_cu->quantized_dc, 3 * MAX_TXB_COUNT * sizeof(int32_t));
    //CHKN uint32_t   is_inter_ctx;
    //CHKN uint32_t                     interp_filters;

    dst_cu->is_inter_ctx   = src_cu->is_inter_ctx;
    dst_cu->interp_filters = src_cu->interp_filters;

    dst_cu->part              = src_cu->part;
    dst_cu->shape             = src_cu->shape;
    dst_cu->mds_idx           = src_cu->mds_idx;
    dst_cu->filter_intra_mode = src_cu->filter_intra_mode;
}
void move_blk_data_redund(PictureControlSet *pcs, ModeDecisionContext *context_ptr,
                          BlkStruct *src_cu, BlkStruct *dst_cu) {
    dst_cu->segment_id       = src_cu->segment_id;
    dst_cu->seg_id_predicted = src_cu->seg_id_predicted;
    dst_cu->ref_qp           = src_cu->ref_qp;

    memcpy(&dst_cu->palette_info.pmi, &src_cu->palette_info.pmi, sizeof(PaletteModeInfo));
    if (svt_av1_allow_palette(pcs->parent_pcs_ptr->palette_mode, context_ptr->blk_geom->bsize))
        memcpy(dst_cu->palette_info.color_idx_map,
               src_cu->palette_info.color_idx_map,
               MAX_PALETTE_SQUARE);
    dst_cu->interp_filters              = src_cu->interp_filters;
    dst_cu->interinter_comp.type        = src_cu->interinter_comp.type;
    dst_cu->interinter_comp.mask_type   = src_cu->interinter_comp.mask_type;
    dst_cu->interinter_comp.wedge_index = src_cu->interinter_comp.wedge_index;
    dst_cu->interinter_comp.wedge_sign  = src_cu->interinter_comp.wedge_sign;
    dst_cu->compound_idx                = src_cu->compound_idx;
    dst_cu->comp_group_idx              = src_cu->comp_group_idx;
    dst_cu->is_interintra_used          = src_cu->is_interintra_used;
    dst_cu->interintra_mode             = src_cu->interintra_mode;
    dst_cu->use_wedge_interintra        = src_cu->use_wedge_interintra;
    dst_cu->interintra_wedge_index      = src_cu->interintra_wedge_index; //inter_intra wedge index
    dst_cu->ii_wedge_sign               = src_cu->ii_wedge_sign; //inter_intra wedge sign=-1
    dst_cu->filter_intra_mode           = src_cu->filter_intra_mode;
    //CHKN TransformUnit_t             txb_array[TRANSFORM_UNIT_MAX_COUNT]; // 2-bytes * 21 = 42-bytes
    memcpy(dst_cu->txb_array, src_cu->txb_array, TRANSFORM_UNIT_MAX_COUNT * sizeof(TransformUnit));

    //CHKN PredictionUnit_t            prediction_unit_array[MAX_NUM_OF_PU_PER_CU];    // 35-bytes * 4 = 140 bytes
    memcpy(dst_cu->prediction_unit_array,
           src_cu->prediction_unit_array,
           MAX_NUM_OF_PU_PER_CU * sizeof(PredictionUnit));
    dst_cu->skip_flag_context    = src_cu->skip_flag_context;
    dst_cu->prediction_mode_flag = src_cu->prediction_mode_flag;
    dst_cu->block_has_coeff      = src_cu->block_has_coeff;
    dst_cu->split_flag_context   = src_cu->split_flag_context;
    dst_cu->qp                   = src_cu->qp;
    dst_cu->delta_qp             = src_cu->delta_qp;
    dst_cu->leaf_index           = src_cu->leaf_index;
    dst_cu->skip_flag            = src_cu->skip_flag;
    dst_cu->tx_depth             = src_cu->tx_depth;
    //CHKN    MacroBlockD*  av1xd;
    memcpy(dst_cu->av1xd, src_cu->av1xd, sizeof(MacroBlockD));

    // uint8_t ref_mv_count[MODE_CTX_REF_FRAMES];

    //CHKN int16_t inter_mode_ctx[MODE_CTX_REF_FRAMES];
    memcpy(dst_cu->inter_mode_ctx, src_cu->inter_mode_ctx, MODE_CTX_REF_FRAMES * sizeof(int16_t));

    //CHKN IntMv ref_mvs[MODE_CTX_REF_FRAMES][MAX_MV_REF_CANDIDATES]; //used only for nonCompound modes.
    memcpy(dst_cu->ref_mvs,
           src_cu->ref_mvs,
           MODE_CTX_REF_FRAMES * MAX_MV_REF_CANDIDATES * sizeof(IntMv));

    //CHKN uint8_t  drl_index;
    //CHKN PredictionMode               pred_mode;
    dst_cu->drl_index = src_cu->drl_index;
    dst_cu->pred_mode = src_cu->pred_mode;

    //CHKN IntMv  predmv[2];

    memcpy(dst_cu->predmv, src_cu->predmv, 2 * sizeof(IntMv));

    //CHKN uint8_t                         skip_coeff_context;
    //CHKN int16_t                        luma_txb_skip_context;
    //CHKN int16_t                        luma_dc_sign_context;
    //CHKN int16_t                        cb_txb_skip_context;
    //CHKN int16_t                        cb_dc_sign_context;
    //CHKN int16_t                        cr_txb_skip_context;
    //CHKN int16_t                        cr_dc_sign_context;
    //CHKN uint8_t                         reference_mode_context;
    //CHKN uint8_t                         compoud_reference_type_context;
    //CHKN uint32_t                        partitionContext;

    dst_cu->skip_coeff_context             = src_cu->skip_coeff_context;
    dst_cu->reference_mode_context         = src_cu->reference_mode_context;
    dst_cu->compoud_reference_type_context = src_cu->compoud_reference_type_context;
    memcpy(dst_cu->quantized_dc, src_cu->quantized_dc, 3 * MAX_TXB_COUNT * sizeof(int32_t));
    //CHKN uint32_t   is_inter_ctx;
    //CHKN uint32_t                     interp_filters;

    dst_cu->is_inter_ctx   = src_cu->is_inter_ctx;
    dst_cu->interp_filters = src_cu->interp_filters;

    dst_cu->part  = src_cu->part;
    dst_cu->shape = src_cu->shape;
    //dst_cu->mds_idx = src_cu->mds_idx;
}

void check_redundant_block(const BlockGeom *blk_geom, ModeDecisionContext *context_ptr,
                           uint8_t *redundant_blk_avail, uint16_t *redundant_blk_mds) {
    if (blk_geom->redund) {
        for (int it = 0; it < blk_geom->redund_list.list_size; it++) {
            if (context_ptr->md_local_blk_unit[blk_geom->redund_list.blk_mds_table[it]]
                    .avail_blk_flag) {
                *redundant_blk_mds   = blk_geom->redund_list.blk_mds_table[it];
                *redundant_blk_avail = 1;
                break;
            }
        }
    }
}

/*******************************************
* ModeDecision SB
*   performs CL (SB)
*******************************************/
EbBool allowed_ns_cu(EbBool is_nsq_table_used, uint8_t nsq_max_shapes_md,
                     ModeDecisionContext *context_ptr, uint8_t is_complete_sb) {
    EbBool ret = 1;
    UNUSED(is_complete_sb);
    if (is_nsq_table_used) {
        if (context_ptr->blk_geom->shape != PART_N) {
            ret = 0;
            for (int i = 0; i < nsq_max_shapes_md; i++) {
                if (context_ptr->blk_geom->shape == context_ptr->nsq_table[i]) ret = 1;
            }
        }
    }

    return ret;
}

/****************************************************
* generate the the size in pixel for partition code
****************************************************/
uint8_t get_part_side(PartitionContextType part) {
    switch (part) {
    case 31: return 4; break;
    case 30: return 8; break;
    case 28: return 16; break;
    case 24: return 32; break;
    case 16: return 64; break;
    case 0: return 128; break;
    default:
        return 255;
        SVT_LOG("error: non supported partition!!\n");
        break;
    }
}
/****************************************************
* Return a predicted Shape based on the above and
* left partitions
****************************************************/
Part get_partition_shape(PartitionContextType above, PartitionContextType left, uint8_t width,
                         uint8_t height) {
    uint8_t above_size = get_part_side(above);
    uint8_t left_size  = get_part_side(left);
    Part    part       = PART_N;

    if (above_size == width && left_size == height)
        part = PART_N;
    else if (above_size > width && left_size > height)
        part = PART_N;
    else if (above_size > width) {
        if (left_size == height)
            part = PART_N;
        else if (left_size < (height / 2))
            part = PART_H4;
        else if (left_size < height)
            part = PART_H;
        else
            SVT_LOG("error: unsupported left_size\n");
    } else if (left_size > height) {
        if (above_size == width)
            part = PART_N;
        else if (above_size < (width / 2))
            part = PART_V4;
        else if (above_size < width)
            part = PART_V;
        else
            SVT_LOG("error: unsupported above_size\n");
    } else if (above_size < width) {
        if (left_size == height)
            part = PART_VA;
        else if (left_size < height)
            part = PART_S;
        else
            SVT_LOG("error: unsupported left_size\n");
    } else if (left_size < height) {
        if (above_size == width)
            part = PART_HA;
        else if (above_size < width)
            part = PART_S;
        else
            SVT_LOG("error: unsupported above_size\n");
    } else if (above_size == width) {
        if (left_size < height)
            part = PART_HB;
        else
            SVT_LOG("error: unsupported left_size\n");
    } else if (left_size == height) {
        if (above_size == width)
            part = PART_HB;
        else
            SVT_LOG("error: unsupported above_size\n");
    } else
        SVT_LOG("error: unsupported above_size && left_size\n");
    return part;
};
/****************************************************
* Reorder the nsq_table in order to keep the most
* probable Shape to be selected in the lowest index
****************************************************/
void order_nsq_table(PictureControlSet *pcs_ptr, ModeDecisionContext *context_ptr,
                     const SequenceControlSet *scs_ptr, SuperBlock *sb_ptr,
                     NeighborArrayUnit *leaf_partition_neighbor_array) {
    FrameHeader *  frm_hdr             = &pcs_ptr->parent_pcs_ptr->frm_hdr;
    const uint32_t sb_addr             = sb_ptr->index;
    EbBool         is_compound_enabled = (frm_hdr->reference_mode == SINGLE_REFERENCE) ? 0 : 1;
    uint32_t       me_sb_addr;
    uint32_t       me_2nx2n_table_offset;
    uint32_t       max_number_of_pus_per_sb;
    uint32_t       geom_offset_x   = 0;
    uint32_t       geom_offset_y   = 0;
    uint8_t        cnt[PART_S + 1] = {0};
    if (scs_ptr->seq_header.sb_size == BLOCK_128X128) {
        uint32_t me_sb_size = scs_ptr->sb_sz;
        uint32_t me_pic_width_in_sb =
            (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) / me_sb_size;
        uint32_t me_sb_x = (context_ptr->blk_origin_x / me_sb_size);
        uint32_t me_sb_y = (context_ptr->blk_origin_y / me_sb_size);
        me_sb_addr       = me_sb_x + me_sb_y * me_pic_width_in_sb;
        geom_offset_x    = (me_sb_x & 0x1) * me_sb_size;
        geom_offset_y    = (me_sb_y & 0x1) * me_sb_size;
    } else
        me_sb_addr = sb_addr;
    max_number_of_pus_per_sb = pcs_ptr->parent_pcs_ptr->max_number_of_pus_per_sb;
    me_2nx2n_table_offset =
        (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 ||
         context_ptr->blk_geom->bwidth == 128 || context_ptr->blk_geom->bheight == 128)
            ? 0
            :

            get_me_info_index(
                max_number_of_pus_per_sb, context_ptr->blk_geom, geom_offset_x, geom_offset_y);

    const MeSbResults *me_results = pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr];
    uint8_t            nsq0       = me_results->me_nsq_0[me_2nx2n_table_offset];
    uint8_t            nsq1       = me_results->me_nsq_1[me_2nx2n_table_offset];
    uint8_t            me_part_0 =
        nsq0 == 0
            ? PART_N
            : nsq0 == 1
                  ? PART_H
                  : nsq0 == 2 ? PART_V
                              : nsq0 == 3 ? PART_H4 : nsq0 == 4 ? PART_V4 : nsq0 == 5 ? PART_S : 0;
    uint8_t me_part_1 =
        nsq1 == 0
            ? PART_N
            : nsq1 == 1
                  ? PART_H
                  : nsq1 == 2 ? PART_V
                              : nsq1 == 3 ? PART_H4 : nsq1 == 4 ? PART_V4 : nsq1 == 5 ? PART_S : 0;

    // Generate Partition context
    uint32_t partition_left_neighbor_index = get_neighbor_array_unit_left_index(
        leaf_partition_neighbor_array, context_ptr->blk_origin_y);
    uint32_t partition_above_neighbor_index =
        get_neighbor_array_unit_top_index(leaf_partition_neighbor_array, context_ptr->blk_origin_x);
    const PartitionContextType above_ctx =
        (((PartitionContext *)
              leaf_partition_neighbor_array->top_array)[partition_above_neighbor_index]
             .above == (int8_t)INVALID_NEIGHBOR_DATA)
            ? 0
            : ((PartitionContext *)
                   leaf_partition_neighbor_array->top_array)[partition_above_neighbor_index]
                  .above;
    const PartitionContextType left_ctx =
        (((PartitionContext *)
              leaf_partition_neighbor_array->left_array)[partition_left_neighbor_index]
             .left == (int8_t)INVALID_NEIGHBOR_DATA)
            ? 0
            : ((PartitionContext *)
                   leaf_partition_neighbor_array->left_array)[partition_left_neighbor_index]
                  .left;

    Part neighbor_part = get_partition_shape(
        above_ctx, left_ctx, context_ptr->blk_geom->bwidth, context_ptr->blk_geom->bheight);

    //init table
    context_ptr->nsq_table[0] = PART_H;
    context_ptr->nsq_table[1] = PART_V;
    context_ptr->nsq_table[2] = PART_HA;
    context_ptr->nsq_table[3] = PART_HB;
    context_ptr->nsq_table[4] = PART_VA;
    context_ptr->nsq_table[5] = PART_VB;
    context_ptr->nsq_table[6] = PART_H4;
    context_ptr->nsq_table[7] = PART_V4;

    if (is_compound_enabled == 0) me_part_1 = me_part_0;

    // Insert predicted Shapes based on ME information
    if (me_part_0 != me_part_1) {
        context_ptr->nsq_table[0] = me_part_0;
        context_ptr->nsq_table[1] = me_part_1;

        if (me_part_0 == PART_H) {
            context_ptr->nsq_table[2] = PART_HA;
            context_ptr->nsq_table[3] = PART_HB;
            context_ptr->nsq_table[4] = me_part_1 != PART_H4 ? PART_H4 : PART_V;
        } else if (me_part_0 == PART_V) {
            context_ptr->nsq_table[2] = PART_VA;
            context_ptr->nsq_table[3] = PART_VB;
            context_ptr->nsq_table[4] = me_part_1 != PART_V4 ? PART_V4 : PART_H;
        } else if (me_part_0 == PART_H4) {
            context_ptr->nsq_table[2] = PART_HA;
            context_ptr->nsq_table[3] = PART_HB;
            context_ptr->nsq_table[4] = me_part_1 != PART_H ? PART_H : PART_V;
        } else if (me_part_0 == PART_V4) {
            context_ptr->nsq_table[2] = PART_VA;
            context_ptr->nsq_table[3] = PART_VB;
            context_ptr->nsq_table[4] = me_part_1 != PART_V ? PART_V : PART_H;
        } else if (me_part_0 == PART_S) {
            context_ptr->nsq_table[2] = PART_VA;
            context_ptr->nsq_table[3] = PART_HB;
            context_ptr->nsq_table[4] = me_part_1 != PART_V ? PART_V : PART_H;
        }
    } else {
        context_ptr->nsq_table[0] = me_part_0;
        if (me_part_0 == PART_H) {
            context_ptr->nsq_table[1] = PART_HA;
            context_ptr->nsq_table[2] = PART_HB;
            context_ptr->nsq_table[3] = PART_H4;
            context_ptr->nsq_table[4] = PART_V;
        } else if (me_part_0 == PART_V) {
            context_ptr->nsq_table[1] = PART_VA;
            context_ptr->nsq_table[2] = PART_VB;
            context_ptr->nsq_table[3] = PART_V4;
            context_ptr->nsq_table[4] = PART_H;
        } else if (me_part_0 == PART_H4) {
            context_ptr->nsq_table[1] = PART_H;
            context_ptr->nsq_table[2] = PART_HA;
            context_ptr->nsq_table[3] = PART_HB;
            context_ptr->nsq_table[4] = PART_V;
        } else if (me_part_0 == PART_V4) {
            context_ptr->nsq_table[1] = PART_V;
            context_ptr->nsq_table[2] = PART_VA;
            context_ptr->nsq_table[3] = PART_VB;
            context_ptr->nsq_table[4] = PART_H;
        } else if (me_part_0 == PART_S) {
            context_ptr->nsq_table[1] = PART_HA;
            context_ptr->nsq_table[2] = PART_VA;
            context_ptr->nsq_table[3] = PART_HB;
            context_ptr->nsq_table[4] = PART_VB;
        }
    }

    // Insert predicted Shapes based on neighbor information
    if (neighbor_part == PART_S && me_part_0 == PART_S && me_part_1 == PART_S) {
        context_ptr->nsq_table[0] = PART_HA;
        context_ptr->nsq_table[1] = PART_VA;
        context_ptr->nsq_table[2] = PART_HB;
        context_ptr->nsq_table[3] = PART_VB;
        context_ptr->nsq_table[4] = PART_H4;
        context_ptr->nsq_table[5] = PART_V4;
    } else {
        if (neighbor_part != PART_N && neighbor_part != PART_S && neighbor_part != me_part_0 &&
            neighbor_part != me_part_1) {
            context_ptr->nsq_table[5] = context_ptr->nsq_table[4];
            context_ptr->nsq_table[4] = context_ptr->nsq_table[3];
            context_ptr->nsq_table[3] = context_ptr->nsq_table[2];
            context_ptr->nsq_table[2] = context_ptr->nsq_table[1];
            context_ptr->nsq_table[1] = context_ptr->nsq_table[0];
            context_ptr->nsq_table[0] = neighbor_part;
        } else
            context_ptr->nsq_table[5] =
                neighbor_part != PART_N && neighbor_part != PART_S ? neighbor_part : me_part_0;
    }

    // Remove duplicate candidates
    for (int pidx = 0; pidx < NSQ_TAB_SIZE; pidx++) cnt[context_ptr->nsq_table[pidx]]++;
    cnt[context_ptr->nsq_table[0]] = 1;
    for (int iter = 0; iter < NSQ_TAB_SIZE - 1; iter++) {
        for (int idx = 1 + iter; idx < NSQ_TAB_SIZE; idx++) {
            if (context_ptr->nsq_table[iter] != context_ptr->nsq_table[idx])
                continue;
            else {
                for (int i = idx; i < NSQ_TAB_SIZE; i++) {
                    if (idx < NSQ_TAB_SIZE - 1)
                        context_ptr->nsq_table[idx] = context_ptr->nsq_table[idx + 1];
                    else if (idx == NSQ_TAB_SIZE - 1) {
                        for (int pidx = 1; pidx < PART_S; pidx++) {
                            if (cnt[pidx] == 0) context_ptr->nsq_table[idx] = (Part)pidx;
                        }
                    }
                }
            }
        }
    }
}
void search_best_independent_uv_mode(PictureControlSet *  pcs_ptr,
                                     EbPictureBufferDesc *input_picture_ptr,
                                     uint32_t             input_cb_origin_in_index,
                                     uint32_t             input_cr_origin_in_index,
                                     uint32_t             cu_chroma_origin_index,
                                     ModeDecisionContext *context_ptr) {
    FrameHeader *frm_hdr = &pcs_ptr->parent_pcs_ptr->frm_hdr;
    // Start uv search path
    context_ptr->uv_search_path = EB_TRUE;

    EbBool use_angle_delta = av1_use_angle_delta(context_ptr->blk_geom->bsize);

    UvPredictionMode uv_mode;

    int coeff_rate[UV_PAETH_PRED + 1][(MAX_ANGLE_DELTA << 1) + 1];
    int distortion[UV_PAETH_PRED + 1][(MAX_ANGLE_DELTA << 1) + 1];

    ModeDecisionCandidate *candidate_array     = context_ptr->fast_candidate_array;
    uint8_t                uv_mode_total_count = 0;
    for (uv_mode = UV_DC_PRED; uv_mode <= UV_PAETH_PRED; uv_mode++) {
        uint8_t uv_angle_delta_candidate_count =
            (use_angle_delta && av1_is_directional_mode((PredictionMode)uv_mode)) ? 7 : 1;
        uint8_t uv_angle_delta_shift = 1;

        for (uint8_t uv_angle_delta_counter = 0;
             uv_angle_delta_counter < uv_angle_delta_candidate_count;
             ++uv_angle_delta_counter) {
            int32_t uv_angle_delta =
                CLIP(uv_angle_delta_shift *
                         (uv_angle_delta_candidate_count == 1
                              ? 0
                              : uv_angle_delta_counter - (uv_angle_delta_candidate_count >> 1)),
                     -MAX_ANGLE_DELTA,
                     MAX_ANGLE_DELTA);

            candidate_array[uv_mode_total_count].type                       = INTRA_MODE;
            candidate_array[uv_mode_total_count].distortion_ready           = 0;
            candidate_array[uv_mode_total_count].use_intrabc                = 0;
            candidate_array[uv_mode_total_count].angle_delta[PLANE_TYPE_UV] = 0;
            candidate_array[uv_mode_total_count].pred_mode                  = DC_PRED;
            candidate_array[uv_mode_total_count].intra_chroma_mode          = uv_mode;
            candidate_array[uv_mode_total_count].is_directional_chroma_mode_flag =
                (uint8_t)av1_is_directional_mode((PredictionMode)uv_mode);
            candidate_array[uv_mode_total_count].angle_delta[PLANE_TYPE_UV]       = uv_angle_delta;
            candidate_array[uv_mode_total_count].tx_depth                         = 0;
            candidate_array[uv_mode_total_count].palette_info.pmi.palette_size[0] = 0;
            candidate_array[uv_mode_total_count].palette_info.pmi.palette_size[1] = 0;
            candidate_array[uv_mode_total_count].filter_intra_mode = FILTER_INTRA_MODES;
            candidate_array[uv_mode_total_count].cfl_alpha_signs   = 0;
            candidate_array[uv_mode_total_count].cfl_alpha_idx     = 0;
            candidate_array[uv_mode_total_count].transform_type[0] = DCT_DCT;
            candidate_array[uv_mode_total_count].ref_frame_type    = INTRA_FRAME;
            candidate_array[uv_mode_total_count].motion_mode       = SIMPLE_TRANSLATION;

            candidate_array[uv_mode_total_count].transform_type_uv =
                av1_get_tx_type(context_ptr->blk_geom->bsize,
                                0,
                                (PredictionMode)NULL,
                                (UvPredictionMode)uv_mode,
                                PLANE_TYPE_UV,
                                0,
                                0,
                                0,
                                context_ptr->blk_geom->txsize_uv[0][0],
                                frm_hdr->reduced_tx_set);

            uv_mode_total_count++;
        }
    }
    // Fast-loop search uv_mode
    for (uint8_t uv_mode_count = 0; uv_mode_count < uv_mode_total_count; uv_mode_count++) {
        ModeDecisionCandidateBuffer *candidate_buffer =
            context_ptr->candidate_buffer_ptr_array[uv_mode_count];
        candidate_buffer->candidate_ptr = &context_ptr->fast_candidate_array[uv_mode_count];

        context_ptr->md_staging_skip_inter_chroma_pred = EB_FALSE;
        product_prediction_fun_table[candidate_buffer->candidate_ptr->type](
            context_ptr->hbd_mode_decision, context_ptr, pcs_ptr, candidate_buffer);

        uint32_t chroma_fast_distortion = 0;
        if (!context_ptr->hbd_mode_decision) {
            chroma_fast_distortion = nxm_sad_kernel_sub_sampled(
                input_picture_ptr->buffer_cb + input_cb_origin_in_index,
                input_picture_ptr->stride_cb,
                candidate_buffer->prediction_ptr->buffer_cb + cu_chroma_origin_index,
                candidate_buffer->prediction_ptr->stride_cb,
                context_ptr->blk_geom->bheight_uv,
                context_ptr->blk_geom->bwidth_uv);

            chroma_fast_distortion += nxm_sad_kernel_sub_sampled(
                input_picture_ptr->buffer_cr + input_cr_origin_in_index,
                input_picture_ptr->stride_cr,
                candidate_buffer->prediction_ptr->buffer_cr + cu_chroma_origin_index,
                candidate_buffer->prediction_ptr->stride_cr,
                context_ptr->blk_geom->bheight_uv,
                context_ptr->blk_geom->bwidth_uv);
        } else {
            chroma_fast_distortion = sad_16b_kernel(
                ((uint16_t *)input_picture_ptr->buffer_cb) + input_cb_origin_in_index,
                input_picture_ptr->stride_cb,
                ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cb) + cu_chroma_origin_index,
                candidate_buffer->prediction_ptr->stride_cb,
                context_ptr->blk_geom->bheight_uv,
                context_ptr->blk_geom->bwidth_uv);

            chroma_fast_distortion += sad_16b_kernel(
                ((uint16_t *)input_picture_ptr->buffer_cr) + input_cr_origin_in_index,
                input_picture_ptr->stride_cr,
                ((uint16_t *)candidate_buffer->prediction_ptr->buffer_cr) + cu_chroma_origin_index,
                candidate_buffer->prediction_ptr->stride_cr,
                context_ptr->blk_geom->bheight_uv,
                context_ptr->blk_geom->bwidth_uv);
        }
        // Do not consider rate @ this stage
        *(candidate_buffer->fast_cost_ptr) = chroma_fast_distortion;
    }

    // Sort uv_mode (in terms of distortion only)
    uint32_t uv_cand_buff_indices[MAX_NFL_BUFF];
    memset(uv_cand_buff_indices, 0xFFFFFFFF, MAX_NFL_BUFF * sizeof(uint32_t));
    sort_fast_candidates(
        context_ptr,
        0,
        uv_mode_total_count, //how many cand buffers to sort. one of the buffers can have max cost.
        uv_cand_buff_indices);

    // Reset *(candidate_buffer->fast_cost_ptr)
    for (uint8_t uv_mode_count = 0; uv_mode_count < uv_mode_total_count; uv_mode_count++) {
        ModeDecisionCandidateBuffer *candidate_buffer =
            context_ptr->candidate_buffer_ptr_array[uv_mode_count];
        *(candidate_buffer->fast_cost_ptr) = MAX_CU_COST;
    }

    // Derive uv_mode_nfl_count
    uint8_t uv_mode_nfl_count;
    if (pcs_ptr->temporal_layer_index == 0)
        uv_mode_nfl_count = uv_mode_total_count;
    else if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag)
        uv_mode_nfl_count = 16;
    else
        uv_mode_nfl_count = 8;

    // Full-loop search uv_mode
    for (uint8_t uv_mode_count = 0; uv_mode_count < MIN(uv_mode_total_count, uv_mode_nfl_count);
         uv_mode_count++) {
        ModeDecisionCandidateBuffer *candidate_buffer =
            context_ptr->candidate_buffer_ptr_array[uv_cand_buff_indices[uv_mode_count]];
        candidate_buffer->candidate_ptr =
            &context_ptr->fast_candidate_array[uv_cand_buff_indices[uv_mode_count]];
        uint16_t cb_qp                               = context_ptr->qp;
        uint16_t cr_qp                               = context_ptr->qp;
        uint64_t cb_coeff_bits                       = 0;
        uint64_t cr_coeff_bits                       = 0;
        uint64_t cb_full_distortion[DIST_CALC_TOTAL] = {0, 0};
        uint64_t cr_full_distortion[DIST_CALC_TOTAL] = {0, 0};

        uint32_t count_non_zero_coeffs[3][MAX_NUM_OF_TU_PER_CU];

        //Cb Residual
        residual_kernel(input_picture_ptr->buffer_cb,
                        input_cb_origin_in_index,
                        input_picture_ptr->stride_cb,
                        candidate_buffer->prediction_ptr->buffer_cb,
                        cu_chroma_origin_index,
                        candidate_buffer->prediction_ptr->stride_cb,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_cb,
                        cu_chroma_origin_index,
                        candidate_buffer->residual_ptr->stride_cb,
                        context_ptr->hbd_mode_decision,
                        context_ptr->blk_geom->bwidth_uv,
                        context_ptr->blk_geom->bheight_uv);

        //Cr Residual
        residual_kernel(input_picture_ptr->buffer_cr,
                        input_cr_origin_in_index,
                        input_picture_ptr->stride_cr,
                        candidate_buffer->prediction_ptr->buffer_cr,
                        cu_chroma_origin_index,
                        candidate_buffer->prediction_ptr->stride_cr,
                        (int16_t *)candidate_buffer->residual_ptr->buffer_cr,
                        cu_chroma_origin_index,
                        candidate_buffer->residual_ptr->stride_cr,
                        context_ptr->hbd_mode_decision,
                        context_ptr->blk_geom->bwidth_uv,
                        context_ptr->blk_geom->bheight_uv);

        full_loop_r(context_ptr->sb_ptr,
                    candidate_buffer,
                    context_ptr,
                    input_picture_ptr,
                    pcs_ptr,
                    PICTURE_BUFFER_DESC_CHROMA_MASK,
                    cb_qp,
                    cr_qp,
                    &(*count_non_zero_coeffs[1]),
                    &(*count_non_zero_coeffs[2]));

        cu_full_distortion_fast_txb_mode_r(context_ptr->sb_ptr,
                                           candidate_buffer,
                                           context_ptr,
                                           candidate_buffer->candidate_ptr,
                                           pcs_ptr,
                                           input_picture_ptr,
                                           cb_full_distortion,
                                           cr_full_distortion,
                                           count_non_zero_coeffs,
                                           COMPONENT_CHROMA,
                                           &cb_coeff_bits,
                                           &cr_coeff_bits,
                                           1);

        coeff_rate[candidate_buffer->candidate_ptr->intra_chroma_mode]
                  [MAX_ANGLE_DELTA + candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_UV]] =
                      (int)(cb_coeff_bits + cr_coeff_bits);
        distortion[candidate_buffer->candidate_ptr->intra_chroma_mode]
                  [MAX_ANGLE_DELTA + candidate_buffer->candidate_ptr->angle_delta[PLANE_TYPE_UV]] =
                      (int)(cb_full_distortion[DIST_CALC_RESIDUAL] +
                            cr_full_distortion[DIST_CALC_RESIDUAL]);
    }

    // Loop over all intra mode, then over all uv move to derive the best uv mode for a given intra mode in term of rate

    // intra_mode loop (luma mode loop)
    for (uint8_t intra_mode = DC_PRED; intra_mode <= PAETH_PRED; ++intra_mode) {
        uint8_t angle_delta_candidate_count =
            (use_angle_delta && av1_is_directional_mode((PredictionMode)intra_mode)) ? 7 : 1;
        uint8_t angle_delta_shift = 1;

        for (uint8_t angle_delta_counter = 0; angle_delta_counter < angle_delta_candidate_count;
             ++angle_delta_counter) {
            int32_t angle_delta =
                CLIP(angle_delta_shift *
                         (angle_delta_candidate_count == 1
                              ? 0
                              : angle_delta_counter - (angle_delta_candidate_count >> 1)),
                     -MAX_ANGLE_DELTA,
                     MAX_ANGLE_DELTA);

            // uv mode loop
            context_ptr->best_uv_cost[intra_mode][MAX_ANGLE_DELTA + angle_delta] = (uint64_t)~0;

            for (uint8_t uv_mode_count = 0;
                 uv_mode_count < MIN(uv_mode_total_count, uv_mode_nfl_count);
                 uv_mode_count++) {
                ModeDecisionCandidate *candidate_ptr =
                    &(context_ptr->fast_candidate_array[uv_cand_buff_indices[uv_mode_count]]);

                candidate_ptr->intra_luma_mode = intra_mode;
                candidate_ptr->is_directional_mode_flag =
                    (uint8_t)av1_is_directional_mode((PredictionMode)intra_mode);
                candidate_ptr->angle_delta[PLANE_TYPE_Y] = angle_delta;
                candidate_ptr->pred_mode                 = (PredictionMode)intra_mode;

                // Fast Cost
                av1_product_fast_cost_func_table[candidate_ptr->type](
                    context_ptr->blk_ptr,
                    candidate_ptr,
                    context_ptr->qp,
                    0,
                    0,
                    0,
                    0,
                    pcs_ptr,
                    &(context_ptr->md_local_blk_unit[context_ptr->blk_geom->blkidx_mds]
                          .ed_ref_mv_stack[candidate_ptr->ref_frame_type][0]),
                    context_ptr->blk_geom,
                    context_ptr->blk_origin_y >> MI_SIZE_LOG2,
                    context_ptr->blk_origin_x >> MI_SIZE_LOG2,
                    context_ptr->md_enable_inter_intra,
                    context_ptr->full_cost_shut_fast_rate_flag,
                    1,
                    context_ptr->intra_luma_left_mode,
                    context_ptr->intra_luma_top_mode);

                uint64_t rate =
                    coeff_rate[candidate_ptr->intra_chroma_mode]
                              [MAX_ANGLE_DELTA + candidate_ptr->angle_delta[PLANE_TYPE_UV]] +
                    candidate_ptr->fast_luma_rate + candidate_ptr->fast_chroma_rate;
                uint64_t uv_cost =
                    RDCOST(context_ptr->full_lambda,
                           rate,
                           distortion[candidate_ptr->intra_chroma_mode]
                                     [MAX_ANGLE_DELTA + candidate_ptr->angle_delta[PLANE_TYPE_UV]]);

                if (uv_cost <
                    context_ptr->best_uv_cost[intra_mode][MAX_ANGLE_DELTA + angle_delta]) {
                    context_ptr->best_uv_mode[intra_mode][MAX_ANGLE_DELTA + angle_delta] =
                        candidate_ptr->intra_chroma_mode;
                    context_ptr->best_uv_angle[intra_mode][MAX_ANGLE_DELTA + angle_delta] =
                        candidate_ptr->angle_delta[PLANE_TYPE_UV];

                    context_ptr->best_uv_cost[intra_mode][MAX_ANGLE_DELTA + angle_delta] = uv_cost;
                    context_ptr->fast_luma_rate[intra_mode][MAX_ANGLE_DELTA + angle_delta] =
                        candidate_ptr->fast_luma_rate;
                    context_ptr->fast_chroma_rate[intra_mode][MAX_ANGLE_DELTA + angle_delta] =
                        candidate_ptr->fast_chroma_rate;
                }
            }
        }
    }

    // End uv search path
    context_ptr->uv_search_path = EB_FALSE;
}
extern AomVarianceFnPtr mefn_ptr[BlockSizeS_ALL];
unsigned int                 eb_av1_get_sby_perpixel_variance(const AomVarianceFnPtr *fn_ptr,
                                                              const uint8_t *src, int stride, BlockSize bs);

void interintra_class_pruning_1(ModeDecisionContext *context_ptr, uint64_t best_md_stage_cost) {
    for (CandClass cand_class_it = CAND_CLASS_0; cand_class_it < CAND_CLASS_TOTAL;
         cand_class_it++) {
        if (context_ptr->md_stage_1_cand_prune_th != (uint64_t)~0 ||
            context_ptr->md_stage_1_class_prune_th != (uint64_t)~0)
            if (context_ptr->md_stage_0_count[cand_class_it] > 0 &&
                context_ptr->md_stage_1_count[cand_class_it] > 0) {
                uint32_t *cand_buff_indices = context_ptr->cand_buff_indices[cand_class_it];
                uint64_t  class_best_cost =
                    *(context_ptr->candidate_buffer_ptr_array[cand_buff_indices[0]]->fast_cost_ptr);

                // inter class pruning
                if (best_md_stage_cost && class_best_cost &&
                    ((((class_best_cost - best_md_stage_cost) * 100) / best_md_stage_cost) >
                     context_ptr->md_stage_1_class_prune_th)) {
                    context_ptr->md_stage_1_count[cand_class_it] = 0;
                    continue;
                }
                // intra class pruning
                uint32_t cand_count = 1;
                if (class_best_cost)
                    while (
                        cand_count < context_ptr->md_stage_1_count[cand_class_it] &&
                        ((((*(context_ptr->candidate_buffer_ptr_array[cand_buff_indices[cand_count]]
                                  ->fast_cost_ptr) -
                            class_best_cost) *
                           100) /
                          class_best_cost) < context_ptr->md_stage_1_cand_prune_th)) {
                        cand_count++;
                    }
                context_ptr->md_stage_1_count[cand_class_it] = cand_count;
            }
        context_ptr->md_stage_1_total_count += context_ptr->md_stage_1_count[cand_class_it];
    }
}

void interintra_class_pruning_2(ModeDecisionContext *context_ptr, uint64_t best_md_stage_cost) {
    for (CandClass cand_class_it = CAND_CLASS_0; cand_class_it < CAND_CLASS_TOTAL;
         cand_class_it++) {
        if (context_ptr->md_stage_2_cand_prune_th != (uint64_t)~0 ||
            context_ptr->md_stage_2_class_prune_th != (uint64_t)~0)
            if (context_ptr->md_stage_1_count[cand_class_it] > 0 &&
                context_ptr->md_stage_2_count[cand_class_it] > 0 &&
                context_ptr->bypass_md_stage_1[cand_class_it] == EB_FALSE) {
                uint32_t *cand_buff_indices = context_ptr->cand_buff_indices[cand_class_it];
                uint64_t  class_best_cost =
                    *(context_ptr->candidate_buffer_ptr_array[cand_buff_indices[0]]->full_cost_ptr);

                // inter class pruning
                if (best_md_stage_cost && class_best_cost &&
                    ((((class_best_cost - best_md_stage_cost) * 100) / best_md_stage_cost) >
                     context_ptr->md_stage_2_class_prune_th)) {
                    context_ptr->md_stage_2_count[cand_class_it] = 0;
                    continue;
                }

                // intra class pruning
                uint32_t cand_count = 1;
                if (class_best_cost)
                    while (
                        cand_count < context_ptr->md_stage_2_count[cand_class_it] &&
                        ((((*(context_ptr->candidate_buffer_ptr_array[cand_buff_indices[cand_count]]
                                  ->full_cost_ptr) -
                            class_best_cost) *
                           100) /
                          class_best_cost) < context_ptr->md_stage_2_cand_prune_th)) {
                        cand_count++;
                    }
                context_ptr->md_stage_2_count[cand_class_it] = cand_count;
            }
        context_ptr->md_stage_2_total_count += context_ptr->md_stage_2_count[cand_class_it];
    }
}

void md_encode_block(SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                     ModeDecisionContext *context_ptr, EbPictureBufferDesc *input_picture_ptr,
                     uint32_t sb_addr,
                     ModeDecisionCandidateBuffer *bestcandidate_buffers[5]) {
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array_base =
        context_ptr->candidate_buffer_ptr_array;
    ModeDecisionCandidateBuffer **candidate_buffer_ptr_array;
    const BlockGeom *             blk_geom = context_ptr->blk_geom;
    ModeDecisionCandidateBuffer * candidate_buffer;
    ModeDecisionCandidate *       fast_candidate_array = context_ptr->fast_candidate_array;
    uint32_t                      candidate_index;
    uint32_t                      fast_candidate_total_count;
    uint32_t                      best_intra_mode = EB_INTRA_MODE_INVALID;
    const uint32_t                input_origin_index =
        (context_ptr->blk_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y +
        (context_ptr->blk_origin_x + input_picture_ptr->origin_x);

    const uint32_t input_cb_origin_in_index =
        ((context_ptr->round_origin_y >> 1) + (input_picture_ptr->origin_y >> 1)) *
            input_picture_ptr->stride_cb +
        ((context_ptr->round_origin_x >> 1) + (input_picture_ptr->origin_x >> 1));
    const uint32_t blk_origin_index = blk_geom->origin_x + blk_geom->origin_y * SB_STRIDE_Y;
    const uint32_t blk_chroma_origin_index =
        ROUND_UV(blk_geom->origin_x) / 2 + ROUND_UV(blk_geom->origin_y) / 2 * SB_STRIDE_UV;
    BlkStruct *blk_ptr        = context_ptr->blk_ptr;
    candidate_buffer_ptr_array = &(candidate_buffer_ptr_array_base[0]);
    for (uint8_t ref_idx = 0; ref_idx < MAX_REF_TYPE_CAND; ref_idx++)
        context_ptr->ref_best_cost_sq_table[ref_idx] = MAX_CU_COST;
    EbBool is_nsq_table_used = (pcs_ptr->slice_type == !I_SLICE &&
                                pcs_ptr->parent_pcs_ptr->pic_depth_mode <= PIC_ALL_C_DEPTH_MODE &&
                                pcs_ptr->parent_pcs_ptr->nsq_search_level >= NSQ_SEARCH_LEVEL1 &&
                                pcs_ptr->parent_pcs_ptr->nsq_search_level < NSQ_SEARCH_FULL)
                                   ? EB_TRUE
                                   : EB_FALSE;
    is_nsq_table_used = (pcs_ptr->enc_mode == ENC_M0 ||
                         pcs_ptr->parent_pcs_ptr->pic_depth_mode == PIC_MULTI_PASS_PD_MODE_0 ||
                         pcs_ptr->parent_pcs_ptr->pic_depth_mode == PIC_MULTI_PASS_PD_MODE_1 ||
                         pcs_ptr->parent_pcs_ptr->pic_depth_mode == PIC_MULTI_PASS_PD_MODE_2 ||
                         pcs_ptr->parent_pcs_ptr->pic_depth_mode == PIC_MULTI_PASS_PD_MODE_3)
                            ? EB_FALSE
                            : is_nsq_table_used;

    if (is_nsq_table_used) {
        if (context_ptr->blk_geom->shape == PART_N) {
            order_nsq_table(pcs_ptr,
                            context_ptr,
                            scs_ptr,
                            context_ptr->sb_ptr,
                            context_ptr->leaf_partition_neighbor_array);
        }
    }

    uint8_t is_complete_sb = pcs_ptr->parent_pcs_ptr->sb_geom[sb_addr].is_complete_sb;

    if (allowed_ns_cu(is_nsq_table_used,
                      pcs_ptr->parent_pcs_ptr->nsq_max_shapes_md,
                      context_ptr,
                      is_complete_sb)) {
        const AomVarianceFnPtr *fn_ptr = &mefn_ptr[context_ptr->blk_geom->bsize];
        context_ptr->source_variance =
            eb_av1_get_sby_perpixel_variance(fn_ptr,
                                             (input_picture_ptr->buffer_y + input_origin_index),
                                             input_picture_ptr->stride_y,
                                             context_ptr->blk_geom->bsize);
        blk_ptr->av1xd->tile.mi_col_start = context_ptr->sb_ptr->tile_info.mi_col_start;
        blk_ptr->av1xd->tile.mi_col_end   = context_ptr->sb_ptr->tile_info.mi_col_end;
        blk_ptr->av1xd->tile.mi_row_start = context_ptr->sb_ptr->tile_info.mi_row_start;
        blk_ptr->av1xd->tile.mi_row_end   = context_ptr->sb_ptr->tile_info.mi_row_end;

        product_coding_loop_init_fast_loop(context_ptr,
                                           context_ptr->skip_coeff_neighbor_array,
                                           context_ptr->inter_pred_dir_neighbor_array,
                                           context_ptr->ref_frame_type_neighbor_array,
                                           context_ptr->intra_luma_mode_neighbor_array,
                                           context_ptr->skip_flag_neighbor_array,
                                           context_ptr->mode_type_neighbor_array,
                                           context_ptr->leaf_depth_neighbor_array,
                                           context_ptr->leaf_partition_neighbor_array);

        // Initialize uv_search_path
        context_ptr->uv_search_path = EB_FALSE;
        // Search the best independent intra chroma mode
        if (context_ptr->chroma_level == CHROMA_MODE_0) {
            if (context_ptr->blk_geom->sq_size < 128) {
                if (context_ptr->blk_geom->has_uv) {
                    search_best_independent_uv_mode(pcs_ptr,
                                                    input_picture_ptr,
                                                    input_cb_origin_in_index,
                                                    input_cb_origin_in_index,
                                                    blk_chroma_origin_index,
                                                    context_ptr);
                }
            }
        }

        FrameHeader *frm_hdr       = &pcs_ptr->parent_pcs_ptr->frm_hdr;
        context_ptr->geom_offset_x = 0;
        context_ptr->geom_offset_y = 0;

        if (scs_ptr->seq_header.sb_size == BLOCK_128X128) {
            uint32_t me_sb_size = scs_ptr->sb_sz;
            uint32_t me_pic_width_in_sb =
                (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) / me_sb_size;
            uint32_t me_sb_x           = (context_ptr->blk_origin_x / me_sb_size);
            uint32_t me_sb_y           = (context_ptr->blk_origin_y / me_sb_size);
            context_ptr->me_sb_addr    = me_sb_x + me_sb_y * me_pic_width_in_sb;
            context_ptr->geom_offset_x = (me_sb_x & 0x1) * me_sb_size;
            context_ptr->geom_offset_y = (me_sb_y & 0x1) * me_sb_size;
        } else
            context_ptr->me_sb_addr = sb_addr;

        context_ptr->me_block_offset =
            (context_ptr->blk_geom->bwidth == 4 || context_ptr->blk_geom->bheight == 4 ||
             context_ptr->blk_geom->bwidth == 128 || context_ptr->blk_geom->bheight == 128)
                ? 0
                : get_me_info_index(pcs_ptr->parent_pcs_ptr->max_number_of_pus_per_sb,
                                    context_ptr->blk_geom,
                                    context_ptr->geom_offset_x,
                                    context_ptr->geom_offset_y);

        // Generate MVP(s)
        if (!context_ptr->md_skip_mvp_generation) {
            if (frm_hdr->allow_intrabc) // pcs_ptr->slice_type == I_SLICE
                generate_av1_mvp_table(&context_ptr->sb_ptr->tile_info,
                                       context_ptr,
                                       context_ptr->blk_ptr,
                                       context_ptr->blk_geom,
                                       context_ptr->blk_origin_x,
                                       context_ptr->blk_origin_y,
                                       pcs_ptr->parent_pcs_ptr->ref_frame_type_arr,
                                       1,
                                       pcs_ptr);
            else if (pcs_ptr->slice_type != I_SLICE)
                generate_av1_mvp_table(&context_ptr->sb_ptr->tile_info,
                                       context_ptr,
                                       context_ptr->blk_ptr,
                                       context_ptr->blk_geom,
                                       context_ptr->blk_origin_x,
                                       context_ptr->blk_origin_y,
                                       pcs_ptr->parent_pcs_ptr->ref_frame_type_arr,
                                       pcs_ptr->parent_pcs_ptr->tot_ref_frame_types,
                                       pcs_ptr);
        } else {
            mvp_bypass_init(pcs_ptr, context_ptr);
        }
        // Perform ME search around the best MVP
        if (context_ptr->predictive_me_level)
            predictive_me_search(
                pcs_ptr, context_ptr, input_picture_ptr, input_origin_index, blk_origin_index);
        //for every CU, perform Luma DC/V/H/S intra prediction to be used later in inter-intra search

        int allow_ii = is_interintra_allowed_bsize(context_ptr->blk_geom->bsize);
        if (context_ptr->md_enable_inter_intra && allow_ii)
            precompute_intra_pred_for_inter_intra(pcs_ptr, context_ptr);

        generate_md_stage_0_cand(
            context_ptr->sb_ptr, context_ptr, &fast_candidate_total_count, pcs_ptr);

        //MD Stages
        //The first stage(old fast loop) and the last stage(old full loop) should remain at their locations, new stages could be created between those two.
        //a bypass mechanism should be added to skip one or all of the intermediate stages, in a way to to be able to fall back to org design (FastLoop->FullLoop)
        set_md_stage_counts(pcs_ptr, context_ptr, fast_candidate_total_count);

        CandClass cand_class_it;
        uint32_t  buffer_start_idx = 0;
        uint32_t  buffer_count_for_curr_class;
        uint32_t  buffer_total_count        = 0;
        context_ptr->md_stage_1_total_count = 0;
        context_ptr->md_stage_2_total_count = 0;
        uint64_t best_md_stage_cost         = (uint64_t)~0;
        for (cand_class_it = CAND_CLASS_0; cand_class_it < CAND_CLASS_TOTAL; cand_class_it++) {
            //number of next level candidates could not exceed number of curr level candidates
            context_ptr->md_stage_1_count[cand_class_it] =
                MIN(context_ptr->md_stage_0_count[cand_class_it],
                    context_ptr->md_stage_1_count[cand_class_it]);

            if (context_ptr->md_stage_0_count[cand_class_it] > 0 &&
                context_ptr->md_stage_1_count[cand_class_it] > 0) {
                buffer_count_for_curr_class =
                    context_ptr->md_stage_0_count[cand_class_it] >
                            context_ptr->md_stage_1_count[cand_class_it]
                        ? (context_ptr->md_stage_1_count[cand_class_it] + 1)
                        : context_ptr->md_stage_1_count[cand_class_it];

                buffer_total_count += buffer_count_for_curr_class;
                assert(buffer_total_count <= MAX_NFL_BUFF && "not enough cand buffers");

                //Input: md_stage_0_count[cand_class_it]  Output:  md_stage_1_count[cand_class_it]
                context_ptr->target_class = cand_class_it;

                md_stage_0(pcs_ptr,
                           context_ptr,
                           candidate_buffer_ptr_array_base,
                           fast_candidate_array,
                           0,
                           fast_candidate_total_count - 1,
                           input_picture_ptr,
                           input_origin_index,
                           input_cb_origin_in_index,
                           input_cb_origin_in_index,
                           blk_ptr,
                           blk_origin_index,
                           blk_chroma_origin_index,
                           buffer_start_idx,
                           buffer_count_for_curr_class,
                           context_ptr->md_stage_0_count[cand_class_it] >
                               context_ptr->md_stage_1_count
                                   [cand_class_it]); //is there need to max the temp buffer

                //Sort:  md_stage_1_count[cand_class_it]
                memset(context_ptr->cand_buff_indices[cand_class_it],
                       0xFFFFFFFF,
                       MAX_NFL_BUFF * sizeof(uint32_t));
                sort_fast_candidates(
                    context_ptr,
                    buffer_start_idx,
                    buffer_count_for_curr_class, //how many cand buffers to sort. one of the buffers can have max cost.
                    context_ptr->cand_buff_indices[cand_class_it]);
                uint32_t *cand_buff_indices = context_ptr->cand_buff_indices[cand_class_it];
                best_md_stage_cost =
                    MIN((*(context_ptr->candidate_buffer_ptr_array[cand_buff_indices[0]]
                               ->fast_cost_ptr)),
                        best_md_stage_cost);

                buffer_start_idx += buffer_count_for_curr_class; //for next iteration.
            }
        }
        interintra_class_pruning_1(context_ptr, best_md_stage_cost);
        memset(
            context_ptr->best_candidate_index_array, 0xFFFFFFFF, MAX_NFL_BUFF * sizeof(uint32_t));
        memset(context_ptr->sorted_candidate_index_array, 0xFFFFFFFF, MAX_NFL * sizeof(uint32_t));

        uint64_t ref_fast_cost = MAX_MODE_COST;
        construct_best_sorted_arrays_md_stage_1(context_ptr,
                                                candidate_buffer_ptr_array,
                                                context_ptr->best_candidate_index_array,
                                                context_ptr->sorted_candidate_index_array,
                                                &ref_fast_cost);

        // 1st Full-Loop
        best_md_stage_cost = (uint64_t)~0;
        for (cand_class_it = CAND_CLASS_0; cand_class_it < CAND_CLASS_TOTAL; cand_class_it++) {
            //number of next level candidates could not exceed number of curr level candidates
            context_ptr->md_stage_2_count[cand_class_it] =
                MIN(context_ptr->md_stage_1_count[cand_class_it],
                    context_ptr->md_stage_2_count[cand_class_it]);
            if (context_ptr->bypass_md_stage_1[cand_class_it] == EB_FALSE &&
                context_ptr->md_stage_1_count[cand_class_it] > 0 &&
                context_ptr->md_stage_2_count[cand_class_it] > 0) {
                context_ptr->target_class = cand_class_it;
                md_stage_1(pcs_ptr,
                           context_ptr->sb_ptr,
                           blk_ptr,
                           context_ptr,
                           input_picture_ptr,
                           input_origin_index,
                           input_cb_origin_in_index,
                           blk_origin_index,
                           blk_chroma_origin_index,
                           ref_fast_cost);

                // Sort the candidates of the target class based on the 1st full loop cost

                //sort the new set of candidates
                if (context_ptr->md_stage_1_count[cand_class_it])
                    sort_stage1_candidates(context_ptr,
                                           context_ptr->md_stage_1_count[cand_class_it],
                                           context_ptr->cand_buff_indices[cand_class_it]);
                uint32_t *cand_buff_indices = context_ptr->cand_buff_indices[cand_class_it];
                best_md_stage_cost =
                    MIN((*(context_ptr->candidate_buffer_ptr_array[cand_buff_indices[0]]
                               ->full_cost_ptr)),
                        best_md_stage_cost);
            }
        }
        interintra_class_pruning_2(context_ptr, best_md_stage_cost);
        assert(context_ptr->md_stage_2_total_count <= MAX_NFL);
        assert(context_ptr->md_stage_2_total_count > 0);
        construct_best_sorted_arrays_md_stage_2(context_ptr,
                                                candidate_buffer_ptr_array,
                                                context_ptr->best_candidate_index_array,
                                                context_ptr->sorted_candidate_index_array);

        // 2nd Full-Loop
        md_stage_2(pcs_ptr,
                   context_ptr->sb_ptr,
                   blk_ptr,
                   context_ptr,
                   input_picture_ptr,
                   input_origin_index,
                   input_cb_origin_in_index,
                   blk_origin_index,
                   blk_chroma_origin_index,
                   context_ptr->md_stage_2_total_count,
                   ref_fast_cost); // fullCandidateTotalCount to number of buffers to process

        // Full Mode Decision (choose the best mode)
        candidate_index = product_full_mode_decision(
            context_ptr,
            blk_ptr,
            candidate_buffer_ptr_array,
            context_ptr->md_stage_2_total_count,
            (context_ptr->full_loop_escape == 2) ? context_ptr->sorted_candidate_index_array
                                                 : context_ptr->best_candidate_index_array,
            context_ptr->prune_ref_frame_for_rec_partitions,
            &best_intra_mode);
        candidate_buffer = candidate_buffer_ptr_array[candidate_index];

        bestcandidate_buffers[0] = candidate_buffer;
        uint8_t sq_index         = LOG2F(context_ptr->blk_geom->sq_size) - 2;
        if (context_ptr->blk_geom->shape == PART_N) {
            context_ptr->parent_sq_type[sq_index] = candidate_buffer->candidate_ptr->type;

            context_ptr->parent_sq_has_coeff[sq_index] =
                (candidate_buffer->candidate_ptr->y_has_coeff ||
                 candidate_buffer->candidate_ptr->u_has_coeff ||
                 candidate_buffer->candidate_ptr->v_has_coeff)
                    ? 1
                    : 0;

            context_ptr->parent_sq_pred_mode[sq_index] = candidate_buffer->candidate_ptr->pred_mode;
        }

        av1_perform_inverse_transform_recon(
            pcs_ptr, context_ptr, candidate_buffer, blk_ptr, context_ptr->blk_geom);

        if (!context_ptr->blk_geom->has_uv) {
            // Store the luma data for 4x* and *x4 blocks to be used for CFL
            EbPictureBufferDesc *recon_ptr       = candidate_buffer->recon_ptr;
            uint32_t             rec_luma_offset = context_ptr->blk_geom->origin_x +
                                       context_ptr->blk_geom->origin_y * recon_ptr->stride_y;
            if (context_ptr->hbd_mode_decision) {
                for (uint32_t j = 0; j < context_ptr->blk_geom->bheight; ++j)
                    memcpy(context_ptr->cfl_temp_luma_recon16bit + rec_luma_offset +
                               j * recon_ptr->stride_y,
                           ((uint16_t *)recon_ptr->buffer_y) +
                               (rec_luma_offset + j * recon_ptr->stride_y),
                           sizeof(uint16_t) * context_ptr->blk_geom->bwidth);
            } else {
                for (uint32_t j = 0; j < context_ptr->blk_geom->bheight; ++j)
                    memcpy(&context_ptr
                                ->cfl_temp_luma_recon[rec_luma_offset + j * recon_ptr->stride_y],
                           recon_ptr->buffer_y + rec_luma_offset + j * recon_ptr->stride_y,
                           context_ptr->blk_geom->bwidth);
            }
        }
        //copy neigh recon data in blk_ptr
        {
            uint32_t             j;
            EbPictureBufferDesc *recon_ptr       = candidate_buffer->recon_ptr;
            uint32_t             rec_luma_offset = context_ptr->blk_geom->origin_x +
                                       context_ptr->blk_geom->origin_y * recon_ptr->stride_y;

            uint32_t rec_cb_offset = ((((context_ptr->blk_geom->origin_x >> 3) << 3) +
                                       ((context_ptr->blk_geom->origin_y >> 3) << 3) *
                                           candidate_buffer->recon_ptr->stride_cb) >>
                                      1);
            uint32_t rec_cr_offset = ((((context_ptr->blk_geom->origin_x >> 3) << 3) +
                                       ((context_ptr->blk_geom->origin_y >> 3) << 3) *
                                           candidate_buffer->recon_ptr->stride_cr) >>
                                      1);

            if (!context_ptr->hbd_mode_decision) {
                memcpy(blk_ptr->neigh_top_recon[0],
                       recon_ptr->buffer_y + rec_luma_offset +
                           (context_ptr->blk_geom->bheight - 1) * recon_ptr->stride_y,
                       context_ptr->blk_geom->bwidth);
                if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
                    memcpy(blk_ptr->neigh_top_recon[1],
                           recon_ptr->buffer_cb + rec_cb_offset +
                               (context_ptr->blk_geom->bheight_uv - 1) * recon_ptr->stride_cb,
                           context_ptr->blk_geom->bwidth_uv);
                    memcpy(blk_ptr->neigh_top_recon[2],
                           recon_ptr->buffer_cr + rec_cr_offset +
                               (context_ptr->blk_geom->bheight_uv - 1) * recon_ptr->stride_cr,
                           context_ptr->blk_geom->bwidth_uv);
                }

                for (j = 0; j < context_ptr->blk_geom->bheight; ++j)
                    blk_ptr->neigh_left_recon[0][j] =
                        recon_ptr->buffer_y[rec_luma_offset + context_ptr->blk_geom->bwidth - 1 +
                                            j * recon_ptr->stride_y];

                if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
                    for (j = 0; j < context_ptr->blk_geom->bheight_uv; ++j) {
                        blk_ptr->neigh_left_recon[1][j] =
                            recon_ptr->buffer_cb[rec_cb_offset + context_ptr->blk_geom->bwidth_uv -
                                                 1 + j * recon_ptr->stride_cb];
                        blk_ptr->neigh_left_recon[2][j] =
                            recon_ptr->buffer_cr[rec_cr_offset + context_ptr->blk_geom->bwidth_uv -
                                                 1 + j * recon_ptr->stride_cr];
                    }
                }
            } else {
                uint16_t sz = sizeof(uint16_t);
                memcpy(blk_ptr->neigh_top_recon_16bit[0],
                       recon_ptr->buffer_y +
                           sz * (rec_luma_offset +
                                 (context_ptr->blk_geom->bheight - 1) * recon_ptr->stride_y),
                       sz * context_ptr->blk_geom->bwidth);
                if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
                    memcpy(blk_ptr->neigh_top_recon_16bit[1],
                           recon_ptr->buffer_cb +
                               sz * (rec_cb_offset + (context_ptr->blk_geom->bheight_uv - 1) *
                                                         recon_ptr->stride_cb),
                           sz * context_ptr->blk_geom->bwidth_uv);
                    memcpy(blk_ptr->neigh_top_recon_16bit[2],
                           recon_ptr->buffer_cr +
                               sz * (rec_cr_offset + (context_ptr->blk_geom->bheight_uv - 1) *
                                                         recon_ptr->stride_cr),
                           sz * context_ptr->blk_geom->bwidth_uv);
                }

                for (j = 0; j < context_ptr->blk_geom->bheight; ++j)
                    blk_ptr->neigh_left_recon_16bit[0][j] =
                        ((uint16_t *)
                             recon_ptr->buffer_y)[rec_luma_offset + context_ptr->blk_geom->bwidth -
                                                  1 + j * recon_ptr->stride_y];

                if (context_ptr->blk_geom->has_uv && context_ptr->chroma_level <= CHROMA_MODE_1) {
                    for (j = 0; j < context_ptr->blk_geom->bheight_uv; ++j) {
                        blk_ptr->neigh_left_recon_16bit[1][j] =
                            ((uint16_t *)recon_ptr
                                 ->buffer_cb)[rec_cb_offset + context_ptr->blk_geom->bwidth_uv - 1 +
                                              j * recon_ptr->stride_cb];
                        blk_ptr->neigh_left_recon_16bit[2][j] =
                            ((uint16_t *)recon_ptr
                                 ->buffer_cr)[rec_cr_offset + context_ptr->blk_geom->bwidth_uv - 1 +
                                              j * recon_ptr->stride_cr];
                    }
                }
            }
        }

#if NO_ENCDEC
        //copy recon
        uint32_t txb_origin_index =
            context_ptr->blk_geom->origin_x + (context_ptr->blk_geom->origin_y * 128);
        uint32_t bwidth  = context_ptr->blk_geom->bwidth;
        uint32_t bheight = context_ptr->blk_geom->bheight;

        if (!context_ptr->hbd_mode_decision) {
            uint8_t *src_ptr =
                &(((uint8_t *)candidate_buffer->recon_ptr->buffer_y)[txb_origin_index]);
            uint8_t *dst_ptr = &(((uint8_t *)context_ptr->blk_ptr->recon_tmp->buffer_y)[0]);

            uint32_t j;
            for (j = 0; j < bheight; j++)
                memcpy(dst_ptr + j * 128, src_ptr + j * 128, bwidth * sizeof(uint8_t));

            if (context_ptr->blk_geom->has_uv) {
                uint32_t txb_origin_index = ((((context_ptr->blk_geom->origin_x >> 3) << 3) +
                                              ((context_ptr->blk_geom->origin_y >> 3) << 3) *
                                                  candidate_buffer->recon_ptr->stride_cb) >>
                                             1);
                bwidth                    = context_ptr->blk_geom->bwidth_uv;
                bheight                   = context_ptr->blk_geom->bheight_uv;

                // Cb
                src_ptr = &(((uint8_t *)candidate_buffer->recon_ptr->buffer_cb)[txb_origin_index]);
                dst_ptr = &(((uint8_t *)context_ptr->blk_ptr->recon_tmp->buffer_cb)[0]);

                for (j = 0; j < bheight; j++)
                    memcpy(dst_ptr + j * 64, src_ptr + j * 64, bwidth * sizeof(uint8_t));

                // Cr
                src_ptr = &(((uint8_t *)candidate_buffer->recon_ptr->buffer_cr)[txb_origin_index]);
                dst_ptr = &(((uint8_t *)context_ptr->blk_ptr->recon_tmp->buffer_cr)[0]);

                for (j = 0; j < bheight; j++)
                    memcpy(dst_ptr + j * 64, src_ptr + j * 64, bwidth * sizeof(uint8_t));
            }
        } else {
            uint16_t *src_ptr =
                ((uint16_t *)candidate_buffer->recon_ptr->buffer_y) + txb_origin_index;
            uint16_t *dst_ptr = (uint16_t *)context_ptr->blk_ptr->recon_tmp->buffer_y;
            for (uint32_t j = 0; j < bheight; j++)
                memcpy(dst_ptr + j * 128, src_ptr + j * 128, bwidth * sizeof(uint16_t));

            if (context_ptr->blk_geom->has_uv) {
                txb_origin_index = ((((context_ptr->blk_geom->origin_x >> 3) << 3) +
                                     ((context_ptr->blk_geom->origin_y >> 3) << 3) *
                                         candidate_buffer->recon_ptr->stride_cb) >>
                                    1);
                bwidth           = context_ptr->blk_geom->bwidth_uv;
                bheight          = context_ptr->blk_geom->bheight_uv;

                // Cb
                src_ptr = ((uint16_t *)candidate_buffer->recon_ptr->buffer_cb) + txb_origin_index;
                dst_ptr = (uint16_t *)context_ptr->blk_ptr->recon_tmp->buffer_cb;
                for (uint32_t j = 0; j < bheight; j++)
                    memcpy(dst_ptr + j * 64, src_ptr + j * 64, bwidth * sizeof(uint16_t));

                // Cr
                src_ptr = ((uint16_t *)candidate_buffer->recon_ptr->buffer_cr) + txb_origin_index;
                dst_ptr = (uint16_t *)context_ptr->blk_ptr->recon_tmp->buffer_cr;
                for (uint32_t j = 0; j < bheight; j++)
                    memcpy(dst_ptr + j * 64, src_ptr + j * 64, bwidth * sizeof(uint16_t));
            }
        }
#endif

        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].avail_blk_flag = EB_TRUE;
    } else {
        context_ptr->md_local_blk_unit[blk_ptr->mds_idx].cost = MAX_MODE_COST;
        blk_ptr->prediction_unit_array->ref_frame_type        = 0;
    }
}

void update_skip_next_nsq_for_a_b_shapes(ModeDecisionContext *context_ptr, uint64_t *sq_cost,
                                         uint64_t *h_cost, uint64_t *v_cost, int *skip_next_nsq) {
    switch (context_ptr->blk_geom->d1i) {
    // NS
    case 0:
        *sq_cost = context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost;
        *h_cost  = 0;
        *v_cost  = 0;
        break;

    // H
    case 1: *h_cost = context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost; break;
    case 2: *h_cost += context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost; break;

    // V
    case 3: *v_cost = context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost; break;
    case 4:
        *v_cost += context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost;
        *skip_next_nsq =
            (*h_cost > ((*sq_cost * context_ptr->sq_weight) / 100)) ? 1 : *skip_next_nsq;
        break;

    // HA
    case 5:
    case 6:
    case 7:

    // HB
    case 8:
    case 9:
        *skip_next_nsq =
            (*h_cost > ((*sq_cost * context_ptr->sq_weight) / 100)) ? 1 : *skip_next_nsq;
        break;
    case 10:

    // VA
    case 11:
    case 12:
    case 13:

    // VB
    case 14:
    case 15:
        *skip_next_nsq =
            (*v_cost > ((*sq_cost * context_ptr->sq_weight) / 100)) ? 1 : *skip_next_nsq;
        break;
    }
}

#define FLT_MAX 3.402823466e+38F // max value
#define FEATURE_SIZE_MAX_MIN_PART_PRED 13

static uint32_t mds_idx_16x16[64] = {

    67,   128,  336,  397,  1168, 1229, 1437, 1498, 189,  250,  458,  519,  1290,
    1351, 1559, 1620, 605,  666,  874,  935,  1706, 1767, 1975, 2036, 727,  788,
    996,  1057, 1828, 1889, 2097, 2158, 2269, 2330, 2538, 2599, 3370, 3431, 3639,
    3700, 2391, 2452, 2660, 2721, 3492, 3553, 3761, 3822, 2807, 2868, 3076, 3137,
    3908, 3969, 4177, 4238, 2929, 2990, 3198, 3259, 4030, 4091, 4299, 4360

};

static EbAv1InterPredictionFuncPtr av1_inter_prediction_function_table[2] = {
    av1_inter_prediction, av1_inter_prediction_hbd};

void av1_get_max_min_partition_features(SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                                        ModeDecisionContext *context_ptr, float *features,
                                        EbPictureBufferDesc *input_picture_ptr,
                                        uint16_t sb_origin_x, uint16_t sb_origin_y) {
    int       f_idx   = 0;
    uint32_t  q_index = pcs_ptr->parent_pcs_ptr->frm_hdr.quantization_params.base_q_idx;
    const int dc_q    = eb_av1_dc_quant_qtx(q_index, 0, scs_ptr->static_config.encoder_bit_depth) >>
                     (scs_ptr->static_config.encoder_bit_depth - 8);

    aom_clear_system_state();
    const float log_q_sq = logf(1.0f + (float)(dc_q * dc_q) / 256.0f);

    // Perform full-pixel single motion search in Y plane of 16x16 mbs in the sb
    float sum_mv_row_sq  = 0;
    float sum_mv_row     = 0;
    float min_abs_mv_row = FLT_MAX;
    float max_abs_mv_row = 0;

    float sum_mv_col_sq  = 0;
    float sum_mv_col     = 0;
    float min_abs_mv_col = FLT_MAX;
    float max_abs_mv_col = 0;

    float sum_log_sse_sq = 0;
    float sum_log_sse    = 0;
    float min_log_sse    = FLT_MAX;
    float max_log_sse    = 0;

    // 16x16 motion search results
    for (uint32_t index = 0; index < 64; index++) {
        float        mv_col = 0;
        float        mv_row = 0;
        unsigned int sse    = 0;

        uint32_t         blk_idx_mds = mds_idx_16x16[index];
        const BlockGeom *blk_geom    = get_blk_geom_mds(blk_idx_mds);

        uint32_t me_sb_size = scs_ptr->sb_sz;
        uint32_t me_pic_width_in_sb =
            (pcs_ptr->parent_pcs_ptr->aligned_width + scs_ptr->sb_sz - 1) / me_sb_size;

        const uint32_t blk_origin_x = sb_origin_x + blk_geom->origin_x;
        const uint32_t blk_origin_y = sb_origin_y + blk_geom->origin_y;

        uint32_t me_sb_x       = (blk_origin_x / me_sb_size);
        uint32_t me_sb_y       = (blk_origin_y / me_sb_size);
        uint32_t me_sb_addr    = me_sb_x + me_sb_y * me_pic_width_in_sb;
        uint32_t geom_offset_x = (me_sb_x & 0x1) * me_sb_size;
        uint32_t geom_offset_y = (me_sb_y & 0x1) * me_sb_size;
        uint32_t me_block_offset =
            get_me_info_index(pcs_ptr->parent_pcs_ptr->max_number_of_pus_per_sb,
                              blk_geom,
                              geom_offset_x,
                              geom_offset_y);

        const MeSbResults *me_results       = pcs_ptr->parent_pcs_ptr->me_results[me_sb_addr];
        uint8_t            total_me_cnt     = me_results->total_me_candidate_index[me_block_offset];
        const MeCandidate *me_block_results = me_results->me_candidate[me_block_offset];

        for (uint8_t me_candidate_index = 0; me_candidate_index < total_me_cnt;
             ++me_candidate_index) {
            const MeCandidate *me_block_results_ptr = &me_block_results[me_candidate_index];
            const uint8_t      inter_direction      = me_block_results_ptr->direction;
            const uint8_t      list0_ref_index      = me_block_results_ptr->ref_idx_l0;

            if (inter_direction == 0 && list0_ref_index == 0) {
                ModeDecisionCandidateBuffer *candidate_buffer =
                    &(context_ptr->candidate_buffer_ptr_array[0][0]);
                candidate_buffer->candidate_ptr     = &(context_ptr->fast_candidate_array[0]);
                EbPictureBufferDesc *prediction_ptr = candidate_buffer->prediction_ptr;
                const InterpFilters  interp_filters =
                    av1_make_interp_filters(EIGHTTAP_REGULAR, EIGHTTAP_REGULAR);

                EbBool is_highbd = (scs_ptr->static_config.encoder_bit_depth == 8)
                                       ? (uint8_t)EB_FALSE
                                       : (uint8_t)EB_TRUE;

                BlkStruct  blk_ptr;
                MacroBlockD av1xd;
                blk_ptr.av1xd   = &av1xd;
                uint32_t mirow  = blk_origin_y >> MI_SIZE_LOG2;
                uint32_t micol  = blk_origin_x >> MI_SIZE_LOG2;
                blk_ptr.mds_idx = blk_idx_mds;

                const int32_t bw              = mi_size_wide[BLOCK_16X16];
                const int32_t bh              = mi_size_high[BLOCK_16X16];
                blk_ptr.av1xd->mb_to_top_edge = -(int32_t)((mirow * MI_SIZE) * 8);
                blk_ptr.av1xd->mb_to_bottom_edge =
                    ((pcs_ptr->parent_pcs_ptr->av1_cm->mi_rows - bw - mirow) * MI_SIZE) * 8;
                blk_ptr.av1xd->mb_to_left_edge = -(int32_t)((micol * MI_SIZE) * 8);
                blk_ptr.av1xd->mb_to_right_edge =
                    ((pcs_ptr->parent_pcs_ptr->av1_cm->mi_cols - bh - micol) * MI_SIZE) * 8;

                MvUnit mv_unit;
                mv_unit.pred_direction = UNI_PRED_LIST_0;
                mv_unit.mv->x = me_results->me_mv_array[me_block_offset][list0_ref_index].x_mv << 1;
                mv_unit.mv->y = me_results->me_mv_array[me_block_offset][list0_ref_index].y_mv << 1;

                av1_inter_prediction_function_table[is_highbd](
                    NULL, //pcs_ptr,
                    (uint32_t)interp_filters,
                    &blk_ptr,
                    0, //ref_frame_type,
                    &mv_unit,
                    0, //use_intrabc,
                    SIMPLE_TRANSLATION,
                    0,
                    0,
                    1, //compound_idx not used
                    NULL, // interinter_comp not used
                    NULL,
                    NULL,
                    NULL,
                    NULL,
                    0,
                    0,
                    0,
                    0,
                    blk_origin_x,
                    blk_origin_y,
                    blk_geom->bwidth,
                    blk_geom->bheight,
                    !is_highbd ? ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[0][0]->object_ptr)
                                     ->reference_picture
                               : ((EbReferenceObject *)pcs_ptr->ref_pic_ptr_array[0][0]->object_ptr)
                                     ->reference_picture16bit, // use last = [List 0][Ref Index 0]
                    NULL, //ref_pic_list1,
                    prediction_ptr,
                    blk_geom->origin_x,
                    blk_geom->origin_y,
                    0, //perform_chroma,
                    (uint8_t)scs_ptr->static_config.encoder_bit_depth);

                const AomVarianceFnPtr *fn_ptr = &mefn_ptr[blk_geom->bsize];

                const uint32_t input_origin_index =
                    (blk_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y +
                    (blk_origin_x + input_picture_ptr->origin_x);
                const uint32_t cu_origin_index =
                    blk_geom->origin_x + blk_geom->origin_y * SB_STRIDE_Y;
                fn_ptr->vf((input_picture_ptr->buffer_y + input_origin_index),
                           input_picture_ptr->stride_y,
                           (prediction_ptr->buffer_y + cu_origin_index),
                           prediction_ptr->stride_y,
                           &sse);

                mv_col = (float)(mv_unit.mv->x >> 3);
                mv_row = (float)(mv_unit.mv->y >> 3);

                break;
            }
        }

        const float log_sse    = logf(1.0f + (float)sse);
        const float abs_mv_row = fabsf(mv_row);
        const float abs_mv_col = fabsf(mv_col);

        sum_mv_row_sq += mv_row * mv_row;
        sum_mv_row += mv_row;
        sum_mv_col_sq += mv_col * mv_col;
        sum_mv_col += mv_col;

        if (abs_mv_row < min_abs_mv_row) min_abs_mv_row = abs_mv_row;
        if (abs_mv_row > max_abs_mv_row) max_abs_mv_row = abs_mv_row;
        if (abs_mv_col < min_abs_mv_col) min_abs_mv_col = abs_mv_col;
        if (abs_mv_col > max_abs_mv_col) max_abs_mv_col = abs_mv_col;

        sum_log_sse_sq += log_sse * log_sse;
        sum_log_sse += log_sse;
        if (log_sse < min_log_sse) min_log_sse = log_sse;
        if (log_sse > max_log_sse) max_log_sse = log_sse;
    }

    aom_clear_system_state();
    const float avg_mv_row = sum_mv_row / 64.0f;
    const float var_mv_row = sum_mv_row_sq / 64.0f - avg_mv_row * avg_mv_row;

    const float avg_mv_col = sum_mv_col / 64.0f;
    const float var_mv_col = sum_mv_col_sq / 64.0f - avg_mv_col * avg_mv_col;

    const float avg_log_sse = sum_log_sse / 64.0f;
    const float var_log_sse = sum_log_sse_sq / 64.0f - avg_log_sse * avg_log_sse;

    features[f_idx++] = avg_log_sse;
    features[f_idx++] = avg_mv_col;
    features[f_idx++] = avg_mv_row;
    features[f_idx++] = log_q_sq;
    features[f_idx++] = max_abs_mv_col;
    features[f_idx++] = max_abs_mv_row;
    features[f_idx++] = max_log_sse;
    features[f_idx++] = min_abs_mv_col;
    features[f_idx++] = min_abs_mv_row;
    features[f_idx++] = min_log_sse;
    features[f_idx++] = var_log_sse;
    features[f_idx++] = var_mv_col;
    features[f_idx++] = var_mv_row;

    assert(f_idx == FEATURE_SIZE_MAX_MIN_PART_PRED);
}

#define MAX_NUM_CLASSES_MAX_MIN_PART_PRED 4
BlockSize av1_predict_max_partition(PictureControlSet *pcs_ptr, const float *features,
                                    EbPictureBufferDesc *input_picture_ptr, uint16_t sb_origin_x,
                                    uint16_t sb_origin_y) {
    float scores[MAX_NUM_CLASSES_MAX_MIN_PART_PRED] = {0.0f},
          probs[MAX_NUM_CLASSES_MAX_MIN_PART_PRED]  = {0.0f};

    const NnConfig *nn_config = &av1_max_part_pred_nn_config;

    assert(pcs_ptr->sf.auto_max_partition_based_on_simple_motion != NOT_IN_USE);

    aom_clear_system_state();
    av1_nn_predict(features, nn_config, 1, scores);
    av1_nn_softmax(scores, probs, MAX_NUM_CLASSES_MAX_MIN_PART_PRED);

    int result = MAX_NUM_CLASSES_MAX_MIN_PART_PRED - 1;

    if (pcs_ptr->sf.auto_max_partition_based_on_simple_motion == DIRECT_PRED) {
        result         = 0;
        float max_prob = probs[0];
        for (int i = 1; i < MAX_NUM_CLASSES_MAX_MIN_PART_PRED; ++i) {
            if (probs[i] > max_prob) {
                max_prob = probs[i];
                result   = i;
            }
        }
    } else if (pcs_ptr->sf.auto_max_partition_based_on_simple_motion == RELAXED_PRED) {
        for (result = MAX_NUM_CLASSES_MAX_MIN_PART_PRED - 1; result >= 0; --result) {
            if (result < MAX_NUM_CLASSES_MAX_MIN_PART_PRED - 1) {
                probs[result] += probs[result + 1];
            }
            if (probs[result] > 0.2) break;
        }
    } else if (pcs_ptr->sf.auto_max_partition_based_on_simple_motion == ADAPT_PRED) {
        const uint32_t input_origin_index =
            (sb_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y +
            (sb_origin_x + input_picture_ptr->origin_x);
        const AomVarianceFnPtr *fn_ptr = &mefn_ptr[BLOCK_128X128];
        const unsigned int           source_variance =
            eb_av1_get_sby_perpixel_variance(fn_ptr,
                                             (input_picture_ptr->buffer_y + input_origin_index),
                                             input_picture_ptr->stride_y,
                                             BLOCK_128X128);

        if (source_variance > 16) {
            const double thresh = source_variance < 128 ? 0.05 : 0.1;
            for (result = MAX_NUM_CLASSES_MAX_MIN_PART_PRED - 1; result >= 0; --result) {
                if (result < MAX_NUM_CLASSES_MAX_MIN_PART_PRED - 1) {
                    probs[result] += probs[result + 1];
                }
                if (probs[result] > thresh) break;
            }
        }
    }

    return (BlockSize)((result + 2) * 3);
}

EB_EXTERN EbErrorType mode_decision_sb(SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                                       const MdcSbData *const mdcResultTbPtr, SuperBlock *sb_ptr,
                                       uint16_t sb_origin_x, uint16_t sb_origin_y, uint32_t sb_addr,
                                       ModeDecisionContext *context_ptr) {
    EbErrorType return_error = EB_ErrorNone;

    //printf("sb_origin_x = %d, sb_origin_y = %d\n", sb_origin_x, sb_origin_y);

    uint32_t                     blk_index;
    ModeDecisionCandidateBuffer *bestcandidate_buffers[5];
    // Pre Intra Search
    uint32_t                   leaf_count      = mdcResultTbPtr->leaf_count;
    const EbMdcLeafData *const leaf_data_array = mdcResultTbPtr->leaf_data_array;
#if TILES_PARALLEL
    const uint16_t tile_idx = context_ptr->tile_index;
#endif
    context_ptr->sb_ptr                        = sb_ptr;
#if !REMOVE_COEFF_BASED_SKIP_ATB
    context_ptr->coeff_based_skip_atb = 0;
#endif
    EbBool all_blk_init = (pcs_ptr->parent_pcs_ptr->pic_depth_mode <= PIC_SQ_DEPTH_MODE);
    if (all_blk_init) {
        init_sq_nsq_block(scs_ptr, context_ptr);
    } else {
        init_sq_non4_block(scs_ptr, context_ptr);
    }
    // Mode Decision Neighbor Arrays
#if TILES_PARALLEL
    context_ptr->intra_luma_mode_neighbor_array =
        pcs_ptr->md_intra_luma_mode_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->intra_chroma_mode_neighbor_array =
        pcs_ptr->md_intra_chroma_mode_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->mv_neighbor_array = pcs_ptr->md_mv_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->skip_flag_neighbor_array =
        pcs_ptr->md_skip_flag_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->mode_type_neighbor_array =
        pcs_ptr->md_mode_type_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->leaf_depth_neighbor_array =
        pcs_ptr->md_leaf_depth_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->leaf_partition_neighbor_array =
        pcs_ptr->mdleaf_partition_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];

    if (!context_ptr->hbd_mode_decision) {
        context_ptr->luma_recon_neighbor_array =
            pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
        context_ptr->cb_recon_neighbor_array =
            pcs_ptr->md_cb_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
        context_ptr->cr_recon_neighbor_array =
            pcs_ptr->md_cr_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    } else {
        context_ptr->luma_recon_neighbor_array16bit =
            pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
        context_ptr->cb_recon_neighbor_array16bit =
            pcs_ptr->md_cb_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
        context_ptr->cr_recon_neighbor_array16bit =
            pcs_ptr->md_cr_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    }
    context_ptr->skip_coeff_neighbor_array =
        pcs_ptr->md_skip_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->luma_dc_sign_level_coeff_neighbor_array =
        pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->cb_dc_sign_level_coeff_neighbor_array =
        pcs_ptr->md_cb_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->cr_dc_sign_level_coeff_neighbor_array =
        pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->txfm_context_array = pcs_ptr->md_txfm_context_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->inter_pred_dir_neighbor_array =
        pcs_ptr->md_inter_pred_dir_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->ref_frame_type_neighbor_array =
        pcs_ptr->md_ref_frame_type_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
    context_ptr->interpolation_type_neighbor_array =
        pcs_ptr->md_interpolation_type_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX][tile_idx];
#else
    context_ptr->intra_luma_mode_neighbor_array =
        pcs_ptr->md_intra_luma_mode_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->intra_chroma_mode_neighbor_array =
        pcs_ptr->md_intra_chroma_mode_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->mv_neighbor_array = pcs_ptr->md_mv_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->skip_flag_neighbor_array =
        pcs_ptr->md_skip_flag_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->mode_type_neighbor_array =
        pcs_ptr->md_mode_type_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->leaf_depth_neighbor_array =
        pcs_ptr->md_leaf_depth_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->leaf_partition_neighbor_array =
        pcs_ptr->mdleaf_partition_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];

    if (!context_ptr->hbd_mode_decision) {
        context_ptr->luma_recon_neighbor_array =
            pcs_ptr->md_luma_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
        context_ptr->cb_recon_neighbor_array =
            pcs_ptr->md_cb_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
        context_ptr->cr_recon_neighbor_array =
            pcs_ptr->md_cr_recon_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    } else {
        context_ptr->luma_recon_neighbor_array16bit =
            pcs_ptr->md_luma_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX];
        context_ptr->cb_recon_neighbor_array16bit =
            pcs_ptr->md_cb_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX];
        context_ptr->cr_recon_neighbor_array16bit =
            pcs_ptr->md_cr_recon_neighbor_array16bit[MD_NEIGHBOR_ARRAY_INDEX];
    }
    context_ptr->skip_coeff_neighbor_array =
        pcs_ptr->md_skip_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->luma_dc_sign_level_coeff_neighbor_array =
        pcs_ptr->md_luma_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->cb_dc_sign_level_coeff_neighbor_array =
        pcs_ptr->md_cb_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->cr_dc_sign_level_coeff_neighbor_array =
        pcs_ptr->md_cr_dc_sign_level_coeff_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->txfm_context_array = pcs_ptr->md_txfm_context_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->inter_pred_dir_neighbor_array =
        pcs_ptr->md_inter_pred_dir_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->ref_frame_type_neighbor_array =
        pcs_ptr->md_ref_frame_type_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
    context_ptr->interpolation_type_neighbor_array =
        pcs_ptr->md_interpolation_type_neighbor_array[MD_NEIGHBOR_ARRAY_INDEX];
#endif
    uint32_t             d1_block_itr      = 0;
    uint32_t             d1_first_block    = 1;
    EbPictureBufferDesc *input_picture_ptr = pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr;
    if (context_ptr->hbd_mode_decision) {
        const uint32_t input_luma_offset =
            ((sb_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_y) +
            (sb_origin_x + input_picture_ptr->origin_x);
        const uint32_t input_bit_inc_luma_offset =
            ((sb_origin_y + input_picture_ptr->origin_y) * input_picture_ptr->stride_bit_inc_y) +
            (sb_origin_x + input_picture_ptr->origin_x);
        const uint32_t input_cb_offset =
            (((sb_origin_y + input_picture_ptr->origin_y) >> 1) * input_picture_ptr->stride_cb) +
            ((sb_origin_x + input_picture_ptr->origin_x) >> 1);
        const uint32_t input_bit_inc_cb_offset =
            (((sb_origin_y + input_picture_ptr->origin_y) >> 1) *
             input_picture_ptr->stride_bit_inc_cb) +
            ((sb_origin_x + input_picture_ptr->origin_x) >> 1);
        const uint32_t input_cr_offset =
            (((sb_origin_y + input_picture_ptr->origin_y) >> 1) * input_picture_ptr->stride_cr) +
            ((sb_origin_x + input_picture_ptr->origin_x) >> 1);
        const uint32_t input_bit_inc_cr_offset =
            (((sb_origin_y + input_picture_ptr->origin_y) >> 1) *
             input_picture_ptr->stride_bit_inc_cr) +
            ((sb_origin_x + input_picture_ptr->origin_x) >> 1);

        uint32_t sb_width =
            MIN(scs_ptr->sb_size_pix, pcs_ptr->parent_pcs_ptr->aligned_width - sb_origin_x);
        uint32_t sb_height =
            MIN(scs_ptr->sb_size_pix, pcs_ptr->parent_pcs_ptr->aligned_height - sb_origin_y);

        pack2d_src(input_picture_ptr->buffer_y + input_luma_offset,
                   input_picture_ptr->stride_y,
                   input_picture_ptr->buffer_bit_inc_y + input_bit_inc_luma_offset,
                   input_picture_ptr->stride_bit_inc_y,
                   (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_y,
                   context_ptr->input_sample16bit_buffer->stride_y,
                   sb_width,
                   sb_height);

        pack2d_src(input_picture_ptr->buffer_cb + input_cb_offset,
                   input_picture_ptr->stride_cb,
                   input_picture_ptr->buffer_bit_inc_cb + input_bit_inc_cb_offset,
                   input_picture_ptr->stride_bit_inc_cb,
                   (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_cb,
                   context_ptr->input_sample16bit_buffer->stride_cb,
                   sb_width >> 1,
                   sb_height >> 1);

        pack2d_src(input_picture_ptr->buffer_cr + input_cr_offset,
                   input_picture_ptr->stride_cr,
                   input_picture_ptr->buffer_bit_inc_cr + input_bit_inc_cr_offset,
                   input_picture_ptr->stride_bit_inc_cr,
                   (uint16_t *)context_ptr->input_sample16bit_buffer->buffer_cr,
                   context_ptr->input_sample16bit_buffer->stride_cr,
                   sb_width >> 1,
                   sb_height >> 1);

        store16bit_input_src(context_ptr->input_sample16bit_buffer,
                             pcs_ptr,
                             sb_origin_x,
                             sb_origin_y,
                             sb_width,
                             sb_height);
        //input_picture_ptr = context_ptr->input_sample16bit_buffer;
        input_picture_ptr = pcs_ptr->input_frame16bit;
    }
    pcs_ptr->sf.auto_max_partition_based_on_simple_motion =
        ADAPT_PRED; //DIRECT_PRED; //RELAXED_PRED;
    BlockSize max_bsize = BLOCK_128X128;
    if (context_ptr->enable_auto_max_partition == 1)
        if (pcs_ptr->slice_type != I_SLICE && scs_ptr->static_config.super_block_size == 128) {
            if ((sb_origin_x + scs_ptr->static_config.super_block_size) <
                 pcs_ptr->parent_pcs_ptr->aligned_width &&
                (sb_origin_y + scs_ptr->static_config.super_block_size) <
                 pcs_ptr->parent_pcs_ptr->aligned_height) {
                float features[FEATURE_SIZE_MAX_MIN_PART_PRED] = {0.0f};

                av1_get_max_min_partition_features(scs_ptr,
                                                   pcs_ptr,
                                                   context_ptr,
                                                   features,
                                                   input_picture_ptr,
                                                   sb_origin_x,
                                                   sb_origin_y);
                max_bsize = MIN(av1_predict_max_partition(
                                    pcs_ptr, features, input_picture_ptr, sb_origin_x, sb_origin_y),
                                max_bsize);
            }
        }

    //CU Loop
    blk_index        = 0; //index over mdc array
    uint64_t sq_cost = 0;
    uint64_t h_cost;
    uint64_t v_cost;

    uint32_t blk_idx_mds               = 0;
    uint32_t d1_blocks_accumlated      = 0;
    int      skip_next_nsq             = 0;
    int      skip_next_sq              = 0;
    uint32_t next_non_skip_blk_idx_mds = 0;
    int64_t  depth_cost[NUMBER_OF_DEPTH]       = {-1, -1, -1, -1, -1, -1};
    uint64_t nsq_cost[NUMBER_OF_SHAPES]        = {MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST,
                                           MAX_CU_COST};
    Part     nsq_shape_table[NUMBER_OF_SHAPES] = {
        PART_N, PART_H, PART_V, PART_HA, PART_HB, PART_VA, PART_VB, PART_H4, PART_V4, PART_S};
    do {
        blk_idx_mds = leaf_data_array[blk_index].mds_idx;

        const BlockGeom *blk_geom = context_ptr->blk_geom = get_blk_geom_mds(blk_idx_mds);
        BlkStruct *     blk_ptr = context_ptr->blk_ptr = &context_ptr->md_blk_arr_nsq[blk_idx_mds];

        context_ptr->cu_size_log2 = blk_geom->bwidth_log2;
        context_ptr->blk_origin_x = sb_origin_x + blk_geom->origin_x;
        context_ptr->blk_origin_y = sb_origin_y + blk_geom->origin_y;

        const EbMdcLeafData *const leaf_data_ptr = &mdcResultTbPtr->leaf_data_array[blk_index];
        context_ptr->sb_sz                       = BLOCK_SIZE_64;
        context_ptr->round_origin_x              = ((context_ptr->blk_origin_x >> 3) << 3);
        context_ptr->round_origin_y              = ((context_ptr->blk_origin_y >> 3) << 3);
        context_ptr->sb_origin_x                 = sb_origin_x;
        context_ptr->sb_origin_y                 = sb_origin_y;
        context_ptr->md_local_blk_unit[blk_idx_mds].tested_blk_flag = EB_TRUE;
        context_ptr->md_ep_pipe_sb[blk_idx_mds].merge_cost          = 0;
        context_ptr->md_ep_pipe_sb[blk_idx_mds].skip_cost           = 0;
        blk_ptr->av1xd->sb_type                                     = blk_geom->bsize;
        blk_ptr->mds_idx                                            = blk_idx_mds;
        context_ptr->md_blk_arr_nsq[blk_idx_mds].mdc_split_flag =
            (uint16_t)leaf_data_ptr->split_flag;
        context_ptr->md_blk_arr_nsq[blk_geom->sqi_mds].split_flag =
            (uint16_t)leaf_data_ptr->split_flag;
        blk_ptr->split_flag =
            (uint16_t)leaf_data_ptr
                ->split_flag; //mdc indicates smallest or non valid CUs with split flag=
        blk_ptr->qp          = context_ptr->qp;
        blk_ptr->best_d1_blk = blk_idx_mds;
        if (leaf_data_ptr->tot_d1_blocks != 1) {
            // We need to get the index of the sq_block for each NSQ branch
            if (d1_first_block) {
                copy_neighbour_arrays( //save a clean neigh in [1], encode uses [0], reload the clean in [0] after done last ns block in a partition
                    pcs_ptr,
                    context_ptr,
                    0,
                    1,
                    blk_geom->sqi_mds,
                    sb_origin_x,
                    sb_origin_y);
            }
        }
        int32_t       mi_row    = context_ptr->blk_origin_y >> MI_SIZE_LOG2;
        int32_t       mi_col    = context_ptr->blk_origin_x >> MI_SIZE_LOG2;
        int           mi_stride = pcs_ptr->parent_pcs_ptr->av1_cm->mi_stride;
        const int32_t offset    = mi_row * mi_stride + mi_col;
        blk_ptr->av1xd->mi      = pcs_ptr->parent_pcs_ptr->av1_cm->pcs_ptr->mi_grid_base + offset;
        ModeInfo *mi_ptr        = *blk_ptr->av1xd->mi;
        blk_ptr->av1xd->up_available   = (mi_row > sb_ptr->tile_info.mi_row_start);
        blk_ptr->av1xd->left_available = (mi_col > sb_ptr->tile_info.mi_col_start);
        if (blk_ptr->av1xd->up_available)
            blk_ptr->av1xd->above_mbmi = &mi_ptr[-mi_stride].mbmi;
        else
            blk_ptr->av1xd->above_mbmi = NULL;
        if (blk_ptr->av1xd->left_available)
            blk_ptr->av1xd->left_mbmi = &mi_ptr[-1].mbmi;
        else
            blk_ptr->av1xd->left_mbmi = NULL;

        uint8_t  redundant_blk_avail = 0;
        uint16_t redundant_blk_mds;
        if (all_blk_init)
            check_redundant_block(blk_geom, context_ptr, &redundant_blk_avail, &redundant_blk_mds);

        if (redundant_blk_avail && context_ptr->redundant_blk) {
            // Copy results
            BlkStruct *src_cu = &context_ptr->md_blk_arr_nsq[redundant_blk_mds];
            BlkStruct *dst_cu = blk_ptr;
            move_blk_data_redund(pcs_ptr, context_ptr, src_cu, dst_cu);
            memcpy(&context_ptr->md_local_blk_unit[blk_ptr->mds_idx],
                   &context_ptr->md_local_blk_unit[redundant_blk_mds],
                   sizeof(MdBlkStruct));

            if (!context_ptr->hbd_mode_decision) {
                memcpy(dst_cu->neigh_left_recon[0], src_cu->neigh_left_recon[0], 128);
                memcpy(dst_cu->neigh_left_recon[1], src_cu->neigh_left_recon[1], 128);
                memcpy(dst_cu->neigh_left_recon[2], src_cu->neigh_left_recon[2], 128);
                memcpy(dst_cu->neigh_top_recon[0], src_cu->neigh_top_recon[0], 128);
                memcpy(dst_cu->neigh_top_recon[1], src_cu->neigh_top_recon[1], 128);
                memcpy(dst_cu->neigh_top_recon[2], src_cu->neigh_top_recon[2], 128);
            } else {
                uint16_t sz = sizeof(uint16_t);
                memcpy(
                    dst_cu->neigh_left_recon_16bit[0], src_cu->neigh_left_recon_16bit[0], 128 * sz);
                memcpy(
                    dst_cu->neigh_left_recon_16bit[1], src_cu->neigh_left_recon_16bit[1], 128 * sz);
                memcpy(
                    dst_cu->neigh_left_recon_16bit[2], src_cu->neigh_left_recon_16bit[2], 128 * sz);
                memcpy(
                    dst_cu->neigh_top_recon_16bit[0], src_cu->neigh_top_recon_16bit[0], 128 * sz);
                memcpy(
                    dst_cu->neigh_top_recon_16bit[1], src_cu->neigh_top_recon_16bit[1], 128 * sz);
                memcpy(
                    dst_cu->neigh_top_recon_16bit[2], src_cu->neigh_top_recon_16bit[2], 128 * sz);
            }

            memcpy(&context_ptr->md_ep_pipe_sb[blk_ptr->mds_idx],
                   &context_ptr->md_ep_pipe_sb[redundant_blk_mds],
                   sizeof(MdEncPassCuData));

            if (context_ptr->blk_geom->shape == PART_N) {
                uint8_t sq_index                      = LOG2F(context_ptr->blk_geom->sq_size) - 2;
                context_ptr->parent_sq_type[sq_index] = src_cu->prediction_mode_flag;
                context_ptr->parent_sq_has_coeff[sq_index] = src_cu->block_has_coeff;
                context_ptr->parent_sq_pred_mode[sq_index] = src_cu->pred_mode;
            }
        } else {
            // Initialize tx_depth
            blk_ptr->tx_depth = 0;
            if (blk_geom->quadi > 0 && d1_block_itr == 0 && !skip_next_sq) {
                uint32_t            blk_mds           = context_ptr->blk_geom->sqi_mds;
                uint64_t            parent_depth_cost = 0, current_depth_cost = 0;
                SequenceControlSet *scs_ptr =
                    (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;
                uint32_t parent_depth_idx_mds = blk_mds;

                // from a given child index, derive the index of the parent
                parent_depth_idx_mds =
                    (context_ptr->blk_geom->sqi_mds -
                     (context_ptr->blk_geom->quadi - 3) *
                         ns_depth_offset[scs_ptr->seq_header.sb_size == BLOCK_128X128]
                                        [context_ptr->blk_geom->depth]) -
                    parent_depth_offset[scs_ptr->seq_header.sb_size == BLOCK_128X128]
                                       [blk_geom->depth];

                if (pcs_ptr->slice_type == I_SLICE && parent_depth_idx_mds == 0 &&
                    scs_ptr->seq_header.sb_size == BLOCK_128X128)
                    parent_depth_cost = MAX_MODE_COST;
                else
                    compute_depth_costs_md_skip(
                        context_ptr,
                        scs_ptr,
                        pcs_ptr->parent_pcs_ptr,
                        parent_depth_idx_mds,
                        ns_depth_offset[scs_ptr->seq_header.sb_size == BLOCK_128X128]
                                       [context_ptr->blk_geom->depth],
                        &parent_depth_cost,
                        &current_depth_cost);

                if (!pcs_ptr->parent_pcs_ptr->sb_geom[sb_addr].block_is_allowed[parent_depth_idx_mds])
                    parent_depth_cost = MAX_MODE_COST;

                // compare the cost of the parent to the cost of the already encoded child + an estimated cost for the remaining child @ the current depth
                // if the total child cost is higher than the parent cost then skip the remaining  child @ the current depth
                // when md_exit_th=0 the estimated cost for the remaining child is not taken into account and the action will be lossless compared to no exit
                // MD_EXIT_THSL could be tuned toward a faster encoder but lossy
                if (parent_depth_cost != MAX_MODE_COST && parent_depth_cost <=
                    current_depth_cost +
                        (current_depth_cost * (4 - context_ptr->blk_geom->quadi) *
                         context_ptr->md_exit_th / context_ptr->blk_geom->quadi / 100)) {
                    skip_next_sq = 1;
                    next_non_skip_blk_idx_mds =
                        parent_depth_idx_mds +
                        ns_depth_offset[scs_ptr->seq_header.sb_size == BLOCK_128X128]
                                       [context_ptr->blk_geom->depth - 1];
                } else
                    skip_next_sq = 0;
            }
            // skip until we reach the next block @ the parent block depth
            if (blk_ptr->mds_idx >= next_non_skip_blk_idx_mds && skip_next_sq == 1)
                skip_next_sq = 0;
            EbBool auto_max_partition_block_skip =
                (context_ptr->blk_geom->bwidth > block_size_wide[max_bsize] ||
                 context_ptr->blk_geom->bheight > block_size_high[max_bsize]) &&
                (mdcResultTbPtr->leaf_data_array[blk_index].split_flag == EB_TRUE);

            if (pcs_ptr->parent_pcs_ptr->sb_geom[sb_addr]
                    .block_is_allowed[blk_ptr->mds_idx] &&
                !skip_next_nsq && !skip_next_sq && !auto_max_partition_block_skip) {
                md_encode_block(scs_ptr,
                                pcs_ptr,
                                context_ptr,
                                input_picture_ptr,
                                sb_addr,
                                bestcandidate_buffers);

            } else if (auto_max_partition_block_skip) {
                if (context_ptr->blk_geom->shape != PART_N)
                    context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost =
                        (MAX_MODE_COST >> 4);
                else
                    context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost =
                        (MAX_MODE_COST >> 10);
            } else if (skip_next_sq) {
                context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost =
                    (MAX_MODE_COST >> 10);
            } else {
                // If the block is out of the boundaries, md is not performed.
                // - For square blocks, since the blocks can be further splitted, they are considered in d2_inter_depth_block_decision with cost of zero.
                // - For non square blocks, since they can not be splitted further the cost is set to a large value (MAX_MODE_COST >> 4) to make sure they are not selected.
                //   The value is set to MAX_MODE_COST >> 4 to make sure there is not overflow when adding costs.
                if (context_ptr->blk_geom->shape != PART_N)
                    context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost =
                        (MAX_MODE_COST >> 4);
                else
                    context_ptr->md_local_blk_unit[context_ptr->blk_ptr->mds_idx].cost = 0;
            }
        }
        skip_next_nsq = 0;
        if (blk_geom->nsi + 1 == blk_geom->totns) {
            nsq_cost[context_ptr->blk_geom->shape] =
                d1_non_square_block_decision(context_ptr, d1_block_itr);
            d1_block_itr++;
        } else if (d1_block_itr) {
            uint64_t tot_cost      = 0;
            uint32_t first_blk_idx = context_ptr->blk_ptr->mds_idx -
                                     (blk_geom->nsi); //index of first block in this partition
            for (int blk_it = 0; blk_it < blk_geom->nsi + 1; blk_it++)
                tot_cost += context_ptr->md_local_blk_unit[first_blk_idx + blk_it].cost;
            nsq_cost[context_ptr->blk_geom->shape] = tot_cost;
            if ((tot_cost + tot_cost * (blk_geom->totns - (blk_geom->nsi + 1)) *
                                context_ptr->md_exit_th / (blk_geom->nsi + 1) / 100) >
                context_ptr->md_local_blk_unit[context_ptr->blk_geom->sqi_mds].cost)
                skip_next_nsq = 1;
        }
        if (context_ptr->sq_weight != (uint32_t)~0 && blk_geom->bsize > BLOCK_8X8)
            update_skip_next_nsq_for_a_b_shapes(
                context_ptr, &sq_cost, &h_cost, &v_cost, &skip_next_nsq);

        if (blk_geom->shape != PART_N) {
            if (blk_geom->nsi + 1 < blk_geom->totns)
                md_update_all_neighbour_arrays(
                    pcs_ptr, context_ptr, blk_idx_mds, sb_origin_x, sb_origin_y);
            else
                copy_neighbour_arrays( //restore [1] in [0] after done last ns block
                    pcs_ptr,
                    context_ptr,
                    1,
                    0,
                    blk_geom->sqi_mds,
                    sb_origin_x,
                    sb_origin_y);
        }
        d1_blocks_accumlated = d1_first_block == 1 ? 1 : d1_blocks_accumlated + 1;
        if (d1_blocks_accumlated == leaf_data_ptr->tot_d1_blocks) {
            //Sorting
            {
                uint32_t i, j, index;
                for (i = 0; i < NUMBER_OF_SHAPES - 1; ++i) {
                    for (j = i + 1; j < NUMBER_OF_SHAPES; ++j) {
                        if (nsq_cost[nsq_shape_table[j]] < nsq_cost[nsq_shape_table[i]]) {
                            index              = nsq_shape_table[i];
                            nsq_shape_table[i] = nsq_shape_table[j];
                            nsq_shape_table[j] = index;
                        }
                    }
                }
                depth_cost[scs_ptr->static_config.super_block_size == 128
                               ? context_ptr->blk_geom->depth
                               : context_ptr->blk_geom->depth + 1] += nsq_cost[nsq_shape_table[0]];
            }

            uint32_t last_blk_index_mds =
                d2_inter_depth_block_decision(context_ptr,
                                              blk_geom->sqi_mds, //input is parent square
                                              sb_ptr,
                                              sb_addr,
                                              sb_origin_x,
                                              sb_origin_y,
                                              context_ptr->full_lambda,
                                              context_ptr->md_rate_estimation_ptr,
                                              pcs_ptr);
            d1_block_itr   = 0;
            d1_first_block = 1;
#if !REMOVE_COEFF_BASED_SKIP_ATB
            context_ptr->coeff_based_skip_atb =
                pcs_ptr->parent_pcs_ptr->coeff_based_skip_atb &&
                        context_ptr->md_local_blk_unit[last_blk_index_mds].avail_blk_flag &&
                        context_ptr->md_blk_arr_nsq[last_blk_index_mds].block_has_coeff == 0
                    ? 1
                    : 0;
#endif
            if (context_ptr->md_blk_arr_nsq[last_blk_index_mds].split_flag == EB_FALSE) {
                md_update_all_neighbour_arrays_multiple(
                    pcs_ptr,
                    context_ptr,
                    context_ptr->md_blk_arr_nsq[last_blk_index_mds].best_d1_blk,
                    sb_origin_x,
                    sb_origin_y);
            }
        } else if (d1_first_block)
            d1_first_block = 0;
        blk_index++;
    } while (blk_index < leaf_count); // End of CU loop

    if (scs_ptr->seq_header.sb_size == BLOCK_64X64) depth_cost[0] = MAX_CU_COST;

    for (uint8_t depth_idx = 0; depth_idx < NUMBER_OF_DEPTH; depth_idx++) {
        sb_ptr->depth_cost[depth_idx] =
            depth_cost[depth_idx] < 0 ? MAX_MODE_COST : depth_cost[depth_idx];
    }

    return return_error;
}
#define MAX_SEARCH_POINT_WIDTH 128
#define MAX_SEARCH_POINT_HEIGHT 128

#define MAX_TATAL_SEARCH_AREA_WIDTH (MAX_SB_SIZE + MAX_SEARCH_POINT_WIDTH + ME_FILTER_TAP)
#define MAX_TATAL_SEARCH_AREA_HEIGHT (MAX_SB_SIZE + MAX_SEARCH_POINT_HEIGHT + ME_FILTER_TAP)

#define MAX_SEARCH_AREA_SIZE MAX_TATAL_SEARCH_AREA_WIDTH *MAX_TATAL_SEARCH_AREA_HEIGHT
