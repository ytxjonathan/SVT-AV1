/*
* Copyright(c) 2019 Intel Corporation
* SPDX - License - Identifier: BSD - 2 - Clause - Patent
*/

#include <stdlib.h>

#include "EbDefinitions.h"
#include "EbSystemResourceManager.h"
#include "EbPictureControlSet.h"
#include "EbSequenceControlSet.h"

#include "EbMotionEstimationResults.h"
#include "EbInitialRateControlProcess.h"
#include "EbInitialRateControlResults.h"
#include "EbMotionEstimationContext.h"
#include "EbUtility.h"
#include "EbReferenceObject.h"
#include "EbTransforms.h"
#include "aom_dsp_rtcd.h"

/**************************************
* Macros
**************************************/
#define PAN_LCU_PERCENTAGE                    75
#define LOW_AMPLITUDE_TH                      16

void GetMv(
    PictureParentControlSet    *picture_control_set_ptr,
    uint32_t                         sb_index,
    int32_t                        *xCurrentMv,
    int32_t                        *yCurrentMv)
{
    uint32_t             meCandidateIndex;

    const MeLcuResults *me_results = picture_control_set_ptr->me_results[sb_index];
    uint8_t total_me_cnt = me_results->total_me_candidate_index[0];
    const MeCandidate *me_block_results = me_results->me_candidate[0];
    for (meCandidateIndex = 0; meCandidateIndex < total_me_cnt; meCandidateIndex++) {
        if (me_block_results->direction == UNI_PRED_LIST_0) {
            *xCurrentMv = me_results->me_mv_array[0][0].x_mv;
            *yCurrentMv = me_results->me_mv_array[0][0].y_mv;
            break;
        }
    }
}

void GetMeDist(
    PictureParentControlSet    *picture_control_set_ptr,
    uint32_t                         sb_index,
    uint32_t                      *distortion)
{
    *distortion = (uint32_t)picture_control_set_ptr->me_results[sb_index]->me_candidate[0][0].distortion;
}

EbBool CheckMvForPanHighAmp(
    uint32_t   hierarchical_levels,
    uint32_t     temporal_layer_index,
    int32_t    *xCurrentMv,
    int32_t    *xCandidateMv)
{
    if (*xCurrentMv * *xCandidateMv > 0                        // both negative or both positives and both different than 0 i.e. same direction and non Stationary)
        && ABS(*xCurrentMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*xCandidateMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*xCurrentMv - *xCandidateMv) < LOW_AMPLITUDE_TH) {    // close amplitude

        return(EB_TRUE);
    }

    else
        return(EB_FALSE);
}

EbBool CheckMvForTiltHighAmp(
    uint32_t   hierarchical_levels,
    uint32_t     temporal_layer_index,
    int32_t    *yCurrentMv,
    int32_t    *yCandidateMv)
{
    if (*yCurrentMv * *yCandidateMv > 0                        // both negative or both positives and both different than 0 i.e. same direction and non Stationary)
        && ABS(*yCurrentMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*yCandidateMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*yCurrentMv - *yCandidateMv) < LOW_AMPLITUDE_TH) {    // close amplitude

        return(EB_TRUE);
    }

    else
        return(EB_FALSE);
}

EbBool CheckMvForPan(
    uint32_t   hierarchical_levels,
    uint32_t     temporal_layer_index,
    int32_t    *xCurrentMv,
    int32_t    *yCurrentMv,
    int32_t    *xCandidateMv,
    int32_t    *yCandidateMv)
{
    if (*yCurrentMv < LOW_AMPLITUDE_TH
        && *yCandidateMv < LOW_AMPLITUDE_TH
        && *xCurrentMv * *xCandidateMv        > 0                        // both negative or both positives and both different than 0 i.e. same direction and non Stationary)
        && ABS(*xCurrentMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*xCandidateMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*xCurrentMv - *xCandidateMv) < LOW_AMPLITUDE_TH) {    // close amplitude

        return(EB_TRUE);
    }

    else
        return(EB_FALSE);
}

EbBool CheckMvForTilt(
    uint32_t   hierarchical_levels,
    uint32_t     temporal_layer_index,
    int32_t    *xCurrentMv,
    int32_t    *yCurrentMv,
    int32_t    *xCandidateMv,
    int32_t    *yCandidateMv)
{
    if (*xCurrentMv < LOW_AMPLITUDE_TH
        && *xCandidateMv < LOW_AMPLITUDE_TH
        && *yCurrentMv * *yCandidateMv        > 0                        // both negative or both positives and both different than 0 i.e. same direction and non Stationary)
        && ABS(*yCurrentMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*yCandidateMv) >= global_motion_threshold[hierarchical_levels][temporal_layer_index]    // high amplitude
        && ABS(*yCurrentMv - *yCandidateMv) < LOW_AMPLITUDE_TH) {    // close amplitude

        return(EB_TRUE);
    }

    else
        return(EB_FALSE);
}

EbBool CheckMvForNonUniformMotion(
    int32_t    *xCurrentMv,
    int32_t    *yCurrentMv,
    int32_t    *xCandidateMv,
    int32_t    *yCandidateMv)
{
    int32_t mvThreshold = 40;//LOW_AMPLITUDE_TH + 18;
    // Either the x or the y direction is greater than threshold
    if ((ABS(*xCurrentMv - *xCandidateMv) > mvThreshold) || (ABS(*yCurrentMv - *yCandidateMv) > mvThreshold))
        return(EB_TRUE);
    else
        return(EB_FALSE);
}

void CheckForNonUniformMotionVectorField(
    PictureParentControlSet    *picture_control_set_ptr)
{
    uint32_t    sb_count;
    uint32_t    picture_width_in_sb = (picture_control_set_ptr->enhanced_picture_ptr->width + BLOCK_SIZE_64 - 1) / BLOCK_SIZE_64;
    uint32_t    sb_origin_x;
    uint32_t    sb_origin_y;

    int32_t    xCurrentMv = 0;
    int32_t    yCurrentMv = 0;
    int32_t    xLeftMv = 0;
    int32_t    yLeftMv = 0;
    int32_t    xTopMv = 0;
    int32_t    yTopMv = 0;
    int32_t    xRightMv = 0;
    int32_t    yRightMv = 0;
    int32_t    xBottomMv = 0;
    int32_t    yBottomMv = 0;
    uint32_t countOfNonUniformNeighbors = 0;

    for (sb_count = 0; sb_count < picture_control_set_ptr->sb_total_count; ++sb_count) {
        countOfNonUniformNeighbors = 0;

        sb_origin_x = (sb_count % picture_width_in_sb) * BLOCK_SIZE_64;
        sb_origin_y = (sb_count / picture_width_in_sb) * BLOCK_SIZE_64;

        if (((sb_origin_x + BLOCK_SIZE_64) <= picture_control_set_ptr->enhanced_picture_ptr->width) &&
            ((sb_origin_y + BLOCK_SIZE_64) <= picture_control_set_ptr->enhanced_picture_ptr->height)) {
            // Current MV
            GetMv(picture_control_set_ptr, sb_count, &xCurrentMv, &yCurrentMv);

            // Left MV
            if (sb_origin_x == 0) {
                xLeftMv = 0;
                yLeftMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count - 1, &xLeftMv, &yLeftMv);
            countOfNonUniformNeighbors += CheckMvForNonUniformMotion(&xCurrentMv, &yCurrentMv, &xLeftMv, &yLeftMv);

            // Top MV
            if (sb_origin_y == 0) {
                xTopMv = 0;
                yTopMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count - picture_width_in_sb, &xTopMv, &yTopMv);
            countOfNonUniformNeighbors += CheckMvForNonUniformMotion(&xCurrentMv, &yCurrentMv, &xTopMv, &yTopMv);

            // Right MV
            if ((sb_origin_x + (BLOCK_SIZE_64 << 1)) > picture_control_set_ptr->enhanced_picture_ptr->width) {
                xRightMv = 0;
                yRightMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count + 1, &xRightMv, &yRightMv);
            countOfNonUniformNeighbors += CheckMvForNonUniformMotion(&xCurrentMv, &yCurrentMv, &xRightMv, &yRightMv);

            // Bottom MV
            if ((sb_origin_y + (BLOCK_SIZE_64 << 1)) > picture_control_set_ptr->enhanced_picture_ptr->height) {
                xBottomMv = 0;
                yBottomMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count + picture_width_in_sb, &xBottomMv, &yBottomMv);
            countOfNonUniformNeighbors += CheckMvForNonUniformMotion(&xCurrentMv, &yCurrentMv, &xBottomMv, &yBottomMv);
        }
    }
}


void DetectGlobalMotion(
    PictureParentControlSet    *picture_control_set_ptr)
{
#if GLOBAL_WARPED_MOTION
#if GM_OPT
    if (picture_control_set_ptr->gm_level <= GM_DOWN) {
#endif
    uint32_t numOfListToSearch = (picture_control_set_ptr->slice_type == P_SLICE)
        ? (uint32_t)REF_LIST_0 : (uint32_t)REF_LIST_1;

    for (uint32_t listIndex = REF_LIST_0; listIndex <= numOfListToSearch; ++listIndex) {

        uint32_t num_of_ref_pic_to_search;
        if (picture_control_set_ptr->is_alt_ref == EB_TRUE)
            num_of_ref_pic_to_search = 1;
        else
            num_of_ref_pic_to_search = picture_control_set_ptr->slice_type == P_SLICE
                ? picture_control_set_ptr->ref_list0_count
                : listIndex == REF_LIST_0
                    ? picture_control_set_ptr->ref_list0_count
                    : picture_control_set_ptr->ref_list1_count;

        // Ref Picture Loop
        for (uint32_t ref_pic_index = 0; ref_pic_index < num_of_ref_pic_to_search;
             ++ref_pic_index)
        {
            picture_control_set_ptr->is_global_motion[listIndex][ref_pic_index] = EB_FALSE;
            if (picture_control_set_ptr->global_motion_estimation[listIndex][ref_pic_index].wmtype > TRANSLATION)
                picture_control_set_ptr->is_global_motion[listIndex][ref_pic_index] = EB_TRUE;
        }
    }
#endif
#if GM_OPT && GLOBAL_WARPED_MOTION || !GLOBAL_WARPED_MOTION
#if GM_OPT && GLOBAL_WARPED_MOTION
    }
    else {
#endif
    uint32_t    sb_count;
    uint32_t    picture_width_in_sb = (picture_control_set_ptr->enhanced_picture_ptr->width + BLOCK_SIZE_64 - 1) / BLOCK_SIZE_64;
    uint32_t    sb_origin_x;
    uint32_t    sb_origin_y;

    uint32_t  totalCheckedLcus = 0;
    uint32_t  totalPanLcus = 0;

    int32_t    xCurrentMv = 0;
    int32_t    yCurrentMv = 0;
    int32_t    xLeftMv = 0;
    int32_t    yLeftMv = 0;
    int32_t    xTopMv = 0;
    int32_t    yTopMv = 0;
    int32_t    xRightMv = 0;
    int32_t    yRightMv = 0;
    int32_t    xBottomMv = 0;
    int32_t    yBottomMv = 0;
    int64_t  xTiltMvSum = 0;
    int64_t  yTiltMvSum = 0;
    int64_t xPanMvSum = 0;
    int64_t yPanMvSum = 0;
    uint32_t  totalTiltLcus = 0;

    uint32_t  totalTiltHighAmpLcus = 0;
    uint32_t  totalPanHighAmpLcus = 0;

    for (sb_count = 0; sb_count < picture_control_set_ptr->sb_total_count; ++sb_count) {
        sb_origin_x = (sb_count % picture_width_in_sb) * BLOCK_SIZE_64;
        sb_origin_y = (sb_count / picture_width_in_sb) * BLOCK_SIZE_64;
        if (((sb_origin_x + BLOCK_SIZE_64) <= picture_control_set_ptr->enhanced_picture_ptr->width) &&
            ((sb_origin_y + BLOCK_SIZE_64) <= picture_control_set_ptr->enhanced_picture_ptr->height)) {
            // Current MV
            GetMv(picture_control_set_ptr, sb_count, &xCurrentMv, &yCurrentMv);

            // Left MV
            if (sb_origin_x == 0) {
                xLeftMv = 0;
                yLeftMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count - 1, &xLeftMv, &yLeftMv);
            // Top MV
            if (sb_origin_y == 0) {
                xTopMv = 0;
                yTopMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count - picture_width_in_sb, &xTopMv, &yTopMv);
            // Right MV
            if ((sb_origin_x + (BLOCK_SIZE_64 << 1)) > picture_control_set_ptr->enhanced_picture_ptr->width) {
                xRightMv = 0;
                yRightMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count + 1, &xRightMv, &yRightMv);
            // Bottom MV
            if ((sb_origin_y + (BLOCK_SIZE_64 << 1)) > picture_control_set_ptr->enhanced_picture_ptr->height) {
                xBottomMv = 0;
                yBottomMv = 0;
            }
            else
                GetMv(picture_control_set_ptr, sb_count + picture_width_in_sb, &xBottomMv, &yBottomMv);
            totalCheckedLcus++;

            if ((EbBool)(CheckMvForPan(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xLeftMv, &yLeftMv) ||
                CheckMvForPan(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xTopMv, &yTopMv) ||
                CheckMvForPan(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xRightMv, &yRightMv) ||
                CheckMvForPan(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xBottomMv, &yBottomMv))) {
                totalPanLcus++;

                xPanMvSum += xCurrentMv;
                yPanMvSum += yCurrentMv;
            }

            if ((EbBool)(CheckMvForTilt(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xLeftMv, &yLeftMv) ||
                CheckMvForTilt(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xTopMv, &yTopMv) ||
                CheckMvForTilt(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xRightMv, &yRightMv) ||
                CheckMvForTilt(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &yCurrentMv, &xBottomMv, &yBottomMv))) {
                totalTiltLcus++;

                xTiltMvSum += xCurrentMv;
                yTiltMvSum += yCurrentMv;
            }

            if ((EbBool)(CheckMvForPanHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &xLeftMv) ||
                CheckMvForPanHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &xTopMv) ||
                CheckMvForPanHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &xRightMv) ||
                CheckMvForPanHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &xCurrentMv, &xBottomMv))) {
                totalPanHighAmpLcus++;
            }

            if ((EbBool)(CheckMvForTiltHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &yCurrentMv, &yLeftMv) ||
                CheckMvForTiltHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &yCurrentMv, &yTopMv) ||
                CheckMvForTiltHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &yCurrentMv, &yRightMv) ||
                CheckMvForTiltHighAmp(picture_control_set_ptr->hierarchical_levels, picture_control_set_ptr->temporal_layer_index, &yCurrentMv, &yBottomMv))) {
                totalTiltHighAmpLcus++;
            }
        }
    }
    picture_control_set_ptr->is_pan = EB_FALSE;
    picture_control_set_ptr->is_tilt = EB_FALSE;

    picture_control_set_ptr->panMvx = 0;
    picture_control_set_ptr->panMvy = 0;
    picture_control_set_ptr->tiltMvx = 0;
    picture_control_set_ptr->tiltMvy = 0;

    // If more than PAN_LCU_PERCENTAGE % of LCUs are PAN
    if ((totalCheckedLcus > 0) && ((totalPanLcus * 100 / totalCheckedLcus) > PAN_LCU_PERCENTAGE)) {
        picture_control_set_ptr->is_pan = EB_TRUE;
        if (totalPanLcus > 0) {
            picture_control_set_ptr->panMvx = (int16_t)(xPanMvSum / totalPanLcus);
            picture_control_set_ptr->panMvy = (int16_t)(yPanMvSum / totalPanLcus);
        }
    }

    if ((totalCheckedLcus > 0) && ((totalTiltLcus * 100 / totalCheckedLcus) > PAN_LCU_PERCENTAGE)) {
        picture_control_set_ptr->is_tilt = EB_TRUE;
        if (totalTiltLcus > 0) {
            picture_control_set_ptr->tiltMvx = (int16_t)(xTiltMvSum / totalTiltLcus);
            picture_control_set_ptr->tiltMvy = (int16_t)(yTiltMvSum / totalTiltLcus);
        }
    }
#if GM_OPT && GLOBAL_WARPED_MOTION
    }
#endif
#endif
}

/************************************************
* Initial Rate Control Context Constructor
************************************************/
EbErrorType initial_rate_control_context_ctor(
    InitialRateControlContext  *context_ptr,
    EbFifo                     *motion_estimation_results_input_fifo_ptr,
    EbFifo                     *initialrate_control_results_output_fifo_ptr)
{
    context_ptr->motion_estimation_results_input_fifo_ptr = motion_estimation_results_input_fifo_ptr;
    context_ptr->initialrate_control_results_output_fifo_ptr = initialrate_control_results_output_fifo_ptr;

    return EB_ErrorNone;
}

/************************************************
* Release Pa Reference Objects
** Check if reference pictures are needed
** release them when appropriate
************************************************/
void ReleasePaReferenceObjects(
    SequenceControlSet              *sequence_control_set_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    // PA Reference Pictures
    uint32_t                             numOfListToSearch;
    uint32_t                             listIndex;
    uint32_t                             ref_pic_index;
    if (picture_control_set_ptr->slice_type != I_SLICE) {
        numOfListToSearch = (picture_control_set_ptr->slice_type == P_SLICE) ? REF_LIST_0 : REF_LIST_1;

        // List Loop
        for (listIndex = REF_LIST_0; listIndex <= numOfListToSearch; ++listIndex) {
            // Release PA Reference Pictures
            uint8_t num_of_ref_pic_to_search = (picture_control_set_ptr->slice_type == P_SLICE) ?
                MIN(picture_control_set_ptr->ref_list0_count, sequence_control_set_ptr->reference_count) :
                (listIndex == REF_LIST_0) ?
                MIN(picture_control_set_ptr->ref_list0_count, sequence_control_set_ptr->reference_count) :
                MIN(picture_control_set_ptr->ref_list1_count, sequence_control_set_ptr->reference_count);

            for (ref_pic_index = 0; ref_pic_index < num_of_ref_pic_to_search; ++ref_pic_index) {
                if (picture_control_set_ptr->ref_pa_pic_ptr_array[listIndex][ref_pic_index] != EB_NULL) {
                    eb_release_object(picture_control_set_ptr->ref_pa_pic_ptr_array[listIndex][ref_pic_index]);
                }
            }
        }
    }

    if (picture_control_set_ptr->pa_reference_picture_wrapper_ptr != EB_NULL) {
        eb_release_object(picture_control_set_ptr->pa_reference_picture_wrapper_ptr);
    }

    return;
}

/************************************************
* Global Motion Detection Based on ME information
** Mark pictures for pan
** Mark pictures for tilt
** No lookahead information used in this function
************************************************/
void MeBasedGlobalMotionDetection(
    PictureParentControlSet         *picture_control_set_ptr)
{
    // PAN Generation
    picture_control_set_ptr->is_pan = EB_FALSE;
    picture_control_set_ptr->is_tilt = EB_FALSE;

    if (picture_control_set_ptr->slice_type != I_SLICE)
        DetectGlobalMotion(picture_control_set_ptr);
    // Check if the motion vector field for temporal layer 0 pictures
    if (picture_control_set_ptr->slice_type != I_SLICE && picture_control_set_ptr->temporal_layer_index == 0)
        CheckForNonUniformMotionVectorField(picture_control_set_ptr);
    return;
}

void StationaryEdgeCountLcu(
    SequenceControlSet        *sequence_control_set_ptr,
    PictureParentControlSet   *picture_control_set_ptr,
    PictureParentControlSet   *temporalPictureControlSetPtr,
    uint32_t                       totalLcuCount)
{
    uint32_t               sb_index;
    for (sb_index = 0; sb_index < totalLcuCount; sb_index++) {
        SbParams sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
        SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];
        if (sb_params.potential_logo_sb &&sb_params.is_complete_sb && sb_stat_ptr->check1_for_logo_stationary_edge_over_time_flag && sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag) {
            SbStat *tempLcuStatPtr = &temporalPictureControlSetPtr->sb_stat_array[sb_index];
            uint32_t rasterScanCuIndex;

            if (tempLcuStatPtr->check1_for_logo_stationary_edge_over_time_flag)
            {
                for (rasterScanCuIndex = RASTER_SCAN_CU_INDEX_16x16_0; rasterScanCuIndex <= RASTER_SCAN_CU_INDEX_16x16_15; rasterScanCuIndex++)
                    sb_stat_ptr->cu_stat_array[rasterScanCuIndex].similar_edge_count += tempLcuStatPtr->cu_stat_array[rasterScanCuIndex].edge_cu;
            }
        }

        if (sb_params.potential_logo_sb &&sb_params.is_complete_sb && sb_stat_ptr->pm_check1_for_logo_stationary_edge_over_time_flag && sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag) {
            SbStat *tempLcuStatPtr = &temporalPictureControlSetPtr->sb_stat_array[sb_index];
            uint32_t rasterScanCuIndex;

            if (tempLcuStatPtr->pm_check1_for_logo_stationary_edge_over_time_flag)
            {
                for (rasterScanCuIndex = RASTER_SCAN_CU_INDEX_16x16_0; rasterScanCuIndex <= RASTER_SCAN_CU_INDEX_16x16_15; rasterScanCuIndex++)
                    sb_stat_ptr->cu_stat_array[rasterScanCuIndex].pm_similar_edge_count += tempLcuStatPtr->cu_stat_array[rasterScanCuIndex].edge_cu;
            }
        }
    }
}

void StationaryEdgeOverUpdateOverTimeLcuPart1(
    SequenceControlSet        *sequence_control_set_ptr,
    PictureParentControlSet   *picture_control_set_ptr)
{
    uint32_t               sb_index;
    int32_t                 xCurrentMv = 0;
    int32_t                 yCurrentMv = 0;

    for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; sb_index++) {
        SbParams sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
        SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

        if (sb_params.potential_logo_sb &&sb_params.is_complete_sb) {
            // Current MV
            if (picture_control_set_ptr->temporal_layer_index > 0)
                GetMv(picture_control_set_ptr, sb_index, &xCurrentMv, &yCurrentMv);

            EbBool lowMotion = picture_control_set_ptr->temporal_layer_index == 0 ? EB_TRUE : (ABS(xCurrentMv) < 16) && (ABS(yCurrentMv) < 16) ? EB_TRUE : EB_FALSE;
            uint16_t *yVariancePtr = picture_control_set_ptr->variance[sb_index];
            uint64_t var0 = yVariancePtr[ME_TIER_ZERO_PU_32x32_0];
            uint64_t var1 = yVariancePtr[ME_TIER_ZERO_PU_32x32_1];
            uint64_t var2 = yVariancePtr[ME_TIER_ZERO_PU_32x32_2];
            uint64_t var3 = yVariancePtr[ME_TIER_ZERO_PU_32x32_3];

            uint64_t averageVar = (var0 + var1 + var2 + var3) >> 2;
            uint64_t varOfVar = (((int32_t)(var0 - averageVar) * (int32_t)(var0 - averageVar)) +
                ((int32_t)(var1 - averageVar) * (int32_t)(var1 - averageVar)) +
                ((int32_t)(var2 - averageVar) * (int32_t)(var2 - averageVar)) +
                ((int32_t)(var3 - averageVar) * (int32_t)(var3 - averageVar))) >> 2;

            if ((varOfVar <= 50000) || !lowMotion)
                sb_stat_ptr->check1_for_logo_stationary_edge_over_time_flag = 0;
            else
                sb_stat_ptr->check1_for_logo_stationary_edge_over_time_flag = 1;
            if ((varOfVar <= 1000))
                sb_stat_ptr->pm_check1_for_logo_stationary_edge_over_time_flag = 0;
            else
                sb_stat_ptr->pm_check1_for_logo_stationary_edge_over_time_flag = 1;
        }
        else {
            sb_stat_ptr->check1_for_logo_stationary_edge_over_time_flag = 0;

            sb_stat_ptr->pm_check1_for_logo_stationary_edge_over_time_flag = 0;
        }
    }
}
void StationaryEdgeOverUpdateOverTimeLcuPart2(
    SequenceControlSet        *sequence_control_set_ptr,
    PictureParentControlSet   *picture_control_set_ptr)
{
    uint32_t               sb_index;

    uint32_t               lowSadTh = (sequence_control_set_ptr->input_resolution < INPUT_SIZE_1080p_RANGE) ? 5 : 2;

    for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; sb_index++) {
        SbParams sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
        SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

        if (sb_params.potential_logo_sb &&sb_params.is_complete_sb) {
            uint32_t meDist = 0;

            EbBool lowSad = EB_FALSE;

            if (picture_control_set_ptr->slice_type == B_SLICE)
                GetMeDist(picture_control_set_ptr, sb_index, &meDist);
            lowSad = (picture_control_set_ptr->slice_type != B_SLICE) ?

                EB_FALSE : (meDist < 64 * 64 * lowSadTh) ? EB_TRUE : EB_FALSE;

            if (lowSad) {
                sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag = 0;
                sb_stat_ptr->low_dist_logo = 1;
            }
            else {
                sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag = 1;

                sb_stat_ptr->low_dist_logo = 0;
            }
        }
        else {
            sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag = 0;

            sb_stat_ptr->low_dist_logo = 0;
        }
        sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag = 1;
    }
}

void StationaryEdgeOverUpdateOverTimeLcu(
    SequenceControlSet        *sequence_control_set_ptr,
    uint32_t                        totalCheckedPictures,
    PictureParentControlSet   *picture_control_set_ptr,
    uint32_t                       totalLcuCount)
{
    uint32_t               sb_index;
    const uint32_t         slideWindowTh = ((totalCheckedPictures / 4) - 1);

    for (sb_index = 0; sb_index < totalLcuCount; sb_index++) {
        SbParams sb_params = sequence_control_set_ptr->sb_params_array[sb_index];

        SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];
        sb_stat_ptr->stationary_edge_over_time_flag = EB_FALSE;
        if (sb_params.potential_logo_sb &&sb_params.is_complete_sb && sb_stat_ptr->check1_for_logo_stationary_edge_over_time_flag && sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag) {
            uint32_t rasterScanCuIndex;
            uint32_t similarEdgeCountLcu = 0;
            // CU Loop
            for (rasterScanCuIndex = RASTER_SCAN_CU_INDEX_16x16_0; rasterScanCuIndex <= RASTER_SCAN_CU_INDEX_16x16_15; rasterScanCuIndex++)
                similarEdgeCountLcu += (sb_stat_ptr->cu_stat_array[rasterScanCuIndex].similar_edge_count > slideWindowTh) ? 1 : 0;
            sb_stat_ptr->stationary_edge_over_time_flag = (similarEdgeCountLcu >= 4) ? EB_TRUE : EB_FALSE;
        }

        sb_stat_ptr->pm_stationary_edge_over_time_flag = EB_FALSE;
        if (sb_params.potential_logo_sb &&sb_params.is_complete_sb && sb_stat_ptr->pm_check1_for_logo_stationary_edge_over_time_flag && sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag) {
            uint32_t rasterScanCuIndex;
            uint32_t similarEdgeCountLcu = 0;
            // CU Loop
            for (rasterScanCuIndex = RASTER_SCAN_CU_INDEX_16x16_0; rasterScanCuIndex <= RASTER_SCAN_CU_INDEX_16x16_15; rasterScanCuIndex++)
                similarEdgeCountLcu += (sb_stat_ptr->cu_stat_array[rasterScanCuIndex].pm_similar_edge_count > slideWindowTh) ? 1 : 0;
            sb_stat_ptr->pm_stationary_edge_over_time_flag = (similarEdgeCountLcu >= 4) ? EB_TRUE : EB_FALSE;
        }
    }
    {
        uint32_t sb_index;
        uint32_t sb_x, sb_y;
        uint32_t countOfNeighbors = 0;

        uint32_t countOfNeighborsPm = 0;

        int32_t lcuHor, lcuVer, lcuVerOffset;
        int32_t lcuHorS, lcuVerS, lcuHorE, lcuVerE;
        uint32_t picture_width_in_sb = sequence_control_set_ptr->picture_width_in_sb;
        uint32_t picture_height_in_sb = sequence_control_set_ptr->picture_height_in_sb;

        for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
            SbParams                sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
            SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

            sb_x = sb_params.horizontal_index;
            sb_y = sb_params.vertical_index;
            if (sb_params.potential_logo_sb &&sb_params.is_complete_sb && sb_stat_ptr->check1_for_logo_stationary_edge_over_time_flag && sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag) {
                {
                    lcuHorS = (sb_x > 0) ? -1 : 0;
                    lcuHorE = (sb_x < picture_width_in_sb - 1) ? 1 : 0;
                    lcuVerS = (sb_y > 0) ? -1 : 0;
                    lcuVerE = (sb_y < picture_height_in_sb - 1) ? 1 : 0;
                    countOfNeighbors = 0;
                    for (lcuVer = lcuVerS; lcuVer <= lcuVerE; lcuVer++) {
                        lcuVerOffset = lcuVer * (int32_t)picture_width_in_sb;
                        for (lcuHor = lcuHorS; lcuHor <= lcuHorE; lcuHor++)
                            countOfNeighbors += (picture_control_set_ptr->sb_stat_array[sb_index + lcuVerOffset + lcuHor].stationary_edge_over_time_flag == 1);
                    }
                    if (countOfNeighbors == 1)
                        sb_stat_ptr->stationary_edge_over_time_flag = 0;
                }
            }

            if (sb_params.potential_logo_sb &&sb_params.is_complete_sb && sb_stat_ptr->pm_check1_for_logo_stationary_edge_over_time_flag && sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag) {
                {
                    lcuHorS = (sb_x > 0) ? -1 : 0;
                    lcuHorE = (sb_x < picture_width_in_sb - 1) ? 1 : 0;
                    lcuVerS = (sb_y > 0) ? -1 : 0;
                    lcuVerE = (sb_y < picture_height_in_sb - 1) ? 1 : 0;
                    countOfNeighborsPm = 0;
                    for (lcuVer = lcuVerS; lcuVer <= lcuVerE; lcuVer++) {
                        lcuVerOffset = lcuVer * (int32_t)picture_width_in_sb;
                        for (lcuHor = lcuHorS; lcuHor <= lcuHorE; lcuHor++)
                            countOfNeighborsPm += (picture_control_set_ptr->sb_stat_array[sb_index + lcuVerOffset + lcuHor].pm_stationary_edge_over_time_flag == 1);
                    }

                    if (countOfNeighborsPm == 1)
                        sb_stat_ptr->pm_stationary_edge_over_time_flag = 0;
                }
            }
        }
    }

    {
        uint32_t sb_index;
        uint32_t sb_x, sb_y;
        uint32_t countOfNeighbors = 0;
        int32_t lcuHor, lcuVer, lcuVerOffset;
        int32_t lcuHorS, lcuVerS, lcuHorE, lcuVerE;
        uint32_t picture_width_in_sb = sequence_control_set_ptr->picture_width_in_sb;
        uint32_t picture_height_in_sb = sequence_control_set_ptr->picture_height_in_sb;

        for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
            SbParams                sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
            SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

            sb_x = sb_params.horizontal_index;
            sb_y = sb_params.vertical_index;

            {
                if (sb_stat_ptr->stationary_edge_over_time_flag == 0 && sb_params.potential_logo_sb && (sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag || !sb_params.is_complete_sb)) {
                    lcuHorS = (sb_x > 0) ? -1 : 0;
                    lcuHorE = (sb_x < picture_width_in_sb - 1) ? 1 : 0;
                    lcuVerS = (sb_y > 0) ? -1 : 0;
                    lcuVerE = (sb_y < picture_height_in_sb - 1) ? 1 : 0;
                    countOfNeighbors = 0;
                    for (lcuVer = lcuVerS; lcuVer <= lcuVerE; lcuVer++) {
                        lcuVerOffset = lcuVer * (int32_t)picture_width_in_sb;
                        for (lcuHor = lcuHorS; lcuHor <= lcuHorE; lcuHor++)
                            countOfNeighbors += (picture_control_set_ptr->sb_stat_array[sb_index + lcuVerOffset + lcuHor].stationary_edge_over_time_flag == 1);
                    }
                    if (countOfNeighbors > 0)
                        sb_stat_ptr->stationary_edge_over_time_flag = 2;
                }
            }
        }

        for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
            SbParams                sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
            SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

            sb_x = sb_params.horizontal_index;
            sb_y = sb_params.vertical_index;

            {
                if (sb_stat_ptr->stationary_edge_over_time_flag == 0 && sb_params.potential_logo_sb && (sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag || !sb_params.is_complete_sb)) {
                    lcuHorS = (sb_x > 0) ? -1 : 0;
                    lcuHorE = (sb_x < picture_width_in_sb - 1) ? 1 : 0;
                    lcuVerS = (sb_y > 0) ? -1 : 0;
                    lcuVerE = (sb_y < picture_height_in_sb - 1) ? 1 : 0;
                    countOfNeighbors = 0;
                    for (lcuVer = lcuVerS; lcuVer <= lcuVerE; lcuVer++) {
                        lcuVerOffset = lcuVer * (int32_t)picture_width_in_sb;
                        for (lcuHor = lcuHorS; lcuHor <= lcuHorE; lcuHor++)
                            countOfNeighbors += (picture_control_set_ptr->sb_stat_array[sb_index + lcuVerOffset + lcuHor].stationary_edge_over_time_flag == 2);
                    }
                    if (countOfNeighbors > 3)
                        sb_stat_ptr->stationary_edge_over_time_flag = 3;
                }
            }
        }
    }

    {
        uint32_t sb_index;
        uint32_t sb_x, sb_y;
        uint32_t countOfNeighbors = 0;
        int32_t lcuHor, lcuVer, lcuVerOffset;
        int32_t lcuHorS, lcuVerS, lcuHorE, lcuVerE;
        uint32_t picture_width_in_sb = sequence_control_set_ptr->picture_width_in_sb;
        uint32_t picture_height_in_sb = sequence_control_set_ptr->picture_height_in_sb;

        for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
            SbParams                sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
            SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

            sb_x = sb_params.horizontal_index;
            sb_y = sb_params.vertical_index;

            {
                if (sb_stat_ptr->pm_stationary_edge_over_time_flag == 0 && sb_params.potential_logo_sb && (sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag || !sb_params.is_complete_sb)) {
                    lcuHorS = (sb_x > 0) ? -1 : 0;
                    lcuHorE = (sb_x < picture_width_in_sb - 1) ? 1 : 0;
                    lcuVerS = (sb_y > 0) ? -1 : 0;
                    lcuVerE = (sb_y < picture_height_in_sb - 1) ? 1 : 0;
                    countOfNeighbors = 0;
                    for (lcuVer = lcuVerS; lcuVer <= lcuVerE; lcuVer++) {
                        lcuVerOffset = lcuVer * (int32_t)picture_width_in_sb;
                        for (lcuHor = lcuHorS; lcuHor <= lcuHorE; lcuHor++)
                            countOfNeighbors += (picture_control_set_ptr->sb_stat_array[sb_index + lcuVerOffset + lcuHor].pm_stationary_edge_over_time_flag == 1);
                    }
                    if (countOfNeighbors > 0)
                        sb_stat_ptr->pm_stationary_edge_over_time_flag = 2;
                }
            }
        }

        for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
            SbParams                sb_params = sequence_control_set_ptr->sb_params_array[sb_index];
            SbStat *sb_stat_ptr = &picture_control_set_ptr->sb_stat_array[sb_index];

            sb_x = sb_params.horizontal_index;
            sb_y = sb_params.vertical_index;

            {
                if (sb_stat_ptr->pm_stationary_edge_over_time_flag == 0 && sb_params.potential_logo_sb && (sb_stat_ptr->check2_for_logo_stationary_edge_over_time_flag || !sb_params.is_complete_sb)) {
                    lcuHorS = (sb_x > 0) ? -1 : 0;
                    lcuHorE = (sb_x < picture_width_in_sb - 1) ? 1 : 0;
                    lcuVerS = (sb_y > 0) ? -1 : 0;
                    lcuVerE = (sb_y < picture_height_in_sb - 1) ? 1 : 0;
                    countOfNeighbors = 0;
                    for (lcuVer = lcuVerS; lcuVer <= lcuVerE; lcuVer++) {
                        lcuVerOffset = lcuVer * (int32_t)picture_width_in_sb;
                        for (lcuHor = lcuHorS; lcuHor <= lcuHorE; lcuHor++)
                            countOfNeighbors += (picture_control_set_ptr->sb_stat_array[sb_index + lcuVerOffset + lcuHor].pm_stationary_edge_over_time_flag == 2);
                    }
                    if (countOfNeighbors > 3)
                        sb_stat_ptr->pm_stationary_edge_over_time_flag = 3;
                }
            }
        }
    }
}

/************************************************
* Global Motion Detection Based on Lookahead
** Mark pictures for pan
** Mark pictures for tilt
** LAD Window: min (8 or sliding window size)
************************************************/
void UpdateGlobalMotionDetectionOverTime(
    EncodeContext                   *encode_context_ptr,
    SequenceControlSet              *sequence_control_set_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    InitialRateControlReorderEntry   *temporaryQueueEntryPtr;
    PictureParentControlSet          *temporaryPictureControlSetPtr;

    uint32_t                                totalPanPictures = 0;
    uint32_t                                totalCheckedPictures = 0;
    uint32_t                                totalTiltPictures = 0;
    uint32_t                                updateIsPanFramesToCheck;
    uint32_t                                inputQueueIndex;
    uint32_t                                framesToCheckIndex;

    (void)sequence_control_set_ptr;

    // Determine number of frames to check (8 frames)
    updateIsPanFramesToCheck = MIN(8, picture_control_set_ptr->frames_in_sw);

    // Walk the first N entries in the sliding window
    inputQueueIndex = encode_context_ptr->initial_rate_control_reorder_queue_head_index;
    uint32_t updateFramesToCheck = updateIsPanFramesToCheck;
    for (framesToCheckIndex = 0; framesToCheckIndex < updateFramesToCheck; framesToCheckIndex++) {
        temporaryQueueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[inputQueueIndex];
        temporaryPictureControlSetPtr = ((PictureParentControlSet*)(temporaryQueueEntryPtr->parent_pcs_wrapper_ptr)->object_ptr);

        if (temporaryPictureControlSetPtr->slice_type != I_SLICE) {
            totalPanPictures += (temporaryPictureControlSetPtr->is_pan == EB_TRUE);

            totalTiltPictures += (temporaryPictureControlSetPtr->is_tilt == EB_TRUE);

            // Keep track of checked pictures
            totalCheckedPictures++;
        }

        // Increment the inputQueueIndex Iterator
        inputQueueIndex = (inputQueueIndex == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : inputQueueIndex + 1;
    }

    picture_control_set_ptr->is_pan = EB_FALSE;
    picture_control_set_ptr->is_tilt = EB_FALSE;

    if (totalCheckedPictures) {
        if (picture_control_set_ptr->slice_type != I_SLICE) {
            if ((totalPanPictures * 100 / totalCheckedPictures) > 75)
                picture_control_set_ptr->is_pan = EB_TRUE;
        }
    }
    return;
}

/************************************************
* Update BEA Information Based on Lookahead
** Average zzCost of Collocated SB throughout lookahead frames
** Set isMostOfPictureNonMoving based on number of non moving LCUs
** LAD Window: min (2xmgpos+1 or sliding window size)
************************************************/

void UpdateBeaInfoOverTime(
    EncodeContext                   *encode_context_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    InitialRateControlReorderEntry   *temporaryQueueEntryPtr;
    PictureParentControlSet          *temporaryPictureControlSetPtr;
    uint32_t                                updateNonMovingIndexArrayFramesToCheck;
    uint16_t                              lcuIdx;
    uint16_t                                framesToCheckIndex;
    uint64_t                                nonMovingIndexSum = 0;
    uint32_t                                inputQueueIndex;

    SequenceControlSet *sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
    // Update motionIndexArray of the current picture by averaging the motionIndexArray of the N future pictures
    // Determine number of frames to check N
    updateNonMovingIndexArrayFramesToCheck = MIN(MIN(((picture_control_set_ptr->pred_struct_ptr->pred_struct_period << 1) + 1), picture_control_set_ptr->frames_in_sw), sequence_control_set_ptr->static_config.look_ahead_distance);
    uint64_t me_dist = 0;
    uint8_t me_dist_pic_count = 0;
    // SB Loop
    for (lcuIdx = 0; lcuIdx < picture_control_set_ptr->sb_total_count; ++lcuIdx) {
        uint16_t nonMovingIndexOverSlidingWindow = picture_control_set_ptr->non_moving_index_array[lcuIdx];

        // Walk the first N entries in the sliding window starting picture + 1
        inputQueueIndex = (encode_context_ptr->initial_rate_control_reorder_queue_head_index == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : encode_context_ptr->initial_rate_control_reorder_queue_head_index + 1;
        for (framesToCheckIndex = 0; framesToCheckIndex < updateNonMovingIndexArrayFramesToCheck - 1; framesToCheckIndex++) {
            temporaryQueueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[inputQueueIndex];
            temporaryPictureControlSetPtr = ((PictureParentControlSet*)(temporaryQueueEntryPtr->parent_pcs_wrapper_ptr)->object_ptr);

            if (temporaryPictureControlSetPtr->slice_type == I_SLICE || temporaryPictureControlSetPtr->end_of_sequence_flag)
                break;
            // Limit the distortion to lower layers 0, 1 and 2 only. Higher layers have close temporal distance and lower distortion that might contaminate the data
            if (temporaryPictureControlSetPtr->temporal_layer_index < MAX((int8_t)picture_control_set_ptr->hierarchical_levels - 1, 2) ) {
                if (lcuIdx == 0)
                    me_dist_pic_count++;
                me_dist += (temporaryPictureControlSetPtr->slice_type == I_SLICE) ? 0 : (uint64_t)temporaryPictureControlSetPtr->rc_me_distortion[lcuIdx];
            }
            // Store the filtered_sse of next ALT_REF picture in the I slice to be used in QP Scaling
            if (picture_control_set_ptr->slice_type == I_SLICE && picture_control_set_ptr->filtered_sse == 0 && lcuIdx == 0 && temporaryPictureControlSetPtr->temporal_layer_index == 0) {
                picture_control_set_ptr->filtered_sse = temporaryPictureControlSetPtr->filtered_sse;
                picture_control_set_ptr->filtered_sse_uv = temporaryPictureControlSetPtr->filtered_sse_uv;
            }
            nonMovingIndexOverSlidingWindow += temporaryPictureControlSetPtr->non_moving_index_array[lcuIdx];

            // Increment the inputQueueIndex Iterator
            inputQueueIndex = (inputQueueIndex == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : inputQueueIndex + 1;
        }
        picture_control_set_ptr->non_moving_index_array[lcuIdx] = (uint8_t)(nonMovingIndexOverSlidingWindow / (framesToCheckIndex + 1));

        nonMovingIndexSum += picture_control_set_ptr->non_moving_index_array[lcuIdx];
    }

    picture_control_set_ptr->non_moving_index_average = (uint16_t)nonMovingIndexSum / picture_control_set_ptr->sb_total_count;
    me_dist_pic_count = MAX(me_dist_pic_count, 1);
    picture_control_set_ptr->qp_scaling_average_complexity = (uint16_t)((uint64_t)me_dist / picture_control_set_ptr->sb_total_count / 256 / me_dist_pic_count);
    return;
}

/****************************************
* Init ZZ Cost array to default values
** Used when no Lookahead is available
****************************************/
void InitZzCostInfo(
    PictureParentControlSet         *picture_control_set_ptr)
{
    uint16_t lcuIdx;
    picture_control_set_ptr->non_moving_index_average = INVALID_ZZ_COST;

    // SB Loop
    for (lcuIdx = 0; lcuIdx < picture_control_set_ptr->sb_total_count; ++lcuIdx)
        picture_control_set_ptr->non_moving_index_array[lcuIdx] = INVALID_ZZ_COST;
    return;
}

/************************************************
* Update uniform motion field
** Update Uniformly moving LCUs using
** collocated LCUs infor in lookahead pictures
** LAD Window: min (2xmgpos+1 or sliding window size)
************************************************/
void UpdateMotionFieldUniformityOverTime(
    EncodeContext                   *encode_context_ptr,
    SequenceControlSet              *sequence_control_set_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    InitialRateControlReorderEntry   *temporaryQueueEntryPtr;
    PictureParentControlSet          *temporaryPictureControlSetPtr;
    uint32_t                                inputQueueIndex;
    uint32_t                              NoFramesToCheck;
    uint32_t                                framesToCheckIndex;
    //printf("To update POC %d\tframesInSw = %d\n", picture_control_set_ptr->picture_number, picture_control_set_ptr->frames_in_sw);

    //Check conditions for statinary edge over time
    StationaryEdgeOverUpdateOverTimeLcuPart2(
        sequence_control_set_ptr,
        picture_control_set_ptr);

    // Determine number of frames to check N
    NoFramesToCheck = MIN(MIN(((picture_control_set_ptr->pred_struct_ptr->pred_struct_period << 1) + 1), picture_control_set_ptr->frames_in_sw), sequence_control_set_ptr->static_config.look_ahead_distance);

    // Walk the first N entries in the sliding window starting picture + 1
    inputQueueIndex = (encode_context_ptr->initial_rate_control_reorder_queue_head_index == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : encode_context_ptr->initial_rate_control_reorder_queue_head_index;
    for (framesToCheckIndex = 0; framesToCheckIndex < NoFramesToCheck - 1; framesToCheckIndex++) {
        temporaryQueueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[inputQueueIndex];
        temporaryPictureControlSetPtr = ((PictureParentControlSet*)(temporaryQueueEntryPtr->parent_pcs_wrapper_ptr)->object_ptr);

        if (temporaryPictureControlSetPtr->end_of_sequence_flag)
            break;
        // The values are calculated for every 4th frame
        if ((temporaryPictureControlSetPtr->picture_number & 3) == 0) {
            StationaryEdgeCountLcu(
                sequence_control_set_ptr,
                picture_control_set_ptr,
                temporaryPictureControlSetPtr,
                picture_control_set_ptr->sb_total_count);
        }
        // Increment the inputQueueIndex Iterator
        inputQueueIndex = (inputQueueIndex == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : inputQueueIndex + 1;
    }
    StationaryEdgeOverUpdateOverTimeLcu(
        sequence_control_set_ptr,
        NoFramesToCheck,
        picture_control_set_ptr,
        picture_control_set_ptr->sb_total_count);
    return;
}
InitialRateControlReorderEntry  * DeterminePictureOffsetInQueue(
    EncodeContext                   *encode_context_ptr,
    PictureParentControlSet         *picture_control_set_ptr,
    MotionEstimationResults         *inputResultsPtr)
{
    InitialRateControlReorderEntry  *queueEntryPtr;
    int32_t                             queueEntryIndex;

    queueEntryIndex = (int32_t)(picture_control_set_ptr->picture_number - encode_context_ptr->initial_rate_control_reorder_queue[encode_context_ptr->initial_rate_control_reorder_queue_head_index]->picture_number);
    queueEntryIndex += encode_context_ptr->initial_rate_control_reorder_queue_head_index;
    queueEntryIndex = (queueEntryIndex > INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? queueEntryIndex - INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH : queueEntryIndex;
    queueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndex];
    queueEntryPtr->parent_pcs_wrapper_ptr = inputResultsPtr->picture_control_set_wrapper_ptr;
    queueEntryPtr->picture_number = picture_control_set_ptr->picture_number;

    return queueEntryPtr;
}

void GetHistogramQueueData(
    SequenceControlSet              *sequence_control_set_ptr,
    EncodeContext                   *encode_context_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    HlRateControlHistogramEntry     *histogramQueueEntryPtr;
    int32_t                             histogramQueueEntryIndex;

    // Determine offset from the Head Ptr for HLRC histogram queue
    eb_block_on_mutex(sequence_control_set_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
    histogramQueueEntryIndex = (int32_t)(picture_control_set_ptr->picture_number - encode_context_ptr->hl_rate_control_historgram_queue[encode_context_ptr->hl_rate_control_historgram_queue_head_index]->picture_number);
    histogramQueueEntryIndex += encode_context_ptr->hl_rate_control_historgram_queue_head_index;
    histogramQueueEntryIndex = (histogramQueueEntryIndex > HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1) ?
        histogramQueueEntryIndex - HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH :
        (histogramQueueEntryIndex < 0) ?
        histogramQueueEntryIndex + HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH :
        histogramQueueEntryIndex;
    histogramQueueEntryPtr = encode_context_ptr->hl_rate_control_historgram_queue[histogramQueueEntryIndex];

    //histogramQueueEntryPtr->parent_pcs_wrapper_ptr  = inputResultsPtr->picture_control_set_wrapper_ptr;
    histogramQueueEntryPtr->picture_number = picture_control_set_ptr->picture_number;
    histogramQueueEntryPtr->end_of_sequence_flag = picture_control_set_ptr->end_of_sequence_flag;
    histogramQueueEntryPtr->slice_type = picture_control_set_ptr->slice_type;
    histogramQueueEntryPtr->temporal_layer_index = picture_control_set_ptr->temporal_layer_index;
    histogramQueueEntryPtr->full_sb_count = picture_control_set_ptr->full_sb_count;
    histogramQueueEntryPtr->life_count = 0;
    histogramQueueEntryPtr->passed_to_hlrc = EB_FALSE;
    histogramQueueEntryPtr->is_coded = EB_FALSE;
    histogramQueueEntryPtr->total_num_bits_coded = 0;
    histogramQueueEntryPtr->frames_in_sw = 0;
    EB_MEMCPY(
        histogramQueueEntryPtr->me_distortion_histogram,
        picture_control_set_ptr->me_distortion_histogram,
        sizeof(uint16_t) * NUMBER_OF_SAD_INTERVALS);

    EB_MEMCPY(
        histogramQueueEntryPtr->ois_distortion_histogram,
        picture_control_set_ptr->ois_distortion_histogram,
        sizeof(uint16_t) * NUMBER_OF_INTRA_SAD_INTERVALS);

    eb_release_mutex(sequence_control_set_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);
    //printf("Test1 POC: %d\t POC: %d\t LifeCount: %d\n", histogramQueueEntryPtr->picture_number, picture_control_set_ptr->picture_number,  histogramQueueEntryPtr->life_count);

    return;
}

void UpdateHistogramQueueEntry(
    SequenceControlSet              *sequence_control_set_ptr,
    EncodeContext                   *encode_context_ptr,
    PictureParentControlSet         *picture_control_set_ptr,
    uint32_t                           frames_in_sw)
{
    HlRateControlHistogramEntry     *histogramQueueEntryPtr;
    int32_t                             histogramQueueEntryIndex;

    eb_block_on_mutex(sequence_control_set_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);

    histogramQueueEntryIndex = (int32_t)(picture_control_set_ptr->picture_number - encode_context_ptr->hl_rate_control_historgram_queue[encode_context_ptr->hl_rate_control_historgram_queue_head_index]->picture_number);
    histogramQueueEntryIndex += encode_context_ptr->hl_rate_control_historgram_queue_head_index;
    histogramQueueEntryIndex = (histogramQueueEntryIndex > HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH - 1) ?
        histogramQueueEntryIndex - HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH :
        (histogramQueueEntryIndex < 0) ?
        histogramQueueEntryIndex + HIGH_LEVEL_RATE_CONTROL_HISTOGRAM_QUEUE_MAX_DEPTH :
        histogramQueueEntryIndex;
    histogramQueueEntryPtr = encode_context_ptr->hl_rate_control_historgram_queue[histogramQueueEntryIndex];
    histogramQueueEntryPtr->passed_to_hlrc = EB_TRUE;
    if (sequence_control_set_ptr->static_config.rate_control_mode == 2)
        histogramQueueEntryPtr->life_count += (int16_t)(sequence_control_set_ptr->static_config.intra_period_length + 1) - 3; // FramelevelRC does not decrease the life count for first picture in each temporal layer
    else
        histogramQueueEntryPtr->life_count += picture_control_set_ptr->historgram_life_count;
    histogramQueueEntryPtr->frames_in_sw = frames_in_sw;
    eb_release_mutex(sequence_control_set_ptr->encode_context_ptr->hl_rate_control_historgram_queue_mutex);

    return;
}
EbAuraStatus AuraDetection64x64Gold(
    PictureControlSet           *picture_control_set_ptr,
    uint8_t                          picture_qp,
    uint32_t                         x_lcu_index,
    uint32_t                         y_lcu_index
);

/******************************************************
Input   : variance
Output  : true if current & neighbors are spatially complex
******************************************************/
EbBool IsSpatiallyComplexArea(
    PictureParentControlSet    *parentPcs,
    uint32_t                       pictureWidthInLcus,
    uint32_t                       lcuAdrr,
    uint32_t                       sb_origin_x,
    uint32_t                       sb_origin_y)
{
    uint32_t availableLcusCount = 0;
    uint32_t highVarianceLcusCount = 0;

    // Check the variance of the current LCU
    if ((parentPcs->variance[lcuAdrr][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH) {
        availableLcusCount++;
        highVarianceLcusCount++;
    }

    // Check the variance of left SB if available
    if (sb_origin_x != 0) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr - 1][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of right SB if available
    if ((sb_origin_x + BLOCK_SIZE_64) < parentPcs->enhanced_picture_ptr->width) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr + 1][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of top SB if available
    if (sb_origin_y != 0) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr - pictureWidthInLcus][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of bottom LCU
    if ((sb_origin_y + BLOCK_SIZE_64) < parentPcs->enhanced_picture_ptr->height) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr + pictureWidthInLcus][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of top-left LCU
    if ((sb_origin_x >= BLOCK_SIZE_64) && (sb_origin_y >= BLOCK_SIZE_64)) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr - pictureWidthInLcus - 1][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of top-right LCU
    if ((sb_origin_x < parentPcs->enhanced_picture_ptr->width - BLOCK_SIZE_64) && (sb_origin_y >= BLOCK_SIZE_64)) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr - pictureWidthInLcus + 1][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of bottom-left LCU
    if ((sb_origin_x >= BLOCK_SIZE_64) && (sb_origin_y < parentPcs->enhanced_picture_ptr->height - BLOCK_SIZE_64)) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr + pictureWidthInLcus - 1][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    // Check the variance of bottom-right LCU
    if ((sb_origin_x < parentPcs->enhanced_picture_ptr->width - BLOCK_SIZE_64) && (sb_origin_y < parentPcs->enhanced_picture_ptr->height - BLOCK_SIZE_64)) {
        availableLcusCount++;
        if ((parentPcs->variance[lcuAdrr + pictureWidthInLcus + 1][ME_TIER_ZERO_PU_64x64]) > IS_COMPLEX_LCU_VARIANCE_TH)
            highVarianceLcusCount++;
    }

    if (highVarianceLcusCount == availableLcusCount)
        return EB_TRUE;
    return EB_FALSE;
}

// Derives blockinessPresentFlag
void DeriveBlockinessPresentFlag(
    SequenceControlSet        *sequence_control_set_ptr,
    PictureParentControlSet   *picture_control_set_ptr)
{
    uint32_t                      sb_index;

    for (sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
        SbParams         *lcuParamPtr = &sequence_control_set_ptr->sb_params_array[sb_index];
        picture_control_set_ptr->complex_sb_array[sb_index] = SB_COMPLEXITY_STATUS_INVALID;

        // Spatially complex SB within a spatially complex area
        if (IsSpatiallyComplexArea(
            picture_control_set_ptr,
            (picture_control_set_ptr->enhanced_picture_ptr->width + BLOCK_SIZE_64 - 1) / BLOCK_SIZE_64,
            sb_index,
            lcuParamPtr->origin_x,
            lcuParamPtr->origin_y)) {
            // Active SB within an active scene (added a check on 4K & non-BASE to restrict the action - could be generated for all resolutions/layers)
            if (picture_control_set_ptr->non_moving_index_array[sb_index] == SB_COMPLEXITY_NON_MOVING_INDEX_TH_0 && picture_control_set_ptr->non_moving_index_average >= SB_COMPLEXITY_NON_MOVING_INDEX_TH_1 && picture_control_set_ptr->temporal_layer_index > 0 && sequence_control_set_ptr->input_resolution == INPUT_SIZE_4K_RANGE)
                picture_control_set_ptr->complex_sb_array[sb_index] = SB_COMPLEXITY_STATUS_2;
            // Active SB within a scene with a moderate acitivity (eg. active foregroud & static background)
            else if (picture_control_set_ptr->non_moving_index_array[sb_index] == SB_COMPLEXITY_NON_MOVING_INDEX_TH_0 && picture_control_set_ptr->non_moving_index_average >= SB_COMPLEXITY_NON_MOVING_INDEX_TH_2 && picture_control_set_ptr->non_moving_index_average < SB_COMPLEXITY_NON_MOVING_INDEX_TH_1)
                picture_control_set_ptr->complex_sb_array[sb_index] = SB_COMPLEXITY_STATUS_1;
            else
                picture_control_set_ptr->complex_sb_array[sb_index] = SB_COMPLEXITY_STATUS_0;
        }
        else
            picture_control_set_ptr->complex_sb_array[sb_index] = SB_COMPLEXITY_STATUS_0;
    }
}

#if CUTREE_LA
static AOM_INLINE void get_quantize_error(MacroblockPlane *p, int plane,
                                          const tran_low_t *coeff, tran_low_t *qcoeff,
                                          tran_low_t *dqcoeff, TxSize tx_size,
                                          uint16_t *eob, int64_t *recon_error,
                                          int64_t *sse) {
  const ScanOrder *const scan_order = &av1_scan_orders[tx_size][DCT_DCT]; //&av1_default_scan_orders[tx_size]
  int pix_num = 1 << num_pels_log2_lookup[txsize_to_bsize[tx_size]];
  const int shift = tx_size == TX_32X32 ? 0 : 2;

  eb_av1_quantize_fp(coeff, pix_num, p->zbin_QTX, p->round_fp_QTX, p->quant_fp_QTX,
                  p->quant_shift_QTX, qcoeff, dqcoeff, p->dequant_QTX, eob,
                  scan_order->scan, scan_order->iscan);

  *recon_error = av1_block_error(coeff, dqcoeff, pix_num, sse) >> shift;
  //*recon_error = av1_block_error_c(coeff, dqcoeff, pix_num, sse) >> shift;
  *recon_error = AOMMAX(*recon_error, 1);

  *sse = (*sse) >> shift;
  *sse = AOMMAX(*sse, 1);
}

static int rate_estimator(tran_low_t *qcoeff, int eob, TxSize tx_size) {
  const ScanOrder *const scan_order = &av1_scan_orders[tx_size][DCT_DCT]; //&av1_default_scan_orders[tx_size]

  assert((1 << num_pels_log2_lookup[txsize_to_bsize[tx_size]]) >= eob);

  int rate_cost = 1;

  for (int idx = 0; idx < eob; ++idx) {
    int abs_level = abs(qcoeff[scan_order->scan[idx]]);
    rate_cost += (int)(log(abs_level + 1.0) / log(2.0)) + 1;
  }

  return (rate_cost << AV1_PROB_COST_SHIFT);
}

static void result_model_store(PictureParentControlSet *picture_control_set_ptr, OisMbResults *ois_mb_results_ptr,
        uint32_t mb_origin_x, uint32_t mb_origin_y, uint32_t picture_width_in_mb) {
  const int mi_height = mi_size_high[TX_16X16];
  const int mi_width = mi_size_wide[TX_16X16];
  const int step = 1 << 2;//cpi->tpl_stats_block_mis_log2; //is_720p_or_larger ? 2 : 1;

  int64_t intra_cost = ois_mb_results_ptr->intra_cost / (mi_height * mi_width);
  int64_t inter_cost = ois_mb_results_ptr->inter_cost / (mi_height * mi_width);
  int64_t srcrf_dist = ois_mb_results_ptr->srcrf_dist / (mi_height * mi_width);
  int64_t recrf_dist = ois_mb_results_ptr->recrf_dist / (mi_height * mi_width);
  int64_t srcrf_rate = ois_mb_results_ptr->srcrf_rate / (mi_height * mi_width);
  int64_t recrf_rate = ois_mb_results_ptr->recrf_rate / (mi_height * mi_width);

  intra_cost = AOMMAX(1, intra_cost);
  inter_cost = AOMMAX(1, inter_cost);
  srcrf_dist = AOMMAX(1, srcrf_dist);
  recrf_dist = AOMMAX(1, recrf_dist);
  srcrf_rate = AOMMAX(1, srcrf_rate);
  recrf_rate = AOMMAX(1, recrf_rate);

  for (int idy = 0; idy < mi_height; idy += step) {
    //TplDepStats *tpl_ptr =
    //    &tpl_stats_ptr[av1_tpl_ptr_pos(cpi, mi_row + idy, mi_col, stride)];
    OisMbResults *dst_ptr = picture_control_set_ptr->ois_mb_results[(mb_origin_y >> 4) * picture_width_in_mb + (mb_origin_x >> 4)];
    for (int idx = 0; idx < mi_width; idx += step) {
      dst_ptr->intra_cost = intra_cost;
      dst_ptr->inter_cost = inter_cost;
      dst_ptr->srcrf_dist = srcrf_dist;
      dst_ptr->recrf_dist = recrf_dist;
      dst_ptr->srcrf_rate = srcrf_rate;
      dst_ptr->recrf_rate = recrf_rate;
      dst_ptr->mv = ois_mb_results_ptr->mv;
      dst_ptr->ref_frame_poc = ois_mb_results_ptr->ref_frame_poc;
      ++dst_ptr;
    }
  }
}

#define TPL_DEP_COST_SCALE_LOG2 4
/************************************************
* Genrate CUTree MC Flow Dispenser  Based on Lookahead
** LAD Window: sliding window size
************************************************/
void cutree_mc_flow_dispenser(
    EncodeContext                   *encode_context_ptr,
    SequenceControlSet              *sequence_control_set_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    InitialRateControlReorderEntry   *temporaryQueueEntryPtr;
    PictureParentControlSet          *temporaryPictureControlSetPtr;

    uint32_t    inputQueueIndex;
    uint32_t    frame_to_check_index;
    uint32_t    picture_width_in_sb = (picture_control_set_ptr->enhanced_picture_ptr->width + BLOCK_SIZE_64 - 1) / BLOCK_SIZE_64;
    uint32_t    picture_width_in_mb = (picture_control_set_ptr->enhanced_picture_ptr->width + 16 - 1) / 16;
    uint32_t    picture_height_in_sb = (picture_control_set_ptr->enhanced_picture_ptr->height + BLOCK_SIZE_64 - 1) / BLOCK_SIZE_64;
    uint32_t    sb_origin_x;
    uint32_t    sb_origin_y;
    int16_t     x_curr_mv = 0;
    int16_t     y_curr_mv = 0;
    uint32_t    me_mb_offset = 0;
    TxSize      tx_size = TX_16X16;
    EbPictureBufferDesc  *ref_pic_ptr;
    EbReferenceObject    *referenceObject;
    struct      ScaleFactors sf;
    BlockGeom   blk_geom;
    Av1Common *cm = picture_control_set_ptr->av1_cm;
    uint32_t    kernel = (EIGHTTAP_REGULAR << 16) | EIGHTTAP_REGULAR;
    EbPictureBufferDesc *input_picture_ptr = picture_control_set_ptr->enhanced_picture_ptr;
    int64_t recon_error = 1, sse = 1;

    (void)sequence_control_set_ptr;

    DECLARE_ALIGNED(32, uint8_t, predictor8[256 * 2]);
    DECLARE_ALIGNED(32, int16_t, src_diff[256]);
    DECLARE_ALIGNED(32, tran_low_t, coeff[256]);
    DECLARE_ALIGNED(32, tran_low_t, qcoeff[256]);
    DECLARE_ALIGNED(32, tran_low_t, dqcoeff[256]);
    DECLARE_ALIGNED(32, tran_low_t, best_coeff[256]);
    uint8_t *predictor = predictor8;

    blk_geom.bwidth  = 16;
    blk_geom.bheight = 16;


    av1_setup_scale_factors_for_frame(
                &sf, picture_width_in_sb * BLOCK_SIZE_64,
                picture_height_in_sb * BLOCK_SIZE_64,
                picture_width_in_sb * BLOCK_SIZE_64,
                picture_height_in_sb * BLOCK_SIZE_64);

    MacroblockPlane mb_plane;
    int32_t qIndex = quantizer_to_qindex[(uint8_t)sequence_control_set_ptr->qp];
    Quants *const quantsMd = &picture_control_set_ptr->quantsMd;
    Dequants *const dequantsMd = &picture_control_set_ptr->deqMd;
    eb_av1_set_quantizer(
        picture_control_set_ptr,
        picture_control_set_ptr->frm_hdr.quantization_params.base_q_idx);
    eb_av1_build_quantizer(
        /*picture_control_set_ptr->hbd_mode_decision ? AOM_BITS_10 :*/ AOM_BITS_8,
        picture_control_set_ptr->frm_hdr.quantization_params.delta_q_dc[AOM_PLANE_Y],
        picture_control_set_ptr->frm_hdr.quantization_params.delta_q_dc[AOM_PLANE_U],
        picture_control_set_ptr->frm_hdr.quantization_params.delta_q_ac[AOM_PLANE_U],
        picture_control_set_ptr->frm_hdr.quantization_params.delta_q_dc[AOM_PLANE_V],
        picture_control_set_ptr->frm_hdr.quantization_params.delta_q_ac[AOM_PLANE_V],
        quantsMd,
        dequantsMd);
    mb_plane.quant_QTX       = picture_control_set_ptr->quantsMd.y_quant[qIndex];
    mb_plane.quant_fp_QTX    = picture_control_set_ptr->quantsMd.y_quant_fp[qIndex];
    mb_plane.round_fp_QTX    = picture_control_set_ptr->quantsMd.y_round_fp[qIndex];
    mb_plane.quant_shift_QTX = picture_control_set_ptr->quantsMd.y_quant_shift[qIndex];
    mb_plane.zbin_QTX        = picture_control_set_ptr->quantsMd.y_zbin[qIndex];
    mb_plane.round_QTX       = picture_control_set_ptr->quantsMd.y_round[qIndex];
    mb_plane.dequant_QTX     = picture_control_set_ptr->deqMd.y_dequant_QTX[qIndex];
//printf("kelvin cutree_mc_flow_dispenser picture_number=%d qIndex=%d mb_plane.zbin_QTX[0~3]=%d %d %d %d\n", picture_control_set_ptr->picture_number, qIndex, mb_plane.zbin_QTX[0], mb_plane.zbin_QTX[1], mb_plane.zbin_QTX[2], mb_plane.zbin_QTX[3]);

    // Walk the first N entries in the sliding window
    inputQueueIndex = encode_context_ptr->initial_rate_control_reorder_queue_head_index;
    //inputQueueIndex = (inputQueueIndex == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : inputQueueIndex + 1;
    for (uint32_t sb_index = 0; sb_index < picture_control_set_ptr->sb_total_count; ++sb_index) {
        sb_origin_x = (sb_index % picture_width_in_sb) * BLOCK_SIZE_64;
        sb_origin_y = (sb_index / picture_width_in_sb) * BLOCK_SIZE_64;
        if (((sb_origin_x + BLOCK_SIZE_64) <= input_picture_ptr->width) &&
            ((sb_origin_y + BLOCK_SIZE_64) <= input_picture_ptr->height)) {
            SbParams *sb_params = &sequence_control_set_ptr->sb_params_array[sb_index];
            uint32_t pa_blk_index = 0;
            while (pa_blk_index < CU_MAX_COUNT) {
                const CodedUnitStats *blk_stats_ptr;
                blk_stats_ptr = get_coded_unit_stats(pa_blk_index);
                uint8_t bsize = blk_stats_ptr->size;
                if(bsize != 16) {
                    pa_blk_index++;
                    continue;
                }
                if (sb_params->raster_scan_cu_validity[md_scan_to_raster_scan[pa_blk_index]]) {
                    uint32_t mb_origin_x = sb_params->origin_x + blk_stats_ptr->origin_x;
                    uint32_t mb_origin_y = sb_params->origin_y + blk_stats_ptr->origin_y;
//printf("kelvin begin sb_index=%d, mb_origin_x=%d, mb_origin_y=%d, pa_blk_index=%d, origin_xy = %d %d\n", sb_index, mb_origin_x, mb_origin_y, pa_blk_index, blk_stats_ptr->origin_x, blk_stats_ptr->origin_y);
                    int64_t inter_cost;
                    int32_t best_rf_idx = -1;
                    int64_t best_inter_cost = INT64_MAX;
                    MV final_best_mv = {0, 0};
                    uint32_t max_inter_ref = ((sequence_control_set_ptr->mrp_mode == 0) ? ME_MV_MRP_MODE_0 : ME_MV_MRP_MODE_1);
                    OisMbResults *ois_mb_results_ptr = picture_control_set_ptr->ois_mb_results[(mb_origin_y >> 4) * picture_width_in_mb + (mb_origin_x >> 4)];
                    int64_t best_intra_cost = ois_mb_results_ptr->intra_cost;
                    uint8_t best_mode = DC_PRED;
                    uint8_t *src_mb = input_picture_ptr->buffer_y + input_picture_ptr->origin_x + mb_origin_x +
                                     (input_picture_ptr->origin_y + mb_origin_y) * input_picture_ptr->stride_y;
#if USE_ORIGIN_YUV
                    if(picture_control_set_ptr->temporal_layer_index == 0) {
                        src_mb = picture_control_set_ptr->save_enhanced_picture_ptr[0] + picture_control_set_ptr->enhanced_picture_ptr->origin_x + mb_origin_x +
                            (picture_control_set_ptr->enhanced_picture_ptr->origin_y + mb_origin_y) * input_picture_ptr->stride_y;
                    }
#endif
                    blk_geom.origin_x = blk_stats_ptr->origin_x;
                    blk_geom.origin_y = blk_stats_ptr->origin_y;
                    for(uint32_t rf_idx = 0; rf_idx < max_inter_ref; rf_idx++) {
                        me_mb_offset = get_me_info_index(picture_control_set_ptr->max_number_of_pus_per_sb, &blk_geom, 0, 0);

                        uint32_t list_index = (sequence_control_set_ptr->mrp_mode == 0) ? (rf_idx < 4 ? 0 : 1)
                                                                                        : (rf_idx < 2 ? 0 : 1);
                        uint32_t ref_pic_index = (sequence_control_set_ptr->mrp_mode == 0) ? (rf_idx >= 4 ? (rf_idx - 4) : rf_idx)
                                                                                           : (rf_idx >= 2 ? (rf_idx - 2) : rf_idx);
                        if(!picture_control_set_ptr->ref_pa_pic_ptr_array[list_index][ref_pic_index])
                            continue;
                        referenceObject = (EbReferenceObject*)picture_control_set_ptr->ref_pa_pic_ptr_array[list_index][ref_pic_index]->object_ptr;
                        ref_pic_ptr = /*is16bit ? (EbPictureBufferDesc*)referenceObject->reference_picture16bit : */(EbPictureBufferDesc*)referenceObject->reference_picture;
                        const int ref_basic_offset = ref_pic_ptr->origin_y * ref_pic_ptr->stride_y + ref_pic_ptr->origin_x;
                        const int ref_mb_offset = mb_origin_y * ref_pic_ptr->stride_y + mb_origin_x;
                        uint8_t *ref_mb = ref_pic_ptr->buffer_y + ref_basic_offset + ref_mb_offset;

                        struct buf_2d ref_buf = { NULL, ref_pic_ptr->buffer_y + ref_basic_offset,
                                                  ref_pic_ptr->width, ref_pic_ptr->height,
                                                  ref_pic_ptr->stride_y };
                        const MeLcuResults *me_results = picture_control_set_ptr->me_results[sb_index];
                        x_curr_mv = me_results->me_mv_array[me_mb_offset][(list_index ? ((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) : 0) + ref_pic_index].x_mv << 1;
                        y_curr_mv = me_results->me_mv_array[me_mb_offset][(list_index ? ((sequence_control_set_ptr->mrp_mode == 0) ? 4 : 2) : 0) + ref_pic_index].y_mv << 1;
//if(picture_control_set_ptr->picture_number == 16 && mb_origin_x == 0 && mb_origin_y == 0) {
//    printf("kelvin ---> inter search poc%d, mb_origin_xy=%d %d, rf_idx=%d, src[0~3]=%d %d %d %d, MV=%d %d, ref[0~3]=%d %d %d %d\n", picture_control_set_ptr->picture_number, mb_origin_x, mb_origin_y, rf_idx, src_mb[0], src_mb[1], src_mb[2], src_mb[3], x_curr_mv, y_curr_mv, ref_mb[0], ref_mb[1], ref_mb[2], ref_mb[3]);
//}
                        InterPredParams inter_pred_params;
                        av1_init_inter_params(&inter_pred_params, 16, 16, mb_origin_y,
                                mb_origin_x, 0, 0, 8/*xd->bd*/, 0/*is_cur_buf_hbd(xd)*/, 0,
                                &sf, &ref_buf, kernel);

                        inter_pred_params.conv_params = get_conv_params(0, 0, 0, 8/*xd->bd*/);

                        MV best_mv = {y_curr_mv, x_curr_mv};
                        av1_build_inter_predictor(ref_mb, input_picture_ptr->stride_y, predictor, 16,
                                &best_mv, mb_origin_x, mb_origin_y, &inter_pred_params);
                        aom_subtract_block(16, 16, src_diff, 16, src_mb, input_picture_ptr->stride_y, predictor, 16);

                        wht_fwd_txfm(src_diff, 16, coeff, tx_size, 8/*xd->bd*/, 0/*is_cur_buf_hbd(xd)*/);

                        inter_cost = aom_satd(coeff, 256);
                        if (inter_cost < best_inter_cost) {
                            memcpy(best_coeff, coeff, sizeof(best_coeff));
                            best_rf_idx = rf_idx;
                            best_inter_cost = inter_cost;
                            final_best_mv = best_mv;

                            if (best_inter_cost < best_intra_cost) best_mode = NEWMV;
                        }
                    } // rf_idx
                    if(best_inter_cost < INT64_MAX) {
                        uint16_t eob;
                        get_quantize_error(&mb_plane, 0, best_coeff, qcoeff, dqcoeff, tx_size, &eob, &recon_error, &sse);
                        int rate_cost = rate_estimator(qcoeff, eob, tx_size);
                        ois_mb_results_ptr->srcrf_rate = rate_cost << TPL_DEP_COST_SCALE_LOG2;
                    }
                    best_intra_cost = AOMMAX(best_intra_cost, 1);
#if 0
//printf("kelvincost1 poc%d sb_index=%d, mb_origin_xy=%d %d, best_mode=%d, best_intra_cost=%d, offset=%d\n", picture_control_set_ptr->picture_number, sb_index, mb_origin_x, mb_origin_y, ois_mb_results_ptr->intra_mode, best_intra_cost, (mb_origin_y >> 4) * picture_width_in_mb + (mb_origin_x >> 4));
//if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 544)
if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 0)
{
    if(mb_origin_x==0)
        printf("mbline%d poc%d\n", mb_origin_y>>4, picture_control_set_ptr->picture_number);
    printf("%d %d \n", best_intra_cost, best_inter_cost);
}
#endif
                    if (picture_control_set_ptr->picture_number == 0)//(frame_idx == 0)
                        best_inter_cost = 0;
                    else
                        best_inter_cost = AOMMIN(best_intra_cost, best_inter_cost);
                    ois_mb_results_ptr->inter_cost = best_inter_cost << TPL_DEP_COST_SCALE_LOG2;
                    ois_mb_results_ptr->intra_cost = best_intra_cost << TPL_DEP_COST_SCALE_LOG2;

                    ois_mb_results_ptr->srcrf_dist = recon_error << (TPL_DEP_COST_SCALE_LOG2);

                    if (best_mode == NEWMV) {
                        // inter recon
                        uint32_t list_index = (sequence_control_set_ptr->mrp_mode == 0) ? (best_rf_idx < 4 ? 0 : 1)
                                                                                        : (best_rf_idx < 2 ? 0 : 1);
                        uint32_t ref_pic_index = (sequence_control_set_ptr->mrp_mode == 0) ? (best_rf_idx >= 4 ? (best_rf_idx - 4) : best_rf_idx)
                                                                                           : (best_rf_idx >= 2 ? (best_rf_idx - 2) : best_rf_idx);
                        referenceObject = (EbReferenceObject*)picture_control_set_ptr->ref_pa_pic_ptr_array[list_index][ref_pic_index]->object_ptr;
                        ref_pic_ptr = /*is16bit ? (EbPictureBufferDesc*)referenceObject->reference_picture16bit : */(EbPictureBufferDesc*)referenceObject->reference_picture;
                        const int ref_basic_offset = ref_pic_ptr->origin_y * ref_pic_ptr->stride_y + ref_pic_ptr->origin_x;
                        const int ref_mb_offset = mb_origin_y * ref_pic_ptr->stride_y + mb_origin_x;
                        uint8_t *ref_mb = ref_pic_ptr->buffer_y + ref_basic_offset + ref_mb_offset;

                        struct buf_2d ref_buf = { NULL, ref_pic_ptr->buffer_y + ref_basic_offset,
                                                  ref_pic_ptr->width, ref_pic_ptr->height,
                                                  ref_pic_ptr->stride_y};
                        InterPredParams inter_pred_params;
                        av1_init_inter_params(&inter_pred_params, 16, 16, mb_origin_y,
                            mb_origin_x, 0, 0, 8/*xd->bd*/, 0/*is_cur_buf_hbd(xd)*/, 0,
                            &sf, &ref_buf, kernel);

                        inter_pred_params.conv_params = get_conv_params(0, 0, 0, 8/*xd->bd*/);

                        av1_build_inter_predictor(ref_mb, input_picture_ptr->stride_y, predictor, 16,
                                &final_best_mv, mb_origin_x, mb_origin_y, &inter_pred_params);
                    } else {
                        // intra recon
                        uint8_t *above_row;
                        uint8_t *left_col;
                        uint32_t mb_stride = (sequence_control_set_ptr->seq_header.max_frame_width + 15) / 16;
                        DECLARE_ALIGNED(16, uint8_t, left_data[MAX_TX_SIZE * 2 + 32]);
                        DECLARE_ALIGNED(16, uint8_t, above_data[MAX_TX_SIZE * 2 + 32]);

                        above_row = above_data + 16;
                        left_col = left_data + 16;
                        TxSize tx_size = TX_16X16;
                        uint8_t *src = input_picture_ptr->buffer_y + picture_control_set_ptr->enhanced_picture_ptr->origin_x + mb_origin_x +
                            (picture_control_set_ptr->enhanced_picture_ptr->origin_y + mb_origin_y) * input_picture_ptr->stride_y;
                        update_neighbor_samples_array_open_loop(above_row - 1, left_col - 1, input_picture_ptr, input_picture_ptr->stride_y, mb_origin_x, mb_origin_y, 16, 16, picture_control_set_ptr);
                        uint8_t ois_intra_mode = ois_mb_results_ptr->intra_mode;
                        int32_t p_angle = av1_is_directional_mode((PredictionMode)ois_intra_mode) ? mode_to_angle_map[(PredictionMode)ois_intra_mode] : 0;
                        // Edge filter
                        if(av1_is_directional_mode((PredictionMode)ois_intra_mode) && 1/*sequence_control_set_ptr->seq_header.enable_intra_edge_filter*/) {
                            filter_intra_edge(picture_control_set_ptr, ois_mb_results_ptr, ois_intra_mode, p_angle, mb_origin_x, mb_origin_y, above_row, left_col);
                        }
                        // PRED
                        intra_prediction_open_loop_mb(p_angle, ois_intra_mode, mb_origin_x, mb_origin_y, tx_size, above_row, left_col, predictor, 16/*dst_stride*/);
                    }

                    aom_subtract_block(16, 16, src_diff, 16, src_mb, input_picture_ptr->stride_y, predictor, 16);
                    wht_fwd_txfm(src_diff, 16, coeff, tx_size, 8/*xd->bd*/, 0/*s_cur_buf_hbd(xd)*/);

                    uint16_t eob;

                    get_quantize_error(&mb_plane, 0, coeff, qcoeff, dqcoeff, tx_size, &eob, &recon_error, &sse);
//if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 0 && mb_origin_x == 0) {
//    printf("kelvin ---> poc16 mb 0 0 recon_error=%d, sse=%d\n", recon_error, sse);
//}

                    int rate_cost = rate_estimator(qcoeff, eob, tx_size);
#if 0
                    if(eob) {
                        /*if(is16bit)
                            av1_inv_transform_recon16bit();
                        else*/
//printf("kelvincost1 poc%d sb_index=%d, mb_origin_xy=%d %d, before av1_inv_transform_recon8bit, eob=%d\n", picture_control_set_ptr->picture_number, sb_index, mb_origin_x, mb_origin_y, eob);
                            av1_inv_transform_recon8bit((int32_t*)dqcoeff, predictor, 16, predictor, 16, TX_16X16, DCT_DCT, PLANE_TYPE_Y, eob, 0 /*lossless*/);
                    }
#endif

                    ois_mb_results_ptr->recrf_dist = recon_error << (TPL_DEP_COST_SCALE_LOG2);
                    ois_mb_results_ptr->recrf_rate = rate_cost << TPL_DEP_COST_SCALE_LOG2;
//if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 0) {
//    if(mb_origin_x==0)
//        printf("mbline%d poc%d\n", mb_origin_y>>4, picture_control_set_ptr->picture_number);
//    printf("%d %d %d\n", ois_mb_results_ptr->srcrf_dist, ois_mb_results_ptr->recrf_dist, best_mode == NEWMV);
//}
                    if (best_mode != NEWMV) {
                        ois_mb_results_ptr->srcrf_dist = recon_error << (TPL_DEP_COST_SCALE_LOG2);
                        ois_mb_results_ptr->srcrf_rate = rate_cost << TPL_DEP_COST_SCALE_LOG2;
                    }
//if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 0) {
//    if(mb_origin_x==0)
//        printf("mbline%d poc%d\n", mb_origin_y>>4, picture_control_set_ptr->picture_number);
//    printf("%d %d %d\n", ois_mb_results_ptr->srcrf_dist, ois_mb_results_ptr->recrf_dist, best_mode == NEWMV);
//}
                    ois_mb_results_ptr->recrf_dist = AOMMAX(ois_mb_results_ptr->srcrf_dist, ois_mb_results_ptr->recrf_dist);
                    ois_mb_results_ptr->recrf_rate = AOMMAX(ois_mb_results_ptr->srcrf_rate, ois_mb_results_ptr->recrf_rate);

                    if (picture_control_set_ptr->picture_number != 0 && best_rf_idx != -1) {
                        ois_mb_results_ptr->mv = final_best_mv;
                        ois_mb_results_ptr->ref_frame_poc = picture_control_set_ptr->ref_order_hint[best_rf_idx];
//if(picture_control_set_ptr->picture_number == 15 && mb_origin_y == 544)
//if(picture_control_set_ptr->picture_number == 15 && ois_mb_results_ptr->ref_frame_poc != 14)
//if(ois_mb_results_ptr->ref_frame_poc == 8)
//printf("kelvin save curr poc %d, ref_frame_poc mb %d %d, mvxy %d %d, ref_frame_poc=%d\n", picture_control_set_ptr->picture_number, mb_origin_x>>4, mb_origin_y>>4, final_best_mv.col, final_best_mv.row, ois_mb_results_ptr->ref_frame_poc);
                    }
// printf one line MB data
//if(picture_control_set_ptr->picture_number == 0 && mb_origin_y == 0)
//if(picture_control_set_ptr->picture_number == 0 && mb_origin_y == 544)
//if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 544)
if(picture_control_set_ptr->picture_number == 16 && mb_origin_y == 0)
{
#if 1
//    if(mb_origin_x==0)
//        printf("mbline%d poc%d\n", mb_origin_y>>4, picture_control_set_ptr->picture_number);
//    printf("%d %d \n", ois_mb_results_ptr->srcrf_dist, ois_mb_results_ptr->recrf_dist);
//    printf("%d %d %d\n", ois_mb_results_ptr->inter_cost, ois_mb_results_ptr->recrf_dist, ois_mb_results_ptr->recrf_rate); //srcrf_dist srcrf_rate
//    printf("%d %d \n", best_intra_cost, best_inter_cost);
//if(mb_origin_x==16)
    //printf("%d %d\n", ois_mb_results_ptr->intra_cost, ois_mb_results_ptr->inter_cost);
    //printf("%d %d %d\n", ois_mb_results_ptr->inter_cost, ois_mb_results_ptr->srcrf_dist, ois_mb_results_ptr->srcrf_rate); //srcrf_dist srcrf_rate
    //printf("%d \n", best_intra_cost);
    //printf("%d %d \n", best_intra_cost, best_inter_cost);
    //printf("mbx%d %d %d isinterwinner%d\n", mb_origin_x >> 4, ois_mb_results_ptr->intra_cost, ois_mb_results_ptr->inter_cost, best_mode == NEWMV);
    //printf("\n");
#else
    if(mb_origin_x==0)
        printf("kelvinmbline%d poc%d\n", mb_origin_y, picture_control_set_ptr->picture_number);
    printf("mbx=%d %d %d\n", mb_origin_x >> 4, ois_mb_results_ptr->recrf_dist, ois_mb_results_ptr->recrf_rate);
    //if((mb_origin_x % 160)==0 || (mb_origin_x + 1 == picture_width_in_mb))
    //    printf("\n");
#endif
}
                    // Motion flow dependency dispenser.
                    result_model_store(picture_control_set_ptr, ois_mb_results_ptr, mb_origin_x, mb_origin_y, picture_width_in_mb);
                }
                pa_blk_index++;
            }
        }
    }

    return;
}

static int get_overlap_area(int grid_pos_row, int grid_pos_col, int ref_pos_row,
                            int ref_pos_col, int block, int/*BLOCK_SIZE*/ bsize) {
  int width = 0, height = 0;
  int bw = 4 << mi_size_wide_log2[bsize];
  int bh = 4 << mi_size_high_log2[bsize];

  switch (block) {
    case 0:
      width = grid_pos_col + bw - ref_pos_col;
      height = grid_pos_row + bh - ref_pos_row;
      break;
    case 1:
      width = ref_pos_col + bw - grid_pos_col;
      height = grid_pos_row + bh - ref_pos_row;
      break;
    case 2:
      width = grid_pos_col + bw - ref_pos_col;
      height = ref_pos_row + bh - grid_pos_row;
      break;
    case 3:
      width = ref_pos_col + bw - grid_pos_col;
      height = ref_pos_row + bh - grid_pos_row;
      break;
    default: assert(0);
  }

  return width * height;
}

static int round_floor(int ref_pos, int bsize_pix) {
  int round;
  if (ref_pos < 0)
    round = -(1 + (-ref_pos - 1) / bsize_pix);
  else
    round = ref_pos / bsize_pix;

  return round;
}

static int64_t delta_rate_cost(int64_t delta_rate, int64_t recrf_dist,
                               int64_t srcrf_dist, int pix_num) {
  double beta = (double)srcrf_dist / recrf_dist;
  int64_t rate_cost = delta_rate;

  if (srcrf_dist <= 128) return rate_cost;

  double dr =
      (double)(delta_rate >> (TPL_DEP_COST_SCALE_LOG2 + AV1_PROB_COST_SHIFT)) /
      pix_num;

  double log_den = log(beta) / log(2.0) + 2.0 * dr;

  if (log_den > log(10.0) / log(2.0)) {
    rate_cost = (int64_t)((log(1.0 / beta) * pix_num) / log(2.0) / 2.0);
    rate_cost <<= (TPL_DEP_COST_SCALE_LOG2 + AV1_PROB_COST_SHIFT);
    return rate_cost;
  }

  double num = pow(2.0, log_den);
  double den = num * beta + (1 - beta) * beta;

  rate_cost = (int64_t)((pix_num * log(num / den)) / log(2.0) / 2.0);

  rate_cost <<= (TPL_DEP_COST_SCALE_LOG2 + AV1_PROB_COST_SHIFT);

  return rate_cost;
}

static AOM_INLINE void tpl_model_update_b(PictureParentControlSet *ref_picture_control_set_ptr, OisMbResults *ois_mb_results_ptr,
                                          int mi_row, int mi_col,
                                          const int/*BLOCK_SIZE*/ bsize) {
  Av1Common *ref_cm = ref_picture_control_set_ptr->av1_cm;
  OisMbResults *ref_ois_mb_results_ptr;

  const int ref_pos_row = mi_row * MI_SIZE + (ois_mb_results_ptr->mv.row >> 3);
  const int ref_pos_col = mi_col * MI_SIZE + (ois_mb_results_ptr->mv.col >> 3);

  const int bw = 4 << mi_size_wide_log2[bsize];
  const int bh = 4 << mi_size_high_log2[bsize];
  const int mi_height = mi_size_high[bsize];
  const int mi_width = mi_size_wide[bsize];
  const int pix_num = bw * bh;

  // top-left on grid block location in pixel
  int grid_pos_row_base = round_floor(ref_pos_row, bh) * bh;
  int grid_pos_col_base = round_floor(ref_pos_col, bw) * bw;
  int block;

  int64_t cur_dep_dist = ois_mb_results_ptr->recrf_dist - ois_mb_results_ptr->srcrf_dist;
  int64_t mc_dep_dist = (int64_t)(
      ois_mb_results_ptr->mc_dep_dist *
      ((double)(ois_mb_results_ptr->recrf_dist - ois_mb_results_ptr->srcrf_dist) /
       ois_mb_results_ptr->recrf_dist));
  int64_t delta_rate = ois_mb_results_ptr->recrf_rate - ois_mb_results_ptr->srcrf_rate;
  int64_t mc_dep_rate =
      delta_rate_cost(ois_mb_results_ptr->mc_dep_rate, ois_mb_results_ptr->recrf_dist,
                      ois_mb_results_ptr->srcrf_dist, pix_num);

  for (block = 0; block < 4; ++block) {
    int grid_pos_row = grid_pos_row_base + bh * (block >> 1);
    int grid_pos_col = grid_pos_col_base + bw * (block & 0x01);

    if (grid_pos_row >= 0 && grid_pos_row < ref_cm->mi_rows * MI_SIZE &&
        grid_pos_col >= 0 && grid_pos_col < ref_cm->mi_cols * MI_SIZE) {
      int overlap_area = get_overlap_area(
          grid_pos_row, grid_pos_col, ref_pos_row, ref_pos_col, block, bsize);
      int ref_mi_row = round_floor(grid_pos_row, bh) * mi_height;
      int ref_mi_col = round_floor(grid_pos_col, bw) * mi_width;
      const int step = 1 << 2;//cpi->tpl_stats_block_mis_log2;

      for (int idy = 0; idy < mi_height; idy += step) {
        for (int idx = 0; idx < mi_width; idx += step) {
          ref_ois_mb_results_ptr = ref_picture_control_set_ptr->ois_mb_results[((ref_mi_row + idy) >> 2) * (ref_cm->mi_rows >> 2)  + ((ref_mi_col + idx) >> 2)]; //cpi->tpl_stats_block_mis_log2;
          ref_ois_mb_results_ptr->mc_dep_dist +=
              ((cur_dep_dist + mc_dep_dist) * overlap_area) / pix_num;
          ref_ois_mb_results_ptr->mc_dep_rate +=
              ((delta_rate + mc_dep_rate) * overlap_area) / pix_num;

          assert(overlap_area >= 0);
        }
      }
    }
  }
}

static AOM_INLINE void tpl_model_update(PictureParentControlSet *picture_control_set_array[60], int32_t frame_idx, int mi_row, int mi_col, const int/*BLOCK_SIZE*/ bsize, uint8_t frames_in_sw) {
  const int mi_height = mi_size_high[bsize];
  const int mi_width = mi_size_wide[bsize];
  const int step = 1 << 2; //cpi->tpl_stats_block_mis_log2;
  const int/*BLOCK_SIZE*/ block_size = BLOCK_16X16; //convert_length_to_bsize(MI_SIZE << cpi->tpl_stats_block_mis_log2);
  PictureParentControlSet *picture_control_set_ptr = picture_control_set_array[frame_idx];
  int i = 0;

  for (int idy = 0; idy < mi_height; idy += step) {
    for (int idx = 0; idx < mi_width; idx += step) {
      OisMbResults *ois_mb_results_ptr = picture_control_set_ptr->ois_mb_results[(((mi_row + idy) * mi_width) >> 4) + ((mi_col + idx) >> 2)];
      while(i<frames_in_sw && picture_control_set_array[i]->picture_number != ois_mb_results_ptr->ref_frame_poc)
        i++;
      //assert(picture_control_set_array[i]->picture_number < frame_idx); //kelvin to check
      if(i<frames_in_sw)
        tpl_model_update_b(picture_control_set_array[i], ois_mb_results_ptr, mi_row + idy, mi_col + idx, block_size);
    }
  }
}

void cutree_mc_flow_synthesizer(
    EncodeContext                   *encode_context_ptr,
    SequenceControlSet              *sequence_control_set_ptr,
    PictureParentControlSet         *picture_control_set_array[60],
    int32_t                          frame_idx,
    uint8_t                          frames_in_sw)
{
    Av1Common *cm = picture_control_set_array[frame_idx]->av1_cm;
    const int/*BLOCK_SIZE*/ bsize = BLOCK_16X16;
    const int mi_height = mi_size_high[bsize];
    const int mi_width = mi_size_wide[bsize];

    for (int mi_row = 0; mi_row < cm->mi_rows; mi_row += mi_height) {
        for (int mi_col = 0; mi_col < cm->mi_cols; mi_col += mi_width) {
            tpl_model_update(picture_control_set_array, frame_idx, mi_row, mi_col, bsize, frames_in_sw);
        }
    }
    return;
}

/************************************************
* Genrate CUTree MC Flow Synthesizer Based on Lookahead
** LAD Window: sliding window size
************************************************/
void update_mc_flow_synthesizer(
    EncodeContext                   *encode_context_ptr,
    SequenceControlSet              *sequence_control_set_ptr,
    PictureParentControlSet         *picture_control_set_ptr)
{
    InitialRateControlReorderEntry   *temporaryQueueEntryPtr;
    PictureParentControlSet          *temp_picture_control_set_ptr;
    PictureParentControlSet          *picture_control_set_array[60] = {NULL, };

    //uint32_t                         frame_idx_checked = 0;
    uint32_t                         inputQueueIndex;
    int32_t                          frame_idx, i;

    (void)sequence_control_set_ptr;

    // Walk the first N entries in the sliding window
    inputQueueIndex = encode_context_ptr->initial_rate_control_reorder_queue_head_index;
    for (frame_idx = 0; frame_idx < picture_control_set_ptr->frames_in_sw; frame_idx++) {
        temporaryQueueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[inputQueueIndex];
        temp_picture_control_set_ptr = ((PictureParentControlSet*)(temporaryQueueEntryPtr->parent_pcs_wrapper_ptr)->object_ptr);

        //printf("kelvin ---> init_rc update_mc_flow_synthesizer curr picture_number=%d %d, inputQueueIndex=%d, temp picture_number=%d, temp decode_order=%d, frames_in_sw=%d\n", picture_control_set_ptr->picture_number, frame_idx, inputQueueIndex, temp_picture_control_set_ptr->picture_number, temp_picture_control_set_ptr->decode_order, picture_control_set_ptr->frames_in_sw);

        // sort to be decode order
        if(frame_idx == 0) {
            picture_control_set_array[0] = temp_picture_control_set_ptr;
        } else {
            for (i = 0; i < frame_idx; i++) {
                if (temp_picture_control_set_ptr->decode_order < picture_control_set_array[i]->decode_order) {
                    for (int32_t j = frame_idx; j > i; j--)
                        picture_control_set_array[j] = picture_control_set_array[j-1];
                    picture_control_set_array[i] = temp_picture_control_set_ptr;
                    break;
                }
            }
            if (i == frame_idx)
                picture_control_set_array[i] = temp_picture_control_set_ptr;
        }

        // Increment the inputQueueIndex Iterator
        inputQueueIndex = (inputQueueIndex == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : inputQueueIndex + 1;
    }

    for(frame_idx = picture_control_set_ptr->frames_in_sw - 1; frame_idx > 0; frame_idx--) {
        printf("kelvin ---> init_rc update_mc_flow_synthesizer frame_idx=%d, reordered picture_number=%d, decode_order=%d\n", frame_idx, picture_control_set_array[frame_idx]->picture_number, picture_control_set_array[frame_idx]->decode_order);
        //kelvinhack
        cutree_mc_flow_synthesizer(encode_context_ptr, sequence_control_set_ptr, picture_control_set_array, frame_idx, picture_control_set_ptr->frames_in_sw); // in decode order
#if 0
        if(picture_control_set_array[frame_idx]->picture_number == 14) {
            // kelvin print out mc_dep_dist and mc_dep_rate
            uint32_t picture_width_in_mb  = (picture_control_set_array[frame_idx]->enhanced_picture_ptr->width  + 16 - 1) / 16;
            uint32_t picture_height_in_mb = (picture_control_set_array[frame_idx]->enhanced_picture_ptr->height + 16 - 1) / 16;
            for (int mb_y = 0; mb_y < picture_height_in_mb; mb_y++) {
                if(mb_y != 0)
                    continue;
                else
                    printf("kelvin mc_flow_synthesizer mbline%d poc%d\n", mb_y, picture_control_set_array[frame_idx]->picture_number);
                for (int mb_x = 0; mb_x < picture_width_in_mb; mb_x++) {
                    OisMbResults *ois_mb_results_ptr = picture_control_set_array[frame_idx]->ois_mb_results[mb_y * picture_width_in_mb  + mb_x];
                    printf("%d %d\n", ois_mb_results_ptr->mc_dep_dist, ois_mb_results_ptr->mc_dep_rate);
                }
            }
        }
#endif
    }
#if 0
        int temp_frame_idx = 0;
        if(picture_control_set_array[temp_frame_idx]->picture_number == 0) {
            // kelvin print out mc_dep_dist and mc_dep_rate
            uint32_t picture_width_in_mb  = (picture_control_set_array[temp_frame_idx]->enhanced_picture_ptr->width  + 16 - 1) / 16;
            uint32_t picture_height_in_mb = (picture_control_set_array[temp_frame_idx]->enhanced_picture_ptr->height + 16 - 1) / 16;
            for (int mb_y = 0; mb_y < picture_height_in_mb; mb_y++) {
                if(mb_y != 0)
                    continue;
                else
                    printf("kelvin mc_flow_synthesizer mbline%d poc%d\n", mb_y, picture_control_set_array[temp_frame_idx]->picture_number);
                for (int mb_x = 0; mb_x < picture_width_in_mb; mb_x++) {
                    OisMbResults *ois_mb_results_ptr = picture_control_set_array[temp_frame_idx]->ois_mb_results[mb_y * picture_width_in_mb  + mb_x];
                    printf("%d %d\n", ois_mb_results_ptr->mc_dep_dist, ois_mb_results_ptr->mc_dep_rate);
                }
            }
        }
#endif

    return;
}

#endif
/************************************************
* Initial Rate Control Kernel
* The Initial Rate Control Process determines the initial bit budget for each
* picture depending on the data gathered in the Picture Analysis and Motion
* Analysis processes as well as the settings determined in the Picture Decision process.
* The Initial Rate Control process also employs a sliding window buffer to analyze multiple
* pictures if the delay is allowed.  Note that through this process, until the subsequent
* Picture Manager process, no reference picture data has been used.
* P.S. Temporal noise reduction is now performed in Initial Rate Control Process.
* In future we might decide to move it to Motion Analysis Process.
************************************************/
void* initial_rate_control_kernel(void *input_ptr)
{
    InitialRateControlContext       *context_ptr = (InitialRateControlContext*)input_ptr;
    PictureParentControlSet         *picture_control_set_ptr;
    PictureParentControlSet         *pictureControlSetPtrTemp;
    EncodeContext                   *encode_context_ptr;
    SequenceControlSet              *sequence_control_set_ptr;

    EbObjectWrapper                 *inputResultsWrapperPtr;
    MotionEstimationResults         *inputResultsPtr;

    EbObjectWrapper                 *outputResultsWrapperPtr;
    InitialRateControlResults       *outputResultsPtr;

    // Queue variables
    uint32_t                             queueEntryIndexTemp;
    uint32_t                             queueEntryIndexTemp2;
    InitialRateControlReorderEntry  *queueEntryPtr;

    EbBool                            moveSlideWondowFlag = EB_TRUE;
    EbBool                            end_of_sequence_flag = EB_TRUE;
    uint8_t                               frames_in_sw;
    uint8_t                               temporal_layer_index;
    EbObjectWrapper                  *reference_picture_wrapper_ptr;

    // Segments
    uint32_t                              segment_index;

    EbObjectWrapper                *output_stream_wrapper_ptr;

    for (;;) {
        // Get Input Full Object
        eb_get_full_object(
            context_ptr->motion_estimation_results_input_fifo_ptr,
            &inputResultsWrapperPtr);

        inputResultsPtr = (MotionEstimationResults*)inputResultsWrapperPtr->object_ptr;
        picture_control_set_ptr = (PictureParentControlSet*)inputResultsPtr->picture_control_set_wrapper_ptr->object_ptr;

        segment_index = inputResultsPtr->segment_index;

        // Set the segment mask
        SEGMENT_COMPLETION_MASK_SET(picture_control_set_ptr->me_segments_completion_mask, segment_index);

        // If the picture is complete, proceed
        if (SEGMENT_COMPLETION_MASK_TEST(picture_control_set_ptr->me_segments_completion_mask, picture_control_set_ptr->me_segments_total_count)) {
            sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
            encode_context_ptr = (EncodeContext*)sequence_control_set_ptr->encode_context_ptr;
            // Mark picture when global motion is detected using ME results
            //reset intraCodedEstimationLcu
            MeBasedGlobalMotionDetection(
                picture_control_set_ptr);
#if CUTREE_LA
            if(picture_control_set_ptr->picture_number == 0)
                printf("kelvin ---> input sqc look_ahead_distance=%d, enable_cutree_in_la=%d\n", sequence_control_set_ptr->static_config.look_ahead_distance, sequence_control_set_ptr->static_config.enable_cutree_in_la);
            if (sequence_control_set_ptr->static_config.look_ahead_distance == 0 || sequence_control_set_ptr->static_config.enable_cutree_in_la == 0) {
                // Release Pa Ref pictures when not needed
                ReleasePaReferenceObjects(
                    sequence_control_set_ptr,
                    picture_control_set_ptr);
            }
            if (sequence_control_set_ptr->static_config.look_ahead_distance != 0 && sequence_control_set_ptr->static_config.enable_cutree_in_la) {
                //if (picture_control_set_ptr->slice_type != I_SLICE)
                    //kelvinhack
                    cutree_mc_flow_dispenser(encode_context_ptr, sequence_control_set_ptr, picture_control_set_ptr);
            }
            printf("kelvin ---> init_rc picture_number=%d to releasePaReference, look_ahead_distance=%d\n", picture_control_set_ptr->picture_number, sequence_control_set_ptr->static_config.look_ahead_distance);
#else
            // Release Pa Ref pictures when not needed
            ReleasePaReferenceObjects(
                sequence_control_set_ptr,
                picture_control_set_ptr);
#endif

            //****************************************************
            // Input Motion Analysis Results into Reordering Queue
            //****************************************************

            if(!picture_control_set_ptr->is_overlay)
            // Determine offset from the Head Ptr
            queueEntryPtr = DeterminePictureOffsetInQueue(
                encode_context_ptr,
                picture_control_set_ptr,
                inputResultsPtr);

            if (sequence_control_set_ptr->static_config.rate_control_mode)
            {
                if (sequence_control_set_ptr->static_config.look_ahead_distance != 0) {
                    // Getting the Histogram Queue Data
                    GetHistogramQueueData(
                        sequence_control_set_ptr,
                        encode_context_ptr,
                        picture_control_set_ptr);
                }
            }

            for (temporal_layer_index = 0; temporal_layer_index < EB_MAX_TEMPORAL_LAYERS; temporal_layer_index++)
                picture_control_set_ptr->frames_in_interval[temporal_layer_index] = 0;
            picture_control_set_ptr->frames_in_sw = 0;
            picture_control_set_ptr->historgram_life_count = 0;
            picture_control_set_ptr->scene_change_in_gop = EB_FALSE;

            //Check conditions for statinary edge over time

            StationaryEdgeOverUpdateOverTimeLcuPart1(
                sequence_control_set_ptr,
                picture_control_set_ptr);

            moveSlideWondowFlag = EB_TRUE;
            while (moveSlideWondowFlag) {
                // Check if the sliding window condition is valid
                queueEntryIndexTemp = encode_context_ptr->initial_rate_control_reorder_queue_head_index;
                if (encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndexTemp]->parent_pcs_wrapper_ptr != EB_NULL)
                    end_of_sequence_flag = (((PictureParentControlSet*)(encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndexTemp]->parent_pcs_wrapper_ptr)->object_ptr))->end_of_sequence_flag;
                else
                    end_of_sequence_flag = EB_FALSE;
                frames_in_sw = 0;
                while (moveSlideWondowFlag && !end_of_sequence_flag &&
                    queueEntryIndexTemp <= encode_context_ptr->initial_rate_control_reorder_queue_head_index + sequence_control_set_ptr->static_config.look_ahead_distance) {
                    // frames_in_sw <= sequence_control_set_ptr->static_config.look_ahead_distance){
                    frames_in_sw++;

                    queueEntryIndexTemp2 = (queueEntryIndexTemp > INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? queueEntryIndexTemp - INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH : queueEntryIndexTemp;

                    moveSlideWondowFlag = (EbBool)(moveSlideWondowFlag && (encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndexTemp2]->parent_pcs_wrapper_ptr != EB_NULL));
                    if (encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndexTemp2]->parent_pcs_wrapper_ptr != EB_NULL) {
                        // check if it is the last frame. If we have reached the last frame, we would output the buffered frames in the Queue.
                        end_of_sequence_flag = ((PictureParentControlSet*)(encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndexTemp2]->parent_pcs_wrapper_ptr)->object_ptr)->end_of_sequence_flag;
                    }
                    else
                        end_of_sequence_flag = EB_FALSE;
                    queueEntryIndexTemp++;
                }

                if (moveSlideWondowFlag) {
                    //get a new entry spot
                    queueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[encode_context_ptr->initial_rate_control_reorder_queue_head_index];
                    picture_control_set_ptr = ((PictureParentControlSet*)(queueEntryPtr->parent_pcs_wrapper_ptr)->object_ptr);
                    sequence_control_set_ptr = (SequenceControlSet*)picture_control_set_ptr->sequence_control_set_wrapper_ptr->object_ptr;
                    // overlay picture was not added to the queue. For the alt_ref picture with an overlay picture, it loops on both alt ref and overlay pictures
                    uint8_t has_overlay = picture_control_set_ptr->is_alt_ref ? 1 : 0;
                    for (uint8_t loop_index = 0; loop_index <= has_overlay; loop_index++) {
                        if (loop_index)
                            picture_control_set_ptr = picture_control_set_ptr->overlay_ppcs_ptr;
                        picture_control_set_ptr->frames_in_sw = frames_in_sw;
                        queueEntryIndexTemp = encode_context_ptr->initial_rate_control_reorder_queue_head_index;
                        end_of_sequence_flag = EB_FALSE;
                        // find the frames_in_interval for the peroid I frames
                        while (!end_of_sequence_flag &&
                            queueEntryIndexTemp <= encode_context_ptr->initial_rate_control_reorder_queue_head_index + sequence_control_set_ptr->static_config.look_ahead_distance) {
                            queueEntryIndexTemp2 = (queueEntryIndexTemp > INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? queueEntryIndexTemp - INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH : queueEntryIndexTemp;
                            pictureControlSetPtrTemp = ((PictureParentControlSet*)(encode_context_ptr->initial_rate_control_reorder_queue[queueEntryIndexTemp2]->parent_pcs_wrapper_ptr)->object_ptr);
                            if (sequence_control_set_ptr->intra_period_length != -1) {
                                if (picture_control_set_ptr->picture_number % ((sequence_control_set_ptr->intra_period_length + 1)) == 0) {
                                    picture_control_set_ptr->frames_in_interval[pictureControlSetPtrTemp->temporal_layer_index] ++;
                                    if (pictureControlSetPtrTemp->scene_change_flag)
                                        picture_control_set_ptr->scene_change_in_gop = EB_TRUE;
                                }
                            }

                            pictureControlSetPtrTemp->historgram_life_count++;
                            end_of_sequence_flag = pictureControlSetPtrTemp->end_of_sequence_flag;
                            queueEntryIndexTemp++;
                        }

                        if ((sequence_control_set_ptr->static_config.look_ahead_distance != 0) && (frames_in_sw < (sequence_control_set_ptr->static_config.look_ahead_distance + 1)))
                            picture_control_set_ptr->end_of_sequence_region = EB_TRUE;
                        else
                            picture_control_set_ptr->end_of_sequence_region = EB_FALSE;

                        if (sequence_control_set_ptr->static_config.rate_control_mode)
                        {
                            // Determine offset from the Head Ptr for HLRC histogram queue and set the life count
                            if (sequence_control_set_ptr->static_config.look_ahead_distance != 0) {
                                // Update Histogram Queue Entry Life count
                                UpdateHistogramQueueEntry(
                                    sequence_control_set_ptr,
                                    encode_context_ptr,
                                    picture_control_set_ptr,
                                    frames_in_sw);
                            }
                        }

                        // Mark each input picture as PAN or not
                        // If a lookahead is present then check PAN for a period of time
                        if (!picture_control_set_ptr->end_of_sequence_flag && sequence_control_set_ptr->static_config.look_ahead_distance != 0) {
                            // Check for Pan,Tilt, Zoom and other global motion detectors over the future pictures in the lookahead
                            UpdateGlobalMotionDetectionOverTime(
                                encode_context_ptr,
                                sequence_control_set_ptr,
                                picture_control_set_ptr);
                        }
                        else {
                            if (picture_control_set_ptr->slice_type != I_SLICE)
                                DetectGlobalMotion(picture_control_set_ptr);
                        }

                        // BACKGROUND ENHANCEMENT PART II
                        if (!picture_control_set_ptr->end_of_sequence_flag && sequence_control_set_ptr->static_config.look_ahead_distance != 0) {
                            // Update BEA information based on Lookahead information
                            UpdateBeaInfoOverTime(
                                encode_context_ptr,
                                picture_control_set_ptr);
                        }
                        else {
                            // Reset zzCost information to default When there's no lookahead available
                            InitZzCostInfo(
                                picture_control_set_ptr);
                        }

                        // Use the temporal layer 0 isLcuMotionFieldNonUniform array for all the other layer pictures in the mini GOP
                        if (!picture_control_set_ptr->end_of_sequence_flag && sequence_control_set_ptr->static_config.look_ahead_distance != 0) {
                            // Updat uniformly moving LCUs based on Collocated LCUs in LookAhead window
                            UpdateMotionFieldUniformityOverTime(
                                encode_context_ptr,
                                sequence_control_set_ptr,
                                picture_control_set_ptr);
                        }
                        // Derive blockinessPresentFlag
                        DeriveBlockinessPresentFlag(
                            sequence_control_set_ptr,
                            picture_control_set_ptr);

                        // Get Empty Reference Picture Object
                        eb_get_empty_object(
                            sequence_control_set_ptr->encode_context_ptr->reference_picture_pool_fifo_ptr,
                            &reference_picture_wrapper_ptr);
                        if (loop_index) {
                            picture_control_set_ptr->reference_picture_wrapper_ptr = reference_picture_wrapper_ptr;
                            // Give the new Reference a nominal live_count of 1
                            eb_object_inc_live_count(
                                picture_control_set_ptr->reference_picture_wrapper_ptr,
                                1);
                        }
                        else {
                            ((PictureParentControlSet*)(queueEntryPtr->parent_pcs_wrapper_ptr->object_ptr))->reference_picture_wrapper_ptr = reference_picture_wrapper_ptr;
                            // Give the new Reference a nominal live_count of 1
                            eb_object_inc_live_count(
                                ((PictureParentControlSet*)(queueEntryPtr->parent_pcs_wrapper_ptr->object_ptr))->reference_picture_wrapper_ptr,
                                1);
                        }
#if TWO_PASS
                        picture_control_set_ptr->stat_struct_first_pass_ptr = picture_control_set_ptr->is_used_as_reference_flag ? &((EbReferenceObject*)picture_control_set_ptr->reference_picture_wrapper_ptr->object_ptr)->stat_struct : &picture_control_set_ptr->stat_struct;
                        if (sequence_control_set_ptr->use_output_stat_file)
                            memset(picture_control_set_ptr->stat_struct_first_pass_ptr, 0, sizeof(stat_struct_t));
#endif
#if CUTREE_LA
                        if (sequence_control_set_ptr->static_config.look_ahead_distance != 0 &&
                            sequence_control_set_ptr->static_config.enable_cutree_in_la &&
                            picture_control_set_ptr->temporal_layer_index == 0) {
                            update_mc_flow_synthesizer(encode_context_ptr, sequence_control_set_ptr, picture_control_set_ptr);
                        }
#endif

                        //OPTION 1:  get the output stream buffer in ressource coordination
                        eb_get_empty_object(
                            sequence_control_set_ptr->encode_context_ptr->stream_output_fifo_ptr,
                            &output_stream_wrapper_ptr);

                        picture_control_set_ptr->output_stream_wrapper_ptr = output_stream_wrapper_ptr;

                        // Get Empty Results Object
                        eb_get_empty_object(
                            context_ptr->initialrate_control_results_output_fifo_ptr,
                            &outputResultsWrapperPtr);

                        outputResultsPtr = (InitialRateControlResults*)outputResultsWrapperPtr->object_ptr;

                        if (loop_index)
                            outputResultsPtr->picture_control_set_wrapper_ptr = picture_control_set_ptr->p_pcs_wrapper_ptr;
                        else
                        outputResultsPtr->picture_control_set_wrapper_ptr = queueEntryPtr->parent_pcs_wrapper_ptr;
#if CUTREE_LA
printf("kelvin ---> loop_index=%d, output picture_number=%d\n", loop_index, loop_index ? picture_control_set_ptr->picture_number : queueEntryPtr->picture_number );
                        if (sequence_control_set_ptr->static_config.look_ahead_distance != 0 && sequence_control_set_ptr->static_config.enable_cutree_in_la
                            && ((has_overlay == 0 && loop_index == 0) || (has_overlay == 1 && loop_index == 1))) {
                            // Release Pa Ref pictures when not needed
                            ReleasePaReferenceObjects(
                                sequence_control_set_ptr,
                                picture_control_set_ptr);
                                //loop_index ? picture_control_set_ptr : queueEntryPtr);
                        }
#endif
                        // Post the Full Results Object
                        eb_post_full_object(outputResultsWrapperPtr);
                    }
                    // Reset the Reorder Queue Entry
                    queueEntryPtr->picture_number += INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH;
                    queueEntryPtr->parent_pcs_wrapper_ptr = (EbObjectWrapper *)EB_NULL;

                    // Increment the Reorder Queue head Ptr
                    encode_context_ptr->initial_rate_control_reorder_queue_head_index =
                        (encode_context_ptr->initial_rate_control_reorder_queue_head_index == INITIAL_RATE_CONTROL_REORDER_QUEUE_MAX_DEPTH - 1) ? 0 : encode_context_ptr->initial_rate_control_reorder_queue_head_index + 1;

                    queueEntryPtr = encode_context_ptr->initial_rate_control_reorder_queue[encode_context_ptr->initial_rate_control_reorder_queue_head_index];
                }
            }
        }

        // Release the Input Results
        eb_release_object(inputResultsWrapperPtr);
    }
    return EB_NULL;
}
