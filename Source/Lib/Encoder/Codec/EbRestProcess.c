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

#include "EbEncHandle.h"
#include "EbRestProcess.h"
#include "EbEncDecResults.h"
#include "EbThreads.h"
#include "EbPictureDemuxResults.h"
#include "EbPsnr.h"
#include "EbReferenceObject.h"
#include "EbPictureControlSet.h"

/**************************************
 * Rest Context
 **************************************/
typedef struct RestContext {
    EbDctor dctor;
    EbFifo *rest_input_fifo_ptr;
    EbFifo *rest_output_fifo_ptr;
    EbFifo *picture_demux_fifo_ptr;

    EbPictureBufferDesc *trial_frame_rst;

    EbPictureBufferDesc *temp_lf_recon_picture_ptr;
    EbPictureBufferDesc *temp_lf_recon_picture16bit_ptr;

    EbPictureBufferDesc *
        org_rec_frame; // while doing the filtering recon gets updated uisng setup/restore processing_stripe_bounadaries
        // many threads doing the above will result in race condition.
        // each thread will hence have his own copy of recon to work on.
        // later we can have a search version that does not need the exact right recon
    int32_t *rst_tmpbuf;
} RestContext;

void recon_output(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr);
void eb_av1_loop_restoration_filter_frame(Yv12BufferConfig *frame, Av1Common *cm,
                                          int32_t optimized_lr);
void copy_statistics_to_ref_obj_ect(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr);
void psnr_calculations(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr);
void pad_ref_and_set_flags(PictureControlSet *pcs_ptr, SequenceControlSet *scs_ptr);
void generate_padding(EbByte src_pic, uint32_t src_stride, uint32_t original_src_width,
                      uint32_t original_src_height, uint32_t padding_width,
                      uint32_t padding_height);
void restoration_seg_search(int32_t *rst_tmpbuf, Yv12BufferConfig *org_fts,
                            const Yv12BufferConfig *src, Yv12BufferConfig *trial_frame_rst,
                            PictureControlSet *pcs_ptr, uint32_t segment_index);
void rest_finish_search(Macroblock *x, Av1Common *const cm);

void av1_upscale_normative_rows(const Av1Common *cm, const uint8_t *src,
                                int src_stride, uint8_t *dst, int dst_stride, int rows, int sub_x, int bd);

// delete - for debug purposes
void save_YUV_to_file(char *filename, EbByte buffer_y, EbByte buffer_u, EbByte buffer_v,
                      uint16_t width, uint16_t height,
                      uint16_t stride_y, uint16_t stride_u, uint16_t stride_v,
                      uint16_t origin_y, uint16_t origin_x,
                      uint32_t ss_x, uint32_t ss_y);

static void rest_context_dctor(EbPtr p)
{
    EbThreadContext *thread_context_ptr = (EbThreadContext *)p;
    RestContext *    obj                = (RestContext *)thread_context_ptr->priv;
    EB_DELETE(obj->temp_lf_recon_picture_ptr);
    EB_DELETE(obj->temp_lf_recon_picture16bit_ptr);
    EB_DELETE(obj->trial_frame_rst);
    EB_DELETE(obj->org_rec_frame);
    EB_FREE_ALIGNED(obj->rst_tmpbuf);
    EB_FREE_ARRAY(obj);
}

/******************************************************
 * Rest Context Constructor
 ******************************************************/
EbErrorType rest_context_ctor(EbThreadContext *  thread_context_ptr,
                              const EbEncHandle *enc_handle_ptr, int index, int demux_index) {
    const SequenceControlSet *      scs_ptr      = enc_handle_ptr->scs_instance_array[0]->scs_ptr;
    const EbSvtAv1EncConfiguration *config       = &scs_ptr->static_config;
    EbBool                          is_16bit     = (EbBool)(config->encoder_bit_depth > EB_8BIT);
    EbColorFormat                   color_format = config->encoder_color_format;

    RestContext *context_ptr;
    EB_CALLOC_ARRAY(context_ptr, 1);
    thread_context_ptr->priv  = context_ptr;
    thread_context_ptr->dctor = rest_context_dctor;

    // Input/Output System Resource Manager FIFOs
    context_ptr->rest_input_fifo_ptr =
        eb_system_resource_get_consumer_fifo(enc_handle_ptr->cdef_results_resource_ptr, index);
    context_ptr->rest_output_fifo_ptr =
        eb_system_resource_get_producer_fifo(enc_handle_ptr->rest_results_resource_ptr, index);
    context_ptr->picture_demux_fifo_ptr = eb_system_resource_get_producer_fifo(
        enc_handle_ptr->picture_demux_results_resource_ptr, demux_index);

    {
        EbPictureBufferDescInitData init_data;

        init_data.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;
        init_data.max_width          = (uint16_t)scs_ptr->max_input_luma_width;
        init_data.max_height         = (uint16_t)scs_ptr->max_input_luma_height;
        init_data.bit_depth          = is_16bit ? EB_16BIT : EB_8BIT;
        init_data.color_format       = color_format;
        init_data.left_padding       = AOM_BORDER_IN_PIXELS;
        init_data.right_padding      = AOM_BORDER_IN_PIXELS;
        init_data.top_padding        = AOM_BORDER_IN_PIXELS;
        init_data.bot_padding        = AOM_BORDER_IN_PIXELS;
        init_data.split_mode         = EB_FALSE;

        EB_NEW(context_ptr->trial_frame_rst, eb_picture_buffer_desc_ctor, (EbPtr)&init_data);

        EB_NEW(context_ptr->org_rec_frame, eb_picture_buffer_desc_ctor, (EbPtr)&init_data);

        EB_MALLOC_ALIGNED(context_ptr->rst_tmpbuf, RESTORATION_TMPBUF_SIZE);
    }

    EbPictureBufferDescInitData temp_lf_recon_desc_init_data;
    temp_lf_recon_desc_init_data.max_width          = (uint16_t)scs_ptr->max_input_luma_width;
    temp_lf_recon_desc_init_data.max_height         = (uint16_t)scs_ptr->max_input_luma_height;
    temp_lf_recon_desc_init_data.buffer_enable_mask = PICTURE_BUFFER_DESC_FULL_MASK;

    temp_lf_recon_desc_init_data.left_padding  = PAD_VALUE;
    temp_lf_recon_desc_init_data.right_padding = PAD_VALUE;
    temp_lf_recon_desc_init_data.top_padding   = PAD_VALUE;
    temp_lf_recon_desc_init_data.bot_padding   = PAD_VALUE;
    temp_lf_recon_desc_init_data.split_mode    = EB_FALSE;
    temp_lf_recon_desc_init_data.color_format  = color_format;

    if (is_16bit) {
        temp_lf_recon_desc_init_data.bit_depth = EB_16BIT;
        EB_NEW(context_ptr->temp_lf_recon_picture16bit_ptr,
               eb_recon_picture_buffer_desc_ctor,
               (EbPtr)&temp_lf_recon_desc_init_data);
    } else {
        temp_lf_recon_desc_init_data.bit_depth = EB_8BIT;
        EB_NEW(context_ptr->temp_lf_recon_picture_ptr,
               eb_recon_picture_buffer_desc_ctor,
               (EbPtr)&temp_lf_recon_desc_init_data);
    }

    return EB_ErrorNone;
}
void get_own_recon(SequenceControlSet *scs_ptr, PictureControlSet *pcs_ptr,
                   RestContext *context_ptr, EbBool is_16bit) {
    EbPictureBufferDesc *recon_picture_ptr;
    if (is_16bit) {
        if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
            recon_picture_ptr =
                ((EbReferenceObject *)
                     pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                    ->reference_picture16bit;
        else
            recon_picture_ptr = pcs_ptr->recon_picture16bit_ptr;

        uint16_t *rec_ptr = (uint16_t *)recon_picture_ptr->buffer_y + recon_picture_ptr->origin_x +
                            recon_picture_ptr->origin_y * recon_picture_ptr->stride_y;
        uint16_t *rec_ptr_cb = (uint16_t *)recon_picture_ptr->buffer_cb +
                               recon_picture_ptr->origin_x / 2 +
                               recon_picture_ptr->origin_y / 2 * recon_picture_ptr->stride_cb;
        uint16_t *rec_ptr_cr = (uint16_t *)recon_picture_ptr->buffer_cr +
                               recon_picture_ptr->origin_x / 2 +
                               recon_picture_ptr->origin_y / 2 * recon_picture_ptr->stride_cr;

        EbPictureBufferDesc *org_rec = context_ptr->org_rec_frame;
        uint16_t *           org_ptr = (uint16_t *)org_rec->buffer_y + org_rec->origin_x +
                            org_rec->origin_y * org_rec->stride_y;
        uint16_t *org_ptr_cb = (uint16_t *)org_rec->buffer_cb + org_rec->origin_x / 2 +
                               org_rec->origin_y / 2 * org_rec->stride_cb;
        uint16_t *org_ptr_cr = (uint16_t *)org_rec->buffer_cr + org_rec->origin_x / 2 +
                               org_rec->origin_y / 2 * org_rec->stride_cr;

        for (int r = 0; r < scs_ptr->seq_header.max_frame_height; ++r)
            memcpy(org_ptr + r * org_rec->stride_y,
                   rec_ptr + r * recon_picture_ptr->stride_y,
                   scs_ptr->seq_header.max_frame_width << 1);

        for (int r = 0; r < scs_ptr->seq_header.max_frame_height / 2; ++r) {
            memcpy(org_ptr_cb + r * org_rec->stride_cb,
                   rec_ptr_cb + r * recon_picture_ptr->stride_cb,
                   (scs_ptr->seq_header.max_frame_width / 2) << 1);
            memcpy(org_ptr_cr + r * org_rec->stride_cr,
                   rec_ptr_cr + r * recon_picture_ptr->stride_cr,
                   (scs_ptr->seq_header.max_frame_width / 2) << 1);
        }
    } else {
        if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
            recon_picture_ptr =
                ((EbReferenceObject *)
                     pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)
                    ->reference_picture;
        else
            recon_picture_ptr = pcs_ptr->recon_picture_ptr;

        uint8_t *rec_ptr =
            &((recon_picture_ptr
                   ->buffer_y)[recon_picture_ptr->origin_x +
                               recon_picture_ptr->origin_y * recon_picture_ptr->stride_y]);
        uint8_t *rec_ptr_cb =
            &((recon_picture_ptr
                   ->buffer_cb)[recon_picture_ptr->origin_x / 2 +
                                recon_picture_ptr->origin_y / 2 * recon_picture_ptr->stride_cb]);
        uint8_t *rec_ptr_cr =
            &((recon_picture_ptr
                   ->buffer_cr)[recon_picture_ptr->origin_x / 2 +
                                recon_picture_ptr->origin_y / 2 * recon_picture_ptr->stride_cr]);

        EbPictureBufferDesc *org_rec = context_ptr->org_rec_frame;
        uint8_t *            org_ptr =
            &((org_rec->buffer_y)[org_rec->origin_x + org_rec->origin_y * org_rec->stride_y]);
        uint8_t *org_ptr_cb = &((org_rec->buffer_cb)[org_rec->origin_x / 2 +
                                                     org_rec->origin_y / 2 * org_rec->stride_cb]);
        uint8_t *org_ptr_cr = &((org_rec->buffer_cr)[org_rec->origin_x / 2 +
                                                     org_rec->origin_y / 2 * org_rec->stride_cr]);

        for (int r = 0; r < scs_ptr->seq_header.max_frame_height; ++r)
            memcpy(org_ptr + r * org_rec->stride_y,
                   rec_ptr + r * recon_picture_ptr->stride_y,
                   scs_ptr->seq_header.max_frame_width);

        for (int r = 0; r < scs_ptr->seq_header.max_frame_height / 2; ++r) {
            memcpy(org_ptr_cb + r * org_rec->stride_cb,
                   rec_ptr_cb + r * recon_picture_ptr->stride_cb,
                   (scs_ptr->seq_header.max_frame_width / 2));
            memcpy(org_ptr_cr + r * org_rec->stride_cr,
                   rec_ptr_cr + r * recon_picture_ptr->stride_cr,
                   (scs_ptr->seq_header.max_frame_width / 2));
        }
    }
}

void derive_blk_pointers_enc(EbPictureBufferDesc *recon_picture_buf, int32_t plane,
                             int32_t blk_col_px, int32_t blk_row_px,
                             void **pp_blk_recon_buf, int32_t *recon_stride,
                             int32_t sub_x, int32_t sub_y)
{
    int32_t block_offset;

    if (plane == 0) {
        block_offset = (recon_picture_buf->origin_y + blk_row_px) *
                       recon_picture_buf->stride_y + (recon_picture_buf->origin_x +
                                                      blk_col_px);
        *recon_stride = recon_picture_buf->stride_y;
    }
    else if (plane == 1) {
        block_offset = ((recon_picture_buf->origin_y >> sub_y) +
                        blk_row_px) * recon_picture_buf->stride_cb +
                       ((recon_picture_buf->origin_x >> sub_x) + blk_col_px);
        *recon_stride = recon_picture_buf->stride_cb;
    }
    else {
        block_offset = ((recon_picture_buf->origin_y >> sub_y) +
                        blk_row_px) * recon_picture_buf->stride_cr +
                       ((recon_picture_buf->origin_x >> sub_x) + blk_col_px);
        *recon_stride = recon_picture_buf->stride_cr;
    }

    if (recon_picture_buf->bit_depth != EB_8BIT) {//16bit
        if (plane == 0)
            *pp_blk_recon_buf = (void *)((uint16_t*)recon_picture_buf->buffer_y
                                         + block_offset);
        else if (plane == 1)
            *pp_blk_recon_buf = (void *)((uint16_t*)recon_picture_buf->buffer_cb
                                         + block_offset);
        else
            *pp_blk_recon_buf = (void *)((uint16_t*)recon_picture_buf->buffer_cr
                                         + block_offset);
    }
    else {
        if (plane == 0)
            *pp_blk_recon_buf = (void *)((uint8_t*)recon_picture_buf->buffer_y
                                         + block_offset);
        else if (plane == 1)
            *pp_blk_recon_buf = (void *)((uint8_t*)recon_picture_buf->buffer_cb
                                         + block_offset);
        else
            *pp_blk_recon_buf = (void *)((uint8_t*)recon_picture_buf->buffer_cr
                                         + block_offset);
    }
}

EbErrorType copy_recon_enc(SequenceControlSet *sequence_control_set_ptr,
                           EbPictureBufferDesc*recon_picture_src,
                           EbPictureBufferDesc *recon_picture_dst,
                           int num_planes,
                           int skip_copy){

    recon_picture_dst->origin_x      = recon_picture_src->origin_x;
    recon_picture_dst->origin_y      = recon_picture_src->origin_y;
    recon_picture_dst->width         = recon_picture_src->width;
    recon_picture_dst->height        = recon_picture_src->height;
    recon_picture_dst->max_width     = recon_picture_src->max_width;
    recon_picture_dst->max_height    = recon_picture_src->max_height;
    recon_picture_dst->bit_depth     = recon_picture_src->bit_depth;
    recon_picture_dst->color_format  = recon_picture_src->color_format;

    recon_picture_dst->stride_y  = recon_picture_src->stride_y;
    recon_picture_dst->stride_cb = recon_picture_src->stride_cb;
    recon_picture_dst->stride_cr = recon_picture_src->stride_cr;

    recon_picture_dst->luma_size    = recon_picture_src->luma_size;
    recon_picture_dst->chroma_size  = recon_picture_src->chroma_size;
    recon_picture_dst->packedFlag   = recon_picture_src->packedFlag;

    recon_picture_dst->stride_bit_inc_y = recon_picture_src->stride_bit_inc_y;
    recon_picture_dst->stride_bit_inc_cb = recon_picture_src->stride_bit_inc_cb;
    recon_picture_dst->stride_bit_inc_cr = recon_picture_src->stride_bit_inc_cr;

    recon_picture_dst->buffer_enable_mask = sequence_control_set_ptr->seq_header.color_config.mono_chrome ?
                                            PICTURE_BUFFER_DESC_LUMA_MASK : PICTURE_BUFFER_DESC_FULL_MASK;

    uint32_t bytesPerPixel = (recon_picture_dst->bit_depth == EB_8BIT) ? 1 : 2;

    // Allocate the Picture Buffers (luma & chroma)
    if (recon_picture_dst->buffer_enable_mask & PICTURE_BUFFER_DESC_Y_FLAG) {
        EB_MALLOC_ALIGNED(recon_picture_dst->buffer_y, recon_picture_dst->luma_size * bytesPerPixel);
        memset(recon_picture_dst->buffer_y, 0,
               recon_picture_dst->luma_size * bytesPerPixel);
    }
    else
        recon_picture_dst->buffer_y = 0;
    if (recon_picture_dst->buffer_enable_mask & PICTURE_BUFFER_DESC_Cb_FLAG) {
        EB_MALLOC_ALIGNED(recon_picture_dst->buffer_cb, recon_picture_dst->chroma_size * bytesPerPixel);
        memset(recon_picture_dst->buffer_cb, 0,
               recon_picture_dst->chroma_size * bytesPerPixel);
    }
    else
        recon_picture_dst->buffer_cb = 0;
    if (recon_picture_dst->buffer_enable_mask & PICTURE_BUFFER_DESC_Cr_FLAG) {
        EB_MALLOC_ALIGNED(recon_picture_dst->buffer_cr, recon_picture_dst->chroma_size * bytesPerPixel);
        memset(recon_picture_dst->buffer_cr, 0,
               recon_picture_dst->chroma_size * bytesPerPixel);
    }
    else
        recon_picture_dst->buffer_cr = 0;

    int use_highbd = (sequence_control_set_ptr->static_config.encoder_bit_depth > 8);

    if(!skip_copy){
        for (int plane = 0; plane < num_planes; ++plane) {
            uint8_t *src_buf, *dst_buf;
            int32_t src_stride, dst_stride;

            int sub_x = plane ? sequence_control_set_ptr->subsampling_x : 0;
            int sub_y = plane ? sequence_control_set_ptr->subsampling_y : 0;

            derive_blk_pointers_enc(recon_picture_src, plane, 0, 0, (void *)&src_buf,
                                    &src_stride, sub_x, sub_y);
            derive_blk_pointers_enc(recon_picture_dst, plane, 0, 0, (void *)&dst_buf,
                                    &dst_stride, sub_x, sub_y);

            int height = (recon_picture_src->height >> sub_y);
            for (int row = 0; row < height; ++row) {
                memcpy(dst_buf, src_buf, (recon_picture_src->width >> sub_x) *
                                         sizeof(*src_buf) << use_highbd);
                src_buf += src_stride << use_highbd;
                dst_buf += dst_stride << use_highbd;
            }
        }
    }

    return EB_ErrorNone;
}

// 8 bit downsample with only 2x1, phase-0, filter
void downsample_width(const uint8_t *input_samples,      // input parameter, input samples Ptr
                      uint32_t input_stride,       // input parameter, input stride
                      uint32_t input_area_width,   // input parameter, input area width
                      uint32_t input_area_height,  // input parameter, input area height
                      uint8_t *decim_samples,      // output parameter, decimated samples Ptr
                      uint32_t decim_stride,       // input parameter, output stride
                      uint32_t decim_step){         // input parameter, decimation amount in pixels
    uint32_t horizontal_index;
    uint32_t vertical_index;
    uint32_t decim_horizontal_index;
    const uint32_t half_decim_step = decim_step >> 1;

    for (vertical_index = 0; vertical_index < input_area_height; vertical_index++) {
        for (horizontal_index = half_decim_step, decim_horizontal_index = 0; horizontal_index < input_area_width; horizontal_index += decim_step, decim_horizontal_index++) {
            uint32_t sum = (uint32_t)input_samples[horizontal_index - 1] + (uint32_t)input_samples[horizontal_index];
            decim_samples[decim_horizontal_index] = (uint8_t)((sum + 1) >> 1);
        }
        input_samples += input_stride;
        decim_samples += decim_stride;
    }
}

void downsample_width_YUV(SequenceControlSet *sequence_control_set_ptr,
                          EbPictureBufferDesc *src_ptr,
                          EbPictureBufferDesc *dst_ptr){
    uint16_t ss_x = sequence_control_set_ptr->subsampling_x;
    uint16_t ss_y = sequence_control_set_ptr->subsampling_y;

    downsample_width(
            &src_ptr->buffer_y[src_ptr->origin_x + src_ptr->origin_y * src_ptr->stride_y],
            src_ptr->stride_y,
            src_ptr->width,
            src_ptr->height,
            &dst_ptr->buffer_y[dst_ptr->origin_x + dst_ptr->origin_y*dst_ptr->stride_y],
            dst_ptr->stride_y,
            2);

    downsample_width(
            &src_ptr->buffer_cb[(src_ptr->origin_x>>ss_x) + (src_ptr->origin_y>>ss_y) * src_ptr->stride_cb],
            src_ptr->stride_cb,
            src_ptr->width >> ss_x,
            src_ptr->height >> ss_y,
            &dst_ptr->buffer_cb[(dst_ptr->origin_x>>ss_x) + (dst_ptr->origin_y>>ss_y)*dst_ptr->stride_cb],
            dst_ptr->stride_cb,
            2);

    downsample_width(
            &src_ptr->buffer_cr[(src_ptr->origin_x>>ss_x) + (src_ptr->origin_y>>ss_y) * src_ptr->stride_cr],
            src_ptr->stride_cr,
            src_ptr->width >> ss_x,
            src_ptr->height >> ss_y,
            &dst_ptr->buffer_cr[(dst_ptr->origin_x>>ss_x) + (dst_ptr->origin_y>>ss_y)*dst_ptr->stride_cr],
            dst_ptr->stride_cr,
            2);
}

void replace_recon_pic(EbPictureBufferDesc *recon_ptr,
                   PictureControlSet *picture_control_set_ptr){
    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
        ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture = recon_ptr;
    else
        picture_control_set_ptr->recon_picture_ptr = recon_ptr;
}

void get_recon_pic(PictureControlSet *picture_control_set_ptr,
               EbPictureBufferDesc **recon_ptr){
    if (picture_control_set_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
        *recon_ptr = ((EbReferenceObject*)picture_control_set_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr->object_ptr)->reference_picture;
    else
        *recon_ptr = picture_control_set_ptr->recon_picture_ptr;
}

// This function applies the normative upscaling of the recontructed picture if downscaling was used before encoding
void downscale_recon_for_test(EbPictureBufferDesc *recon_ptr_half,
                              PictureControlSet *picture_control_set_ptr,
                              SequenceControlSet *sequence_control_set_ptr){
    // Set these parameters for testing since they are not correctly populated yet
    EbPictureBufferDesc *recon_ptr;
    const int num_planes = sequence_control_set_ptr->seq_header.color_config.mono_chrome ? 1 : MAX_MB_PLANE;

    get_recon_pic(picture_control_set_ptr,
              &recon_ptr);

    EbErrorType return_error = copy_recon_enc(sequence_control_set_ptr, recon_ptr,
                                              recon_ptr_half, num_planes, 1);
    if (return_error != EB_ErrorNone) {
        recon_ptr_half = NULL;
        assert(0);
    }

    downsample_width_YUV(sequence_control_set_ptr, recon_ptr, recon_ptr_half);

    // debug only
    save_YUV_to_file("recon_640x720.yuv", recon_ptr_half->buffer_y, recon_ptr_half->buffer_cb, recon_ptr_half->buffer_cr,
                     recon_ptr_half->width >> 1, recon_ptr_half->height,
                     recon_ptr_half->stride_y, recon_ptr_half->stride_cb, recon_ptr_half->stride_cr,
                     recon_ptr_half->origin_y, recon_ptr_half->origin_x,
                     1, 1);

    replace_recon_pic(recon_ptr_half,
                  picture_control_set_ptr);


}

void eb_av1_superres_upscale_frame(struct Av1Common *cm,
                                   PictureControlSet *picture_control_set_ptr,
                                   SequenceControlSet *sequence_control_set_ptr)
{
    // Set these parameters for testing since they are not correctly populated yet
    EbPictureBufferDesc *recon_ptr;

    get_recon_pic(picture_control_set_ptr,
              &recon_ptr);

    uint16_t ss_x = sequence_control_set_ptr->subsampling_x;
    uint16_t ss_y = sequence_control_set_ptr->subsampling_y;
    const int num_planes = sequence_control_set_ptr->seq_header.color_config.mono_chrome ? 1 : MAX_MB_PLANE;

    EbPictureBufferDesc recon_pic_temp;
    EbPictureBufferDesc *ps_recon_pic_temp;
    ps_recon_pic_temp = &recon_pic_temp;

    EbErrorType return_error = copy_recon_enc(sequence_control_set_ptr, recon_ptr, ps_recon_pic_temp, num_planes, 0);

    if (return_error != EB_ErrorNone) {
        ps_recon_pic_temp = NULL;
        assert(0);
    }

    EbPictureBufferDesc *src = ps_recon_pic_temp;
    EbPictureBufferDesc *dst = recon_ptr;

    for (int plane = 0; plane < num_planes; ++plane) {
        uint8_t *src_buf, *dst_buf;
        int32_t src_stride, dst_stride;

        int sub_x = plane ? ss_x : 0;
        int sub_y = plane ? ss_y : 0;
        derive_blk_pointers_enc(src, plane, 0, 0, (void *) &src_buf, &src_stride,
                                sub_x, sub_y);
        derive_blk_pointers_enc(dst, plane, 0, 0, (void *) &dst_buf, &dst_stride,
                                sub_x, sub_y);

        av1_upscale_normative_rows(cm, (const uint8_t *) src_buf, src_stride, dst_buf,
                                   dst_stride, src->height >> sub_x,
                                   sub_x, src->bit_depth);
    }

    // free the memory
    EB_FREE_ALIGNED_ARRAY(ps_recon_pic_temp->buffer_y);
    EB_FREE_ALIGNED_ARRAY(ps_recon_pic_temp->buffer_cb);
    EB_FREE_ALIGNED_ARRAY(ps_recon_pic_temp->buffer_cr);

}

/******************************************************
 * Rest Kernel
 ******************************************************/
void *rest_kernel(void *input_ptr) {
    // Context & SCS & PCS
    EbThreadContext *   thread_context_ptr = (EbThreadContext *)input_ptr;
    RestContext *       context_ptr        = (RestContext *)thread_context_ptr->priv;
    PictureControlSet * pcs_ptr;
    SequenceControlSet *scs_ptr;
    FrameHeader *       frm_hdr;

    //// Input
    EbObjectWrapper *cdef_results_wrapper_ptr;
    CdefResults *    cdef_results_ptr;

    //// Output
    EbObjectWrapper *    rest_results_wrapper_ptr;
    RestResults *        rest_results_ptr;
    EbObjectWrapper *    picture_demux_results_wrapper_ptr;
    PictureDemuxResults *picture_demux_results_rtr;
    // SB Loop variables

    for (;;) {
        // Get Cdef Results
        eb_get_full_object(context_ptr->rest_input_fifo_ptr, &cdef_results_wrapper_ptr);

        cdef_results_ptr = (CdefResults *)cdef_results_wrapper_ptr->object_ptr;
        pcs_ptr          = (PictureControlSet *)cdef_results_ptr->pcs_wrapper_ptr->object_ptr;
        scs_ptr          = (SequenceControlSet *)pcs_ptr->scs_wrapper_ptr->object_ptr;
        frm_hdr          = &pcs_ptr->parent_pcs_ptr->frm_hdr;
        uint8_t    sb_size_log2 = (uint8_t)Log2f(scs_ptr->sb_size_pix);
        EbBool     is_16bit     = (EbBool)(scs_ptr->static_config.encoder_bit_depth > EB_8BIT);
        Av1Common *cm           = pcs_ptr->parent_pcs_ptr->av1_cm;

        if (scs_ptr->seq_header.enable_restoration && frm_hdr->allow_intrabc == 0) {
            get_own_recon(scs_ptr, pcs_ptr, context_ptr, is_16bit);

            Yv12BufferConfig cpi_source;
            link_eb_to_aom_buffer_desc(is_16bit ? pcs_ptr->input_frame16bit
                                                : pcs_ptr->parent_pcs_ptr->enhanced_picture_ptr,
                                       &cpi_source);

            Yv12BufferConfig trial_frame_rst;
            link_eb_to_aom_buffer_desc(context_ptr->trial_frame_rst, &trial_frame_rst);

            Yv12BufferConfig org_fts;
            link_eb_to_aom_buffer_desc(context_ptr->org_rec_frame, &org_fts);

            restoration_seg_search(context_ptr->rst_tmpbuf,
                                   &org_fts,
                                   &cpi_source,
                                   &trial_frame_rst,
                                   pcs_ptr,
                                   cdef_results_ptr->segment_index);
        }

        //all seg based search is done. update total processed segments. if all done, finish the search and perfrom application.
        eb_block_on_mutex(pcs_ptr->rest_search_mutex);

        pcs_ptr->tot_seg_searched_rest++;
        if (pcs_ptr->tot_seg_searched_rest == pcs_ptr->rest_segments_total_count) {
            if (scs_ptr->seq_header.enable_restoration && frm_hdr->allow_intrabc == 0) {
                rest_finish_search(pcs_ptr->parent_pcs_ptr->av1x, pcs_ptr->parent_pcs_ptr->av1_cm);

                if (cm->rst_info[0].frame_restoration_type != RESTORE_NONE ||
                    cm->rst_info[1].frame_restoration_type != RESTORE_NONE ||
                    cm->rst_info[2].frame_restoration_type != RESTORE_NONE) {
                    
                    // ------- start: Normative upscaling frame - super-resolution tool
                    // Right now, since the downscaling and encoding of the donwscaled picture is not yet implemented, a small test was added,
                    // which will downscale the reconstructed picture by a factor of 2 (denominator 16) and upscale normatively back to the
                    // original recon dimensions - no change is done to the reconstructed picture for this test

                    // TODO: remove the downscaling and YUV output
                    if(!av1_superres_unscaled(&cm->frm_size)) {
                        cm->frm_size.superres_upscaled_width = cm->frm_size.frame_width;
                        cm->frm_size.frame_width = cm->frm_size.frame_width >> 1; // this will be the encoded width (not the source width as it is now)
                        cm->frm_size.superres_denominator = 16; // denominator
                        cm->mi_cols = cm->mi_cols >> 1;
                        cm->mi_stride = cm->mi_stride >> 1;

                        EbPictureBufferDesc recon_ptr_half;
                        EbPictureBufferDesc *curr_recon_ptr;
                        EbPictureBufferDesc *recon_save_ptr;

                        get_recon_pic(picture_control_set_ptr,
                                      &recon_save_ptr);

                        // debug only
                        save_YUV_to_file("recon_before_1280x720.yuv", recon_save_ptr->buffer_y,
                                         recon_save_ptr->buffer_cb, recon_save_ptr->buffer_cr,
                                         recon_save_ptr->width, recon_save_ptr->height,
                                         recon_save_ptr->stride_y, recon_save_ptr->stride_cb, recon_save_ptr->stride_cr,
                                         recon_save_ptr->origin_y, recon_save_ptr->origin_x,
                                         1, 1);

                        downscale_recon_for_test(&recon_ptr_half,
                                                 picture_control_set_ptr,
                                                 sequence_control_set_ptr);

                        eb_av1_superres_upscale_frame(cm,
                                                      picture_control_set_ptr,
                                                      sequence_control_set_ptr);

                        get_recon_pic(picture_control_set_ptr,
                                      &curr_recon_ptr);

                        // debug only
                        save_YUV_to_file("recon_donscaled_av1upscaled_1280x720.yuv", curr_recon_ptr->buffer_y,
                                         curr_recon_ptr->buffer_cb, curr_recon_ptr->buffer_cr,
                                         curr_recon_ptr->width, curr_recon_ptr->height,
                                         curr_recon_ptr->stride_y, curr_recon_ptr->stride_cb, curr_recon_ptr->stride_cr,
                                         curr_recon_ptr->origin_y, curr_recon_ptr->origin_x,
                                         1, 1);

                        replace_recon_pic(recon_save_ptr,
                                          picture_control_set_ptr);

                        get_recon_pic(picture_control_set_ptr,
                                      &curr_recon_ptr);

                        // debug only
                        save_YUV_to_file("recon_after_1280x720.yuv", curr_recon_ptr->buffer_y,
                                         curr_recon_ptr->buffer_cb, curr_recon_ptr->buffer_cr,
                                         curr_recon_ptr->width, curr_recon_ptr->height,
                                         curr_recon_ptr->stride_y, curr_recon_ptr->stride_cb, curr_recon_ptr->stride_cr,
                                         curr_recon_ptr->origin_y, curr_recon_ptr->origin_x,
                                         1, 1);

                        // free the memory
                        EB_FREE_ALIGNED_ARRAY(recon_ptr_half.buffer_y);
                        EB_FREE_ALIGNED_ARRAY(recon_ptr_half.buffer_cb);
                        EB_FREE_ALIGNED_ARRAY(recon_ptr_half.buffer_cr);

                        // restore back original info in cm for this test
                        cm->frm_size.frame_width = cm->frm_size.frame_width
                                << 1; // this will be the encoded width (not the source width as it is now)
                        cm->frm_size.superres_denominator = 8; // denominator
                        cm->mi_cols = cm->mi_cols << 1;
                        cm->mi_stride = cm->mi_stride << 1;
                    }

                    // ------- end: Normative upscaling frame - super-resolution tool

                    eb_av1_loop_restoration_filter_frame(cm->frame_to_show, cm, 0);
                }
            } else {
                cm->rst_info[0].frame_restoration_type = RESTORE_NONE;
                cm->rst_info[1].frame_restoration_type = RESTORE_NONE;
                cm->rst_info[2].frame_restoration_type = RESTORE_NONE;
            }

            uint8_t best_ep_cnt = 0;
            uint8_t best_ep     = 0;
            for (uint8_t i = 0; i < SGRPROJ_PARAMS; i++) {
                if (cm->sg_frame_ep_cnt[i] > best_ep_cnt) {
                    best_ep     = i;
                    best_ep_cnt = cm->sg_frame_ep_cnt[i];
                }
            }
            cm->sg_frame_ep = best_ep;

            if (pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr != NULL) {
                // copy stat to ref object (intra_coded_area, Luminance, Scene change detection flags)
                copy_statistics_to_ref_obj_ect(pcs_ptr, scs_ptr);
            }

            // PSNR Calculation
            if (scs_ptr->static_config.stat_report) psnr_calculations(pcs_ptr, scs_ptr);

            // Pad the reference picture and set ref POC
            if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag == EB_TRUE)
                pad_ref_and_set_flags(pcs_ptr, scs_ptr);
            if (scs_ptr->static_config.recon_enabled) { recon_output(pcs_ptr, scs_ptr); }

            if (pcs_ptr->parent_pcs_ptr->is_used_as_reference_flag) {
                // Get Empty PicMgr Results
                eb_get_empty_object(context_ptr->picture_demux_fifo_ptr,
                                    &picture_demux_results_wrapper_ptr);

                picture_demux_results_rtr =
                    (PictureDemuxResults *)picture_demux_results_wrapper_ptr->object_ptr;
                picture_demux_results_rtr->reference_picture_wrapper_ptr =
                    pcs_ptr->parent_pcs_ptr->reference_picture_wrapper_ptr;
                picture_demux_results_rtr->scs_wrapper_ptr = pcs_ptr->scs_wrapper_ptr;
                picture_demux_results_rtr->picture_number  = pcs_ptr->picture_number;
                picture_demux_results_rtr->picture_type    = EB_PIC_REFERENCE;

                // Post Reference Picture
                eb_post_full_object(picture_demux_results_wrapper_ptr);
            }

            // Get Empty rest Results to EC
            eb_get_empty_object(context_ptr->rest_output_fifo_ptr, &rest_results_wrapper_ptr);
            rest_results_ptr = (struct RestResults *)rest_results_wrapper_ptr->object_ptr;
            rest_results_ptr->pcs_wrapper_ptr              = cdef_results_ptr->pcs_wrapper_ptr;
            rest_results_ptr->completed_sb_row_index_start = 0;
            rest_results_ptr->completed_sb_row_count =
                ((scs_ptr->seq_header.max_frame_height + scs_ptr->sb_size_pix - 1) >> sb_size_log2);
            // Post Rest Results
            eb_post_full_object(rest_results_wrapper_ptr);
        }
        eb_release_mutex(pcs_ptr->rest_search_mutex);

        // Release input Results
        eb_release_object(cdef_results_wrapper_ptr);
    }

    return EB_NULL;
}
