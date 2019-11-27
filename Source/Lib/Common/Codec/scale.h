#ifndef SVT_AV1_SCALE_H
#define SVT_AV1_SCALE_H

#include "EbDefinitions.h"
#include "EbPictureBufferDesc.h"
#include "EbInterPrediction.h"

void av1_resize_and_extend_frame(const EbPictureBufferDesc *src,
                                 EbPictureBufferDesc *dst,
                                 int bd,
                                 const int num_planes);

#endif //SVT_AV1_SCALE_H


