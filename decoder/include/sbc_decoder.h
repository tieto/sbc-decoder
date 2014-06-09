/******************************************************************************
 *
 *  Copyright (C) 2014 Tieto Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  This file contains constants and structures used by Decoder.
 *
 ******************************************************************************/

#ifndef SBC_DECODER_H
#define SBC_DECODER_H

#define DECODER_VERSION "0025"

#ifdef BUILDCFG
    #include "bt_target.h"
#endif

/*DEFINES*/
#define MINIMUM_DEC_VX_BUFFER_SIZE (8*10*2)
#ifndef DEC_VX_BUFFER_SIZE
#define DEC_VX_BUFFER_SIZE (MINIMUM_DEC_VX_BUFFER_SIZE + 64)
/*#define DEC_VX_BUFFER_SIZE MINIMUM_DEC_VX_BUFFER_SIZE + 1024*/
#endif

#include "sbc_types.h"
#include "sbc_encoder.h"

typedef struct SBC_DEC_PARAMS_TAG
{
    /*-----------output parameter---------------*/
    // the final decoded PCM for one frame
    SINT16 as16SbBuffer[SBC_MAX_NUM_OF_CHANNELS * SBC_MAX_NUM_OF_SUBBANDS *  SBC_MAX_NUM_OF_BLOCKS];
    // the length of PCM for one frame so that the caller can copy
    SINT16 s16PcmLength;
    // the lenght of a decoded SBC frame
    SINT16 s16SbcFrameLength;

    /*-----------input parameter---------------*/
    // the data need to be decoded
    UINT8  *pu8Packet;
    UINT16 u16PacketLength;

}SBC_DEC_PARAMS;

#ifdef __cplusplus
extern "C"
{
#endif
SBC_API extern void SBC_Decoder(SBC_DEC_PARAMS *strDecParams);

SBC_API extern void SBC_Decoder_Init(SBC_DEC_PARAMS *strDecParams);

#ifdef __cplusplus
}
#endif

#endif
