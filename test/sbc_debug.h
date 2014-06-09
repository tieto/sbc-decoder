/*
 *
 *  Bluetooth low-complexity, subband codec (SBC) decoder
 *
 *  Copyright (C) 2008-2010  Nokia Corporation
 *  Copyright (C) 2004-2010  Marcel Holtmann <marcel@holtmann.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

/******************************************************************************
 *
 *  This file contains constants and structures used by Decoder.
 *
 ******************************************************************************/
#ifndef SBC_DEBUG_H
#define SBC_DEBUF_H

#define fabs(x) ((x) < 0 ? -(x) : (x))
#define ASR(val, bits) ((-2 >> 1 == -1) ? \
         ((SINT32)(val)) >> (bits) : ((SINT32) (val)) / (1 << (bits)))
#define SCALE_SPROTO4_TBL    12
#define SCALE_SPROTO8_TBL    14
#define SCALE_NPROTO4_TBL    11
#define SCALE_NPROTO8_TBL    11
#define SCALE4_STAGED1_BITS    15
#define SCALE4_STAGED2_BITS    16
#define SCALE8_STAGED1_BITS    15
#define SCALE8_STAGED2_BITS    16
typedef SINT32 sbc_fixed_t;
#define SCALE4_STAGED1(src) ASR(src, SCALE4_STAGED1_BITS)
#define SCALE4_STAGED2(src) ASR(src, SCALE4_STAGED2_BITS)
#define SCALE8_STAGED1(src) ASR(src, SCALE8_STAGED1_BITS)
#define SCALE8_STAGED2(src) ASR(src, SCALE8_STAGED2_BITS)
#define SBC_FIXED_0(val) { val = 0; }
#define MUL(a, b)        ((a) * (b))
#if defined(__arm__) && (!defined(__thumb__) || defined(__thumb2__))
#define MULA(a, b, res) ({                \
        int tmp = res;            \
        __asm__(                \
            "mla %0, %2, %3, %0"        \
            : "=&r" (tmp)            \
            : "0" (tmp), "r" (a), "r" (b));    \
        tmp; })
#else
#define MULA(a, b, res)  ((a) * (b) + (res))
#endif
static const int sbc_offset4[4][4] = {
    { -1, 0, 0, 0 },
    { -2, 0, 0, 1 },
    { -2, 0, 0, 1 },
    { -2, 0, 0, 1 }
};
static const int sbc_offset8[4][8] = {
    { -2, 0, 0, 0, 0, 0, 0, 1 },
    { -3, 0, 0, 0, 0, 0, 1, 2 },
    { -4, 0, 0, 0, 0, 0, 1, 2 },
    { -4, 0, 0, 0, 0, 0, 1, 2 }
};
#define SBCDEC_FIXED_EXTRA_BITS 2
#define SS4(val) ASR(val, SCALE_SPROTO4_TBL)
#define SS8(val) ASR(val, SCALE_SPROTO8_TBL)
#define SN4(val) ASR(val, SCALE_NPROTO4_TBL + 1 + SBCDEC_FIXED_EXTRA_BITS)
#define SN8(val) ASR(val, SCALE_NPROTO8_TBL + 1 + SBCDEC_FIXED_EXTRA_BITS)
static const SINT32 sbc_proto_4_40m0[] = {
    SS4(0x00000000), SS4(0xffa6982f), SS4(0xfba93848), SS4(0x0456c7b8),
    SS4(0x005967d1), SS4(0xfffb9ac7), SS4(0xff589157), SS4(0xf9c2a8d8),
    SS4(0x027c1434), SS4(0x0019118b), SS4(0xfff3c74c), SS4(0xff137330),
    SS4(0xf81b8d70), SS4(0x00ec1b8b), SS4(0xfff0b71a), SS4(0xffe99b00),
    SS4(0xfef84470), SS4(0xf6fb4370), SS4(0xffcdc351), SS4(0xffe01dc7)
};
static const SINT32 sbc_proto_4_40m1[] = {
    SS4(0xffe090ce), SS4(0xff2c0475), SS4(0xf694f800), SS4(0xff2c0475),
    SS4(0xffe090ce), SS4(0xffe01dc7), SS4(0xffcdc351), SS4(0xf6fb4370),
    SS4(0xfef84470), SS4(0xffe99b00), SS4(0xfff0b71a), SS4(0x00ec1b8b),
    SS4(0xf81b8d70), SS4(0xff137330), SS4(0xfff3c74c), SS4(0x0019118b),
    SS4(0x027c1434), SS4(0xf9c2a8d8), SS4(0xff589157), SS4(0xfffb9ac7)
};
static const SINT32 sbc_proto_8_80m0[] = {
    SS8(0x00000000), SS8(0xfe8d1970), SS8(0xee979f00), SS8(0x11686100),
    SS8(0x0172e690), SS8(0xfff5bd1a), SS8(0xfdf1c8d4), SS8(0xeac182c0),
    SS8(0x0d9daee0), SS8(0x00e530da), SS8(0xffe9811d), SS8(0xfd52986c),
    SS8(0xe7054ca0), SS8(0x0a00d410), SS8(0x006c1de4), SS8(0xffdba705),
    SS8(0xfcbc98e8), SS8(0xe3889d20), SS8(0x06af2308), SS8(0x000bb7db),
    SS8(0xffca00ed), SS8(0xfc3fbb68), SS8(0xe071bc00), SS8(0x03bf7948),
    SS8(0xffc4e05c), SS8(0xffb54b3b), SS8(0xfbedadc0), SS8(0xdde26200),
    SS8(0x0142291c), SS8(0xff960e94), SS8(0xff9f3e17), SS8(0xfbd8f358),
    SS8(0xdbf79400), SS8(0xff405e01), SS8(0xff7d4914), SS8(0xff8b1a31),
    SS8(0xfc1417b8), SS8(0xdac7bb40), SS8(0xfdbb828c), SS8(0xff762170)
};
static const SINT32 sbc_proto_8_80m1[] = {
    SS8(0xff7c272c), SS8(0xfcb02620), SS8(0xda612700), SS8(0xfcb02620),
    SS8(0xff7c272c), SS8(0xff762170), SS8(0xfdbb828c), SS8(0xdac7bb40),
    SS8(0xfc1417b8), SS8(0xff8b1a31), SS8(0xff7d4914), SS8(0xff405e01),
    SS8(0xdbf79400), SS8(0xfbd8f358), SS8(0xff9f3e17), SS8(0xff960e94),
    SS8(0x0142291c), SS8(0xdde26200), SS8(0xfbedadc0), SS8(0xffb54b3b),
    SS8(0xffc4e05c), SS8(0x03bf7948), SS8(0xe071bc00), SS8(0xfc3fbb68),
    SS8(0xffca00ed), SS8(0x000bb7db), SS8(0x06af2308), SS8(0xe3889d20),
    SS8(0xfcbc98e8), SS8(0xffdba705), SS8(0x006c1de4), SS8(0x0a00d410),
    SS8(0xe7054ca0), SS8(0xfd52986c), SS8(0xffe9811d), SS8(0x00e530da),
    SS8(0x0d9daee0), SS8(0xeac182c0), SS8(0xfdf1c8d4), SS8(0xfff5bd1a)
};
static const SINT32 synmatrix4[8][4] = {
    { SN4(0x05a82798), SN4(0xfa57d868), SN4(0xfa57d868), SN4(0x05a82798) },
    { SN4(0x030fbc54), SN4(0xf89be510), SN4(0x07641af0), SN4(0xfcf043ac) },
    { SN4(0x00000000), SN4(0x00000000), SN4(0x00000000), SN4(0x00000000) },
    { SN4(0xfcf043ac), SN4(0x07641af0), SN4(0xf89be510), SN4(0x030fbc54) },
    { SN4(0xfa57d868), SN4(0x05a82798), SN4(0x05a82798), SN4(0xfa57d868) },
    { SN4(0xf89be510), SN4(0xfcf043ac), SN4(0x030fbc54), SN4(0x07641af0) },
    { SN4(0xf8000000), SN4(0xf8000000), SN4(0xf8000000), SN4(0xf8000000) },
    { SN4(0xf89be510), SN4(0xfcf043ac), SN4(0x030fbc54), SN4(0x07641af0) }
};
static const SINT32 synmatrix8[16][8] = {
    { SN8(0x05a82798), SN8(0xfa57d868), SN8(0xfa57d868), SN8(0x05a82798),
      SN8(0x05a82798), SN8(0xfa57d868), SN8(0xfa57d868), SN8(0x05a82798) },
    { SN8(0x0471ced0), SN8(0xf8275a10), SN8(0x018f8b84), SN8(0x06a6d988),
      SN8(0xf9592678), SN8(0xfe70747c), SN8(0x07d8a5f0), SN8(0xfb8e3130) },
    { SN8(0x030fbc54), SN8(0xf89be510), SN8(0x07641af0), SN8(0xfcf043ac),
      SN8(0xfcf043ac), SN8(0x07641af0), SN8(0xf89be510), SN8(0x030fbc54) },
    { SN8(0x018f8b84), SN8(0xfb8e3130), SN8(0x06a6d988), SN8(0xf8275a10),
      SN8(0x07d8a5f0), SN8(0xf9592678), SN8(0x0471ced0), SN8(0xfe70747c) },
    { SN8(0x00000000), SN8(0x00000000), SN8(0x00000000), SN8(0x00000000),
      SN8(0x00000000), SN8(0x00000000), SN8(0x00000000), SN8(0x00000000) },
    { SN8(0xfe70747c), SN8(0x0471ced0), SN8(0xf9592678), SN8(0x07d8a5f0),
      SN8(0xf8275a10), SN8(0x06a6d988), SN8(0xfb8e3130), SN8(0x018f8b84) },
    { SN8(0xfcf043ac), SN8(0x07641af0), SN8(0xf89be510), SN8(0x030fbc54),
      SN8(0x030fbc54), SN8(0xf89be510), SN8(0x07641af0), SN8(0xfcf043ac) },
    { SN8(0xfb8e3130), SN8(0x07d8a5f0), SN8(0xfe70747c), SN8(0xf9592678),
      SN8(0x06a6d988), SN8(0x018f8b84), SN8(0xf8275a10), SN8(0x0471ced0) },
    { SN8(0xfa57d868), SN8(0x05a82798), SN8(0x05a82798), SN8(0xfa57d868),
      SN8(0xfa57d868), SN8(0x05a82798), SN8(0x05a82798), SN8(0xfa57d868) },
    { SN8(0xf9592678), SN8(0x018f8b84), SN8(0x07d8a5f0), SN8(0x0471ced0),
      SN8(0xfb8e3130), SN8(0xf8275a10), SN8(0xfe70747c), SN8(0x06a6d988) },
    { SN8(0xf89be510), SN8(0xfcf043ac), SN8(0x030fbc54), SN8(0x07641af0),
      SN8(0x07641af0), SN8(0x030fbc54), SN8(0xfcf043ac), SN8(0xf89be510) },
    { SN8(0xf8275a10), SN8(0xf9592678), SN8(0xfb8e3130), SN8(0xfe70747c),
      SN8(0x018f8b84), SN8(0x0471ced0), SN8(0x06a6d988), SN8(0x07d8a5f0) },
    { SN8(0xf8000000), SN8(0xf8000000), SN8(0xf8000000), SN8(0xf8000000),
      SN8(0xf8000000), SN8(0xf8000000), SN8(0xf8000000), SN8(0xf8000000) },
    { SN8(0xf8275a10), SN8(0xf9592678), SN8(0xfb8e3130), SN8(0xfe70747c),
      SN8(0x018f8b84), SN8(0x0471ced0), SN8(0x06a6d988), SN8(0x07d8a5f0) },
    { SN8(0xf89be510), SN8(0xfcf043ac), SN8(0x030fbc54), SN8(0x07641af0),
      SN8(0x07641af0), SN8(0x030fbc54), SN8(0xfcf043ac), SN8(0xf89be510) },
    { SN8(0xf9592678), SN8(0x018f8b84), SN8(0x07d8a5f0), SN8(0x0471ced0),
      SN8(0xfb8e3130), SN8(0xf8275a10), SN8(0xfe70747c), SN8(0x06a6d988) }
};
#define    EPERM         1    /* Operation not permitted */
#define    ENOENT         2    /* No such file or directory */
#define    ESRCH         3    /* No such process */
#define    EINTR         4    /* Interrupted system call */
#define    EIO         5    /* I/O error */
#define    ENXIO         6    /* No such device or address */
#define    E2BIG         7    /* Argument list too long */
#define    ENOEXEC         8    /* Exec format error */
#define    EBADF         9    /* Bad file number */
#define    ECHILD        10    /* No child processes */
#define    EAGAIN        11    /* Try again */
#define    ENOMEM        12    /* Out of memory */
#define    EACCES        13    /* Permission denied */
#define    EFAULT        14    /* Bad address */
#define    ENOTBLK        15    /* Block device required */
#define    EBUSY        16    /* Device or resource busy */
#define    EEXIST        17    /* File exists */
#define    EXDEV        18    /* Cross-device link */
#define    ENODEV        19    /* No such device */
#define    ENOTDIR        20    /* Not a directory */
#define    EISDIR        21    /* Is a directory */
#define    EINVAL        22    /* Invalid argument */
#define    ENFILE        23    /* File table overflow */
#define    EMFILE        24    /* Too many open files */
#define    ENOTTY        25    /* Not a typewriter */
#define    ETXTBSY        26    /* Text file busy */
#define    EFBIG        27    /* File too large */
#define    ENOSPC        28    /* No space left on device */
#define    ESPIPE        29    /* Illegal seek */
#define    EROFS        30    /* Read-only file system */
#define    EMLINK        31    /* Too many links */
#define    EPIPE        32    /* Broken pipe */
#define    EDOM        33    /* Math argument out of domain of func */
#define    ERANGE        34    /* Math result not representable */
#define SBC_BLK_4        0x00
#define SBC_BLK_8        0x01
#define SBC_BLK_12        0x02
#define SBC_BLK_16        0x03
#define SBC_SB_4        0x00
#define SBC_SB_8        0x01
#define SBC_FREQ_16000        0x00
#define SBC_FREQ_32000        0x01
#define SBC_FREQ_44100        0x02
#define SBC_FREQ_48000        0x03
#define SBC_MODE_MONO        0x00
#define SBC_MODE_DUAL_CHANNEL    0x01
#define SBC_MODE_STEREO        0x02
#define SBC_MODE_JOINT_STEREO    0x03
#define SBC_ALIGN_BITS 4
#define SBC_ALIGN_MASK ((1 << (SBC_ALIGN_BITS)) - 1)
#define SBC_SYNCWORD    0x9C
#define SBC_LE            0x00
#define SBC_BE            0x01
#ifdef __GNUC__
#define SBC_ALIGNED __attribute__((aligned(1 << (SBC_ALIGN_BITS))))
#else
#define SBC_ALIGNED
#endif
struct sbc_decoder_state {
    int subbands;
    SINT32 V[2][170];
    int offset[2][16];
};
struct sbc_struct {
    unsigned long flags;
    UINT8 frequency;
    UINT8 blocks;
    UINT8 subbands;
    UINT8 mode;
    UINT8 allocation;
    UINT8 bitpool;
    UINT8 endian;
    void *priv;
    void *priv_alloc_base;
};
typedef struct sbc_struct sbc_t;
struct sbc_frame {
    UINT8 frequency;
    UINT8 block_mode;
    UINT8 blocks;
    enum {
        MONO        = SBC_MONO,
        DUAL_CHANNEL    = SBC_DUAL,
        STEREO        = SBC_STEREO,
        JOINT_STEREO    = SBC_JOINT_STEREO
    } mode;
    UINT8 channels;
    enum {
        LOUDNESS    = SBC_LOUDNESS,
        SNR        = SBC_SNR
    } allocation;
    UINT8 subband_mode;
    UINT8 subbands;
    UINT8 bitpool;
    UINT16 codesize;
    UINT8 length;
    UINT8 joint;
    UINT32   scale_factor[2][8];
    SINT32  sb_sample_f[16][2][8];
    SINT32  sb_sample[16][2][8];
    SINT16  pcm_sample[2][16*8];
};
struct sbc_priv {
    int init;
    struct  sbc_frame frame;
    struct  sbc_decoder_state dec_state;
};
#endif
