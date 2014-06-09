/******************************************************************************
 *
 *  Bluetooth low-complexity, subband codec (SBC) library
 *
 *  Copyright (C) 2014  Tieto Corporation
 *  Copyright (C) 2008-2010  Nokia Corporation
 *  Copyright (C) 2004-2010  Marcel Holtmann <marcel@holtmann.org>
 *  Copyright (C) 2004-2005  Henryk Ploetz <henryk@ploetzli.ch>
 *  Copyright (C) 2005-2008  Brad Midgley <bmidgley@xmission.com>
 *
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

/******************************************************************************
 *
 *  contains code for decoder flow and initalization of decoder
 *
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include "sbc_decoder.h"
#include "../../../bluedroid/include/bt_trace.h"

#define fabs(x) ((x) < 0 ? -(x) : (x))
#define ASR(val, bits) ((-2 >> 1 == -1) ? \
         ((int)(val)) >> (bits) : ((int) (val)) / (1 << (bits)))
#define SCALE_SPROTO4_TBL    12
#define SCALE_SPROTO8_TBL    14
#define SCALE_NPROTO4_TBL    11
#define SCALE_NPROTO8_TBL    11
#define SCALE4_STAGED1_BITS    15
#define SCALE4_STAGED2_BITS    16
#define SCALE8_STAGED1_BITS    15
#define SCALE8_STAGED2_BITS    16
typedef int sbc_fixed_t;
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

#define    EPERM           1    /* Operation not permitted */
#define    ENOENT          2    /* No such file or directory */
#define    ESRCH           3    /* No such process */
#define    EINTR           4    /* Interrupted system call */
#define    EIO             5    /* I/O error */
#define    ENXIO           6    /* No such device or address */
#define    E2BIG           7    /* Argument list too long */
#define    ENOEXEC         8    /* Exec format error */
#define    EBADF           9    /* Bad file number */
#define    ECHILD          10    /* No child processes */
#define    EAGAIN          11    /* Try again */
#define    ENOMEM          12    /* Out of memory */
#define    EACCES          13    /* Permission denied */
#define    EFAULT          14    /* Bad address */
#define    ENOTBLK         15    /* Block device required */
#define    EBUSY           16    /* Device or resource busy */
#define    EEXIST          17    /* File exists */
#define    EXDEV           18    /* Cross-device link */
#define    ENODEV          19    /* No such device */
#define    ENOTDIR         20    /* Not a directory */
#define    EISDIR          21    /* Is a directory */
#define    EINVAL          22    /* Invalid argument */
#define    ENFILE          23    /* File table overflow */
#define    EMFILE          24    /* Too many open files */
#define    ENOTTY          25    /* Not a typewriter */
#define    ETXTBSY         26    /* Text file busy */
#define    EFBIG           27    /* File too large */
#define    ENOSPC          28    /* No space left on device */
#define    ESPIPE          29    /* Illegal seek */
#define    EROFS           30    /* Read-only file system */
#define    EMLINK          31    /* Too many links */
#define    EPIPE           32    /* Broken pipe */
#define    EDOM            33    /* Math argument out of domain of func */
#define    ERANGE          34    /* Math result not representable */
#define SBC_BLK_4        0x00
#define SBC_BLK_8        0x01
#define SBC_BLK_12      0x02
#define SBC_BLK_16      0x03
#define SBC_SB_4         0x00
#define SBC_SB_8         0x01
#define SBC_FREQ_16000        0x00
#define SBC_FREQ_32000        0x01
#define SBC_FREQ_44100        0x02
#define SBC_FREQ_48000        0x03
#define SBC_MODE_MONO         0x00
#define SBC_MODE_DUAL_CHANNEL    0x01
#define SBC_MODE_STEREO          0x02
#define SBC_MODE_JOINT_STEREO    0x03
#define SBC_ALIGN_BITS 4
#define SBC_ALIGN_MASK ((1 << (SBC_ALIGN_BITS)) - 1)
#define SBC_SYNCWORD    0x9C
#define SBC_LE                 0x00
#define SBC_BE                 0x01
#ifdef __GNUC__
#define SBC_ALIGNED __attribute__((aligned(1 << (SBC_ALIGN_BITS))))
#else
#define SBC_ALIGNED
#endif


struct sbc_decoder_state {
    int subbands;
    int V[2][170];
    int offset[2][16];
};
struct sbc_struct {
    unsigned long flags;
    unsigned char frequency;
    unsigned char blocks;
    unsigned char subbands;
    unsigned char mode;
    unsigned char allocation;
    unsigned char bitpool;
    unsigned char endian;
    void *priv;
    void *priv_alloc_base;
};
typedef struct sbc_struct sbc_t;
struct sbc_frame {
    unsigned char frequency;
    unsigned char block_mode;
    unsigned char blocks;
    enum {
        MONO        = SBC_MONO,
        DUAL_CHANNEL    = SBC_DUAL,
        STEREO        = SBC_STEREO,
        JOINT_STEREO    = SBC_JOINT_STEREO
    } mode;
    unsigned char channels;
    enum {
        LOUDNESS    = SBC_LOUDNESS,
        SNR        = SBC_SNR
    } allocation;
    unsigned char subband_mode;
    unsigned char subbands;
    unsigned char bitpool;
    unsigned short codesize;
    unsigned char length;
    unsigned char joint;
    unsigned int scale_factor[2][8];
    int sb_sample_f[16][2][8];
    int sb_sample[16][2][8];
    short pcm_sample[2][16*8];
};
struct sbc_priv {
    int init;
    struct  sbc_frame frame;
    struct  sbc_decoder_state dec_state;
};
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
static const int sbc_proto_4_40m0[] = {
        SS4(0x00000000), SS4(0xffa6982f), SS4(0xfba93848), SS4(0x0456c7b8),
        SS4(0x005967d1), SS4(0xfffb9ac7), SS4(0xff589157), SS4(0xf9c2a8d8),
        SS4(0x027c1434), SS4(0x0019118b), SS4(0xfff3c74c), SS4(0xff137330),
        SS4(0xf81b8d70), SS4(0x00ec1b8b), SS4(0xfff0b71a), SS4(0xffe99b00),
        SS4(0xfef84470), SS4(0xf6fb4370), SS4(0xffcdc351), SS4(0xffe01dc7)
};
static const int sbc_proto_4_40m1[] = {
        SS4(0xffe090ce), SS4(0xff2c0475), SS4(0xf694f800), SS4(0xff2c0475),
        SS4(0xffe090ce), SS4(0xffe01dc7), SS4(0xffcdc351), SS4(0xf6fb4370),
        SS4(0xfef84470), SS4(0xffe99b00), SS4(0xfff0b71a), SS4(0x00ec1b8b),
        SS4(0xf81b8d70), SS4(0xff137330), SS4(0xfff3c74c), SS4(0x0019118b),
        SS4(0x027c1434), SS4(0xf9c2a8d8), SS4(0xff589157), SS4(0xfffb9ac7)
};
static const int sbc_proto_8_80m0[] = {
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
static const int sbc_proto_8_80m1[] = {
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
static const int synmatrix4[8][4] = {
    { SN4(0x05a82798), SN4(0xfa57d868), SN4(0xfa57d868), SN4(0x05a82798) },
    { SN4(0x030fbc54), SN4(0xf89be510), SN4(0x07641af0), SN4(0xfcf043ac) },
    { SN4(0x00000000), SN4(0x00000000), SN4(0x00000000), SN4(0x00000000) },
    { SN4(0xfcf043ac), SN4(0x07641af0), SN4(0xf89be510), SN4(0x030fbc54) },
    { SN4(0xfa57d868), SN4(0x05a82798), SN4(0x05a82798), SN4(0xfa57d868) },
    { SN4(0xf89be510), SN4(0xfcf043ac), SN4(0x030fbc54), SN4(0x07641af0) },
    { SN4(0xf8000000), SN4(0xf8000000), SN4(0xf8000000), SN4(0xf8000000) },
    { SN4(0xf89be510), SN4(0xfcf043ac), SN4(0x030fbc54), SN4(0x07641af0) }
};
static const int synmatrix8[16][8] = {
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
/*
 * Calculates the CRC-8 of the first len bits in data
 */
static const unsigned char crc_table[256] = {
    0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
    0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
    0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
    0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
    0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
    0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
    0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
    0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
    0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
    0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
    0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
    0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
    0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
    0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
    0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
    0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
    0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
    0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
    0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
    0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
    0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
    0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
    0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
    0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
    0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
    0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
    0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
    0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
    0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
    0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
    0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
    0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

static unsigned char sbc_crc8(const unsigned char *data, unsigned int len);
static void sbc_set_defaults(sbc_t *sbc, unsigned long flags);
static unsigned int sbc_get_codesize(sbc_t *sbc);
static int sbc_init(sbc_t *sbc, unsigned long flags);
static unsigned int sbc_parse(sbc_t *sbc, const void *input, unsigned int input_len);
static unsigned int sbc_decode(sbc_t *sbc, const void *input, unsigned int input_len,
                         void *output, unsigned int output_len, unsigned int *written);
static inline void sbc_dec_copy_pcmbuffer(SBC_DEC_PARAMS * pDestPcmBuffer, struct sbc_frame *pSrcPcm, int len);
static int sbc_unpack_frame(const unsigned char *data, struct sbc_frame *frame,
                            unsigned int len);
static void sbc_decoder_init(struct sbc_decoder_state *state,
                             const struct sbc_frame *frame);
static inline short sbc_clip16(int s);
static inline void sbc_synthesize_four(struct sbc_decoder_state *state,
                                       struct sbc_frame *frame, int ch, int blk);
static inline void sbc_synthesize_eight(struct sbc_decoder_state *state,
struct sbc_frame *frame, int ch, int blk);
static int sbc_synthesize_audio(struct sbc_decoder_state *state,
                                struct sbc_frame *frame);
static inline void sbc_calculate_bits_internal(const struct sbc_frame *frame,
                                               int (*bits)[8], int subbands);
static void sbc_calculate_bits(const struct sbc_frame *frame, int (*bits)[8]);

static unsigned char sbc_crc8(const unsigned char *data, unsigned int len)
{
    unsigned char crc = 0x0f;
    unsigned int i;
    unsigned char octet;

    for (i = 0; i < len / 8; i++)
        crc = crc_table[crc ^ data[i]];

    octet = data[i];
    for (i = 0; i < len % 8; i++) {
        char bit = ((octet ^ crc) & 0x80) >> 7;

        crc = ((crc & 0x7f) << 1) ^ (bit ? 0x1d : 0);

        octet = octet << 1;
    }

    return crc;
}

static void sbc_set_defaults(sbc_t *sbc, unsigned long flags)
{
    sbc->frequency = SBC_sf44100;
    sbc->mode = SBC_STEREO;
    sbc->subbands = SBC_SB_8;
    sbc->blocks = SBC_BLK_16;
    sbc->bitpool = 32;
#if __BIG_ENDIAN != TRUE
    sbc->endian = SBC_LE;
#else
    sbc->endian = SBC_BE;
#endif
}

unsigned int sbc_get_codesize(sbc_t *sbc)
{
    unsigned short subbands, channels, blocks;
    struct sbc_priv *priv;

    priv = sbc->priv;
    if (!priv->init) {
        subbands = sbc->subbands ? 8 : 4;
        blocks = 4 + (sbc->blocks * 4);
        channels = sbc->mode == SBC_MONO ? 1 : 2;
    } else {
        subbands = priv->frame.subbands;
        blocks = priv->frame.blocks;
        channels = priv->frame.channels;
    }

    return subbands * blocks * channels * 2;
}

int sbc_init(sbc_t *sbc, unsigned long flags)
{
    if (!sbc)
        return -EIO;

    memset(sbc, 0, sizeof(sbc_t));

    sbc->priv_alloc_base = malloc(sizeof(struct sbc_priv) + SBC_ALIGN_MASK);
    if (!sbc->priv_alloc_base)
        return -ENOMEM;

    sbc->priv = (void *) (((unsigned int) sbc->priv_alloc_base +
            SBC_ALIGN_MASK) & ~((unsigned int) SBC_ALIGN_MASK));

    memset(sbc->priv, 0, sizeof(struct sbc_priv));

    sbc_set_defaults(sbc, flags);

    return 0;
}

unsigned int sbc_parse(sbc_t *sbc, const void *input, unsigned int input_len)
{
    return sbc_decode(sbc, input, input_len, NULL, 0, NULL);
}

unsigned int sbc_decode(sbc_t *sbc, const void *input, unsigned int input_len,
            void *output, unsigned int output_len, unsigned int *written)
{
    struct sbc_priv *priv;
    char *ptr;
    int i, ch, framelen, samples;

    if (!sbc || !input)
        return -EIO;

    priv = sbc->priv;

    framelen = sbc_unpack_frame(input, &priv->frame, input_len);

    if (!priv->init) {
        sbc_decoder_init(&priv->dec_state, &priv->frame);
        priv->init = 1;

        sbc->frequency = priv->frame.frequency;
        sbc->mode = priv->frame.mode;
        sbc->subbands = priv->frame.subband_mode;
        sbc->blocks = priv->frame.block_mode;
        sbc->allocation = priv->frame.allocation;
        sbc->bitpool = priv->frame.bitpool;

        priv->frame.codesize = sbc_get_codesize(sbc);
        priv->frame.length = framelen;
    } else if (priv->frame.bitpool != sbc->bitpool) {
        priv->frame.length = framelen;
        sbc->bitpool = priv->frame.bitpool;
    }

    if (!output)
        return framelen;

    if (written)
        *written = 0;

    if (framelen <= 0)
        return framelen;

    samples = sbc_synthesize_audio(&priv->dec_state, &priv->frame);

    ptr = output;

    if (output_len < (unsigned int) (samples * priv->frame.channels * 2))
        samples = output_len / (priv->frame.channels * 2);

    for (i = 0; i < samples; i++) {
        for (ch = 0; ch < priv->frame.channels; ch++) {
            short s;
            s = priv->frame.pcm_sample[ch][i];

            if (sbc->endian == SBC_BE) {
                *ptr++ = (s & 0xff00) >> 8;
                *ptr++ = (s & 0x00ff);
            } else {
                *ptr++ = (s & 0x00ff);
                *ptr++ = (s & 0xff00) >> 8;
            }

        }
    }

    if (written)
        *written = samples * priv->frame.channels * 2;

    return framelen;
}

static inline void sbc_dec_copy_pcmbuffer(SBC_DEC_PARAMS * pDestPcmBuffer, struct sbc_frame *pSrcPcm, int len)
{
    int i=0;
    int ch=0;
    short *ptr=pDestPcmBuffer->as16SbBuffer;

    for (i = 0; i < len; i++) {
        for (ch = 0; ch < pSrcPcm->channels; ch++) {
            short s;
            s = pSrcPcm->pcm_sample[ch][i];  //16 bits
            *ptr++= s;
        }
    }

    // len = suband*block
    pDestPcmBuffer->s16PcmLength= len * pSrcPcm->channels * 2;

}

void SBC_Decoder(SBC_DEC_PARAMS *pstrDecParams)
{
    int i=0, ch=0, framelen=0, samples=0;
    struct sbc_frame frame={0};
    struct sbc_decoder_state dec_state={0};

    if ( !pstrDecParams)
        return ;

    framelen = sbc_unpack_frame(pstrDecParams->pu8Packet, &frame, pstrDecParams->u16PacketLength);
    pstrDecParams->s16SbcFrameLength= framelen;

    sbc_decoder_init(&dec_state, &frame);
    //frame.codesize = sbc_get_codesize(&sbc);
    frame.length = framelen;

    if (framelen <= 0)
          return ;
    samples = sbc_synthesize_audio(&dec_state, &frame);
    sbc_dec_copy_pcmbuffer(pstrDecParams, &frame, samples);
}

void SBC_Decoder_Init(SBC_DEC_PARAMS *pstrDecParams)
{
}

/*
 * Unpacks a SBC frame at the beginning of the stream in data,
 * which has at most len bytes into frame.
 * Returns the length in bytes of the packed frame, or a negative
 * value on error. The error codes are:
 *
 *  -1   Data stream too short
 *  -2   Sync byte incorrect
 *  -3   CRC8 incorrect
 *  -4   Bitpool value out of bounds
 */
static int sbc_unpack_frame(const unsigned char *data, struct sbc_frame *frame,
                                unsigned int len)
{
    unsigned int consumed;
    /* Will copy the parts of the header that are relevant to crc
     * calculation here */
    unsigned char crc_header[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int crc_pos = 0;
    int temp;

    int audio_sample;
    int ch, sb, blk, bit;    /* channel, subband, block and bit standard
                   counters */
    int bits[2][8];        /* bits distribution */
    unsigned int levels[2][8];    /* levels derived from that */

    if (len < 4)
        return -1;

    if (data[0] != SBC_SYNCWORD)
        return -2;

    frame->frequency = (data[1] >> 6) & 0x03;

    frame->block_mode = (data[1] >> 4) & 0x03;
    switch (frame->block_mode) {
    case SBC_BLK_4:
        frame->blocks = 4;
        break;
    case SBC_BLK_8:
        frame->blocks = 8;
        break;
    case SBC_BLK_12:
        frame->blocks = 12;
        break;
    case SBC_BLK_16:
        frame->blocks = 16;
        break;
    }

    frame->mode = (data[1] >> 2) & 0x03;
    switch (frame->mode) {
    case MONO:
        frame->channels = 1;
        break;
    case DUAL_CHANNEL:    /* fall-through */
    case STEREO:
    case JOINT_STEREO:
        frame->channels = 2;
        break;
    }

    frame->allocation = (data[1] >> 1) & 0x01;

    frame->subband_mode = (data[1] & 0x01);
    frame->subbands = frame->subband_mode ? 8 : 4;

    frame->bitpool = data[2];

    if ((frame->mode == MONO || frame->mode == DUAL_CHANNEL) &&
            frame->bitpool > 16 * frame->subbands)
        return -4;

    if ((frame->mode == STEREO || frame->mode == JOINT_STEREO) &&
            frame->bitpool > 32 * frame->subbands)
        return -4;

    /* data[3] is crc, we're checking it later */

    consumed = 32;

    crc_header[0] = data[1];
    crc_header[1] = data[2];
    crc_pos = 16;

    if (frame->mode == JOINT_STEREO) {
        if (len * 8 < consumed + frame->subbands)
            return -1;

        frame->joint = 0x00;
        for (sb = 0; sb < frame->subbands - 1; sb++)
            frame->joint |= ((data[4] >> (7 - sb)) & 0x01) << sb;

        if (frame->subbands == 4)
            crc_header[crc_pos / 8] = data[4] & 0xf0;
        else
            crc_header[crc_pos / 8] = data[4];

        consumed += frame->subbands;
        crc_pos += frame->subbands;
    }

    if (len * 8 < consumed + (4 * frame->subbands * frame->channels))
        return -1;


    for (ch = 0; ch < frame->channels; ch++) {
        for (sb = 0; sb < frame->subbands; sb++) {
            /* FIXME assert(consumed % 4 == 0); */
            frame->scale_factor[ch][sb] =
                (data[consumed >> 3] >> (4 - (consumed & 0x7))) & 0x0F;

            crc_header[crc_pos >> 3] |=
                frame->scale_factor[ch][sb] << (4 - (crc_pos & 0x7));

            consumed += 4;
            crc_pos += 4;
        }
    }

    if (data[3] != sbc_crc8(crc_header, crc_pos))
        return -3;

    sbc_calculate_bits(frame, bits);

    for (ch = 0; ch < frame->channels; ch++) {
        for (sb = 0; sb < frame->subbands; sb++){
                  levels[ch][sb] = (1 << bits[ch][sb]) - 1;
        }
    }


    for (blk = 0; blk < frame->blocks; blk++) {
        for (ch = 0; ch < frame->channels; ch++) {
            for (sb = 0; sb < frame->subbands; sb++) {
                unsigned int shift;

                if (levels[ch][sb] == 0) {
                    frame->sb_sample[blk][ch][sb] = 0;
                    continue;
                }

                shift = frame->scale_factor[ch][sb] +
                        1 + SBCDEC_FIXED_EXTRA_BITS;

                audio_sample = 0;
                for (bit = 0; bit < bits[ch][sb]; bit++) {
                    if (consumed > len * 8)
                        return -1;

                    if ((data[consumed >> 3] >> (7 - (consumed & 0x7))) & 0x01)
                        audio_sample |= 1 << (bits[ch][sb] - bit - 1);

                    consumed++;
                }

                frame->sb_sample[blk][ch][sb] = (int)
                    (((((unsigned long long) audio_sample << 1) | 1) << shift) /
                    levels[ch][sb]) - (1 << shift);
            }
        }
    }


    if (frame->mode == SBC_JOINT_STEREO) {
        for (blk = 0; blk < frame->blocks; blk++) {
            for (sb = 0; sb < frame->subbands; sb++) {
                if (frame->joint & (0x01 << sb)) {
                    temp = frame->sb_sample[blk][0][sb] +
                        frame->sb_sample[blk][1][sb];
                    frame->sb_sample[blk][1][sb] =
                        frame->sb_sample[blk][0][sb] -
                        frame->sb_sample[blk][1][sb];
                    frame->sb_sample[blk][0][sb] = temp;
                }
            }
        }
    }

    if ((consumed & 0x7) != 0)
        consumed += 8 - (consumed & 0x7);

    return consumed >> 3;
}

static void sbc_decoder_init(struct sbc_decoder_state *state,
                    const struct sbc_frame *frame)
{
    int i, ch;

    memset(state->V, 0, sizeof(state->V));
    state->subbands = frame->subbands;

    for (ch = 0; ch < 2; ch++)
        for (i = 0; i < frame->subbands * 2; i++)
            state->offset[ch][i] = (10 * i + 10);
}

static inline short sbc_clip16(int s)
{
    if (s > 0x7FFF)
        return 0x7FFF;
    else if (s < -0x8000)
        return -0x8000;
    else
        return s;
}

static inline void sbc_synthesize_four(struct sbc_decoder_state *state,
                struct sbc_frame *frame, int ch, int blk)
{
    int i, k, idx;
    int *v = state->V[ch];
    int *offset = state->offset[ch];

    for (i = 0; i < 8; i++) {
        /* Shifting */
        offset[i]--;
        if (offset[i] < 0) {
            offset[i] = 79;
            memcpy(v + 80, v, 9 * sizeof(*v));
        }

        /* Distribute the new matrix value to the shifted position */
        v[offset[i]] = SCALE4_STAGED1(
            MULA(synmatrix4[i][0], frame->sb_sample[blk][ch][0],
            MULA(synmatrix4[i][1], frame->sb_sample[blk][ch][1],
            MULA(synmatrix4[i][2], frame->sb_sample[blk][ch][2],
            MUL (synmatrix4[i][3], frame->sb_sample[blk][ch][3])))));
    }

    /* Compute the samples */
    for (idx = 0, i = 0; i < 4; i++, idx += 5) {
        k = (i + 4) & 0xf;

        /* Store in output, Q0 */
        frame->pcm_sample[ch][blk * 4 + i] = sbc_clip16(SCALE4_STAGED1(
            MULA(v[offset[i] + 0], sbc_proto_4_40m0[idx + 0],
            MULA(v[offset[k] + 1], sbc_proto_4_40m1[idx + 0],
            MULA(v[offset[i] + 2], sbc_proto_4_40m0[idx + 1],
            MULA(v[offset[k] + 3], sbc_proto_4_40m1[idx + 1],
            MULA(v[offset[i] + 4], sbc_proto_4_40m0[idx + 2],
            MULA(v[offset[k] + 5], sbc_proto_4_40m1[idx + 2],
            MULA(v[offset[i] + 6], sbc_proto_4_40m0[idx + 3],
            MULA(v[offset[k] + 7], sbc_proto_4_40m1[idx + 3],
            MULA(v[offset[i] + 8], sbc_proto_4_40m0[idx + 4],
            MUL( v[offset[k] + 9], sbc_proto_4_40m1[idx + 4]))))))))))));
    }
}

static inline void sbc_synthesize_eight(struct sbc_decoder_state *state,
                struct sbc_frame *frame, int ch, int blk)
{
    int i, j, k, idx;
    int *offset = state->offset[ch];

    for (i = 0; i < 16; i++) {
        /* Shifting */
        offset[i]--;
        if (offset[i] < 0) {
            offset[i] = 159;
            for (j = 0; j < 9; j++)
                state->V[ch][j + 160] = state->V[ch][j];
        }

        /* Distribute the new matrix value to the shifted position */
        state->V[ch][offset[i]] = SCALE8_STAGED1(
            MULA(synmatrix8[i][0], frame->sb_sample[blk][ch][0],
            MULA(synmatrix8[i][1], frame->sb_sample[blk][ch][1],
            MULA(synmatrix8[i][2], frame->sb_sample[blk][ch][2],
            MULA(synmatrix8[i][3], frame->sb_sample[blk][ch][3],
            MULA(synmatrix8[i][4], frame->sb_sample[blk][ch][4],
            MULA(synmatrix8[i][5], frame->sb_sample[blk][ch][5],
            MULA(synmatrix8[i][6], frame->sb_sample[blk][ch][6],
            MUL( synmatrix8[i][7], frame->sb_sample[blk][ch][7])))))))));
    }

    /* Compute the samples */
    for (idx = 0, i = 0; i < 8; i++, idx += 5) {
        k = (i + 8) & 0xf;

        /* Store in output, Q0 */
        frame->pcm_sample[ch][blk * 8 + i] = sbc_clip16(SCALE8_STAGED1(
            MULA(state->V[ch][offset[i] + 0], sbc_proto_8_80m0[idx + 0],
            MULA(state->V[ch][offset[k] + 1], sbc_proto_8_80m1[idx + 0],
            MULA(state->V[ch][offset[i] + 2], sbc_proto_8_80m0[idx + 1],
            MULA(state->V[ch][offset[k] + 3], sbc_proto_8_80m1[idx + 1],
            MULA(state->V[ch][offset[i] + 4], sbc_proto_8_80m0[idx + 2],
            MULA(state->V[ch][offset[k] + 5], sbc_proto_8_80m1[idx + 2],
            MULA(state->V[ch][offset[i] + 6], sbc_proto_8_80m0[idx + 3],
            MULA(state->V[ch][offset[k] + 7], sbc_proto_8_80m1[idx + 3],
            MULA(state->V[ch][offset[i] + 8], sbc_proto_8_80m0[idx + 4],
            MUL( state->V[ch][offset[k] + 9], sbc_proto_8_80m1[idx + 4]))))))))))));
    }
}

static int sbc_synthesize_audio(struct sbc_decoder_state *state,
                        struct sbc_frame *frame)
{
    int ch, blk;

    switch (frame->subbands) {
    case 4:
        for (ch = 0; ch < frame->channels; ch++) {
            for (blk = 0; blk < frame->blocks; blk++)
                sbc_synthesize_four(state, frame, ch, blk);
        }
        return frame->blocks * 4;

    case 8:
        for (ch = 0; ch < frame->channels; ch++) {
            for (blk = 0; blk < frame->blocks; blk++)
                sbc_synthesize_eight(state, frame, ch, blk);
        }
        return frame->blocks * 8;

    default:
        return -EIO;
    }
}

/*
 * Code straight from the spec to calculate the bits array
 * Takes a pointer to the frame in question, a pointer to the bits array and
 * the sampling frequency (as 2 bit integer)
 */
static inline void sbc_calculate_bits_internal(
        const struct sbc_frame *frame, int (*bits)[8], int subbands)
{
    unsigned char sf = frame->frequency;

    if (frame->mode == SBC_MONO|| frame->mode == SBC_DUAL) {
        int bitneed[2][8], loudness, max_bitneed, bitcount, slicecount, bitslice;
        int ch, sb;

        for (ch = 0; ch < frame->channels; ch++) {
            max_bitneed = 0;
            if (frame->allocation == SBC_SNR) {
                for (sb = 0; sb < subbands; sb++) {
                    bitneed[ch][sb] = frame->scale_factor[ch][sb];
                    if (bitneed[ch][sb] > max_bitneed)
                        max_bitneed = bitneed[ch][sb];
                }
            } else {
                for (sb = 0; sb < subbands; sb++) {
                    if (frame->scale_factor[ch][sb] == 0)
                        bitneed[ch][sb] = -5;
                    else {
                        if (subbands == 4)
                            loudness = frame->scale_factor[ch][sb] - sbc_offset4[sf][sb];
                        else
                            loudness = frame->scale_factor[ch][sb] - sbc_offset8[sf][sb];
                        if (loudness > 0)
                            bitneed[ch][sb] = loudness / 2;
                        else
                            bitneed[ch][sb] = loudness;
                    }
                    if (bitneed[ch][sb] > max_bitneed)
                        max_bitneed = bitneed[ch][sb];
                }
            }

            bitcount = 0;
            slicecount = 0;
            bitslice = max_bitneed + 1;
            do {
                bitslice--;
                bitcount += slicecount;
                slicecount = 0;
                for (sb = 0; sb < subbands; sb++) {
                    if ((bitneed[ch][sb] > bitslice + 1) && (bitneed[ch][sb] < bitslice + 16))
                        slicecount++;
                    else if (bitneed[ch][sb] == bitslice + 1)
                        slicecount += 2;
                }
            } while (bitcount + slicecount < frame->bitpool);

            if (bitcount + slicecount == frame->bitpool) {
                bitcount += slicecount;
                bitslice--;
            }

            for (sb = 0; sb < subbands; sb++) {
                if (bitneed[ch][sb] < bitslice + 2)
                    bits[ch][sb] = 0;
                else {
                    bits[ch][sb] = bitneed[ch][sb] - bitslice;
                    if (bits[ch][sb] > 16)
                        bits[ch][sb] = 16;
                }
            }

            for (sb = 0; bitcount < frame->bitpool &&
                            sb < subbands; sb++) {
                if ((bits[ch][sb] >= 2) && (bits[ch][sb] < 16)) {
                    bits[ch][sb]++;
                    bitcount++;
                } else if ((bitneed[ch][sb] == bitslice + 1) && (frame->bitpool > bitcount + 1)) {
                    bits[ch][sb] = 2;
                    bitcount += 2;
                }
            }

            for (sb = 0; bitcount < frame->bitpool &&
                            sb < subbands; sb++) {
                if (bits[ch][sb] < 16) {
                    bits[ch][sb]++;
                    bitcount++;
                }
            }

        }

    }
    else if (frame->mode == SBC_STEREO|| frame->mode == SBC_JOINT_STEREO) {
        int bitneed[2][8], loudness, max_bitneed, bitcount, slicecount, bitslice;
        int ch, sb;

        max_bitneed = 0;
        if (frame->allocation == SBC_SNR) {
            for (ch = 0; ch < 2; ch++) {
                for (sb = 0; sb < subbands; sb++) {
                    bitneed[ch][sb] = frame->scale_factor[ch][sb];
                    if (bitneed[ch][sb] > max_bitneed)
                        max_bitneed = bitneed[ch][sb];
                }
            }
        } else {
            for (ch = 0; ch < 2; ch++) {
                for (sb = 0; sb < subbands; sb++) {
                    if (frame->scale_factor[ch][sb] == 0)
                        bitneed[ch][sb] = -5;
                    else {
                        if (subbands == 4)
                            loudness = frame->scale_factor[ch][sb] - sbc_offset4[sf][sb];
                        else
                            loudness = frame->scale_factor[ch][sb] - sbc_offset8[sf][sb];
                        if (loudness > 0)
                            bitneed[ch][sb] = loudness / 2;
                        else
                            bitneed[ch][sb] = loudness;
                    }
                    if (bitneed[ch][sb] > max_bitneed)
                        max_bitneed = bitneed[ch][sb];
                }
            }
        }

        bitcount = 0;
        slicecount = 0;
        bitslice = max_bitneed + 1;
        do {
            bitslice--;
            bitcount += slicecount;
            slicecount = 0;
            for (ch = 0; ch < 2; ch++) {
                for (sb = 0; sb < subbands; sb++) {
                    if ((bitneed[ch][sb] > bitslice + 1) && (bitneed[ch][sb] < bitslice + 16))
                        slicecount++;
                    else if (bitneed[ch][sb] == bitslice + 1)
                        slicecount += 2;
                }
            }
        } while (bitcount + slicecount < frame->bitpool);

        if (bitcount + slicecount == frame->bitpool) {
            bitcount += slicecount;
            bitslice--;
        }

        for (ch = 0; ch < 2; ch++) {
            for (sb = 0; sb < subbands; sb++) {
                if (bitneed[ch][sb] < bitslice + 2) {
                    bits[ch][sb] = 0;
                } else {
                    bits[ch][sb] = bitneed[ch][sb] - bitslice;
                    if (bits[ch][sb] > 16)
                        bits[ch][sb] = 16;
                }
            }
        }

        ch = 0;
        sb = 0;
        while (bitcount < frame->bitpool) {
            if ((bits[ch][sb] >= 2) && (bits[ch][sb] < 16)) {
                bits[ch][sb]++;
                bitcount++;
            } else if ((bitneed[ch][sb] == bitslice + 1) && (frame->bitpool > bitcount + 1)) {
                bits[ch][sb] = 2;
                bitcount += 2;
            }
            if (ch == 1) {
                ch = 0;
                sb++;
                if (sb >= subbands)
                    break;
            } else
                ch = 1;
        }

        ch = 0;
        sb = 0;
        while (bitcount < frame->bitpool) {
            if (bits[ch][sb] < 16) {
                bits[ch][sb]++;
                bitcount++;
            }
            if (ch == 1) {
                ch = 0;
                sb++;
                if (sb >= subbands)
                    break;
            } else
                ch = 1;
        }

    }

}


static void sbc_calculate_bits(const struct sbc_frame *frame, int (*bits)[8])
{
    if (frame->subbands == 4)
        sbc_calculate_bits_internal(frame, bits, 4);
    else
        sbc_calculate_bits_internal(frame, bits, 8);
}



