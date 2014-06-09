/*
 *
 *  Bluetooth low-complexity, subband codec (SBC) decoder
 *
 *  Copyright (C) 2014  Tieto Corporation
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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <dlfcn.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>

#include "../encoder/include/sbc_types.h"
#include "../../../bluedroid/include/bt_trace.h"

#include "sbc_decoder.h"
#include "sbc_debug.h"

#define BUF_SIZE 8192
#define FILESIZE 7000

#define NUM 119*1

#define bswap_16(x) \
({ \
UINT16 __x = (x); \
((UINT16)( \
        (((UINT16)(__x) & (UINT16)0x00ffU) << 8) | \
        (((UINT16)(__x) & (UINT16)0xff00U) >> 8) )); \
})

#define bswap_32(x) \
({ \
    UINT32 __x = (x); \
    ((UINT32)( \
        (((UINT32)(__x) & (UINT32)0x000000ffUL) << 24) | \
        (((UINT32)(__x) & (UINT32)0x0000ff00UL) <<  8) | \
        (((UINT32)(__x) & (UINT32)0x00ff0000UL) >>  8) | \
        (((UINT32)(__x) & (UINT32)0xff000000UL) >> 24) )); \
})

#if __BIG_ENDIAN != TRUE
#define COMPOSE_ID(a,b,c,d)    ((a) | ((b)<<8) | ((c)<<16) | ((d)<<24))
#define LE_SHORT(v)        (v)
#define LE_INT(v)        (v)
#define BE_SHORT(v)        bswap_16(v)
#define BE_INT(v)        bswap_32(v)
#else
#define COMPOSE_ID(a,b,c,d)    ((d) | ((c)<<8) | ((b)<<16) | ((a)<<24))
#define LE_SHORT(v)        bswap_16(v)
#define LE_INT(v)        bswap_32(v)
#define BE_SHORT(v)        (v)
#define BE_INT(v)        (v)
#endif

//#define COMPOSE_ID(a,b,c,d)    ((a) | ((b)<<8) | ((c)<<16) | ((d)<<24))
#define AU_MAGIC        COMPOSE_ID('.','s','n','d')

//#define O_ACCMODE    00000003
//#define O_RDONLY    00000000
//#define O_WRONLY    00000001

/* allocation method */
#define SBC_AM_LOUDNESS        0x00
#define SBC_AM_SNR        0x01

static int verbose = 1;

#define AU_FMT_ULAW        1
#define AU_FMT_LIN8        2
#define AU_FMT_LIN16        3

struct au_header {
    UINT32 magic;        /* '.snd' */
    UINT32 hdr_size;    /* size of header (min 24) */
    UINT32 data_size;    /* size of data */
    UINT32 encoding;    /* see to AU_FMT_XXXX */
    UINT32 sample_rate;    /* sample rate */
    UINT32 channels;    /* number of channels (voices) */
};

struct stat
{
    UINT32 st_size;
};

struct option
{
#if defined (__STDC__) && __STDC__
  const char *name;
#else
  char *name;
#endif
  /* has_arg can't be an enum because some compilers complain about
     type mismatches in all the code that assumes it is an int.  */
  int has_arg;
  int *flag;
  int val;
};

static void decode(char *filename, char *output, int tofile);
static void usage(void);
int main(int argc, char *argv[]);

void bdt_log(const char *fmt_str, ...)
{
    static char buffer[1024];
    va_list ap;

    va_start(ap, fmt_str);
    vsnprintf(buffer, 1024, fmt_str, ap);
    va_end(ap);

    fprintf(stdout, "%s\n", buffer);
}

unsigned char TransferBinary2Int(unsigned char* p)
{
    int i=0,sum=0;
    char str[8];
    int len=8;
      int temp=0;

    memcpy(str,p,8);
    static int n=0;


    for (i=0;i<len;i++)
    {
        sum+=(str[len-i-1]-'0')*(1<<i);
    }
      n++;
    printf("%03d:%03d  ",n,sum);

    for (i=0;i<len;i++)
    {
        temp=str[i]-'0';
        printf("%d",temp);
    }
    printf("\n");

    return (unsigned char)sum;
}

int  getIntBufFromBinary(unsigned char* pBinaryBuf, unsigned char* pInfBuf)
{
    unsigned char* pstream=pBinaryBuf,*pbuf=pInfBuf;
    int n=0;

    while (*pstream!='\n')
    {
        *pbuf=(unsigned char)TransferBinary2Int(pstream);
        pstream+=9;
        pbuf++;

        n++;
    }

    return n;
}

void sbc_finish(sbc_t *sbc)
{
    if (!sbc)
        return;

    free(sbc->priv_alloc_base);

    memset(sbc, 0, sizeof(sbc_t));
}

static void decode(char *filename, char *output, int tofile)
{
    unsigned char buf[BUF_SIZE], *stream,*pBuf, *p;
    struct stat st;
    st.st_size=8*NUM;
    sbc_t sbc;
    int fd, ad, pos, streamlen, framelen, count;
    UINT32 len;
    int  frequency, channels; //format = AFMT_S16_BE,
    UINT32 written;
    FILE *fp=NULL;
    int n=0;

    stream =(unsigned char*) malloc(FILESIZE);
    memset(stream,0, FILESIZE);
    p=stream;

    pBuf=(unsigned char*) malloc(FILESIZE/8);
    memset(pBuf,0, FILESIZE/8);

    if (!stream) {
        bdt_log("Can't allocate memory for %s\n",
        filename);

        return;
    }

    fp = fopen(filename, "r");
    if (!fp) {
        bdt_log( "Can't open file %s\n",
        filename);
        goto free;
    }

    while (!feof(fp))
    {
        fread(p++,1,1,fp);
        n++;

        if (feof(fp))
        {
            break;
        }
    }
    *(p)='\n';

    streamlen= getIntBufFromBinary(stream, pBuf);

    fclose(fp);

    pos = 0;


    if (tofile)
        ad = open(output, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    else
        ad = open(output, O_WRONLY, 0);

    if (ad < 0) {
        bdt_log( "Can't open output %s\n",
                        output);

        goto free;
    }



    sbc_init(&sbc, 0L);
    sbc.endian = SBC_BE;

    framelen = sbc_decode(&sbc, pBuf, streamlen, buf, sizeof(buf), &len);
    channels = sbc.mode == SBC_MONO ? 1 : 2;
    switch (sbc.frequency) {
    case SBC_FREQ_16000:
        frequency = 16000;
        break;

    case SBC_FREQ_32000:
        frequency = 32000;
        break;

    case SBC_FREQ_44100:
        frequency = 44100;
        break;

    case SBC_FREQ_48000:
        frequency = 48000;
        break;
    default:
        frequency = 0;
    }
#if 1
    if (verbose) {
        bdt_log("decoding %s with rate %d, %d subbands, "
            "%d bits, allocation method %s and mode %s\n",
            filename, frequency, sbc.subbands * 4 + 4, sbc.bitpool,
            sbc.allocation == SBC_AM_SNR ? "SNR" : "LOUDNESS",
            sbc.mode == SBC_MODE_MONO ? "MONO" :
                    sbc.mode == SBC_MODE_STEREO ?
                        "STEREO" : "JOINTSTEREO");
        }
#endif
#if 1
    if (tofile) {
        struct au_header au_hdr;

        au_hdr.magic       = AU_MAGIC;
        au_hdr.hdr_size    = BE_INT(24);
        au_hdr.data_size   =BE_INT (0);
        au_hdr.encoding    = BE_INT(AU_FMT_LIN16);
        au_hdr.sample_rate =BE_INT (frequency);
        au_hdr.channels    =BE_INT (channels);

        written = write(ad, &au_hdr, sizeof(au_hdr));
        if (written < (UINT32) sizeof(au_hdr)) {
            bdt_log( "Failed to write header\n");
            goto close;
        }
    }
#endif
    #if 0
    else {
        if (ioctl(ad, SNDCTL_DSP_SETFMT, &format) < 0) {
            //fprintf(stderr, "Can't set audio format on %s: %s\n",
                        //output, strerror(errno));
            goto close;
        }

        if (ioctl(ad, SNDCTL_DSP_CHANNELS, &channels) < 0) {
            //fprintf(stderr, "Can't set number of channels on %s: %s\n",
                    //    output, strerror(errno));
            goto close;
        }

        if (ioctl(ad, SNDCTL_DSP_SPEED, &frequency) < 0) {
            //fprintf(stderr, "Can't set audio rate on %s: %s\n",
                        //output, strerror(errno));
            goto close;
        }
    }
#endif
    count = len;

    while (framelen > 0) {
        /* we have completed an sbc_decode at this point sbc.len is the
         * length of the frame we just decoded count is the number of
         * decoded bytes yet to be written */

        if (count + len >= BUF_SIZE) {
            /* buffer is too full to stuff decoded audio in so it
             * must be written to the device */
            written = write(ad, buf, count);
            if (written > 0)
                count -= written;
        }

        /* sanity check */
        if (count + len >= BUF_SIZE) {
            //fprintf(stderr,
                //"buffer size of %d is too small for decoded"
                //" data (%lu)\n", BUF_SIZE, (unsigned long) (len + count));
            exit(1);
        }

        /* push the pointer in the file forward to the next bit to be
         * decoded tell the decoder to decode up to the remaining
         * length of the file (!) */
        pos += framelen;
        framelen = sbc_decode(&sbc, pBuf + pos, streamlen - pos,
                    buf + count, sizeof(buf) - count, &len);

        /* increase the count */
        count += len;
    }

    if (count > 0) {
        written = write(ad, buf, count);
        if (written > 0)
            count -= written;
    }

close:
    sbc_finish(&sbc);

    close(ad);

free:
    free(stream);
    free(pBuf);
}

static void usage(void)
{
    //printf("SBC decoder utility ver %s\n", VERSION);
    printf("Copyright (c) 2004-2010  Marcel Holtmann\n\n");

    printf("Usage:\n"
        "\tsbcdec [options] file(s)\n"
        "\n");

    printf("Options:\n"
        "\t-h, --help           Display help\n"
        "\t-v, --verbose        Verbose mode\n"
        "\t-d, --device <dsp>   Sound device\n"
        "\t-f, --file <file>    Decode to a file\n"
        "\n");
}

static struct option main_options[] = {
    { "help",    0, 0, 'h' },
    { "device",    1, 0, 'd' },
    { "verbose",    0, 0, 'v' },
    { "file",    1, 0, 'f' },
    { 0, 0, 0, 0 }
};

int main(int argc, char *argv[])
{

    char *output = NULL;
    int i, opt, tofile = 0;

    while ((opt = getopt_long(argc, argv, "+hvd:f:",
                        main_options, NULL)) != -1) {
        switch(opt) {
        case 'h':
            usage();
            exit(0);

        case 'v':
            verbose = 1;
            break;

        case 'd':
            free(output);
            output = strdup(optarg);
            tofile = 0;
            break;

        case 'f' :
            free(output);
            output = strdup(optarg);
            tofile = 1;
            break;

        default:
            exit(1);
        }
    }

    argc -= optind;
    argv += optind;
    optind = 0;

    if (argc < 1) {
        usage();
        exit(1);
    }

    for (i = 0; i < argc; i++)
        decode(argv[i], output ? output : "/dev/dsp", tofile);

    free(output);

    return 0;
}
