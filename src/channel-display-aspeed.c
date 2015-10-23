/* -*- Mode: C; c-basic-offset: 4; indent-tabs-mode: nil -*- */
/*
   Copyright (C) 2010 Red Hat, Inc.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/
#include "config.h"

#include "spice-client.h"
#include "spice-common.h"
#include "spice-channel-priv.h"

#include "channel-display-priv.h"

struct ASTHeader
{
    short version;
    short headlen;

    short src_mode_x;
    short src_mode_y;
    short src_mode_depth;
    short src_mode_rate;
    char src_mode_index;

    short dst_mode_x;
    short dst_mode_y;
    short dst_mode_depth;
    short dst_mode_rate;
    char dst_mode_index;

    int frame_start;
    int frame_num;
    short frame_vsize;
    short frame_hsize;

    int rsvd[2];

    char compression;
    char jpeg_scale;
    char jpeg_table;
    char jpeg_yuv;
    char sharp_mode;
    char adv_table;
    char adv_scale;
    int num_of_MB;
    char rc4_en;
    char rc4_reset;

    char mode420;

    char inf_downscale;
    char inf_diff;
    short inf_analog_thr;
    short inf_dig_thr;
    char inf_ext_sig;
    char inf_auto_mode;
    char inf_vqmode;

    int comp_frame_size;
    int comp_size;
    int comp_hdebug;
    int comp_vdebug;

    char input_signal;
    short cur_xpos;
    short cur_ypos;
} __attribute__((packed));

struct HuffmanTable {
    int8_t Length[17];
    short minor_code[17];
    short major_code[17];
    short V[65536];
    short Len[65536];
};

struct ast_decoder {
    int WIDTH;
    int HEIGHT;
    int RealWIDTH;
    int RealHEIGHT;
    int tmp_WIDTHBy16;
    int tmp_HEIGHTBy16;
    int8_t SCALEFACTOR;
    int8_t SCALEFACTORUV;
    int8_t ADVANCESCALEFACTOR;
    int8_t ADVANCESCALEFACTORUV;
    int byte_pos;
    int m_Mode420;
    int selector;
    int advance_selector;
    int Mapping;
    int32_t *buf;
    uint32_t length;
    int32_t *m_decodeBuf;
    int _index;
    int m_newbits;
    int txb;
    int tyb;
    struct {
        int32_t Color[4];
        uint8_t Index[4];
        uint8_t BitMapBits;
    } m_VQ;
    struct HuffmanTable m_HTDC[4];
    struct HuffmanTable m_HTAC[4];
    int32_t YUVTile[768];
    int8_t  Y[64];
    int8_t  Cb[64];
    int8_t  Cr[64];
    int8_t  rangeLimitTable[1408];
    int32_t calculatedRGBofY[256];
    int32_t calculatedRGBofCrToR[256];
    int32_t calculatedRGBofCbToB[256];
    int32_t calculatedRGBofCrToG[256];
    int32_t calculatedRGBofCbToG[256];
    int32_t *YValueInTile;
    int32_t YValueInTile420[4][64];
    int32_t CbValueInTile[64];
    int32_t CrValueInTile[64];

    int32_t m_QT[4][64];

    int32_t previousYUVData[0x753000];
};

#include <byteswap.h>

#define GET_SHORT(byte0) ((short)(byte0 & 0xff))
#define WORD_hi_lo(byte0, byte1) (GET_SHORT(byte0) << 8 | GET_SHORT(byte1))
#define GET_INT(i) ((int)(i & 0xffff))

#if 0
#define GET_LONG(x) (x)
#else
static uint32_t GET_LONG(uint32_t x) {
    printf("-- %x\n", x);
    return x;
}
#endif

#define FIX_G(d) (int)((double)d * 65536 + 0.5)

static void initColorTable(struct ast_decoder *dec)
{
    int l = 0x10000;
    int i1 = l >> 1;
    int i = 0;
    for (int j = -128; i < 256; j++) {
        dec->calculatedRGBofCrToR[i] = (FIX_G(1.597656) * j + i1) >> 16;
        dec->calculatedRGBofCbToB[i] = (FIX_G(2.015625) * j + i1) >> 16;
        dec->calculatedRGBofCrToG[i] = (-FIX_G(0.8125) * j + i1) >> 16;
        dec->calculatedRGBofCbToG[i] = (-FIX_G(0.390625) * j + i1) >> 16;
        i++;
    }

    i = 0;
    for (int k = -16; i < 256; k++) {
        dec->calculatedRGBofY[i] = (FIX_G(1.1639999999999999) * k + i1) >> 16;
        i++;
    }
}

static void initRangeLimitTable(struct ast_decoder *dec)
{
    memset(dec->rangeLimitTable, 0, 255);
    for (short word0 = 0; word0 < 256; word0++)
    {
        dec->rangeLimitTable[256 + word0] = (int8_t)word0;
//        rangeLimitTableShort[256 + word0] = word0;
    }

    memset(dec->rangeLimitTable+512, -1, 896-512);
//    Arrays.fill(rangeLimitTableShort, 512, 895, (short)255);
    memset(dec->rangeLimitTable+896, 0, 1280-896);
//    Arrays.fill(rangeLimitTableShort, 896, 1279, (short)0);
    for (short word1 = 1280; word1 < 1408; word1++) {
        dec->rangeLimitTable[word1] = (int8_t)word1;
//        rangeLimitTableShort[word1] = (short)(word1 & 0xff);
    }
}

#include "channel-display-aspeed-jtables.h"

static void loadHuffmanTable(struct HuffmanTable *huffmantable, int8_t *abyte0, short *aword0, int *ai)
{
    for (int8_t byte2 = 1; byte2 <= 16; byte2++)
        huffmantable->Length[byte2] = abyte0[byte2];

    int i = 0;
    for (int8_t byte0 = 1; byte0 <= 16; byte0++)
    {
        for (int8_t byte3 = 0; byte3 < GET_SHORT(huffmantable->Length[byte0]); byte3++)
        {
            huffmantable->V[GET_INT(WORD_hi_lo(byte0, byte3))] = aword0[i];
            i++;
        }
    }

    int j = 0;
    for (int8_t byte1 = 1; byte1 <= 16; byte1++)
    {
        huffmantable->minor_code[byte1] = (short)j;
        for (int8_t byte4 = 1; byte4 <= GET_SHORT(huffmantable->Length[byte1]); byte4++)
            j++;

        huffmantable->major_code[byte1] = (short)(j - 1);
        j *= 2;
        if (GET_SHORT(huffmantable->Length[byte1]) == 0)
        {
            huffmantable->minor_code[byte1] = -1;
            huffmantable->major_code[byte1] = 0;
        }
    }

    huffmantable->Len[0] = 2;
    i = 2;
    for (int k = 1; k < 65535; k++) {
        if(k < ai[i])
        {
            huffmantable->Len[k] = (int8_t)(ai[i + 1] & 0xff);
        } else
        {
            i += 2;
            huffmantable->Len[k] = (int8_t)(ai[i + 1] & 0xff);
        }
    }
}

static void initHuffmanTable(struct ast_decoder *dec)
{
    loadHuffmanTable(&dec->m_HTDC[0],
                     std_dc_luminance_nrcodes,
                     std_dc_luminance_values,
                     DC_LUMINANCE_HUFFMANCODE);
    loadHuffmanTable(&dec->m_HTAC[0],
                     std_ac_luminance_nrcodes,
                     std_ac_luminance_values,
                     AC_LUMINANCE_HUFFMANCODE);
    loadHuffmanTable(&dec->m_HTDC[1],
                     std_dc_chrominance_nrcodes,
                     std_dc_chrominance_values,
                     DC_CHROMINANCE_HUFFMANCODE);
    loadHuffmanTable(&dec->m_HTAC[1],
                     std_ac_chrominance_nrcodes,
                     std_ac_chrominance_values,
                     AC_CHROMINANCE_HUFFMANCODE);
}

void convertYUVtoRGB(struct ast_decoder *dec, int i, int j)
{
        if(dec->m_Mode420 == 0)
        {
            dec->YValueInTile = dec->YUVTile;
            for(int k = 0; k < 64; k++)
            {
                dec->CbValueInTile[k] = dec->YUVTile[64 + k];
                dec->CrValueInTile[k] = dec->YUVTile[128 + k];
            }

            int k5 = i * 8;
            int i6 = j * 8;
            int k2 = i6 * dec->RealWIDTH + k5;
            int j8 = dec->RealWIDTH - k5;
            if(j8 == 0 || j8 > 8)
                j8 = 8;
            for(int k1 = 0; k1 < 8; k1++)
            {
                for(int l = 0; l < j8; l++)
                {
                    int i3 = (k1 << 3) + l;
                    int k3 = (k2 + l) * 3;
                    int i4 = dec->YValueInTile[i3];
                    int k4 = dec->CbValueInTile[i3];
                    int i5 = dec->CrValueInTile[i3];
                    dec->previousYUVData[k3] = i4;
                    dec->previousYUVData[k3 + 1] = k4;
                    dec->previousYUVData[k3 + 2] = i5;
                    int l6 = dec->calculatedRGBofY[i4] + dec->calculatedRGBofCbToB[k4];
                    int j7 = dec->calculatedRGBofY[i4] + (dec->calculatedRGBofCbToG[k4] + dec->calculatedRGBofCrToG[i5]);
                    int l7 = dec->calculatedRGBofY[i4] + dec->calculatedRGBofCrToR[i5];
                    if(l6 >= 0)
                        l6 += 256;
                    else
                        l6 = 0;
                    if(j7 >= 0)
                        j7 += 256;
                    else
                        j7 = 0;
                    if(l7 >= 0)
                        l7 += 256;
                    else
                        l7 = 0;
                    if(k3 < dec->RealWIDTH * dec->RealHEIGHT * 3)
                    {
                        dec->m_decodeBuf[k3] = dec->rangeLimitTable[l6];
                        dec->m_decodeBuf[k3 + 1] = dec->rangeLimitTable[j7];
                        dec->m_decodeBuf[k3 + 2] = dec->rangeLimitTable[l7];
                        printf("444 %d %d, %d %d %d\n", k5 + l, i6 + k1, dec->m_decodeBuf[k3], dec->m_decodeBuf[k3 + 1], dec->m_decodeBuf[k3 + 2]);
                    }
                }

                k2 += dec->RealWIDTH;
            }

        } else
        {
            int k6 = 0;
            for(int i1 = 0; i1 < 4; i1++)
            {
                for(int l1 = 0; l1 < 64; l1++)
                {
                    dec->YValueInTile420[i1][l1] = dec->YUVTile[k6];
                    k6++;
                }

            }

            for(int i2 = 0; i2 < 64; i2++)
            {
                dec->CbValueInTile[i2] = dec->YUVTile[k6];
                dec->CrValueInTile[i2] = dec->YUVTile[k6 + 64];
                k6++;
            }

            int l5 = i * 16;
            int j6 = j * 16;
            int l2 = j6 * dec->WIDTH + l5;
            int l8 = 0;
            int i9 = 0;
            int j9 = 0;
            int k9 = 0;
            int8_t byte1 = 16;
            if(dec->HEIGHT == 608 && j == 37)
                byte1 = 8;
            for(int j2 = 0; j2 < byte1; j2++)
            {
                int i10 = (j2 >> 3) * 2;
                int j10 = (j2 >> 1) << 3;
                for(int j1 = 0; j1 < 16; j1++)
                {
                    int l9 = i10 + (j1 >> 3);
                    int k8;
                    switch(l9)
                    {
                    case 0: // '\0'
                        k8 = l8++;
                        break;

                    case 1: // '\001'
                        k8 = i9++;
                        break;

                    case 2: // '\002'
                        k8 = j9++;
                        break;

                    default:
                        k8 = k9++;
                        break;
                    }
                    int l3 = (l2 + j1) * 3;
                    int j3 = j10 + (j1 >> 1);
                    int j4 = dec->YValueInTile420[l9][k8];
                    int l4 = dec->CbValueInTile[j3];
                    int j5 = dec->CrValueInTile[j3];
                    int i7 = dec->calculatedRGBofY[j4] + dec->calculatedRGBofCbToB[l4];
                    int k7 = dec->calculatedRGBofY[j4] + (dec->calculatedRGBofCbToG[l4] + dec->calculatedRGBofCrToG[j5]);
                    int i8 = dec->calculatedRGBofY[j4] + dec->calculatedRGBofCrToR[j5];
                    if(i7 >= 0)
                        dec->m_decodeBuf[l3] = dec->rangeLimitTable[i7 + 256];
                    else
                        dec->m_decodeBuf[l3] = 0;
                    if(k7 >= 0)
                        dec->m_decodeBuf[l3 + 1] = dec->rangeLimitTable[k7 + 256];
                    else
                        dec->m_decodeBuf[l3 + 1] = 0;
                    if(i8 >= 0)
                        dec->m_decodeBuf[l3 + 2] = dec->rangeLimitTable[i8 + 256];
                    else
                        dec->m_decodeBuf[l3 + 2] = 0;
                    printf("420 %d %d, %d %d %d\n", l5 + j1, j6 + j2, dec->m_decodeBuf[l3], dec->m_decodeBuf[l3 + 1], dec->m_decodeBuf[l3 + 2]);
                }

                l2 += dec->RealWIDTH;
            }
        }
}

static void setQuantizationTable(int8_t *abyte0, int8_t byte0, int8_t *abyte1)
{
    for(int8_t byte1 = 0; byte1 < 64; byte1++)
    {
        int i = (abyte0[byte1] * 16) / byte0;
        if (i <= 0)
            i = 1;
        if (i > 255)
            i = 255;
        abyte1[zigzag[byte1]] = (int8_t)i;
    }
}

static void loadLuminanceQuantizationTable(struct ast_decoder *dec, int32_t *al)
{
    float af[] = {
        1.0F, 1.38704F, 1.306563F, 1.175876F, 1.0F, 0.785695F, 0.5411961F, 0.2758994F
    };
    int8_t abyte0[64];
    int8_t *std_luminance_qt;

    switch (dec->selector)
    {
    case 0: // '\0'
        std_luminance_qt = Tbl_000Y;
        break;

    case 1: // '\001'
        std_luminance_qt = Tbl_014Y;
        break;

    case 2: // '\002'
        std_luminance_qt = Tbl_029Y;
        break;

    case 3: // '\003'
        std_luminance_qt = Tbl_043Y;
        break;

    case 4: // '\004'
        std_luminance_qt = Tbl_057Y;
        break;

    case 5: // '\005'
        std_luminance_qt = Tbl_071Y;
        break;

    case 6: // '\006'
        std_luminance_qt = Tbl_086Y;
        break;

    case 7: // '\007'
        std_luminance_qt = Tbl_100Y;
        break;

    default:
        return;
    }
    setQuantizationTable(std_luminance_qt, dec->SCALEFACTOR, abyte0);
    for (int8_t byte0 = 0; byte0 <= 63; byte0++)
        al[byte0] = GET_SHORT(abyte0[zigzag[byte0]]);

    int8_t byte1 = 0;
    for(int8_t byte2 = 0; byte2 <= 7; byte2++)
    {
        for(int8_t byte3 = 0; byte3 <= 7; byte3++)
        {
            int i = (int)((float)al[byte1] * (af[byte2] * af[byte3]));
            al[byte1] = i * 0x10000;
            byte1++;
        }

    }

    dec->byte_pos += 64;
}

void loadChrominanceQuantizationTable(struct ast_decoder *dec, int32_t *al)
{
    float af[] = {
        1.0F, 1.38704F, 1.306563F, 1.175876F, 1.0F, 0.785695F, 0.5411961F, 0.2758994F
    };
    int8_t abyte0[64];
    int8_t *std_chrominance_qt;

    if (dec->Mapping == 1)
        switch(dec->selector)
        {
        case 0: // '\0'
            std_chrominance_qt = Tbl_000Y;
            break;

        case 1: // '\001'
            std_chrominance_qt = Tbl_014Y;
            break;

        case 2: // '\002'
            std_chrominance_qt = Tbl_029Y;
            break;

        case 3: // '\003'
            std_chrominance_qt = Tbl_043Y;
            break;

        case 4: // '\004'
            std_chrominance_qt = Tbl_057Y;
            break;

        case 5: // '\005'
            std_chrominance_qt = Tbl_071Y;
            break;

        case 6: // '\006'
            std_chrominance_qt = Tbl_086Y;
            break;

        case 7: // '\007'
            std_chrominance_qt = Tbl_100Y;
            break;

        default:
            return;
        }
    else
        switch(dec->selector)
        {
        case 0: // '\0'
            std_chrominance_qt = Tbl_000UV;
            break;

        case 1: // '\001'
            std_chrominance_qt = Tbl_014UV;
            break;

        case 2: // '\002'
            std_chrominance_qt = Tbl_029UV;
            break;

        case 3: // '\003'
            std_chrominance_qt = Tbl_043UV;
            break;

        case 4: // '\004'
            std_chrominance_qt = Tbl_057UV;
            break;

        case 5: // '\005'
            std_chrominance_qt = Tbl_071UV;
            break;

        case 6: // '\006'
            std_chrominance_qt = Tbl_086UV;
            break;

        case 7: // '\007'
            std_chrominance_qt = Tbl_100UV;
            break;

        default:
            return;
        }
    setQuantizationTable(std_chrominance_qt, dec->SCALEFACTORUV, abyte0);
    for(int8_t byte0 = 0; byte0 <= 63; byte0++)
        al[byte0] = GET_SHORT(abyte0[zigzag[byte0]]);

    int8_t byte1 = 0;
    for(int8_t byte2 = 0; byte2 <= 7; byte2++)
    {
        for(int8_t byte3 = 0; byte3 <= 7; byte3++)
        {
            int i = (int)((float)al[byte1] * (af[byte2] * af[byte3]));
            al[byte1] = i * 0x10000;
            byte1++;
        }
    }

    dec->byte_pos += 64;
}

void loadPass2LuminanceQuantizationTable(struct ast_decoder *dec, int32_t *al)
{
    float af[] = {
        1.0F, 1.38704F, 1.306563F, 1.175876F, 1.0F, 0.785695F, 0.5411961F, 0.2758994F
    };
    int8_t abyte0[64];
    int8_t *std_luminance_qt;

    switch(dec->advance_selector)
    {
    case 0: // '\0'
        std_luminance_qt = Tbl_000Y;
        break;

    case 1: // '\001'
        std_luminance_qt = Tbl_014Y;
        break;

    case 2: // '\002'
        std_luminance_qt = Tbl_029Y;
        break;

    case 3: // '\003'
        std_luminance_qt = Tbl_043Y;
        break;

    case 4: // '\004'
        std_luminance_qt = Tbl_057Y;
        break;

    case 5: // '\005'
        std_luminance_qt = Tbl_071Y;
        break;

    case 6: // '\006'
        std_luminance_qt = Tbl_086Y;
        break;

    case 7: // '\007'
        std_luminance_qt = Tbl_100Y;
        break;

    default:
        return;
    }
    setQuantizationTable(std_luminance_qt, dec->ADVANCESCALEFACTOR, abyte0);
    for(int8_t byte0 = 0; byte0 <= 63; byte0++)
        al[byte0] = GET_SHORT(abyte0[zigzag[byte0]]);

    int8_t byte1 = 0;
    for(int8_t byte2 = 0; byte2 <= 7; byte2++)
    {
        for(int8_t byte3 = 0; byte3 <= 7; byte3++)
        {
            int i = (int)((float)al[byte1] * (af[byte2] * af[byte3]));
            al[byte1] = i * 0x10000;
            byte1++;
        }

    }

    dec->byte_pos += 64;
}

void loadPass2ChrominanceQuantizationTable(struct ast_decoder *dec, int32_t *al)
{
    float af[] = {
        1.0F, 1.38704F, 1.306563F, 1.175876F, 1.0F, 0.785695F, 0.5411961F, 0.2758994F
    };
    int8_t abyte0[64];
    int8_t *std_chrominance_qt;

    if(dec->Mapping == 1)
        switch(dec->advance_selector)
        {
        case 0: // '\0'
            std_chrominance_qt = Tbl_000Y;
            break;

        case 1: // '\001'
            std_chrominance_qt = Tbl_014Y;
            break;

        case 2: // '\002'
            std_chrominance_qt = Tbl_029Y;
            break;

        case 3: // '\003'
            std_chrominance_qt = Tbl_043Y;
            break;

        case 4: // '\004'
            std_chrominance_qt = Tbl_057Y;
            break;

        case 5: // '\005'
            std_chrominance_qt = Tbl_071Y;
            break;

        case 6: // '\006'
            std_chrominance_qt = Tbl_086Y;
            break;

        case 7: // '\007'
            std_chrominance_qt = Tbl_100Y;
            break;

        default:
            return;
        }
    else
        switch(dec->advance_selector)
        {
        case 0: // '\0'
            std_chrominance_qt = Tbl_000UV;
            break;

        case 1: // '\001'
            std_chrominance_qt = Tbl_014UV;
            break;

        case 2: // '\002'
            std_chrominance_qt = Tbl_029UV;
            break;

        case 3: // '\003'
            std_chrominance_qt = Tbl_043UV;
            break;

        case 4: // '\004'
            std_chrominance_qt = Tbl_057UV;
            break;

        case 5: // '\005'
            std_chrominance_qt = Tbl_071UV;
            break;

        case 6: // '\006'
            std_chrominance_qt = Tbl_086UV;
            break;

        case 7: // '\007'
            std_chrominance_qt = Tbl_100UV;
            break;

        default:
            return;
        }
    setQuantizationTable(std_chrominance_qt, dec->ADVANCESCALEFACTORUV, abyte0);
    for(int8_t byte0 = 0; byte0 <= 63; byte0++)
        al[byte0] = GET_SHORT(abyte0[zigzag[byte0]]);

    int8_t byte1 = 0;
    for(int8_t byte2 = 0; byte2 <= 7; byte2++)
    {
        for(int8_t byte3 = 0; byte3 <= 7; byte3++)
        {
            int i = (int)((float)al[byte1] * (af[byte2] * af[byte3]));
            al[byte1] = i * 0x10000;
            byte1++;
        }
    }

    dec->byte_pos += 64;
}


static void updateReadBuf(struct ast_decoder *dec, int i)
{
    int32_t uprdbuf_readbuf;
    printf("--> updateReadBuf(%d)\n", i);
    if (dec->m_newbits - i <= 0) {
        uprdbuf_readbuf = GET_LONG(dec->buf[dec->_index]);
        dec->_index++;
        dec->buf[0] = (GET_LONG(dec->buf[0]) << i) | ((GET_LONG(dec->buf[1]) | uprdbuf_readbuf >> dec->m_newbits) >> (32 - i));
        dec->buf[1] = uprdbuf_readbuf << (i - dec->m_newbits);
        dec->m_newbits += 32 - i;
    } else {
        dec->buf[0] = (GET_LONG(dec->buf[0]) << i) | (GET_LONG(dec->buf[1]) >> (32 - i));
        dec->buf[1] = GET_LONG(dec->buf[1]) << i;
        dec->m_newbits -= i;
    }
}

static short lookKbits(struct ast_decoder *dec, uint8_t byte0)
{
    return GET_LONG(dec->buf[0]) >> (32 - byte0);
}

static void skipKbits(struct ast_decoder *dec, uint8_t byte0)
{
    if (dec->m_newbits - byte0 <= 0) {
        if (dec->_index > dec->length - 1)
            dec->_index = dec->length - 1;
        dec->buf[0] = (GET_LONG(dec->buf[0]) << byte0) | ((GET_LONG(dec->buf[1]) | GET_LONG(dec->buf[dec->_index]) >> dec->m_newbits) >> (32 - byte0));
        dec->buf[1] = GET_LONG(dec->buf[dec->_index]) << (byte0 - dec->m_newbits);
        dec->m_newbits += 32 - byte0;
        dec->_index++;
    } else {
        dec->buf[0] = (GET_LONG(dec->buf[0]) << byte0) | (GET_LONG(dec->buf[1]) >> (32 - byte0));
        dec->buf[1] = GET_LONG(dec->buf[1]) << byte0;
        dec->m_newbits -= byte0;
    }
}

static void moveBlockIndex(struct ast_decoder *dec)
{
    dec->txb++;
    printf("--> moveBlockIndex()\n");
    if (dec->txb == dec->tmp_WIDTHBy16) {
        // m_view.repaint((dec->txb - 1) * 16, dec->tyb * 16, 16, 16);
        printf("repaint\n");
    }
    if (dec->m_Mode420 == 0)
    {
        if (dec->txb >= dec->tmp_WIDTHBy16 / 8)
        {
            dec->tyb++;
            if (dec->tyb >= dec->tmp_HEIGHTBy16 / 8)
                dec->tyb = 0;
            dec->txb = 0;
        }
    } else if (dec->txb >= dec->tmp_WIDTHBy16 / 16)
    {
        dec->tyb++;
        if (dec->tyb >= dec->tmp_HEIGHTBy16 / 16)
            dec->tyb = 0;
        dec->txb = 0;
    }
//    SOCIVTPPktHdr.gTxb = dec->txb;
//    SOCIVTPPktHdr.gTyb = dec->tyb;
//    pixels += 256L;
}


static void decompressVQ(struct ast_decoder *dec, int i, int j, char byte0)
{
    int k = 0;
    printf("--> decompressVQ(%d, %d, %d)\n", i, j, byte0);
    if (dec->m_VQ.BitMapBits == 0) {
        for (int l = 0; l < 64; l++) {
            dec->YUVTile[k + 0] = (dec->m_VQ.Color[dec->m_VQ.Index[0]] & 0xff0000L) >> 16;
            dec->YUVTile[k + 64] = (dec->m_VQ.Color[dec->m_VQ.Index[0]] & 65280L) >> 8;
            dec->YUVTile[k + 128] = (dec->m_VQ.Color[dec->m_VQ.Index[0]] & 255L);
            k++;
        }
    } else {
        for (int i1 = 0; i1 < 64; i1++) {
            short word0 = lookKbits(dec, dec->m_VQ.BitMapBits);
            dec->YUVTile[k + 0] = (dec->m_VQ.Color[dec->m_VQ.Index[word0]] & 0xff0000L) >> 16;
            dec->YUVTile[k + 64] = (dec->m_VQ.Color[dec->m_VQ.Index[word0]] & 65280L) >> 8;
            dec->YUVTile[k + 128] = (dec->m_VQ.Color[dec->m_VQ.Index[word0]] & 255L);
            k++;
            skipKbits(dec, dec->m_VQ.BitMapBits);
        }
    }

    convertYUVtoRGB(dec, i, j);
}

G_GNUC_INTERNAL
void stream_aspeed_init(display_stream *st)
{
    st->dec = calloc(sizeof(struct ast_decoder), 1);

    initColorTable(st->dec);
    initRangeLimitTable(st->dec);
    initHuffmanTable(st->dec);
}

G_GNUC_INTERNAL
void stream_aspeed_data(display_stream *st)
{
    int width;
    int height;
    uint8_t *dest;
    size_t len;
    int j;
    struct ASTHeader *hdr;
    struct ast_decoder *dec = st->dec;

    stream_get_dimensions(st, &width, &height);
    dest = g_malloc0(width * height * 4);

    g_free(st->out_frame);
    st->out_frame = dest;

    len = stream_get_current_frame(st, (void *)&dec->buf);

    hdr = (struct ASTHeader *)dec->buf;
    printf("### decode_frame(%zd): %dx%d (%dx%d)\n", len, width, height, hdr->src_mode_x, hdr->src_mode_y);
    dec->buf += 88 >> 2;

    j = hdr->comp_size >> 2;

    dec->m_decodeBuf = (void *)st->out_frame;
    dec->_index = 2;
    dec->m_newbits = 32;
    dec->txb = dec->tyb = 0;
    dec->byte_pos = 0;
    dec->selector = hdr->jpeg_table;
    dec->advance_selector = hdr->adv_table;
    dec->Mapping = hdr->jpeg_yuv;

    dec->SCALEFACTOR = dec->SCALEFACTORUV = 16;
    dec->ADVANCESCALEFACTOR = dec->ADVANCESCALEFACTORUV = 16;

    for (int l = 0; l < 4; l++)
        dec->m_VQ.Index[l] = l;

    dec->m_VQ.Color[0] = 32896L;
    dec->m_VQ.Color[1] = 0xff8080L;
    dec->m_VQ.Color[2] = 0x808080L;
    dec->m_VQ.Color[3] = 0xc08080L;

    dec->WIDTH = hdr->src_mode_x;
    dec->HEIGHT = hdr->src_mode_y;
    dec->RealWIDTH = hdr->src_mode_x;
    dec->RealHEIGHT = hdr->src_mode_y;
    dec->m_Mode420 = hdr->mode420;
    if (dec->m_Mode420 == 1) {
        if (dec->WIDTH % 16 != 0)
            dec->WIDTH = (dec->WIDTH + 16) - dec->WIDTH % 16;
        if (dec->HEIGHT % 16 != 0)
            dec->HEIGHT = (dec->HEIGHT + 16) - dec->HEIGHT % 16;
    } else {
        if (dec->WIDTH % 8 != 0)
            dec->WIDTH = (dec->WIDTH + 8) - dec->WIDTH % 8;
        if (dec->HEIGHT % 8 != 0)
            dec->HEIGHT = (dec->HEIGHT + 8) - dec->HEIGHT % 8;
    }

    dec->tmp_WIDTHBy16 = hdr->dst_mode_x;
    dec->tmp_HEIGHTBy16 = hdr->dst_mode_y;
    if (dec->m_Mode420 == 1) {
        if (dec->tmp_WIDTHBy16 % 16 != 0)
            dec->tmp_WIDTHBy16 = (dec->tmp_WIDTHBy16 + 16) - dec->tmp_WIDTHBy16 % 16;
        if (dec->tmp_HEIGHTBy16 % 16 != 0)
            dec->tmp_HEIGHTBy16 = (dec->tmp_HEIGHTBy16 + 16) - dec->tmp_HEIGHTBy16 % 16;
    } else {
        if (dec->tmp_WIDTHBy16 % 8 != 0)
            dec->tmp_WIDTHBy16 = (dec->tmp_WIDTHBy16 + 8) - dec->tmp_WIDTHBy16 % 8;
        if (dec->tmp_HEIGHTBy16 % 8 != 0)
            dec->tmp_HEIGHTBy16 = (dec->tmp_HEIGHTBy16 + 8) - dec->tmp_HEIGHTBy16 % 8;
    }

    loadLuminanceQuantizationTable(dec, dec->m_QT[0]);
    loadChrominanceQuantizationTable(dec, dec->m_QT[1]);
    loadPass2LuminanceQuantizationTable(dec, dec->m_QT[2]);
    loadPass2ChrominanceQuantizationTable(dec, dec->m_QT[3]);

    int k = 0;
    do {
        switch ((GET_LONG(dec->buf[0]) >> 28) & 15L) {
        case 0:
            updateReadBuf(dec, 4);
            //decompressJPEG(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 8:
            dec->txb = (GET_LONG(dec->buf[0]) & 0xff00000L) >> 20;
            dec->tyb = (GET_LONG(dec->buf[0]) & 0xff000L) >> 12;
            updateReadBuf(dec, 20);
            // decompressJPEG(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 2:
            updateReadBuf(dec, 4);
            // decompressJPEGPass2(dec, dec->txb, dec->tyb, 2);
            moveBlockIndex(dec);
            break;
        case 10:
            dec->txb = (GET_LONG(dec->buf[0]) & 0xff00000L) >> 20;
            dec->tyb = (GET_LONG(dec->buf[0]) & 0xff000L) >> 12;
            updateReadBuf(dec, 20);
            // decompressJPEGPass2(dec, dec->txb, dec->tyb, 2);
            moveBlockIndex(dec);
            break;
        case 5:
            updateReadBuf(dec, 4);
            dec->m_VQ.BitMapBits = 0;
            for (int i1 = 0; i1 < 1; i1++) {
                dec->m_VQ.Index[i1] = (GET_LONG(dec->buf[0]) >> 29) & 3L;
                if (((GET_LONG(dec->buf[0]) >> 31) & 1L) == 0) {
                    updateReadBuf(dec, 3);
                } else {
                    dec->m_VQ.Color[dec->m_VQ.Index[i1]] = (GET_LONG(dec->buf[0]) >> 5) & 0xffffff;
                    updateReadBuf(dec, 27);
                }

                decompressVQ(dec, dec->txb, dec->tyb, 0);
                moveBlockIndex(dec);
            }
            break;
        case 13:
            dec->txb = (GET_LONG(dec->buf[0]) & 0xff00000L) >> 20;
            dec->tyb = (GET_LONG(dec->buf[0]) & 0xff000L) >> 12;
            updateReadBuf(dec, 20);
            dec->m_VQ.BitMapBits = 0;
            for(int j1 = 0; j1 < 1; j1++)
            {
                dec->m_VQ.Index[j1] = (GET_LONG(dec->buf[0]) >> 29) & 3L;
                if(((GET_LONG(dec->buf[0]) >> 31) & 1L) == 0L)
                {
                    updateReadBuf(dec, 3);
                } else
                {
                    dec->m_VQ.Color[dec->m_VQ.Index[j1]] = (GET_LONG(dec->buf[0]) >> 5) & 0xffffffL;
                    updateReadBuf(dec, 27);
                }
            }

            decompressVQ(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 6:
            updateReadBuf(dec, 4);
            dec->m_VQ.BitMapBits = 1;
            for(int k1 = 0; k1 < 2; k1++)
            {
                dec->m_VQ.Index[k1] = (GET_LONG(dec->buf[0]) >> 29) & 3L;
                if(((GET_LONG(dec->buf[0]) >> 31) & 1L) == 0L)
                {
                    updateReadBuf(dec, 3);
                } else
                {
                    dec->m_VQ.Color[dec->m_VQ.Index[k1]] = (GET_LONG(dec->buf[0]) >> 5) & 0xffffffL;
                    updateReadBuf(dec, 27);
                }
            }
            decompressVQ(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 14:
            dec->txb = (GET_LONG(dec->buf[0]) & 0xff00000L) >> 20;
            dec->tyb = (GET_LONG(dec->buf[0]) & 0xff000L) >> 12;
            updateReadBuf(dec, 20);
            dec->m_VQ.BitMapBits = 1;
            for(int l1 = 0; l1 < 2; l1++)
            {
                dec->m_VQ.Index[l1] = (GET_LONG(dec->buf[0]) >> 29) & 3L;
                if(((GET_LONG(dec->buf[0]) >> 31) & 1L) == 0L)
                {
                    updateReadBuf(dec, 3);
                } else
                {
                    dec->m_VQ.Color[dec->m_VQ.Index[l1]] = (GET_LONG(dec->buf[0]) >> 5) & 0xffffffL;
                    updateReadBuf(dec, 27);
                }
            }

            decompressVQ(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 7:
            updateReadBuf(dec, 4);
            dec->m_VQ.BitMapBits = 2;
            for(int i2 = 0; i2 < 4; i2++)
            {
                dec->m_VQ.Index[i2] = (GET_LONG(dec->buf[0]) >> 29) & 3L;
                if(((GET_LONG(dec->buf[0]) >> 31) & 1L) == 0L)
                {
                    updateReadBuf(dec, 3);
                } else
                {
                    dec->m_VQ.Color[dec->m_VQ.Index[i2]] = (GET_LONG(dec->buf[0]) >> 5) & 0xffffffL;
                    updateReadBuf(dec, 27);
                }
            }

            decompressVQ(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 15:
            dec->txb = (GET_LONG(dec->buf[0]) & 0xff00000L) >> 20;
            dec->tyb = (GET_LONG(dec->buf[0]) & 0xff000L) >> 12;
            updateReadBuf(dec, 20);
            dec->m_VQ.BitMapBits = 2;
            for(int j2 = 0; j2 < 4; j2++)
            {
                dec->m_VQ.Index[j2] = (GET_LONG(dec->buf[0]) >> 29) & 3L;
                if(((GET_LONG(dec->buf[0]) >> 31) & 1L) == 0L)
                {
                    updateReadBuf(dec, 3);
                } else
                {
                    dec->m_VQ.Color[dec->m_VQ.Index[j2]] = (GET_LONG(dec->buf[0]) >> 5) & 0xffffffL;
                    updateReadBuf(dec, 27);
                }
            }

            decompressVQ(dec, dec->txb, dec->tyb, 0);
            moveBlockIndex(dec);
            break;
        case 4:
            updateReadBuf(dec, 4);
            // decompressJPEG(dec, dec->txb, dec->tyb, 2);
            moveBlockIndex(dec);
            break;
        case 12:
            dec->txb = (GET_LONG(dec->buf[0]) & 0xff00000L) >> 20;
            dec->tyb = (GET_LONG(dec->buf[0]) & 0xff000L) >> 12;
            updateReadBuf(dec, 20);
            // decompressJPEG(dec, dec->txb, dec->tyb, 2);
            moveBlockIndex(dec);
            break;
        case 9:
            if(((GET_LONG(dec->buf[0]) >> 28) & 15L) == 9L)
                goto done;
            fprintf(stderr, "Unknow Marco Block type %08x\n", GET_LONG(dec->buf[0]) >> 28);
            moveBlockIndex(dec);
            break;
        }
        // m_view.repaint((txb - 1) * 16, tyb * 16, 16, 16);
        k++;
    } while (dec->_index < j);

done:
    return;
}

G_GNUC_INTERNAL
void stream_aspeed_cleanup(display_stream *st)
{
    jpeg_destroy_decompress(&st->mjpeg_cinfo);
    g_free(st->out_frame);
    st->out_frame = NULL;
}
