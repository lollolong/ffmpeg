/*
 * DTS-UHD common audio frame parsing code
 * Copyright (c) 2023 Xperi Corporation / DTS, Inc.
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Parse DTS-UHD audio frame headers, report frame sizes and configuration.
 * Specification: ETSI TS 103 491 V1.2.1
 */

#include <string.h>

#include "dtsuhd_common.h"
#include "get_bits.h"
#include "libavutil/mem.h"
#include "libavutil/channel_layout.h"

#define DTSUHD_ALLOC_INCREMENT 16
#define DTSUHD_CHUNK_HEADER    16

enum RepType {
    REP_TYPE_CH_MASK_BASED,
    REP_TYPE_MTRX2D_CH_MASK_BASED,
    REP_TYPE_MTRX3D_CH_MASK_BASED,
    REP_TYPE_BINAURAL,
    REP_TYPE_AMBISONIC,
    REP_TYPE_AUDIO_TRACKS,
    REP_TYPE_3D_OBJECT_SINGLE_SRC_PER_WF,
    REP_TYPE_3D_MONO_OBJECT_SINGLE_SRC_PER_WF,
};

typedef struct MDObject {
    int started;  /* Object seen since last reset. */
    int pres_index;
    int rep_type;
    int ch_activity_mask;
} MDObject;

typedef struct MD01 {
    GetBitContext gb;
    MDObject object[257]; /* object id max value is 256 */
    int chunk_id;
    int object_list[256]; int object_list_count;
    int packets_acquired;
    int static_md_extracted;
    int static_md_packets;
    int static_md_packet_size;
    int static_md_update_flag;
    uint8_t *buf; int buf_bytes; /* temporary buffer to accumulate static data */
} MD01;

typedef struct NAVI {
    int bytes;
    int id;
    int index;
    int present;
} NAVI;

typedef struct UHDAudio {
    int mask;
    int selectable;
} UHDAudio;

typedef struct UHDChunk {
    int crc_flag;
    int bytes;
} UHDChunk;

struct DTSUHD {
    const uint8_t *data; int data_bytes;  /* Original audio frame buffer. */
    GetBitContext gb;
    MD01 *md01; int md01_count;
    NAVI *navi; int navi_alloc, navi_count;
    UHDAudio audio[256];
    UHDChunk *chunk; int chunk_alloc, chunk_count;
    int chunk_bytes;
    int clock_rate;
    int frame_bytes;
    int frame_duration;
    int frame_duration_code;
    int ftoc_bytes;
    int major_version;
    int num_audio_pres;
    int sample_rate;
    int sample_rate_mod;
    unsigned full_channel_mix_flag:1;
    unsigned interactive_obj_limits_present:1;
    unsigned is_sync_frame:1;
    unsigned saw_sync:1;
};

/* Read from the MD01 buffer (if present), falling back to the frame buffer */
static inline int get_bits_md01(DTSUHD *h, MD01 *md01, int bits)
{
    if (md01->buf)
        return get_bits(&md01->gb, bits);
    return get_bits(&h->gb, bits);
}

/* In the specification, the pseudo code defaults the 'add' parameter to true.
   Table 7-30 shows passing an explicit false, most other calls do not
   pass the extractAndAdd parameter.

   Function based on code in Table 5-2
*/
static int get_bits_var(GetBitContext *gb, const uint8_t table[], int add)
{
    static const int bits_used[8] = { 1, 1, 1, 1, 2, 2, 3, 3 };
    static const int index_table[8] = { 0, 0, 0, 0, 1, 1, 2, 3 };
    int code = show_bits(gb, 3); /* value range is [0, 7] */
    int i;
    int index = index_table[code];
    int value = 0;

    skip_bits(gb, bits_used[code]);
    if (table[index] > 0) {
        if (add) {
            for (i = 0; i < index; i++)
                value += 1 << table[i];
        }
        value += get_bits_long(gb, table[index]);
    }

    return value;
}

/* Implied by Table 6-2, MD01 chunk objects appended in for loop */
static MD01 *chunk_append_md01(DTSUHD *h, int id)
{
    int md01_alloc = h->md01_count + 1;
    if (av_reallocp_array(&h->md01, md01_alloc, sizeof(*h->md01)))
        return NULL;

    memset(h->md01 + h->md01_count, 0, sizeof(*h->md01));
    h->md01[h->md01_count].chunk_id = id;
    return h->md01 + h->md01_count++;
}

/* Return existing MD01 chunk based on chunkID */
static MD01 *chunk_find_md01(DTSUHD *h, int id)
{
    int i;

    for (i = 0; i < h->md01_count; i++)
        if (id == h->md01[i].chunk_id)
            return h->md01 + i;

    return NULL;
}

/* Table 6-3 */
static void chunk_reset(DTSUHD *h)
{
    int i;

    for (i = 0; i < h->md01_count; i++)
        av_freep(&h->md01[i].buf);
    av_freep(&h->md01);
    h->md01_count = 0;
}

static MDObject *find_default_audio(DTSUHD *h)
{
    MDObject *object;
    int i, j;
    int obj_index = -1;

    for (i = 0; i < h->md01_count; i++) {
        for (j = 0; j < 257; j++) {
            object = h->md01[i].object + j;
            if (object->started && h->audio[object->pres_index].selectable) {
                if (obj_index < 0 || (object->pres_index < h->md01[i].object[obj_index].pres_index))
                    obj_index = j;
            }
        }
        if (obj_index >= 0)
            return h->md01[i].object + obj_index;
    }

    return NULL;
}

/* Save channel mask, count, and rep type to descriptor info.
   ETSI TS 103 491 Table 7-28 channel activity mask bits
   mapping and SCTE DVS 243-4 Rev. 0.2 DG X Table 4.  Convert activity mask and
   representation type to channel mask and channel counts.
*/
static void extract_object_info(MDObject *object, DTSUHDDescriptorInfo *info)
{
    int i;
    static const struct {
        uint32_t activity_mask;
        uint32_t channel_mask; // Mask as defined by ETSI TS 103 491
        uint64_t ffmpeg_channel_mask; // Mask as defined in ffmpeg
    } activity_map[] = {
        // act mask | chan mask | ffmpeg channel mask
        { 0x000001, 0x00000001, AV_CH_FRONT_CENTER },
        { 0x000002, 0x00000006, AV_CH_FRONT_LEFT | AV_CH_FRONT_RIGHT },
        { 0x000004, 0x00000018, AV_CH_SIDE_LEFT | AV_CH_SIDE_RIGHT },
        { 0x000008, 0x00000020, AV_CH_LOW_FREQUENCY },
        { 0x000010, 0x00000040, AV_CH_BACK_CENTER },
        { 0x000020, 0x0000A000, AV_CH_TOP_FRONT_LEFT | AV_CH_TOP_FRONT_RIGHT },
        { 0x000040, 0x00000180, AV_CH_BACK_LEFT | AV_CH_BACK_RIGHT },
        { 0x000080, 0x00004000, AV_CH_TOP_FRONT_CENTER },
        { 0x000100, 0x00080000, AV_CH_TOP_CENTER },
        { 0x000200, 0x00001800, AV_CH_FRONT_LEFT_OF_CENTER | AV_CH_FRONT_RIGHT_OF_CENTER },
        { 0x000400, 0x00060000, AV_CHAN_WIDE_LEFT | AV_CHAN_WIDE_RIGHT },
        { 0x000800, 0x00000600, AV_CH_SURROUND_DIRECT_LEFT | AV_CH_SURROUND_DIRECT_RIGHT },
        { 0x001000, 0x00010000, AV_CH_LOW_FREQUENCY_2 },
        { 0x002000, 0x00300000, AV_CH_TOP_SIDE_LEFT | AV_CH_TOP_SIDE_RIGHT },
        { 0x004000, 0x00400000, AV_CH_TOP_BACK_CENTER },
        { 0x008000, 0x01800000, AV_CH_TOP_BACK_LEFT | AV_CH_TOP_BACK_RIGHT },
        { 0x010000, 0x02000000, AV_CH_BOTTOM_FRONT_CENTER },
        { 0x020000, 0x0C000000, AV_CH_BOTTOM_FRONT_LEFT | AV_CH_BOTTOM_FRONT_RIGHT },
        { 0x140000, 0x30000000, AV_CH_TOP_FRONT_LEFT | AV_CH_TOP_FRONT_RIGHT },
        { 0x080000, 0xC0000000, AV_CH_TOP_BACK_LEFT | AV_CH_TOP_BACK_RIGHT },
        { 0 } // Terminator
    };

    if (object) {
        for (i = 0; activity_map[i].activity_mask; i++) {
            if (activity_map[i].activity_mask & object->ch_activity_mask) {
                info->channel_mask |= activity_map[i].channel_mask;
                info->ffmpeg_channel_mask |= activity_map[i].ffmpeg_channel_mask;
            }
        }
        info->channel_count = av_popcount(info->channel_mask);
        info->rep_type = object->rep_type;
    }
}

/* Assemble information for MP4 Sample Entry box.  Sample Size is always
   16 bits.  The coding name is the name of the SampleEntry sub-box and is
   'dtsx' unless the version of the bitstream is > 2.
   If DecoderProfile == 2, then MaxPayloadCode will be zero.
*/
static void update_descriptor(DTSUHD *h, DTSUHDDescriptorInfo *info)
{
    static const char *coding_name[] = { "dtsx", "dtsy" };

    memset(info, 0, sizeof(*info));
    memcpy(info->coding_name, coding_name[h->major_version > 2], 5);
    extract_object_info(find_default_audio(h), info);
    info->base_sample_freq_code = h->sample_rate == 48000;
    info->decoder_profile_code = h->major_version - 2;
    info->frame_duration_code = h->frame_duration_code;
    info->max_payload_code = 0 + (h->major_version > 2);
    info->num_pres_code = h->num_audio_pres - 1;
    info->sample_rate = h->sample_rate;
    info->sample_rate_mod = h->sample_rate_mod;
    info->sample_size = 16;
    info->valid = 1;
}

/* Table 6-17 p47 */
static int parse_explicit_object_lists(DTSUHD *h, int mask, int index)
{
    GetBitContext *gb = &h->gb;
    int i;
    static const uint8_t table[4] = { 4, 8, 16, 32 };

    for (i = 0; i < index; i++) {
        if ((mask >> i) & 0x01) {
            if (h->is_sync_frame || get_bits1(gb))
                get_bits_var(gb, table, 1);
        }
    }

    return 0;
}

/* Table 6-15 p44, Table 6-16 p45 */
static int parse_aud_pres_params(DTSUHD *h)
{
    GetBitContext *gb = &h->gb;
    int audio;
    int i;
    int read_mask;
    static const uint8_t table[4] = { 0, 2, 4, 5 };

    if (h->is_sync_frame) {
        if (h->full_channel_mix_flag)
            h->num_audio_pres = 1;
        else
            h->num_audio_pres = get_bits_var(gb, table, 1) + 1;
        memset(h->audio, 0, sizeof(h->audio[0]) * h->num_audio_pres);
    }

    for (audio = 0; audio < h->num_audio_pres; audio++) {
        if (h->is_sync_frame) {
            if (h->full_channel_mix_flag)
                h->audio[audio].selectable = 1;
            else
                h->audio[audio].selectable = get_bits1(gb);
        }

        if (h->audio[audio].selectable) {
            if (h->is_sync_frame) {
                read_mask = (audio > 0) ? get_bits(gb, audio) : 0;
                h->audio[audio].mask = 0;
                for (i = 0; read_mask; i++, read_mask >>= 1) {
                    if (read_mask & 0x01)
                        h->audio[audio].mask |= get_bits1(gb) << i;
                }
            }

            if (parse_explicit_object_lists(h, h->audio[audio].mask, audio))
                return 1;
        } else {
            h->audio[audio].mask = 0;
        }
    }

    return 0;
}

/* Table 6-9 p 38 */
static int check_crc(DTSUHD *h, int bit, int bytes)
{
    GetBitContext gb;
    int i;
    static const uint16_t lookup[16] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
    };
    uint16_t crc = 0xFFFF;

    init_get_bits(&gb, h->data, h->data_bytes * 8);
    skip_bits(&gb, bit);
    for (i = -bytes; i < bytes; i++)
        crc = (crc << 4) ^ lookup[(crc >> 12) ^ get_bits(&gb, 4)];

    return crc != 0;
}

/* Table 6-12 p 40 */
static void decode_version(DTSUHD *h)
{
    GetBitContext *gb = &h->gb;
    int bits = get_bits1(gb) ? 3 : 6;

    h->major_version = get_bits(gb, bits) + 2;
    skip_bits(gb, bits);
}

/* Table 6-12 p 40 */
static int parse_stream_params(DTSUHD *h)
{
    GetBitContext *gb = &h->gb;
    int has_ftoc_crc;
    static const uint32_t table_base_duration[4] = { 512, 480, 384, 0 };
    static const uint32_t table_clock_rate[4] = { 32000, 44100, 48000, 0 };

    if (h->is_sync_frame)
        h->full_channel_mix_flag = get_bits1(gb);

    has_ftoc_crc = !h->full_channel_mix_flag || h->is_sync_frame;
    if (has_ftoc_crc && check_crc(h, 0, h->ftoc_bytes))
        return 1;

    if (h->is_sync_frame) {
        if (h->full_channel_mix_flag)
            h->major_version = 2;
        else
            decode_version(h);

        h->frame_duration = table_base_duration[get_bits(gb, 2)];
        h->frame_duration_code = get_bits(gb, 3);
        h->frame_duration *= (h->frame_duration_code + 1);
        h->clock_rate = table_clock_rate[get_bits(gb, 2)];
        if (h->frame_duration == 0 || h->clock_rate == 0)
            return 1; /* bitstream error */

        skip_bits(gb, 36 * get_bits1(gb));  /* bTimeStampPresent */
        h->sample_rate_mod = get_bits(gb, 2);
        h->sample_rate = h->clock_rate * (1 << h->sample_rate_mod);

        if (h->full_channel_mix_flag) {
            h->interactive_obj_limits_present = 0;
        } else {
            skip_bits1(gb);  /* reserved flag. */
            h->interactive_obj_limits_present = get_bits1(gb);
        }
    }

    return 0;
}

/* Table 6-24 p52 */
static void navi_purge(DTSUHD *h)
{
    int i;

    for (i = 0; i < h->navi_count; i++)
        if (!h->navi[i].present)
            h->navi[i].bytes = 0;
}

/* Table 6-21 p50 */
static void navi_clear(DTSUHD *h)
{
    if (h->navi)
        memset(h->navi, 0, sizeof(h->navi[0]) * h->navi_count);
    h->navi_count = 0;
}

/* Table 6-22 p51 */
static void navi_clear_present(DTSUHD *h)
{
    int i;

    for (i = 0; i < h->navi_count; i++)
        h->navi[i].present = 0;
}

/* Table 6-23 p51.  Return 0 on success, and the index is returned in
   the *listIndex parameter.
*/
static int navi_find_index(DTSUHD *h, int desired_index, int *list_index)
{
    int avail_index = h->navi_count;
    int i;
    int navi_alloc;

    for (i = 0; i < h->navi_count; i++) {
        if (h->navi[i].index == desired_index) {
            *list_index = i;
            h->navi[i].present = 1;
            return 0;
        }

        if ((h->navi[i].present == 0) && (h->navi[i].bytes == 0) && (avail_index > i))
            avail_index = i;
    }

    if (avail_index >= h->navi_count) {
        if (h->navi_count >= h->navi_alloc) {
            navi_alloc = h->navi_count + DTSUHD_ALLOC_INCREMENT;
            if (av_reallocp_array(&h->navi, navi_alloc, sizeof(*h->navi)))
                return 1;
            h->navi_alloc = navi_alloc;
        }
        h->navi_count++;
    }

    *list_index = avail_index;
    h->navi[avail_index].bytes = 0;
    h->navi[avail_index].present = 1;
    h->navi[avail_index].id = 256;
    h->navi[avail_index].index = desired_index;

    return 0;
}

/* Table 6-20 p48 */
static int parse_chunk_navi(DTSUHD *h)
{
    GetBitContext *gb = &h->gb;
    int audio_chunks = 1;
    int bytes;
    int i;
    int id;
    int id_present;
    int index;
    int list_index;
    static const uint8_t table2468[4] = { 2, 4, 6, 8 };
    static const uint8_t table_audio_chunk_sizes[4] = { 9, 11, 13, 16 };
    static const uint8_t table_chunk_sizes[4] = { 6, 9, 12, 15 };

    h->chunk_bytes = 0;
    if (h->full_channel_mix_flag)
        h->chunk_count = h->is_sync_frame;
    else
        h->chunk_count = get_bits_var(gb, table2468, 1);

    if (h->chunk_count >= h->chunk_alloc) {
        int chunk_alloc = h->chunk_count + DTSUHD_ALLOC_INCREMENT;
        if (av_reallocp_array(&h->chunk, chunk_alloc, sizeof(*h->chunk)))
            return 1;
        h->chunk_alloc = chunk_alloc;
    }

    for (i = 0; i < h->chunk_count; i++) {
        h->chunk_bytes += h->chunk[i].bytes = get_bits_var(gb, table_chunk_sizes, 1);
        if (h->full_channel_mix_flag)
            h->chunk[i].crc_flag = 0;
        else
        h->chunk[i].crc_flag = get_bits1(gb);
    }

    if (!h->full_channel_mix_flag)
        audio_chunks = get_bits_var(gb, table2468, 1);

    if (h->is_sync_frame)
        navi_clear(h);
    else
        navi_clear_present(h);

    for (i = 0; i < audio_chunks; i++) {
        if (h->full_channel_mix_flag)
            index = 0;
        else
            index = get_bits_var(gb, table2468, 1);

        if (navi_find_index(h, index, &list_index))
            return 1;

        if (h->is_sync_frame)
            id_present = 1;
        else if (h->full_channel_mix_flag)
            id_present = 0;
        else
            id_present = get_bits1(gb);

        if (id_present) {
            id = get_bits_var(gb, table2468, 1);
            h->navi[list_index].id = id;
        }

        bytes = get_bits_var(gb, table_audio_chunk_sizes, 1);
        h->chunk_bytes += bytes;
        h->navi[list_index].bytes = bytes;
    }

    navi_purge(h);

    return 0;
}


/* Table 6-6 */
static int parse_md_chunk_list(DTSUHD *h, MD01 *md01)
{
    GetBitContext *gb = &h->gb;
    const uint8_t table1[4] = { 3, 4, 6, 8 };
    int i;

    if (h->full_channel_mix_flag) {
        md01->object_list_count = 1;
        md01->object_list[0] = 256;
    } else {
        md01->object_list_count = get_bits_var(gb, table1, 1);
        for (i = 0; i < md01->object_list_count; i++)
            md01->object_list[i] = get_bits(gb, get_bits1(gb) ? 8 : 4);
    }

    return 0;
}

/* Table 7-9 */
static void skip_mp_param_set(DTSUHD *h, MD01 *md01, int nominal_flag)
{
    get_bits_md01(h, md01, 6); /* rLoudness */
    if (nominal_flag == 0)
        get_bits_md01(h, md01, 5);

    get_bits_md01(h, md01, nominal_flag ? 2 : 4);
}

/* Table 7-8 */
static int parse_static_md_params(DTSUHD *h, MD01 *md01, int only_first)
{
    int i;
    int loudness_sets = 1;
    int nominal_flag = 1;

    if (h->full_channel_mix_flag == 0)
        nominal_flag = get_bits_md01(h, md01, 1);

    if (nominal_flag) {
        if (h->full_channel_mix_flag == 0)
            loudness_sets = get_bits_md01(h, md01, 1) ? 3 : 1;
    } else {
        loudness_sets = get_bits_md01(h, md01, 4) + 1;
    }

    for (i = 0; i < loudness_sets; i++)
        skip_mp_param_set(h, md01, nominal_flag);

    if (only_first)
        return 0;

    if (nominal_flag == 0)
        get_bits_md01(h, md01, 1);

    for (i = 0; i < 3; i++) { /* Table 7-12 suggest 3 types */
        if (get_bits_md01(h, md01, 1)) {
            if (get_bits_md01(h, md01, 4) == 15) /* Table 7-14 */
                get_bits_md01(h, md01, 15);
        }
        if (get_bits_md01(h, md01, 1)) /* smooth md present */
            get_bits_md01(h, md01, 6 * 6);
    }

    if (h->full_channel_mix_flag == 0) {
        i = md01->static_md_packets * md01->static_md_packet_size - get_bits_count(&md01->gb);
        skip_bits(&md01->gb, i);
    }
    md01->static_md_extracted = 1;

    return 0;
}

/* Table 7-7 */
static int parse_multi_frame_md(DTSUHD *h, MD01 *md01)
{
    GetBitContext *gb = &h->gb;
    int i, n;
    static const uint8_t table1[4] = { 0, 6, 9, 12 };
    static const uint8_t table2[4] = { 5, 7, 9, 11 };

    if (h->is_sync_frame) {
        md01->packets_acquired = 0;
        if (h->full_channel_mix_flag) {
            md01->static_md_packets = 1;
            md01->static_md_packet_size = 0;
        } else {
            md01->static_md_packets = get_bits_var(gb, table1, 1) + 1;
            md01->static_md_packet_size = get_bits_var(gb, table2, 1) + 3;
        }

        n = md01->static_md_packets * md01->static_md_packet_size;
        if (n > md01->buf_bytes) {
            if (av_reallocp(&md01->buf, n))
                return 1;
            md01->buf_bytes = n;
        }

        init_get_bits(&md01->gb, md01->buf, md01->buf_bytes * 8);
        if (md01->static_md_packets > 1)
            md01->static_md_update_flag = get_bits1(gb);
        else
            md01->static_md_update_flag = 1;
    }

    if (md01->packets_acquired < md01->static_md_packets) {
        n = md01->packets_acquired * md01->static_md_packet_size;
        for (i = 0; i < md01->static_md_packet_size; i++)
            md01->buf[n + i] = get_bits(gb, 8);
        md01->packets_acquired++;

        if (md01->packets_acquired == md01->static_md_packets) {
            if (md01->static_md_update_flag || !md01->static_md_extracted)
                if (parse_static_md_params(h, md01, 0))
                    return 1;
        } else if (md01->packets_acquired == 1) {
            if (md01->static_md_update_flag || !md01->static_md_extracted)
                if (parse_static_md_params(h, md01, 1))
                    return 1;
        }
    }

    return 0;
}

/* Return 1 if suitable, 0 if not.  Table 7-18.  OBJGROUPIDSTART=224 Sec 7.8.7 p75 */
static int is_suitable_for_render(DTSUHD *h, MD01 *md01, int object_id)
{
    GetBitContext *gb = &h->gb;
    static const uint8_t table[4] = { 8, 10, 12, 14 };

    if (object_id >= 224 || get_bits1(gb))
        return 1;

    /*  Reject the render and skip the render data. */
    skip_bits1(gb);
    skip_bits(gb, get_bits_var(gb, table, 1));

    return 0;
}

/* Table 7-26 */
static void parse_ch_mask_params(DTSUHD *h, MD01 *md01, MDObject *object)
{
    GetBitContext *gb = &h->gb;
    const int ch_index = object->rep_type == REP_TYPE_BINAURAL ? 1 : get_bits(gb, 4);
    static const int mask_table[14] = { /* Table 7-27 */
        0x000001, 0x000002, 0x000006, 0x00000F, 0x00001F, 0x00084B, 0x00002F,
        0x00802F, 0x00486B, 0x00886B, 0x03FBFB, 0x000003, 0x000007, 0x000843,
    };

    if (ch_index == 14)
        object->ch_activity_mask = get_bits(gb, 16);
    else if (ch_index == 15)
        object->ch_activity_mask = get_bits(gb, 32);
    else
        object->ch_activity_mask = mask_table[ch_index];
}

/* Table 7-22 */
static int parse_object_metadata(DTSUHD *h, MD01 *md01, MDObject *object,
                                 int start_frame_flag, int object_id)
{
    GetBitContext *gb = &h->gb;
    int ch_mask_object_flag = 0;
    int object_3d_metadata_flag = 0;
    static const uint8_t table2[4] = { 1, 4, 4, 8 };
    static const uint8_t table3[4] = { 3, 3, 4, 8 };

    skip_bits(gb, object_id != 256);

    if (start_frame_flag) {
        object->rep_type = get_bits(gb, 3);
        switch (object->rep_type) {
            case REP_TYPE_BINAURAL:
            case REP_TYPE_CH_MASK_BASED:
            case REP_TYPE_MTRX2D_CH_MASK_BASED:
            case REP_TYPE_MTRX3D_CH_MASK_BASED:
                ch_mask_object_flag = 1;
                break;

            case REP_TYPE_3D_OBJECT_SINGLE_SRC_PER_WF:
            case REP_TYPE_3D_MONO_OBJECT_SINGLE_SRC_PER_WF:
                object_3d_metadata_flag = 1;
                break;
        }

        if (ch_mask_object_flag) {
            if (object_id != 256) {
                skip_bits(gb, 3);  /* Object Importance Level */
                if (get_bits1(gb))
                    skip_bits(gb, get_bits1(gb) ? 3 : 5);

                get_bits_var(gb, table2, 1);
                get_bits_var(gb, table3, 1);

                /* Skip optional Loudness block. */
                if (get_bits1(gb))
                    skip_bits(gb, 8);

                /* Skip optional Object Interactive MD (Table 7-25). */
                if (get_bits1(gb) && h->interactive_obj_limits_present) {
                    if (get_bits1(gb))
                        skip_bits(gb, 5 + 6 * object_3d_metadata_flag);
                }
            }

            parse_ch_mask_params(h, md01, object);
        }
    }

    /* Skip rest of object */
    return 0;
}

/* Table 7-4 */
static int parse_md01(DTSUHD *h, MD01 *md01, int pres_index)
{
    GetBitContext *gb = &h->gb;
    uint32_t i;
    uint32_t id;
    uint32_t start_flag;

    if (h->audio[pres_index].selectable) {
        for (i = 0; i < 4; i++)  /* Table 7-5.  Scaling data. */
            skip_bits(gb, 5 * get_bits1(gb));

        if (get_bits1(gb) && parse_multi_frame_md(h, md01))
            return 1;
    }

    /* Table 7-16: Object metadata. */
    memset(md01->object, 0, sizeof(md01->object));
    if (!h->full_channel_mix_flag)
        skip_bits(gb, 11 * get_bits1(gb));

    for (i = 0; i < md01->object_list_count; i++) {
        id = md01->object_list[i];
        if (!is_suitable_for_render(h, md01, id))
            continue;

        md01->object[id].pres_index = pres_index;
        start_flag = 0;
        if (!md01->object[id].started) {
            skip_bits(gb, id != 256);
            start_flag = md01->object[id].started = 1;
        }

        if ((id < 224 || id > 255) &&
            parse_object_metadata(h, md01, md01->object + id, start_flag, id)) {
            return 1;
        }

        break;
    }

    return 0;
}

/* Table 6-2 */
static int parse_chunks(DTSUHD *h)
{
    GetBitContext *gb = &h->gb;
    MD01 *md01;
    int bit_next;
    int i;
    static const uint8_t table_aud_pres[4] = { 0, 2, 4, 4 };
    int pres_index;
    uint32_t id;

    for (i = 0; i < h->chunk_count; i++) {
        bit_next = get_bits_count(gb) + h->chunk[i].bytes * 8;
        if (h->chunk[i].crc_flag && check_crc(h, get_bits_count(gb), h->chunk[i].bytes))
            return 1;

        id = get_bits(gb, 8);
        if (id == 1) {
            pres_index = get_bits_var(gb, table_aud_pres, 1);
        if (pres_index > 255)
            return 1;
        md01 = chunk_find_md01(h, id);
        if (md01 == NULL)
            md01 = chunk_append_md01(h, id);
        if (md01 == NULL)
            return 1;
        if (parse_md_chunk_list(h, md01))
            return 1;
        if (parse_md01(h, md01, pres_index))
            return 1;
        }

        skip_bits(gb, bit_next - get_bits_count(gb));
    }

    return 0;
}

/** Allocate parsing handle.  The parsing handle should be used to parse
    one DTS:X Profile 2 Audio stream, then freed by calling DTSUHD_destroy().
    Do not use the same parsing handle to parse multiple audio streams.

  @return Parsing handle for use with other functions, or NULL on failure.
*/
DTSUHD *dtsuhd_create(void)
{
    return av_calloc(1, sizeof(DTSUHD));
}

/** Free all resources used by the parsing handle.

  @param[in] h Handle allocated by dtshd_create
*/
void dtsuhd_destroy(DTSUHD *h)
{
    if (h) {
        chunk_reset(h);
        av_freep(&h->chunk);
        av_freep(&h->navi);
        av_freep(&h);
    }
}

/** Parse a single DTS:X Profile 2 frame.
    The frame must start at the first byte of the data buffer, and enough
    of the frame must be present to decode the majority of the FTOC.
    From Table 6-11 p40.

    A sync frame must be the first frame provided, before any non-sync frames.
    Signatures: sync=0x40411BF2, non-sync=0x71C442E8.

  @param[in] h Handle allocated by DTSUHD_create
  @param[in] First byte of a buffer containing the frame to parse
  @param[in] nData Number of valid bytes in 'data'
  @param[out] fi Results of frame parsing, may be NULL
  @param[out] di Results of descriptor parsing, may be NULL
  @return 0 on success, DTSUHDStatus enumeration on error
*/
int dtsuhd_frame(DTSUHD *h, const uint8_t *data, size_t data_bytes,
                 DTSUHDFrameInfo *fi, DTSUHDDescriptorInfo *di)
{
    GetBitContext *gb;
    int fraction = 1;
    int i;
    int syncword;
    static const uint8_t table_payload[4] = { 5, 8, 10, 12 };

    if (!h || !data)
        return DTSUHD_NULL;

    if (data_bytes < 4)
        return DTSUHD_INCOMPLETE; /* Data buffer does not contain the signature */

    h->data = data;
    h->data_bytes = data_bytes;
    gb = &h->gb;
    init_get_bits(gb, data, data_bytes * 8);

    syncword = get_bits_long(gb, 32);
    h->is_sync_frame = syncword == DTSUHD_SYNCWORD;
    h->saw_sync |= h->is_sync_frame;
    if (!h->saw_sync || (!h->is_sync_frame && syncword != DTSUHD_NONSYNCWORD))
        return DTSUHD_NOSYNC;  /* Invalid frame or have not parsed sync frame. */

    h->ftoc_bytes = get_bits_var(gb, table_payload, 1) + 1;
    if (h->ftoc_bytes < 5 || h->ftoc_bytes >= data_bytes)
        return DTSUHD_INCOMPLETE;  /* Data buffer does not contain entire FTOC */

    if (parse_stream_params(h))
        return DTSUHD_INVALID_FRAME;

    if (parse_aud_pres_params(h))
        return DTSUHD_INVALID_FRAME;

    if (parse_chunk_navi(h))  /* AudioChunkTypes and payload sizes. */
        return DTSUHD_INVALID_FRAME;

    /* At this point in the parsing, we can calculate the size of the frame. */
    h->frame_bytes = h->ftoc_bytes + h->chunk_bytes;
    if (h->frame_bytes > data_bytes)
        return DTSUHD_INCOMPLETE;

    if (di && h->is_sync_frame) {
        /* Skip PBRSmoothParams (Table 6-26) and align to the chunks immediately
           following the FTOC CRC.
        */
        skip_bits(gb, h->ftoc_bytes * 8 - get_bits_count(gb));
        if (parse_chunks(h))
            return DTSUHD_INVALID_FRAME;
        update_descriptor(h, di);
    }

    /* 6.3.6.9: audio frame duration may be a fraction of metadata frame duration. */
    for (i = 0; i < h->navi_count; i++) {
        if (h->navi[i].present) {
            if (h->navi[i].id == 3)
                fraction = 2;
            else if (h->navi[i].id == 4)
                fraction = 4;
        }
    }

    if (fi) {
        fi->sync = h->is_sync_frame;
        fi->frame_bytes = h->frame_bytes;
        fi->sample_rate = h->sample_rate;
        fi->sample_count = (h->frame_duration * fi->sample_rate) / (h->clock_rate * fraction);
        fi->duration = (double)fi->sample_count / fi->sample_rate;
    }

    return DTSUHD_OK;
}

/** Return the offset of the first UHD audio frame.
    When supplied a buffer containing DTSHDHDR file content, the DTSHD
    headers are skipped and the offset to the first byte of the STRMDATA
    chunk is returned, along with the size of that chunk.

  @param[in] dataStart DTS:X Profile 2 file content to parse
  @param[in] dataSize Number of valid bytes in 'dataStart'
  @param[out] Number of leading DTS:X Profile 2 audio frames to discard,
              may be NULL
  @param[out] Size of STRMDATA payload, may be NULL
  @return STRMDATA payload offset or 0 if not a valid DTS:X Profile 2 file
*/
int dtsuhd_strmdata_payload(const uint8_t *data_start, int data_size, size_t *strmdata_size)
{
    const uint8_t *data = data_start;
    const uint8_t *data_end = data + data_size;
    uint64_t chunk_size = 0;

    if (data + DTSUHD_CHUNK_HEADER >= data_end || memcmp(data, "DTSHDHDR", 8))
        return 0;

    for (; data + DTSUHD_CHUNK_HEADER + 4 <= data_end; data += chunk_size + DTSUHD_CHUNK_HEADER) {
        chunk_size = AV_RB64(data + 8);

        if (!memcmp(data, "STRMDATA", 8)) {
            if (strmdata_size)
                *strmdata_size = chunk_size;
            return (int)(data - data_start) + DTSUHD_CHUNK_HEADER;
        }
    }

    return 0;
}
