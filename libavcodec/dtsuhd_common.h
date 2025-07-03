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

#ifndef AVCODEC_DTSUHD_COMMON_H
#define AVCODEC_DTSUHD_COMMON_H

#include <stdint.h>
#include <stdlib.h>

#define DTSUHD_NONSYNCWORD 0x71C442E8
#define DTSUHD_SYNCWORD    0x40411BF2

#define DTSUHD_MAX_FRAME_SIZE 0x1000

/* Return codes from dtsuhd_frame */
enum DTSUHDStatus {
    DTSUHD_OK,
    DTSUHD_INCOMPLETE,    /* Entire frame not in buffer. */
    DTSUHD_INVALID_FRAME, /* Error parsing frame. */
    DTSUHD_NOSYNC,        /* No sync frame prior to non-sync frame. */
    DTSUHD_NULL,          /* Function parameter may not be NULL. */
};

/* Return stream information from an audio frame parsed by dtsuhd_frame, */
typedef struct DTSUHDDescriptorInfo {
    unsigned valid:1; /* True if descriptor info is valid. */
    char coding_name[5]; /* Four character, null term SampleEntry box name. */
    int base_sample_freq_code;
    int channel_count;
    int decoder_profile_code;
    int frame_duration_code;
    int max_payload_code;
    int num_pres_code;
    int rep_type;
    int sample_rate;
    int sample_rate_mod;
    int sample_size;
    int channel_mask;
    uint64_t ffmpeg_channel_mask;
} DTSUHDDescriptorInfo;

/* Return frame information from an audio frame parsed by dtsuhd_frame. */
typedef struct DTSUHDFrameInfo {
    double duration;  /* Duration of frame in seconds (seconds per frame). */
    int frame_bytes;  /* Size of entire frame in bytes. */
    int sample_count; /* Number of samples in frame (samples per frame). */
    int sample_rate;  /* Sample rate of frame (samples per second). */
    unsigned sync:1;  /* True if frame is a sync frame. */
} DTSUHDFrameInfo;

struct DTSUHD;
typedef struct DTSUHD DTSUHD;

struct DTSUHD *dtsuhd_create(void);
void dtsuhd_destroy(DTSUHD*);
int dtsuhd_frame(DTSUHD*, const uint8_t *data, size_t nData,
                 DTSUHDFrameInfo*, DTSUHDDescriptorInfo*);
int dtsuhd_strmdata_payload(const uint8_t *data_start, int data_size,
                            size_t *strmdata_size);

static inline int dtsuhd_is_syncword(uint32_t syncword)
{
    return syncword == DTSUHD_NONSYNCWORD || syncword == DTSUHD_SYNCWORD;
}

#endif /* AVCODEC_DTSUHD_COMMON_H */
