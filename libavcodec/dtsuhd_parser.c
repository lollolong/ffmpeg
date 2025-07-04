/*
 * DTS-UHD audio frame parsing code
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
 * Parse raw DTS-UHD audio frame input and return individual audio frames.
 */

#include "dtsuhd_common.h"
#include "libavutil/mem.h"
#include "libavutil/intreadwrite.h"
#include "parser.h"

#define DTSUHD_BUFFER_SIZE (DTSUHD_MAX_FRAME_SIZE * 128)

typedef struct DTSUHDParseContext {
    DTSUHD *dtsuhd;
    int buf_offset;
    int buf_bytes;
    int frame_bytes;
    uint8_t *buf;
} DTSUHDParseContext;

static av_cold int parser_init(AVCodecParserContext *s)
{
    DTSUHDParseContext *pc = s->priv_data;

    pc->dtsuhd = dtsuhd_create();
    pc->buf = av_calloc(DTSUHD_BUFFER_SIZE + AV_INPUT_BUFFER_PADDING_SIZE, 1);
    if (!pc->dtsuhd || !pc->buf)
        return AVERROR(ENOMEM);

    return 0;
}

static void parser_close(AVCodecParserContext *s)
{
    DTSUHDParseContext *pc = s->priv_data;

    dtsuhd_destroy(pc->dtsuhd);
    pc->dtsuhd = NULL;
    av_freep(&pc->buf);
    ff_parse_close(s);
}

// Keep data in contiguous buffer as required by dtsuhd_frame.
static int append_buffer(DTSUHDParseContext *pc, const uint8_t **buf, int *buf_size, int *input_consumed)
{
    int copy_bytes;

    pc->buf_offset += pc->frame_bytes;
    pc->frame_bytes = 0;

    // Buffer almost full, move partial frame to start of buffer for more space.
    if (*buf_size > 0 && pc->buf_bytes + *buf_size > DTSUHD_BUFFER_SIZE) {
        memmove(pc->buf, pc->buf + pc->buf_offset, pc->buf_bytes);
        pc->buf_bytes -= pc->buf_offset;
        pc->buf_offset = 0;
    }

    copy_bytes = FFMAX(0, FFMIN(DTSUHD_BUFFER_SIZE - pc->buf_bytes, *buf_size));

    // Append input buffer to our context.
    if (copy_bytes) {
        memcpy(pc->buf + pc->buf_bytes, *buf, copy_bytes);
        pc->buf_bytes += copy_bytes;
    }

    // Ensure buffer starts with a syncword
    while (pc->buf_offset + 4 < pc->buf_bytes && !dtsuhd_is_syncword(AV_RB32(pc->buf + pc->buf_offset)))
        pc->buf_offset++;

    *input_consumed = copy_bytes;
    *buf = pc->buf + pc->buf_offset;
    *buf_size = pc->buf_bytes - pc->buf_offset;

    return copy_bytes && pc->buf_bytes - pc->buf_offset < DTSUHD_MAX_FRAME_SIZE;
}

static int parser_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                        const uint8_t **poutbuf, int *poutbuf_size,
                        const uint8_t *buf, int buf_size)
{
    DTSUHDParseContext *pc = s->priv_data;
    DTSUHDFrameInfo fi;
    int input_consumed = 0;

    if (append_buffer(pc, &buf, &buf_size, &input_consumed)) {
        *poutbuf = NULL;
        *poutbuf_size = 0;
        return input_consumed;
    }

    switch (dtsuhd_frame(pc->dtsuhd, buf, buf_size, &fi, NULL)) {
    case DTSUHD_OK:
        if (fi.sample_count)
            s->duration = fi.sample_count;
        if (fi.sample_rate)
            avctx->sample_rate = fi.sample_rate;
        buf_size = pc->frame_bytes = fi.frame_bytes;
        break;
    case DTSUHD_INCOMPLETE:
        pc->frame_bytes = buf_size;
        buf = NULL;
        buf_size = 0;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "Unable to process DTS-UHD file. File may be invalid.\n");
        return AVERROR_INVALIDDATA;
    }

    *poutbuf      = buf;
    *poutbuf_size = buf_size;

    return input_consumed;
}

AVCodecParser ff_dtsuhd_parser = {
    .codec_ids      = { AV_CODEC_ID_DTSUHD },
    .priv_data_size = sizeof(DTSUHDParseContext),
    .parser_init    = parser_init,
    .parser_parse   = parser_parse,
    .parser_close   = parser_close,
};
