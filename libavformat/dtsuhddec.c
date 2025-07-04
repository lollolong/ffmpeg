/*
 * DTS-UHD audio demuxer
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
 * Report DTS-UHD audio stream configuration and extract raw packet data.
 */

#include "demux.h"
#include "internal.h"
#include "libavcodec/dtsuhd_common.h"
#include "libavcodec/put_bits.h"
#include "libavutil/mem.h"
#include "libavutil/intreadwrite.h"

#define DTSUHD_BUFFER_SIZE (1024 * 1024)

typedef struct DTSUHDDemuxContext {
    size_t data_end;
    struct DTSUHD *dtsuhd;
    uint8_t *buf;
} DTSUHDDemuxContext;

static int probe(const AVProbeData *p)
{
    int offset = dtsuhd_strmdata_payload(p->buf, p->buf_size, NULL);
    int score = 0;
    struct DTSUHD *h = dtsuhd_create();

    for (; offset + 4 < p->buf_size; offset++) {
        if (dtsuhd_is_syncword(AV_RB32(p->buf + offset))) {
            if (DTSUHD_OK == dtsuhd_frame(h, p->buf + offset, p->buf_size - offset, NULL, NULL)) {
                score = AVPROBE_SCORE_MAX - 3;
                break;
            }
        }
    }

    dtsuhd_destroy(h);
    return score;
}

static av_cold int read_close(AVFormatContext *s)
{
    DTSUHDDemuxContext *dtsxs = s->priv_data;

    av_freep(&dtsxs->buf);
    dtsuhd_destroy(dtsxs->dtsuhd);
    dtsxs->dtsuhd = NULL;

    return 0;
}

static int find_first_syncword(DTSUHDDemuxContext *dtsuhd, int data_start)
{
    while (data_start + 4 < DTSUHD_BUFFER_SIZE &&
        !dtsuhd_is_syncword(AV_RB32(dtsuhd->buf + data_start))) {
        data_start++;
    }

    return data_start;
}

static int write_extradata(AVCodecParameters *par, DTSUHDDescriptorInfo *di)
{
    PutBitContext pbc;
    int ret;
    int size;
    uint8_t udts[32];

    init_put_bits(&pbc, udts, sizeof(udts));
    put_bits32(&pbc, 0); // udts box size
    put_bits(&pbc, 8, 'u'); // udts box signature
    put_bits(&pbc, 8, 'd');
    put_bits(&pbc, 8, 't');
    put_bits(&pbc, 8, 's');
    put_bits(&pbc, 6, di->decoder_profile_code);
    put_bits(&pbc, 2, di->frame_duration_code);
    put_bits(&pbc, 3, di->max_payload_code);
    put_bits(&pbc, 5, di->num_pres_code);
    put_bits32(&pbc,  di->channel_mask);
    put_bits(&pbc, 1, di->base_sample_freq_code);
    put_bits(&pbc, 2, di->sample_rate_mod);
    put_bits(&pbc, 3, di->rep_type);
    put_bits(&pbc, 3, 0);
    put_bits(&pbc, 1, 0);
    put_bits64(&pbc, di->num_pres_code + 1, 0); // ID Tag present for each presentation.
    flush_put_bits(&pbc); // byte align
    size = put_bits_count(&pbc) >> 3;
    AV_WB32(udts, size);

    ret = ff_alloc_extradata(par, size);
    if (ret < 0)
        return ret;

    memcpy(par->extradata, udts, size);

    return 0;
}

static int read_header(AVFormatContext *s)
{
    AVIOContext *pb = s->pb;
    AVStream *st = avformat_new_stream(s, NULL);
    DTSUHDDemuxContext *dtsuhd = s->priv_data;
    DTSUHDDescriptorInfo di;
    DTSUHDFrameInfo fi;
    int buf_bytes;
    int ret = DTSUHD_INVALID_FRAME;
    int data_start;

    if (!(pb->seekable & AVIO_SEEKABLE_NORMAL))
        return AVERROR(EIO);

    dtsuhd->buf = av_malloc(DTSUHD_BUFFER_SIZE);
    dtsuhd->dtsuhd = dtsuhd_create();
    if (!dtsuhd->buf || !dtsuhd->dtsuhd || !st)
        return AVERROR(ENOMEM);

    buf_bytes = avio_read(pb, dtsuhd->buf, DTSUHD_BUFFER_SIZE);
    if (buf_bytes < 0)
        return buf_bytes;

    data_start = dtsuhd_strmdata_payload(dtsuhd->buf, buf_bytes, &dtsuhd->data_end);
    dtsuhd->data_end += data_start;
    if (data_start == 0)
        dtsuhd->data_end = avio_size(pb); // Not a DTSHDHDR chunk file, decode frames to end of file.

    data_start = find_first_syncword(dtsuhd, data_start);
    if (avio_seek(pb, data_start, SEEK_SET) < 0)
        return AVERROR(EINVAL);

    ret = dtsuhd_frame(dtsuhd->dtsuhd, dtsuhd->buf + data_start,
        buf_bytes - data_start, &fi, &di);
    if (ret != DTSUHD_OK || !di.valid) {
        av_log(s, AV_LOG_ERROR, "Unable to process DTS-UHD file. File may be invalid.\n");
        return AVERROR_INVALIDDATA;
    }

    ffstream(st)->need_parsing = AVSTREAM_PARSE_FULL_RAW;
    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_DTS;
    st->codecpar->ch_layout.order = AV_CHANNEL_ORDER_NATIVE;
    st->codecpar->ch_layout.nb_channels = di.channel_count;
    st->codecpar->ch_layout.u.mask = di.ffmpeg_channel_mask;
    st->codecpar->codec_tag = AV_RL32(di.coding_name);
    st->codecpar->frame_size = 512 << di.frame_duration_code;
    st->codecpar->sample_rate = di.sample_rate;

#if FF_API_OLD_CHANNEL_LAYOUT
FF_DISABLE_DEPRECATION_WARNINGS
    st->codecpar->channels = di.channel_count;
    st->codecpar->channel_layout = di.ffmpeg_channel_mask;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    ret = write_extradata(st->codecpar, &di);
    if (ret < 0)
        return ret;

    if (st->codecpar->sample_rate)
        avpriv_set_pts_info(st, 64, 1, st->codecpar->sample_rate);

    return 0;
}

static int read_packet(AVFormatContext *s, AVPacket *pkt)
{
    DTSUHDDemuxContext *dtsuhd = s->priv_data;
    int64_t size, left;
    int ret;

    left = dtsuhd->data_end - avio_tell(s->pb);
    size = FFMIN(left, DTSUHD_MAX_FRAME_SIZE);
    if (size <= 0)
        return AVERROR_EOF;

    ret = av_get_packet(s->pb, pkt, size);
    if (ret < 0)
        return ret;

    pkt->stream_index = 0;

    return ret;
}

FFInputFormat ff_dtsuhd_demuxer = {
    .p.name         = "dtsuhd",
    .p.long_name    = NULL_IF_CONFIG_SMALL("DTS-UHD"),
    .p.flags        = AVFMT_GENERIC_INDEX,
    .p.extensions   = "dtsx",
    .priv_data_size = sizeof(DTSUHDDemuxContext),
    .read_probe     = probe,
    .read_header    = read_header,
    .read_packet    = read_packet,
    .read_close     = read_close,
    .raw_codec_id   = AV_CODEC_ID_DTSUHD,
};
