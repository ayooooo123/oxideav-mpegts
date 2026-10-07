// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavformat/demux.c: parse_packet, the parser path
// of read_frame_internal and av_read_frame, avformat_find_stream_info's
// read-ahead (its probesize / analyzeduration limits and the codec context
// the decoders it opens report) and estimate_timings_from_pts (the packet
// queue flushed, parsers closed and the input read again from the start),
// ff_read_frame_flush / ff_update_cur_dts on seek; with libavformat/mpegts.c's
// stream setup (every PES stream is parsed with AVSTREAM_PARSE_FULL; stream
// types 0x03 and 0x04 start as MP3; AVFMTCTX_NOHEADER).
// Copyright (c) 2000, 2001, 2002 Fabrice Bellard (demux.c)
// Copyright (c) 2002-2003 Fabrice Bellard (mpegts.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

//! The demuxer the `"mpegts"` container registration returns: packets as
//! FFmpeg's `av_read_frame` returns them for a transport stream.
//!
//! H.264, MPEG-1/2 video, MPEG audio, AC-3 / E-AC-3 and ADTS AAC streams
//! go through ports of FFmpeg's codec parsers: one packet per access unit,
//! with FFmpeg's timestamps (PES timestamps fetched for the unit that
//! starts in the PES, the rest interpolated), durations and keyframe
//! flags. The transport stream's own random-access indicator rides
//! [`PacketMetadata::container_keyframe`] on the unit that starts in the
//! announced PES. Streams FFmpeg has no port for here pass through as
//! PES payloads.
//!
//! Opening reads ahead as `avformat_find_stream_info` does (at most
//! FFmpeg's 5 MB probe size or about five seconds), which establishes
//! each parsed stream's parameters (size, pixel format and frame rate;
//! sample rate, channels and the codec the headers name) and the codec
//! state its packets are timed with. Then, as FFmpeg's
//! `estimate_timings_from_pts` does for a seekable transport stream, the
//! read-ahead's packets are dropped and demuxing starts over at the first
//! byte with fresh parsers: the first packets are timed with what the
//! read-ahead learnt (a frame rate stated only in a later SPS, an AAC
//! frame size only the decoder knows).

use std::collections::VecDeque;

use oxideav_core::{
    CodecId, Demuxer, Error as CoreError, MediaType, Packet, PacketMetadata, PixelFormat, Rational,
    Result as CoreResult, StreamInfo, TimeBase,
};

use crate::demuxer::{MpegTsDemuxer, PesInfo};
use crate::ff::h264::OutputPicture;
use crate::ff::parser::{
    is_relative, pict, rescale, rescale_rnd, Codec, CodecCtx, Parser, PixFmt, Rnd, MAX_HELD_BYTES,
    NOPTS, RELATIVE_TS_BASE,
};
use crate::ff::timing::{r_frame_rate, Pending, Rfps, StreamTiming, TIME_BASE};

/// avformat_find_stream_info's default probesize: payload bytes the
/// read-ahead returns.
const PROBE_BYTES: u64 = 5_000_000;
/// Input the read-ahead reads at most, beyond which it ends as if the
/// probe size were reached. FFmpeg has no such bound (input of null
/// packets keeps it reading); this one only binds on input like that,
/// far past the probe size and analyze duration of any real stream.
const READ_AHEAD_TS_BYTES: u64 = 128 << 20;
/// Memory the read-ahead's queued packets may take, each charged its
/// payload allocation and [`QUEUE_ENTRY_BYTES`]: the bound for input of
/// many tiny packets, which the probe size (payload bytes) does not see.
const READ_AHEAD_QUEUE_BYTES: usize = 64 << 20;
/// A queued packet's own cost beyond its payload: the entry and the
/// payload allocation's bookkeeping.
const QUEUE_ENTRY_BYTES: usize = std::mem::size_of::<Pending>() + 32;
/// max_analyze_duration once every stream is analyzed (5 s).
const ANALYZE_ALL_US: i64 = 5_000_000;
/// max_stream_analyze_duration for "mpegts" (7 s).
const ANALYZE_STREAM_US: i64 = 7_000_000;
/// fps_analyze_framecount: H.264 and MPEG-2 time bases are unreliable.
const FPS_ANALYZE_FRAMES: u64 = 20;
/// max_ts_probe: units a stream may yield without a timestamp.
const MAX_TS_PROBE: u64 = 50;
/// Parser calls one PES may take: each call consumes input or returns a
/// held unit, so this only bounds a parser bug, not real input.
const MAX_CALLS_PER_PES: usize = 1 << 20;

/// The parser stage of one stream.
struct ParsedStream {
    parser: Parser,
    avctx: CodecCtx,
    timing: StreamTiming,
    /// codec_info_nb_frames: units find_stream_info has decoded.
    nb_frames: u64,
    /// codec_info_duration, in 90 kHz units.
    info_duration: i64,
    fps_first_dts: i64,
    fps_last_dts: i64,
    /// A video stream's r_frame_rate estimation, while reading ahead.
    rfps: Option<Rfps>,
    /// The H.264 decoder's POC history, while reading ahead.
    output_order: OutputOrder,
}

/// H264_MAX_DPB_FRAMES.
const MAX_DPB_FRAMES: usize = 16;

/// The POC history h264_select_output_frame grows the reorder depth
/// from (libavcodec/h264_slice.c:1328-1351): the highest POCs of the
/// pictures decoded since the last reset, ascending. Reset by an IDR
/// (idr()), after an MMCO_RESET picture (ff_h264_execute_ref_pic_marking)
/// and on an impossible order. The decoder also resets it on a missing
/// reference or a frame_num gap, which need its reference lists and are
/// not emulated.
struct OutputOrder {
    last_pocs: [i32; MAX_DPB_FRAMES],
}

impl OutputOrder {
    fn new() -> Self {
        Self {
            last_pocs: [i32::MIN; MAX_DPB_FRAMES],
        }
    }

    /// One decoded picture: the reorder depth its order shows raises
    /// `has_b_frames` unless the SPS states one (`restriction`).
    fn select(&mut self, picture: OutputPicture, has_b_frames: &mut i32, restriction: bool) {
        if picture.idr {
            *self = Self::new();
        }
        let pocs = &mut self.last_pocs;
        let mut i = 0;
        loop {
            if i == MAX_DPB_FRAMES || picture.poc < pocs[i] {
                if i > 0 {
                    pocs[i - 1] = picture.poc;
                }
                break;
            } else if i > 0 {
                pocs[i - 1] = pocs[i];
            }
            i += 1;
        }
        let mut out_of_order = MAX_DPB_FRAMES - i;
        let gap = pocs[MAX_DPB_FRAMES - 2] > i32::MIN
            && i64::from(pocs[MAX_DPB_FRAMES - 1]) - i64::from(pocs[MAX_DPB_FRAMES - 2]) > 2;
        if picture.b || gap {
            out_of_order = out_of_order.max(1);
        }
        if out_of_order == MAX_DPB_FRAMES {
            *self = Self::new();
            self.last_pocs[0] = picture.poc;
        } else if (*has_b_frames as usize) < out_of_order && !restriction {
            *has_b_frames = out_of_order as i32;
        }
        if picture.mmco_reset {
            *self = Self::new();
        }
    }
}

enum Stage {
    /// No parser: PES payloads as they are. `intra_only` is
    /// ff_is_intra_only for the codec (subtitles, DTS, PCM): those
    /// packets are keyframes; the rest keep the container's indicator.
    /// `seen` records whether the read-ahead met a packet.
    Raw {
        intra_only: bool,
        seen: bool,
    },
    Parsed(Box<ParsedStream>),
}

/// The FFmpeg codec whose parser a stream goes through, if ported.
fn parsed_codec(codec_id: &str, stream_type: Option<u8>) -> Option<Codec> {
    Some(match codec_id {
        "h264" => Codec::H264,
        // ISO_types: both MPEG video stream types start as MPEG-2; the
        // parser names MPEG-1 from a sequence header without extension.
        "mpeg1video" | "mpeg2video" => Codec::Mpeg2Video,
        // ISO_types: MPEG-1 and MPEG-2 audio are AV_CODEC_ID_MP3 until
        // the parser reads a frame header's layer.
        "mp1" | "mp2" | "mp3" => Codec::Mp3,
        // Only ADTS carries the sync words the aac parser splits on.
        "aac" if stream_type == Some(0x0F) => Codec::Aac,
        "ac3" => Codec::Ac3,
        "eac3" => Codec::Eac3,
        _ => return None,
    })
}

fn pixel_format(fmt: PixFmt) -> Option<PixelFormat> {
    Some(match fmt {
        PixFmt::Yuv420p => PixelFormat::Yuv420P,
        PixFmt::Yuv422p => PixelFormat::Yuv422P,
        PixFmt::Yuv444p => PixelFormat::Yuv444P,
        PixFmt::Yuv420p10 => PixelFormat::Yuv420P10Le,
        PixFmt::Yuv422p10 => PixelFormat::Yuv422P10Le,
        PixFmt::Yuv444p10 => PixelFormat::Yuv444P10Le,
        PixFmt::Yuv420p9 | PixFmt::Yuv422p9 | PixFmt::Yuv444p9 => return None,
    })
}

/// MPEG-TS demuxed as FFmpeg demuxes it. See the module documentation.
pub struct ParsedDemuxer {
    inner: MpegTsDemuxer,
    streams: Vec<StreamInfo>,
    stages: Vec<Stage>,
    /// FFmpeg's packet_buffer and parse_queue: packets not yet returned.
    queue: VecDeque<Pending>,
    meta: PacketMetadata,
    eof: bool,
    /// Inside the open-time read-ahead (find_stream_info).
    probing: bool,
    /// A read error met during the read-ahead of an input that cannot
    /// be read again, returned after the packets read before it.
    deferred: Option<CoreError>,
}

impl std::fmt::Debug for ParsedDemuxer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ParsedDemuxer")
            .field("streams", &self.streams.len())
            .field("queued", &self.queue.len())
            .field("eof", &self.eof)
            .finish_non_exhaustive()
    }
}

impl ParsedDemuxer {
    /// Wrap an opened [`MpegTsDemuxer`]: read ahead for the stream
    /// parameters, then start over at the first byte.
    pub fn new(inner: MpegTsDemuxer) -> CoreResult<Self> {
        let streams = inner.streams().to_vec();
        let stages = streams
            .iter()
            .map(
                |s| match parsed_codec(s.params.codec_id.as_str(), inner.stream_type(s.index)) {
                    Some(codec) => Stage::Parsed(Box::new(ParsedStream {
                        parser: Parser::new(codec),
                        avctx: CodecCtx::new(codec),
                        timing: StreamTiming::new(),
                        nb_frames: 0,
                        info_duration: 0,
                        fps_first_dts: NOPTS,
                        fps_last_dts: NOPTS,
                        rfps: codec.is_video().then(Rfps::new),
                        output_order: OutputOrder::new(),
                    })),
                    None => Stage::Raw {
                        intra_only: s.params.media_type == MediaType::Subtitle
                            || s.params.codec_id.as_str() == "dts"
                            || s.params.codec_id.as_str().starts_with("pcm_"),
                        seen: false,
                    },
                },
            )
            .collect();
        let mut d = Self {
            inner,
            streams,
            stages,
            queue: VecDeque::new(),
            meta: PacketMetadata::default(),
            eof: false,
            probing: true,
            deferred: None,
        };
        d.find_stream_info();
        // ff_rfps_calculate and the r_frame_rate find_stream_info leaves.
        for stage in &mut d.stages {
            if let Stage::Parsed(p) = stage {
                if let Some(rfps) = p.rfps.take() {
                    let estimated = rfps.calculate(&p.avctx, p.info_duration);
                    p.avctx.r_frame_rate = r_frame_rate(&p.avctx, estimated);
                }
            }
        }
        d.apply_parameters();
        d.probing = false;
        // estimate_timings_from_pts. An input that cannot seek back keeps
        // the read-ahead's packets, as FFmpeg does for an unseekable one.
        if d.inner.rewind().is_ok() {
            d.queue.clear();
            d.deferred = None;
            d.eof = false;
            for stage in &mut d.stages {
                if let Stage::Parsed(p) = stage {
                    p.parser = Parser::new(p.avctx.codec);
                    p.timing.restart();
                }
            }
        }
        Ok(d)
    }

    /// The read-ahead, until the probe size, the analyze duration, the
    /// end of input, or a read error; and, on input those never end (no
    /// packets, or packets too small to add up), until
    /// [`READ_AHEAD_TS_BYTES`] of input or [`READ_AHEAD_QUEUE_BYTES`] of
    /// queued packets.
    fn find_stream_info(&mut self) {
        let mut read_size = 0u64;
        let mut queued = 0usize;
        let limit = self.inner.position().saturating_add(READ_AHEAD_TS_BYTES);
        while !self.eof && read_size < PROBE_BYTES {
            let before = self.queue.len();
            match self.read_pes(Some(limit)) {
                Ok(true) => {}
                Ok(false) => return,
                Err(e) => {
                    self.deferred = Some(e);
                    return;
                }
            }
            // The units just queued, returned one by one as
            // read_frame_internal returns them.
            for at in before..self.queue.len() {
                let (stream, size, dts, duration) = {
                    let e = &self.queue[at];
                    queued += QUEUE_ENTRY_BYTES + e.data.capacity();
                    (e.stream as usize, e.data.len() as u64, e.dts, e.duration)
                };
                if queued > READ_AHEAD_QUEUE_BYTES {
                    return;
                }
                read_size += size;
                let all = self.analyzed_all();
                match self.stages.get_mut(stream) {
                    Some(Stage::Raw { seen, .. }) => *seen = true,
                    Some(Stage::Parsed(p)) => {
                        if !p.returned(dts, duration, all) {
                            return;
                        }
                        // extract_extradata, while the stream has none.
                        if p.avctx.extradata.is_empty() && p.avctx.codec == Codec::H264 {
                            if let Some(extradata) =
                                crate::ff::h264::extract_extradata(&self.queue[at].data)
                            {
                                p.avctx.extradata = extradata;
                            }
                        }
                    }
                    None => {}
                }
            }
        }
    }

    /// Every stream passes find_stream_info's per-stream checks.
    fn analyzed_all(&self) -> bool {
        self.stages.iter().all(|stage| match stage {
            Stage::Raw { seen, .. } => *seen,
            Stage::Parsed(p) => p.analyzed(),
        })
    }

    /// What the streams state after the read-ahead, as find_stream_info
    /// leaves it in codecpar.
    fn apply_parameters(&mut self) {
        for (info, stage) in self.streams.iter_mut().zip(&self.stages) {
            let Stage::Parsed(p) = stage else { continue };
            let params = &mut info.params;
            params.codec_id = CodecId::new(p.avctx.codec.name());
            if p.avctx.codec.is_video() {
                if p.avctx.width > 0 && p.avctx.height > 0 {
                    params.width = Some(p.avctx.width as u32);
                    params.height = Some(p.avctx.height as u32);
                }
                if let Some(fmt) = p.parser.state.format.and_then(pixel_format) {
                    params.pixel_format = Some(fmt);
                }
                // The codec's rate, else the one estimated from the DTS
                // (ffprobe's r_frame_rate), not the 1/time_base default.
                let fr = p.avctx.framerate;
                let r = p.avctx.r_frame_rate;
                if fr.num > 0 && fr.den > 0 {
                    params.frame_rate = Some(Rational::new(fr.num, fr.den));
                } else if r.num > 0 && r.den > 0 && (r.num, r.den) != (TIME_BASE.den, TIME_BASE.num)
                {
                    params.frame_rate = Some(Rational::new(r.num, r.den));
                }
            } else {
                if p.avctx.sample_rate > 0 {
                    params.sample_rate = Some(p.avctx.sample_rate as u32);
                }
                if p.avctx.channels > 0 {
                    params.channels = Some(p.avctx.channels as u16);
                }
            }
        }
    }

    /// read_frame_internal's step: one PES through its stream's stage, or
    /// at the end of input every parser flushed. With a `limit`, false
    /// when the input position reached it before a PES did.
    fn read_pes(&mut self, limit: Option<u64>) -> CoreResult<bool> {
        let next = match limit {
            Some(limit) => self.inner.next_pes_before(limit),
            None => self.inner.next_pes().map(Some),
        };
        match next {
            Ok(None) => Ok(false),
            Ok(Some((pkt, info))) => {
                let index = pkt.stream_index as usize;
                match self.stages.get_mut(index) {
                    Some(Stage::Raw { intra_only, .. }) => {
                        let key = *intra_only || pkt.flags.keyframe;
                        self.queue.push_back(Pending {
                            stream: pkt.stream_index,
                            pts: pkt.pts.unwrap_or(NOPTS),
                            dts: pkt.dts.unwrap_or(NOPTS),
                            duration: pkt.duration.unwrap_or(0),
                            key,
                            container_key: info.random_access,
                            data: pkt.data,
                        });
                    }
                    Some(Stage::Parsed(p)) => {
                        let probing = self.probing;
                        parse_packet(
                            p,
                            &mut self.queue,
                            pkt.stream_index,
                            &pkt,
                            info,
                            false,
                            probing,
                        )?;
                    }
                    None => {}
                }
                Ok(true)
            }
            Err(CoreError::Eof) => {
                let probing = self.probing;
                for (index, stage) in self.stages.iter_mut().enumerate() {
                    if let Stage::Parsed(p) = stage {
                        let flush = Packet::new(index as u32, TimeBase::new(1, 90_000), Vec::new());
                        parse_packet(
                            p,
                            &mut self.queue,
                            index as u32,
                            &flush,
                            PesInfo::default(),
                            true,
                            probing,
                        )?;
                    }
                }
                self.eof = true;
                Ok(true)
            }
            Err(e) => Err(e),
        }
    }

    fn to_packet(e: Pending) -> Packet {
        let ts = |t: i64| {
            (t != NOPTS).then(|| {
                if is_relative(t) {
                    t.wrapping_sub(RELATIVE_TS_BASE)
                } else {
                    t
                }
            })
        };
        let mut pkt = Packet::new(e.stream, TimeBase::new(1, 90_000), e.data).with_keyframe(e.key);
        pkt.pts = ts(e.pts);
        pkt.dts = ts(e.dts);
        pkt.duration = (e.duration != 0).then_some(e.duration);
        pkt
    }
}

impl ParsedStream {
    /// find_stream_info's per-stream "still needs to be handled" checks.
    fn analyzed(&self) -> bool {
        let video = self.avctx.codec.is_video();
        let parameters = self.has_codec_parameters();
        // extract_extradata: H.264 waits for SPS and PPS, MPEG video for a
        // sequence header.
        let extradata = match self.avctx.codec {
            Codec::H264 => self.parser.h264_reorder().is_some(),
            Codec::Mpeg1Video | Codec::Mpeg2Video => self.avctx.width > 0,
            _ => true,
        };
        let fps = !video || self.nb_frames.saturating_sub(1) >= FPS_ANALYZE_FRAMES;
        let timestamps = self.timing.first_dts != NOPTS || self.nb_frames >= MAX_TS_PROBE;
        parameters && extradata && fps && timestamps
    }

    /// has_codec_parameters, as far as the parser and the emulated
    /// decoder state it.
    fn has_codec_parameters(&self) -> bool {
        if self.avctx.codec.is_video() {
            self.avctx.width > 0 && self.avctx.height > 0 && self.parser.state.format.is_some()
        } else {
            // A decoded frame tells the sample format and frame size.
            self.avctx.sample_rate > 0 && self.avctx.channels > 0 && self.nb_frames > 0
        }
    }

    /// The bookkeeping find_stream_info does on a unit it returns: false
    /// when the analyze duration is reached (the read-ahead ends there).
    fn returned(&mut self, dts: i64, duration: i64, analyzed_all: bool) -> bool {
        if dts != NOPTS && self.nb_frames > 1 {
            if self.fps_first_dts == NOPTS {
                self.fps_first_dts = dts;
            }
            self.fps_last_dts = dts;
        }
        if self.nb_frames > 1 {
            let mut t = rescale(self.info_duration, 1_000_000, 90_000);
            if t == 0
                && self.nb_frames > 30
                && self.fps_first_dts != NOPTS
                && self.fps_last_dts != NOPTS
            {
                t = rescale(
                    self.fps_last_dts.saturating_sub(self.fps_first_dts),
                    1_000_000,
                    90_000,
                );
            }
            let limit = if analyzed_all {
                ANALYZE_ALL_US
            } else {
                ANALYZE_STREAM_US
            };
            if t >= limit {
                return false;
            }
            if duration > 0 {
                self.info_duration = self.info_duration.saturating_add(duration);
            }
        }
        // ff_rfps_add_frame for video.
        if let Some(rfps) = &mut self.rfps {
            rfps.add_frame(dts);
        }
        // try_decode_frame, while it still decodes (has_codec_parameters
        // and has_decode_delay_been_guessed stop it): the reorder depth
        // FFmpeg's H.264 decoder finds reaches the codec context, and the
        // AAC frame size.
        if !(self.has_codec_parameters() && self.decode_delay_guessed()) {
            if let Some((restriction, reorder)) = self.parser.h264_reorder() {
                if restriction {
                    self.avctx.has_b_frames = self.avctx.has_b_frames.max(reorder);
                }
                if let Some(picture) = self.parser.h264_picture() {
                    self.output_order
                        .select(picture, &mut self.avctx.has_b_frames, restriction);
                }
            }
        }
        if let Some(adts) = self.parser.last_adts() {
            self.avctx.frame_size = adts.samples;
            self.avctx.sample_rate = adts.sample_rate;
            if adts.channels > 0 {
                self.avctx.channels = adts.channels;
            }
        }
        self.nb_frames += 1;
        true
    }

    /// has_decode_delay_been_guessed while find_stream_info decodes.
    fn decode_delay_guessed(&self) -> bool {
        if self.avctx.codec != Codec::H264 {
            return true;
        }
        let has_b_frames = self.avctx.has_b_frames;
        if has_b_frames != 0
            && self.parser.h264_reorder().map(|(_, reorder)| reorder) == Some(has_b_frames)
        {
            return true;
        }
        let decoded = self.nb_frames.saturating_sub(has_b_frames.max(0) as u64);
        match has_b_frames {
            ..=2 => decoded >= 7,
            3 => decoded >= 18,
            _ => decoded >= 20,
        }
    }
}

/// parse_packet: one demuxed PES (or, with `flush`, the end of input)
/// through the parser; each unit timed by compute_pkt_fields and queued.
/// A unit longer than the parser holds is an error; the stream's parser
/// starts over and the rest of this PES is dropped.
fn parse_packet(
    p: &mut ParsedStream,
    queue: &mut VecDeque<Pending>,
    stream: u32,
    pkt: &Packet,
    info: PesInfo,
    flush: bool,
    probing: bool,
) -> CoreResult<()> {
    let mut rest: &[u8] = &pkt.data;
    let (mut pts, mut dts) = (pkt.pts.unwrap_or(NOPTS), pkt.dts.unwrap_or(NOPTS));
    let mut pos = info.pos as i64;
    let mut rai = info.random_access;
    let mut got_output = flush;
    let mut calls = 0usize;
    while (!rest.is_empty() || (flush && got_output)) && calls < MAX_CALLS_PER_PES {
        calls += 1;
        let (next_pts, next_dts) = (pts, dts);
        let Ok((len, unit)) = p.parser.parse2(&mut p.avctx, rest, pts, dts, pos, rai) else {
            p.parser = Parser::new(p.avctx.codec);
            return Err(CoreError::ResourceExhausted(format!(
                "mpegts: stream {stream}: an access unit longer than {MAX_HELD_BYTES} bytes"
            )));
        };
        pts = NOPTS;
        dts = NOPTS;
        pos = -1;
        rai = false;
        rest = &rest[len.min(rest.len())..];
        got_output = unit.is_some();
        let Some(data) = unit else {
            if len == 0 {
                break;
            }
            continue;
        };
        let s = &p.parser.state;
        let mut e = Pending {
            stream,
            pts: s.pts,
            dts: s.dts,
            duration: 0,
            key: s.key_frame == 1 || (s.key_frame == -1 && s.pict_type == pict::I),
            container_key: s.rai,
            data,
        };
        if !p.avctx.codec.is_video() && p.avctx.sample_rate > 0 && s.duration > 0 {
            e.duration = rescale_rnd(
                i64::from(s.duration),
                90_000,
                i64::from(p.avctx.sample_rate),
                Rnd::Down,
            );
        }
        // Outside find_stream_info sti->info is gone: guessed.
        let guessed = !probing || p.decode_delay_guessed();
        p.timing.compute_pkt_fields(
            &mut p.avctx,
            &p.parser.state,
            &mut e,
            next_dts,
            next_pts,
            queue,
            guessed,
        );
        queue.push_back(e);
    }
    Ok(())
}

impl Demuxer for ParsedDemuxer {
    fn format_name(&self) -> &str {
        "mpegts"
    }

    fn streams(&self) -> &[StreamInfo] {
        &self.streams
    }

    fn next_packet(&mut self) -> CoreResult<Packet> {
        self.meta = PacketMetadata::default();
        loop {
            if let Some(e) = self.queue.pop_front() {
                self.meta.container_keyframe = e.container_key;
                return Ok(Self::to_packet(e));
            }
            if let Some(e) = self.deferred.take() {
                return Err(e);
            }
            if self.eof {
                return Err(CoreError::Eof);
            }
            // Units the failing read completed come out before its error.
            if let Err(e) = self.read_pes(None) {
                self.deferred = Some(e);
            }
        }
    }

    fn packet_metadata(&self) -> PacketMetadata {
        self.meta.clone()
    }

    /// The inner demuxer's keyframe-accurate seek, then FFmpeg's flush:
    /// queued packets dropped, parsers restarted, timing continued from
    /// the landing.
    fn seek_to(&mut self, stream_index: u32, pts: i64) -> CoreResult<i64> {
        self.meta = PacketMetadata::default();
        let landed = self.inner.seek_to(stream_index, pts)?;
        self.queue.clear();
        self.deferred = None;
        self.eof = false;
        for stage in &mut self.stages {
            if let Stage::Parsed(p) = stage {
                p.parser = Parser::new(p.avctx.codec);
                p.timing.seeked(landed);
            }
        }
        Ok(landed)
    }

    fn duration_micros(&self) -> Option<i64> {
        self.inner.duration_micros()
    }
}
