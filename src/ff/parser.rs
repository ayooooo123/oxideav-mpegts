// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/parser.c (av_parser_init,
// av_parser_parse2, ff_fetch_timestamp, ff_combine_frame) and the parts of
// libavutil/rational.c (av_reduce, av_mul_q) and libavutil/mathematics.c
// (av_rescale_rnd, av_add_stable) the parser stage uses.
// Copyright (c) 2003 Fabrice Bellard
// Copyright (c) 2003 Michael Niedermayer (parser.c)
// Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at> (rational.c)
// Copyright (c) 2005-2012 Michael Niedermayer <michaelni@gmx.at> (mathematics.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::{aac_ac3, h264, lcevc, mpegaudio, mpegvideo, opus};

/// AV_NOPTS_VALUE.
pub(crate) const NOPTS: i64 = i64::MIN;
/// RELATIVE_TS_BASE (avformat_internal.h).
pub(crate) const RELATIVE_TS_BASE: i64 = i64::MAX - (1 << 48);
/// END_NOT_FOUND (parser.h).
pub(crate) const END_NOT_FOUND: i64 = -100;
/// AV_PARSER_PTS_NB.
const PTS_NB: usize = 4;
/// The most one parser holds while it looks for the end of a unit. FFmpeg
/// grows without bound; a longer unit is an [`Overflow`].
pub(crate) const MAX_HELD_BYTES: usize = 32 << 20;

/// A unit grew past [`MAX_HELD_BYTES`] before its end was found. The
/// held bytes and the frame search were dropped with it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Overflow;

pub(crate) fn is_relative(ts: i64) -> bool {
    ts >= RELATIVE_TS_BASE - (1 << 48)
}

/// AVPictureType values.
pub(crate) mod pict {
    pub const NONE: i32 = 0;
    pub const I: i32 = 1;
    pub const P: i32 = 2;
    pub const B: i32 = 3;
    pub const SI: i32 = 5;
    pub const SP: i32 = 6;
}

/// The codecs this stage parses, by FFmpeg's codec id.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Codec {
    H264,
    Mpeg1Video,
    Mpeg2Video,
    Mp1,
    Mp2,
    Mp3,
    Aac,
    Ac3,
    Eac3,
    Opus,
    Lcevc,
}

impl Codec {
    pub fn name(self) -> &'static str {
        match self {
            Codec::H264 => "h264",
            Codec::Mpeg1Video => "mpeg1video",
            Codec::Mpeg2Video => "mpeg2video",
            Codec::Mp1 => "mp1",
            Codec::Mp2 => "mp2",
            Codec::Mp3 => "mp3",
            Codec::Aac => "aac",
            Codec::Ac3 => "ac3",
            Codec::Eac3 => "eac3",
            Codec::Opus => "opus",
            Codec::Lcevc => "lcevc",
        }
    }

    pub fn is_video(self) -> bool {
        matches!(
            self,
            Codec::H264 | Codec::Mpeg1Video | Codec::Mpeg2Video | Codec::Lcevc
        )
    }

    /// AV_CODEC_PROP_FIELDS (codec_desc.c).
    pub fn fields(self) -> bool {
        matches!(self, Codec::H264 | Codec::Mpeg1Video | Codec::Mpeg2Video)
    }

    /// ff_is_intra_only: AV_CODEC_PROP_INTRA_ONLY audio (AAC has no such
    /// property).
    pub fn intra_only(self) -> bool {
        matches!(
            self,
            Codec::Mp1 | Codec::Mp2 | Codec::Mp3 | Codec::Ac3 | Codec::Eac3 | Codec::Opus
        )
    }
}

/// An AVRational.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Q {
    pub num: i64,
    pub den: i64,
}

fn gcd(a: i64, b: i64) -> i64 {
    let (mut a, mut b) = (a.unsigned_abs(), b.unsigned_abs());
    while b != 0 {
        (a, b) = (b, a % b);
    }
    a as i64
}

/// av_reduce.
pub(crate) fn reduce(num: i64, den: i64, max: i64) -> Q {
    let (mut a0, mut a1) = (Q { num: 0, den: 1 }, Q { num: 1, den: 0 });
    let sign = (num < 0) ^ (den < 0);
    let (mut num, mut den) = (
        num.checked_abs().unwrap_or(i64::MAX),
        den.checked_abs().unwrap_or(i64::MAX),
    );
    let g = gcd(num, den);
    if g != 0 {
        num /= g;
        den /= g;
    }
    if num <= max && den <= max {
        a1 = Q { num, den };
        den = 0;
    }
    while den != 0 {
        let x = num / den;
        let next_den = num - den * x;
        let a2n = x.saturating_mul(a1.num).saturating_add(a0.num);
        let a2d = x.saturating_mul(a1.den).saturating_add(a0.den);
        if a2n > max || a2d > max {
            let mut x = x;
            if a1.num != 0 {
                x = (max - a0.num) / a1.num;
            }
            if a1.den != 0 {
                x = x.min((max - a0.den) / a1.den);
            }
            if (den as i128) * (2 * x as i128 * a1.den as i128 + a0.den as i128)
                > num as i128 * a1.den as i128
            {
                a1 = Q {
                    num: x * a1.num + a0.num,
                    den: x * a1.den + a0.den,
                };
            }
            break;
        }
        a0 = a1;
        a1 = Q { num: a2n, den: a2d };
        num = den;
        den = next_den;
    }
    Q {
        num: if sign { -a1.num } else { a1.num },
        den: a1.den,
    }
}

pub(crate) const INT_MAX: i64 = i32::MAX as i64;

/// av_mul_q.
pub(crate) fn mul_q(b: Q, c: Q) -> Q {
    reduce(
        b.num.saturating_mul(c.num),
        b.den.saturating_mul(c.den),
        INT_MAX,
    )
}

/// AVRounding.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Rnd {
    Down,
    NearInf,
}

/// av_rescale_rnd for the rounding modes used here.
pub(crate) fn rescale_rnd(a: i64, b: i64, c: i64, rnd: Rnd) -> i64 {
    if c <= 0 || b < 0 {
        return i64::MIN;
    }
    if a < 0 {
        // DOWN flips to UP for the magnitude; NEAR_INF stays.
        let mag = rescale_abs(
            -(a.max(-i64::MAX)) as i128,
            b,
            c,
            if rnd == Rnd::Down { c - 1 } else { c / 2 },
        );
        return mag.map_or(i64::MIN, |m| m.wrapping_neg());
    }
    rescale_abs(a as i128, b, c, if rnd == Rnd::Down { 0 } else { c / 2 }).unwrap_or(i64::MIN)
}

fn rescale_abs(a: i128, b: i64, c: i64, r: i64) -> Option<i64> {
    let v = (a * b as i128 + r as i128) / c as i128;
    i64::try_from(v).ok()
}

/// av_rescale (round to nearest).
pub(crate) fn rescale(a: i64, b: i64, c: i64) -> i64 {
    rescale_rnd(a, b, c, Rnd::NearInf)
}

/// av_rescale_q.
pub(crate) fn rescale_q(a: i64, bq: Q, cq: Q) -> i64 {
    rescale_rnd(
        a,
        bq.num.saturating_mul(cq.den),
        cq.num.saturating_mul(bq.den),
        Rnd::NearInf,
    )
}

/// av_add_stable with `inc` = 1.
pub(crate) fn add_stable(ts_tb: Q, ts: i64, inc_tb: Q) -> i64 {
    let m = inc_tb.num.saturating_mul(ts_tb.den);
    let d = inc_tb.den.saturating_mul(ts_tb.num);
    if d == 0 {
        return ts;
    }
    if m % d == 0 && ts <= i64::MAX - m / d {
        return ts + m / d;
    }
    if m < d {
        return ts;
    }
    let old = rescale_q(ts, ts_tb, inc_tb);
    let old_ts = rescale_q(old, inc_tb, ts_tb);
    if old == i64::MAX || old == NOPTS || old_ts == NOPTS {
        return ts;
    }
    rescale_q(old.wrapping_add(1), inc_tb, ts_tb).saturating_add(ts.wrapping_sub(old_ts))
}

/// The AVCodecContext fields the parsers and the timing code read and
/// write.
#[derive(Clone, Debug)]
pub(crate) struct CodecCtx {
    pub codec: Codec,
    pub sample_rate: i32,
    pub channels: i32,
    pub frame_size: i32,
    pub width: i32,
    pub height: i32,
    pub coded_width: i32,
    pub coded_height: i32,
    pub framerate: Q,
    pub has_b_frames: i32,
    /// avctx->extradata: the parameter sets find_stream_info's
    /// extract_extradata took from the first unit that had them, which
    /// every parser opened later loads.
    pub extradata: Vec<u8>,
    /// The stream's r_frame_rate (0/1 until find_stream_info settles it):
    /// a video stream's frame duration when the codec states no rate.
    pub r_frame_rate: Q,
}

impl CodecCtx {
    pub fn new(codec: Codec) -> Self {
        Self {
            codec,
            sample_rate: 0,
            channels: 0,
            frame_size: 0,
            width: 0,
            height: 0,
            coded_width: 0,
            coded_height: 0,
            framerate: Q { num: 0, den: 1 },
            has_b_frames: 0,
            extradata: Vec::new(),
            r_frame_rate: Q { num: 0, den: 1 },
        }
    }
}

/// ParseContext with ff_combine_frame.
#[derive(Debug, Default)]
pub(crate) struct ParseContext {
    buffer: Vec<u8>,
    index: usize,
    last_index: usize,
    pub state: u32,
    pub state64: u64,
    pub frame_start_found: i32,
    overread: usize,
    overread_index: usize,
}

impl ParseContext {
    pub fn new() -> Self {
        Self::default()
    }

    fn put(&mut self, at: usize, b: u8) {
        if at < self.buffer.len() {
            self.buffer[at] = b;
        } else {
            self.buffer.resize(at, 0);
            self.buffer.push(b);
        }
    }

    /// ff_combine_frame: `None` while the unit is incomplete (`buf`
    /// kept) or `next` lies past `buf`, else the whole unit.
    pub fn combine(&mut self, mut next: i64, buf: &[u8]) -> Result<Option<Vec<u8>>, Overflow> {
        while self.overread > 0 {
            let b = self.buffer.get(self.overread_index).copied().unwrap_or(0);
            self.put(self.index, b);
            self.index += 1;
            self.overread_index += 1;
            self.overread -= 1;
        }
        if next > buf.len() as i64 {
            return Ok(None);
        }
        if buf.is_empty() && next == END_NOT_FOUND {
            next = 0;
        }
        self.last_index = self.index;
        if next == END_NOT_FOUND {
            if self.index + buf.len() > MAX_HELD_BYTES {
                *self = Self::default();
                self.state = !0;
                self.state64 = !0;
                return Err(Overflow);
            }
            self.buffer.truncate(self.index);
            self.buffer.extend_from_slice(buf);
            self.index += buf.len();
            return Ok(None);
        }
        let size = (self.index as i64 + next).max(0) as usize;
        self.overread_index = size;
        let unit = if self.index > 0 {
            self.buffer.truncate(self.index);
            if next > 0 {
                self.buffer.extend_from_slice(&buf[..next as usize]);
            }
            self.index = 0;
            self.buffer[..size.min(self.buffer.len())].to_vec()
        } else {
            buf[..next.max(0) as usize].to_vec()
        };
        if next < -8 {
            self.overread += (-8 - next) as usize;
            next = -8;
        }
        while next < 0 {
            let at = self.last_index as i64 + next;
            let b = usize::try_from(at)
                .ok()
                .and_then(|i| self.buffer.get(i))
                .copied()
                .unwrap_or(0);
            self.state = self.state << 8 | u32::from(b);
            self.state64 = self.state64 << 8 | u64::from(b);
            self.overread += 1;
            next += 1;
        }
        Ok(Some(unit))
    }

    /// `&pc->buffer[pc->last_index + next]`, `-next` bytes: where a unit
    /// that ended before the buffer just offered left off.
    pub fn before_last(&self, next: i64) -> &[u8] {
        let start = (self.last_index as i64 + next).max(0) as usize;
        let end = self.last_index.min(self.buffer.len());
        self.buffer.get(start.min(end)..end).unwrap_or(&[])
    }

    /// `pc->index`: bytes held for the unit in progress.
    pub fn held(&self) -> usize {
        self.index
    }
}

/// The generic part of AVCodecParserContext: timestamp bookkeeping and
/// what the codec parser reports about the unit it returns.
#[derive(Clone, Debug)]
pub(crate) struct ParserState {
    fetched_offset: bool,
    fetch_timestamp: bool,
    pub cur_offset: i64,
    pub frame_offset: i64,
    pub next_frame_offset: i64,
    cur_frame_start_index: usize,
    cur_frame_offset: [i64; PTS_NB],
    cur_frame_end: [i64; PTS_NB],
    cur_frame_pts: [i64; PTS_NB],
    cur_frame_dts: [i64; PTS_NB],
    /// The container's random-access indicator of each remembered
    /// packet, fetched with its timestamps.
    cur_frame_rai: [bool; PTS_NB],
    pub pts: i64,
    pub dts: i64,
    /// The unit starts in a packet the container marked as a
    /// random-access point.
    pub rai: bool,
    pub key_frame: i32,
    pub pict_type: i32,
    /// AVPictureStructure of the unit returned.
    pub picture_structure: i32,
    pub repeat_pict: i32,
    pub duration: i32,
    pub width: i32,
    pub height: i32,
    pub coded_width: i32,
    pub coded_height: i32,
    pub format: Option<PixFmt>,
    pub dts_sync_point: i32,
    pub dts_ref_dts_delta: i32,
    pub pts_dts_delta: i32,
}

/// The pixel formats the video parsers report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum PixFmt {
    Yuv420p,
    Yuv422p,
    Yuv444p,
    Yuv420p9,
    Yuv422p9,
    Yuv444p9,
    Yuv420p10,
    Yuv422p10,
    Yuv444p10,
    Yuv420p12,
    Yuv422p12,
    Yuv444p12,
    Yuv420p14,
    Yuv422p14,
    Yuv444p14,
    Gray8,
    Gray10,
    Gray12,
    Gray14,
}

impl ParserState {
    /// av_parser_init.
    fn new(pict_type: i32) -> Self {
        Self {
            fetched_offset: false,
            fetch_timestamp: true,
            cur_offset: 0,
            frame_offset: 0,
            next_frame_offset: 0,
            cur_frame_start_index: 0,
            cur_frame_offset: [0; PTS_NB],
            cur_frame_end: [0; PTS_NB],
            cur_frame_pts: [0; PTS_NB],
            cur_frame_dts: [0; PTS_NB],
            cur_frame_rai: [false; PTS_NB],
            pts: 0,
            dts: 0,
            rai: false,
            key_frame: -1,
            pict_type,
            picture_structure: 0,
            repeat_pict: 0,
            duration: 0,
            width: 0,
            height: 0,
            coded_width: 0,
            coded_height: 0,
            format: None,
            dts_sync_point: i32::MIN,
            dts_ref_dts_delta: i32::MIN,
            pts_dts_delta: i32::MIN,
        }
    }

    #[cfg(test)]
    pub fn new_for_tests() -> Self {
        Self::new(pict::I)
    }

    /// ff_fetch_timestamp.
    pub fn fetch_timestamp(&mut self, off: i64, remove: bool, fuzzy: bool) {
        if !fuzzy {
            self.dts = NOPTS;
            self.pts = NOPTS;
            self.rai = false;
        }
        for i in 0..PTS_NB {
            let at = self.cur_offset.wrapping_add(off);
            if at >= self.cur_frame_offset[i]
                && (self.frame_offset < self.cur_frame_offset[i]
                    || (self.frame_offset == 0 && self.next_frame_offset == 0))
                && self.cur_frame_end[i] != 0
            {
                if !fuzzy || self.cur_frame_dts[i] != NOPTS {
                    self.dts = self.cur_frame_dts[i];
                    self.pts = self.cur_frame_pts[i];
                    self.rai = self.cur_frame_rai[i];
                }
                if remove {
                    self.cur_frame_offset[i] = i64::MAX;
                }
                if at < self.cur_frame_end[i] {
                    break;
                }
            }
        }
    }
}

enum Kind {
    H264(Box<h264::H264Parser>),
    MpegVideo(mpegvideo::MpegVideoParser),
    MpegAudio(mpegaudio::MpegAudioParser),
    AacAc3(aac_ac3::AacAc3Parser),
    Opus(opus::OpusParser),
    Lcevc(lcevc::LcevcParser),
}

/// An FFmpeg parser: codec state plus AVCodecParserContext.
pub(crate) struct Parser {
    pub state: ParserState,
    kind: Kind,
}

impl Parser {
    /// av_parser_init for `codec`.
    pub fn new(codec: Codec) -> Self {
        let (kind, pict_type) = match codec {
            Codec::H264 => (Kind::H264(Box::new(h264::H264Parser::new())), pict::I),
            // mpegvideo_parse_init: the first unit may be partial.
            Codec::Mpeg1Video | Codec::Mpeg2Video => (
                Kind::MpegVideo(mpegvideo::MpegVideoParser::new()),
                pict::NONE,
            ),
            Codec::Mp1 | Codec::Mp2 | Codec::Mp3 => {
                (Kind::MpegAudio(mpegaudio::MpegAudioParser::new()), pict::I)
            }
            Codec::Aac => (Kind::AacAc3(aac_ac3::AacAc3Parser::aac()), pict::I),
            Codec::Ac3 | Codec::Eac3 => (Kind::AacAc3(aac_ac3::AacAc3Parser::ac3()), pict::I),
            Codec::Opus => (Kind::Opus(opus::OpusParser::new()), pict::I),
            Codec::Lcevc => (Kind::Lcevc(lcevc::LcevcParser::new()), pict::I),
        };
        Self {
            state: ParserState::new(pict_type),
            kind,
        }
    }

    /// av_parser_parse2: one parser call on `buf` (empty at the end of
    /// input). Returns the bytes consumed and the unit completed, if any.
    /// `rai` is the container's random-access indicator of the packet
    /// `buf` starts. [`Overflow`] when a unit outgrew
    /// [`MAX_HELD_BYTES`]; the parser must then be replaced.
    #[allow(clippy::too_many_arguments)]
    pub fn parse2(
        &mut self,
        avctx: &mut CodecCtx,
        buf: &[u8],
        pts: i64,
        dts: i64,
        pos: i64,
        rai: bool,
    ) -> Result<(usize, Option<Vec<u8>>), Overflow> {
        let s = &mut self.state;
        if !s.fetched_offset {
            s.next_frame_offset = pos;
            s.cur_offset = pos;
            s.fetched_offset = true;
        }
        let len = buf.len() as i64;
        if !buf.is_empty()
            && s.cur_offset.wrapping_add(len) != s.cur_frame_end[s.cur_frame_start_index]
        {
            let i = (s.cur_frame_start_index + 1) & (PTS_NB - 1);
            s.cur_frame_start_index = i;
            s.cur_frame_offset[i] = s.cur_offset;
            s.cur_frame_end[i] = s.cur_offset.wrapping_add(len);
            s.cur_frame_pts[i] = pts;
            s.cur_frame_dts[i] = dts;
            s.cur_frame_rai[i] = rai;
        }
        if s.fetch_timestamp {
            s.fetch_timestamp = false;
            s.fetch_timestamp(0, false, false);
        }
        let (index, out) = match &mut self.kind {
            Kind::H264(p) => p.parse(&mut self.state, avctx, buf)?,
            Kind::MpegVideo(p) => p.parse(&mut self.state, avctx, buf)?,
            Kind::MpegAudio(p) => p.parse(&mut self.state, avctx, buf)?,
            Kind::AacAc3(p) => p.parse(&mut self.state, avctx, buf)?,
            Kind::Opus(p) => p.parse(&mut self.state, avctx, buf)?,
            Kind::Lcevc(p) => p.parse(&mut self.state, avctx, buf)?,
        };
        let s = &mut self.state;
        if avctx.codec.is_video() {
            if s.coded_width > 0 && avctx.coded_width <= 0 {
                avctx.coded_width = s.coded_width;
            }
            if s.coded_height > 0 && avctx.coded_height <= 0 {
                avctx.coded_height = s.coded_height;
            }
            if s.width > 0 && avctx.width <= 0 {
                avctx.width = s.width;
            }
            if s.height > 0 && avctx.height <= 0 {
                avctx.height = s.height;
            }
        }
        let out = out.filter(|unit| !unit.is_empty());
        if out.is_some() {
            s.frame_offset = s.next_frame_offset;
            s.next_frame_offset = s.cur_offset.wrapping_add(index);
            s.fetch_timestamp = true;
        }
        let index = index.clamp(0, len.max(0));
        s.cur_offset = s.cur_offset.wrapping_add(index);
        Ok((index as usize, out))
    }

    /// The active H.264 SPS's `(bitstream_restriction_flag,
    /// num_reorder_frames)`: what FFmpeg's decoder raises `has_b_frames`
    /// to while find_stream_info decodes.
    pub fn h264_reorder(&self) -> Option<(bool, i32)> {
        match &self.kind {
            Kind::H264(p) => p
                .active_sps
                .as_ref()
                .map(|s| (s.bitstream_restriction_flag, s.num_reorder_frames)),
            _ => None,
        }
    }

    /// The H.264 picture the last unit completes, if it decodes.
    pub fn h264_picture(&self) -> Option<h264::OutputPicture> {
        match &self.kind {
            Kind::H264(p) => p.picture,
            _ => None,
        }
    }

    /// The ADTS header of the last AAC unit returned: the rate and frame
    /// size FFmpeg's decoder reports once it has decoded it.
    pub fn last_adts(&self) -> Option<aac_ac3::AdtsHeader> {
        match &self.kind {
            Kind::AacAc3(p) => p.last_adts,
            _ => None,
        }
    }
}

/// avpriv_find_start_code over `buf[from..end]`; `end` may lie one byte
/// past `buf` (the zero padding FFmpeg's buffers carry). Returns the
/// index just past the code and leaves the last four bytes in `state`.
pub(crate) fn find_start_code(buf: &[u8], from: usize, end: usize, state: &mut u32) -> usize {
    let at = |i: usize| buf.get(i).copied().unwrap_or(0);
    let mut p = from;
    if p >= end {
        return end;
    }
    for _ in 0..3 {
        let tmp = *state << 8;
        *state = tmp.wrapping_add(u32::from(at(p)));
        p += 1;
        if tmp == 0x100 || p == end {
            return p;
        }
    }
    while p < end {
        if at(p - 1) > 1 {
            p += 3;
        } else if at(p - 2) != 0 {
            p += 2;
        } else if at(p - 3) | at(p - 1).wrapping_sub(1) != 0 {
            p += 1;
        } else {
            p += 1;
            break;
        }
    }
    let p = p.min(end) - 4;
    *state = u32::from_be_bytes([at(p), at(p + 1), at(p + 2), at(p + 3)]);
    p + 4
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rational_helpers_match_libavutil() {
        assert_eq!(reduce(1, 50, INT_MAX), Q { num: 1, den: 50 });
        assert_eq!(reduce(2, 50, INT_MAX), Q { num: 1, den: 25 });
        assert_eq!(reduce(1, 0, INT_MAX), Q { num: 1, den: 0 });
        assert_eq!(reduce(-6, 4, INT_MAX), Q { num: -3, den: 2 });
        assert_eq!(rescale_rnd(1, 2 * 90_000, 50, Rnd::Down), 3600);
        assert_eq!(rescale_rnd(1152, 90_000, 48_000, Rnd::Down), 2160);
        assert_eq!(rescale_rnd(1024, 90_000, 44_100, Rnd::Down), 2089);
        assert_eq!(rescale_rnd(-7, 1, 2, Rnd::Down), -4);
        assert_eq!(rescale(-7, 1, 2), -4);
        // Exact increments add exactly; inexact ones track the true time.
        let tb = Q {
            num: 1,
            den: 90_000,
        };
        assert_eq!(
            add_stable(
                tb,
                1000,
                Q {
                    num: 1024,
                    den: 48_000
                }
            ),
            2920
        );
        let mut t = 0;
        for _ in 0..441 {
            t = add_stable(
                tb,
                t,
                Q {
                    num: 1024,
                    den: 44_100,
                },
            );
        }
        assert_eq!(t, 921_600);
    }

    #[test]
    fn combine_holds_until_the_end_and_keeps_bytes_read_past_it() {
        let mut pc = ParseContext::new();
        assert_eq!(pc.combine(END_NOT_FOUND, b"abcd"), Ok(None));
        // The unit ended two bytes before this buffer.
        assert_eq!(pc.combine(-2, b"ef").unwrap().as_deref(), Some(&b"ab"[..]));
        assert_eq!(pc.before_last(-2), b"cd");
        // The two bytes come back in front of the next unit.
        assert_eq!(pc.combine(1, b"ef").unwrap().as_deref(), Some(&b"cde"[..]));
        // The end of input flushes what is held.
        assert_eq!(pc.combine(END_NOT_FOUND, b"gh"), Ok(None));
        assert_eq!(
            pc.combine(END_NOT_FOUND, b"").unwrap().as_deref(),
            Some(&b"gh"[..])
        );
    }
}
