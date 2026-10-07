// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/opus/parser.c with ff_opus_parse_packet
// and the channel and stream counts of ff_opus_parse_extradata
// (libavcodec/opus/parse.c), and ff_opus_frame_duration
// (libavcodec/opus/frame_duration_tab.c).
// Copyright (c) 2013-2014 Mozilla Corporation (parser.c, parse.c)
// Copyright (c) 2012 Andrew D'Addesio (parse.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::parser::{CodecCtx, Overflow, ParseContext, ParserState, END_NOT_FOUND};

/// The 11-bit control_header_prefix that starts each Opus access unit in
/// MPEG-TS (FFmpeg's OPUS_TS_HEADER / OPUS_TS_MASK).
const OPUS_TS_HEADER: u32 = 0x7FE0;
const OPUS_TS_MASK: u32 = 0xFFE0;
const OPUS_MAX_FRAME_SIZE: usize = 1275;
const OPUS_MAX_FRAMES: usize = 48;
const OPUS_MAX_PACKET_DUR: u32 = 5760;

/// ff_opus_frame_duration, in 48 kHz samples, by TOC configuration.
const FRAME_DURATION: [u32; 32] = [
    480, 960, 1920, 2880, 480, 960, 1920, 2880, 480, 960, 1920, 2880, 480, 960, 480, 960, 120, 240,
    480, 960, 120, 240, 480, 960, 120, 240, 480, 960, 120, 240, 480, 960,
];

/// OpusParserContext.
pub(crate) struct OpusParser {
    pc: ParseContext,
    extradata_parsed: bool,
    ts_framing: bool,
    /// OpusParseContext.nb_streams: a multistream packet's first stream
    /// is self-delimited.
    nb_streams: u32,
}

/// The parse failed: FFmpeg's `goto fail` / AVERROR_INVALIDDATA.
struct Invalid;

impl OpusParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            extradata_parsed: false,
            ts_framing: false,
            nb_streams: 1,
        }
    }

    /// set_frame_duration: the packet's frames times their duration.
    fn set_frame_duration(&self, s: &mut ParserState, buf: &[u8]) -> Result<(), Invalid> {
        let (count, duration) = parse_packet(buf, self.nb_streams > 1).ok_or(Invalid)?;
        s.duration = (count * duration) as i32;
        Ok(())
    }

    /// opus_find_frame_end: the end of the unit and the length of its TS
    /// control header.
    fn find_frame_end(&mut self, s: &mut ParserState, buf: &[u8]) -> Result<(i64, usize), Invalid> {
        let mut header_len = 0;
        if buf.is_empty() {
            return Ok((0, 0));
        }
        let mut start_found = self.pc.frame_start_found != 0;
        let mut state = self.pc.state;
        let mut payload: Option<usize> = Some(0);
        let mut payload_len = 0usize;
        if !self.ts_framing && buf.len() > 2 {
            let hdr = u32::from(u16::from_be_bytes([buf[0], buf[1]]));
            if hdr & OPUS_TS_MASK == OPUS_TS_HEADER {
                self.ts_framing = true;
            }
        }
        if self.ts_framing && !start_found {
            for i in 0..buf.len().saturating_sub(2) {
                state = (state << 8) | u32::from(buf[i]);
                if state & OPUS_TS_MASK == OPUS_TS_HEADER {
                    // The header is read from the start of the buffer, as
                    // FFmpeg reads it.
                    let Some((at, len)) = ts_header(buf, buf.len() - i) else {
                        return Err(Invalid);
                    };
                    payload = Some(at);
                    payload_len = len;
                    header_len = at;
                    start_found = true;
                    break;
                }
            }
        }
        if !self.ts_framing {
            payload_len = buf.len();
        }
        if payload_len <= buf.len() && (!self.ts_framing || start_found) {
            let from = payload.unwrap_or(0);
            let unit = buf.get(from..from + payload_len).unwrap_or(&[]);
            if self.set_frame_duration(s, unit).is_err() {
                self.pc.frame_start_found = 0;
                return Err(Invalid);
            }
        }
        if self.ts_framing {
            if start_found && payload_len + header_len <= buf.len() {
                self.pc.frame_start_found = 0;
                self.pc.state = u32::MAX;
                return Ok(((payload_len + header_len) as i64, header_len));
            }
            self.pc.frame_start_found = i32::from(start_found);
            self.pc.state = state;
            return Ok((END_NOT_FOUND, header_len));
        }
        Ok((buf.len() as i64, header_len))
    }

    /// opus_parse: one access unit without its TS control header.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        let fail = Ok((buf.len() as i64, None));
        avctx.sample_rate = 48_000;
        if !self.extradata_parsed {
            if avctx.extradata.is_empty() {
                // What the decoder reports without extradata
                // (ff_opus_parse_extradata on opus_default_extradata).
                if avctx.channels == 0 {
                    avctx.channels = 2;
                }
            } else {
                let Some((channels, streams)) = extradata_counts(&avctx.extradata) else {
                    return fail;
                };
                avctx.channels = channels;
                self.nb_streams = streams;
                self.extradata_parsed = true;
            }
        }
        let Ok((next, header_len)) = self.find_frame_end(s, buf) else {
            return fail;
        };
        if !self.ts_framing {
            return Ok((next, Some(buf.to_vec())));
        }
        let Some(unit) = self.pc.combine(next, buf)? else {
            return fail;
        };
        Ok((next, Some(unit.get(header_len..).unwrap_or(&[]).to_vec())))
    }
}

/// parse_opus_ts_header on `buf`, the control header at its start, with
/// `buf_len` bytes readable after the prefix byte: where the payload
/// starts and its au_size.
fn ts_header(buf: &[u8], buf_len: usize) -> Option<(usize, usize)> {
    let body = buf.get(1..1 + buf_len.min(buf.len() - 1))?;
    let mut at = 0usize;
    // bytestream2 reads past the end as zero.
    let byte = |at: usize| body.get(at).copied().unwrap_or(0);
    let flags = byte(at);
    at += 1;
    let start_trim = (flags >> 4) & 1 != 0;
    let end_trim = (flags >> 3) & 1 != 0;
    let control_extension = (flags >> 2) & 1 != 0;
    let mut payload_len = 0usize;
    while at < body.len() && byte(at) == 0xFF {
        payload_len += 0xFF;
        at += 1;
    }
    payload_len += usize::from(byte(at));
    at = (at + 1).min(body.len());
    if start_trim {
        at = (at + 2).min(body.len());
    }
    if end_trim {
        at = (at + 2).min(body.len());
    }
    if control_extension {
        let len = usize::from(byte(at));
        at = (at + 1 + len).min(body.len());
    }
    if at + payload_len > buf_len {
        return None;
    }
    Some((1 + at, payload_len))
}

/// The channel count and stream count of an OpusHead, as
/// ff_opus_parse_extradata takes them; `None` where it fails.
fn extradata_counts(extradata: &[u8]) -> Option<(i32, u32)> {
    if extradata.len() < 19 || extradata[8] > 15 {
        return None;
    }
    let channels = extradata[9];
    if channels == 0 {
        return None;
    }
    match extradata[18] {
        0 if channels <= 2 => Some((i32::from(channels), 1)),
        1 | 2 | 255 => {
            if extradata.len() < 21 + usize::from(channels) {
                return None;
            }
            let (streams, stereo) = (extradata[19], extradata[20]);
            if streams == 0 || stereo > streams || u32::from(streams) + u32::from(stereo) > 255 {
                return None;
            }
            Some((i32::from(channels), u32::from(streams)))
        }
        _ => None,
    }
}

/// xiph_lacing_16bit: a one- or two-byte frame length.
fn lacing_16bit(buf: &[u8], at: &mut usize, end: usize) -> Option<usize> {
    if *at >= end {
        return None;
    }
    let mut val = usize::from(buf[*at]);
    *at += 1;
    if val >= 252 {
        if *at >= end {
            return None;
        }
        val += 4 * usize::from(buf[*at]);
        *at += 1;
    }
    Some(val)
}

/// xiph_lacing_full: a code 3 padding length.
fn lacing_full(buf: &[u8], at: &mut usize, end: usize) -> Option<usize> {
    let mut val = 0usize;
    loop {
        if *at >= end || val > i32::MAX as usize - 254 {
            return None;
        }
        let next = usize::from(buf[*at]);
        *at += 1;
        val += next;
        if next < 255 {
            return Some(val);
        }
        val -= 1;
    }
}

/// ff_opus_parse_packet: the frame count and the frame duration (48 kHz
/// samples), `None` for a packet it rejects.
fn parse_packet(buf: &[u8], self_delimiting: bool) -> Option<(u32, u32)> {
    let mut end = buf.len();
    let toc = *buf.first()?;
    let mut at = 1usize;
    let code = toc & 0x3;
    let config = usize::from(toc >> 3);
    if code >= 2 && buf.len() < 2 {
        return None;
    }
    let frame_count: usize;
    match code {
        0 => {
            frame_count = 1;
            if self_delimiting {
                let len = lacing_16bit(buf, &mut at, end)?;
                if len > end - at {
                    return None;
                }
                end = at + len;
            }
            if end - at > OPUS_MAX_FRAME_SIZE {
                return None;
            }
        }
        1 => {
            frame_count = 2;
            if self_delimiting {
                let len = lacing_16bit(buf, &mut at, end)?;
                if 2 * len > end - at {
                    return None;
                }
                end = at + 2 * len;
            }
            let frame_bytes = end - at;
            if frame_bytes & 1 != 0 || frame_bytes >> 1 > OPUS_MAX_FRAME_SIZE {
                return None;
            }
        }
        2 => {
            frame_count = 2;
            let first = lacing_16bit(buf, &mut at, end)?;
            if self_delimiting {
                let len = lacing_16bit(buf, &mut at, end)?;
                if len + first > end - at {
                    return None;
                }
                end = at + first + len;
            }
            let second = (end - at).checked_sub(first)?;
            if second > OPUS_MAX_FRAME_SIZE {
                return None;
            }
        }
        _ => {
            let i = *buf.get(at)?;
            at += 1;
            frame_count = usize::from(i & 0x3F);
            let has_padding = (i >> 6) & 1 != 0;
            let vbr = (i >> 7) & 1 != 0;
            if frame_count == 0 || frame_count > OPUS_MAX_FRAMES {
                return None;
            }
            let padding = if has_padding {
                lacing_full(buf, &mut at, end)?
            } else {
                0
            };
            if vbr {
                let mut total = 0usize;
                for _ in 0..frame_count - 1 {
                    total += lacing_16bit(buf, &mut at, end)?;
                }
                if self_delimiting {
                    let len = lacing_16bit(buf, &mut at, end)?;
                    if len + total + padding > end - at {
                        return None;
                    }
                    end = at + total + len + padding;
                }
                let frame_bytes = (end - at).checked_sub(padding)?;
                if total > frame_bytes {
                    return None;
                }
            } else if self_delimiting {
                let frame_bytes = lacing_16bit(buf, &mut at, end)?;
                if frame_count * frame_bytes + padding > end - at {
                    return None;
                }
            } else {
                let frame_bytes = (end - at).checked_sub(padding)?;
                if frame_bytes % frame_count != 0 || frame_bytes / frame_count > OPUS_MAX_FRAME_SIZE
                {
                    return None;
                }
            }
        }
    }
    let duration = FRAME_DURATION[config];
    if duration * frame_count as u32 > OPUS_MAX_PACKET_DUR {
        return None;
    }
    Some((frame_count as u32, duration))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_durations_follow_the_toc() {
        // Config 31 (CELT 20 ms), code 0: one 960-sample frame.
        assert_eq!(parse_packet(&[31 << 3, 1, 2, 3], false), Some((1, 960)));
        // Code 1 needs an even payload: two frames.
        assert_eq!(parse_packet(&[(31 << 3) | 1, 1, 2], false), Some((2, 960)));
        assert_eq!(parse_packet(&[(31 << 3) | 1, 1, 2, 3], false), None);
        // Code 3, CBR, three 10 ms SILK frames (config 0 = 480).
        assert_eq!(parse_packet(&[3, 3, 0, 0, 0], false), Some((3, 480)));
        // Seven 20 ms frames is 140 ms: over the 120 ms bound.
        assert_eq!(
            parse_packet(&[(31 << 3) | 3, 7, 0, 0, 0, 0, 0, 0, 0], false),
            None
        );
        assert_eq!(parse_packet(&[], false), None);
    }
}
