// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/dca_parser.c with
// avpriv_dca_convert_bitstream and ff_dca_parse_core_frame_header
// (libavcodec/dca.c), ff_dca_sample_rates (dca_sample_rate_tab.h) and the
// sync words of dca_syncwords.h.
// Copyright (C) 2004 Gildas Bazin
// Copyright (C) 2004 Benjamin Zores
// Copyright (C) 2006 Benjamin Larsson
// Copyright (C) 2007 Konstantin Shishkov
// (dca_parser.c, dca.c and dca_sample_rate_tab.h each carry these lines)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::bits::BitReader;
use super::parser::{rescale, CodecCtx, Overflow, ParseContext, ParserState, END_NOT_FOUND};

const SYNC_CORE_BE: u64 = 0x7FFE_8001;
const SYNC_CORE_LE: u64 = 0xFE7F_0180;
const SYNC_CORE_14B_BE: u64 = 0x1FFF_E800;
const SYNC_CORE_14B_LE: u64 = 0xFF1F_00E8;
const SYNC_SUBSTREAM: u64 = 0x6458_2025;

/// DCA_CORE_FRAME_HEADER_SIZE.
const CORE_FRAME_HEADER_SIZE: usize = 18;
/// DCA_PCMBLOCK_SAMPLES.
const PCMBLOCK_SAMPLES: u32 = 32;

/// ff_dca_sample_rates.
const SAMPLE_RATES: [u32; 16] = [
    0, 8000, 16000, 32000, 0, 0, 11025, 22050, 44100, 0, 0, 12000, 24000, 48000, 96000, 192000,
];
/// ff_dca_bits_per_sample.
const BITS_PER_SAMPLE: [u8; 8] = [16, 16, 20, 20, 0, 24, 24, 0];
/// Channels of the core audio modes (ff_dca_channels): what FFmpeg's
/// decoder reports for a core without extensions.
const AMODE_CHANNELS: [i32; 10] = [1, 2, 2, 2, 2, 3, 3, 4, 4, 5];

fn is_core_marker(state: u64) -> bool {
    state & 0xFFFF_FFFF_F0FF == (SYNC_CORE_14B_LE << 16) | 0xF007
        || state & 0xFFFF_FFFF_FFF0 == (SYNC_CORE_14B_BE << 16) | 0x07F0
        || state & 0xFFFF_FFFF_00FC == (SYNC_CORE_LE << 16) | 0x00FC
        || state & 0xFFFF_FFFF_FC00 == (SYNC_CORE_BE << 16) | 0xFC00
}

fn is_exss_marker(state: u64) -> bool {
    state & 0xFFFF_FFFF == SYNC_SUBSTREAM
}

fn is_marker(state: u64) -> bool {
    is_core_marker(state) || is_exss_marker(state)
}

fn core_marker(state: u64) -> u64 {
    (state >> 16) & 0xFFFF_FFFF
}

fn state_le(state: u64) -> u64 {
    ((state & 0xFF00_FF00) >> 8) | ((state & 0x00FF_00FF) << 8)
}

fn state_14(state: u64) -> u64 {
    ((state & 0x3FFF_0000) >> 8) | ((state & 0x0000_3FFF) >> 6)
}

fn core_framesize(state: u64) -> i64 {
    ((state >> 4) & 0x3FFF) as i64 + 1
}

fn exss_framesize(state: u64) -> i64 {
    if state & 0x20_0000_0000 != 0 {
        ((state >> 5) & 0xF_FFFF) as i64 + 1
    } else {
        ((state >> 13) & 0xFFFF) as i64 + 1
    }
}

/// DCAParseContext.
pub(crate) struct DcaParser {
    pc: ParseContext,
    lastmarker: u64,
    size: i64,
    framesize: i64,
    startpos: usize,
}

impl DcaParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            lastmarker: 0,
            size: 0,
            framesize: 0,
            startpos: 0,
        }
    }

    /// dca_find_frame_end: a core frame, with the extension substream
    /// after it, or a substream frame alone.
    fn find_frame_end(&mut self, buf: &[u8]) -> i64 {
        let mut start_found = self.pc.frame_start_found;
        let mut state = self.pc.state64;
        let mut size = self.size;
        let mut i = 0usize;
        if start_found == 0 {
            while i < buf.len() {
                size += 1;
                state = (state << 8) | u64::from(buf[i]);
                if is_marker(state)
                    && (self.lastmarker == 0
                        || self.lastmarker == core_marker(state)
                        || self.lastmarker == SYNC_SUBSTREAM)
                {
                    if self.lastmarker == 0 {
                        let back = if is_exss_marker(state) { 4 } else { 6 };
                        self.startpos = (size - back).max(0) as usize;
                    }
                    self.lastmarker = if is_exss_marker(state) {
                        state & 0xFFFF_FFFF
                    } else {
                        core_marker(state)
                    };
                    start_found = 1;
                    size = 0;
                    i += 1;
                    break;
                }
                i += 1;
            }
        }
        if start_found != 0 {
            while i < buf.len() {
                size += 1;
                state = (state << 8) | u64::from(buf[i]);
                if start_found == 1 {
                    match self.lastmarker {
                        SYNC_CORE_BE if size == 2 => {
                            self.framesize = core_framesize(state);
                            start_found = 2;
                        }
                        SYNC_CORE_LE if size == 2 => {
                            self.framesize = core_framesize(state_le(state));
                            start_found = 4;
                        }
                        SYNC_CORE_14B_BE if size == 4 => {
                            self.framesize = core_framesize(state_14(state));
                            start_found = 4;
                        }
                        SYNC_CORE_14B_LE if size == 4 => {
                            self.framesize = core_framesize(state_14(state_le(state)));
                            start_found = 4;
                        }
                        SYNC_SUBSTREAM if size == 6 => {
                            self.framesize = exss_framesize(state);
                            start_found = 4;
                        }
                        _ => {}
                    }
                    i += 1;
                    continue;
                }
                if start_found == 2 && is_exss_marker(state) && self.framesize <= size + 2 {
                    self.framesize = size + 2;
                    start_found = 3;
                    i += 1;
                    continue;
                }
                if start_found == 3 {
                    if size == self.framesize + 4 {
                        self.framesize += exss_framesize(state);
                        start_found = 4;
                    }
                    i += 1;
                    continue;
                }
                if self.framesize > size {
                    i += 1;
                    continue;
                }
                if is_marker(state)
                    && (self.lastmarker == core_marker(state) || self.lastmarker == SYNC_SUBSTREAM)
                {
                    self.pc.frame_start_found = 0;
                    self.pc.state64 = u64::MAX;
                    self.size = 0;
                    return if is_exss_marker(state) {
                        i as i64 - 3
                    } else {
                        i as i64 - 5
                    };
                }
                i += 1;
            }
        }
        self.pc.frame_start_found = start_found;
        self.pc.state64 = state;
        self.size = size;
        END_NOT_FOUND
    }

    /// dca_parse: one frame, its duration and the stream's sample rate.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        let next = self.find_frame_end(buf);
        let Some(mut unit) = self.pc.combine(next, buf)? else {
            return Ok((buf.len() as i64, None));
        };
        // Skip the initial padding.
        if unit.len() > self.startpos {
            unit.drain(..self.startpos);
        }
        self.startpos = 0;
        match parse_params(&unit) {
            Some(params) => {
                if avctx.sample_rate == 0 {
                    avctx.sample_rate = params.sample_rate as i32;
                }
                s.duration = rescale(
                    i64::from(params.duration),
                    i64::from(avctx.sample_rate),
                    i64::from(params.sample_rate),
                ) as i32;
                if let Some(channels) = params.channels {
                    avctx.channels = channels;
                }
            }
            None => s.duration = 0,
        }
        Ok((next, Some(unit)))
    }
}

struct FrameParams {
    duration: u32,
    sample_rate: u32,
    /// The core's channels, for a frame with no extension.
    channels: Option<i32>,
}

/// dca_parse_params for a frame that starts with a core header. A frame
/// that starts with an extension substream needs ff_dca_exss_parse, not
/// ported: no duration.
fn parse_params(buf: &[u8]) -> Option<FrameParams> {
    if buf.len() < CORE_FRAME_HEADER_SIZE {
        return None;
    }
    let hdr = convert_bitstream(&buf[..CORE_FRAME_HEADER_SIZE])?;
    let h = parse_core_frame_header(&hdr)?;
    let sample_rate = SAMPLE_RATES[h.sr_code];
    // An extension substream after the core (DTS-HD) carries channels
    // the core does not state.
    let frame_size = (h.frame_size + 3) & !3;
    let extended =
        buf.len() >= frame_size + 4 && buf[frame_size..frame_size + 4] == [0x64, 0x58, 0x20, 0x25];
    let channels = (!extended && !h.ext_audio_present)
        .then(|| AMODE_CHANNELS[h.audio_mode] + i32::from(h.lfe_present != 0));
    Some(FrameParams {
        duration: h.npcmblocks * PCMBLOCK_SAMPLES,
        sample_rate,
        channels,
    })
}

/// avpriv_dca_convert_bitstream: the header in 16-bit big-endian words.
fn convert_bitstream(src: &[u8]) -> Option<Vec<u8>> {
    let mrk = u64::from(u32::from_be_bytes(src.get(..4)?.try_into().ok()?));
    match mrk {
        SYNC_CORE_BE | SYNC_SUBSTREAM => Some(src.to_vec()),
        SYNC_CORE_LE => {
            let mut dst = Vec::with_capacity(src.len() + 1);
            for pair in src.chunks(2) {
                dst.push(pair.get(1).copied().unwrap_or(0));
                dst.push(pair[0]);
            }
            dst.truncate(src.len());
            Some(dst)
        }
        SYNC_CORE_14B_BE | SYNC_CORE_14B_LE => {
            // put_bits of 14 bits per 16-bit word, flushed.
            let mut dst = Vec::with_capacity(src.len());
            let (mut acc, mut bits) = (0u32, 0u32);
            for pair in src.chunks(2) {
                let (a, b) = (
                    u32::from(pair[0]),
                    u32::from(pair.get(1).copied().unwrap_or(0)),
                );
                let word = if mrk == SYNC_CORE_14B_BE {
                    (a << 8) | b
                } else {
                    (b << 8) | a
                };
                acc = (acc << 14) | (word & 0x3FFF);
                bits += 14;
                while bits >= 8 {
                    bits -= 8;
                    dst.push((acc >> bits) as u8);
                }
                acc &= (1 << bits) - 1;
            }
            if bits > 0 {
                dst.push((acc << (8 - bits)) as u8);
            }
            Some(dst)
        }
        _ => None,
    }
}

struct CoreHeader {
    npcmblocks: u32,
    frame_size: usize,
    audio_mode: usize,
    sr_code: usize,
    ext_audio_present: bool,
    lfe_present: u32,
}

/// ff_dca_parse_core_frame_header.
fn parse_core_frame_header(hdr: &[u8]) -> Option<CoreHeader> {
    let mut gb = BitReader::new(hdr);
    let sync = (u64::from(gb.read(16)) << 16) | u64::from(gb.read(16));
    if sync != SYNC_CORE_BE {
        return None;
    }
    gb.skip(1); // normal_frame
    if gb.read(5) + 1 != PCMBLOCK_SAMPLES {
        return None;
    }
    let crc_present = gb.read1();
    let npcmblocks = gb.read(7) + 1;
    if npcmblocks & 7 != 0 {
        return None;
    }
    let frame_size = gb.read(14) as usize + 1;
    if frame_size < 96 {
        return None;
    }
    let audio_mode = gb.read(6) as usize;
    if audio_mode >= AMODE_CHANNELS.len() {
        return None;
    }
    let sr_code = gb.read(4) as usize;
    if SAMPLE_RATES[sr_code] == 0 {
        return None;
    }
    gb.skip(5); // br_code
    if gb.read1() {
        return None;
    }
    gb.skip(4); // drc, ts, aux, hdcd
    gb.skip(3); // ext_audio_type
    let ext_audio_present = gb.read1();
    gb.skip(1); // sync_ssf
    let lfe_present = gb.read(2);
    if lfe_present == 3 {
        return None;
    }
    gb.skip(1); // predictor_history
    if crc_present {
        gb.skip(16);
    }
    gb.skip(1 + 4 + 2); // filter_perfect, encoder_rev, copy_hist
    if BITS_PER_SAMPLE[gb.read(3) as usize] == 0 {
        return None;
    }
    Some(CoreHeader {
        npcmblocks,
        frame_size,
        audio_mode,
        sr_code,
        ext_audio_present,
        lfe_present,
    })
}
