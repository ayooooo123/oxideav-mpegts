// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/latm_parser.c, with the part of the
// LATM decoder's read_stream_mux_config (libavcodec/aac/aacdec_latm.h) and
// of ff_mpeg4audio_get_config_gb (libavcodec/mpeg4audio.c) that decides
// the rate, channels and frame size FFmpeg's decoder reports, and
// ff_mpeg4audio_sample_rates (mpeg4audio_sample_rates.h).
// copyright (c) 2008 Paul Kendall <paul@kcbbs.gen.nz> (latm_parser.c)
// Copyright (c) 2005-2006 Oded Shimon ( ods15 ods15 dyndns org ) (aacdec_latm.h)
// Copyright (c) 2006-2007 Maxim Gavrilov ( maxim.gavrilov gmail com ) (aacdec_latm.h)
// Copyright (c) 2008-2013 Alex Converse <alex.converse@gmail.com> (aacdec_latm.h)
// Copyright (c) 2008-2010 Paul Kendall <paul@kcbbs.gen.nz> (aacdec_latm.h)
// Copyright (c) 2010      Janne Grunau <janne-libav@jannau.net> (aacdec_latm.h)
// Copyright (c) 2013 MIPS Technologies, Inc., California. (aacdec_latm.h)
// Copyright (c) 2008 Baptiste Coudurier <baptiste.coudurier@free.fr> (mpeg4audio.c, mpeg4audio_sample_rates.h)
// Copyright (c) 2009 Alex Converse <alex.converse@gmail.com> (mpeg4audio.c, mpeg4audio_sample_rates.h)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::aac_ac3::MPEG4AUDIO_CHANNELS;
use super::bits::BitReader;
use super::parser::{CodecCtx, Overflow, ParseContext, ParserState, END_NOT_FOUND};

/// LATM_HEADER / LATM_MASK / LATM_SIZE_MASK: the 11-bit LOAS sync word
/// and the 13-bit length after it.
const LATM_HEADER: u32 = 0x56_E000;
const LATM_MASK: u32 = 0xFF_E000;
const LATM_SIZE_MASK: u32 = 0x00_1FFF;

/// ff_mpeg4audio_sample_rates.
const SAMPLE_RATES: [u32; 16] = [
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350, 0, 0,
    0,
];

const AOT_AAC_MAIN: u32 = 1;
const AOT_AAC_LC: u32 = 2;
const AOT_AAC_SSR: u32 = 3;
const AOT_AAC_LTP: u32 = 4;
const AOT_SBR: u32 = 5;
const AOT_ER_AAC_LC: u32 = 17;
const AOT_ER_AAC_LD: u32 = 23;
const AOT_ER_BSAC: u32 = 22;
const AOT_PS: u32 = 29;
const AOT_ESCAPE: u32 = 31;

/// What FFmpeg's LATM decoder reports once it has decoded a frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct LatmConfig {
    pub sample_rate: i32,
    /// 0 when a program config element states them (not read here).
    pub channels: i32,
    pub frame_size: i32,
}

/// LATMParseContext.
pub(crate) struct LatmParser {
    pc: ParseContext,
    count: i64,
    /// The configuration of the last unit that carried one.
    pub config: Option<LatmConfig>,
}

impl LatmParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            count: 0,
            config: None,
        }
    }

    /// latm_find_frame_end.
    fn find_frame_end(&mut self, buf: &[u8]) -> i64 {
        let mut pic_found = self.pc.frame_start_found != 0;
        let mut state = self.pc.state;
        if !pic_found {
            for (i, &b) in buf.iter().enumerate() {
                state = (state << 8) | u32::from(b);
                if state & LATM_MASK == LATM_HEADER {
                    self.count = -(i as i64 + 1);
                    pic_found = true;
                    break;
                }
            }
        }
        if pic_found {
            // EOF is the end of a unit.
            if buf.is_empty() {
                return 0;
            }
            let size = i64::from(state & LATM_SIZE_MASK);
            if size - self.count <= buf.len() as i64 {
                self.pc.frame_start_found = 0;
                self.pc.state = u32::MAX;
                return size - self.count;
            }
        }
        self.count += buf.len() as i64;
        self.pc.frame_start_found = i32::from(pic_found);
        self.pc.state = state;
        END_NOT_FOUND
    }

    /// latm_parse.
    pub fn parse(
        &mut self,
        _s: &mut ParserState,
        _avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        let next = self.find_frame_end(buf);
        let Some(unit) = self.pc.combine(next, buf)? else {
            return Ok((buf.len() as i64, None));
        };
        if let Some(config) = unit_config(&unit) {
            self.config = Some(config);
        }
        Ok((next, Some(unit)))
    }
}

/// latm_get_value.
fn latm_get_value(gb: &mut BitReader<'_>) -> u64 {
    let length = gb.read(2) + 1;
    let mut v = 0u64;
    for _ in 0..length {
        v = (v << 8) | u64::from(gb.read(8));
    }
    v
}

/// get_object_type.
fn object_type(gb: &mut BitReader<'_>) -> u32 {
    let aot = gb.read(5);
    if aot == AOT_ESCAPE {
        32 + gb.read(6)
    } else {
        aot
    }
}

/// get_sample_rate.
fn sample_rate(gb: &mut BitReader<'_>) -> u32 {
    let index = gb.read(4) as usize;
    if index == 0x0F {
        gb.read(24)
    } else {
        SAMPLE_RATES[index]
    }
}

/// The configuration a LOAS unit's AudioMuxElement states, when it has a
/// StreamMuxConfig (useSameStreamMux 0) FFmpeg's decoder accepts.
fn unit_config(unit: &[u8]) -> Option<LatmConfig> {
    let mut gb = BitReader::new(unit);
    if gb.read(11) != 0x2B7 {
        return None;
    }
    gb.skip(13);
    if gb.read1() {
        return None; // useSameStreamMux
    }
    // read_stream_mux_config.
    let audio_mux_version = gb.read1();
    if audio_mux_version && gb.read1() {
        return None; // audioMuxVersionA: no configuration read
    }
    if audio_mux_version {
        latm_get_value(&mut gb); // taraFullness
    }
    gb.skip(1 + 6); // allStreamSameTimeFraming, numSubFrames
    if gb.read(4) != 0 || gb.read(3) != 0 {
        return None; // several programs or layers
    }
    if audio_mux_version {
        // latm_decode_audio_specific_config with ascLen: the sync
        // extension is looked for within those bits only.
        let asc_len = latm_get_value(&mut gb);
        let at = gb.count();
        let mut asc =
            BitReader::with_bits(unit, at.saturating_add(asc_len).min(unit.len() as u64 * 8));
        asc.skip(at);
        return audio_specific_config(&mut asc, asc_len > 0);
    }
    audio_specific_config(&mut gb, false)
}

/// ff_mpeg4audio_get_config_gb, and the GASpecificConfig frame length
/// flag; the reported rate and frame size follow aac_decode_frame_int.
fn audio_specific_config(gb: &mut BitReader<'_>, sync_extension: bool) -> Option<LatmConfig> {
    let mut aot = object_type(gb);
    let core_rate = sample_rate(gb);
    let chan_config = gb.read(4) as usize;
    let mut channels = *MPEG4AUDIO_CHANNELS.get(chan_config)?;
    let mut sbr = -1;
    let mut ps = -1;
    let mut ext_rate = 0;
    if aot == AOT_SBR || (aot == AOT_PS && !(gb.peek(3) & 0x03 != 0 && gb.peek(9) & 0x3F == 0)) {
        if aot == AOT_PS {
            ps = 1;
        }
        sbr = 1;
        ext_rate = sample_rate(gb);
        aot = object_type(gb);
        if aot == AOT_ER_BSAC {
            gb.skip(4);
        }
    }
    let mut frame_length_short = false;
    if matches!(
        aot,
        AOT_AAC_MAIN | AOT_AAC_LC | AOT_AAC_SSR | AOT_AAC_LTP | AOT_ER_AAC_LC | AOT_ER_AAC_LD
    ) {
        // decode_ga_specific_config: frameLengthFlag first. The sync
        // extension search below starts at the same bit.
        frame_length_short = gb.peek(1) != 0;
    }
    if sync_extension {
        // The sync extension (0x2B7) after the specific config.
        while gb.left() > 15 {
            if gb.peek(11) == 0x2B7 {
                gb.skip(11);
                let ext_aot = object_type(gb);
                if ext_aot == AOT_SBR {
                    sbr = i32::from(gb.read1());
                    if sbr == 1 {
                        ext_rate = sample_rate(gb);
                        if ext_rate == core_rate {
                            sbr = -1;
                        }
                    }
                }
                if gb.left() > 11 && gb.read(11) == 0x548 {
                    ps = i32::from(gb.read1());
                }
                break;
            }
            gb.skip(1);
        }
    }
    if sbr == 0 {
        ps = 0;
    }
    if (ps == -1 && aot != AOT_AAC_LC) || channels & !1 != 0 {
        ps = 0;
    }
    if ps == 1 && channels == 1 {
        channels = 2;
    }
    if core_rate == 0 {
        return None;
    }
    let multiplier = u32::from(sbr == 1 && ext_rate > core_rate);
    Some(LatmConfig {
        sample_rate: (core_rate << multiplier) as i32,
        channels,
        frame_size: (if frame_length_short { 960 } else { 1024 }) << multiplier,
    })
}
