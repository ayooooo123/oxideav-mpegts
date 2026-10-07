// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/lcevc_parser.c with the tables of
// libavcodec/lcevctab.c, get_mb of libavcodec/lcevc_parse.h and the LCEVC
// NAL header of libavcodec/h2645_parse.c (lcevc_parse_nal_header).
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::bits::BitReader;
use super::h264::{extract_rbsp, find_nal_start};
use super::parser::{pict, CodecCtx, Overflow, ParseContext, ParserState, PixFmt, END_NOT_FOUND};

/// start_code_prefix_one_3bytes.
const START_CODE: u32 = 0x000001;
const LCEVC_NON_IDR_NUT: u32 = 28;
const LCEVC_IDR_NUT: u32 = 29;
const PAYLOAD_TYPE_GLOBAL_CONFIG: u32 = 1;

/// ff_lcevc_resolution_type.
const RESOLUTION_TYPE: [(i32, i32); 46] = [
    (0, 0),
    (360, 200),
    (400, 240),
    (480, 320),
    (640, 360),
    (640, 480),
    (768, 480),
    (800, 600),
    (852, 480),
    (854, 480),
    (856, 480),
    (960, 540),
    (960, 640),
    (1024, 576),
    (1024, 600),
    (1024, 768),
    (1152, 864),
    (1280, 720),
    (1280, 800),
    (1280, 1024),
    (1360, 768),
    (1366, 768),
    (1920, 1200),
    (2048, 1080),
    (2048, 1152),
    (2048, 1536),
    (2160, 1440),
    (2560, 1440),
    (2560, 1600),
    (2560, 2048),
    (3200, 1800),
    (3200, 2048),
    (3200, 2400),
    (3440, 1440),
    (3840, 1600),
    (3840, 2160),
    (3840, 2400),
    (4096, 2160),
    (4096, 3072),
    (5120, 2880),
    (5120, 3200),
    (5120, 4096),
    (6400, 4096),
    (6400, 4800),
    (7680, 4320),
    (7680, 4800),
];

/// ff_lcevc_depth_type[enhancement_depth_type][chroma_format_idc].
const DEPTH_TYPE: [[PixFmt; 4]; 4] = [
    [
        PixFmt::Gray8,
        PixFmt::Yuv420p,
        PixFmt::Yuv422p,
        PixFmt::Yuv444p,
    ],
    [
        PixFmt::Gray10,
        PixFmt::Yuv420p10,
        PixFmt::Yuv422p10,
        PixFmt::Yuv444p10,
    ],
    [
        PixFmt::Gray12,
        PixFmt::Yuv420p12,
        PixFmt::Yuv422p12,
        PixFmt::Yuv444p12,
    ],
    [
        PixFmt::Gray14,
        PixFmt::Yuv420p14,
        PixFmt::Yuv422p14,
        PixFmt::Yuv444p14,
    ],
];

/// LCEVCParserContext for Annex B input (no lvcC extradata in TS).
pub(crate) struct LcevcParser {
    pc: ParseContext,
}

impl LcevcParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
        }
    }

    /// lcevc_find_frame_end: a unit ends where the next IDR or non-IDR
    /// NAL unit starts.
    fn find_frame_end(&mut self, buf: &[u8]) -> i64 {
        let pc = &mut self.pc;
        for (i, &b) in buf.iter().enumerate() {
            pc.state = (pc.state << 8) | u32::from(b);
            if (pc.state >> 8) & 0xFF_FFFF != START_CODE {
                continue;
            }
            let nut = (pc.state >> 1) & 0x1F;
            if nut == LCEVC_IDR_NUT || nut == LCEVC_NON_IDR_NUT {
                if pc.frame_start_found == 0 {
                    pc.frame_start_found = 1;
                } else {
                    pc.frame_start_found = 0;
                    return i as i64 - 3;
                }
            }
        }
        END_NOT_FOUND
    }

    /// lcevc_parse.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        _avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        let next = self.find_frame_end(buf);
        let Some(unit) = self.pc.combine(next, buf)? else {
            return Ok((buf.len() as i64, None));
        };
        parse_nal_units(s, &unit);
        Ok((next, Some(unit)))
    }
}

/// parse_nal_units over ff_h2645_packet_split's NAL units: an IDR unit
/// is a key frame; IDR and non-IDR units carry the configuration blocks.
fn parse_nal_units(s: &mut ParserState, buf: &[u8]) {
    s.pict_type = pict::NONE;
    s.key_frame = 0;
    s.picture_structure = 0;
    let mut at = 0;
    loop {
        at = find_nal_start(buf, at);
        if at >= buf.len() {
            return;
        }
        let (consumed, nal) = extract_rbsp(&buf[at..]);
        at += consumed.max(1);
        // lcevc_parse_nal_header: forbidden_zero_bit 0, forbidden_one_bit 1.
        let Some(&header) = nal.first() else { continue };
        if header & 0xC0 != 0x40 {
            continue;
        }
        match u32::from(header >> 1) & 0x1F {
            LCEVC_IDR_NUT => {
                s.key_frame = 1;
                parse_nal_unit(s, &nal);
            }
            LCEVC_NON_IDR_NUT => parse_nal_unit(s, &nal),
            _ => {}
        }
    }
}

/// get_mb: a multi-byte value, seven bits a byte, at most ten bytes.
fn get_mb(gb: &mut BitReader<'_>) -> u64 {
    let mut mb = 0u64;
    for _ in 0..10 {
        let byte = gb.read(8);
        mb = (mb << 7) | u64::from(byte & 0x7F);
        if byte & 0x80 == 0 {
            break;
        }
    }
    mb
}

/// parse_nal_unit: the payload blocks after the two-byte NAL header; the
/// global configuration states the coded size and pixel format.
fn parse_nal_unit(s: &mut ParserState, data: &[u8]) {
    let mut at = 2usize;
    while data.len().saturating_sub(at) > 1 {
        let rest = &data[at..];
        let mut gb = BitReader::new(rest);
        let payload_size_type = gb.read(3);
        let payload_type = gb.read(5);
        let payload_size = match payload_size_type {
            6 => return,
            7 => get_mb(&mut gb),
            n => u64::from(n),
        };
        let header = gb.count() >> 3;
        if payload_size > i32::MAX as u64 - header {
            return;
        }
        let block_size = payload_size + header;
        if block_size >= rest.len() as u64 {
            return;
        }
        if payload_type == PAYLOAD_TYPE_GLOBAL_CONFIG {
            global_config(s, &mut gb);
        }
        at += block_size as usize;
    }
}

/// LCEVC_PAYLOAD_TYPE_GLOBAL_CONFIG.
fn global_config(s: &mut ParserState, gb: &mut BitReader<'_>) {
    let processed_planes_type_flag = gb.read1();
    let resolution_type = gb.read(6) as usize;
    gb.skip(1);
    let chroma_format_idc = gb.read(2) as usize;
    gb.skip(2);
    let bit_depth = gb.read(2) as usize;
    s.format = Some(DEPTH_TYPE[bit_depth][chroma_format_idc]);
    if resolution_type < 63 {
        // Entries past the table are zero in FFmpeg's 63-entry array.
        let (width, height) = RESOLUTION_TYPE
            .get(resolution_type)
            .copied()
            .unwrap_or((0, 0));
        s.width = width;
        s.height = height;
        return;
    }
    let temporal_step_width_modifier_signalled_flag = gb.read1();
    gb.skip(3);
    let upsample_type = gb.read(3);
    let level1_filtering_signalled_flag = gb.read1();
    gb.skip(4);
    let tile_dimensions_type = gb.read(2);
    gb.skip(4);
    if processed_planes_type_flag {
        gb.skip(4);
    }
    if temporal_step_width_modifier_signalled_flag {
        gb.skip(8);
    }
    if upsample_type != 0 {
        gb.skip(64);
    }
    if level1_filtering_signalled_flag {
        gb.skip(8);
    }
    if tile_dimensions_type != 0 {
        if tile_dimensions_type == 3 {
            gb.skip(32);
        }
        gb.skip(8);
    }
    s.width = gb.read(16) as i32;
    s.height = gb.read(16) as i32;
}
