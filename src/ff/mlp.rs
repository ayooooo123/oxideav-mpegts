// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/mlp_parser.c with ff_mlp_read_major_sync
// (libavcodec/mlp_parse.c), mlp_samplerate and truehd_channels
// (libavcodec/mlp_parse.h) and ff_mlp_checksum16 (libavcodec/mlp.c).
// Copyright (c) 2007 Ian Caulfield (mlp_parser.c, mlp_parse.c, mlp_parse.h)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::bits::BitReader;
use super::parser::{CodecCtx, Overflow, ParseContext, ParserState, END_NOT_FOUND};

/// mlp_quants.
const QUANTS: [i32; 16] = [16, 20, 24, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
/// mlp_channels.
const MLP_CHANNELS: [i32; 32] = [
    1, 2, 3, 4, 3, 4, 5, 3, 4, 5, 4, 5, 6, 4, 5, 4, 5, 6, 5, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];
/// thd_chancount.
const THD_CHANCOUNT: [i32; 13] = [2, 1, 1, 2, 2, 2, 2, 1, 1, 2, 2, 1, 1];

/// mlp_samplerate.
fn samplerate(code: u32) -> i32 {
    if code == 0xF {
        return 0;
    }
    (if code & 8 != 0 { 44_100 } else { 48_000 }) << (code & 7)
}

/// truehd_channels.
fn truehd_channels(chanmap: u32) -> i32 {
    (0..13)
        .map(|i| THD_CHANCOUNT[i] * ((chanmap >> i) & 1) as i32)
        .sum()
}

/// av_crc on crc_2D (16 bits, polynomial 0x2D, MSB first) from 0.
fn crc16_2d(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x2D
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// What ff_mlp_read_major_sync reports and the parser uses.
struct MajorSync {
    sample_rate: i32,
    access_unit_size: i32,
    channels: i32,
    num_substreams: i32,
}

/// mlp_get_major_sync_size.
fn major_sync_size(buf: &[u8]) -> Option<usize> {
    if buf.len() < 28 {
        return None;
    }
    let mut size = 28;
    if buf[..4] == [0xF8, 0x72, 0x6F, 0xBA] && buf[25] & 1 != 0 {
        size += 2 + usize::from(buf[26] >> 4) * 2;
    }
    Some(size)
}

/// ff_mlp_read_major_sync on `buf`, which starts at the sync words.
fn read_major_sync(buf: &[u8]) -> Option<MajorSync> {
    let header_size = major_sync_size(buf)?;
    if buf.len() < header_size {
        return None;
    }
    // ff_mlp_checksum16 over header_size - 2 bytes against the last two.
    // av_crc keeps a big-endian CRC byte-swapped and FFmpeg reads the
    // stored words little-endian: the same as big-endian on both sides.
    let crc = crc16_2d(&buf[..header_size - 4])
        ^ u16::from_be_bytes([buf[header_size - 4], buf[header_size - 3]]);
    if crc != u16::from_be_bytes([buf[header_size - 2], buf[header_size - 1]]) {
        return None;
    }
    let mut gb = BitReader::new(&buf[..header_size]);
    if gb.read(24) != 0xF8_726F {
        return None;
    }
    let stream_type = gb.read(8);
    let ratebits;
    let channels;
    match stream_type {
        0xBB => {
            let _group1_bits = QUANTS[gb.read(4) as usize];
            gb.skip(4);
            ratebits = gb.read(4);
            gb.skip(4);
            gb.skip(11);
            channels = MLP_CHANNELS[gb.read(5) as usize];
        }
        0xBA => {
            ratebits = gb.read(4);
            gb.skip(4);
            gb.skip(2 + 2);
            let stream1 = truehd_channels(gb.read(5));
            gb.skip(2);
            let stream2 = truehd_channels(gb.read(13));
            channels = if stream2 == 0 { stream1 } else { stream2 };
        }
        _ => return None,
    }
    gb.skip(48);
    gb.skip(1 + 15); // is_vbr, peak_bitrate
    let num_substreams = gb.read(4) as i32;
    Some(MajorSync {
        sample_rate: samplerate(ratebits),
        access_unit_size: 40 << (ratebits & 7),
        channels,
        num_substreams,
    })
}

/// MLPParseContext.
pub(crate) struct MlpParser {
    pc: ParseContext,
    bytes_left: i64,
    in_sync: bool,
    num_substreams: i32,
}

impl MlpParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            bytes_left: 0,
            in_sync: false,
            num_substreams: 0,
        }
    }

    /// mlp_parse.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        s.key_frame = 0;
        if buf.is_empty() {
            return Ok((0, None));
        }
        if !self.in_sync {
            // Find a major sync with the access unit header before it.
            let mut found = None;
            for (i, &b) in buf.iter().enumerate() {
                self.pc.state = (self.pc.state << 8) | u32::from(b);
                if self.pc.state & 0xFFFF_FFFE == 0xF872_6FBA && self.pc.held() + i >= 7 {
                    found = Some(i);
                    break;
                }
            }
            let Some(i) = found else {
                self.pc.combine(END_NOT_FOUND, buf)?;
                return Ok((buf.len() as i64, None));
            };
            self.in_sync = true;
            self.bytes_left = 0;
            let next = i as i64 - 7;
            self.pc.combine(next, buf)?;
            return Ok((next, None));
        }
        if self.bytes_left == 0 {
            self.pc.move_overread();
            let held = self.pc.held();
            if held + buf.len() < 2 {
                self.pc.combine(END_NOT_FOUND, buf)?;
                return Ok((buf.len() as i64, None));
            }
            let first = if held > 0 {
                self.pc.held_byte(0)
            } else {
                buf[0]
            };
            let second = if held > 1 {
                self.pc.held_byte(1)
            } else {
                buf[1 - held]
            };
            self.bytes_left = i64::from((u16::from(first) << 8 | u16::from(second)) & 0xFFF) * 2;
            if self.bytes_left <= 0 {
                self.in_sync = false;
                return Ok((1, None));
            }
            self.bytes_left -= held as i64;
        }
        let next = if self.bytes_left > buf.len() as i64 {
            END_NOT_FOUND
        } else {
            self.bytes_left
        };
        let Some(unit) = self.pc.combine(next, buf)? else {
            self.bytes_left -= buf.len() as i64;
            return Ok((buf.len() as i64, None));
        };
        self.bytes_left = 0;
        // (AV_RB32(buf + 4) & 0xfffffffe) == 0xf8726fba: TrueHD or MLP.
        let sync = unit.len() >= 8 && unit[4..7] == [0xF8, 0x72, 0x6F] && unit[7] & 0xFE == 0xBA;
        if !sync {
            // The parity of the access unit header and the substream
            // headers.
            let byte = |p: usize| unit.get(p).copied().unwrap_or(0);
            let (mut parity, mut p) = (0u8, 0usize);
            for i in -1..self.num_substreams {
                parity ^= byte(p) ^ byte(p + 1);
                p += 2;
                if i < 0 || byte(p - 2) & 0x80 != 0 {
                    parity ^= byte(p) ^ byte(p + 1);
                    p += 2;
                }
            }
            if ((parity >> 4) ^ parity) & 0xF != 0xF {
                self.in_sync = false;
                return Ok((1, None));
            }
        } else {
            let Some(mh) = read_major_sync(&unit[4..]) else {
                self.in_sync = false;
                return Ok((1, None));
            };
            s.key_frame = 1;
            avctx.sample_rate = mh.sample_rate;
            avctx.frame_size = mh.access_unit_size;
            s.duration = mh.access_unit_size;
            avctx.channels = mh.channels;
            self.num_substreams = mh.num_substreams;
        }
        Ok((next, Some(unit)))
    }
}
