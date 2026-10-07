// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/aac_ac3_parser.c (ff_aac_ac3_parse),
// libavcodec/ac3_parser.c (ac3_sync, ff_ac3_find_syncword,
// ff_ac3_parse_header, avpriv_ac3_parse_header), libavcodec/aac_parser.c
// (aac_sync), libavcodec/adts_header.c (ff_adts_header_parse), the tables of
// libavcodec/ac3tab.c and mpeg4audio_sample_rates.h, and the CRC-16 of
// libavutil/crc.c (AV_CRC_16_ANSI).
// Copyright (c) 2003 Fabrice Bellard, 2003 Michael Niedermayer,
//           (c) 2006 Justin Ruggles
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::bits::BitReader;
use super::parser::{Codec, CodecCtx, ParseContext, ParserState, END_NOT_FOUND};

const AC3_HEADER_SIZE: i64 = 7;
const ADTS_HEADER_SIZE: i64 = 7;

const EAC3_FRAME_TYPE_DEPENDENT: u32 = 1;
const EAC3_FRAME_TYPE_AC3_CONVERT: u32 = 2;
const EAC3_FRAME_TYPE_RESERVED: u32 = 3;

/// ff_ac3_frame_size_tab (16-bit words).
const AC3_FRAME_SIZE_TAB: [[u16; 3]; 38] = [
    [64, 69, 96],
    [64, 70, 96],
    [80, 87, 120],
    [80, 88, 120],
    [96, 104, 144],
    [96, 105, 144],
    [112, 121, 168],
    [112, 122, 168],
    [128, 139, 192],
    [128, 140, 192],
    [160, 174, 240],
    [160, 175, 240],
    [192, 208, 288],
    [192, 209, 288],
    [224, 243, 336],
    [224, 244, 336],
    [256, 278, 384],
    [256, 279, 384],
    [320, 348, 480],
    [320, 349, 480],
    [384, 417, 576],
    [384, 418, 576],
    [448, 487, 672],
    [448, 488, 672],
    [512, 557, 768],
    [512, 558, 768],
    [640, 696, 960],
    [640, 697, 960],
    [768, 835, 1152],
    [768, 836, 1152],
    [896, 975, 1344],
    [896, 976, 1344],
    [1024, 1114, 1536],
    [1024, 1115, 1536],
    [1152, 1253, 1728],
    [1152, 1254, 1728],
    [1280, 1393, 1920],
    [1280, 1394, 1920],
];

/// ff_ac3_channels_tab.
const AC3_CHANNELS_TAB: [i32; 8] = [2, 1, 2, 3, 3, 4, 4, 5];
/// ff_ac3_sample_rate_tab.
const AC3_SAMPLE_RATE_TAB: [i32; 4] = [48000, 44100, 32000, 0];
/// eac3_blocks.
const EAC3_BLOCKS: [i32; 4] = [1, 2, 3, 6];
/// The speaker count of each ff_eac3_custom_channel_map_locations mask.
const EAC3_CUSTOM_CHANNELS: [u32; 16] = [1, 1, 1, 1, 1, 2, 2, 1, 1, 2, 2, 2, 1, 2, 1, 1];
const EAC3_MAX_CHANNELS: u32 = 16;
/// ff_mpeg4audio_sample_rates.
pub(crate) const MPEG4_SAMPLE_RATES: [i32; 16] = [
    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350, 0, 0,
    0,
];

/// The AC3HeaderInfo fields used here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Ac3Header {
    bitstream_id: u32,
    frame_type: u32,
    frame_size: i64,
    sample_rate: i32,
    num_blocks: i32,
    channels: i32,
}

/// ff_ac3_parse_header, as far as it can fail.
fn ac3_parse_header(gb: &mut BitReader<'_>) -> Option<Ac3Header> {
    if gb.read(16) != 0x0B77 {
        return None;
    }
    let bitstream_id = gb.peek(29) & 0x1F;
    if bitstream_id > 16 {
        return None;
    }
    if bitstream_id <= 10 {
        gb.skip(16); // crc1
        let sr_code = gb.read(2) as usize;
        if sr_code == 3 {
            return None;
        }
        let frame_size_code = gb.read(6) as usize;
        if frame_size_code > 37 {
            return None;
        }
        gb.skip(5); // bsid
        gb.skip(3); // bsmod
        let channel_mode = gb.read(3) as usize;
        if channel_mode == 2 {
            gb.skip(2);
        } else {
            if channel_mode & 1 != 0 && channel_mode != 1 {
                gb.skip(2);
            }
            if channel_mode & 4 != 0 {
                gb.skip(2);
            }
        }
        let lfe_on = gb.read(1) as i32;
        let sr_shift = bitstream_id.max(8) - 8;
        Some(Ac3Header {
            bitstream_id,
            frame_type: EAC3_FRAME_TYPE_AC3_CONVERT,
            frame_size: i64::from(AC3_FRAME_SIZE_TAB[frame_size_code][sr_code]) * 2,
            sample_rate: AC3_SAMPLE_RATE_TAB[sr_code] >> sr_shift,
            num_blocks: 6,
            channels: AC3_CHANNELS_TAB[channel_mode] + lfe_on,
        })
    } else {
        let frame_type = gb.read(2);
        if frame_type == EAC3_FRAME_TYPE_RESERVED {
            return None;
        }
        let substreamid = gb.read(3);
        let frame_size = (i64::from(gb.read(11)) + 1) << 1;
        if frame_size < AC3_HEADER_SIZE {
            return None;
        }
        let sr_code = gb.read(2) as usize;
        let (sample_rate, num_blocks) = if sr_code == 3 {
            let sr_code2 = gb.read(2) as usize;
            if sr_code2 == 3 {
                return None;
            }
            (AC3_SAMPLE_RATE_TAB[sr_code2] / 2, 6)
        } else {
            let blocks = EAC3_BLOCKS[gb.read(2) as usize];
            (AC3_SAMPLE_RATE_TAB[sr_code], blocks)
        };
        let channel_mode = gb.read(3) as usize;
        let lfe_on = gb.read(1) as i32;
        // eac3_parse_header, up to its last failure point.
        if substreamid != 0 {
            return None;
        }
        gb.skip(5); // bsid
        for _ in 0..if channel_mode != 0 { 1 } else { 2 } {
            gb.skip(5); // dialnorm
            if gb.read1() {
                gb.skip(8);
            }
        }
        if frame_type == EAC3_FRAME_TYPE_DEPENDENT && gb.read1() {
            let channel_map = gb.read(16);
            let count: u32 = (0..16)
                .filter(|&i| channel_map & (1 << (EAC3_MAX_CHANNELS - i - 1)) != 0)
                .map(|i| EAC3_CUSTOM_CHANNELS[i as usize])
                .sum();
            if count > EAC3_MAX_CHANNELS {
                return None;
            }
        }
        Some(Ac3Header {
            bitstream_id,
            frame_type,
            frame_size,
            sample_rate,
            num_blocks,
            channels: AC3_CHANNELS_TAB[channel_mode] + lfe_on,
        })
    }
}

/// ac3_sync: the frame size of a syncframe header in the last seven
/// bytes of `state`, with whether it starts a frame and whether the next
/// header must be found before the frame ends.
fn ac3_sync(state: u64) -> Option<(i64, bool, bool)> {
    let mut tmp = state.to_be_bytes();
    if tmp[1] == 0x77 && tmp[2] == 0x0B {
        tmp.swap(1, 2);
        tmp.swap(3, 4);
        tmp.swap(5, 6);
    }
    let mut gb = BitReader::with_bits(&tmp[1..8], 54);
    let hdr = ac3_parse_header(&mut gb)?;
    let new_frame_start = hdr.frame_type != EAC3_FRAME_TYPE_DEPENDENT;
    let need_next_header = new_frame_start || hdr.frame_type != EAC3_FRAME_TYPE_AC3_CONVERT;
    Some((hdr.frame_size, need_next_header, new_frame_start))
}

/// The AACADTSHeaderInfo fields used here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct AdtsHeader {
    pub sample_rate: i32,
    pub samples: i32,
    pub channels: i32,
    pub frame_length: i64,
}

/// ff_adts_header_parse over the first seven bytes of `buf`.
fn adts_header(buf: &[u8]) -> Option<AdtsHeader> {
    let mut gb = BitReader::new(&buf[..buf.len().min(7)]);
    if gb.read(12) != 0xFFF {
        return None;
    }
    gb.skip(3); // id, layer
    gb.skip(1); // protection_absent
    gb.skip(2); // profile_objecttype
    let sr = gb.read(4) as usize;
    let sample_rate = MPEG4_SAMPLE_RATES[sr];
    if sample_rate == 0 {
        return None;
    }
    gb.skip(1);
    let channels = gb.read(3) as i32;
    gb.skip(4);
    let size = i64::from(gb.read(13));
    if size < ADTS_HEADER_SIZE {
        return None;
    }
    gb.skip(11);
    let rdb = gb.read(2) as i32;
    Some(AdtsHeader {
        sample_rate,
        samples: (rdb + 1) * 1024,
        channels,
        frame_length: size,
    })
}

/// aac_sync: ADTS frames are trusted to their length.
fn aac_sync(state: u64) -> Option<(i64, bool, bool)> {
    let tmp = state.to_be_bytes();
    adts_header(&tmp[1..8]).map(|h| (h.frame_length, false, true))
}

/// ff_ac3_find_syncword.
fn ac3_find_syncword(buf: &[u8]) -> Option<usize> {
    let at = |i: usize| buf.get(i).copied().unwrap_or(0);
    let mut i = 1;
    while i < buf.len() {
        if buf[i] == 0x77 || buf[i] == 0x0B {
            if buf[i] ^ buf[i - 1] == 0x77 ^ 0x0B {
                return Some(i - 1);
            }
            if buf[i] ^ at(i + 1) == 0x77 ^ 0x0B {
                return Some(i);
            }
        }
        i += 2;
    }
    None
}

/// av_crc with AV_CRC_16_ANSI from zero.
fn crc16_ansi(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                crc << 1 ^ 0x8005
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[derive(Debug)]
pub(crate) struct AacAc3Parser {
    pc: ParseContext,
    state: u64,
    remaining_size: i64,
    need_next_header: bool,
    aac: bool,
    /// The ADTS header of the last unit returned, for the timing code
    /// (what FFmpeg's decoder would have set on the codec context).
    pub last_adts: Option<AdtsHeader>,
}

impl AacAc3Parser {
    pub fn aac() -> Self {
        Self {
            pc: ParseContext::new(),
            state: 0,
            remaining_size: 0,
            need_next_header: false,
            aac: true,
            last_adts: None,
        }
    }

    pub fn ac3() -> Self {
        Self {
            aac: false,
            ..Self::aac()
        }
    }

    fn sync(&self, state: u64) -> Option<(i64, bool, bool)> {
        if self.aac {
            aac_sync(state)
        } else {
            ac3_sync(state)
        }
    }

    /// ff_aac_ac3_parse.
    pub fn parse(
        &mut self,
        s1: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> (i64, Option<Vec<u8>>) {
        s1.key_frame = -1;
        let header_size = if self.aac {
            ADTS_HEADER_SIZE
        } else {
            AC3_HEADER_SIZE
        };
        let size = buf.len() as i64;
        let mut got_frame = false;
        let mut i;
        loop {
            i = END_NOT_FOUND;
            if self.remaining_size <= size {
                if self.remaining_size != 0 && !self.need_next_header {
                    i = self.remaining_size;
                    self.remaining_size = 0;
                } else {
                    let mut found = None;
                    let mut at = self.remaining_size.max(0);
                    while at < size {
                        self.state = (self.state << 8).wrapping_add(u64::from(buf[at as usize]));
                        if let Some(hit) = self.sync(self.state).filter(|h| h.0 != 0) {
                            found = Some(hit);
                            break;
                        }
                        at += 1;
                    }
                    if let Some((len, need_next_header, new_frame_start)) =
                        found.filter(|h| h.0 > 0)
                    {
                        self.need_next_header = need_next_header;
                        got_frame = true;
                        self.state = 0;
                        i = at - (header_size - 1);
                        self.remaining_size = len;
                        if !new_frame_start || self.pc_index() + i <= 0 {
                            self.remaining_size += i;
                            continue;
                        } else if i < 0 {
                            self.remaining_size += i;
                        }
                    }
                }
            }
            break;
        }
        let Some(unit) = self.pc.combine(i, buf) else {
            self.remaining_size -= self.remaining_size.min(size);
            return (size, None);
        };
        if got_frame {
            if self.aac {
                if let Some(hdr) = (unit.len() >= 7).then(|| adts_header(&unit)).flatten() {
                    s1.key_frame = 1;
                    self.last_adts = Some(hdr);
                }
            } else if let Some(hdr) = Self::last_valid_ac3(&unit) {
                avctx.sample_rate = hdr.sample_rate;
                if hdr.bitstream_id > 10 {
                    avctx.codec = Codec::Eac3;
                }
                if avctx.codec != Codec::Eac3 {
                    avctx.channels = hdr.channels;
                }
                s1.duration = hdr.num_blocks * 256;
            }
        }
        (i, Some(unit))
    }

    /// The header of the last syncframe of a unit whose frames chain to
    /// its end and whose last frame passes its CRC.
    fn last_valid_ac3(unit: &[u8]) -> Option<Ac3Header> {
        let mut buf = &unit[ac3_find_syncword(unit)?..];
        loop {
            let hdr = ac3_parse_header(&mut BitReader::new(buf))?;
            let frame_size = usize::try_from(hdr.frame_size).ok()?;
            if frame_size > buf.len() {
                return None;
            }
            if buf.len() > frame_size {
                buf = &buf[frame_size..];
                continue;
            }
            return (crc16_ansi(&buf[2..frame_size]) == 0).then_some(hdr);
        }
    }

    fn pc_index(&self) -> i64 {
        self.pc.held() as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_matches_libavutil() {
        // CRC-16/UMTS ("123456789") is the same polynomial and register.
        assert_eq!(crc16_ansi(b"123456789"), 0xFEE8);
    }

    #[test]
    fn adts_and_ac3_headers() {
        // ADTS: 48 kHz, mono, 217-byte frame, one raw data block.
        let adts = [0xFF, 0xF1, 0x4C, 0x40, 0x1B, 0x3F, 0xFC];
        assert_eq!(
            adts_header(&adts),
            Some(AdtsHeader {
                sample_rate: 48000,
                samples: 1024,
                channels: 1,
                frame_length: 217
            })
        );
        // AC-3: 48 kHz, frmsizecod 12 (192 kbit/s: 384 bytes), 2/0 mode.
        let ac3: [u8; 7] = [0x0B, 0x77, 0, 0, 0x0C, 0x40, 0x40];
        let mut state = 0u64;
        for b in ac3 {
            state = state << 8 | u64::from(b);
        }
        assert_eq!(ac3_sync(state), Some((384, true, true)));
        assert_eq!(ac3_find_syncword(&[1, 2, 0x0B, 0x77, 5]), Some(2));
        assert_eq!(ac3_find_syncword(&[1, 0x0B, 0x77, 5]), Some(1));
    }
}
