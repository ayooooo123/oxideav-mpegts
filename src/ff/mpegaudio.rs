// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/mpegaudio_parser.c with
// libavcodec/mpegaudiodecheader.[ch] (ff_mpa_check_header,
// ff_mpegaudio_decode_header, ff_mpa_decode_header) and the tables of
// libavcodec/mpegaudiotabs.h.
// Copyright (c) 2003 Fabrice Bellard, 2003 Michael Niedermayer
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::parser::{Codec, CodecCtx, ParseContext, ParserState, END_NOT_FOUND};

/// header + layer + freq + lsf/mpeg25
const SAME_HEADER_MASK: u32 = 0xFFE0_0000 | 3 << 17 | 3 << 10 | 3 << 19;

/// ff_mpa_bitrate_tab.
const BITRATE_TAB: [[[u16; 15]; 3]; 2] = [
    [
        [
            0, 32, 64, 96, 128, 160, 192, 224, 256, 288, 320, 352, 384, 416, 448,
        ],
        [
            0, 32, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 384,
        ],
        [
            0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320,
        ],
    ],
    [
        [
            0, 32, 48, 56, 64, 80, 96, 112, 128, 144, 160, 176, 192, 224, 256,
        ],
        [0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160],
        [0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160],
    ],
];

/// ff_mpa_freq_tab.
const FREQ_TAB: [i32; 3] = [44100, 48000, 32000];

/// What ff_mpa_decode_header reports for a valid header.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct MpaHeader {
    /// Frame bytes, header included.
    pub frame_size: i32,
    pub sample_rate: i32,
    pub channels: i32,
    /// Samples per frame.
    pub samples: i32,
    pub codec: Codec,
}

/// ff_mpa_check_header + ff_mpegaudio_decode_header + ff_mpa_decode_header:
/// `None` where those return a negative value or report free format.
pub(crate) fn decode_header(header: u32) -> Option<MpaHeader> {
    if header & 0xFFE0_0000 != 0xFFE0_0000
        || header & (3 << 19) == 1 << 19
        || header & (3 << 17) == 0
        || header & (0xF << 12) == 0xF << 12
        || header & (3 << 10) == 3 << 10
    {
        return None;
    }
    let (lsf, mpeg25) = if header & (1 << 20) != 0 {
        (u32::from(header & (1 << 19) == 0), 0)
    } else {
        (1, 1)
    };
    let layer = 4 - ((header >> 17) & 3);
    let sample_rate_index = ((header >> 10) & 3) as usize;
    let sample_rate = FREQ_TAB[sample_rate_index] >> (lsf + mpeg25);
    let bitrate_index = ((header >> 12) & 0xF) as usize;
    let padding = ((header >> 9) & 1) as i32;
    let channels = if (header >> 6) & 3 == 3 { 1 } else { 2 };
    if bitrate_index == 0 {
        return None;
    }
    let kbps = i32::from(BITRATE_TAB[lsf as usize][layer as usize - 1][bitrate_index]);
    let frame_size = match layer {
        1 => (kbps * 12000 / sample_rate + padding) * 4,
        2 => kbps * 144_000 / sample_rate + padding,
        _ => kbps * 144_000 / (sample_rate << lsf) + padding,
    };
    let (codec, samples) = match layer {
        1 => (Codec::Mp1, 384),
        2 => (Codec::Mp2, 1152),
        _ => (Codec::Mp3, if lsf != 0 { 576 } else { 1152 }),
    };
    Some(MpaHeader {
        frame_size,
        sample_rate,
        channels,
        samples,
        codec,
    })
}

#[derive(Debug)]
pub(crate) struct MpegAudioParser {
    pc: ParseContext,
    frame_size: i32,
    header: u32,
    header_count: i32,
}

impl MpegAudioParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            frame_size: 0,
            header: 0,
            header_count: 0,
        }
    }

    /// mpegaudio_parse.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> (i64, Option<Vec<u8>>) {
        let mut state = self.pc.state;
        let size = buf.len();
        let mut i = 0usize;
        let mut next = END_NOT_FOUND;
        let flush = buf.is_empty();
        'outer: while i < size {
            if self.frame_size != 0 {
                let inc = (size - i).min(self.frame_size as usize);
                i += inc;
                self.frame_size -= inc as i32;
                state = 0;
                if self.frame_size == 0 {
                    next = i as i64;
                    break;
                }
            } else {
                while i < size {
                    state = (state << 8).wrapping_add(u32::from(buf[i]));
                    i += 1;
                    match decode_header(state).filter(|h| h.frame_size >= 4) {
                        None => {
                            if i > 4 {
                                self.header_count = -2;
                            }
                        }
                        Some(h) => {
                            let header_threshold = i32::from(avctx.codec != h.codec);
                            if state & SAME_HEADER_MASK != self.header & SAME_HEADER_MASK
                                && self.header != 0
                            {
                                self.header_count = -3;
                            }
                            self.header = state;
                            self.header_count += 1;
                            self.frame_size = h.frame_size - 4;
                            if self.header_count > header_threshold {
                                avctx.sample_rate = h.sample_rate;
                                avctx.channels = h.channels;
                                s.duration = h.samples;
                                avctx.codec = h.codec;
                            }
                            continue 'outer;
                        }
                    }
                }
            }
        }
        self.pc.state = state;
        let Some(unit) = self.pc.combine(next, buf) else {
            return (size as i64, None);
        };
        if flush && unit.len() >= 128 && unit.starts_with(b"TAG") {
            return (next, None);
        }
        if flush && unit.len() >= 32 && unit.starts_with(b"APETAGEX") {
            return (next, None);
        }
        (next, Some(unit))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layer_two_and_three_headers() {
        // MPEG-1 layer II, 48 kHz, 224 kbit/s, stereo: 672-byte frames.
        let h = decode_header(0xFFFD_B404).unwrap();
        assert_eq!(
            (h.frame_size, h.sample_rate, h.channels, h.samples, h.codec),
            (672, 48000, 2, 1152, Codec::Mp2)
        );
        // MPEG-2 layer III, 24 kHz, 48 kbit/s: 576 samples in 144 bytes.
        let h = decode_header(0xFFF3_6400).unwrap();
        assert_eq!(
            (h.frame_size, h.sample_rate, h.samples, h.codec),
            (144, 24000, 576, Codec::Mp3)
        );
        // Free format and reserved values are no frame.
        assert_eq!(decode_header(0xFFFD_0404), None);
        assert_eq!(decode_header(0xFFFD_F404), None);
        assert_eq!(decode_header(0xFFE9_A404), None);
    }
}
