// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/mpegvideo_parser.c (mpeg1_find_frame_end,
// mpegvideo_extract_headers, mpegvideo_parse), with the frame rate table of
// libavcodec/mpeg12framerate.c and ff_set_dimensions (libavcodec/utils.c).
// Copyright (c) 2000,2001 Fabrice Bellard
// Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at> (mpegvideo_parser.c)
// Copyright (c) 2001 Fabrice Bellard
// Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at> (utils.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use super::parser::{
    Codec, CodecCtx, Overflow, ParseContext, ParserState, PixFmt, END_NOT_FOUND, Q,
};

const PICTURE_START_CODE: u32 = 0x100;
const SLICE_MIN_START_CODE: u32 = 0x101;
const SLICE_MAX_START_CODE: u32 = 0x1AF;
const SEQ_START_CODE: u32 = 0x1B3;
const EXT_START_CODE: u32 = 0x1B5;
const SEQ_END_CODE: u32 = 0x1B7;

/// ff_mpeg12_frame_rate_tab.
const FRAME_RATES: [Q; 16] = [
    Q { num: 0, den: 0 },
    Q {
        num: 24000,
        den: 1001,
    },
    Q { num: 24, den: 1 },
    Q { num: 25, den: 1 },
    Q {
        num: 30000,
        den: 1001,
    },
    Q { num: 30, den: 1 },
    Q { num: 50, den: 1 },
    Q {
        num: 60000,
        den: 1001,
    },
    Q { num: 60, den: 1 },
    Q { num: 15, den: 1 },
    Q { num: 5, den: 1 },
    Q { num: 10, den: 1 },
    Q { num: 12, den: 1 },
    Q { num: 15, den: 1 },
    Q { num: 0, den: 0 },
    Q { num: 0, den: 0 },
];

/// AV_PICTURE_STRUCTURE_FRAME.
const STRUCT_FRAME: i32 = 3;

#[derive(Debug)]
pub(crate) struct MpegVideoParser {
    pc: ParseContext,
    frame_rate: Q,
    progressive_sequence: bool,
    width: i32,
    height: i32,
}

/// avpriv_find_start_code over `buf[from..]`.
fn find_start_code(buf: &[u8], from: usize, state: &mut u32) -> usize {
    super::parser::find_start_code(buf, from, buf.len(), state)
}

/// ff_set_dimensions: zero on a size av_image_check_size2 rejects.
fn set_dimensions(avctx: &mut CodecCtx, width: i32, height: i32) {
    let ok = width > 0
        && height > 0
        && ((i64::from(width) + 128) as u64) * ((i64::from(height) + 128) as u64)
            < (i32::MAX / 8) as u64;
    let (w, h) = if ok { (width, height) } else { (0, 0) };
    avctx.width = w;
    avctx.coded_width = w;
    avctx.height = h;
    avctx.coded_height = h;
}

impl MpegVideoParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            frame_rate: Q { num: 0, den: 1 },
            progressive_sequence: false,
            width: 0,
            height: 0,
        }
    }

    /// mpeg1_find_frame_end.
    fn find_frame_end(&mut self, s: &mut ParserState, buf: &[u8]) -> i64 {
        let pc = &mut self.pc;
        let mut state = pc.state;
        if buf.is_empty() {
            return 0;
        }
        let size = buf.len();
        let mut i = 0usize;
        while i < size {
            if pc.frame_start_found & 1 != 0 {
                if state == EXT_START_CODE && buf[i] & 0xF0 != 0x80 {
                    pc.frame_start_found -= 1;
                } else if state == EXT_START_CODE + 2 {
                    if buf[i] & 3 == 3 {
                        pc.frame_start_found = 0;
                    } else {
                        pc.frame_start_found = (pc.frame_start_found + 1) & 3;
                    }
                }
                state = state.wrapping_add(1);
            } else {
                i = find_start_code(buf, i, &mut state) - 1;
                if pc.frame_start_found == 0
                    && (SLICE_MIN_START_CODE..=SLICE_MAX_START_CODE).contains(&state)
                {
                    i += 1;
                    pc.frame_start_found = 4;
                }
                if state == SEQ_END_CODE {
                    pc.frame_start_found = 0;
                    pc.state = u32::MAX;
                    return i as i64 + 1;
                }
                if pc.frame_start_found == 2 && state == SEQ_START_CODE {
                    pc.frame_start_found = 0;
                }
                if pc.frame_start_found < 4 && state == EXT_START_CODE {
                    pc.frame_start_found += 1;
                }
                if pc.frame_start_found == 4
                    && state & 0xFFFF_FF00 == 0x100
                    && !(SLICE_MIN_START_CODE..=SLICE_MAX_START_CODE).contains(&state)
                {
                    pc.frame_start_found = 0;
                    pc.state = u32::MAX;
                    return i as i64 - 3;
                }
                if pc.frame_start_found == 0 && state == PICTURE_START_CODE {
                    s.fetch_timestamp(i as i64 - 3, true, i > 3);
                }
            }
            i += 1;
        }
        pc.state = state;
        END_NOT_FOUND
    }

    /// mpegvideo_extract_headers.
    fn extract_headers(&mut self, s: &mut ParserState, avctx: &mut CodecCtx, buf: &[u8]) {
        let size = buf.len();
        let mut did_set_size = false;
        let mut pix_fmt: Option<PixFmt> = None;
        let mut nb_pic_ext = 0;
        let mut pos = 0usize;
        while pos < size {
            let mut start_code = u32::MAX;
            pos = find_start_code(buf, pos, &mut start_code);
            let b = &buf[pos..];
            match start_code {
                PICTURE_START_CODE => {
                    if b.len() >= 2 {
                        s.pict_type = i32::from((b[1] >> 3) & 7);
                    }
                }
                SEQ_START_CODE => {
                    if b.len() >= 7 {
                        self.width = i32::from(b[0]) << 4 | i32::from(b[1] >> 4);
                        self.height = i32::from(b[1] & 0x0F) << 8 | i32::from(b[2]);
                        if avctx.width == 0
                            || avctx.height == 0
                            || avctx.coded_width == 0
                            || avctx.coded_height == 0
                        {
                            set_dimensions(avctx, self.width, self.height);
                            did_set_size = true;
                        }
                        pix_fmt = Some(PixFmt::Yuv420p);
                        self.frame_rate = FRAME_RATES[usize::from(b[3] & 0x0F)];
                        avctx.framerate = self.frame_rate;
                        avctx.codec = Codec::Mpeg1Video;
                    }
                }
                EXT_START_CODE => {
                    if !b.is_empty() {
                        match b[0] >> 4 {
                            0x1 if b.len() >= 6 => {
                                let horiz_size_ext =
                                    i32::from(b[1] & 1) << 1 | i32::from(b[2] >> 7);
                                let vert_size_ext = i32::from(b[2] >> 5) & 3;
                                let frame_rate_ext_n = i64::from(b[5] >> 5) & 3;
                                let frame_rate_ext_d = i64::from(b[5] & 0x1F);
                                self.progressive_sequence = b[1] & (1 << 3) != 0;
                                avctx.has_b_frames = i32::from(b[5] >> 7 == 0);
                                match (b[1] >> 1) & 3 {
                                    1 => pix_fmt = Some(PixFmt::Yuv420p),
                                    2 => pix_fmt = Some(PixFmt::Yuv422p),
                                    3 => pix_fmt = Some(PixFmt::Yuv444p),
                                    _ => {}
                                }
                                self.width = (self.width & 0xFFF) | horiz_size_ext << 12;
                                self.height = (self.height & 0xFFF) | vert_size_ext << 12;
                                if did_set_size {
                                    set_dimensions(avctx, self.width, self.height);
                                }
                                avctx.framerate = Q {
                                    num: self.frame_rate.num * (frame_rate_ext_n + 1),
                                    den: self.frame_rate.den * (frame_rate_ext_d + 1),
                                };
                                avctx.codec = Codec::Mpeg2Video;
                            }
                            0x8 if b.len() >= 5 => {
                                let top_field_first = b[3] & (1 << 7) != 0;
                                let repeat_first_field = b[3] & (1 << 1) != 0;
                                let progressive_frame = b[4] & (1 << 7) != 0;
                                s.repeat_pict = 1;
                                if repeat_first_field {
                                    if self.progressive_sequence {
                                        s.repeat_pict = if top_field_first { 5 } else { 3 };
                                    } else if progressive_frame {
                                        s.repeat_pict = 2;
                                    }
                                }
                                s.picture_structure = i32::from(b[2] & 3);
                                nb_pic_ext += 1;
                            }
                            _ => {}
                        }
                    }
                }
                u32::MAX => break,
                code if (SLICE_MIN_START_CODE..=SLICE_MAX_START_CODE).contains(&code) => break,
                _ => {}
            }
        }
        if let Some(fmt) = pix_fmt {
            s.format = Some(fmt);
            s.width = self.width;
            s.height = self.height;
            s.coded_width = (self.width + 15) & !15;
            s.coded_height = (self.height + 15) & !15;
        }
        if avctx.codec == Codec::Mpeg1Video || nb_pic_ext > 1 {
            s.repeat_pict = 1;
            s.picture_structure = STRUCT_FRAME;
        }
    }

    /// mpegvideo_parse.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        let next = self.find_frame_end(s, buf);
        let Some(unit) = self.pc.combine(next, buf)? else {
            return Ok((buf.len() as i64, None));
        };
        self.extract_headers(s, avctx, &unit);
        Ok((next, Some(unit)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_code_search() {
        let buf = [0xAA, 0, 0, 1, 0xB3, 0x14, 0, 0, 1];
        let mut state = u32::MAX;
        assert_eq!(find_start_code(&buf, 0, &mut state), 5);
        assert_eq!(state, 0x1B3);
        assert_eq!(find_start_code(&buf, 5, &mut state), buf.len());
        assert_eq!(state, 0x1400_0001);
    }
}
