// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/h264_parser.c, with the parts of
// h264_ps.c (SPS/PPS decoding), h264_sei.c and h2645_sei.c (the SEI messages
// the parser reads), h264_parse.c (ff_h264_parse_ref_count,
// ff_h264_pred_weight_table, ff_h264_init_poc), h2645_vui.c,
// h2645_parse.c (ff_h2645_extract_rbsp), startcode.c
// (avpriv_find_start_code, ff_startcode_find_candidate_c) and
// libavutil/imgutils.c (av_image_check_size) it calls.
// Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
// Copyright (C) 2012 - 2013 Guillaume Martres
// Copyright (C) 2012 - 2013 Mickael Raulet (h2645_vui.c)
// Copyright (C) 2012 - 2013 Gildas Cocherel
// Copyright (C) 2013 Vittorio Giovara (h2645_sei.c, h2645_vui.c)
// Copyright (c) 2003-2010 Michael Niedermayer <michaelni@gmx.at> (startcode.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

//! H.264 Annex B access-unit splitting and the per-unit facts FFmpeg's
//! parser reports: key frame (IDR, recovery point, or the single-reference
//! intra heuristic), picture type, field structure and repeat count, the
//! cropped size and pixel format, the VUI frame rate, and HRD-derived
//! timestamps. Parameter sets are kept per id (32 SPS, 256 PPS), so state
//! is bounded; every read of the bitstream is bounds-checked.

use std::borrow::Cow;
use std::sync::Arc;

use super::bits::BitReader;
use super::parser::{
    find_start_code, pict, reduce, rescale, CodecCtx, Overflow, ParseContext, ParserState, PixFmt,
    END_NOT_FOUND, NOPTS, Q,
};

const NAL_SLICE: u8 = 1;
const NAL_DPA: u8 = 2;
const NAL_IDR: u8 = 5;
const NAL_SEI: u8 = 6;
const NAL_SPS: u8 = 7;
const NAL_PPS: u8 = 8;
const NAL_AUD: u8 = 9;

const MAX_SPS_COUNT: usize = 32;
const MAX_PPS_COUNT: usize = 256;
const MAX_LOG2_MAX_FRAME_NUM: i32 = 12 + 4;
const MAX_DPB_FRAMES: i32 = 16;
const MAX_MMCO_COUNT: usize = 2 * 2 * MAX_DPB_FRAMES as usize + 3;

/// PICT_TOP_FIELD, PICT_BOTTOM_FIELD, PICT_FRAME (mpegutils.h).
const PICT_TOP_FIELD: i32 = 1;
const PICT_BOTTOM_FIELD: i32 = 2;
const PICT_FRAME: i32 = 3;

/// ff_h264_golomb_to_pict_type.
const GOLOMB_TO_PICT_TYPE: [i32; 5] = [pict::P, pict::B, pict::I, pict::SP, pict::SI];

/// level_max_dpb_mbs (h264_ps.c).
const LEVEL_MAX_DPB_MBS: [(i32, i32); 16] = [
    (10, 396),
    (11, 900),
    (12, 2376),
    (13, 2376),
    (20, 2376),
    (21, 4752),
    (22, 8100),
    (30, 8100),
    (31, 18000),
    (32, 20480),
    (40, 32768),
    (41, 32768),
    (42, 34816),
    (50, 110400),
    (51, 184320),
    (52, 184320),
];

/// sei_num_clock_ts_table is not needed: nothing after pic_struct is used.
const PIC_STRUCT_TRIPLING: u32 = 8;

/// The SPS fields the parser uses.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Sps {
    profile_idc: i32,
    level_idc: i32,
    constraint_set_flags: i32,
    pub chroma_format_idc: i32,
    pub bit_depth_luma: i32,
    log2_max_frame_num: i32,
    poc_type: i32,
    log2_max_poc_lsb: i32,
    delta_pic_order_always_zero_flag: bool,
    offset_for_non_ref_pic: i32,
    offset_for_top_to_bottom_field: i32,
    offset_for_ref_frame: Vec<i32>,
    ref_frame_count: i32,
    mb_width: i32,
    mb_height: i32,
    frame_mbs_only_flag: bool,
    mb_aff: bool,
    crop_left: i32,
    crop_right: i32,
    crop_top: i32,
    crop_bottom: i32,
    timing_info_present_flag: bool,
    num_units_in_tick: u32,
    time_scale: u32,
    nal_hrd_parameters_present_flag: bool,
    vcl_hrd_parameters_present_flag: bool,
    cpb_removal_delay_length: u32,
    dpb_output_delay_length: u32,
    pic_struct_present_flag: bool,
    pub bitstream_restriction_flag: bool,
    pub num_reorder_frames: i32,
}

/// The PPS fields the parser uses, with the SPS it was decoded against.
#[derive(Clone, Debug)]
struct Pps {
    pic_order_present: bool,
    ref_count: [u32; 2],
    weighted_pred: bool,
    weighted_bipred_idc: u32,
    redundant_pic_cnt_present: bool,
    sps: Arc<Sps>,
}

#[derive(Clone, Debug, Default)]
struct Poc {
    poc_lsb: i32,
    poc_msb: i32,
    delta_poc_bottom: i32,
    delta_poc: [i32; 2],
    frame_num: i32,
    prev_poc_msb: i32,
    prev_poc_lsb: i32,
    frame_num_offset: i32,
    prev_frame_num_offset: i32,
    prev_frame_num: i32,
}

#[derive(Clone, Debug)]
struct Sei {
    recovery_frame_cnt: i32,
    timing_present: bool,
    timing_payload: Vec<u8>,
    cpb_removal_delay: i64,
    dpb_output_delay: i64,
    pic_struct: u32,
    buffering_period_present: bool,
    x264_build: i32,
}

impl Sei {
    /// ff_h264_sei_uninit plus the parser's per-unit resets.
    fn reset() -> Self {
        Sei {
            recovery_frame_cnt: -1,
            timing_present: false,
            timing_payload: Vec::new(),
            cpb_removal_delay: -1,
            dpb_output_delay: 0,
            pic_struct: 0,
            buffering_period_present: false,
            x264_build: -1,
        }
    }
}

pub(crate) struct H264Parser {
    pc: ParseContext,
    sps_list: Vec<Option<Arc<Sps>>>,
    pps_list: Vec<Option<Arc<Pps>>>,
    /// The SPS of the last slice: what the decoder would apply.
    pub active_sps: Option<Arc<Sps>>,
    poc: Poc,
    sei: Sei,
    picture_structure: i32,
    parse_history: [u8; 6],
    parse_history_count: usize,
    parse_last_mb: u32,
    reference_dts: i64,
    last_frame_num: i32,
    last_picture_structure: i32,
    /// got_first: the codec context's extradata was loaded.
    got_first: bool,
    /// The picture the last unit completes, as the decoder's
    /// h264_select_output_frame sees it.
    pub picture: Option<OutputPicture>,
    /// A first field waiting for its pair: (POC, structure, frame_num,
    /// IDR).
    pending_field: Option<(i32, i32, i32, bool)>,
}

/// A picture the decoder completes (a frame, or the second field of a
/// complementary pair), in decode order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OutputPicture {
    /// cur->poc: the lower of its field POCs.
    pub poc: i32,
    pub b: bool,
    /// An IDR: the decoder's idr() clears its POC history first.
    pub idr: bool,
    /// MMCO_RESET: the history is cleared once the picture is marked.
    pub mmco_reset: bool,
}

/// AV_PICTURE_STRUCTURE_* (avcodec.h).
const STRUCT_UNKNOWN: i32 = 0;
const STRUCT_TOP: i32 = 1;
const STRUCT_BOTTOM: i32 = 2;
const STRUCT_FRAME: i32 = 3;

impl H264Parser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            sps_list: vec![None; MAX_SPS_COUNT],
            pps_list: vec![None; MAX_PPS_COUNT],
            active_sps: None,
            poc: Poc::default(),
            sei: Sei::reset(),
            picture_structure: 0,
            parse_history: [0; 6],
            parse_history_count: 0,
            parse_last_mb: 0,
            reference_dts: NOPTS,
            last_frame_num: i32::MAX,
            last_picture_structure: STRUCT_UNKNOWN,
            got_first: false,
            picture: None,
            pending_field: None,
        }
    }

    /// ff_h264_decode_extradata for Annex B extradata (decode_extradata_ps):
    /// its SPS and PPS into the parameter set lists; the first that
    /// fails to decode ends it. An SPS is tried as FFmpeg tries it, from
    /// the NAL without emulation prevention, then from its raw bytes.
    fn decode_extradata(&mut self, data: &[u8]) {
        let mut at = 0;
        loop {
            at = find_nal_start(data, at);
            if at >= data.len() {
                return;
            }
            let (consumed, nal) = extract_rbsp(&data[at..]);
            let raw = &data[at..at + consumed];
            at += consumed.max(1);
            let Some(&header) = nal.first() else { continue };
            match header & 0x1F {
                NAL_SPS => {
                    let sps = decode_sps(&mut BitReader::new(&nal[1..]))
                        .or_else(|| decode_sps(&mut BitReader::new(&raw[1..])));
                    let Some((id, sps)) = sps else { return };
                    self.sps_list[id] = Some(Arc::new(sps));
                }
                NAL_PPS => {
                    let Some((id, pps)) =
                        decode_pps(&mut BitReader::new(&nal[1..]), &self.sps_list)
                    else {
                        return;
                    };
                    self.pps_list[id] = Some(Arc::new(pps));
                }
                _ => {}
            }
        }
    }

    /// h264_find_frame_end (Annex B input).
    fn find_frame_end(&mut self, buf: &[u8]) -> i64 {
        let pc = &mut self.pc;
        let mut state = pc.state;
        if state > 13 {
            state = 7;
        }
        let size = buf.len() as i64;
        let mut i: i64 = 0;
        while i < size {
            let b = buf[i as usize];
            if state == 7 {
                i += first_zero(&buf[i as usize..]) as i64;
                if i < size {
                    state = 2;
                }
            } else if state <= 2 {
                if b == 1 {
                    state ^= 5;
                } else if b != 0 {
                    state = 7;
                } else {
                    state >>= 1;
                }
            } else if state <= 5 {
                let nalu_type = b & 0x1F;
                if matches!(nalu_type, NAL_SEI | NAL_SPS | NAL_PPS | NAL_AUD) {
                    if pc.frame_start_found != 0 {
                        i += 1;
                        return Self::found(pc, i, state);
                    }
                } else if matches!(nalu_type, NAL_SLICE | NAL_DPA | NAL_IDR) {
                    state += 8;
                    i += 1;
                    continue;
                }
                state = 7;
            } else {
                let last_mb = self.parse_last_mb;
                self.parse_history[self.parse_history_count] = b;
                self.parse_history_count += 1;
                let mut gb = BitReader::new(&self.parse_history[..self.parse_history_count]);
                let mb = gb.ue_long();
                if gb.left() > 0 || self.parse_history_count > 5 {
                    self.parse_last_mb = mb;
                    if pc.frame_start_found != 0 {
                        if mb <= last_mb {
                            i -= self.parse_history_count as i64 - 1;
                            self.parse_history_count = 0;
                            return Self::found(pc, i, state);
                        }
                    } else {
                        pc.frame_start_found = 1;
                    }
                    self.parse_history_count = 0;
                    state = 7;
                }
            }
            i += 1;
        }
        pc.state = state;
        END_NOT_FOUND
    }

    fn found(pc: &mut ParseContext, i: i64, state: u32) -> i64 {
        pc.state = 7;
        pc.frame_start_found = 0;
        i - i64::from(state & 5)
    }

    /// h264_parse.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        if !self.got_first {
            self.got_first = true;
            let extradata = std::mem::take(&mut avctx.extradata);
            self.decode_extradata(&extradata);
            avctx.extradata = extradata;
        }
        let next = self.find_frame_end(buf);
        let Some(unit) = self.pc.combine(next, buf)? else {
            return Ok((buf.len() as i64, None));
        };
        if next < 0 && next != END_NOT_FOUND {
            let held = self.pc.before_last(next).to_vec();
            self.find_frame_end(&held);
        }
        self.parse_nal_units(s, avctx, &unit);

        let time_base = if avctx.framerate.num != 0 {
            // av_inv_q(av_mul_q(framerate, 2/1))
            let m = super::parser::mul_q(avctx.framerate, Q { num: 2, den: 1 });
            Q {
                num: m.den,
                den: m.num,
            }
        } else {
            Q { num: 0, den: 1 }
        };
        if self.sei.cpb_removal_delay >= 0 {
            s.dts_sync_point = i32::from(self.sei.buffering_period_present);
            s.dts_ref_dts_delta = self.sei.cpb_removal_delay as i32;
            s.pts_dts_delta = self.sei.dpb_output_delay as i32;
        } else {
            s.dts_sync_point = i32::MIN;
            s.dts_ref_dts_delta = i32::MIN;
            s.pts_dts_delta = i32::MIN;
        }
        if s.dts_sync_point >= 0 {
            // avctx->pkt_timebase is the stream's 1/90000.
            let den = time_base.den.saturating_mul(1);
            if den > 0 {
                let num = time_base.num.saturating_mul(90_000);
                if s.dts != NOPTS {
                    self.reference_dts =
                        s.dts
                            .saturating_sub(rescale(i64::from(s.dts_ref_dts_delta), num, den));
                } else if self.reference_dts != NOPTS {
                    s.dts = self.reference_dts.saturating_add(rescale(
                        i64::from(s.dts_ref_dts_delta),
                        num,
                        den,
                    ));
                }
                if self.reference_dts != NOPTS && s.pts == NOPTS {
                    let pts_dts_delta = rescale(i64::from(s.pts_dts_delta), num, den);
                    let pts = (s.dts as u64).wrapping_add(pts_dts_delta as u64);
                    if pts == s.dts.saturating_add(pts_dts_delta) as u64 {
                        s.pts = pts as i64;
                    }
                }
                if s.dts_sync_point > 0 {
                    self.reference_dts = s.dts;
                }
            }
        }
        Ok((next, Some(unit)))
    }

    /// parse_nal_units.
    fn parse_nal_units(&mut self, s: &mut ParserState, avctx: &mut CodecCtx, buf: &[u8]) {
        s.pict_type = pict::I;
        s.key_frame = 0;
        s.picture_structure = STRUCT_UNKNOWN;
        self.sei = Sei::reset();
        self.picture = None;
        if buf.is_empty() {
            return;
        }
        let size = buf.len();
        let mut buf_index = 0usize;
        loop {
            buf_index = find_nal_start(buf, buf_index);
            if buf_index >= size {
                break;
            }
            let mut src_length = size - buf_index;
            let state = buf[buf_index];
            let nal_type = state & 0x1F;
            if matches!(nal_type, NAL_SLICE | NAL_IDR | NAL_DPA) {
                if nal_type == NAL_IDR || (state >> 5) & 3 == 0 {
                    src_length = src_length.min(60);
                } else {
                    src_length = src_length.min(1000);
                }
            }
            let (consumed, nal) = extract_rbsp(&buf[buf_index..buf_index + src_length]);
            buf_index += consumed;
            let mut gb = BitReader::new(&nal);
            gb.skip(1);
            let ref_idc = gb.read(2);
            let nal_type = gb.read(5) as u8;
            match nal_type {
                NAL_SPS => {
                    if let Some((id, sps)) = decode_sps(&mut gb) {
                        self.sps_list[id] = Some(Arc::new(sps));
                    }
                }
                NAL_PPS => {
                    if let Some((id, pps)) = decode_pps(&mut gb, &self.sps_list) {
                        self.pps_list[id] = Some(Arc::new(pps));
                    }
                }
                NAL_SEI => self.decode_sei(&nal[1.min(nal.len())..]),
                NAL_IDR | NAL_SLICE | NAL_DPA => {
                    if nal_type == NAL_IDR {
                        s.key_frame = 1;
                        self.poc.prev_frame_num = 0;
                        self.poc.prev_frame_num_offset = 0;
                        self.poc.prev_poc_msb = 0;
                        self.poc.prev_poc_lsb = 0;
                    }
                    self.slice(s, avctx, &mut gb, nal_type, ref_idc);
                    return;
                }
                _ => {}
            }
        }
    }

    /// The slice-header part of parse_nal_units: everything after the
    /// first slice's NAL header. Returns early where FFmpeg jumps to fail.
    fn slice(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        gb: &mut BitReader<'_>,
        nal_type: u8,
        ref_idc: u32,
    ) {
        gb.ue_long(); // first_mb_in_slice
        let slice_type = gb.ue31();
        s.pict_type = GOLOMB_TO_PICT_TYPE[(slice_type as u32 % 5) as usize];
        if self.sei.recovery_frame_cnt >= 0 {
            s.key_frame = 1;
        }
        let pps_id = gb.ue() as u32;
        if pps_id as usize >= MAX_PPS_COUNT {
            return;
        }
        let Some(pps) = self.pps_list[pps_id as usize].clone() else {
            return;
        };
        let sps = Arc::clone(&pps.sps);
        self.active_sps = Some(Arc::clone(&sps));
        if sps.ref_frame_count <= 1 && pps.ref_count[0] <= 1 && s.pict_type == pict::I {
            s.key_frame = 1;
        }
        self.poc.frame_num = gb.read(sps.log2_max_frame_num as u32) as i32;

        s.coded_width = 16 * sps.mb_width;
        s.coded_height = 16 * sps.mb_height;
        s.width = s.coded_width - (sps.crop_right + sps.crop_left);
        s.height = s.coded_height - (sps.crop_top + sps.crop_bottom);
        if s.width <= 0 || s.height <= 0 {
            s.width = s.coded_width;
            s.height = s.coded_height;
        }
        s.format = match (sps.bit_depth_luma, sps.chroma_format_idc) {
            (9, 3) => Some(PixFmt::Yuv444p9),
            (9, 2) => Some(PixFmt::Yuv422p9),
            (9, _) => Some(PixFmt::Yuv420p9),
            (10, 3) => Some(PixFmt::Yuv444p10),
            (10, 2) => Some(PixFmt::Yuv422p10),
            (10, _) => Some(PixFmt::Yuv420p10),
            (8, 3) => Some(PixFmt::Yuv444p),
            (8, 2) => Some(PixFmt::Yuv422p),
            (8, _) => Some(PixFmt::Yuv420p),
            _ => None,
        };

        if sps.frame_mbs_only_flag {
            self.picture_structure = PICT_FRAME;
        } else if gb.read1() {
            self.picture_structure = PICT_TOP_FIELD + i32::from(gb.read1());
        } else {
            self.picture_structure = PICT_FRAME;
        }
        if nal_type == NAL_IDR {
            gb.ue_long(); // idr_pic_id
        }
        if sps.poc_type == 0 {
            self.poc.poc_lsb = gb.read(sps.log2_max_poc_lsb as u32) as i32;
            if pps.pic_order_present && self.picture_structure == PICT_FRAME {
                self.poc.delta_poc_bottom = gb.se();
            }
        }
        if sps.poc_type == 1 && !sps.delta_pic_order_always_zero_flag {
            self.poc.delta_poc[0] = gb.se();
            if pps.pic_order_present && self.picture_structure == PICT_FRAME {
                self.poc.delta_poc[1] = gb.se();
            }
        }
        let mut field_poc = [i32::MAX; 2];
        if !init_poc(
            &mut field_poc,
            &sps,
            &mut self.poc,
            self.picture_structure,
            ref_idc,
        ) {
            return;
        }
        let mut got_reset = false;
        if ref_idc != 0 && nal_type != NAL_IDR {
            match scan_mmco_reset(gb, &pps, s.pict_type, self.picture_structure) {
                Some(reset) => got_reset = reset,
                None => return,
            }
        }
        // The picture the decoder completes: a frame, or the second field
        // of a pair (same frame_num, the other parity; h264_field_start).
        let poc = field_poc[0].min(field_poc[1]);
        let idr = nal_type == NAL_IDR;
        let b = s.pict_type == pict::B;
        self.picture = if self.picture_structure == PICT_FRAME {
            self.pending_field = None;
            Some(OutputPicture {
                poc,
                b,
                idr,
                mmco_reset: got_reset,
            })
        } else {
            match self.pending_field.take() {
                Some((first, structure, frame_num, first_idr))
                    if structure != self.picture_structure && frame_num == self.poc.frame_num =>
                {
                    Some(OutputPicture {
                        poc: first.min(poc),
                        b,
                        idr: idr || first_idr,
                        mmco_reset: got_reset,
                    })
                }
                _ => {
                    self.pending_field =
                        Some((poc, self.picture_structure, self.poc.frame_num, idr));
                    None
                }
            }
        };
        self.poc.prev_frame_num = if got_reset { 0 } else { self.poc.frame_num };
        self.poc.prev_frame_num_offset = if got_reset {
            0
        } else {
            self.poc.frame_num_offset
        };
        if ref_idc != 0 {
            if !got_reset {
                self.poc.prev_poc_msb = self.poc.poc_msb;
                self.poc.prev_poc_lsb = self.poc.poc_lsb;
            } else {
                self.poc.prev_poc_msb = 0;
                self.poc.prev_poc_lsb = if self.picture_structure == PICT_BOTTOM_FIELD {
                    0
                } else {
                    field_poc[0]
                };
            }
        }

        if self.sei.timing_present && !self.process_picture_timing(&sps) {
            self.sei.timing_present = false;
        }
        let timing = sps.pic_struct_present_flag && self.sei.timing_present;
        s.repeat_pict = if timing {
            match self.sei.pic_struct {
                1 | 2 => 0,
                0 | 3 | 4 => 1,
                5 | 6 => 2,
                7 => 3,
                8 => 5,
                _ => i32::from(self.picture_structure == PICT_FRAME),
            }
        } else {
            i32::from(self.picture_structure == PICT_FRAME)
        };
        if self.picture_structure == PICT_FRAME {
            s.picture_structure = STRUCT_FRAME;
        } else {
            s.picture_structure = if self.picture_structure == PICT_TOP_FIELD {
                STRUCT_TOP
            } else {
                STRUCT_BOTTOM
            };
            self.last_picture_structure = s.picture_structure;
            self.last_frame_num = self.poc.frame_num;
        }
        if sps.timing_info_present_flag {
            let mut den = i64::from(sps.time_scale);
            if (self.sei.x264_build as u32) < 44 {
                den *= 2;
            }
            let r = reduce(
                i64::from(sps.num_units_in_tick.wrapping_mul(2)),
                den,
                1 << 30,
            );
            avctx.framerate = Q {
                num: r.den,
                den: r.num,
            };
        }
    }

    /// ff_h264_sei_process_picture_timing: the delays and pic_struct.
    fn process_picture_timing(&mut self, sps: &Sps) -> bool {
        let payload = std::mem::take(&mut self.sei.timing_payload);
        let mut gb = BitReader::new(&payload);
        if sps.nal_hrd_parameters_present_flag || sps.vcl_hrd_parameters_present_flag {
            self.sei.cpb_removal_delay = i64::from(gb.read(sps.cpb_removal_delay_length));
            self.sei.dpb_output_delay = i64::from(gb.read(sps.dpb_output_delay_length));
        }
        let mut ok = true;
        if sps.pic_struct_present_flag {
            self.sei.pic_struct = gb.read(4);
            ok = self.sei.pic_struct <= PIC_STRUCT_TRIPLING;
        }
        self.sei.timing_payload = payload;
        ok
    }

    /// ff_h264_sei_decode over the SEI RBSP after its NAL header byte.
    fn decode_sei(&mut self, data: &[u8]) {
        let mut at = 0usize;
        while data.len() - at > 2 && (data[at] != 0 || data[at + 1] != 0) {
            let mut payload_type = 0usize;
            loop {
                let Some(&b) = data.get(at) else { return };
                payload_type += usize::from(b);
                at += 1;
                if b != 255 {
                    break;
                }
            }
            let mut size = 0usize;
            loop {
                let Some(&b) = data.get(at) else { return };
                size += usize::from(b);
                at += 1;
                if b != 255 {
                    break;
                }
            }
            if size > data.len() - at {
                return;
            }
            let payload = &data[at..at + size];
            let ok = match payload_type {
                // SEI_TYPE_PIC_TIMING
                1 => {
                    if size > 40 {
                        false
                    } else {
                        self.sei.timing_payload = payload.to_vec();
                        self.sei.timing_present = true;
                        true
                    }
                }
                // SEI_TYPE_RECOVERY_POINT
                6 => {
                    let mut gb = BitReader::new(payload);
                    let cnt = gb.ue_long();
                    if cnt >= 1 << MAX_LOG2_MAX_FRAME_NUM {
                        false
                    } else {
                        self.sei.recovery_frame_cnt = cnt as i32;
                        true
                    }
                }
                // SEI_TYPE_BUFFERING_PERIOD: AVERROR_PS_NOT_FOUND keeps going.
                0 => {
                    let mut gb = BitReader::new(payload);
                    let sps_id = gb.ue31();
                    if sps_id > 31 {
                        false
                    } else {
                        if self.sps_list[sps_id as usize].is_some() {
                            self.sei.buffering_period_present = true;
                        }
                        true
                    }
                }
                // SEI_TYPE_USER_DATA_UNREGISTERED
                5 => {
                    if size < 16 {
                        false
                    } else {
                        if let Some(build) = x264_build(&payload[16..]) {
                            self.sei.x264_build = build;
                        }
                        true
                    }
                }
                // Mastering display, content light level, ambient viewing
                // environment, alternative transfer: their size checks.
                137 => size >= 24,
                144 => size >= 4,
                148 => size >= 8,
                147 => size >= 1,
                _ => true,
            };
            if !ok {
                return;
            }
            at += size;
        }
    }
}

/// The `x264 - core %d` user data of x264's SEI.
fn x264_build(text: &[u8]) -> Option<i32> {
    let text = &text[..text.iter().position(|&b| b == 0).unwrap_or(text.len())];
    let mut rest = text;
    let skip_ws = |r: &mut &[u8]| {
        while let [b' ' | b'\t' | b'\n' | b'\r' | 0x0B | 0x0C, tail @ ..] = *r {
            *r = tail;
        }
    };
    for (lit, ws) in [(&b"x264"[..], true), (b"-", true), (b"core", true)] {
        rest = rest.strip_prefix(lit)?;
        if ws {
            skip_ws(&mut rest);
        }
    }
    let (negative, digits) = match rest {
        [b'-', d @ ..] => (true, d),
        [b'+', d @ ..] => (false, d),
        d => (false, d),
    };
    let n = digits.iter().take_while(|b| b.is_ascii_digit()).count();
    if n == 0 {
        return None;
    }
    let mut v: i64 = 0;
    for &d in &digits[..n] {
        v = (v * 10 + i64::from(d - b'0')).min(i64::from(u32::MAX) + 1);
    }
    let build = if negative { -v } else { v } as i32;
    let build = if build == 1 && text.starts_with(b"x264 - core 0000") {
        67
    } else {
        build
    };
    (build > 0).then_some(build)
}

/// extract_extradata_h2645 for H.264 (libavcodec/bsf/extract_extradata.c):
/// the SPS and PPS NAL units of `unit`, each behind a four-byte start
/// code, if it has an SPS.
pub(crate) fn extract_extradata(unit: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let mut has_sps = false;
    let mut at = 0;
    loop {
        at = find_nal_start(unit, at);
        if at >= unit.len() {
            break;
        }
        let (consumed, _) = extract_rbsp(&unit[at..]);
        let raw = &unit[at..at + consumed];
        at += consumed.max(1);
        match raw.first().map(|h| h & 0x1F) {
            Some(NAL_SPS) => has_sps = true,
            Some(NAL_PPS) => {}
            _ => continue,
        }
        out.extend_from_slice(&[0, 0, 0, 1]);
        out.extend_from_slice(raw);
    }
    has_sps.then_some(out)
}

/// ff_startcode_find_candidate_c: the first zero byte.
fn first_zero(buf: &[u8]) -> usize {
    buf.iter().position(|&b| b == 0).unwrap_or(buf.len())
}

/// The parser's find_start_code: the index of the next NAL header byte
/// at or after `from`, or `buf.len()`.
pub(crate) fn find_nal_start(buf: &[u8], from: usize) -> usize {
    let mut state = u32::MAX;
    let after = find_start_code(buf, from, buf.len() + 1, &mut state);
    (after - 1).min(buf.len())
}

/// ff_h2645_extract_rbsp (small padding): the bytes consumed and the NAL
/// with emulation prevention removed, ending at the next start code.
pub(crate) fn extract_rbsp(src: &[u8]) -> (usize, Cow<'_, [u8]>) {
    let mut length = src.len();
    let mut i = 0usize;
    let mut escaped = false;
    while i + 2 < length {
        if src[i] == 0 && src[i + 1] == 0 && (src[i + 2] == 3 || src[i + 2] == 1) {
            if src[i + 2] == 1 {
                length = i;
            } else {
                escaped = true;
            }
            break;
        }
        i += 1;
    }
    if !escaped {
        return (length, Cow::Borrowed(&src[..length]));
    }
    let mut dst = Vec::with_capacity(length);
    dst.extend_from_slice(&src[..i]);
    let mut si = i;
    while si + 2 < length {
        if src[si + 2] > 3 {
            dst.push(src[si]);
            dst.push(src[si + 1]);
            si += 2;
        } else if src[si] == 0 && src[si + 1] == 0 && src[si + 2] != 0 {
            if src[si + 2] == 3 {
                dst.push(0);
                dst.push(0);
                si += 3;
                continue;
            }
            return (si, Cow::Owned(dst));
        } else {
            dst.push(src[si]);
            si += 1;
        }
    }
    dst.extend_from_slice(&src[si..length]);
    (length, Cow::Owned(dst))
}

/// av_image_check_size.
fn image_size_ok(w: i64, h: i64) -> bool {
    w > 0 && h > 0 && ((w + 128) as u64) * ((h + 128) as u64) < (i32::MAX / 8) as u64
}

/// decode_scaling_list (values are not kept: the parser never uses them).
fn scaling_list(gb: &mut BitReader<'_>, size: usize) -> bool {
    if !gb.read1() {
        return true;
    }
    let (mut last, mut next) = (8i32, 8i32);
    for i in 0..size {
        if next != 0 {
            let v = gb.se();
            if !(-128..=127).contains(&v) {
                return false;
            }
            next = (last + v) & 0xFF;
        }
        if i == 0 && next == 0 {
            break;
        }
        if next != 0 {
            last = next;
        }
    }
    true
}

/// decode_hrd_parameters: the lengths the timing SEI needs.
fn hrd(gb: &mut BitReader<'_>, sps: &mut Sps) -> bool {
    let cpb_count = gb.ue31() + 1;
    if cpb_count as u32 > 32 {
        return false;
    }
    gb.skip(8); // bit_rate_scale, cpb_size_scale
    for _ in 0..cpb_count {
        gb.ue_long();
        gb.ue_long();
        gb.skip(1);
    }
    gb.skip(5); // initial_cpb_removal_delay_length
    sps.cpb_removal_delay_length = gb.read(5) + 1;
    sps.dpb_output_delay_length = gb.read(5) + 1;
    gb.skip(5); // time_offset_length
    true
}

/// decode_vui_parameters.
fn vui(gb: &mut BitReader<'_>, sps: &mut Sps) -> bool {
    // ff_h2645_decode_common_vui_params
    if gb.read1() && gb.read(8) == 255 {
        gb.skip(32); // sar
    }
    if gb.read1() {
        gb.skip(1);
    }
    if gb.read1() {
        gb.skip(4);
        if gb.read1() {
            gb.skip(24);
        }
    }
    if gb.read1() {
        gb.ue31();
        gb.ue31();
    }

    if gb.peek(1) != 0 && gb.left() < 10 {
        return true;
    }
    sps.timing_info_present_flag = gb.read1();
    if sps.timing_info_present_flag {
        let num_units_in_tick = gb.read(32);
        let time_scale = gb.read(32);
        if num_units_in_tick == 0 || time_scale == 0 {
            sps.timing_info_present_flag = false;
        } else {
            sps.num_units_in_tick = num_units_in_tick;
            sps.time_scale = time_scale;
        }
        gb.skip(1); // fixed_frame_rate_flag
    }
    sps.nal_hrd_parameters_present_flag = gb.read1();
    if sps.nal_hrd_parameters_present_flag && !hrd(gb, sps) {
        return false;
    }
    sps.vcl_hrd_parameters_present_flag = gb.read1();
    if sps.vcl_hrd_parameters_present_flag && !hrd(gb, sps) {
        return false;
    }
    if sps.nal_hrd_parameters_present_flag || sps.vcl_hrd_parameters_present_flag {
        gb.skip(1); // low_delay_hrd_flag
    }
    sps.pic_struct_present_flag = gb.read1();
    if gb.left() == 0 {
        return true;
    }
    sps.bitstream_restriction_flag = gb.read1();
    if sps.bitstream_restriction_flag {
        gb.skip(1);
        for _ in 0..4 {
            gb.ue31();
        }
        sps.num_reorder_frames = gb.ue31();
        gb.ue31(); // max_dec_frame_buffering
        if gb.left() < 0 {
            sps.num_reorder_frames = 0;
            sps.bitstream_restriction_flag = false;
        }
        if sps.num_reorder_frames as u32 > 16 {
            return false;
        }
    }
    true
}

/// ff_h264_decode_seq_parameter_set (ignore_truncation = 0).
fn decode_sps(gb: &mut BitReader<'_>) -> Option<(usize, Sps)> {
    let profile_idc = gb.read(8) as i32;
    let mut constraint_set_flags = 0;
    for bit in 0..6 {
        constraint_set_flags |= i32::from(gb.read1()) << bit;
    }
    gb.skip(2);
    let level_idc = gb.read(8) as i32;
    let sps_id = gb.ue31() as u32;
    if sps_id as usize >= MAX_SPS_COUNT {
        return None;
    }
    let mut sps = Sps {
        profile_idc,
        level_idc,
        constraint_set_flags,
        chroma_format_idc: 1,
        bit_depth_luma: 8,
        log2_max_frame_num: 0,
        poc_type: 0,
        log2_max_poc_lsb: 0,
        delta_pic_order_always_zero_flag: false,
        offset_for_non_ref_pic: 0,
        offset_for_top_to_bottom_field: 0,
        offset_for_ref_frame: Vec::new(),
        ref_frame_count: 0,
        mb_width: 0,
        mb_height: 0,
        frame_mbs_only_flag: false,
        mb_aff: false,
        crop_left: 0,
        crop_right: 0,
        crop_top: 0,
        crop_bottom: 0,
        timing_info_present_flag: false,
        num_units_in_tick: 0,
        time_scale: 0,
        nal_hrd_parameters_present_flag: false,
        vcl_hrd_parameters_present_flag: false,
        cpb_removal_delay_length: 0,
        dpb_output_delay_length: 0,
        pic_struct_present_flag: false,
        bitstream_restriction_flag: false,
        num_reorder_frames: 0,
    };
    if matches!(
        profile_idc,
        100 | 110 | 122 | 244 | 44 | 83 | 86 | 118 | 128 | 138 | 144
    ) {
        sps.chroma_format_idc = gb.ue31();
        if sps.chroma_format_idc as u32 > 3 {
            return None;
        }
        if sps.chroma_format_idc == 3 && gb.read1() {
            return None; // separate colour planes
        }
        sps.bit_depth_luma = gb.ue31() + 8;
        let bit_depth_chroma = gb.ue31() + 8;
        if bit_depth_chroma != sps.bit_depth_luma || !(8..=14).contains(&sps.bit_depth_luma) {
            return None;
        }
        gb.skip(1); // transform_bypass
        if gb.read1() {
            let mut ok = true;
            for _ in 0..6 {
                ok &= scaling_list(gb, 16);
            }
            ok &= scaling_list(gb, 64);
            ok &= scaling_list(gb, 64);
            if sps.chroma_format_idc == 3 {
                for _ in 0..4 {
                    ok &= scaling_list(gb, 64);
                }
            }
            if !ok {
                return None;
            }
        }
    }
    let log2_max_frame_num_minus4 = gb.ue31();
    if !(0..=MAX_LOG2_MAX_FRAME_NUM - 4).contains(&log2_max_frame_num_minus4) {
        return None;
    }
    sps.log2_max_frame_num = log2_max_frame_num_minus4 + 4;
    sps.poc_type = gb.ue31();
    if sps.poc_type == 0 {
        let t = gb.ue31() as u32;
        if t > 12 {
            return None;
        }
        sps.log2_max_poc_lsb = t as i32 + 4;
    } else if sps.poc_type == 1 {
        sps.delta_pic_order_always_zero_flag = gb.read1();
        sps.offset_for_non_ref_pic = gb.se_long();
        sps.offset_for_top_to_bottom_field = gb.se_long();
        if sps.offset_for_non_ref_pic == i32::MIN || sps.offset_for_top_to_bottom_field == i32::MIN
        {
            return None;
        }
        let cycle = gb.ue() as u32;
        if cycle >= 256 {
            return None;
        }
        for _ in 0..cycle {
            let v = gb.se_long();
            if v == i32::MIN {
                return None;
            }
            sps.offset_for_ref_frame.push(v);
        }
    } else if sps.poc_type != 2 {
        return None;
    }
    sps.ref_frame_count = gb.ue31();
    if sps.ref_frame_count > MAX_DPB_FRAMES {
        return None;
    }
    gb.skip(1); // gaps_in_frame_num_allowed_flag
    sps.mb_width = gb.ue().wrapping_add(1);
    sps.mb_height = gb.ue().wrapping_add(1);
    sps.frame_mbs_only_flag = gb.read1();
    if sps.mb_height as u32 >= i32::MAX as u32 / 2 {
        return None;
    }
    sps.mb_height *= 2 - i32::from(sps.frame_mbs_only_flag);
    sps.mb_aff = !sps.frame_mbs_only_flag && gb.read1();
    if sps.mb_width as u32 >= i32::MAX as u32 / 16
        || sps.mb_height as u32 >= i32::MAX as u32 / 16
        || !image_size_ok(16 * i64::from(sps.mb_width), 16 * i64::from(sps.mb_height))
    {
        return None;
    }
    gb.skip(1); // direct_8x8_inference_flag
    if gb.read1() {
        let crop_left = gb.ue() as u32;
        let crop_right = gb.ue() as u32;
        let crop_top = gb.ue() as u32;
        let crop_bottom = gb.ue() as u32;
        let width = 16 * i64::from(sps.mb_width);
        let height = 16 * i64::from(sps.mb_height);
        let vsub = u32::from(sps.chroma_format_idc == 1);
        let hsub = u32::from(sps.chroma_format_idc == 1 || sps.chroma_format_idc == 2);
        let step_x = 1u32 << hsub;
        let step_y = (2 - u32::from(sps.frame_mbs_only_flag)) << vsub;
        let max = i32::MAX as u32 / 4;
        if crop_left > max / step_x
            || crop_right > max / step_x
            || crop_top > max / step_y
            || crop_bottom > max / step_y
            || i64::from(crop_left + crop_right) * i64::from(step_x) >= width
            || i64::from(crop_top + crop_bottom) * i64::from(step_y) >= height
        {
            return None;
        }
        sps.crop_left = (crop_left * step_x) as i32;
        sps.crop_right = (crop_right * step_x) as i32;
        sps.crop_top = (crop_top * step_y) as i32;
        sps.crop_bottom = (crop_bottom * step_y) as i32;
    }
    if gb.read1() && !vui(gb, &mut sps) {
        return None;
    }
    if gb.left() < 0 {
        return None;
    }
    if !sps.bitstream_restriction_flag && sps.ref_frame_count != 0 {
        sps.num_reorder_frames = MAX_DPB_FRAMES - 1;
        if let Some(&(_, mbs)) = LEVEL_MAX_DPB_MBS
            .iter()
            .find(|(level, _)| *level == sps.level_idc)
        {
            sps.num_reorder_frames =
                (mbs / (sps.mb_width * sps.mb_height)).min(sps.num_reorder_frames);
        }
    }
    Some((sps_id as usize, sps))
}

/// ff_h264_decode_picture_parameter_set as the parser calls it: with a
/// zero bit length, so nothing past redundant_pic_cnt_present is read.
fn decode_pps(gb: &mut BitReader<'_>, sps_list: &[Option<Arc<Sps>>]) -> Option<(usize, Pps)> {
    let pps_id = gb.ue() as u32;
    if pps_id as usize >= MAX_PPS_COUNT {
        return None;
    }
    let sps_id = gb.ue31() as u32;
    let sps = sps_list.get(sps_id as usize)?.clone()?;
    if sps.bit_depth_luma > 14 || sps.bit_depth_luma == 11 || sps.bit_depth_luma == 13 {
        return None;
    }
    gb.skip(1); // cabac
    let pic_order_present = gb.read1();
    let slice_group_count = gb.ue().wrapping_add(1);
    if slice_group_count > 1 {
        return None; // FMO
    }
    let ref_count = [
        (gb.ue() as u32).wrapping_add(1),
        (gb.ue() as u32).wrapping_add(1),
    ];
    if ref_count[0].wrapping_sub(1) > 31 || ref_count[1].wrapping_sub(1) > 31 {
        return None;
    }
    let weighted_pred = gb.read1();
    let weighted_bipred_idc = gb.read(2);
    gb.se(); // init_qp
    gb.se(); // init_qs
    let chroma_qp_index_offset = gb.se();
    if !(-12..=12).contains(&chroma_qp_index_offset) {
        return None;
    }
    gb.skip(2); // deblocking_filter_parameters_present, constrained_intra_pred
    let redundant_pic_cnt_present = gb.read1();
    Some((
        pps_id as usize,
        Pps {
            pic_order_present,
            ref_count,
            weighted_pred,
            weighted_bipred_idc,
            redundant_pic_cnt_present,
            sps,
        },
    ))
}

/// ff_h264_init_poc: false where FFmpeg returns AVERROR_INVALIDDATA.
fn init_poc(
    pic_field_poc: &mut [i32; 2],
    sps: &Sps,
    pc: &mut Poc,
    picture_structure: i32,
    nal_ref_idc: u32,
) -> bool {
    let max_frame_num = 1i32 << sps.log2_max_frame_num;
    pc.frame_num_offset = pc.prev_frame_num_offset;
    if pc.frame_num < pc.prev_frame_num {
        pc.frame_num_offset = pc.frame_num_offset.wrapping_add(max_frame_num);
    }
    let field_poc: [i64; 2] = if sps.poc_type == 0 {
        let max_poc_lsb = 1i32 << sps.log2_max_poc_lsb;
        if pc.prev_poc_lsb < 0 {
            pc.prev_poc_lsb = pc.poc_lsb;
        }
        pc.poc_msb = if pc.poc_lsb < pc.prev_poc_lsb
            && pc.prev_poc_lsb - pc.poc_lsb >= max_poc_lsb / 2
        {
            pc.prev_poc_msb.wrapping_add(max_poc_lsb)
        } else if pc.poc_lsb > pc.prev_poc_lsb && pc.prev_poc_lsb - pc.poc_lsb < -max_poc_lsb / 2 {
            pc.prev_poc_msb.wrapping_sub(max_poc_lsb)
        } else {
            pc.prev_poc_msb
        };
        let top = i64::from(pc.poc_msb) + i64::from(pc.poc_lsb);
        let bottom = if picture_structure == PICT_FRAME {
            top + i64::from(pc.delta_poc_bottom)
        } else {
            top
        };
        [top, bottom]
    } else if sps.poc_type == 1 {
        let cycle = sps.offset_for_ref_frame.len() as i64;
        let mut abs_frame_num = if cycle != 0 {
            i64::from(pc.frame_num_offset) + i64::from(pc.frame_num)
        } else {
            0
        };
        if nal_ref_idc == 0 && abs_frame_num > 0 {
            abs_frame_num -= 1;
        }
        let expected_delta_per_poc_cycle: i64 =
            sps.offset_for_ref_frame.iter().map(|&v| i64::from(v)).sum();
        let mut expectedpoc = if abs_frame_num > 0 {
            let poc_cycle_cnt = (abs_frame_num - 1) / cycle;
            let frame_num_in_poc_cycle = (abs_frame_num - 1) % cycle;
            let mut e = poc_cycle_cnt.wrapping_mul(expected_delta_per_poc_cycle);
            for i in 0..=frame_num_in_poc_cycle as usize {
                e = e.wrapping_add(i64::from(sps.offset_for_ref_frame[i]));
            }
            e
        } else {
            0
        };
        if nal_ref_idc == 0 {
            expectedpoc = expectedpoc.wrapping_add(i64::from(sps.offset_for_non_ref_pic));
        }
        let top = expectedpoc.wrapping_add(i64::from(pc.delta_poc[0]));
        let mut bottom = top.wrapping_add(i64::from(sps.offset_for_top_to_bottom_field));
        if picture_structure == PICT_FRAME {
            bottom = bottom.wrapping_add(i64::from(pc.delta_poc[1]));
        }
        [top, bottom]
    } else {
        let mut poc = 2 * (i64::from(pc.frame_num_offset) + i64::from(pc.frame_num));
        if nal_ref_idc == 0 {
            poc -= 1;
        }
        [poc, poc]
    };
    let (Ok(top), Ok(bottom)) = (i32::try_from(field_poc[0]), i32::try_from(field_poc[1])) else {
        return false;
    };
    if picture_structure != PICT_BOTTOM_FIELD {
        pic_field_poc[0] = top;
    }
    if picture_structure != PICT_TOP_FIELD {
        pic_field_poc[1] = bottom;
    }
    true
}

/// ff_h264_parse_ref_count: `None` where FFmpeg fails.
fn parse_ref_count(
    gb: &mut BitReader<'_>,
    pps: &Pps,
    slice_type_nos: i32,
    picture_structure: i32,
) -> Option<(usize, [u32; 2])> {
    let mut ref_count = pps.ref_count;
    if slice_type_nos == pict::I {
        return Some((0, [0, 0]));
    }
    let max = if picture_structure == PICT_FRAME {
        15u32
    } else {
        31
    };
    if gb.read1() {
        ref_count[0] = (gb.ue() as u32).wrapping_add(1);
        ref_count[1] = if slice_type_nos == pict::B {
            (gb.ue() as u32).wrapping_add(1)
        } else {
            1
        };
    }
    let list_count = if slice_type_nos == pict::B { 2 } else { 1 };
    if ref_count[0].wrapping_sub(1) > max || (list_count == 2 && ref_count[1].wrapping_sub(1) > max)
    {
        return None;
    }
    if ref_count[1].wrapping_sub(1) > max {
        ref_count[1] = 0;
    }
    Some((list_count, ref_count))
}

/// ff_h264_pred_weight_table, reading only: `false` on an out-of-range
/// weight.
fn pred_weight_table(
    gb: &mut BitReader<'_>,
    sps: &Sps,
    ref_count: [u32; 2],
    slice_type_nos: i32,
) -> bool {
    gb.ue31(); // luma_log2_weight_denom
    if sps.chroma_format_idc != 0 {
        gb.ue31();
    }
    for &count in &ref_count {
        for _ in 0..count {
            if gb.read1() {
                let (w, o) = (gb.se(), gb.se());
                if w != i32::from(w as i8) || o != i32::from(o as i8) {
                    return false;
                }
            }
            if sps.chroma_format_idc != 0 && gb.read1() {
                for _ in 0..2 {
                    let (w, o) = (gb.se(), gb.se());
                    if w != i32::from(w as i8) || o != i32::from(o as i8) {
                        return false;
                    }
                }
            }
        }
        if slice_type_nos != pict::B {
            break;
        }
    }
    true
}

/// scan_mmco_reset: `Some(true)` for MMCO_RESET, `None` on error.
fn scan_mmco_reset(
    gb: &mut BitReader<'_>,
    pps: &Pps,
    pict_type: i32,
    picture_structure: i32,
) -> Option<bool> {
    let slice_type_nos = pict_type & 3;
    if pps.redundant_pic_cnt_present {
        gb.ue();
    }
    if slice_type_nos == pict::B {
        gb.skip(1); // direct_spatial_mv_pred
    }
    let (list_count, ref_count) = parse_ref_count(gb, pps, slice_type_nos, picture_structure)?;
    if slice_type_nos != pict::I {
        for &count in &ref_count[..list_count] {
            if gb.read1() {
                let mut index = 0u32;
                loop {
                    let idc = gb.ue31() as u32;
                    if idc < 3 {
                        gb.ue_long();
                    } else if idc > 3 {
                        return None;
                    } else {
                        break;
                    }
                    if index >= count {
                        return None;
                    }
                    index += 1;
                }
            }
        }
    }
    if (pps.weighted_pred && slice_type_nos == pict::P)
        || (pps.weighted_bipred_idc == 1 && slice_type_nos == pict::B)
    {
        // Out-of-range weights fail there too, before the MMCOs.
        let _ = pred_weight_table(gb, &pps.sps, ref_count, slice_type_nos);
    }
    if gb.read1() {
        for _ in 0..MAX_MMCO_COUNT {
            if gb.left() < 1 {
                return None;
            }
            let opcode = gb.ue31() as u32;
            if opcode > 6 {
                return None;
            }
            match opcode {
                0 => return Some(false),
                5 => return Some(true),
                _ => {}
            }
            if opcode == 1 || opcode == 3 {
                gb.ue_long();
            }
            if matches!(opcode, 2 | 3 | 4 | 6) {
                gb.ue31();
            }
        }
    }
    Some(false)
}

impl std::fmt::Debug for H264Parser {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("H264Parser").finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_codes_and_rbsp() {
        let au = [
            0, 0, 0, 1, 9, 0xF0, 0, 0, 1, 0x67, 0, 0, 3, 1, 0, 0, 1, 0x68,
        ];
        assert_eq!(find_nal_start(&au, 0), 4);
        assert_eq!(find_nal_start(&au, 5), 9);
        assert_eq!(find_nal_start(&au, 10), 17);
        assert_eq!(find_nal_start(&au, 17), au.len());
        let (consumed, nal) = extract_rbsp(&au[9..]);
        assert_eq!((consumed, &nal[..]), (5, &[0x67, 0, 0, 1][..]));
        let (consumed, nal) = extract_rbsp(&au[4..]);
        assert_eq!((consumed, &nal[..]), (2, &[9, 0xF0][..]));
    }

    #[test]
    fn x264_user_data() {
        assert_eq!(x264_build(b"x264 - core 164 r3108"), Some(164));
        assert_eq!(x264_build(b"x264 - core 0000"), None);
        assert_eq!(x264_build(b"x264 - core 1"), Some(1));
        assert_eq!(x264_build(b"x264 -core  42"), Some(42));
        assert_eq!(x264_build(b"Lavc"), None);
    }
}
