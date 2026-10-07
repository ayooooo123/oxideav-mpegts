// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/hevc/parser.c with what it calls: the
// VPS, SPS and PPS readers of libavcodec/hevc/ps.c (as far as the parser
// uses them), the picture timing and active parameter sets SEI of
// libavcodec/hevc/sei.c, the common VUI of libavcodec/h2645_vui.c, and the
// HEVC NAL header and bit length of libavcodec/h2645_parse.c.
// Copyright (C) 2012 - 2013 Guillaume Martres (parser.c, ps.c, sei.c)
// Copyright (C) 2012 - 2013 Mickael Raulet (ps.c)
// Copyright (C) 2012 - 2013 Gildas Cocherel (ps.c, sei.c)
// Copyright (C) 2013 Vittorio Giovara (ps.c, sei.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

use std::sync::Arc;

use super::bits::BitReader;
use super::h264::{extract_rbsp, find_nal_start};
use super::parser::{
    pict, reduce, CodecCtx, Overflow, ParseContext, ParserState, PixFmt, END_NOT_FOUND, Q,
};

const START_CODE: u64 = 0x000001;
const NAL_RASL_R: u32 = 9;
const NAL_BLA_W_LP: u32 = 16;
const NAL_CRA_NUT: u32 = 21;
const NAL_VPS: u32 = 32;
const NAL_SPS: u32 = 33;
const NAL_PPS: u32 = 34;
const NAL_EOB_NUT: u32 = 37;
const NAL_SEI_PREFIX: u32 = 39;
const NAL_SEI_SUFFIX: u32 = 40;

const MAX_VPS: usize = 16;
const MAX_SPS: usize = 16;
const MAX_PPS: usize = 64;
const MAX_SUB_LAYERS: u32 = 7;
const MAX_DPB_SIZE: u32 = 16;
const MAX_SHORT_TERM_RPS: u32 = 64;
const MAX_LONG_TERM_REF_PICS: u32 = 32;
const MAX_REFS: u32 = 16;

/// AV_PICTURE_STRUCTURE_* and HEVC_SEI_PIC_STRUCT_FRAME_DOUBLING/TRIPLING.
const STRUCT_UNKNOWN: i32 = 0;
const STRUCT_TOP: i32 = 1;
const STRUCT_BOTTOM: i32 = 2;
const STRUCT_FRAME_DOUBLING: i32 = 7;
const STRUCT_FRAME_TRIPLING: i32 = 8;

struct Vps {
    /// The NAL bytes: a repeat of a stored VPS keeps it.
    data: Vec<u8>,
    max_sub_layers: u32,
    /// vps_num_units_in_tick and vps_time_scale, when stated.
    timing: Option<(u32, u32)>,
}

struct Sps {
    /// The NAL bytes: a repeat keeps the stored SPS (compare_sps).
    data: Vec<u8>,
    vps: Arc<Vps>,
    width: i32,
    height: i32,
    /// The output window: conformance plus default display window.
    window: [i32; 4],
    format: Option<PixFmt>,
    ctb_width: u32,
    ctb_height: u32,
    separate_colour_plane: bool,
    vui_timing: Option<(u32, u32)>,
    frame_field_info_present: bool,
    /// The highest sub-layer's num_reorder_pics: the has_b_frames
    /// FFmpeg's decoder sets when it activates the SPS.
    num_reorder: i32,
}

struct Pps {
    sps_id: usize,
    sps: Arc<Sps>,
    dependent_slice_segments_enabled: bool,
    output_flag_present: bool,
    num_extra_slice_header_bits: u32,
}

/// HEVCParserContext.
pub(crate) struct HevcParser {
    pc: ParseContext,
    vps_list: Vec<Option<Arc<Vps>>>,
    sps_list: Vec<Option<Arc<Sps>>>,
    pps_list: Vec<Option<Arc<Pps>>>,
    /// HEVCSEI.active_seq_parameter_set_id and picture_timing: kept from
    /// one unit to the next (ff_hevc_reset_sei leaves them).
    active_sps: usize,
    picture_struct: i32,
    parsed_extradata: bool,
    /// num_reorder_pics of the SPS the last slice header used.
    pub active_reorder: Option<i32>,
}

impl HevcParser {
    pub fn new() -> Self {
        Self {
            pc: ParseContext::new(),
            vps_list: (0..MAX_VPS).map(|_| None).collect(),
            sps_list: (0..MAX_SPS).map(|_| None).collect(),
            pps_list: (0..MAX_PPS).map(|_| None).collect(),
            active_sps: 0,
            picture_struct: STRUCT_UNKNOWN,
            parsed_extradata: false,
            active_reorder: None,
        }
    }

    /// hevc_find_frame_end.
    fn find_frame_end(&mut self, buf: &[u8]) -> i64 {
        let pc = &mut self.pc;
        for (i, &b) in buf.iter().enumerate() {
            pc.state64 = (pc.state64 << 8) | u64::from(b);
            if (pc.state64 >> 24) & 0xFF_FFFF != START_CODE {
                continue;
            }
            let nut = ((pc.state64 >> 17) & 0x3F) as u32;
            let layer_id = (pc.state64 >> 11) & 0x3F;
            if layer_id > 0 {
                continue;
            }
            let back = |pc: &ParseContext| {
                if (pc.state64 >> 48) & 0xFF == 0 {
                    i as i64 - 6
                } else {
                    i as i64 - 5
                }
            };
            if (NAL_VPS..=NAL_EOB_NUT).contains(&nut)
                || nut == NAL_SEI_PREFIX
                || (41..=44).contains(&nut)
                || (48..=55).contains(&nut)
            {
                if pc.frame_start_found != 0 {
                    pc.frame_start_found = 0;
                    return back(pc);
                }
            } else if nut <= NAL_RASL_R || (NAL_BLA_W_LP..=NAL_CRA_NUT).contains(&nut) {
                let first_slice_segment_in_pic = b >> 7 != 0;
                if first_slice_segment_in_pic {
                    if pc.frame_start_found == 0 {
                        pc.frame_start_found = 1;
                    } else {
                        pc.frame_start_found = 0;
                        return back(pc);
                    }
                }
            }
        }
        END_NOT_FOUND
    }

    /// hevc_parse.
    pub fn parse(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        buf: &[u8],
    ) -> Result<(i64, Option<Vec<u8>>), Overflow> {
        if !avctx.extradata.is_empty() && !self.parsed_extradata {
            // ff_hevc_decode_extradata for Annex B extradata: its NAL units
            // as in-band ones, without a picture.
            let extradata = std::mem::take(&mut avctx.extradata);
            let mut scratch = s.clone();
            self.parse_nal_units(&mut scratch, avctx, &extradata);
            avctx.extradata = extradata;
            self.parsed_extradata = true;
        }
        let next = self.find_frame_end(buf);
        let Some(unit) = self.pc.combine(next, buf)? else {
            return Ok((buf.len() as i64, None));
        };
        if !unit.is_empty() {
            self.parse_nal_units(s, avctx, &unit);
        }
        Ok((next, Some(unit)))
    }

    /// parse_nal_units: the parameter sets and SEI of the unit, then its
    /// first slice header.
    fn parse_nal_units(&mut self, s: &mut ParserState, avctx: &mut CodecCtx, buf: &[u8]) {
        s.pict_type = pict::I;
        s.key_frame = 0;
        s.picture_structure = STRUCT_UNKNOWN;
        let mut at = 0;
        loop {
            at = find_nal_start(buf, at);
            if at >= buf.len() {
                return;
            }
            let (consumed, nal) = extract_rbsp(&buf[at..]);
            at += consumed.max(1);
            // hevc_parse_nal_header: forbidden_zero_bit, type, layer id,
            // temporal id plus one (not 0).
            if nal.len() < 2 || nal[0] & 0x80 != 0 || nal[1] & 7 == 0 {
                continue;
            }
            let nal_type = u32::from(nal[0] >> 1) & 0x3F;
            let layer_id = (u32::from(nal[0] & 1) << 5) | u32::from(nal[1] >> 3);
            let temporal_id = u32::from(nal[1] & 7) - 1;
            if layer_id > 0 {
                continue;
            }
            let mut gb = BitReader::with_bits(&nal, bit_length(&nal));
            gb.skip(16);
            match nal_type {
                NAL_VPS => self.decode_vps(&mut gb, &nal),
                NAL_SPS => self.decode_sps(&mut gb, &nal),
                NAL_PPS => self.decode_pps(&mut gb),
                NAL_SEI_PREFIX | NAL_SEI_SUFFIX => self.decode_sei(&nal[2..], nal_type),
                0..=9 | 16..=21 => {
                    if self.picture_struct == STRUCT_FRAME_DOUBLING {
                        s.repeat_pict = 1;
                    } else if self.picture_struct == STRUCT_FRAME_TRIPLING {
                        s.repeat_pict = 2;
                    }
                    // A slice header read or rejected ends the unit.
                    self.slice_header(s, avctx, &mut gb, nal_type, temporal_id);
                    return;
                }
                _ => {}
            }
        }
    }

    /// hevc_parse_slice_header, as far as the parser reads it.
    fn slice_header(
        &mut self,
        s: &mut ParserState,
        avctx: &mut CodecCtx,
        gb: &mut BitReader<'_>,
        nal_type: u32,
        _temporal_id: u32,
    ) {
        let first_slice_in_pic = gb.read1();
        s.picture_structure = self.picture_struct;
        if (16..=23).contains(&nal_type) {
            s.key_frame = 1;
            gb.skip(1); // no_output_of_prior_pics_flag
        }
        let pps_id = gb.ue_long() as usize;
        let Some(pps) = self.pps_list.get(pps_id).and_then(Clone::clone) else {
            return;
        };
        let sps = Arc::clone(&pps.sps);
        self.active_reorder = Some(sps.num_reorder);
        s.coded_width = sps.width;
        s.coded_height = sps.height;
        s.width = sps.width - sps.window[0] - sps.window[1];
        s.height = sps.height - sps.window[2] - sps.window[3];
        s.format = sps.format;
        let (num, den) = sps.vps.timing.or(sps.vui_timing).unwrap_or((0, 0));
        if num > 0 && den > 0 {
            let r = reduce(i64::from(num), i64::from(den), 1 << 30);
            avctx.framerate = Q {
                num: r.den,
                den: r.num,
            };
        }
        let dependent = if !first_slice_in_pic {
            let dependent = pps.dependent_slice_segments_enabled && gb.read1();
            let ctbs = sps.ctb_width * sps.ctb_height;
            let length = ceil_log2(ctbs);
            let address = if length > 0 { gb.read(length) } else { 0 };
            if address >= ctbs {
                return;
            }
            dependent
        } else {
            false
        };
        if dependent {
            return;
        }
        gb.skip(u64::from(pps.num_extra_slice_header_bits));
        let slice_type = gb.ue31();
        s.pict_type = match slice_type {
            0 => pict::B,
            1 => pict::P,
            2 => pict::I,
            _ => return,
        };
        if pps.output_flag_present {
            gb.skip(1);
        }
        if sps.separate_colour_plane {
            gb.skip(2);
        }
        // What follows (the picture order count) only numbers output.
    }

    /// ff_hevc_decode_nal_vps up to the timing information. A repeat of
    /// the stored VPS keeps it.
    fn decode_vps(&mut self, gb: &mut BitReader<'_>, data: &[u8]) {
        let id = gb.read(4) as usize;
        if self.vps_list[id].as_ref().is_some_and(|v| v.data == data) {
            return;
        }
        if !gb.read1() || !gb.read1() {
            return; // base layer not internal or not available
        }
        gb.skip(6); // vps_max_layers_minus1
        let max_sub_layers = gb.read(3) + 1;
        gb.skip(1);
        if gb.read(16) != 0xFFFF || max_sub_layers > MAX_SUB_LAYERS {
            return;
        }
        if !skip_ptl(gb, max_sub_layers) {
            return;
        }
        let ordering = gb.read1();
        let start = if ordering { 0 } else { max_sub_layers - 1 };
        for _ in start..max_sub_layers {
            let buffering = gb.ue_long().wrapping_add(1);
            gb.ue_long();
            gb.ue_long();
            if buffering > MAX_DPB_SIZE || buffering == 0 {
                return;
            }
        }
        let max_layer_id = u64::from(gb.read(6));
        let num_layer_sets = u64::from(gb.ue_long()) + 1;
        if !(1..=1024).contains(&num_layer_sets)
            || (num_layer_sets - 1) * (max_layer_id + 1) > gb.left().max(0) as u64
        {
            return;
        }
        gb.skip((num_layer_sets - 1) * (max_layer_id + 1));
        let timing = if gb.read1() {
            let num = gb.read(32);
            let den = gb.read(32);
            Some((num, den))
        } else {
            None
        };
        self.vps_list[id] = Some(Arc::new(Vps {
            data: data.to_vec(),
            max_sub_layers,
            timing,
        }));
    }

    /// ff_hevc_parse_sps as far as the parser uses it, with the checks
    /// that reject an SPS.
    fn decode_sps(&mut self, gb: &mut BitReader<'_>, data: &[u8]) {
        let vps_id = gb.read(4) as usize;
        let Some(vps) = self.vps_list[vps_id].clone() else {
            return;
        };
        let max_sub_layers = gb.read(3) + 1;
        if max_sub_layers > vps.max_sub_layers {
            return;
        }
        gb.skip(1); // temporal_id_nesting
        if !skip_ptl(gb, max_sub_layers) {
            return;
        }
        let sps_id = gb.ue_long() as usize;
        if sps_id >= MAX_SPS {
            return;
        }
        let mut chroma_format_idc = gb.ue_long();
        if chroma_format_idc > 3 {
            return;
        }
        let mut separate_colour_plane = false;
        if chroma_format_idc == 3 {
            separate_colour_plane = gb.read1();
        }
        if separate_colour_plane {
            chroma_format_idc = 0;
        }
        let width = gb.ue_long();
        let height = gb.ue_long();
        // av_image_check_size.
        if width == 0
            || height == 0
            || (u64::from(width) + 128) * (u64::from(height) + 128) >= (i32::MAX / 8) as u64
        {
            return;
        }
        let (width, height) = (width as i32, height as i32);
        let mut window = [0i32; 4];
        if gb.read1() {
            match read_window(gb, chroma_format_idc, width, height) {
                Some(w) => window = w,
                None => return,
            }
        }
        let bit_depth = gb.ue31() + 8;
        let bit_depth_chroma = gb.ue31() + 8;
        if bit_depth > 16
            || bit_depth_chroma > 16
            || (chroma_format_idc != 0 && bit_depth_chroma != bit_depth)
        {
            return;
        }
        let Some(format) = pixel_format(bit_depth as u32, chroma_format_idc) else {
            return;
        };
        let log2_max_poc_lsb = gb.ue_long() + 4;
        if log2_max_poc_lsb > 16 {
            return;
        }
        let ordering = gb.read1();
        let start = if ordering { 0 } else { max_sub_layers - 1 };
        let mut num_reorder = 0;
        for _ in start..max_sub_layers {
            let buffering = gb.ue_long().wrapping_add(1);
            let reorder = gb.ue_long();
            gb.ue_long();
            if buffering > MAX_DPB_SIZE {
                return;
            }
            if reorder > buffering.wrapping_sub(1) && reorder > MAX_DPB_SIZE - 1 {
                return;
            }
            num_reorder = reorder as i32;
        }
        let log2_min_cb_size = gb.ue_long().wrapping_add(3);
        let log2_diff_max_min_cb = gb.ue_long();
        let log2_min_tb_size = gb.ue_long().wrapping_add(2);
        let log2_diff_max_min_tb = gb.ue_long();
        if !(3..=30).contains(&log2_min_cb_size)
            || log2_diff_max_min_cb > 30
            || log2_min_tb_size >= log2_min_cb_size
            || log2_min_tb_size < 2
            || log2_diff_max_min_tb > 30
        {
            return;
        }
        let depth_inter = gb.ue_long();
        let depth_intra = gb.ue_long();
        if gb.read1() && gb.read1() && !skip_scaling_list_data(gb) {
            return; // scaling_list_enabled, sps_scaling_list_data_present
        }
        gb.skip(2); // amp, sao
        if gb.read1() {
            // pcm
            let pcm_depth = gb.read(4) + 1;
            let pcm_depth_chroma = gb.read(4) + 1;
            gb.ue_long();
            gb.ue_long();
            if pcm_depth.max(pcm_depth_chroma) > bit_depth as u32 {
                return;
            }
            gb.skip(1);
        }
        let nb_st_rps = gb.ue_long();
        if nb_st_rps > MAX_SHORT_TERM_RPS {
            return;
        }
        let mut rps_delta_pocs: Vec<u32> = Vec::with_capacity(nb_st_rps as usize);
        for i in 0..nb_st_rps as usize {
            let Some(n) = skip_short_term_rps(gb, i, &rps_delta_pocs) else {
                return;
            };
            rps_delta_pocs.push(n);
        }
        if gb.read1() {
            // long_term_ref_pics_present
            let n = gb.ue_long();
            if n > MAX_LONG_TERM_REF_PICS {
                return;
            }
            for _ in 0..n {
                gb.skip(u64::from(log2_max_poc_lsb) + 1);
            }
        }
        gb.skip(2); // temporal_mvp, strong_intra_smoothing
        let mut vui = Vui::default();
        if gb.read1() {
            decode_vui(
                gb,
                &mut vui,
                chroma_format_idc,
                width,
                height,
                max_sub_layers,
            );
        }
        for (w, d) in window.iter_mut().zip(vui.def_disp_win) {
            *w += d;
        }
        if window[0] + window[1] >= width || window[2] + window[3] >= height {
            window = [0; 4]; // "Displaying the whole video surface."
        }
        let log2_ctb_size = log2_min_cb_size + log2_diff_max_min_cb;
        if !(4..=6).contains(&log2_ctb_size) {
            return;
        }
        let ctb = 1i32 << log2_ctb_size;
        let ctb_width = ((width + ctb - 1) >> log2_ctb_size) as u32;
        let ctb_height = ((height + ctb - 1) >> log2_ctb_size) as u32;
        let cb_mask = (1i32 << log2_min_cb_size) - 1;
        if width & cb_mask != 0 || height & cb_mask != 0 {
            return;
        }
        let tb_range = log2_ctb_size - log2_min_tb_size;
        if depth_inter > tb_range
            || depth_intra > tb_range
            || log2_diff_max_min_tb + log2_min_tb_size > log2_ctb_size.min(5)
        {
            return;
        }
        let sps = Sps {
            data: data.to_vec(),
            vps,
            width,
            height,
            window,
            format: Some(format),
            ctb_width,
            ctb_height,
            separate_colour_plane,
            vui_timing: vui.timing,
            frame_field_info_present: vui.frame_field_info_present,
            num_reorder,
        };
        // A repeat keeps the stored SPS; another one drops the PPS that
        // refer to its id (remove_sps).
        if self.sps_list[sps_id]
            .as_ref()
            .is_some_and(|old| old.data == sps.data)
        {
            return;
        }
        for pps in &mut self.pps_list {
            if pps.as_ref().is_some_and(|p| p.sps_id == sps_id) {
                *pps = None;
            }
        }
        self.sps_list[sps_id] = Some(Arc::new(sps));
    }

    /// ff_hevc_decode_nal_pps up to the fields the slice header needs.
    fn decode_pps(&mut self, gb: &mut BitReader<'_>) {
        let pps_id = gb.ue_long() as usize;
        if pps_id >= MAX_PPS {
            return;
        }
        let sps_id = gb.ue_long() as usize;
        let Some(sps) = self.sps_list.get(sps_id).and_then(Clone::clone) else {
            return;
        };
        let dependent_slice_segments_enabled = gb.read1();
        let output_flag_present = gb.read1();
        let num_extra_slice_header_bits = gb.read(3);
        self.pps_list[pps_id] = Some(Arc::new(Pps {
            sps_id,
            sps,
            dependent_slice_segments_enabled,
            output_flag_present,
            num_extra_slice_header_bits,
        }));
    }

    /// ff_hevc_decode_nal_sei: the picture timing and active parameter
    /// sets messages of a prefix SEI.
    fn decode_sei(&mut self, payload: &[u8], nal_type: u32) {
        let mut at = 0usize;
        loop {
            let mut payload_type = 0usize;
            loop {
                if payload.len().saturating_sub(at) < 2 {
                    return;
                }
                let byte = payload[at];
                at += 1;
                payload_type += usize::from(byte);
                if byte != 0xFF {
                    break;
                }
            }
            let mut payload_size = 0usize;
            loop {
                if payload.len().saturating_sub(at) < 1 + payload_size {
                    return;
                }
                let byte = payload[at];
                at += 1;
                payload_size += usize::from(byte);
                if byte != 0xFF {
                    break;
                }
            }
            if payload.len() - at < payload_size {
                return;
            }
            let message = &payload[at..at + payload_size];
            at += payload_size;
            if nal_type == NAL_SEI_PREFIX {
                let mut gb = BitReader::new(message);
                match payload_type {
                    1 => {
                        // decode_nal_sei_pic_timing.
                        let Some(sps) = self.sps_list[self.active_sps].clone() else {
                            return;
                        };
                        if sps.frame_field_info_present {
                            let pic_struct = gb.read(4);
                            self.picture_struct = match pic_struct {
                                2 | 10 | 12 => STRUCT_BOTTOM,
                                1 | 9 | 11 => STRUCT_TOP,
                                7 => STRUCT_FRAME_DOUBLING,
                                8 => STRUCT_FRAME_TRIPLING,
                                _ => STRUCT_UNKNOWN,
                            };
                        }
                    }
                    129 => {
                        // decode_nal_sei_active_parameter_sets.
                        gb.skip(4 + 1 + 1);
                        if gb.ue_long() > 15 {
                            return;
                        }
                        let id = gb.ue_long() as usize;
                        if id >= MAX_SPS {
                            return;
                        }
                        self.active_sps = id;
                    }
                    _ => {}
                }
            }
            if at >= payload.len() {
                return;
            }
        }
    }
}

/// extract_extradata_h2645 for HEVC: the VPS, SPS and PPS NAL units of
/// `unit`, each behind a four-byte start code, if it has a VPS and an SPS.
pub(crate) fn extract_extradata(unit: &[u8]) -> Option<Vec<u8>> {
    let mut out = Vec::new();
    let (mut has_vps, mut has_sps) = (false, false);
    let mut at = 0;
    loop {
        at = find_nal_start(unit, at);
        if at >= unit.len() {
            break;
        }
        let (consumed, _) = extract_rbsp(&unit[at..]);
        let raw = &unit[at..at + consumed];
        at += consumed.max(1);
        match raw.first().map(|h| u32::from(h >> 1) & 0x3F) {
            Some(NAL_VPS) => has_vps = true,
            Some(NAL_SPS) => has_sps = true,
            Some(NAL_PPS) => {}
            _ => continue,
        }
        out.extend_from_slice(&[0, 0, 0, 1]);
        out.extend_from_slice(raw);
    }
    (has_vps && has_sps).then_some(out)
}

/// av_ceil_log2.
fn ceil_log2(v: u32) -> u32 {
    if v <= 1 {
        0
    } else {
        32 - (v - 1).leading_zeros()
    }
}

/// get_bit_length: the NAL's bits without its trailing zero bytes, stop
/// bit and the zero bits after it.
fn bit_length(nal: &[u8]) -> u64 {
    let mut size = nal.len();
    while size > 0 && nal[size - 1] == 0 {
        size -= 1;
    }
    if size == 0 {
        return 0;
    }
    let v = nal[size - 1];
    // A NAL of at most min_size (2) bytes keeps all 16 bits.
    if size <= 2 {
        return 16;
    }
    let padding = v.trailing_zeros() as u64 + 1;
    size as u64 * 8 - padding
}

/// map_pixel_format.
fn pixel_format(bit_depth: u32, chroma_format_idc: u32) -> Option<PixFmt> {
    Some(match (bit_depth, chroma_format_idc) {
        (8, 0) => PixFmt::Gray8,
        (8, 1) => PixFmt::Yuv420p,
        (8, 2) => PixFmt::Yuv422p,
        (8, 3) => PixFmt::Yuv444p,
        (9, 1) => PixFmt::Yuv420p9,
        (9, 2) => PixFmt::Yuv422p9,
        (9, 3) => PixFmt::Yuv444p9,
        (10, 0) => PixFmt::Gray10,
        (10, 1) => PixFmt::Yuv420p10,
        (10, 2) => PixFmt::Yuv422p10,
        (10, 3) => PixFmt::Yuv444p10,
        (12, 0) => PixFmt::Gray12,
        (12, 1) => PixFmt::Yuv420p12,
        (12, 2) => PixFmt::Yuv422p12,
        (12, 3) => PixFmt::Yuv444p12,
        _ => return None,
    })
}

/// hevc_sub_width_c / hevc_sub_height_c.
const SUB_WIDTH: [i64; 4] = [1, 2, 2, 1];
const SUB_HEIGHT: [i64; 4] = [1, 2, 1, 1];

/// read_window: left, right, top, bottom offsets in luma samples.
fn read_window(gb: &mut BitReader<'_>, chroma_format_idc: u32, w: i32, h: i32) -> Option<[i32; 4]> {
    let horiz = SUB_WIDTH[chroma_format_idc as usize];
    let vert = SUB_HEIGHT[chroma_format_idc as usize];
    let left = i64::from(gb.ue_long()) * horiz;
    let right = i64::from(gb.ue_long()) * horiz;
    let top = i64::from(gb.ue_long()) * vert;
    let bottom = i64::from(gb.ue_long()) * vert;
    if i64::from(w) <= left + right || i64::from(h) <= top + bottom {
        return None;
    }
    Some([left as i32, right as i32, top as i32, bottom as i32])
}

/// parse_ptl over a profile_tier_level(1, max_sub_layers - 1): 88 bits
/// per stated profile, 8 per stated level.
fn skip_ptl(gb: &mut BitReader<'_>, max_sub_layers: u32) -> bool {
    let subs = max_sub_layers - 1;
    if gb.left() < 88 || gb.left() - 88 < 8 + if subs > 0 { 16 } else { 0 } {
        return false;
    }
    gb.skip(88 + 8);
    let mut flags = Vec::with_capacity(subs as usize);
    for _ in 0..subs {
        flags.push((gb.read1(), gb.read1()));
    }
    if subs > 0 {
        gb.skip(u64::from(8 - subs) * 2);
    }
    for (profile, level) in flags {
        if profile {
            if gb.left() < 88 {
                return false;
            }
            gb.skip(88);
        }
        if level {
            if gb.left() < 8 {
                return false;
            }
            gb.skip(8);
        }
    }
    true
}

/// scaling_list_data, for its length and its checks.
fn skip_scaling_list_data(gb: &mut BitReader<'_>) -> bool {
    for size_id in 0..4u32 {
        let step = if size_id == 3 { 3 } else { 1 };
        let mut matrix_id = 0u32;
        while matrix_id < 6 {
            if !gb.read1() {
                let delta = gb.ue_long().saturating_mul(step);
                if delta != 0 && matrix_id < delta {
                    return false;
                }
            } else {
                let coef_num = 64.min(1u32 << (4 + (size_id << 1)));
                if size_id > 1 {
                    let dc = gb.se();
                    if !(-7..=247).contains(&dc) {
                        return false;
                    }
                }
                for _ in 0..coef_num {
                    gb.se();
                }
            }
            matrix_id += step;
        }
    }
    true
}

/// ff_hevc_decode_short_term_rps in an SPS (set `index`), for its length
/// and checks: the set's num_delta_pocs, given the earlier sets'.
fn skip_short_term_rps(gb: &mut BitReader<'_>, index: usize, earlier: &[u32]) -> Option<u32> {
    let predict = index != 0 && gb.read1();
    if predict {
        let ref_num = earlier[index - 1];
        gb.skip(1); // delta_rps_sign
        let abs_delta = u64::from(gb.ue_long()) + 1;
        if abs_delta > 32768 {
            return None;
        }
        let mut k = 0u32;
        for _ in 0..=ref_num {
            let used = gb.read1();
            let use_delta = if used { false } else { gb.read1() };
            if used || use_delta {
                k += 1;
            }
        }
        if k >= 32 {
            return None;
        }
        Some(k)
    } else {
        let negative = gb.ue_long();
        let positive = gb.ue_long();
        if negative >= MAX_REFS || positive >= MAX_REFS {
            return None;
        }
        for _ in 0..negative + positive {
            let delta = u64::from(gb.ue_long()) + 1;
            if delta > 32768 {
                return None;
            }
            gb.skip(1);
        }
        Some(negative + positive)
    }
}

#[derive(Default)]
struct Vui {
    def_disp_win: [i32; 4],
    timing: Option<(u32, u32)>,
    frame_field_info_present: bool,
}

/// decode_vui, with FFmpeg's retries for the alternate syntax.
fn decode_vui(
    gb: &mut BitReader<'_>,
    vui: &mut Vui,
    chroma_format_idc: u32,
    width: i32,
    height: i32,
    max_sub_layers: u32,
) {
    // ff_h2645_decode_common_vui_params.
    if gb.read1() && gb.read(8) == 255 {
        gb.skip(32); // EXTENDED_SAR
    }
    if gb.read1() {
        gb.skip(1);
    }
    if gb.read1() {
        gb.skip(3 + 1); // video_format, video_full_range_flag
        if gb.read1() {
            gb.skip(24);
        }
    }
    if gb.read1() {
        gb.ue31();
        gb.ue31();
    }
    gb.skip(1); // neutral_chroma_indication
    gb.skip(1); // field_seq
    vui.frame_field_info_present = gb.read1();
    let backup = gb.clone();
    let mut alt = false;
    let has_window = if gb.left() >= 68 && gb.peek(21) == 0x10_0000 {
        false // "Invalid default display window"
    } else {
        gb.read1()
    };
    if has_window {
        vui.def_disp_win = read_window(gb, chroma_format_idc, width, height).unwrap_or([0; 4]);
    }
    loop {
        // timing_info:
        vui.timing = None;
        if gb.read1() {
            if gb.left() < 66 && !alt {
                *gb = backup.clone();
                vui.def_disp_win = [0; 4];
                alt = true;
                continue;
            }
            let num = gb.read(32);
            let den = gb.read(32);
            vui.timing = Some((num, den));
            if gb.read1() {
                gb.ue_long();
            }
            if gb.read1() {
                skip_hrd(gb, max_sub_layers);
            }
        }
        if gb.read1() {
            // bitstream_restriction
            if gb.left() < 8 && !alt {
                *gb = backup.clone();
                vui.def_disp_win = [0; 4];
                alt = true;
                continue;
            }
            gb.skip(3);
            for _ in 0..5 {
                gb.ue_long();
            }
        }
        if gb.left() < 1 && !alt {
            *gb = backup.clone();
            vui.def_disp_win = [0; 4];
            alt = true;
            continue;
        }
        return;
    }
}

/// decode_hrd with common_inf_present, for its length. Its error (a
/// cpb_cnt over 31) ends it early; the VUI goes on, as in FFmpeg.
fn skip_hrd(gb: &mut BitReader<'_>, max_sub_layers: u32) {
    let nal = gb.read1();
    let vcl = gb.read1();
    let mut subpic = false;
    if nal || vcl {
        subpic = gb.read1();
        if subpic {
            gb.skip(8 + 5 + 1 + 5);
        }
        gb.skip(4 + 4);
        if subpic {
            gb.skip(4);
        }
        gb.skip(5 + 5 + 5);
    }
    for _ in 0..max_sub_layers {
        let fixed_general = gb.read1();
        let fixed_within_cvs = !fixed_general && gb.read1();
        let mut low_delay = false;
        if fixed_general || fixed_within_cvs {
            gb.ue_long(); // elemental_duration_in_tc_minus1
        } else {
            low_delay = gb.read1();
        }
        let mut cpb_cnt = 1;
        if !low_delay {
            let minus1 = gb.ue_long();
            if minus1 > 31 {
                return;
            }
            cpb_cnt = minus1 + 1;
        }
        // decode_sublayer_hrd for NAL and VCL.
        for present in [nal, vcl] {
            if present {
                for _ in 0..cpb_cnt {
                    gb.ue_long();
                    gb.ue_long();
                    if subpic {
                        gb.ue_long();
                        gb.ue_long();
                    }
                    gb.skip(1);
                }
            }
        }
    }
}
