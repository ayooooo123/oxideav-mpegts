// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavformat/demux.c: compute_pkt_fields,
// compute_frame_duration, update_initial_timestamps,
// update_initial_durations, update_dts_from_pts, select_from_pts_buffer,
// and the codec cases of libavcodec/utils.c av_get_audio_frame_duration
// the parsed codecs reach.
// Copyright (c) 2000-2003 Fabrice Bellard
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

//! Packet timing as av_read_frame returns it. A [`Pending`] is a packet
//! FFmpeg would hold in `packet_buffer` or `parse_queue`: it can still be
//! amended when a stream's first timestamp or duration appears.

use std::collections::VecDeque;

use super::parser::{
    add_stable, is_relative, mul_q, pict, reduce, rescale_rnd, Codec, CodecCtx, ParserState, Rnd,
    INT_MAX, NOPTS, Q, RELATIVE_TS_BASE,
};

/// MAX_REORDER_DELAY (avformat_internal.h).
const MAX_REORDER_DELAY: usize = 16;
/// avpriv_set_pts_info(st, 33, 1, 90000).
const PTS_WRAP_BITS: u32 = 33;
pub(crate) const TIME_BASE: Q = Q {
    num: 1,
    den: 90_000,
};

/// One packet still in FFmpeg's buffers.
#[derive(Clone, Debug)]
pub(crate) struct Pending {
    pub stream: u32,
    pub pts: i64,
    pub dts: i64,
    pub duration: i64,
    pub key: bool,
    pub container_key: bool,
    pub data: Vec<u8>,
}

/// The FFStream timing state of one parsed stream.
#[derive(Clone, Debug)]
pub(crate) struct StreamTiming {
    pub first_dts: i64,
    pub cur_dts: i64,
    last_ip_pts: i64,
    last_ip_duration: i64,
    last_dts_for_order_check: i64,
    dts_ordered: u32,
    dts_misordered: u32,
    pts_buffer: [i64; MAX_REORDER_DELAY + 1],
    pts_reorder_error: [i64; MAX_REORDER_DELAY + 1],
    pts_reorder_error_count: [u8; MAX_REORDER_DELAY + 1],
    update_initial_durations_done: bool,
}

impl StreamTiming {
    pub fn new() -> Self {
        Self {
            first_dts: NOPTS,
            cur_dts: RELATIVE_TS_BASE,
            last_ip_pts: NOPTS,
            last_ip_duration: 0,
            last_dts_for_order_check: NOPTS,
            dts_ordered: 0,
            dts_misordered: 0,
            pts_buffer: [NOPTS; MAX_REORDER_DELAY + 1],
            pts_reorder_error: [0; MAX_REORDER_DELAY + 1],
            pts_reorder_error_count: [0; MAX_REORDER_DELAY + 1],
            update_initial_durations_done: false,
        }
    }

    /// ff_read_frame_flush, then ff_update_cur_dts to `timestamp`.
    pub fn seeked(&mut self, timestamp: i64) {
        self.last_ip_pts = NOPTS;
        self.last_dts_for_order_check = NOPTS;
        self.pts_buffer = [NOPTS; MAX_REORDER_DELAY + 1];
        self.cur_dts = timestamp;
    }

    /// The end of estimate_timings_from_pts: timing resumes at the first
    /// timestamp the read-ahead saw.
    pub fn restart(&mut self) {
        self.cur_dts = self.first_dts;
        self.last_ip_pts = NOPTS;
        self.last_dts_for_order_check = NOPTS;
        self.pts_buffer = [NOPTS; MAX_REORDER_DELAY + 1];
    }

    /// compute_pkt_fields for a packet of `stream` out of `pc` (the
    /// parser), before it joins `queue`. `guessed` is
    /// has_decode_delay_been_guessed.
    #[allow(clippy::too_many_arguments)]
    pub fn compute_pkt_fields(
        &mut self,
        avctx: &mut CodecCtx,
        pc: &ParserState,
        pkt: &mut Pending,
        next_dts: i64,
        next_pts: i64,
        queue: &mut VecDeque<Pending>,
        guessed: bool,
    ) {
        let onein_oneout = avctx.codec != Codec::H264;
        let video = avctx.codec.is_video();

        if video && pkt.dts != NOPTS {
            if pkt.dts == pkt.pts && self.last_dts_for_order_check != NOPTS {
                if self.last_dts_for_order_check <= pkt.dts {
                    self.dts_ordered += 1;
                } else {
                    self.dts_misordered += 1;
                }
                if self.dts_ordered + self.dts_misordered > 250 {
                    self.dts_ordered >>= 1;
                    self.dts_misordered >>= 1;
                }
            }
            self.last_dts_for_order_check = pkt.dts;
            if self.dts_ordered < 8 * self.dts_misordered && pkt.dts == pkt.pts {
                pkt.dts = NOPTS;
            }
        }

        if pc.pict_type == pict::B && avctx.has_b_frames == 0 {
            avctx.has_b_frames = 1;
        }
        let delay = avctx.has_b_frames.max(0) as usize;
        let mut presentation_delayed = delay != 0 && pc.pict_type != pict::B;

        let wrap = 1i64 << PTS_WRAP_BITS;
        if pkt.pts != NOPTS
            && pkt.dts != NOPTS
            && pkt.dts > i64::MIN + wrap
            && pkt.dts - (1i64 << (PTS_WRAP_BITS - 1)) > pkt.pts
        {
            if is_relative(self.cur_dts) || pkt.dts - (1i64 << (PTS_WRAP_BITS - 1)) > self.cur_dts {
                pkt.dts -= wrap;
            } else {
                pkt.pts += wrap;
            }
        }

        if delay == 1 && pkt.dts == pkt.pts && pkt.dts != NOPTS && presentation_delayed {
            pkt.dts = NOPTS;
        }

        let mut duration = mul_q(
            Q {
                num: pkt.duration,
                den: 1,
            },
            TIME_BASE,
        );
        if pkt.duration <= 0 {
            if let Some(d) = frame_duration(avctx, pc, pkt.data.len()) {
                duration = d;
                pkt.duration =
                    rescale_rnd(1, d.num * TIME_BASE.den, d.den * TIME_BASE.num, Rnd::Down);
            }
        }

        if pkt.duration > 0 && !queue.is_empty() {
            self.update_initial_durations(avctx, pkt.stream, pkt.duration, queue);
        }

        if pkt.dts != NOPTS && pkt.pts != NOPTS && pkt.pts > pkt.dts {
            presentation_delayed = true;
        }

        if (delay == 0 || delay == 1) && onein_oneout {
            if presentation_delayed {
                if pkt.dts == NOPTS {
                    pkt.dts = self.last_ip_pts;
                }
                self.update_initial_timestamps(avctx, pkt.stream, pkt.dts, queue, guessed);
                if pkt.dts == NOPTS {
                    pkt.dts = self.cur_dts;
                }
                if self.last_ip_duration == 0 && (pkt.duration as u64) <= i32::MAX as u64 {
                    self.last_ip_duration = pkt.duration;
                }
                if pkt.dts != NOPTS {
                    self.cur_dts = pkt.dts.saturating_add(self.last_ip_duration);
                }
                if pkt.dts != NOPTS
                    && pkt.pts == NOPTS
                    && self.last_ip_duration > 0
                    && (self.cur_dts as u64)
                        .wrapping_sub(next_dts as u64)
                        .wrapping_add(1)
                        <= 2
                    && next_dts != next_pts
                    && next_pts != NOPTS
                {
                    pkt.pts = next_dts;
                }
                if (pkt.duration as u64) <= i32::MAX as u64 {
                    self.last_ip_duration = pkt.duration;
                }
                self.last_ip_pts = pkt.pts;
            } else if pkt.pts != NOPTS || pkt.dts != NOPTS || pkt.duration > 0 {
                if pkt.pts == NOPTS {
                    pkt.pts = pkt.dts;
                }
                self.update_initial_timestamps(avctx, pkt.stream, pkt.pts, queue, guessed);
                if pkt.pts == NOPTS {
                    pkt.pts = self.cur_dts;
                }
                pkt.dts = pkt.pts;
                if pkt.pts != NOPTS && duration.num >= 0 {
                    self.cur_dts = add_stable(TIME_BASE, pkt.pts, duration);
                }
            }
        }

        if pkt.pts != NOPTS && delay <= MAX_REORDER_DELAY {
            self.pts_buffer[0] = pkt.pts;
            let mut i = 0;
            while i < delay && self.pts_buffer[i] > self.pts_buffer[i + 1] {
                self.pts_buffer.swap(i, i + 1);
                i += 1;
            }
            if guessed {
                let buffer = self.pts_buffer;
                pkt.dts = self.select_from_pts_buffer(avctx, &buffer, pkt.dts);
            }
        }
        if !onein_oneout {
            self.update_initial_timestamps(avctx, pkt.stream, pkt.dts, queue, guessed);
        }
        if pkt.dts > self.cur_dts {
            self.cur_dts = pkt.dts;
        }
        if avctx.codec.intra_only() {
            pkt.key = true;
        }
    }

    /// select_from_pts_buffer.
    // select_from_pts_buffer walks three parallel arrays by index.
    #[allow(clippy::needless_range_loop)]
    fn select_from_pts_buffer(
        &mut self,
        avctx: &CodecCtx,
        pts_buffer: &[i64; MAX_REORDER_DELAY + 1],
        mut dts: i64,
    ) -> i64 {
        if avctx.codec == Codec::H264 {
            let delay = (avctx.has_b_frames.max(0) as usize).min(MAX_REORDER_DELAY);
            if dts == NOPTS {
                let mut best_score = i64::MAX;
                for i in 0..delay {
                    if self.pts_reorder_error_count[i] != 0 {
                        let score =
                            self.pts_reorder_error[i] / i64::from(self.pts_reorder_error_count[i]);
                        if score < best_score {
                            best_score = score;
                            dts = pts_buffer[i];
                        }
                    }
                }
            } else {
                for i in 0..delay {
                    if pts_buffer[i] != NOPTS {
                        let mut diff = pts_buffer[i].abs_diff(dts);
                        if diff > (i64::MAX - self.pts_reorder_error[i]) as u64 {
                            diff = i64::MAX as u64;
                        } else {
                            diff += self.pts_reorder_error[i] as u64;
                        }
                        self.pts_reorder_error[i] = diff as i64;
                        self.pts_reorder_error_count[i] =
                            self.pts_reorder_error_count[i].wrapping_add(1);
                        if self.pts_reorder_error_count[i] > 250 {
                            self.pts_reorder_error[i] >>= 1;
                            self.pts_reorder_error_count[i] >>= 1;
                        }
                    }
                }
            }
        }
        if dts == NOPTS {
            dts = pts_buffer[0];
        }
        dts
    }

    /// update_dts_from_pts over the queued packets of `stream`.
    fn update_dts_from_pts(
        &mut self,
        avctx: &CodecCtx,
        stream: u32,
        queue: &mut VecDeque<Pending>,
    ) {
        let delay = avctx.has_b_frames.max(0) as usize;
        let mut pts_buffer = [NOPTS; MAX_REORDER_DELAY + 1];
        for e in queue.iter_mut().filter(|e| e.stream == stream) {
            if e.pts != NOPTS && delay <= MAX_REORDER_DELAY {
                pts_buffer[0] = e.pts;
                let mut i = 0;
                while i < delay && pts_buffer[i] > pts_buffer[i + 1] {
                    pts_buffer.swap(i, i + 1);
                    i += 1;
                }
                e.dts = self.select_from_pts_buffer(avctx, &pts_buffer, e.dts);
            }
        }
    }

    /// update_initial_timestamps.
    fn update_initial_timestamps(
        &mut self,
        avctx: &CodecCtx,
        stream: u32,
        dts: i64,
        queue: &mut VecDeque<Pending>,
        guessed: bool,
    ) {
        let int_min = i64::from(i32::MIN);
        if self.first_dts != NOPTS
            || dts == NOPTS
            || self.cur_dts == NOPTS
            || self.cur_dts < int_min + RELATIVE_TS_BASE
            || (dts as i128) < int_min as i128 + (self.cur_dts as i128 - RELATIVE_TS_BASE as i128)
            || is_relative(dts)
        {
            return;
        }
        self.first_dts = dts.wrapping_sub(self.cur_dts - RELATIVE_TS_BASE);
        self.cur_dts = dts;
        let shift = (self.first_dts as u64).wrapping_sub(RELATIVE_TS_BASE as u64);
        for e in queue.iter_mut().filter(|e| e.stream == stream) {
            if is_relative(e.pts) {
                e.pts = (e.pts as u64).wrapping_add(shift) as i64;
            }
            if is_relative(e.dts) {
                e.dts = (e.dts as u64).wrapping_add(shift) as i64;
            }
        }
        if guessed {
            self.update_dts_from_pts(avctx, stream, queue);
        }
    }

    /// update_initial_durations.
    fn update_initial_durations(
        &mut self,
        avctx: &CodecCtx,
        stream: u32,
        duration: i64,
        queue: &mut VecDeque<Pending>,
    ) {
        let mut cur_dts = RELATIVE_TS_BASE;
        if self.first_dts != NOPTS {
            if self.update_initial_durations_done {
                return;
            }
            self.update_initial_durations_done = true;
            cur_dts = self.first_dts;
            let mut found = None;
            for e in queue.iter().filter(|e| e.stream == stream) {
                if e.pts != e.dts || e.dts != NOPTS || e.duration != 0 {
                    found = Some(e.dts);
                    break;
                }
                cur_dts = cur_dts.wrapping_sub(duration);
            }
            match found {
                Some(dts) if dts == self.first_dts => {}
                _ => return,
            }
            self.first_dts = cur_dts;
        } else if self.cur_dts != RELATIVE_TS_BASE {
            return;
        }
        let mut finished = true;
        for e in queue.iter_mut().filter(|e| e.stream == stream) {
            if (e.pts == e.dts || e.pts == NOPTS)
                && (e.dts == NOPTS || e.dts == self.first_dts || e.dts == RELATIVE_TS_BASE)
                && e.duration == 0
                && cur_dts.checked_add(duration).is_some()
            {
                e.dts = cur_dts;
                if avctx.has_b_frames == 0 {
                    e.pts = cur_dts;
                }
                e.duration = duration;
            } else {
                finished = false;
                break;
            }
            cur_dts = e.dts.wrapping_add(e.duration);
        }
        if finished {
            self.cur_dts = cur_dts;
        }
    }
}

/// compute_frame_duration for a parsed stream (no r_frame_rate: FFmpeg
/// only has one after find_stream_info for streams whose codec states no
/// frame rate). `None` when there is no duration.
fn frame_duration(avctx: &CodecCtx, pc: &ParserState, size: usize) -> Option<Q> {
    if avctx.codec.is_video() {
        let fr = avctx.framerate;
        if fr.den.saturating_mul(1000) > fr.num {
            let ticks_per_frame = if avctx.codec.fields() { 2 } else { 1 };
            let mut q = reduce(fr.den, fr.num.saturating_mul(ticks_per_frame), INT_MAX);
            if pc.repeat_pict != 0 {
                q = reduce(
                    q.num.saturating_mul(1 + i64::from(pc.repeat_pict)),
                    q.den,
                    INT_MAX,
                );
            }
            if q.num != 0 && q.den != 0 {
                return Some(q);
            }
        }
        return None;
    }
    let frame_size = audio_frame_duration(avctx, size);
    (frame_size > 0 && avctx.sample_rate > 0).then_some(Q {
        num: i64::from(frame_size),
        den: i64::from(avctx.sample_rate),
    })
}

/// av_get_audio_frame_duration for the parsed audio codecs.
fn audio_frame_duration(avctx: &CodecCtx, frame_bytes: usize) -> i32 {
    match avctx.codec {
        Codec::Mp1 => 384,
        Codec::Mp2 => 1152,
        Codec::Ac3 => 1536,
        Codec::Mp3 if avctx.sample_rate > 0 => {
            if avctx.sample_rate <= 24000 {
                576
            } else {
                1152
            }
        }
        _ if avctx.frame_size > 1 && frame_bytes != 0 => avctx.frame_size,
        _ => 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pending(pts: i64, duration: i64) -> Pending {
        Pending {
            stream: 0,
            pts,
            dts: pts,
            duration,
            key: false,
            container_key: false,
            data: vec![0; 10],
        }
    }

    /// The frames after the first in a PES carry no timestamps: they
    /// follow from the first and each frame's duration, and a duration
    /// learned later is filled in for the frames still queued.
    #[test]
    fn audio_frames_are_timed_from_the_first_and_their_durations() {
        let mut avctx = CodecCtx::new(Codec::Aac);
        let pc = ParserState::new_for_tests();
        let mut t = StreamTiming::new();
        let mut queue = VecDeque::new();
        // Two frames before the decoder knows the frame size.
        let mut p0 = pending(126_000, 0);
        t.compute_pkt_fields(&mut avctx, &pc, &mut p0, 126_000, 126_000, &mut queue, true);
        queue.push_back(p0);
        let mut p1 = pending(NOPTS, 0);
        p1.dts = NOPTS;
        t.compute_pkt_fields(&mut avctx, &pc, &mut p1, NOPTS, NOPTS, &mut queue, true);
        queue.push_back(p1);
        assert_eq!((queue[1].pts, queue[1].duration), (NOPTS, 0));
        // The next PES: 1024 samples at 48 kHz.
        avctx.frame_size = 1024;
        avctx.sample_rate = 48_000;
        let mut p2 = pending(129_840, 0);
        t.compute_pkt_fields(&mut avctx, &pc, &mut p2, 129_840, 129_840, &mut queue, true);
        let timeline: Vec<_> = queue.iter().map(|p| (p.pts, p.dts, p.duration)).collect();
        assert_eq!(
            timeline,
            [(126_000, 126_000, 1920), (127_920, 127_920, 1920)]
        );
        assert_eq!((p2.pts, p2.dts, p2.duration), (129_840, 129_840, 1920));
        // A frame without a timestamp continues the stable timeline.
        let mut p3 = pending(NOPTS, 0);
        p3.dts = NOPTS;
        t.compute_pkt_fields(&mut avctx, &pc, &mut p3, NOPTS, NOPTS, &mut queue, true);
        assert_eq!((p3.pts, p3.dts), (131_760, 131_760));
    }
}
