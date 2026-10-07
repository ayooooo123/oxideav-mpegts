// SPDX-License-Identifier: LGPL-2.1-or-later
// Port of FFmpeg 2da55bf libavcodec/get_bits.h (the non-cached big-endian
// reader) and libavcodec/golomb.h / golomb.c (get_ue_golomb,
// get_ue_golomb_31, get_ue_golomb_long, get_se_golomb, get_se_golomb_long).
// Copyright (c) 2004 Michael Niedermayer <michaelni@gmx.at> (get_bits.h)
// Copyright (c) 2003 Michael Niedermayer <michaelni@gmx.at>
// Copyright (c) 2004 Alex Beregszaszi (golomb.h, golomb.c)
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

//! Bit reader over an untrusted byte slice. Bits past the slice read as
//! zero (FFmpeg's zeroed input padding); the position keeps counting, so
//! [`BitReader::left`] goes negative on an overread exactly as
//! `get_bits_left` does. Nothing here can panic or allocate.

/// AVERROR_INVALIDDATA as the golomb readers return it.
pub(crate) const AVERROR_INVALIDDATA: i32 = -1_094_995_529;

#[derive(Clone, Debug)]
pub(crate) struct BitReader<'a> {
    data: &'a [u8],
    size_bits: u64,
    index: u64,
}

impl<'a> BitReader<'a> {
    /// `init_get_bits8`: the whole slice.
    pub fn new(data: &'a [u8]) -> Self {
        Self::with_bits(data, data.len() as u64 * 8)
    }

    /// `init_get_bits` with an explicit bit size; bytes of `data` past it
    /// still read as data, as FFmpeg reads the buffer behind its size.
    pub fn with_bits(data: &'a [u8], size_bits: u64) -> Self {
        Self {
            data,
            size_bits,
            index: 0,
        }
    }

    fn byte(&self, at: u64) -> u8 {
        usize::try_from(at)
            .ok()
            .and_then(|i| self.data.get(i))
            .copied()
            .unwrap_or(0)
    }

    /// AV_RB32 at the current byte, shifted by the bit offset: the
    /// non-cached reader's UPDATE_CACHE (low bits zero-filled).
    fn cache(&self) -> u32 {
        let b = self.index >> 3;
        let word = u32::from_be_bytes([
            self.byte(b),
            self.byte(b + 1),
            self.byte(b + 2),
            self.byte(b + 3),
        ]);
        word << (self.index & 7)
    }

    /// The next `n` bits (`n` ≤ 32), exactly.
    pub fn peek(&self, n: u32) -> u32 {
        if n == 0 {
            return 0;
        }
        let b = self.index >> 3;
        let mut word = 0u64;
        for k in 0..5 {
            word = word << 8 | u64::from(self.byte(b + k));
        }
        let word = word << (self.index & 7);
        ((word >> (40 - n)) & ((1u64 << n) - 1)) as u32
    }

    /// `get_bits` / `get_bits_long`, `n` ≤ 32.
    pub fn read(&mut self, n: u32) -> u32 {
        let v = self.peek(n);
        self.skip(u64::from(n));
        v
    }

    pub fn read1(&mut self) -> bool {
        self.read(1) != 0
    }

    pub fn skip(&mut self, n: u64) {
        self.index = self.index.saturating_add(n);
    }

    /// `get_bits_count`.
    #[cfg(test)]
    pub fn count(&self) -> u64 {
        self.index
    }

    /// `get_bits_left`.
    pub fn left(&self) -> i64 {
        (self.size_bits as i128 - self.index as i128).clamp(i64::MIN as i128, i64::MAX as i128)
            as i64
    }

    /// `get_ue_golomb`: values above 8190 (13+ leading zeros) are
    /// AVERROR_INVALIDDATA.
    pub fn ue(&mut self) -> i32 {
        let buf = self.cache();
        if buf >= 1 << 27 {
            let idx = (buf >> 23) as usize;
            self.skip(u64::from(GOLOMB_VLC_LEN[idx]));
            i32::from(UE_GOLOMB_VLC_CODE[idx])
        } else {
            let log = 2 * log2(buf) - 31;
            self.skip((32 - log) as u64);
            if log < 7 {
                return AVERROR_INVALIDDATA;
            }
            ((buf >> log) - 1) as i32
        }
    }

    /// `get_ue_golomb_31`: exact for values up to 30.
    pub fn ue31(&mut self) -> i32 {
        let idx = (self.cache() >> 23) as usize;
        self.skip(u64::from(GOLOMB_VLC_LEN[idx]));
        i32::from(UE_GOLOMB_VLC_CODE[idx])
    }

    /// `get_ue_golomb_long`.
    pub fn ue_long(&mut self) -> u32 {
        let buf = self.peek(32);
        let log = 31 - log2(buf) as u32;
        self.skip(u64::from(log));
        self.read(log + 1).wrapping_sub(1)
    }

    /// `get_se_golomb` (non-cached reader).
    pub fn se(&mut self) -> i32 {
        let buf = self.cache();
        if buf >= 1 << 27 {
            let idx = (buf >> 23) as usize;
            self.skip(u64::from(GOLOMB_VLC_LEN[idx]));
            i32::from(SE_GOLOMB_VLC_CODE[idx])
        } else {
            let log = log2(buf);
            self.skip((31 - log) as u64);
            let buf = self.cache() >> log;
            self.skip((32 - log) as u64);
            let sign = (buf & 1).wrapping_neg();
            (((buf >> 1) ^ sign).wrapping_sub(sign)) as i32
        }
    }

    /// `get_se_golomb_long`.
    pub fn se_long(&mut self) -> i32 {
        let buf = self.ue_long();
        let sign = (buf & 1).wrapping_sub(1);
        (((buf >> 1) ^ sign).wrapping_add(1)) as i32
    }
}

/// `av_log2` (0 for 0).
pub(crate) fn log2(v: u32) -> i32 {
    if v == 0 {
        0
    } else {
        31 - v.leading_zeros() as i32
    }
}

const fn leading_zeros9(i: usize) -> usize {
    let mut z = 0;
    while z < 9 && (i >> (8 - z)) & 1 == 0 {
        z += 1;
    }
    z
}

/// ff_golomb_vlc_len: the code length of the 9-bit window `i`.
const GOLOMB_VLC_LEN: [u8; 512] = {
    let mut t = [0u8; 512];
    let mut i = 0;
    while i < 512 {
        t[i] = (2 * leading_zeros9(i) + 1) as u8;
        i += 1;
    }
    t
};

/// ff_ue_golomb_vlc_code.
const UE_GOLOMB_VLC_CODE: [u8; 512] = {
    let mut t = [0u8; 512];
    let mut i = 0;
    while i < 512 {
        let z = leading_zeros9(i);
        t[i] = if z <= 4 {
            ((i >> (8 - 2 * z)) - 1) as u8
        } else if z == 5 && i == 8 {
            31
        } else {
            32
        };
        i += 1;
    }
    t
};

/// ff_se_golomb_vlc_code.
const SE_GOLOMB_VLC_CODE: [i8; 512] = {
    let mut t = [0i8; 512];
    let mut i = 0;
    while i < 512 {
        let z = leading_zeros9(i);
        t[i] = if z <= 4 {
            let ue = (i >> (8 - 2 * z)) as i32 - 1;
            if ue & 1 == 1 {
                ((ue + 1) / 2) as i8
            } else {
                (-(ue / 2)) as i8
            }
        } else if z == 5 && i == 8 {
            16
        } else {
            17
        };
        i += 1;
    }
    t
};

#[cfg(test)]
mod tests {
    use super::*;

    /// The generated tables equal golomb.c's first rows.
    #[test]
    fn tables_match_ffmpeg() {
        assert_eq!(
            &GOLOMB_VLC_LEN[..20],
            &[19, 17, 15, 15, 13, 13, 13, 13, 11, 11, 11, 11, 11, 11, 11, 11, 9, 9, 9, 9]
        );
        assert_eq!(
            &UE_GOLOMB_VLC_CODE[..20],
            &[32, 32, 32, 32, 32, 32, 32, 32, 31, 32, 32, 32, 32, 32, 32, 32, 15, 16, 17, 18]
        );
        assert_eq!(&UE_GOLOMB_VLC_CODE[32..40], &[7, 7, 7, 7, 8, 8, 8, 8]);
        assert_eq!(
            &SE_GOLOMB_VLC_CODE[..20],
            &[17, 17, 17, 17, 17, 17, 17, 17, 16, 17, 17, 17, 17, 17, 17, 17, 8, -8, 9, -9]
        );
        assert_eq!(&SE_GOLOMB_VLC_CODE[32..40], &[4, 4, 4, 4, -4, -4, -4, -4]);
        assert_eq!(UE_GOLOMB_VLC_CODE[256], 0);
        assert_eq!(SE_GOLOMB_VLC_CODE[128], 1);
    }

    #[test]
    fn exp_golomb_codes_and_overread() {
        // 1 | 010 | 011 | 00100 | 0001000 | 00000000 ...
        let data = [0b1010_0110, 0b0100_0001, 0b0000_0000];
        let mut r = BitReader::new(&data);
        assert_eq!(r.ue(), 0);
        assert_eq!(r.ue(), 1);
        assert_eq!(r.se(), -1);
        assert_eq!(r.ue31(), 3);
        assert_eq!(r.ue_long(), 7);
        assert_eq!(r.count(), 19);
        // Past the end the bits are zero and the count goes negative.
        assert_eq!(r.ue(), AVERROR_INVALIDDATA);
        assert!(r.left() < 0);
        assert_eq!(r.read(32), 0);
    }
}
