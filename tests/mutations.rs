//! Seeded mutations of an FFmpeg-muxed stream (`data/ffmpeg_mux.ts`:
//! H.264, AC-3, MP2, ADTS AAC, E-AC-3) through the registry demuxer and
//! its FFmpeg parser stage: every mutant opens or fails cleanly, never
//! panics, and ends — at most its own bytes come back out, in at most
//! as many packets, before and after a seek.

use std::io::{Cursor, Read, Seek, SeekFrom};

use oxideav_core::{Demuxer, NullCodecResolver};
use oxideav_mpegts::{open_demuxer, TS_PACKET_LEN};

const TS: &[u8] = include_bytes!("data/ffmpeg_mux.ts");
const MUTANTS: usize = 2500;

/// xorshift64* with a fixed seed: the same mutants on every run.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

/// A cursor that stops seeking once `limit` bytes were read, so the
/// demuxer opens but cannot go back for its second pass.
struct LateUnseekable {
    inner: Cursor<Vec<u8>>,
    read: usize,
    limit: usize,
}

impl Read for LateUnseekable {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.read += n;
        Ok(n)
    }
}

impl Seek for LateUnseekable {
    fn seek(&mut self, pos: SeekFrom) -> std::io::Result<u64> {
        if self.read > self.limit {
            return Err(std::io::Error::other("not seekable"));
        }
        self.inner.seek(pos)
    }
}

/// Packets and payload bytes until the stream ends or errors, failing
/// once either exceeds what the input could hold.
fn drain(demuxer: &mut dyn Demuxer, len: usize, mutant: usize) {
    let (mut packets, mut bytes) = (0usize, 0usize);
    while let Ok(p) = demuxer.next_packet() {
        packets += 1;
        bytes += p.data.len();
        assert!(
            packets <= len && bytes <= len,
            "mutant {mutant}: {packets} packets / {bytes} bytes from {len}"
        );
    }
}

#[test]
fn mutated_streams_end_cleanly() {
    // Packets that start a PES carry its header and the codec headers
    // the parsers read (SPS/PPS/SEI, sync frames): half the edits go
    // there.
    let heads: Vec<usize> = (0..TS.len() / TS_PACKET_LEN)
        .map(|i| i * TS_PACKET_LEN)
        .filter(|&at| TS[at + 1] & 0x40 != 0)
        .collect();
    let mut rng = Rng(0x7EA5_5EED_2DA5_5BF0);
    let mut opened = 0;
    for mutant in 0..MUTANTS {
        let mut bytes = TS.to_vec();
        for _ in 0..1 + rng.below(8) {
            let at = if rng.below(2) == 0 {
                heads[rng.below(heads.len())] + rng.below(64)
            } else {
                rng.below(bytes.len())
            };
            if at < bytes.len() {
                bytes[at] = rng.next() as u8;
            }
        }
        match rng.below(8) {
            // Cut anywhere, mid-packet included.
            0 => bytes.truncate(rng.below(bytes.len())),
            // Repeat a run of packets elsewhere (continuity breaks,
            // repeated or interleaved PES).
            1 => {
                let from = rng.below(bytes.len() / TS_PACKET_LEN) * TS_PACKET_LEN;
                let run = (1 + rng.below(6)) * TS_PACKET_LEN;
                let chunk = bytes[from..(from + run).min(bytes.len())].to_vec();
                let to = rng.below(bytes.len() / TS_PACKET_LEN) * TS_PACKET_LEN;
                bytes.splice(to..to, chunk);
            }
            // Wipe a span (PES headers, start codes, sync words).
            2 => {
                let from = rng.below(bytes.len());
                let end = (from + 1 + rng.below(400)).min(bytes.len());
                bytes[from..end].fill(0);
            }
            _ => {}
        }
        let len = bytes.len();
        let input: Box<dyn oxideav_core::ReadSeek> = if mutant % 5 == 0 {
            let limit = rng.below(len.max(1));
            Box::new(LateUnseekable {
                inner: Cursor::new(bytes),
                read: 0,
                limit,
            })
        } else {
            Box::new(Cursor::new(bytes))
        };
        let Ok(mut demuxer) = open_demuxer(input, &NullCodecResolver) else {
            continue;
        };
        opened += 1;
        drain(&mut *demuxer, len, mutant);
        let streams = demuxer.streams().len() as u32;
        let pts = 126_000 + rng.below(60_000) as i64;
        if demuxer
            .seek_to(rng.below(streams as usize) as u32, pts)
            .is_ok()
        {
            drain(&mut *demuxer, len, mutant);
        }
    }
    // Most mutants keep a usable PAT/PMT: the parser stage really ran.
    assert!(
        opened > MUTANTS / 2,
        "only {opened} of {MUTANTS} mutants opened"
    );
}
