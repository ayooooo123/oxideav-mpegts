//! Opening reads ahead as FFmpeg's avformat_find_stream_info does, but on
//! input that never ends the read-ahead, the PSI search and the packet
//! queue stay bounded: endless null packets after a PMT, endless one-byte
//! PES, endless bytes without a PAT. Each input is generated as it is
//! read and panics past 300 MiB, so an unbounded read fails the test
//! instead of hanging it. A parser holds at most 32 MiB of one unit, and
//! reports a longer one as an error rather than dropping its bytes.

use std::io::{Read, Seek, SeekFrom};

use oxideav_core::{Demuxer, Error, NullCodecResolver};
use oxideav_mpegts::{open_demuxer, TS_PACKET_LEN};

const PANIC_AT: u64 = 300 << 20;

/// CRC-32/MPEG-2 of a PSI section.
fn crc32_mpeg2(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &b in data {
        crc ^= u32::from(b) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 {
                (crc << 1) ^ 0x04C1_1DB7
            } else {
                crc << 1
            };
        }
    }
    crc
}

fn section_packet(pid: u16, mut section: Vec<u8>) -> [u8; TS_PACKET_LEN] {
    section.extend_from_slice(&crc32_mpeg2(&section).to_be_bytes());
    let mut p = [0xFF; TS_PACKET_LEN];
    p[..5].copy_from_slice(&[0x47, 0x40 | (pid >> 8) as u8, pid as u8, 0x10, 0]);
    p[5..5 + section.len()].copy_from_slice(&section);
    p
}

/// PAT (program 1 on PID 0x1000) and a PMT listing `streams` as
/// (stream_type, PID, descriptors).
fn psi(streams: &[(u8, u16, &[u8])]) -> Vec<u8> {
    let pat = vec![0x00, 0xB0, 13, 0, 1, 0xC1, 0, 0, 0, 1, 0xF0, 0x00];
    let mut es = Vec::new();
    for &(st, pid, desc) in streams {
        es.extend_from_slice(&[
            st,
            0xE0 | (pid >> 8) as u8,
            pid as u8,
            0xF0,
            desc.len() as u8,
        ]);
        es.extend_from_slice(desc);
    }
    let len = 9 + es.len() + 4;
    let mut pmt = vec![
        0x02,
        0xB0 | (len >> 8) as u8,
        len as u8,
        0,
        1,
        0xC1,
        0,
        0,
        0xE1,
        0x00,
        0xF0,
        0,
    ];
    pmt.extend_from_slice(&es);
    let mut out = section_packet(0, pat).to_vec();
    out.extend_from_slice(&section_packet(0x1000, pmt));
    out
}

/// `head`, then `tail(i)` for the i-th packet after it, forever.
struct Endless<F: Fn(u64) -> [u8; TS_PACKET_LEN]> {
    head: Vec<u8>,
    tail: F,
    pos: u64,
    max_read: u64,
}

impl<F: Fn(u64) -> [u8; TS_PACKET_LEN]> Read for Endless<F> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        assert!(
            self.pos < PANIC_AT,
            "read {} bytes: the read is not bounded",
            self.pos
        );
        let mut n = 0;
        while n < buf.len() {
            let at = self.pos as usize;
            let byte = if at < self.head.len() {
                self.head[at]
            } else {
                let k = (at - self.head.len()) as u64;
                (self.tail)(k / TS_PACKET_LEN as u64)[(k % TS_PACKET_LEN as u64) as usize]
            };
            buf[n] = byte;
            n += 1;
            self.pos += 1;
        }
        self.max_read = self.max_read.max(self.pos);
        Ok(n)
    }
}

impl<F: Fn(u64) -> [u8; TS_PACKET_LEN]> Seek for Endless<F> {
    fn seek(&mut self, to: SeekFrom) -> std::io::Result<u64> {
        match to {
            SeekFrom::Start(p) => self.pos = p,
            SeekFrom::Current(d) => self.pos = self.pos.checked_add_signed(d).unwrap(),
            SeekFrom::End(_) => return Err(std::io::Error::other("endless input has no end")),
        }
        Ok(self.pos)
    }
}

/// Opens an endless input; returns the open result and the furthest
/// byte read.
fn open_endless<F>(head: Vec<u8>, tail: F) -> (oxideav_core::Result<Box<dyn Demuxer>>, u64)
where
    F: Fn(u64) -> [u8; TS_PACKET_LEN] + Send + 'static,
{
    use std::sync::atomic::{AtomicU64, Ordering};
    use std::sync::Arc;
    struct Tracked<F: Fn(u64) -> [u8; TS_PACKET_LEN]>(Endless<F>, Arc<AtomicU64>);
    impl<F: Fn(u64) -> [u8; TS_PACKET_LEN]> Read for Tracked<F> {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let n = self.0.read(buf)?;
            self.1.store(self.0.max_read, Ordering::Relaxed);
            Ok(n)
        }
    }
    impl<F: Fn(u64) -> [u8; TS_PACKET_LEN]> Seek for Tracked<F> {
        fn seek(&mut self, to: SeekFrom) -> std::io::Result<u64> {
            self.0.seek(to)
        }
    }
    let read = Arc::new(AtomicU64::new(0));
    let input = Tracked(
        Endless {
            head,
            tail,
            pos: 0,
            max_read: 0,
        },
        read.clone(),
    );
    let result = open_demuxer(Box::new(input), &NullCodecResolver);
    (result, read.load(Ordering::Relaxed))
}

fn null_packet(_: u64) -> [u8; TS_PACKET_LEN] {
    let mut p = [0xFF; TS_PACKET_LEN];
    p[..4].copy_from_slice(&[0x47, 0x1F, 0xFF, 0x10]);
    p
}

#[test]
fn endless_null_packets_after_the_pmt_end_the_read_ahead() {
    // An H.264 stream the PMT declares but no PES ever carries.
    let (result, read) = open_endless(psi(&[(0x1B, 0x100, &[])]), null_packet);
    let demuxer = result.expect("opens");
    assert_eq!(demuxer.streams().len(), 1);
    assert!(read <= 130 << 20, "read {read} bytes while opening");
}

#[test]
fn endless_one_byte_pes_keep_the_queue_bounded() {
    // DVB subtitles (a stream without a parser): every TS packet carries a
    // whole PES with one payload byte, as small as packets get.
    let head = psi(&[(0x06, 0x101, &[0x59, 8, b'e', b'n', b'g', 0x10, 0, 1, 0, 1])]);
    let (result, read) = open_endless(head, |i| {
        let mut p = [0xFF; TS_PACKET_LEN];
        let pts = 90_000 + i * 3600;
        p[..4].copy_from_slice(&[0x47, 0x41, 0x01, 0x10 | (i & 15) as u8]);
        p[4..14].copy_from_slice(&[
            0,
            0,
            1,
            0xBD,
            0,
            9,
            0x80,
            0x80,
            5,
            0x21 | ((pts >> 29) & 0x0E) as u8,
        ]);
        p[14..18].copy_from_slice(&[
            (pts >> 22) as u8,
            0x01 | ((pts >> 14) as u8 & 0xFE),
            (pts >> 7) as u8,
            0x01 | ((pts << 1) as u8 & 0xFE),
        ]);
        p[18] = 0x20;
        p
    });
    let mut demuxer = result.expect("opens");
    assert!(read <= 130 << 20, "read {read} bytes while opening");
    let p = demuxer.next_packet().expect("packets follow");
    assert_eq!((p.stream_index, p.data.as_slice()), (0, &[0x20][..]));
}

#[test]
fn endless_bytes_without_a_pat_fail_the_open() {
    let (result, read) = open_endless(Vec::new(), null_packet);
    assert!(
        matches!(result, Err(Error::ResourceExhausted(_))),
        "{:?}",
        result.err()
    );
    assert!(read <= 130 << 20, "read {read} bytes while opening");
}

#[test]
fn an_access_unit_past_the_parser_bound_is_an_error() {
    // H.264: an IDR slice NAL followed by 33 MiB without a start code,
    // in 64 KiB PES, then a second IDR slice.
    let mut ts = psi(&[(0x1B, 0x100, &[])]);
    let mut cc = 0u8;
    let mut pes = |payload: &[u8], pts: u64, ts: &mut Vec<u8>| {
        let mut data = vec![0, 0, 1, 0xE0, 0, 0, 0x80, 0x80, 5];
        data.extend_from_slice(&[
            0x21 | ((pts >> 29) & 0x0E) as u8,
            (pts >> 22) as u8,
            0x01 | ((pts >> 14) as u8 & 0xFE),
            (pts >> 7) as u8,
            0x01 | ((pts << 1) as u8 & 0xFE),
        ]);
        data.extend_from_slice(payload);
        for (k, chunk) in data.chunks(184).enumerate() {
            let mut p = [0xFF; TS_PACKET_LEN];
            let pusi = if k == 0 { 0x40 } else { 0 };
            if chunk.len() == 184 {
                p[..4].copy_from_slice(&[0x47, pusi | 0x01, 0x00, 0x10 | cc]);
                p[4..].copy_from_slice(chunk);
            } else {
                let stuff = 184 - chunk.len();
                p[..4].copy_from_slice(&[0x47, pusi | 0x01, 0x00, 0x30 | cc]);
                p[4] = (stuff - 1) as u8;
                if stuff > 1 {
                    p[5] = 0;
                }
                p[4 + stuff..].copy_from_slice(chunk);
            }
            cc = (cc + 1) & 15;
            ts.extend_from_slice(&p);
        }
    };
    let filler = vec![0x55u8; 64 << 10];
    let mut first = vec![0, 0, 0, 1, 0x65, 0x88];
    first.extend_from_slice(&filler);
    pes(&first, 90_000, &mut ts);
    for i in 1..=(33 << 20) / filler.len() as u64 {
        pes(&filler, 90_000 + i * 3600, &mut ts);
    }
    pes(
        &[0, 0, 0, 1, 0x65, 0x88, 0x80],
        90_000 + 600 * 3600,
        &mut ts,
    );

    let mut demuxer =
        open_demuxer(Box::new(std::io::Cursor::new(ts)), &NullCodecResolver).expect("opens");
    let mut largest = 0;
    let error = loop {
        match demuxer.next_packet() {
            Ok(p) => largest = largest.max(p.data.len()),
            Err(Error::Eof) => panic!("no error; the largest unit held {largest} bytes"),
            Err(e) => break e,
        }
    };
    assert!(matches!(error, Error::ResourceExhausted(_)), "{error}");
}
