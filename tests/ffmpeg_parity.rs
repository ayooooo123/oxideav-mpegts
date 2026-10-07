//! The registry demuxer against FFmpeg's on a stream FFmpeg muxed:
//! H.264 with B-frames, AC-3 (stream type 0x81, `AC-3` registration),
//! MP2 (0x03), ADTS AAC (0x0F) and E-AC-3 (ATSC stream type 0x87),
//! several audio frames per PES.
//!
//! `data/ffmpeg_mux.ts` was made by FFmpeg 9.0.2 with
//!
//! ```text
//! ffmpeg -f lavfi -i testsrc=size=64x48:rate=25 -f lavfi -i sine=frequency=440:sample_rate=48000 \
//!   -t 0.6 -map 0:v -map 1:a -map 1:a -map 1:a -map 1:a -c:v libx264 -preset veryfast -bf 2 -g 8 \
//!   -pix_fmt yuv420p -c:a:0 ac3 -b:a:0 96k -c:a:1 mp2 -b:a:1 64k -c:a:2 aac -b:a:2 32k \
//!   -c:a:3 eac3 -b:a:3 96k -f mpegts ffmpeg_mux.ts
//! ```
//!
//! and `data/ffmpeg_mux.packets` is what `ffprobe -show_data_hash CRC32
//! -show_entries packet=stream_index,pts,dts,duration,size,flags,data_hash
//! -of compact=p=0:nk=1` prints for it — and for the same bytes without
//! their leading SDT/PAT/PMT, or with a partial packet after them.

use std::io::Cursor;

use oxideav_core::{Demuxer, Error, NullCodecResolver, Packet, PixelFormat, Rational};
use oxideav_mpegts::{open_demuxer, MpegTsDemuxer, TS_PACKET_LEN};

const TS: &[u8] = include_bytes!("data/ffmpeg_mux.ts");
const FFPROBE: &str = include_str!("data/ffmpeg_mux.packets");

/// CRC-32 as FFmpeg's `CRC32` hash computes it (IEEE, reflected).
fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (crc & 1).wrapping_neg());
        }
    }
    !crc
}

/// A packet as one `ffprobe` compact line.
fn row(p: &Packet) -> String {
    let t = |v: Option<i64>| v.map_or_else(|| "N/A".to_owned(), |v| v.to_string());
    format!(
        "{}|{}|{}|{}|{}|{}__|CRC32:{:08x}|",
        p.stream_index,
        t(p.pts),
        t(p.dts),
        t(p.duration),
        p.data.len(),
        if p.flags.keyframe { 'K' } else { '_' },
        crc32(&p.data)
    )
}

/// Every packet until end of stream; any other error fails the test.
fn drain(demuxer: &mut dyn Demuxer) -> Vec<String> {
    let mut rows = Vec::new();
    loop {
        match demuxer.next_packet() {
            Ok(p) => rows.push(row(&p)),
            Err(Error::Eof) => return rows,
            Err(e) => panic!("demux error after {} packets: {e}", rows.len()),
        }
    }
}

fn open(bytes: Vec<u8>) -> Box<dyn Demuxer> {
    open_demuxer(Box::new(Cursor::new(bytes)), &NullCodecResolver).expect("open")
}

fn assert_ffmpeg_packets(ours: &[String]) {
    let theirs: Vec<&str> = FFPROBE.lines().collect();
    if let Some(i) = (0..ours.len().max(theirs.len()))
        .find(|&i| ours.get(i).map(String::as_str) != theirs.get(i).copied())
    {
        panic!(
            "packet {i} differs ({} ours, {} FFmpeg):\n  ours   {:?}\n  FFmpeg {:?}",
            ours.len(),
            theirs.len(),
            ours.get(i),
            theirs.get(i)
        );
    }
}

#[test]
fn packets_equal_ffmpeg() {
    assert_ffmpeg_packets(&drain(&mut *open(TS.to_vec())));
}

#[test]
fn pes_before_the_first_pmt_are_delivered() {
    // Without the leading SDT/PAT/PMT, the first PES of each stream
    // precede the first PMT; FFmpeg reads them after seeking back.
    assert_ffmpeg_packets(&drain(&mut *open(TS[3 * TS_PACKET_LEN..].to_vec())));
}

#[test]
fn a_partial_last_packet_ends_the_stream() {
    let mut bytes = TS.to_vec();
    bytes.extend_from_slice(&TS[3 * TS_PACKET_LEN..3 * TS_PACKET_LEN + 40]);
    assert_ffmpeg_packets(&drain(&mut *open(bytes.clone())));
    // The PES-level demuxer ends there too, with the same PES.
    let mut whole = MpegTsDemuxer::open_program(Box::new(Cursor::new(TS.to_vec())), 1).unwrap();
    let mut cut = MpegTsDemuxer::open_program(Box::new(Cursor::new(bytes)), 1).unwrap();
    assert_eq!(drain(&mut cut), drain(&mut whole));
}

#[test]
fn stream_parameters_equal_ffmpeg() {
    // ffprobe -show_entries stream=codec_name,width,height,pix_fmt,
    // r_frame_rate,sample_rate,channels
    let demuxer = open(TS.to_vec());
    let streams = demuxer.streams();
    let codecs: Vec<&str> = streams.iter().map(|s| s.params.codec_id.as_str()).collect();
    assert_eq!(codecs, ["h264", "ac3", "mp2", "aac", "eac3"]);
    let video = &streams[0].params;
    assert_eq!((video.width, video.height), (Some(64), Some(48)));
    assert_eq!(video.pixel_format, Some(PixelFormat::Yuv420P));
    assert_eq!(video.frame_rate, Some(Rational::new(25, 1)));
    for audio in &streams[1..] {
        assert_eq!(
            audio.params.sample_rate,
            Some(48_000),
            "{}",
            audio.params.codec_id.as_str()
        );
    }
    // FFmpeg's parsers name AC-3, MP2 and ADTS AAC channel counts (its
    // E-AC-3 count comes from decoding, not the parser).
    for audio in &streams[1..4] {
        assert_eq!(
            audio.params.channels,
            Some(1),
            "{}",
            audio.params.codec_id.as_str()
        );
    }
}

#[test]
fn random_access_indicator_marks_the_video_keyframes() {
    // FFmpeg's muxer sets the random_access_indicator on each key
    // frame's PES; it reaches the consumer as container_keyframe.
    let mut demuxer = open(TS.to_vec());
    let mut keyframes = 0;
    loop {
        let p = match demuxer.next_packet() {
            Ok(p) => p,
            Err(Error::Eof) => break,
            Err(e) => panic!("{e}"),
        };
        if p.stream_index == 0 {
            assert_eq!(
                demuxer.packet_metadata().container_keyframe,
                p.flags.keyframe,
                "video pts {:?}",
                p.pts
            );
            keyframes += usize::from(p.flags.keyframe);
        }
    }
    assert_eq!(keyframes, 2);
}
