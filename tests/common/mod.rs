//! Packet tables as `ffprobe -show_data_hash CRC32 -show_entries
//! packet=stream_index,pts,dts,duration,size,flags,data_hash -of
//! compact=p=0:nk=1` prints them, for comparing the registry demuxer with
//! FFmpeg 9.0.2's on the fixtures in `data/`.

#![allow(dead_code)]

use std::io::Cursor;

use oxideav_core::{Demuxer, Error, MediaType, NullCodecResolver, Packet};
use oxideav_mpegts::open_demuxer;

/// CRC-32 as FFmpeg's `CRC32` hash computes it (IEEE, reflected).
pub fn crc32(data: &[u8]) -> u32 {
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
pub fn row(p: &Packet) -> String {
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
pub fn drain(demuxer: &mut dyn Demuxer) -> Vec<String> {
    let mut rows = Vec::new();
    loop {
        match demuxer.next_packet() {
            Ok(p) => rows.push(row(&p)),
            Err(Error::Eof) => return rows,
            Err(e) => panic!("demux error after {} packets: {e}", rows.len()),
        }
    }
}

pub fn open(bytes: Vec<u8>) -> Box<dyn Demuxer> {
    open_demuxer(Box::new(Cursor::new(bytes)), &NullCodecResolver).expect("open")
}

/// `ours` equals the ffprobe table `theirs`, row by row.
pub fn assert_table(ours: &[String], theirs: &str) {
    let theirs: Vec<&str> = theirs.lines().collect();
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

/// A stream as `ffprobe -show_streams` states it: codec name and the
/// parameters the test checks (`None`: not checked).
#[derive(Debug)]
pub struct Want {
    pub codec: &'static str,
    pub kind: MediaType,
    pub sample_rate: Option<u32>,
    pub channels: Option<u16>,
    pub size: Option<(u32, u32)>,
}

impl Want {
    pub fn audio(codec: &'static str, sample_rate: u32, channels: u16) -> Self {
        Want {
            codec,
            kind: MediaType::Audio,
            sample_rate: Some(sample_rate),
            channels: Some(channels),
            size: None,
        }
    }
    pub fn video(codec: &'static str, width: u32, height: u32) -> Self {
        Want {
            codec,
            kind: MediaType::Video,
            sample_rate: None,
            channels: None,
            size: Some((width, height)),
        }
    }
    pub fn data(codec: &'static str) -> Self {
        Want {
            codec,
            kind: MediaType::Data,
            sample_rate: None,
            channels: None,
            size: None,
        }
    }
    pub fn any(codec: &'static str, kind: MediaType) -> Self {
        Want {
            codec,
            kind,
            sample_rate: None,
            channels: None,
            size: None,
        }
    }
}

/// The fixture `data/<name>.<ext>` opens with `want`'s streams and its
/// packets equal `data/<name>.packets`.
pub fn assert_fixture(file: &str, want: &[Want]) {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let bytes = std::fs::read(dir.join(file)).expect("fixture");
    let table = std::fs::read_to_string(dir.join(file).with_extension("packets")).expect("table");
    let mut demuxer = open(bytes);
    let streams = demuxer.streams();
    let got: Vec<String> = streams
        .iter()
        .map(|s| {
            let p = &s.params;
            format!(
                "{} {:?} rate={:?} ch={:?} size={:?}",
                p.codec_id.as_str(),
                p.media_type,
                p.sample_rate,
                p.channels,
                p.width.zip(p.height)
            )
        })
        .collect();
    assert_eq!(streams.len(), want.len(), "{file}: streams {got:?}");
    for (s, w) in streams.iter().zip(want) {
        let p = &s.params;
        let ok = p.codec_id.as_str() == w.codec
            && p.media_type == w.kind
            && (w.sample_rate.is_none() || p.sample_rate == w.sample_rate)
            && (w.channels.is_none() || p.channels == w.channels)
            && (w.size.is_none() || p.width.zip(p.height) == w.size);
        assert!(
            ok,
            "{file}: stream {} is {:?}, FFmpeg's is {w:?}",
            s.index, got[s.index as usize]
        );
    }
    assert_table(&drain(&mut *demuxer), &table);
}
