//! The registry demuxer against FFmpeg 9.0.2 on small fixtures, each a
//! stream FFmpeg muxed from lavfi sources (or such a stream with bytes
//! changed as stated): streams and packet tables equal ffprobe's.
//!
//! - `aac_pce_71.ts`: `-f lavfi -i "sine=…,aformat=channel_layouts=7.1(wide)"
//!   -t 0.3 -c:a aac -b:a 256k -f mpegts` (ADTS channel configuration 0,
//!   a program config element per frame).
//! - `aac_pce_stereo.ts`: stereo, `-c:a aac -aac_pce 1 -b:a 64k`.
//! - `aac_config7.ts`: `aac_pce_71.ts` with every ADTS header's
//!   channel_configuration rewritten to 7, which FFmpeg's decoder reads
//!   as eight channels (libavcodec/mpeg4audio.c ff_mpeg4audio_channels).
//! - `h264_late_sps.ts`: `-f lavfi -i testsrc=size=64x48:rate=25 -t 0.96
//!   -c:v libx264 -preset veryfast -bf 2 -g 8 -keyint_min 8 -x264-params
//!   repeat-headers=1:scenecut=0 -pix_fmt yuv420p -f mpegts`, with the SPS
//!   and PPS NAL units cut from the first access unit: the first eight
//!   units are timed with the parameter sets the read-ahead found later.
//! - `h264_no_vui_timing.ts`, `…_short.ts`, `…_ntsc.ts`: `-f lavfi -i
//!   testsrc=size=64x48:rate=25 -t 1.6` (25 fps, 1.6 s; 0.48 s; and
//!   `rate=30000/1001 -t 1.2`) `-c:v libx264 -preset veryfast -bf 2 -g 8
//!   -pix_fmt yuv420p -f mpegts`, with each SPS's timing_info_present_flag
//!   cleared (its 65 bits removed): the SPS states no frame rate, so FFmpeg
//!   times the packets with the rate it estimates from their DTS (the gcd
//!   of the deltas, or the closest standard rate when fewer than 16).
//! - `h264_pyramid_no_restriction.ts`: `-f lavfi -i testsrc=size=64x48:rate=25
//!   -t 1.6 -c:v libx264 -preset veryfast -bf 3 -g 16 -x264-params
//!   b-pyramid=normal:b-adapt=0:scenecut=0 -pix_fmt yuv420p -f mpegts`,
//!   each SPS without its bitstream restriction and each PES with its PTS
//!   only: the reorder depth (2) comes from the picture order FFmpeg's
//!   decoder sees, and decides the DTS FFmpeg gives.
//! - `opus_51.ts`: 5.1 Opus (libopus) as FFmpeg's muxer writes it: an
//!   `Opus` registration, then the DVB extension descriptor 0x7F/0x80
//!   with channel configuration 6. `opus_ext_first.ts`: the same with the
//!   extension descriptor first, as FATE's `test-8-7.1.opus-small.ts`
//!   orders them.
//! - `id3.ts`: FATE's `mpegts/id3.ts` (stream type 0x15, a metadata
//!   descriptor naming `ID3 `). `timed_id3.ts`: FFmpeg-muxed MP2 (lavfi
//!   sine) with such a stream added to the PMT and two ID3 PES.
//! - `vvc.ts`: FATE's `lcevc/L_VVC_640x360p_8bit8bit_2D_dd.ts` (stream
//!   type 0x33). `lcevc_dual_track.ts`: FATE's
//!   `lcevc/L_H264_640x360p_8bit8bit_2D_dd_dualTrack.ts` (H.264 and an
//!   LCEVC enhancement track, stream type 0x36).
//! - `pmt_change.ts`: an FFmpeg-muxed MP2 file, then one with MP2 and
//!   AC-3 on a new PID whose PMT is rewritten to version 1.
//! - `dts_core.ts`: a stereo DTS core (stream type 0x82), one frame per
//!   PES. `dts_two_per_pes.ts`: the first 203 packets of FATE's
//!   `dts/dts.ts`, cut where a PES starts (private PES, two 5.1 frames
//!   per PES).
//!   `dtshd_ma.ts`: FATE's `dts/master_audio_7.1_24bit.dts` remuxed by
//!   `ffmpeg -t 0.3 -c copy -f mpegts` (a core and its DTS-HD extension
//!   substream in each frame).
//! - `truehd.ts`: FFmpeg-muxed stereo TrueHD (stream type 0x83).
//!   `truehd_hdmv.m2ts`: the same muxed as an m2ts, whose PMT carries the
//!   `HDMV` registration.

mod common;

use common::{assert_fixture, assert_table, drain, open, Want};

/// A packet row without its duration and flags: the fields a stream
/// FFmpeg runs through a parser this crate does not port (VVC, LCEVC)
/// still matches.
fn without_parser_fields(row: &str) -> String {
    let f: Vec<&str> = row.split('|').collect();
    format!("{}|{}|{}|{}|{}", f[0], f[1], f[2], f[4], f[6])
}

/// The fixture opens with `want`'s streams; its rows equal ffprobe's,
/// those of `unparsed` streams without their parser fields.
fn assert_fixture_unparsed(file: &str, want: &[Want], unparsed: &[&str]) {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let bytes = std::fs::read(dir.join(file)).expect("fixture");
    let table = std::fs::read_to_string(dir.join(file).with_extension("packets")).expect("table");
    let mut demuxer = open(bytes);
    let streams: Vec<(String, oxideav_core::MediaType)> = demuxer
        .streams()
        .iter()
        .map(|s| (s.params.codec_id.as_str().to_owned(), s.params.media_type))
        .collect();
    let wanted: Vec<(String, oxideav_core::MediaType)> =
        want.iter().map(|w| (w.codec.to_owned(), w.kind)).collect();
    assert_eq!(streams, wanted, "{file}");
    let strip = |row: &str| {
        if unparsed.contains(&row.split('|').next().unwrap_or("")) {
            without_parser_fields(row)
        } else {
            row.to_owned()
        }
    };
    let ours: Vec<String> = drain(&mut *demuxer).iter().map(|r| strip(r)).collect();
    let theirs: String = table.lines().map(|r| strip(r) + "\n").collect();
    assert_table(&ours, &theirs);
}

#[test]
fn opus_opens_with_the_channels_its_extension_descriptor_states() {
    // FFmpeg reads the first PMT twice (the header scan, then again after
    // seeking back), so the descriptor applies in either order.
    for file in ["opus_51.ts", "opus_ext_first.ts"] {
        assert_fixture(file, &[Want::audio("opus", 48_000, 6)]);
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
        let demuxer = open(std::fs::read(dir.join(file)).expect("fixture"));
        // ffprobe -show_streams -show_data: the OpusHead FFmpeg builds.
        assert_eq!(
            demuxer.streams()[0].params.extradata,
            b"OpusHead\x01\x06\x00\x00\x80\xbb\x00\x00\x00\x00\x01\x04\x02\x00\x04\x01\x02\x03\x05",
            "{file}"
        );
    }
}

#[test]
fn timed_id3_opens_as_a_data_stream() {
    assert_fixture("id3.ts", &[Want::data("timed_id3")]);
    assert_fixture(
        "timed_id3.ts",
        &[Want::audio("mp2", 48_000, 1), Want::data("timed_id3")],
    );
}

#[test]
fn vvc_opens() {
    assert_fixture_unparsed(
        "vvc.ts",
        &[Want::any("vvc", oxideav_core::MediaType::Video)],
        &["0"],
    );
}

#[test]
fn an_lcevc_enhancement_track_is_its_own_stream() {
    // FFmpeg's lcevc_parser splits and flags its units; durations come
    // from the rate estimated from their DTS.
    assert_fixture_unparsed(
        "lcevc_dual_track.ts",
        &[
            Want::any("h264", oxideav_core::MediaType::Video),
            Want::any("lcevc", oxideav_core::MediaType::Video),
        ],
        &[],
    );
}

#[test]
fn a_pmt_version_that_adds_a_pid_adds_its_stream() {
    // The new PID's PES count from the PMT that lists it on: FFmpeg's
    // header scan stops at the first PMT, so no seek back covers it.
    assert_fixture(
        "pmt_change.ts",
        &[Want::audio("mp2", 48_000, 1), Want::audio("ac3", 48_000, 1)],
    );
}

#[test]
fn dts_frames_are_split_and_timed_as_ffmpeg_splits_them() {
    assert_fixture("dts_core.ts", &[Want::audio("dts", 48_000, 2)]);
    assert_fixture("dts_two_per_pes.ts", &[Want::audio("dts", 48_000, 6)]);
    // FFmpeg's 8 channels come from the decoder reading the lossless
    // extension; the parser states the rate.
    let dts_hd = Want {
        sample_rate: Some(48_000),
        ..Want::any("dts", oxideav_core::MediaType::Audio)
    };
    assert_fixture("dtshd_ma.ts", &[dts_hd]);
}

#[test]
fn truehd_access_units_are_split_and_timed_as_ffmpeg_splits_them() {
    assert_fixture("truehd.ts", &[Want::audio("truehd", 48_000, 2)]);
    // HDMV TrueHD carries an AC-3 version of the track on its PID
    // (extended_stream_id 0x76); FFmpeg lists it as a second stream.
    assert_fixture(
        "truehd_hdmv.m2ts",
        &[
            Want::audio("truehd", 48_000, 2),
            Want::any("ac3", oxideav_core::MediaType::Audio),
        ],
    );
}

#[test]
fn adts_channel_configuration_seven_is_eight_channels() {
    assert_fixture("aac_config7.ts", &[Want::audio("aac", 48_000, 8)]);
}

#[test]
fn adts_program_config_elements_state_the_channels() {
    assert_fixture("aac_pce_71.ts", &[Want::audio("aac", 48_000, 8)]);
    assert_fixture("aac_pce_stereo.ts", &[Want::audio("aac", 48_000, 2)]);
}

#[test]
fn parameter_sets_found_while_reading_ahead_time_the_first_units() {
    assert_fixture("h264_late_sps.ts", &[Want::video("h264", 64, 48)]);
}

#[test]
fn a_seek_keeps_the_parameter_sets_found_while_reading_ahead() {
    // A seek opens new parsers too. Landing on the first unit, the units
    // after it are timed as a read from the start times them.
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data");
    let bytes = std::fs::read(dir.join("h264_late_sps.ts")).expect("fixture");
    let table = std::fs::read_to_string(dir.join("h264_late_sps.packets")).expect("table");
    let mut demuxer = open(bytes);
    drain(&mut *demuxer);
    demuxer.seek_to(0, 133_200).expect("seek");
    assert_table(&drain(&mut *demuxer), &table);
}

#[test]
fn video_without_a_stated_rate_is_timed_with_the_estimated_one() {
    for file in [
        "h264_no_vui_timing.ts",
        "h264_no_vui_timing_short.ts",
        "h264_no_vui_timing_ntsc.ts",
    ] {
        assert_fixture(file, &[Want::video("h264", 64, 48)]);
    }
}

#[test]
fn the_reorder_depth_comes_from_picture_order_without_a_stated_one() {
    assert_fixture(
        "h264_pyramid_no_restriction.ts",
        &[Want::video("h264", 64, 48)],
    );
}
