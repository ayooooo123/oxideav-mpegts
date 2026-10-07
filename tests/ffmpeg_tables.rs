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

mod common;

use common::{assert_fixture, assert_table, drain, open, Want};

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
