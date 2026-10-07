// SPDX-License-Identifier: LGPL-2.1-or-later
// Ports of FFmpeg 2da55bf code; each file names its sources.
//
// This file is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1, or (at your option) any later version.
// It is distributed WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See LICENSE-LGPL.

//! FFmpeg's parser stage for the codecs FFmpeg's MPEG-TS demuxer parses:
//! the codec parsers (`h264`, `mpegvideo`, `mpegaudio`, `aac_ac3`,
//! `opus`, `lcevc`), the generic parser bookkeeping (`parser`) and the
//! demuxer-side packet timing (`timing`). [`crate::parsed::ParsedDemuxer`]
//! drives them.

pub(crate) mod aac_ac3;
pub(crate) mod bits;
pub(crate) mod dca;
pub(crate) mod h264;
pub(crate) mod latm;
pub(crate) mod lcevc;
pub(crate) mod mlp;
pub(crate) mod mpegaudio;
pub(crate) mod mpegvideo;
pub(crate) mod opus;
pub(crate) mod parser;
pub(crate) mod timing;
