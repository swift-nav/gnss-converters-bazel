use deku::DekuContainerWrite;

use serde::{Deserialize, Deserializer, Serialize};
use serde_json::Value;
use serde_json::{de::IoRead, StreamDeserializer};
use serde_with::{base64::Base64, serde_as};
use std::io::Read;

use crate as rtcm;

#[serde_as]
#[derive(Debug, Serialize)]
pub struct RtcmJson {
    #[serde(flatten)]
    pub timestamp: Option<Timestamp>,
    #[serde_as(as = "Base64")]
    pub rtcm_b64: Vec<u8>,
    #[serde(rename = "rtcm")]
    pub frame: RtcmFrame,
}

impl TryFrom<rtcm::Frame> for RtcmJson {
    type Error = rtcm::Error;
    fn try_from(value: rtcm::Frame) -> Result<Self, Self::Error> {
        let timestamp = None;
        let rtcm_b64 = value.to_bytes()?;
        let frame = match rtcm::Frame::from_slice(&rtcm_b64[..]) {
            Some(res) => {
                let frame = res?;
                RtcmFrame::Frame(Box::new(frame))
            }
            None => return Err(rtcm::Error::IncompleteDecoding),
        };
        Ok(Self {
            timestamp,
            rtcm_b64,
            frame,
        })
    }
}

impl<'de> Deserialize<'de> for RtcmJson {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[serde_as]
        #[derive(Debug, Serialize, Deserialize)]
        struct RtcmJsonRaw {
            #[serde(flatten)]
            pub timestamp: Option<Timestamp>,
            #[serde_as(as = "Base64")]
            pub rtcm_b64: Vec<u8>,
            #[serde(rename = "rtcm")]
            pub frame: Option<rtcm::Frame>,
        }
        let des = Value::deserialize(deserializer)?;
        // First check for rtcm::Frame input.
        let rtcm_frame: Result<rtcm::Frame, anyhow::Error> =
            Deserialize::deserialize(&des).map_err(|e| anyhow::format_err!(e.to_string()));
        let (timestamp, rtcm_b64) = match rtcm_frame {
            Ok(frame) => {
                let timestamp = None;
                (
                    timestamp,
                    frame.to_bytes().map_err(serde::de::Error::custom)?,
                )
            }
            Err(_) => {
                // If it is not an rtcm::Frame then assume it is an RtcmJson variant.
                let RtcmJsonRaw {
                    timestamp,
                    rtcm_b64,
                    ..
                } = Deserialize::deserialize(des).map_err(serde::de::Error::custom)?;
                (timestamp, rtcm_b64)
            }
        };

        // Due to the nested structure of this wrapper type, the rtcm::Frame
        // appears to break on deserialization. So we instead rely on the rtcm
        // buffer to generate the Frame.
        let frame = rtcm::Frame::from_slice(&rtcm_b64[..])
            .ok_or_else(|| serde::de::Error::custom("incomplete rtcm slice"))?;
        let frame = frame.map_err(serde::de::Error::custom)?;
        let frame = RtcmFrame::Frame(Box::new(frame));
        Ok(Self {
            timestamp,
            rtcm_b64,
            frame,
        })
    }
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(untagged)]
pub enum RtcmFrame {
    Frame(Box<rtcm::Frame>),
    None,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Timestamp {
    #[serde(deserialize_with = "Timestamp::de_from_string_or_i64")]
    pub seconds: i64,
    pub nanoseconds: i32,
}

impl Timestamp {
    fn de_from_string_or_i64<'de, D>(deserializer: D) -> Result<i64, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = serde_json::value::Value::deserialize(deserializer)?;
        if let Some(s) = s.as_str() {
            s.parse::<i64>().map_err(serde::de::Error::custom)
        } else {
            let s: i64 = s.as_i64().ok_or_else(|| {
                serde::de::Error::custom("unable to deserialize from string or i64")
            })?;
            Ok(s)
        }
    }
}

impl RtcmJson {
    pub fn iter_messages<'de, R: Read>(input: R) -> StreamDeserializer<'de, IoRead<R>, RtcmJson> {
        serde_json::Deserializer::from_reader(input).into_iter::<RtcmJson>()
    }
}

macro_rules! make_rtcm_json_converter {
    () => {
        pub fn from_json<R: Read, W: Write>(reader: R, writer: W) -> Result<(), rtcm::Error> {
            let reader = BufReader::new(reader);
            let mut writer = BufWriter::new(writer);
            RtcmJson::iter_messages(reader).try_for_each(|frame| {
                match frame {
                    Ok(frame) => {
                        let frame = Frame::from(frame);
                        let frame = serde_json::to_string(&frame)?;
                        writeln!(writer, "{frame}")?;
                    }
                    Err(err) => {
                        eprintln!("{err:?}");
                    }
                }
                Ok(())
            })
        }
    };
}

macro_rules! make_rtcm_json_try_converter {
    () => {
        pub fn from_json<R: Read, W: Write>(reader: R, writer: W) -> Result<(), rtcm::Error> {
            let reader = BufReader::new(reader);
            let mut writer = BufWriter::new(writer);
            RtcmJson::iter_messages(reader).try_for_each(|frame| {
                match frame {
                    Ok(frame) => {
                        let frame = Frame::try_from(frame)?;
                        let frame = serde_json::to_string(&frame)?;
                        writeln!(writer, "{frame}")?;
                    }
                    Err(err) => {
                        eprintln!("{err:?}");
                    }
                }
                Ok(())
            })
        }
    };
}

macro_rules! make_rtcm_converter {
    () => {
        pub fn from_raw<R: Read, W: Write>(reader: R, writer: W) -> Result<(), rtcm::Error> {
            let reader = BufReader::new(reader);
            let mut writer = BufWriter::new(writer);
            rtcm::iter_messages(reader).try_for_each(|frame| {
                match frame {
                    Ok(frame) => {
                        let frame = Frame::from(RtcmJson::try_from(frame)?);
                        let frame = serde_json::to_string(&frame)?;
                        writeln!(writer, "{frame}")?;
                    }
                    Err(err) => {
                        eprintln!("{err:?}");
                    }
                }
                Ok(())
            })
        }
    };
}

pub mod expanded_plus_timestamps {
    use super::{rtcm, serde_as, Base64, Deserialize, RtcmFrame, RtcmJson, Serialize, Timestamp};
    use clap::ValueEnum;
    use std::{
        fs::File,
        io::{BufReader, BufWriter, Read, Write},
        path::PathBuf,
    };
    #[serde_as]
    #[derive(Debug, Deserialize, Serialize)]
    pub struct Frame {
        #[serde(flatten)]
        pub timestamp: Timestamp,
        #[serde_as(as = "Base64")]
        pub rtcm_b64: Vec<u8>,
        #[serde(rename = "rtcm")]
        pub frame: RtcmFrame,
    }

    impl TryFrom<RtcmJson> for Frame {
        type Error = rtcm::Error;
        fn try_from(frame: RtcmJson) -> Result<Self, Self::Error> {
            let RtcmJson {
                timestamp,
                rtcm_b64,
                frame,
            } = frame;
            let timestamp = timestamp.ok_or(rtcm::Error::InvalidTimestamp)?;
            Ok(Self {
                timestamp,
                rtcm_b64,
                frame,
            })
        }
    }

    make_rtcm_json_try_converter!();

    pub fn from_json_with_splits<R: Read>(
        reader: R,
        mut writer: TimeSplitWriter,
    ) -> Result<(), rtcm::Error> {
        let reader = BufReader::new(reader);
        RtcmJson::iter_messages(reader).try_for_each(|frame| {
            match frame {
                Ok(frame) => {
                    let frame = Frame::try_from(frame)?;
                    let seconds = Some(frame.timestamp.seconds);
                    let frame = serde_json::to_string(&frame)?;
                    writer.serialize(frame, seconds)?;
                }
                Err(err) => {
                    eprintln!("{err:?}");
                }
            }
            Ok(())
        })
    }

    #[derive(Debug)]
    pub struct TimeSplitWriter {
        path: PathBuf,
        wtr: Option<BufWriter<File>>,
        split_interval: SplitInterval,
        current_split: usize,
        last_split_seconds: i64,
    }
    impl TimeSplitWriter {
        pub fn new(path: PathBuf, split_interval: Option<SplitInterval>) -> Self {
            let split_interval = split_interval.unwrap_or(SplitInterval::None);
            Self {
                path,
                wtr: None,
                split_interval,
                current_split: 0,
                last_split_seconds: 0,
            }
        }
        fn fetch_split_interval_path(&mut self, seconds: i64) -> Option<PathBuf> {
            let initialize_first_split = self.current_split == 0;
            let seconds_have_changed_since_last_split = seconds != self.last_split_seconds;
            let seconds_are_interval_of_desired_split = seconds % (self.split_interval as i64) == 0;

            if initialize_first_split
                || seconds_have_changed_since_last_split && seconds_are_interval_of_desired_split
            {
                self.last_split_seconds = seconds;
                self.current_split += 1;

                let parent = self.path.parent();
                let path = self.path.file_stem().expect("unable to extract file stem");
                let path = if let Some(parent) = parent {
                    PathBuf::from(parent).join(path)
                } else {
                    PathBuf::from(path)
                };
                let mut path = PathBuf::from(format!("{}_{}", path.display(), self.current_split));
                let extension = self.path.extension();
                if let Some(extension) = extension {
                    path.set_extension(extension);
                }
                Some(path)
            } else {
                None
            }
        }

        fn check_writer(&mut self, seconds: i64) -> Result<(), rtcm::Error> {
            let path = if !matches!(self.split_interval, SplitInterval::None) {
                self.fetch_split_interval_path(seconds)
            } else if self.wtr.is_none() {
                Some(self.path.clone())
            } else {
                None
            };
            if let Some(path) = path {
                let wtr = std::fs::File::create(path)?;
                let wtr = BufWriter::new(wtr);
                self.wtr = Some(wtr);
            }
            Ok(())
        }
        pub fn serialize(
            &mut self,
            frame: String,
            seconds: Option<i64>,
        ) -> Result<(), rtcm::Error> {
            if let Some(seconds) = seconds {
                self.check_writer(seconds)?;
            }
            let wtr = self.wtr.as_mut().expect("unable to open writer");
            writeln!(wtr, "{frame}")?;
            Ok(())
        }
    }

    #[derive(Debug, Clone, Copy, ValueEnum)]
    #[repr(i64)]
    pub enum SplitInterval {
        None = 1,
        Minute = 60,
        HalfHour = 60 * 30,
        Hour = 60 * 60,
    }
}

pub mod minimal_plus_timestamps {
    use super::{rtcm, serde_as, Base64, Deserialize, RtcmJson, Serialize, Timestamp};
    use std::io::{BufReader, BufWriter, Read, Write};
    #[serde_as]
    #[derive(Debug, Deserialize, Serialize)]
    pub struct Frame {
        #[serde(flatten)]
        pub timestamp: Timestamp,
        #[serde_as(as = "Base64")]
        pub rtcm_b64: Vec<u8>,
    }

    impl TryFrom<RtcmJson> for Frame {
        type Error = rtcm::Error;
        fn try_from(frame: RtcmJson) -> Result<Self, Self::Error> {
            let RtcmJson {
                timestamp,
                rtcm_b64,
                ..
            } = frame;
            let timestamp = timestamp.ok_or(rtcm::Error::InvalidTimestamp)?;
            Ok(Self {
                timestamp,
                rtcm_b64,
            })
        }
    }

    make_rtcm_json_try_converter!();
}

pub mod expanded {
    use super::{rtcm, Deserialize, RtcmFrame, RtcmJson, Serialize};
    use std::io::{BufReader, BufWriter, Read, Write};
    #[derive(Debug, Deserialize, Serialize)]
    pub struct Frame {
        #[serde(flatten)]
        pub frame: RtcmFrame,
    }

    impl From<RtcmJson> for Frame {
        fn from(frame: RtcmJson) -> Self {
            let RtcmJson { frame, .. } = frame;
            Self { frame }
        }
    }

    make_rtcm_json_converter!();
    make_rtcm_converter!();
}

pub mod minimal {
    use super::{rtcm, serde_as, Base64, Deserialize, RtcmJson, Serialize};
    use std::io::{BufReader, BufWriter, Read, Write};
    #[serde_as]
    #[derive(Debug, Deserialize, Serialize)]
    pub struct Frame {
        #[serde_as(as = "Base64")]
        pub rtcm_b64: Vec<u8>,
    }

    impl From<RtcmJson> for Frame {
        fn from(frame: RtcmJson) -> Self {
            let RtcmJson { rtcm_b64, .. } = frame;
            Self { rtcm_b64 }
        }
    }

    make_rtcm_json_converter!();
    make_rtcm_converter!();
}

pub mod raw {
    use super::{rtcm, RtcmJson};
    use std::io::{BufReader, BufWriter, Read, Write};
    #[derive(Debug)]
    pub struct Frame {
        pub bytes: Vec<u8>,
    }

    impl From<RtcmJson> for Frame {
        fn from(frame: RtcmJson) -> Self {
            let RtcmJson { rtcm_b64, .. } = frame;
            Self { bytes: rtcm_b64 }
        }
    }
    pub fn from_json<R: Read, W: Write>(reader: R, writer: W) -> Result<(), rtcm::Error> {
        let reader = BufReader::new(reader);
        let mut writer = BufWriter::new(writer);
        RtcmJson::iter_messages(reader).try_for_each(|frame| {
            match frame {
                Ok(frame) => {
                    let Frame { bytes } = Frame::from(frame);
                    writer
                        .write_all(&bytes[..])
                        .map_err(|e| anyhow::anyhow!(e.to_string()))?;
                }
                Err(err) => {
                    writeln!(writer, "{err:?}")?;
                }
            }
            Ok(())
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{
        expanded_plus_timestamps::{self, SplitInterval, TimeSplitWriter},
        *,
    };
    use std::{
        fs::File,
        io::{BufRead, BufReader, Write},
        path::{Path, PathBuf},
    };
    use tempfile::NamedTempFile;

    const EXPANDED_PLUS_TIMESTAMPS_PATH: &str =
        "../tests/data/piksi-5Hz_signed.expanded_plus_timestamps";
    const MINIMAL_PLUS_TIMESTAMPS_PATH: &str =
        "../tests/data/piksi-5Hz_signed.minimal_plus_timestamps";
    const MINIMAL_PLUS_TIMESTAMPS_PATH_WITH_SECONDS_AS_STRINGS: &str =
        "../tests/data/piksi-5Hz_signed_seconds_as_strings.minimal_plus_timestamps";
    const RTCM_PATH: &str = "../tests/data/piksi-5Hz_signed.rtcm";
    const MINIMAL_PATH: &str = "../tests/data/piksi-5Hz_signed.minimal";
    const EXPANDED_PATH: &str = "../tests/data/piksi-5Hz_signed.expanded";
    const TIMESTAMPS_PER_SECOND: i64 = 1;
    const NUM_ROWS: i64 = 220;

    fn create_test_file<P: AsRef<Path>>(path: P) {
        let mut minimal_plus_timestamps_file = File::create(&path).unwrap();
        let rtcm_file = File::open(RTCM_PATH).unwrap();
        let mut count = 0;
        rtcm::iter_messages(rtcm_file)
            .try_for_each(|frame| {
                let rtcm_b64 = frame.unwrap().to_bytes().unwrap();
                let seconds = count / TIMESTAMPS_PER_SECOND;
                #[allow(clippy::modulo_one)]
                // If we ever choose another file this logic should just work.
                let nanoseconds = (count % TIMESTAMPS_PER_SECOND) as i32;
                let timestamp = Timestamp {
                    seconds,
                    nanoseconds,
                };
                let packet = minimal_plus_timestamps::Frame {
                    timestamp,
                    rtcm_b64,
                };
                count += 1;

                let json = serde_json::to_string(&packet).unwrap();
                writeln!(&mut minimal_plus_timestamps_file, "{}", json)
            })
            .unwrap();
    }

    #[test]
    fn test_minimal_plus_timestamps2expanded_plus_timestamps_with_split_none() {
        let minimal_plus_timestamps_path = PathBuf::from(MINIMAL_PLUS_TIMESTAMPS_PATH);
        let minimal_plus_timestamps_file = File::open(minimal_plus_timestamps_path).unwrap();
        let reader = BufReader::new(minimal_plus_timestamps_file);
        let expanded_plus_timestamps_file = tempfile::NamedTempFile::new().unwrap();
        let expanded_plus_timestamps_file =
            expanded_plus_timestamps_file.into_temp_path().to_path_buf();
        let writer = TimeSplitWriter::new(expanded_plus_timestamps_file.clone(), None);
        expanded_plus_timestamps::from_json_with_splits(reader, writer).unwrap();
        let f = File::open(expanded_plus_timestamps_file).unwrap();
        let f = BufReader::new(f);
        let lines = f.lines().count();
        assert_eq!(lines as i64, NUM_ROWS);
    }

    #[test]
    fn test_minimal_plus_timestamps2expanded_plus_timestamps_with_split() {
        let minimal_plus_timestamps_path = PathBuf::from(MINIMAL_PLUS_TIMESTAMPS_PATH);
        let minimal_plus_timestamps_file = File::open(minimal_plus_timestamps_path).unwrap();
        let reader = BufReader::new(minimal_plus_timestamps_file);

        let expanded_plus_timestamps_dir = tempfile::tempdir().unwrap();
        let expanded_plus_timestamps_dir = expanded_plus_timestamps_dir.into_path();

        let prefix = "asdf";
        let expanded_plus_timestamps_file =
            expanded_plus_timestamps_dir.join(format!("{prefix}.json"));
        let writer =
            TimeSplitWriter::new(expanded_plus_timestamps_file, Some(SplitInterval::Minute));
        expanded_plus_timestamps::from_json_with_splits(reader, writer).unwrap();

        let mut count = 1;
        let timestamps_per_minute = SplitInterval::Minute as i64 * TIMESTAMPS_PER_SECOND;

        // We offset the count by one as the files generated are (index 1).
        while count < (NUM_ROWS / timestamps_per_minute + 1) {
            let path = expanded_plus_timestamps_dir.join(format!("{prefix}_{count}.json"));
            let f = File::open(path).unwrap();
            let f = BufReader::new(f);
            let lines = f.lines().count();
            if count < NUM_ROWS / timestamps_per_minute {
                assert_eq!(lines as i64, timestamps_per_minute);
            } else {
                // Check the last file is leq timestamps_per_minute.
                assert!(lines as i64 <= timestamps_per_minute)
            }
            count += 1;
        }
        assert_eq!(count, (NUM_ROWS / timestamps_per_minute + 1));
    }

    /// This test is actually validating that the generated minimal_plus_timestamps file is valid.
    /// If you delete the generated file, you can rerun this test first to regenerate it.
    #[test]
    fn test_generated_minimal_plus_timestamps2raw() {
        let mut raw_path = PathBuf::from(RTCM_PATH);
        raw_path.set_extension("minimal_plus_timestamps");

        if !raw_path.exists() {
            create_test_file(&raw_path);
        }

        let minimal_plus_timestamps_path = File::open(raw_path).unwrap();
        assert!(minimal_plus_timestamps_path.metadata().unwrap().len() > 0);

        let reader = BufReader::new(minimal_plus_timestamps_path);
        let rtcm_stream = reader
            .lines()
            .map(|js| {
                let packet: minimal_plus_timestamps::Frame =
                    serde_json::from_str(&js.unwrap()).unwrap();
                packet.rtcm_b64
            })
            .collect::<Vec<Vec<u8>>>();

        let rtcm_stream = rtcm_stream.concat();
        let rtcm_stream = std::io::Cursor::new(&rtcm_stream);
        let mut count = 0;
        let _: Result<(), rtcm::Error> = rtcm::iter_messages(rtcm_stream).try_for_each(|frame| {
            frame.unwrap();
            count += 1;
            Ok(())
        });
        assert_eq!(count, NUM_ROWS);
    }
    #[test]
    fn test_generated_minimal_plus_timestamps_with_seconds_as_strings2raw() {
        let raw_path = PathBuf::from(MINIMAL_PLUS_TIMESTAMPS_PATH_WITH_SECONDS_AS_STRINGS);

        let minimal_plus_timestamps_path = File::open(raw_path).unwrap();

        let reader = BufReader::new(minimal_plus_timestamps_path);
        let rtcm_stream = reader
            .lines()
            .map(|js| {
                let packet: minimal_plus_timestamps::Frame =
                    serde_json::from_str(&js.unwrap()).unwrap();
                packet.rtcm_b64
            })
            .collect::<Vec<Vec<u8>>>();

        let rtcm_stream = rtcm_stream.concat();
        let rtcm_stream = std::io::Cursor::new(&rtcm_stream);
        let mut count = 0;
        let _: Result<(), rtcm::Error> = rtcm::iter_messages(rtcm_stream).try_for_each(|frame| {
            frame.unwrap();
            count += 1;
            Ok(())
        });
        assert!(count > 0);
    }

    fn compare_two_line_delimited_files(left_path: PathBuf, right_path: PathBuf, count: i64) {
        let left_file = BufReader::new(File::open(&left_path).unwrap()).lines();
        let right_file = BufReader::new(File::open(&right_path).unwrap()).lines();
        let mut actual_count: i64 = 0;
        for (left, right) in left_file.zip(right_file) {
            assert_eq!(left.unwrap(), right.unwrap());
            actual_count += 1;
        }
        assert_eq!(count, actual_count);
    }

    fn compare_two_rtcm_files(left_path: PathBuf, right_path: PathBuf, count: i64) {
        let left_file = rtcm::iter_messages(BufReader::new(File::open(&left_path).unwrap()));
        let right_file = rtcm::iter_messages(BufReader::new(File::open(&right_path).unwrap()));
        let mut actual_count: i64 = 0;
        for (left, right) in left_file.zip(right_file) {
            assert_eq!(left.unwrap(), right.unwrap());
            actual_count += 1;
        }
        assert_eq!(count, actual_count);
    }

    #[test]
    fn test_expanded_plus_timestamps2minimal_plus_timestamps() {
        let reader = File::open(EXPANDED_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(MINIMAL_PLUS_TIMESTAMPS_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        minimal_plus_timestamps::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_expanded_plus_timestamps2expanded() {
        let reader = File::open(EXPANDED_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(EXPANDED_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        expanded::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_expanded_plus_timestamps2minimal() {
        let reader = File::open(EXPANDED_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(MINIMAL_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        minimal::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_expanded_plus_timestamps2raw() {
        let reader = File::open(EXPANDED_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(RTCM_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        raw::from_json(reader, writer).unwrap();
        compare_two_rtcm_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_minimal_plus_timestamps2expanded_plus_timestamps() {
        let reader = File::open(MINIMAL_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(EXPANDED_PLUS_TIMESTAMPS_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        expanded_plus_timestamps::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_minimal_plus_timestamps2expanded() {
        let reader = File::open(MINIMAL_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(EXPANDED_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        expanded::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_minimal_plus_timestamps2minimal() {
        let reader = File::open(MINIMAL_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(MINIMAL_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        minimal::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_minimal_plus_timestamps2raw() {
        let reader = File::open(MINIMAL_PLUS_TIMESTAMPS_PATH).unwrap();
        let ref_path = PathBuf::from(RTCM_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        raw::from_json(reader, writer).unwrap();
        compare_two_rtcm_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_expanded2minimal() {
        let reader = File::open(EXPANDED_PATH).unwrap();
        let ref_path = PathBuf::from(MINIMAL_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        minimal::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_expanded2raw() {
        let reader = File::open(EXPANDED_PATH).unwrap();
        let ref_path = PathBuf::from(RTCM_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        raw::from_json(reader, writer).unwrap();
        compare_two_rtcm_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_minimal2expanded() {
        let reader = File::open(MINIMAL_PATH).unwrap();
        let ref_path = PathBuf::from(EXPANDED_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        expanded::from_json(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_minimal2raw() {
        let reader = File::open(MINIMAL_PATH).unwrap();
        let ref_path = PathBuf::from(RTCM_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        raw::from_json(reader, writer).unwrap();
        compare_two_rtcm_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_raw2expanded() {
        let reader = File::open(RTCM_PATH).unwrap();
        let ref_path = PathBuf::from(EXPANDED_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        expanded::from_raw(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }

    #[test]
    fn test_raw2minimal() {
        let reader = File::open(RTCM_PATH).unwrap();
        let ref_path = PathBuf::from(MINIMAL_PATH);
        let temp_path = NamedTempFile::new().unwrap().into_temp_path().to_path_buf();
        let writer = File::create(&temp_path).unwrap();
        minimal::from_raw(reader, writer).unwrap();
        compare_two_line_delimited_files(temp_path, ref_path, NUM_ROWS);
    }
}
