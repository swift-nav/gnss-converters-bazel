use anyhow::Result;
use clap::{Parser, ValueEnum};
use rtcm::Frame;
use serde::{Deserialize, Serialize};
use serde_with::{base64::Base64, serde_as};
use std::{
    fs::File,
    io::{BufReader, BufWriter, Read, Write},
    path::PathBuf,
};

mod cli;

struct TimeSplitWriter {
    path: PathBuf,
    wtr: Option<BufWriter<File>>,
    split_interval: SplitInterval,
    current_split: usize,
    last_split_seconds: i64,
}
impl TimeSplitWriter {
    fn new(path: PathBuf, split_interval: SplitInterval) -> Self {
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

    fn check_writer(&mut self, seconds: i64) -> Result<()> {
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
    fn serialize(&mut self, frame: &RtjsFrame, seconds: i64) -> Result<()> {
        self.check_writer(seconds)?;
        let mut wtr = self.wtr.as_mut().expect("unable to open writer");
        serde_json::to_writer(&mut wtr, frame)?;
        wtr.write_all(b"\n")?;
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

/// Possible errors while decoding an Mrtjs/Rtjs frames
#[derive(thiserror::Error, Debug)]
pub enum RtjsError {
    #[error(transparent)]
    RtcmError(#[from] rtcm::Error),
    #[error(transparent)]
    AnyhowError(#[from] anyhow::Error),
    #[error("decoding rtcm incomplete")]
    IncompleteDecodingError,
}

#[serde_as]
#[derive(Serialize, Deserialize)]
struct MrtjsFrame {
    seconds: i64,
    nanoseconds: i32,
    #[serde_as(as = "Base64")]
    rtcm_b64: Vec<u8>,
}

#[serde_as]
#[derive(Serialize, Deserialize)]
struct RtjsFrame {
    seconds: i64,
    nanoseconds: i32,
    #[serde_as(as = "Base64")]
    rtcm_b64: Vec<u8>,
    rtcm: Option<Frame>,
}

impl TryFrom<MrtjsFrame> for RtjsFrame {
    type Error = RtjsError;

    fn try_from(value: MrtjsFrame) -> Result<Self, Self::Error> {
        let MrtjsFrame {
            seconds,
            nanoseconds,
            rtcm_b64,
        } = value;

        let frame = match rtcm::Frame::from_slice(&rtcm_b64[..]) {
            Some(result) => Some(result?),
            None => return Err(RtjsError::IncompleteDecodingError),
        };
        Ok(RtjsFrame {
            seconds,
            nanoseconds,
            rtcm_b64,
            rtcm: frame,
        })
    }
}

fn mrtjs2rtjs<R: Read>(reader: R, mut writer: TimeSplitWriter) -> Result<()> {
    let reader = BufReader::new(reader);
    let stream = serde_json::Deserializer::from_reader(reader).into_iter::<MrtjsFrame>();
    for msg in stream {
        let mrtjs = msg?;
        let seconds = mrtjs.seconds;
        let rtjs = RtjsFrame::try_from(mrtjs)?;

        writer.serialize(&rtjs, seconds)?;
    }
    Ok(())
}

fn main() -> Result<()> {
    let args = cli::Cli::parse();
    let input = args.input()?;
    let split_interval = args.split_interval.unwrap_or(SplitInterval::None);
    let output = TimeSplitWriter::new(args.file_out, split_interval);
    let reader = BufReader::new(input);
    mrtjs2rtjs(reader, output)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use deku::DekuContainerWrite;
    use std::fs;
    use std::io::{BufRead, BufReader, Write};
    use std::path::Path;

    use super::*;

    const SSR_CORRECTIONS_SIGNED_PATH: &str = "../../tests/data/piksi-5Hz_signed.rtcm";
    const TIMESTAMPS_PER_SECOND: i64 = 1;
    const SSR_CORRECTIONS_SIGNED_NUM_ROWS: i64 = 220;

    fn create_test_file<P: AsRef<Path>>(mrtjs_path: P) {
        let mut mrtjs_file = File::create(&mrtjs_path).unwrap();
        let rtcm_file = File::open(SSR_CORRECTIONS_SIGNED_PATH).unwrap();
        let mut count = 0;
        rtcm::iter_messages(rtcm_file)
            .try_for_each(|frame| {
                let rtcm_b64 = frame.unwrap().to_bytes().unwrap();

                let packet = MrtjsFrame {
                    seconds: count / TIMESTAMPS_PER_SECOND,
                    nanoseconds: (count % TIMESTAMPS_PER_SECOND) as i32,
                    rtcm_b64,
                };
                count += 1;

                let json = serde_json::to_string(&packet).unwrap();
                writeln!(&mut mrtjs_file, "{}", json)
            })
            .unwrap();
    }

    #[test]
    fn test_mrtjs2rtjs() {
        let mut mrtjs_path = PathBuf::from(SSR_CORRECTIONS_SIGNED_PATH);
        mrtjs_path.set_extension("mrtjs");
        let mrtjs_file = File::open(mrtjs_path).unwrap();
        let reader = BufReader::new(mrtjs_file);
        let rtjs_file = tempfile::NamedTempFile::new().unwrap();
        let rtjs_file = rtjs_file.into_temp_path().to_path_buf();
        let writer = TimeSplitWriter::new(rtjs_file.clone(), SplitInterval::None);
        mrtjs2rtjs(reader, writer).unwrap();
        let f = fs::File::open(rtjs_file).unwrap();
        let f = BufReader::new(f);
        let lines = f.lines().count();
        assert_eq!(lines as i64, SSR_CORRECTIONS_SIGNED_NUM_ROWS);
    }

    #[test]
    fn test_mrtjs2rtjs_with_split() {
        let mut mrtjs_path = PathBuf::from(SSR_CORRECTIONS_SIGNED_PATH);
        mrtjs_path.set_extension("mrtjs");
        let mrtjs_file = File::open(mrtjs_path).unwrap();
        let reader = BufReader::new(mrtjs_file);

        let rtjs_dir = tempfile::tempdir().unwrap();
        let rtjs_dir = rtjs_dir.into_path();

        let prefix = "asdf";
        let rtjs_file = rtjs_dir.join(format!("{prefix}.json"));
        let writer = TimeSplitWriter::new(rtjs_file, SplitInterval::Minute);
        mrtjs2rtjs(reader, writer).unwrap();

        let mut count = 1;
        let timestamps_per_minute = SplitInterval::Minute as i64 * TIMESTAMPS_PER_SECOND;

        // We offset the count by one as the files generated are (index 1).
        while count < (SSR_CORRECTIONS_SIGNED_NUM_ROWS / timestamps_per_minute + 1) {
            let path = rtjs_dir.join(format!("{prefix}_{count}.json"));
            let f = fs::File::open(path).unwrap();
            let f = BufReader::new(f);
            let lines = f.lines().count();
            if count < SSR_CORRECTIONS_SIGNED_NUM_ROWS / timestamps_per_minute {
                assert_eq!(lines as i64, timestamps_per_minute);
            } else {
                // Check the last file is leq timestamps_per_minute.
                assert!(lines as i64 <= timestamps_per_minute)
            }
            count += 1;
        }
        assert_eq!(
            count,
            (SSR_CORRECTIONS_SIGNED_NUM_ROWS / timestamps_per_minute + 1)
        );
    }

    /// This test is actually validating that the generated mrtjs file is valid.
    /// If you delete the generated file, you can rerun this test first to regenerate it.
    #[test]
    fn test_decode_generated_mrtjs() {
        let mut mrtjs_path = PathBuf::from(SSR_CORRECTIONS_SIGNED_PATH);
        mrtjs_path.set_extension("mrtjs");

        if !mrtjs_path.exists() {
            create_test_file(&mrtjs_path);
        }

        let mrtjs_file = File::open(mrtjs_path).unwrap();
        assert!(mrtjs_file.metadata().unwrap().len() > 0);

        let reader = BufReader::new(mrtjs_file);
        let rtcm_stream = reader
            .lines()
            .map(|js| {
                let packet: MrtjsFrame = serde_json::from_str(&js.unwrap()).unwrap();
                let rtcm_ = packet.rtcm_b64;
                rtcm_
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
}
