use anyhow::Result;
use std::path::PathBuf;

use crate::SplitInterval;

#[derive(clap::Parser, Debug)]
#[clap(
    name="mrtjs2rtjs",
    author="Swift Navigation <dev@swiftnav.com>",
    version=env!("VERGEN_GIT_SEMVER"),
    about="Convert RTCM data to RTCM JSON w/ TIMESTAMPS"
)]
pub struct Cli {
    #[clap(long = "split-interval", value_enum)]
    pub split_interval: Option<SplitInterval>,
    file_in: Option<PathBuf>,
    #[clap(long = "file-out")]
    pub file_out: PathBuf,
}
impl Cli {
    pub fn input(&self) -> Result<Box<dyn std::io::Read + 'static>> {
        match &self.file_in {
            Some(path) => Ok(Box::new(std::fs::File::open(&path)?)),
            None => Ok(Box::new(std::io::stdin())),
        }
    }
}
