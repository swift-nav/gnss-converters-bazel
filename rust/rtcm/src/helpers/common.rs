use anyhow::Result;
use clap::Parser;
use std::path::PathBuf;

#[derive(Debug, Parser)]
#[clap(
    author="Swift Navigation <dev@swiftnav.com>",
    version=env!("VERGEN_SEMVER_LIGHTWEIGHT"),
)]
pub struct GenericCli {
    /// Pass in the path to a file or read from stdin.
    pub generic_in: Option<PathBuf>,
    /// Pass in the path to a file or write to stdout.
    pub generic_out: Option<PathBuf>,
    #[clap(skip)]
    pub file_in: Option<PathBuf>,
    #[clap(skip)]
    pub file_out: Option<PathBuf>,
}
impl GenericCli {
    pub fn input(&mut self) -> Result<Box<dyn std::io::Read + 'static>> {
        match &self.generic_in {
            Some(path) => {
                self.file_in = Some(path.clone());
                Ok(Box::new(std::fs::File::open(&path)?))
            }
            None => Ok(Box::new(std::io::stdin())),
        }
    }

    pub fn output(&mut self) -> Result<Box<dyn std::io::Write + 'static>> {
        match &self.generic_out {
            Some(path) => {
                self.file_out = Some(path.clone());
                Ok(Box::new(std::fs::File::create(&path)?))
            }
            None => Ok(Box::new(std::io::stdout())),
        }
    }
}
