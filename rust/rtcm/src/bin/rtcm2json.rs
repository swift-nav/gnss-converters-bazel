use anyhow::Result;
use clap::Parser;

use rtcm::{conversions::rtcm2json, helpers::common::GenericCli};

#[derive(Debug, Parser)]
#[clap(
    name = "rtcm32json",
    about="Convert RTCM data to JSON",
    author="Swift Navigation <dev@swiftnav.com>",
    version=env!("VERGEN_SEMVER_LIGHTWEIGHT"),
)]
pub struct Cli {
    #[clap(flatten)]
    cli: GenericCli,
}

fn main() -> Result<()> {
    let mut args = Cli::parse();
    let input = args.cli.input()?;
    let output = args.cli.output()?;
    rtcm2json::rtcm2json(input, output)
}
