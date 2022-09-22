use anyhow::Result;
use clap::Parser;

use rtcm::{conversions::json2rtcm, helpers::common::GenericCli};

#[derive(Debug, Parser)]
#[clap(
    name = "json2rtcm3",
    about="Convert JSON data to RTCM",
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
    json2rtcm::json2rtcm(input, output)
}
