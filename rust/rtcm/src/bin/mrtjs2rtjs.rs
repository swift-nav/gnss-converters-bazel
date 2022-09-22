use anyhow::Result;
use clap::Parser;

use rtcm::{conversions::json::expanded_plus_timestamps, helpers::common::GenericCli};

#[derive(Debug, Parser)]
#[clap(
    name = "mrtjs2rtjs",
    about="Convert minimal_plus_timestamps rtcm json data to expanded_plus_timestamps rtcm json data.",
    author="Swift Navigation <dev@swiftnav.com>",
    version=env!("VERGEN_SEMVER_LIGHTWEIGHT"),
)]
pub struct Cli {
    #[clap(flatten)]
    cli: GenericCli,
    #[clap(long, value_enum)]
    split_interval: Option<expanded_plus_timestamps::SplitInterval>,
}

fn main() -> Result<()> {
    let mut args = Cli::parse();
    let input = args.cli.input()?;
    let output = args.cli.output()?;
    if let Some(path) = args.cli.file_out {
        let wtr = expanded_plus_timestamps::TimeSplitWriter::new(path, args.split_interval);
        expanded_plus_timestamps::from_json_with_splits(input, wtr)?;
    } else {
        expanded_plus_timestamps::from_json(input, output)?;
    }
    Ok(())
}
