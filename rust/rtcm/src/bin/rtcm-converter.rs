use anyhow::{bail, Result};
use clap::Parser;
use rtcm::{
    conversions::json::{
        expanded, expanded_plus_timestamps, minimal, minimal_plus_timestamps, raw,
    },
    helpers::common::GenericCli,
};

#[derive(Debug, Clone, PartialEq, Eq, clap::ValueEnum)]
enum ConversionFormat {
    /// Stream Data:
    /// <rtcm binary stream>
    Rtcm,
    /// Newline delimited JSON data:
    /// {
    ///   "rtcm_b64":"<rtcm message encoded in base64>"
    /// }
    Minimal,
    /// Newline delimited JSON data:
    /// {
    ///   "seconds":<timestamp seconds>,
    ///   "nanoseconds":<timestamp nanoseconds>,
    ///   "rtcm_b64":"<rtcm message encoded in base64>"
    /// }
    MinimalPlusTimestamps,
    /// Newline delimited JSON data:
    /// {
    ///   "seconds":<timestamp seconds>,
    ///   "nanoseconds":<timestamp nanoseconds>,
    ///   "rtcm_b64":"<rtcm message encoded in base64>",
    ///   "rtcm":{<decoded rtcm frame json>}
    /// }
    ExpandedPlusTimestamps,
    /// Newline delimited JSON data:
    /// {<decoded rtcm frame json>}
    Expanded,
}

#[derive(Debug, Parser)]
#[clap(
    name="rtcm-converter",
    author="Swift Navigation <dev@swiftnav.com>",
    version=env!("VERGEN_SEMVER_LIGHTWEIGHT"),
    about="Convert to and from rtcm data with various formats.",
)]
pub struct Cli {
    #[clap(
        value_enum,
        long_help = "
Conversion Formats:

rtcm [stream]:
    <rtcm data>

minimal [delimited json]:
    { 
        \"rtcm_b64\": \"<rtcm message encoded in base64>\"
    }

minimal-plus-timestamps [newline delimited JSON]:
    {
        \"seconds\": <timestamp seconds>,
        \"nanoseconds\": <timestamp nanoseconds>,
        \"rtcm_b64\": \"<rtcm message encoded in base64>\"
    }

expanded-plus-timestamps [delimited json]:
    {
        \"seconds\": <timestamp seconds>,
        \"nanoseconds\": <timestamp nanoseconds>,
        \"rtcm_b64\": \"<rtcm message encoded in base64>\",
        \"rtcm\": {<decoded rtcm frame json>}
    }

expanded [delimited json]:
    {<decoded rtcm frame json>}

"
    )]
    input_format: ConversionFormat,
    /// See input-format for usage.
    #[clap(value_enum)]
    output_format: ConversionFormat,
    #[clap(long, value_enum)]
    split_interval: Option<expanded_plus_timestamps::SplitInterval>,
    #[clap(flatten)]
    cli: GenericCli,
}
impl Cli {
    pub fn validate_conversion(&self) -> Result<()> {
        if self.input_format == self.output_format {
            bail!("Input and output formats are the same.")
        }
        if matches!(
            self.output_format,
            ConversionFormat::ExpandedPlusTimestamps | ConversionFormat::MinimalPlusTimestamps
        ) && !matches!(
            self.input_format,
            ConversionFormat::ExpandedPlusTimestamps | ConversionFormat::MinimalPlusTimestamps
        ) {
            bail!(
                "Can only convert expanded_plus_timestamps -> minimal_plus_timestamps\n  
                or minimal_plus_timestamps -> expanded_plus_timestamps"
            );
        }
        if !matches!(self.output_format, ConversionFormat::ExpandedPlusTimestamps)
            && self.split_interval.is_some()
        {
            bail!("Currently split-interval can only be used with out-format: expanded_plus_timestamps");
        }
        Ok(())
    }
    pub fn run() -> Result<()> {
        let mut args = Self::parse();
        args.validate_conversion()?;
        let input = args.cli.input()?;
        let output = args.cli.output()?;
        match args.output_format {
            ConversionFormat::Rtcm => {
                raw::from_json(input, output)?;
            }
            ConversionFormat::Minimal => {
                minimal::from_json(input, output)?;
            }
            ConversionFormat::Expanded => {
                expanded::from_json(input, output)?;
            }
            ConversionFormat::MinimalPlusTimestamps => {
                minimal_plus_timestamps::from_json(input, output)?;
            }
            ConversionFormat::ExpandedPlusTimestamps => {
                if let Some(path) = args.cli.file_out {
                    let wtr =
                        expanded_plus_timestamps::TimeSplitWriter::new(path, args.split_interval);
                    expanded_plus_timestamps::from_json_with_splits(input, wtr)?;
                } else {
                    expanded_plus_timestamps::from_json(input, output)?;
                }
            }
        }
        Ok(())
    }
}

fn main() -> Result<()> {
    Cli::run()
}
