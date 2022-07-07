use clap::{ArgMatches, Command};

pub fn parse_args() -> ArgMatches {
    Command::new("rtcm2json")
        .author("Swift Navigation <dev@swiftnav.com>")
        .version(env!("CARGO_PKG_VERSION"))
        .about("Convert RTCM data to JSON")
        .get_matches()
}
