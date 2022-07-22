use clap::{ArgMatches, Command};

pub fn parse_args() -> ArgMatches {
    Command::new("rtcm32json")
        .author("Swift Navigation <dev@swiftnav.com>")
        .version(env!("VERGEN_GIT_SEMVER"))
        .about("Convert RTCM data to JSON")
        .get_matches()
}
