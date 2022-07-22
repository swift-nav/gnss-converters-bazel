use clap::{ArgMatches, Command};

pub fn parse_args() -> ArgMatches {
    Command::new("json2rtcm")
        .author("Swift Navigation <dev@swiftnav.com>")
        .version(env!("VERGEN_GIT_SEMVER"))
        .about("Convert JSON data to RTCM")
        .get_matches()
}
