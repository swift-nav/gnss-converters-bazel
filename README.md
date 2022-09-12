gnss-converters-private
=======================

[![Build Status](https://jenkins-webhooks.ci.swift-nav.com/buildStatus/icon?job=swift-nav/gnss-converters-private/master&subject=Jenkins&style=flat)](https://jenkins.ci.swift-nav.com/job/swift-nav/job/gnss-converters-private/job/master/) [![CI](https://github.com/swift-nav/gnss-converters-private/actions/workflows/ci.yaml/badge.svg)](https://github.com/swift-nav/gnss-converters-private/actions/workflows/ci.yaml) [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=swift-nav_gnss-converters-private&metric=alert_status&token=b5bc9e8c800e8c371ae68e0e8c2471eb73d389c2)](https://sonarcloud.io/dashboard?id=swift-nav_gnss-converters-private)

## Quick install

Pre-built binaries for various platforms are availale on the [releases page][5].
To quickly build and install the latest version, you can use Rust `cargo` tool.

First, [install rust][6]:

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install build dependencies, and a nightly toolchain for Rust:

```
sudo apt install build-essential cmake clang sqlite3
```

Then build and install with `cargo`:

```
git clone --recursive git@github.com:swift-nav/gnss-converters-private.git
cd gnss-converters-private
cargo install --path ./rust
cargo install --path ./rust/rtcm/rtcm32json
cargo install --path ./rust/rtcm/json2rtcm3
```

To build and install the `nov2sbp` tool as well, also run:

```
cargo install --path ./rust --bin nov2sbp --features nov2sbp
```

The following tools will be installed in `~/.cargo/bin` (which should be in the path):

- sbp2rtcm
- rtcm3tosbp
- ubx2sbp
- ubx2json
- ixcom2sbp
- nov2sbp
- rtcm32json
- json2rtcm3
- mrtjs2rtjs

## Build locally

This repository is a bit of a chimera; its products are the
gnss-converters library which provides funtionality for converting
RTCM to SBP, a tool written in C for converting RTCM to SBP, and a
variety of haskell tools that will convert between RTCM, SBP, and
json.

To build the C tool and the library, just follow the usual steps:

```
git submodule update --init --recursive
cd c/
mkdir build
cd build/
cmake ..
make -j8
```

Here is an example of how to run the C tool.  This should (eventually)
result in some colorful json on your terminal (the ntripping tool
can be found [on GitHub](https://github.com/swift-nav/ntripping)):

```
ntripping --url http://bmookerji:bmookerji@tiburon.geo.berkeley.edu:2101/MONB_RTCM3 | ./rtcm3tosbp | sbp2json | jq .
```

[5]: https://github.com/swift-nav/gnss-converters-private/releases
[6]: https://www.rust-lang.org/tools/install
