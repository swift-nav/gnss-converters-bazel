## Development

Use `debug_deku` feature to print additional info during decoding e.g.:  
`cargo test msm7_full --features debug_deku -- --nocapture`

## Fuzz testing

In order to run fuzz testing you need nightly compiler and cargo-fuzzy:

`rustup toolchain install nightly`

`cargo install cargo-fuzz`

Then run the following tests:

`cargo +nightly fuzz run rtcm2json --fuzz-dir rtcm/fuzz/`  
`cargo +nightly fuzz run --features=json json2rtcm --fuzz-dir rtcm/fuzz/`
