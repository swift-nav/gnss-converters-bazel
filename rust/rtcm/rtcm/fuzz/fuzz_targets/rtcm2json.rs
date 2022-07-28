#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| { for _ in rtcm::iter_messages(data) {} });
