#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| { for _ in rtcm::json::iter_messages(data) {} });
