/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#[cfg(target_os = "windows")]
#[windows_subsystem = "console"]
#[cfg(target_os = "windows")]
#[link(name = "msvcrt")]
extern "C" {}

use std::boxed::Box;
use std::error::Error;
use std::ffi::CString;
use std::fs::File;
use std::io::Cursor;
use std::result::Result;

use libc::{c_char, c_void};
use tempdir::TempDir;
use xz2::write::XzDecoder;

use gnss_converters::*;

#[link(name = "nov2sbp_main", kind = "static")]
#[link(name = "novatel-parser", kind = "static")]
#[link(name = "sbp", kind = "static")]
#[link(name = "swiftnav", kind = "static")]
#[link(name = "proj", kind = "static")]
extern "C" {
    fn nov2sbp_main(
        argc: i32,
        argv: *const *const i8,
        addition_opts_help: *const c_char,
        readfn: unsafe extern "C" fn(*mut u8, usize, *mut c_void) -> i32,
        writefn: unsafe extern "C" fn(*const u8, usize, *mut c_void) -> i32,
        context: *mut c_void,
    ) -> i32;

    fn nov2sbp_proj_context_set_search_paths(path: *const c_char);
}

pub fn populate_proj_lib(proj_lib: &TempDir) -> Result<(), Box<dyn Error>> {
    const EGM96_15_GTX: &str = "egm96_15.gtx";
    const PROJ_DB: &str = "proj.db";
    let proj_lib_path = proj_lib.path();
    let gtx_path = proj_lib_path.join(EGM96_15_GTX);
    let gtx_output = File::create(gtx_path)
        .map_err(|err| format!("failed to create {}, err: {}", EGM96_15_GTX, err))?;
    let proj_db_path = proj_lib_path.join(PROJ_DB);
    let proj_db_output = File::create(proj_db_path)
        .map_err(|err| format!("failed to create {}, err: {}", PROJ_DB, err))?;
    let mut gtx_xz_decoder = XzDecoder::new(gtx_output);
    let mut proj_db_xz_decoder = XzDecoder::new(proj_db_output);
    let mut gtx_xz_bytes = Cursor::new(include_bytes!("egm96_15.gtx.xz"));
    let mut proj_db_xz_bytes = Cursor::new(include_bytes!("proj.db.xz"));
    std::io::copy(&mut gtx_xz_bytes, &mut gtx_xz_decoder)?;
    std::io::copy(&mut proj_db_xz_bytes, &mut proj_db_xz_decoder)?;
    gtx_xz_decoder
        .finish()
        .map_err(|err| format!("failed to decompress {}.gz, err: {}", EGM96_15_GTX, err))?;
    proj_db_xz_decoder
        .finish()
        .map_err(|err| format!("failed to decompress {}.gz, err: {}", PROJ_DB, err))?;
    let proj_lib_path = proj_lib_path.to_string_lossy();
    let proj_lib_path = proj_lib_path.as_bytes();
    let proj_lib_path = CString::new(proj_lib_path)?;
    unsafe { nov2sbp_proj_context_set_search_paths(proj_lib_path.as_ptr()) };
    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    let exit_code = {
        let proj_lib = TempDir::new("nov2sbp")
            .map_err(|err| format!("could not create temporary directory: {}", err))?;
        populate_proj_lib(&proj_lib)?;
        let _stdout_flusher = StdoutFlusher::new();
        let (reader, writer) = fetch_io("nov2sbp");
        let mut context = Context { reader, writer };
        let cargs = CArgs::new();
        let argv = cargs.argv();
        let (argc, argv) = (cargs.len(), argv.as_ptr());
        unsafe {
            nov2sbp_main(
                argc,
                argv,
                ADDITIONAL_OPTS_HELP.as_ptr(),
                readfn,
                writefn,
                &mut context as *mut Context as *mut c_void,
            )
        }
    };
    std::process::exit(exit_code);
}
