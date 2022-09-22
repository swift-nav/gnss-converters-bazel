use anyhow::Result;
use if_chain::if_chain;
use std::io;

use crate as rtcm;

pub fn rtcm2json<R: io::Read, W: io::Write>(inp: R, mut out: W) -> Result<()> {
    let result: Result<(), rtcm::Error> = rtcm::iter_messages(inp).try_for_each(|frame| {
        let frame = match frame {
            Ok(frame) => Ok(Some(frame)),
            Err(err) => match err {
                rtcm::Error::ParseError(_) | rtcm::Error::CrcError(_) => writeln!(out, "{:?}", err)
                    .map(|_| None)
                    .map_err(|e| e.into()),
                _ => Err(err),
            },
        };
        if let Some(frame) = frame? {
            let json = serde_json::to_string(&frame)?;
            writeln!(out, "{}", json).map_err(|e| e.into())
        } else {
            Ok(())
        }
    });
    if_chain! {
        if let Err(rtcm::Error::IoError(ref err)) = result;
        if err.kind() == io::ErrorKind::BrokenPipe;
        then {
            Ok(())
        } else {
            result.map_err(|e| anyhow::anyhow!(e))
        }
    }
}
