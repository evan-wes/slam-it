//! `slamlab_vo`: a whole sequence through the edge pipeline, its run folder written
//! (`slamlab_orchestration::vo`). `slamlab.orchestration.vo.run` runs it for the `rust`
//! backend.
//!
//! Exit codes: 0 for a run written; `EXIT_USAGE` (2) for a command line it cannot read, with
//! the usage; `exit_code` of the error's kind (from 3) for a failed run. Messages go to stderr,
//! a one-line summary to stdout.

use std::process::ExitCode;

use slamlab_orchestration::vo;

fn main() -> ExitCode {
    let arguments: Vec<String> = std::env::args_os()
        .skip(1)
        .map(|argument| argument.to_string_lossy().into_owned())
        .collect();
    let request = match vo::parse(&arguments) {
        Ok(request) => request,
        Err(error) => {
            eprintln!("slamlab_vo: {}", error.message);
            eprintln!("{}", vo::USAGE);
            return ExitCode::from(vo::EXIT_USAGE);
        }
    };
    match vo::run(&request) {
        Ok(written) => {
            println!(
                "slamlab_vo: {} of {} frames tracked, {} keyframes, {} points -> {}",
                written.timestamps.len(),
                written.diagnostics.len(),
                written.keyframe_timestamps.len(),
                written.points.len(),
                request.out.display()
            );
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("slamlab_vo: {}", error.message);
            ExitCode::from(vo::exit_code(error.kind))
        }
    }
}
