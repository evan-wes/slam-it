//! `slam_it_vo`: a whole sequence through the edge pipeline, its run folder written
//! (`slam_it_orchestration::vo`). `slam_it.orchestration.vo.run` runs it for the `rust`
//! backend.
//!
//! Exit codes: 0 for a run written; `EXIT_USAGE` (2) for a command line it cannot read, with
//! the usage; `exit_code` of the error's kind (from 3) for a failed run. Messages go to stderr,
//! a one-line summary to stdout.

use std::path::Path;
use std::process::ExitCode;

use slam_it_eval::{budgets, runs};
use slam_it_orchestration::vo;

fn main() -> ExitCode {
    let arguments: Vec<String> = std::env::args_os()
        .skip(1)
        .map(|argument| argument.to_string_lossy().into_owned())
        .collect();
    let request = match vo::parse(&arguments) {
        Ok(request) => request,
        Err(error) => {
            eprintln!("slam_it_vo: {}", error.message);
            eprintln!("{}", vo::USAGE);
            return ExitCode::from(vo::EXIT_USAGE);
        }
    };
    let written = vo::run(&request).and_then(|mut done| {
        // What the program records of its own process, before it writes the run.
        done.peak_memory = budgets::peak_memory(Path::new(budgets::STATUS));
        done.cpus = budgets::allowed_cpus(Path::new(budgets::STATUS));
        done.memory_cap =
            budgets::memory_cap(Path::new(budgets::CGROUP), Path::new(budgets::CGROUP_ROOT));
        runs::write_run(&request.out, &done)?;
        Ok(done)
    });
    match written {
        Ok(done) => {
            println!(
                "slam_it_vo: {} of {} frames tracked, {} keyframes, {} points -> {}",
                done.timestamps.len(),
                done.diagnostics.len(),
                done.keyframe_timestamps.len(),
                done.points.len(),
                request.out.display()
            );
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("slam_it_vo: {}", error.message);
            ExitCode::from(vo::exit_code(error.kind))
        }
    }
}
