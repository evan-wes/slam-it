//! Builds the bridge to GTSAM (`cxx/graphs_bridge.cpp`, `src/graphs.rs`'s `ffi`) and links
//! GTSAM's static libraries.
//!
//! GTSAM is SLAM It!'s own build (`scripts/setup_slam_it.sh`: 4.3.0, static, with
//! `EIGEN_NO_CPUID`), installed into a prefix: `SLAM_IT_GTSAM_PREFIX` if set, else the
//! active conda environment's (`CONDA_PREFIX`). The bridge compiles with the C++ compiler `CXX`
//! names, with the cores' flags (`EIGEN_NO_CPUID`, `-ffp-contract=off`).

use std::env;
use std::path::PathBuf;

/// GTSAM's static libraries, each before those it depends on.
const LIBRARIES: [&str; 3] = ["gtsam", "metis-gtsam", "cephes-gtsam"];

fn main() {
    println!("cargo:rerun-if-env-changed=SLAM_IT_GTSAM_PREFIX");
    println!("cargo:rerun-if-env-changed=CONDA_PREFIX");
    println!("cargo:rerun-if-changed=cxx/graphs_bridge.cpp");
    println!("cargo:rerun-if-changed=cxx/graphs_bridge.hpp");
    let Some(prefix) = env::var_os("SLAM_IT_GTSAM_PREFIX")
        .or_else(|| env::var_os("CONDA_PREFIX"))
        .map(PathBuf::from)
    else {
        println!(
            "cargo:warning=GTSAM not found: set SLAM_IT_GTSAM_PREFIX or activate the conda env"
        );
        std::process::exit(1);
    };
    let mut bridge = cxx_build::bridge("src/graphs.rs");
    bridge
        .file("cxx/graphs_bridge.cpp")
        .include(prefix.join("include"))
        .include(prefix.join("include").join("eigen3"));
    // Eigen's headers: GTSAM's prefix, or the conda env's when GTSAM is installed elsewhere.
    if let Some(conda) = env::var_os("CONDA_PREFIX") {
        bridge.include(PathBuf::from(conda).join("include").join("eigen3"));
    }
    bridge
        .std("c++20")
        .define("EIGEN_NO_CPUID", None)
        .flag("-ffp-contract=off")
        .compile("slam_it_graphs_bridge");
    println!(
        "cargo:rustc-link-search=native={}",
        prefix.join("lib").display()
    );
    for library in LIBRARIES {
        println!("cargo:rustc-link-lib=static={library}");
    }
}
