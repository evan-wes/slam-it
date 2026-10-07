//! A whole sequence through the edge pipeline: the Rust mirror of `slamlab.orchestration.vo`
//! (its docstring states the run), behind the native program `slamlab_vo` (`src/main.rs`).

use std::path::{Path, PathBuf};

use nalgebra::Vector2;

use slamlab_edge::frames::{self, Camera, DepthMap, Frame};
use slamlab_edge::map::Map;
use slamlab_edge::options::{Mode, Options, Setting, check_options};
use slamlab_edge::pipeline::{FrameResult, Pipeline, State};
use slamlab_eval::runs::{self, JsonObject, JsonValue, Run};
use slamlab_eval::{euroc, png, text, tum};
use slamlab_foundations::camera::Model;
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::image::{self, Image};
use slamlab_foundations::{orb, se3, stereo};

/// The datasets a sequence can come from (`slamlab.orchestration.vo.DATASETS`).
pub const DATASETS: [&str; 2] = ["tum", "euroc"];

/// An RGB-D camera's virtual baseline times its focal length, in pixel-metres: ORB-SLAM's
/// `Camera.bf` for TUM (`RGBD_BF`).
pub const RGBD_BF: f64 = 40.0;

/// The program's command line.
pub const USAGE: &str = "usage: slamlab_vo --dataset tum|euroc --sequence PATH --mode \
                         monocular|stereo|rgbd --out RUN_DIR [--first I] [--count N] \
                         [--option KEY=VALUE ...]";

/// The program's exit code for a command line it cannot read.
pub const EXIT_USAGE: u8 = 2;

/// Its exit code for a failed run: [`exit_code`] adds the error's kind.
pub const EXIT_FAILED: u8 = 3;

/// A run asked for: the program's command line.
#[derive(Debug, Clone, PartialEq)]
pub struct Request {
    /// One of [`DATASETS`].
    pub dataset: String,
    /// The sequence's folder.
    pub sequence: PathBuf,
    /// The sensor mode.
    pub mode: Mode,
    /// The run's folder, written.
    pub out: PathBuf,
    /// The first frame run.
    pub first: i64,
    /// The frames run from it, or all.
    pub count: Option<i64>,
    /// The pipeline's settings.
    pub options: Options,
}

impl Request {
    /// A request, its other fields default: from the first frame, every frame, the default
    /// settings.
    ///
    /// # Arguments
    ///
    /// * `dataset` - One of [`DATASETS`].
    /// * `sequence` - The sequence's folder.
    /// * `mode` - The sensor mode.
    /// * `out` - The run's folder.
    ///
    /// # Returns
    ///
    /// The request.
    #[must_use]
    pub fn new(dataset: &str, sequence: &Path, mode: Mode, out: &Path) -> Self {
        Self {
            dataset: dataset.to_owned(),
            sequence: sequence.to_path_buf(),
            mode,
            out: out.to_path_buf(),
            first: 0,
            count: None,
            options: Options::default(),
        }
    }
}

/// The exit code of a run that failed with an error of a kind: [`EXIT_FAILED`] for
/// `InvalidInput`, then `Degenerate`, then `NotConverged`
/// (`slamlab.orchestration.vo.EXIT_FAILED`).
///
/// # Arguments
///
/// * `kind` - The error's kind.
///
/// # Returns
///
/// The exit code.
#[must_use]
pub const fn exit_code(kind: ErrorKind) -> u8 {
    EXIT_FAILED
        + match kind {
            ErrorKind::InvalidInput => 0,
            ErrorKind::Degenerate => 1,
            ErrorKind::NotConverged => 2,
        }
}

/// Builds the frame at an index with the extraction settings the pipeline asks for: `None`
/// for a frame the run skips.
type Builder = Box<dyn Fn(usize, &orb::Options) -> Result<Option<Frame>>>;

/// A sequence opened for a run.
struct Source {
    /// The camera the pipeline runs with.
    camera: Camera,
    /// The sequence's frames.
    frames: usize,
    /// Builds one.
    frame: Builder,
}

/// Opens a TUM-format folder: its camera, an RGB-D one with the virtual baseline.
///
/// # Arguments
///
/// * `root` - The folder.
/// * `mode` - Monocular or RGB-D.
///
/// # Returns
///
/// The source.
///
/// # Errors
///
/// The reader's.
fn tum_source(root: &Path, mode: Mode) -> Result<Source> {
    let read = tum::read_frames(root, tum::MAX_DIFFERENCE)?;
    let sensor = &read.camera;
    let camera = Camera {
        model: sensor.model,
        intrinsics: sensor.intrinsics.clone(),
        width: sensor.width,
        height: sensor.height,
        baseline: if mode == Mode::Rgbd {
            RGBD_BF / sensor.intrinsics[0]
        } else {
            0.0
        },
    };
    let count = read.images.len();
    let built = camera.clone();
    let frame: Builder = Box::new(move |i, extraction| {
        let image = png::read_gray(&read.images[i])?;
        let timestamp = read.timestamps[i];
        if mode == Mode::Monocular {
            return frames::monocular(&image, timestamp, &built, extraction).map(Some);
        }
        let Some(depth_file) = &read.depths[i] else {
            return Ok(None);
        };
        let depth = tum::read_depth(depth_file, read.camera.depth_scale)?;
        let depth_map = DepthMap {
            rows: depth.rows,
            cols: depth.cols,
            values: depth.values,
        };
        frames::rgbd(&image, &depth_map, timestamp, &built, extraction).map(Some)
    });
    Ok(Source {
        camera,
        frames: count,
        frame,
    })
}

/// A EuRoC camera as the rectification takes it.
///
/// # Arguments
///
/// * `camera` - The camera.
///
/// # Returns
///
/// Its model (radial-tangential) and intrinsics.
fn radtan(camera: &euroc::Camera) -> stereo::Camera<'_> {
    stereo::Camera {
        model: Model::RadTan,
        intrinsics: &camera.intrinsics,
    }
}

/// Reads an image and rectifies it.
///
/// # Arguments
///
/// * `path` - The file.
/// * `coordinates` - Its rectification map.
/// * `camera` - The rectified camera.
///
/// # Returns
///
/// The rectified image.
///
/// # Errors
///
/// The reader's or the remap's.
fn rectified(path: &Path, coordinates: &[Vector2<f64>], camera: &Camera) -> Result<Image> {
    image::remap(
        &png::read_gray(path)?,
        coordinates,
        camera.height,
        camera.width,
    )
}

/// Opens a EuRoC sequence's stereo pairs, rectified: the rectified left camera (pinhole) with
/// the pair's baseline.
///
/// # Arguments
///
/// * `read` - The sequence's frames.
///
/// # Returns
///
/// The source.
///
/// # Errors
///
/// The rectification's.
fn stereo_source(read: euroc::Frames) -> Result<Source> {
    let (left, right) = (&read.left_camera, &read.right_camera);
    let pose = euroc::right_from_left(left, right)?;
    let rectification = stereo::rectify(radtan(left), radtan(right), &pose, left.rows, left.cols)?;
    let maps = [
        (left, &rectification.rotation_left),
        (right, &rectification.rotation_right),
    ]
    .map(|(side, rotation)| {
        stereo::rectification_map(
            radtan(side),
            rotation,
            &rectification.intrinsics,
            left.rows,
            left.cols,
        )
    });
    let [map_left, map_right] = maps;
    let maps = [map_left?, map_right?];
    let camera = Camera {
        model: Model::Pinhole,
        intrinsics: rectification.intrinsics.as_slice().to_vec(),
        width: left.cols,
        height: left.rows,
        baseline: rectification.baseline,
    };
    let count = read.left.len();
    let built = camera.clone();
    let frame: Builder = Box::new(move |i, extraction| {
        let image_left = rectified(&read.left[i], &maps[0], &built)?;
        let image_right = rectified(&read.right[i], &maps[1], &built)?;
        frames::stereo_pair(
            (&image_left, &image_right),
            read.timestamps[i],
            &built,
            extraction,
            &stereo::MatchOptions::default(),
        )
        .map(Some)
    });
    Ok(Source {
        camera,
        frames: count,
        frame,
    })
}

/// Opens a EuRoC sequence: its left camera for monocular, the pairs rectified for stereo.
///
/// # Arguments
///
/// * `root` - The sequence folder.
/// * `mode` - Monocular or stereo.
///
/// # Returns
///
/// The source.
///
/// # Errors
///
/// The reader's.
fn euroc_source(root: &Path, mode: Mode) -> Result<Source> {
    let read = euroc::read_frames(root)?;
    if mode == Mode::Stereo {
        return stereo_source(read);
    }
    let left = &read.left_camera;
    let camera = Camera {
        model: Model::RadTan,
        intrinsics: left.intrinsics.clone(),
        width: left.cols,
        height: left.rows,
        baseline: 0.0,
    };
    let count = read.left.len();
    let built = camera.clone();
    let frame: Builder = Box::new(move |i, extraction| {
        let image = png::read_gray(&read.left[i])?;
        frames::monocular(&image, read.timestamps[i], &built, extraction).map(Some)
    });
    Ok(Source {
        camera,
        frames: count,
        frame,
    })
}

/// The settings as a run records them, by their Python names.
///
/// # Arguments
///
/// * `options` - The settings.
///
/// # Returns
///
/// Each one's name and value.
fn settings(options: &Options) -> JsonObject {
    options
        .settings()
        .into_iter()
        .map(|(name, value)| {
            let value = match value {
                Setting::Integer(integer) => JsonValue::Integer(integer),
                Setting::Real(real) => JsonValue::Number(real),
            };
            (name.to_owned(), value)
        })
        .collect()
}

/// A sequence's name: its folder's, the path resolved.
///
/// # Arguments
///
/// * `sequence` - The folder.
///
/// # Returns
///
/// The name.
fn sequence_name(sequence: &Path) -> String {
    let resolved = std::fs::canonicalize(sequence).unwrap_or_else(|_| sequence.to_path_buf());
    resolved
        .file_name()
        .map_or_else(String::new, |name| name.to_string_lossy().into_owned())
}

/// Records a frame's result: its diagnostics, and its pose if it was tracked.
///
/// # Arguments
///
/// * `out` - The run.
/// * `result` - The result.
///
/// # Errors
///
/// The pose's inversion's.
fn record(out: &mut Run, result: &FrameResult) -> Result<()> {
    out.diagnostics.push(vec![
        ("timestamp".to_owned(), JsonValue::Number(result.timestamp)),
        (
            "state".to_owned(),
            JsonValue::Text(result.state.name().to_owned()),
        ),
        (
            "tracked".to_owned(),
            JsonValue::Integer(i64::try_from(result.tracked).unwrap_or(i64::MAX)),
        ),
        ("keyframe".to_owned(), JsonValue::Bool(result.keyframe)),
        ("seconds".to_owned(), JsonValue::Number(result.seconds)),
    ]);
    if let Some(pose) = &result.pose {
        out.poses.push(se3::inverse(pose)?);
        out.timestamps.push(result.timestamp);
    }
    Ok(())
}

/// Records the map: its keyframes' times and poses, and its points.
///
/// # Arguments
///
/// * `out` - The run.
/// * `m` - The map.
///
/// # Errors
///
/// A pose's inversion's.
fn record_map(out: &mut Run, m: &Map) -> Result<()> {
    out.keyframe_timestamps.clone_from(&m.kf_time);
    out.keyframe_poses = m.kf_pose.iter().map(se3::inverse).collect::<Result<_>>()?;
    out.points.clone_from(&m.pt_xyz);
    Ok(())
}

/// Tracks a sequence's frames, from the first asked for, until the end or the track is lost.
///
/// # Arguments
///
/// * `source` - The sequence.
/// * `pipeline` - The pipeline.
/// * `request` - The request (checked).
/// * `out` - The run, each frame recorded.
///
/// # Errors
///
/// A builder's or the pipeline's.
fn track(source: &Source, pipeline: &mut Pipeline, request: &Request, out: &mut Run) -> Result<()> {
    let first = usize::try_from(request.first).unwrap_or(usize::MAX);
    let end = request.count.map_or(source.frames, |count| {
        source
            .frames
            .min(first.saturating_add(usize::try_from(count).unwrap_or(0)))
    });
    for i in first..end {
        let Some(frame) = (source.frame)(i, &pipeline.extraction())? else {
            continue;
        };
        let result = pipeline.track(&frame)?;
        record(out, &result)?;
        if result.state == State::Lost {
            break;
        }
    }
    Ok(())
}

/// Fails with `InvalidInput`.
///
/// # Arguments
///
/// * `message` - What is wrong.
///
/// # Errors
///
/// Always.
fn invalid<T>(message: String) -> Result<T> {
    fail(ErrorKind::InvalidInput, message)
}

/// Reads a mode's name.
///
/// # Arguments
///
/// * `text` - The name, as `slamlab.edge.options.MODES` spells it.
///
/// # Returns
///
/// The mode.
///
/// # Errors
///
/// `InvalidInput` for another name.
fn mode_named(text: &str) -> Result<Mode> {
    Mode::ALL
        .into_iter()
        .find(|mode| mode.name() == text)
        .map_or_else(
            || {
                invalid(format!(
                    "--mode: expected monocular, stereo or rgbd, got '{text}'"
                ))
            },
            Ok,
        )
}

/// Reads a whole integer.
///
/// # Arguments
///
/// * `flag` - The flag it was given with, for the message.
/// * `text` - The text.
///
/// # Returns
///
/// The integer.
///
/// # Errors
///
/// `InvalidInput` for text that is not one.
fn integer_of(flag: &str, text: &str) -> Result<i64> {
    text::integer(text).map_or_else(
        || invalid(format!("{flag}: expected an integer, got '{text}'")),
        Ok,
    )
}

/// Reads one `--option KEY=VALUE` into the settings.
///
/// # Arguments
///
/// * `text` - `KEY=VALUE`.
/// * `options` - The settings.
///
/// # Errors
///
/// `InvalidInput` for an unknown key or a value of the wrong kind.
fn read_option(text: &str, options: &mut Options) -> Result<()> {
    let Some((key, value)) = text.split_once('=') else {
        return invalid(format!("--option: expected KEY=VALUE, got '{text}'"));
    };
    let Some((_, kind)) = options
        .settings()
        .into_iter()
        .find(|&(name, _)| name == key)
    else {
        return invalid(format!("--option: no setting '{key}'"));
    };
    let setting = match kind {
        Setting::Integer(_) => Setting::Integer(integer_of(key, value)?),
        Setting::Real(_) => match text::number(value) {
            Some(number) if number.is_finite() => Setting::Real(number),
            _ => return invalid(format!("{key}: expected a finite number, got '{value}'")),
        },
    };
    options.set(key, setting)
}

/// Reads one flag's value into the request.
///
/// # Arguments
///
/// * `out` - The request.
/// * `flag` - The flag.
/// * `value` - Its value.
///
/// # Errors
///
/// `InvalidInput` for an unknown flag or a bad value.
fn read_flag(out: &mut Request, flag: &str, value: &str) -> Result<()> {
    match flag {
        "--dataset" => value.clone_into(&mut out.dataset),
        "--sequence" => out.sequence = PathBuf::from(value),
        "--out" => out.out = PathBuf::from(value),
        "--mode" => out.mode = mode_named(value)?,
        "--first" => out.first = integer_of(flag, value)?,
        "--count" => out.count = Some(integer_of(flag, value)?),
        "--option" => read_option(value, &mut out.options)?,
        _ => return invalid(format!("unknown argument '{flag}'")),
    }
    Ok(())
}

/// Reads the command line: [`USAGE`], flags in any order, `--option` repeatable and naming a
/// field of `slamlab.edge.options.Options`.
///
/// # Arguments
///
/// * `arguments` - The arguments after the program's name.
///
/// # Returns
///
/// The request.
///
/// # Errors
///
/// `InvalidInput` for an unknown, repeated or missing flag, a flag without its value, an
/// unknown mode or setting, or a value that is not a number.
pub fn parse(arguments: &[String]) -> Result<Request> {
    let mut out = Request::new("", Path::new(""), Mode::Monocular, Path::new(""));
    let mut seen: Vec<&str> = Vec::new();
    for pair in arguments.chunks(2) {
        let flag = pair[0].as_str();
        let Some(value) = pair.get(1) else {
            return invalid(format!("{flag}: expected a value"));
        };
        if flag != "--option" && seen.contains(&flag) {
            return invalid(format!("{flag}: given twice"));
        }
        seen.push(flag);
        read_flag(&mut out, flag, value)?;
    }
    for required in ["--dataset", "--sequence", "--mode", "--out"] {
        if !seen.contains(&required) {
            return invalid(format!("{required}: missing"));
        }
    }
    Ok(out)
}

/// Checks a request (`slamlab.orchestration.vo.check`).
///
/// # Arguments
///
/// * `request` - The request.
///
/// # Errors
///
/// `InvalidInput` for an unknown dataset, a mode it does not record (TUM has no stereo pairs,
/// EuRoC no depth maps), a negative `first` or `count`, or settings out of range.
pub fn check(request: &Request) -> Result<()> {
    if !DATASETS.contains(&request.dataset.as_str()) {
        return invalid(format!(
            "dataset: expected tum or euroc, got '{}'",
            request.dataset
        ));
    }
    if request.dataset == "tum" && request.mode == Mode::Stereo {
        return invalid("mode: a tum sequence has no stereo pairs".to_owned());
    }
    if request.dataset == "euroc" && request.mode == Mode::Rgbd {
        return invalid("mode: a euroc sequence has no depth maps".to_owned());
    }
    if request.first < 0 || request.count.unwrap_or(0) < 0 {
        return invalid("first, count: must be at least 0".to_owned());
    }
    check_options(&request.options)
}

/// Runs a sequence through the pipeline and writes its run folder
/// (`slamlab.orchestration.vo.run`).
///
/// # Arguments
///
/// * `request` - The request.
///
/// # Returns
///
/// The run written.
///
/// # Errors
///
/// The check's, a reader's, the pipeline's or the writer's.
pub fn run(request: &Request) -> Result<Run> {
    check(request)?;
    let source = if request.dataset == "tum" {
        tum_source(&request.sequence, request.mode)?
    } else {
        euroc_source(&request.sequence, request.mode)?
    };
    let mut pipeline = Pipeline::new(request.mode, source.camera.clone(), request.options)?;
    let mut out = Run {
        dataset: request.dataset.clone(),
        sequence: sequence_name(&request.sequence),
        mode: request.mode.name().to_owned(),
        backend: "rust".to_owned(),
        settings: settings(&request.options),
        ..Run::default()
    };
    track(&source, &mut pipeline, request, &mut out)?;
    if let Some(m) = pipeline.map() {
        record_map(&mut out, m)?;
    }
    runs::write_run(&request.out, &out)?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests of reading the command line, the request's checks, and a run of no frames
    //! on a fixture folder (the fixtures list images without holding them). The whole room is
    //! run, through every backend, by `tests/orchestration`.

    use super::*;

    /// A fixture sequence's folder.
    fn folder(name: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../tests/fixtures/eval/sequences")
            .join(name)
    }

    /// A command line with its required flags, and more.
    fn command(more: &[&str]) -> Vec<String> {
        let sequence = folder("tum_camera");
        let mut out = vec![
            "--dataset",
            "tum",
            "--sequence",
            sequence.to_str().unwrap(),
            "--mode",
            "rgbd",
            "--out",
            "run",
        ];
        out.extend_from_slice(more);
        out.into_iter().map(str::to_owned).collect()
    }

    /// Every flag is read; the options named are set, the rest default.
    #[test]
    fn parses_a_command_line() {
        let request = parse(&command(&[
            "--first",
            "3",
            "--count",
            "10",
            "--option",
            "window=7",
            "--option",
            "close_depth=35.5",
        ]))
        .unwrap();
        assert_eq!(
            (
                request.dataset.as_str(),
                request.mode,
                request.first,
                request.count
            ),
            ("tum", Mode::Rgbd, 3, Some(10))
        );
        assert_eq!(
            (request.options.window, request.options.n_features),
            (7, 1000)
        );
        assert!((request.options.close_depth - 35.5).abs() < f64::EPSILON);
        let plain = parse(&command(&[])).unwrap();
        assert_eq!((plain.first, plain.count), (0, None));
    }

    /// A bad flag, value or setting, a repeated flag, or a missing one is refused.
    #[test]
    fn refuses_bad_command_lines() {
        for more in [
            &["--mode", "lidar"][..],
            &["--first", "x"],
            &["--count"],
            &["--option", "windows=7"],
            &["--option", "window=7.5"],
            &["--option", "close_depth=inf"],
            &["--option", "window"],
            &["--dataset", "euroc"],
            &["--verbose", "1"],
        ] {
            assert_eq!(
                parse(&command(more)).unwrap_err().kind,
                ErrorKind::InvalidInput,
                "{more:?}"
            );
        }
        let short = ["--dataset".to_owned(), "tum".to_owned()];
        assert_eq!(parse(&short).unwrap_err().kind, ErrorKind::InvalidInput);
    }

    /// Unknown datasets, modes a dataset does not record, negative counts and settings out of
    /// range are refused.
    #[test]
    fn checks_requests() {
        let good = Request::new(
            "tum",
            &folder("tum_camera"),
            Mode::Monocular,
            Path::new("run"),
        );
        check(&good).unwrap();
        let mut bads = vec![
            Request {
                dataset: "kitti".to_owned(),
                ..good.clone()
            },
            Request {
                mode: Mode::Stereo,
                ..good.clone()
            },
            Request::new("euroc", &folder("euroc"), Mode::Rgbd, Path::new("run")),
            Request {
                count: Some(-1),
                ..good.clone()
            },
        ];
        bads.push(Request {
            options: Options {
                window: 0,
                ..Options::default()
            },
            ..good
        });
        for bad in bads {
            assert_eq!(check(&bad).unwrap_err().kind, ErrorKind::InvalidInput);
        }
    }

    /// A run of no frames records the sequence and every setting; a missing image or sequence
    /// fails the run.
    #[test]
    fn runs_the_fixture_folders() {
        let out = std::env::temp_dir().join("slamlab_test_vo_rust");
        let mut request = Request::new("tum", &folder("tum_camera"), Mode::Rgbd, &out);
        request.count = Some(0);
        let empty = run(&request).unwrap();
        assert!(empty.timestamps.is_empty());
        assert_eq!(
            (
                empty.sequence.as_str(),
                empty.backend.as_str(),
                empty.settings.len()
            ),
            ("tum_camera", "rust", Options::default().settings().len())
        );
        let text = std::fs::read_to_string(out.join("run.json")).unwrap();
        assert!(text.contains("\"window\": 10") && out.join("trajectory.txt").exists());
        request.count = Some(1);
        assert_eq!(run(&request).unwrap_err().kind, ErrorKind::InvalidInput);
        let mut stereo = Request::new("euroc", &folder("euroc"), Mode::Stereo, &out);
        stereo.count = Some(1);
        assert_eq!(run(&stereo).unwrap_err().kind, ErrorKind::InvalidInput);
        stereo.sequence = folder("no_such_sequence");
        assert_eq!(run(&stereo).unwrap_err().kind, ErrorKind::InvalidInput);
        std::fs::remove_dir_all(&out).unwrap();
    }
}
