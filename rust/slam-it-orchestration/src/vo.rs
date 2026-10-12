//! A whole sequence through the edge pipeline: the Rust mirror of `slam_it.orchestration.vo`
//! (its docstring states the run), behind the native program `slam_it_vo` (`src/main.rs`).

use std::path::{Path, PathBuf};
use std::time::Instant;

use nalgebra::Vector2;

use slam_it_edge::adjust::Solve;
use slam_it_edge::frames::{self, Camera, DepthMap, Frame};
use slam_it_edge::options::{Mode, Options, Setting, check_options};
use slam_it_edge::pipeline::{Fates, FrameResult, FrameSummary, Lifetimes, Pipeline, State};
use slam_it_eval::runs::{FeatureWriter, JsonObject, JsonValue, Run};
use slam_it_eval::{euroc, png, text, tum};
use slam_it_foundations::camera::Model;
use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::image::{self, Image};
use slam_it_foundations::{orb, se3, stereo};

/// The datasets a sequence can come from (`slam_it.orchestration.vo.DATASETS`).
pub const DATASETS: [&str; 2] = ["tum", "euroc"];

/// The program's command line.
pub const USAGE: &str = "usage: slam_it_vo --dataset tum|euroc --sequence PATH --mode \
                         monocular|stereo|rgbd --out RUN_DIR [--first I] [--count N] \
                         [--record-features] [--option KEY=VALUE ...]";

/// The one flag without a value: record each frame's keypoints, with their fates.
const RECORD_FEATURES: &str = "--record-features";

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
    /// Whether the run records each frame's keypoints, with their fates.
    pub record_features: bool,
}

impl Request {
    /// A request, its other fields default: from the first frame, every frame, the default
    /// settings, no feature records.
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
            record_features: false,
        }
    }
}

/// The exit code of a run that failed with an error of a kind: [`EXIT_FAILED`] for
/// `InvalidInput`, then `Degenerate`, then `NotConverged`
/// (`slam_it.orchestration.vo.EXIT_FAILED`).
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

/// A frame's images as a camera delivers them, decoded, and a stereo pair rectified.
enum Images {
    /// A monocular frame's image.
    Monocular(Image),
    /// A stereo pair: the left image, then the right.
    Stereo(Image, Image),
    /// An RGB-D frame's image and its depth map, in metres.
    Rgbd(Image, DepthMap),
}

/// A frame's images and its time (`slam_it.orchestration.vo._Delivered`).
struct Delivered {
    /// The frame's time.
    timestamp: f64,
    /// Its images.
    images: Images,
}

/// Delivers the images at an index: `None` for a frame the run skips.
type Reader = Box<dyn Fn(usize) -> Result<Option<Delivered>>>;

/// A sequence opened for a run.
struct Source {
    /// The camera the pipeline runs with.
    camera: Camera,
    /// The sequence's frames.
    frames: usize,
    /// Reads one.
    read: Reader,
}

/// Builds a frame from its images, for the mode they were delivered for.
///
/// # Arguments
///
/// * `delivered` - The images.
/// * `camera` - The camera.
/// * `extraction` - The extraction settings the pipeline asks for.
/// * `matching` - The stereo matcher's settings.
///
/// # Returns
///
/// The frame.
///
/// # Errors
///
/// The frame builder's.
fn build(
    delivered: &Delivered,
    camera: &Camera,
    extraction: &orb::Options,
    matching: &stereo::MatchOptions,
) -> Result<Frame> {
    let timestamp = delivered.timestamp;
    match &delivered.images {
        Images::Monocular(image) => frames::monocular(image, timestamp, camera, extraction),
        Images::Stereo(left, right) => {
            frames::stereo_pair((left, right), timestamp, camera, extraction, matching)
        }
        Images::Rgbd(image, depth) => frames::rgbd(image, depth, timestamp, camera, extraction),
    }
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
    let baseline = match (mode, sensor.bf) {
        (Mode::Rgbd, Some(bf)) => bf / sensor.intrinsics[0],
        (Mode::Rgbd, None) => {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "{}: an RGB-D run needs the depth sensor's bf: add a 'bf' line to {}",
                    root.display(),
                    tum::CAMERA_FILE
                ),
            );
        }
        _ => 0.0,
    };
    let camera = Camera {
        model: sensor.model,
        intrinsics: sensor.intrinsics.clone(),
        width: sensor.width,
        height: sensor.height,
        baseline,
    };
    let count = read.images.len();
    let deliver: Reader = Box::new(move |i| {
        let image = png::read_gray(&read.images[i])?;
        let timestamp = read.timestamps[i];
        if mode == Mode::Monocular {
            return Ok(Some(Delivered {
                timestamp,
                images: Images::Monocular(image),
            }));
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
        Ok(Some(Delivered {
            timestamp,
            images: Images::Rgbd(image, depth_map),
        }))
    });
    Ok(Source {
        camera,
        frames: count,
        read: deliver,
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
    let rectifying = camera.clone();
    let deliver: Reader = Box::new(move |i| {
        let image_left = rectified(&read.left[i], &maps[0], &rectifying)?;
        let image_right = rectified(&read.right[i], &maps[1], &rectifying)?;
        Ok(Some(Delivered {
            timestamp: read.timestamps[i],
            images: Images::Stereo(image_left, image_right),
        }))
    });
    Ok(Source {
        camera,
        frames: count,
        read: deliver,
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
    let deliver: Reader = Box::new(move |i| {
        Ok(Some(Delivered {
            timestamp: read.timestamps[i],
            images: Images::Monocular(png::read_gray(&read.left[i])?),
        }))
    });
    Ok(Source {
        camera,
        frames: count,
        read: deliver,
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

/// A count as JSON.
///
/// # Arguments
///
/// * `count` - The count.
///
/// # Returns
///
/// Its value.
fn count(count: usize) -> JsonValue {
    JsonValue::Integer(i64::try_from(count).unwrap_or(i64::MAX))
}

/// A count as JSON, or null if there is none.
///
/// # Arguments
///
/// * `value` - The count, if any.
///
/// # Returns
///
/// Its value.
fn count_or_null(value: Option<usize>) -> JsonValue {
    value.map_or(JsonValue::Null, count)
}

/// A name as JSON.
///
/// # Arguments
///
/// * `name` - The name.
///
/// # Returns
///
/// Its value.
fn name_json(name: &str) -> JsonValue {
    JsonValue::Text(name.to_owned())
}

/// An object as JSON, from its keys and values.
///
/// # Arguments
///
/// * `items` - The keys and values, in order.
///
/// # Returns
///
/// The object.
fn object<const N: usize>(items: [(&str, JsonValue); N]) -> JsonValue {
    JsonValue::Object(
        items
            .into_iter()
            .map(|(key, value)| (key.to_owned(), value))
            .collect(),
    )
}

/// Bundle-adjustment solves as JSON (`slam_it.edge.adjust.Solve.as_dict`, each).
///
/// # Arguments
///
/// * `solves` - The solves.
///
/// # Returns
///
/// One object per solve.
fn solves_json(solves: &[Solve]) -> JsonValue {
    JsonValue::Array(
        solves
            .iter()
            .map(|solve| {
                object([
                    ("initial_cost", JsonValue::Number(solve.initial_cost)),
                    ("final_cost", JsonValue::Number(solve.final_cost)),
                    ("iterations", count(solve.iterations)),
                    ("termination", name_json(solve.termination.name())),
                ])
            })
            .collect(),
    )
}

/// How a frame's stages went, as a run records it
/// (`slam_it.edge.pipeline.FrameSummary.as_dict`): each stage an object, or null.
///
/// # Arguments
///
/// * `summary` - The summary.
///
/// # Returns
///
/// Its keys and values, in order.
fn summary_json(summary: &FrameSummary) -> JsonObject {
    let placed_by = summary
        .placed_by
        .map_or(JsonValue::Null, |placement| name_json(placement.name()));
    let init = summary.init.as_ref().map_or(JsonValue::Null, |s| {
        object([
            ("matches", count_or_null(s.matches)),
            ("outcome", name_json(s.outcome.name())),
            ("ba", solves_json(&s.solves)),
        ])
    });
    let motion = summary.motion.map_or(JsonValue::Null, |s| {
        object([
            ("matches", count(s.matches)),
            ("widened", JsonValue::Bool(s.widened)),
            ("inliers", count_or_null(s.inliers)),
        ])
    });
    let pnp = summary.pnp.map_or(JsonValue::Null, |s| {
        let termination = if s.degenerate {
            name_json("degenerate")
        } else {
            s.termination
                .map_or(JsonValue::Null, |termination| name_json(termination.name()))
        };
        object([
            ("candidates", count(s.candidates)),
            ("matches", count_or_null(s.matches)),
            ("iterations", count_or_null(s.iterations)),
            ("termination", termination),
            ("inliers", count_or_null(s.inliers)),
            ("refined", count_or_null(s.refined)),
        ])
    });
    let local_map = summary.local_map.map_or(JsonValue::Null, |s| {
        object([
            ("in_view", count(s.in_view)),
            ("matches", count(s.matches)),
            ("inliers", count(s.inliers)),
        ])
    });
    let mapping = summary.mapping.as_ref().map_or(JsonValue::Null, |s| {
        object([
            ("from_depth", count(s.from_depth)),
            ("culled_points", count(s.culled_points)),
            ("triangulated", count(s.triangulated)),
            ("fused", count(s.fused)),
            ("ba", solves_json(&s.solves)),
            ("outliers", count(s.outliers)),
            ("weak_points", count(s.weak_points)),
            ("culled_keyframes", count(s.culled_keyframes)),
        ])
    });
    [
        ("placed_by", placed_by),
        ("init", init),
        ("motion", motion),
        ("pnp", pnp),
        ("local_map", local_map),
        ("mapping", mapping),
    ]
    .into_iter()
    .map(|(key, value)| (key.to_owned(), value))
    .collect()
}

/// Records a frame's result: its diagnostics, and its pose if it was tracked.
///
/// # Arguments
///
/// * `out` - The run.
/// * `result` - The result.
/// * `read_seconds` - The time delivering its images took, in seconds.
/// * `build_seconds` - The time building the frame took, in seconds.
///
/// # Errors
///
/// The pose's inversion's.
fn record(
    out: &mut Run,
    result: &FrameResult,
    read_seconds: f64,
    build_seconds: f64,
) -> Result<()> {
    let mut diagnostic = vec![
        ("timestamp".to_owned(), JsonValue::Number(result.timestamp)),
        ("state".to_owned(), name_json(result.state.name())),
        ("tracked".to_owned(), count(result.tracked)),
        ("keyframe".to_owned(), JsonValue::Bool(result.keyframe)),
        ("read_seconds".to_owned(), JsonValue::Number(read_seconds)),
        ("build_seconds".to_owned(), JsonValue::Number(build_seconds)),
        (
            "track_seconds".to_owned(),
            JsonValue::Number(result.track_seconds),
        ),
        (
            "map_seconds".to_owned(),
            JsonValue::Number(result.map_seconds),
        ),
    ];
    diagnostic.extend(summary_json(&result.summary));
    out.diagnostics.push(diagnostic);
    if let Some(pose) = &result.pose {
        out.poses.push(se3::inverse(pose)?);
        out.timestamps.push(result.timestamp);
    }
    Ok(())
}

/// Records every keyframe and point the run made, with their lives.
///
/// # Arguments
///
/// * `out` - The run.
/// * `lives` - The pipeline's lifetimes.
///
/// # Errors
///
/// A pose's inversion's.
fn record_lives(out: &mut Run, lives: Lifetimes) -> Result<()> {
    out.keyframe_poses = lives
        .keyframe_poses
        .iter()
        .map(se3::inverse)
        .collect::<Result<_>>()?;
    out.keyframe_timestamps = lives.keyframe_times;
    out.points = lives.points;
    out.point_lives = lives.point_lives;
    out.keyframe_lives = lives.keyframe_lives;
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
/// * `features` - The writer of each frame's keypoints, if the run records them.
///
/// # Errors
///
/// A reader's, a frame builder's, the pipeline's or the feature writer's.
fn track(
    source: &Source,
    pipeline: &mut Pipeline,
    request: &Request,
    out: &mut Run,
    mut features: Option<&mut FeatureWriter>,
) -> Result<()> {
    let first = usize::try_from(request.first).unwrap_or(usize::MAX);
    let end = request.count.map_or(source.frames, |count| {
        source
            .frames
            .min(first.saturating_add(usize::try_from(count).unwrap_or(0)))
    });
    let matching = request.options.stereo_matching();
    for i in first..end {
        let start = Instant::now();
        let delivered = (source.read)(i)?;
        let read = start.elapsed().as_secs_f64();
        let Some(delivered) = delivered else {
            continue;
        };
        let frame = build(
            &delivered,
            &source.camera,
            &pipeline.extraction(),
            &matching,
        )?;
        let built = start.elapsed().as_secs_f64();
        let result = pipeline.track(&frame)?;
        record(out, &result, read, built - read)?;
        if let Some(writer) = features.as_deref_mut() {
            write_features(writer, &frame, &result.fates)?;
        }
        if result.state == State::Lost {
            break;
        }
    }
    Ok(())
}

/// Appends a frame's keypoints, with their fates, to the run's feature files.
///
/// # Arguments
///
/// * `features` - The writer.
/// * `frame` - The frame.
/// * `fates` - What became of each of its keypoints.
///
/// # Errors
///
/// The writer's.
fn write_features(features: &mut FeatureWriter, frame: &Frame, fates: &Fates) -> Result<()> {
    let keypoints = &frame.features.keypoints;
    let xy: Vec<[f64; 2]> = keypoints.iter().map(|k| k.xy).collect();
    let level: Vec<i64> = keypoints
        .iter()
        .map(|k| i64::try_from(k.level).unwrap_or(i64::MAX))
        .collect();
    let fate: Vec<i64> = fates.fate.iter().map(|&f| f as i64).collect();
    features.write(&xy, &level, &fate, &fates.point)
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
/// * `text` - The name, as `slam_it.edge.options.MODES` spells it.
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

/// Reads the command line: [`USAGE`], flags in any order, each with its value but
/// `--record-features`, `--option` repeatable and naming a field of
/// `slam_it.edge.options.Options`.
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
    let mut rest = arguments;
    while let Some((flag, after)) = rest.split_first() {
        let flag = flag.as_str();
        if flag != "--option" && seen.contains(&flag) {
            return invalid(format!("{flag}: given twice"));
        }
        seen.push(flag);
        if flag == RECORD_FEATURES {
            out.record_features = true;
            rest = after;
            continue;
        }
        let Some((value, after)) = after.split_first() else {
            return invalid(format!("{flag}: expected a value"));
        };
        read_flag(&mut out, flag, value)?;
        rest = after;
    }
    for required in ["--dataset", "--sequence", "--mode", "--out"] {
        if !seen.contains(&required) {
            return invalid(format!("{required}: missing"));
        }
    }
    Ok(out)
}

/// Checks a request (`slam_it.orchestration.vo.check`).
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

/// Runs a sequence through the pipeline: the run, not yet written, as Python's program builds
/// it before writing (`slam_it.orchestration.vo.main`). The program records its own process in
/// it (`slam_it_eval::budgets`) and writes it to `request.out`.
///
/// # Arguments
///
/// * `request` - The request.
///
/// # Returns
///
/// The run.
///
/// # Errors
///
/// The check's, a reader's or the pipeline's.
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
        record_features: request.record_features,
        ..Run::default()
    };
    let mut features = if request.record_features {
        Some(FeatureWriter::open(&request.out)?)
    } else {
        None
    };
    track(&source, &mut pipeline, request, &mut out, features.as_mut())?;
    record_lives(&mut out, pipeline.lifetimes())?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests of reading the command line, the request's checks, and a run of no frames
    //! on a fixture folder (the fixtures list images without holding them). The whole room is
    //! run, through every backend, by `tests/orchestration`.

    use super::*;

    /// A scratch folder, removed when the test ends, failed or not.
    struct Scratch(std::path::PathBuf);

    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

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

    /// Each error kind exits with its own code, from `EXIT_FAILED` on (Python's `_ERRORS`).
    #[test]
    fn each_error_kind_has_an_exit_code() {
        assert_eq!(
            [
                exit_code(ErrorKind::InvalidInput),
                exit_code(ErrorKind::Degenerate),
                exit_code(ErrorKind::NotConverged),
            ],
            [3, 4, 5]
        );
    }

    /// Every flag is read; the options named are set, the rest default.
    #[test]
    fn parses_a_command_line() {
        let request = parse(&command(&[
            "--first",
            "3",
            "--record-features",
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
        assert!(request.record_features);
        let plain = parse(&command(&[])).unwrap();
        assert_eq!(
            (plain.first, plain.count, plain.record_features),
            (0, None, false)
        );
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
            &["--option", "close_depth=+-5"],
            &["--option", "window"],
            &["--dataset", "euroc"],
            &["--verbose", "1"],
            &["--record-features", "--record-features"],
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
        let out = std::env::temp_dir().join(format!("slam_it_test_vo_rust_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let _guard = Scratch(out.clone());
        let mut request = Request::new("tum", &folder("tum_camera"), Mode::Rgbd, &out);
        request.count = Some(0);
        let empty = run(&request).unwrap();
        assert_eq!(empty.timestamps, Vec::<f64>::new());
        assert!(
            !out.join("run.json").exists(),
            "the run is the program's to write"
        );
        slam_it_eval::runs::write_run(&out, &empty).unwrap();
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
    }
}
