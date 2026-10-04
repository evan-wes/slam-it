//! Reading a Bundle Adjustment in the Large (BAL) problem into a [`Problem`], natively, and
//! the BAL demo's solver settings; shared by the `bal` example and the benchmarks.
//!
//! The same conversion as the Python reader (`slamlab.eval.readers.read_bal`): BAL's camera
//! frame flips `y` and `z` relative to this package's, its cameras are world-to-camera
//! `(ω, t, f, k1, k2)`, and its observations are centred pixels with `y` up. The result has
//! one radial intrinsics group per camera, `f, k1, k2` free and the principal point fixed.
//! Plain text only: decompress a downloaded `.bz2` first (`bunzip2 -k`).

use std::path::Path;

use nalgebra::{Matrix3, Vector2, Vector3};
use slamlab_foundations::ba::{self, Gauge, Intrinsics, Problem};
use slamlab_foundations::camera::Model;
use slamlab_foundations::{nls, se3, so3};

/// BAL's camera frame to this package's: flip `y` and `z`.
fn flip() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, -1.0))
}

/// Reads a BAL problem from a plain-text file.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The problem, or a message naming what is wrong with the file.
pub fn read_bal(path: &Path) -> Result<Problem, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut words = text.split_whitespace();
    let mut count = || -> Result<usize, String> {
        words
            .next()
            .and_then(|w| w.parse().ok())
            .ok_or_else(|| "not a BAL header".to_string())
    };
    let (n_cams, n_points, n_obs) = (count()?, count()?, count()?);
    let numbers: Vec<f64> = text
        .split_whitespace()
        .skip(3)
        .map(str::parse)
        .collect::<Result<_, _>>()
        .map_err(|e| format!("not a BAL problem: {e}"))?;
    let expected = 4 * n_obs + 9 * n_cams + 3 * n_points;
    if numbers.len() != expected {
        return Err(format!(
            "expected {expected} numbers, found {}",
            numbers.len()
        ));
    }
    let (obs, rest) = numbers.split_at(4 * n_obs);
    let (cams, points) = rest.split_at(9 * n_cams);
    let index = |v: f64| format!("{v:.0}").parse::<i64>().map_err(|e| e.to_string());
    let mut cam_idx = Vec::with_capacity(n_obs);
    let mut pt_idx = Vec::with_capacity(n_obs);
    let mut observations = Vec::with_capacity(n_obs);
    for o in obs.as_chunks::<4>().0 {
        cam_idx.push(index(o[0])?);
        pt_idx.push(index(o[1])?);
        observations.push(Vector2::new(o[2], -o[3]));
    }
    let mut poses = Vec::with_capacity(n_cams);
    let mut values = Vec::with_capacity(5 * n_cams);
    for c in cams.as_chunks::<9>().0 {
        let rotation = flip() * so3::exp(&Vector3::new(c[0], c[1], c[2]));
        let translation = flip() * Vector3::new(c[3], c[4], c[5]);
        let t_cam_world = se3::from_parts(&rotation, &translation);
        poses.push(se3::inverse(&t_cam_world).map_err(|e| e.to_string())?);
        values.extend([c[6], 0.0, 0.0, c[7], c[8]]);
    }
    let fixed_intrinsics = (0..n_cams)
        .flat_map(|_| [false, true, true, false, false])
        .collect();
    let points = points
        .as_chunks::<3>()
        .0
        .iter()
        .map(|&p| Vector3::from(p))
        .collect();
    let intrinsics = Intrinsics { values, params: 5 };
    let mut problem = Problem::with_defaults(
        Model::Radial,
        intrinsics,
        poses,
        points,
        observations,
        cam_idx,
        pt_idx,
    );
    problem.intrinsics_idx = (0..n_cams)
        .map(|c| i64::try_from(c).map_err(|e| e.to_string()))
        .collect::<Result<_, _>>()?;
    problem.fixed_intrinsics = fixed_intrinsics;
    Ok(problem)
}

/// The BAL demo's settings (`scripts/demo_bal.py`): Levenberg-Marquardt, at most 100
/// iterations, and no gauge removal, as BAL's own reference solves have none.
///
/// # Returns
///
/// The options.
pub fn demo_options() -> ba::Options {
    ba::Options {
        solver: nls::Options {
            max_iterations: 100,
            ..nls::Options::default()
        },
        gauge: Gauge::None,
        ..ba::Options::default()
    }
}

#[cfg(test)]
mod tests {
    //! The conversion against `tests/fixtures/eval/bal.json`, which the Python reader
    //! generates and the C++ benchmark's reader is tested against too: the model and every
    //! array field, defaults included.
    #![allow(
        clippy::cast_precision_loss,
        reason = "small indices compared as numbers"
    )]

    use std::path::PathBuf;

    use nalgebra::Matrix4;
    use serde_json::Value;

    use super::*;

    /// The fixture directory.
    fn fixtures() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/eval")
    }

    /// The values of a nested JSON array, row-major: numbers as they are, booleans as 1/0,
    /// null as NaN.
    fn flat(value: &Value) -> Vec<f64> {
        match value {
            Value::Array(items) => items.iter().flat_map(flat).collect(),
            Value::Bool(b) => vec![f64::from(u8::from(*b))],
            Value::Null => vec![f64::NAN],
            other => vec![other.as_f64().expect("a number")],
        }
    }

    /// Poses as row-major values.
    fn pose_values(poses: &[Matrix4<f64>]) -> Vec<f64> {
        poses
            .iter()
            .flat_map(|p| p.transpose().iter().copied().collect::<Vec<_>>())
            .collect()
    }

    /// Booleans as 1/0.
    fn bools(values: &[bool]) -> Vec<f64> {
        values.iter().map(|&b| f64::from(u8::from(b))).collect()
    }

    /// Indices as numbers.
    fn ints(values: &[i64]) -> Vec<f64> {
        values.iter().map(|&i| i as f64).collect()
    }

    /// Every array field of a problem, by the Python field's name, row-major.
    fn fields(problem: &Problem) -> Vec<(&'static str, Vec<f64>)> {
        let rows2 = |v: &[nalgebra::Vector2<f64>]| v.iter().flat_map(|x| [x[0], x[1]]).collect();
        let rows3 = |v: &[Vector3<f64>]| v.iter().flat_map(|x| [x[0], x[1], x[2]]).collect();
        vec![
            ("intrinsics", problem.intrinsics.values.clone()),
            ("poses", pose_values(&problem.poses)),
            ("points", rows3(&problem.points)),
            ("observations", rows2(&problem.observations)),
            ("cam_idx", ints(&problem.cam_idx)),
            ("pt_idx", ints(&problem.pt_idx)),
            ("intrinsics_idx", ints(&problem.intrinsics_idx)),
            ("weights", problem.weights.clone()),
            ("fixed_poses", bools(&problem.fixed_poses)),
            ("fixed_intrinsics", bools(&problem.fixed_intrinsics)),
            ("fixed_points", bools(&problem.fixed_points)),
            ("depth_prior", problem.depth_prior.clone()),
            ("depth_prior_weights", problem.depth_prior_weights.clone()),
            ("depth_affine", rows2(&problem.depth_affine)),
            ("fixed_depth_affine", bools(&problem.fixed_depth_affine)),
            ("pose_prior", pose_values(&problem.pose_prior)),
            ("pose_prior_weights", rows2(&problem.pose_prior_weights)),
            ("point_anchor", ints(&problem.point_anchor)),
        ]
    }

    /// The reader gives the problem the Python reader does, every field.
    #[test]
    fn matches_the_python_reader() {
        let text = std::fs::read_to_string(fixtures().join("bal.json")).unwrap();
        let fixture: Value = serde_json::from_str(&text).unwrap();
        let expected = &fixture["expected"];
        let atol = fixture["tolerance"]["atol"].as_f64().unwrap();
        let problem = read_bal(&fixtures().join(fixture["input"].as_str().unwrap())).unwrap();
        assert_eq!(problem.model.name(), expected["model"].as_str().unwrap());
        let arrays = expected["arrays"].as_object().unwrap();
        let ours = fields(&problem);
        assert_eq!(ours.len(), arrays.len(), "every field is compared");
        for (name, got) in ours {
            let want = flat(&arrays[name]);
            assert_eq!(got.len(), want.len(), "{name}");
            let same = |(g, w): (&f64, &f64)| (g.is_nan() && w.is_nan()) || (g - w).abs() <= atol;
            assert!(got.iter().zip(&want).all(same), "{name}");
        }
    }
}
