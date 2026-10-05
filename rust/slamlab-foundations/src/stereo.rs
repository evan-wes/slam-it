//! Rectified stereo: the Rust core's mirror of `slamlab.foundations.stereo`.
//!
//! Bouguet's rectification of a calibrated pair, and the maps that resample each original
//! image into the rectified view (the Python module's docstring defines both).

use nalgebra::{Matrix3, Matrix4, Vector2, Vector3, Vector4};

use crate::camera::{self, Model};
use crate::error::{ErrorKind, Result, fail};
use crate::image::MAX_SIDE;
use crate::{se3, so3};

/// Camera centres closer than this count as coinciding.
pub const MIN_BASELINE: f64 = 1e-9;
/// Where a map sends a pixel whose ray points behind the camera: outside every image.
pub const OUTSIDE: f64 = -2.0;

/// A rectified stereo pair.
#[derive(Debug, Clone, PartialEq)]
pub struct Rectification {
    /// `R_rect_left`: turns left-camera coordinates into the rectified frame.
    pub rotation_left: Matrix3<f64>,
    /// `R_rect_right`: turns right-camera coordinates into the rectified right frame.
    pub rotation_right: Matrix3<f64>,
    /// Both rectified cameras' pinhole intrinsics `[f, f, cx, cy]`.
    pub intrinsics: Vector4<f64>,
    /// The distance between the camera centres.
    pub baseline: f64,
}

/// One calibrated camera: its model and intrinsics.
#[derive(Debug, Clone, Copy)]
pub struct Camera<'a> {
    /// The model.
    pub model: Model,
    /// Its intrinsics.
    pub intrinsics: &'a [f64],
}

/// A camera's smallest focal length.
///
/// # Arguments
///
/// * `camera` - The camera, its intrinsics checked.
///
/// # Returns
///
/// `min(fx, fy)`, or `f` for the radial model.
fn smallest_focal(camera: Camera<'_>) -> f64 {
    let k = camera.intrinsics;
    if camera.model == Model::Radial {
        k[0]
    } else {
        k[0].min(k[1])
    }
}

/// Checks an image shape: each side from 1 to [`MAX_SIDE`].
///
/// # Arguments
///
/// * `rows` - The rows.
/// * `cols` - The columns.
///
/// # Errors
///
/// `InvalidInput` if a side is out of range.
fn check_shape(rows: usize, cols: usize) -> Result<()> {
    if (1..=MAX_SIDE).contains(&rows) && (1..=MAX_SIDE).contains(&cols) {
        Ok(())
    } else {
        fail(
            ErrorKind::InvalidInput,
            format!("shape: expected (rows, cols), each from 1 to {MAX_SIDE}"),
        )
    }
}

/// Rectifies a calibrated pair (Bouguet's algorithm).
///
/// # Arguments
///
/// * `left` - The left camera.
/// * `right` - The right camera.
/// * `t_right_left` - The rigid transform from left-camera to right-camera coordinates.
/// * `rows` - The images' rows.
/// * `cols` - The images' columns.
///
/// # Returns
///
/// The rectification.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics, a non-rigid transform, a bad shape or a baseline closer
/// to vertical; `Degenerate` if the centres coincide.
#[allow(
    clippy::cast_precision_loss,
    reason = "image sides are at most 2^20, exact in f64"
)]
pub fn rectify(
    left: Camera<'_>,
    right: Camera<'_>,
    t_right_left: &Matrix4<f64>,
    rows: usize,
    cols: usize,
) -> Result<Rectification> {
    camera::check_intrinsics(left.model, left.intrinsics)?;
    camera::check_intrinsics(right.model, right.intrinsics)?;
    se3::check_transform(t_right_left)?;
    check_shape(rows, cols)?;
    let rotation: Matrix3<f64> = t_right_left.fixed_view::<3, 3>(0, 0).into();
    let translation: Vector3<f64> = t_right_left.fixed_view::<3, 1>(0, 3).into();
    let baseline = translation.norm();
    if baseline <= MIN_BASELINE {
        return fail(
            ErrorKind::Degenerate,
            "T_right_left: the camera centres coincide",
        );
    }
    // Each camera turns half the way, then both turn the baseline onto the x axis.
    let half = so3::exp(&(-0.5 * so3::log(&rotation)?));
    let t = half * translation;
    if t.x.abs() <= t.y.abs() {
        return fail(
            ErrorKind::InvalidInput,
            "T_right_left: the baseline is closer to vertical; only side-by-side pairs rectify",
        );
    }
    let target = Vector3::new(1.0_f64.copysign(t.x), 0.0, 0.0);
    let mut axis = t.cross(&target);
    let norm = axis.norm();
    if norm > 0.0 {
        axis *= (t.x.abs() / t.norm()).min(1.0).acos() / norm;
    }
    let onto_x = so3::exp(&axis);
    let focal = smallest_focal(left).min(smallest_focal(right));
    Ok(Rectification {
        rotation_left: onto_x * half.transpose(),
        rotation_right: onto_x * half,
        intrinsics: Vector4::new(
            focal,
            focal,
            (cols - 1) as f64 / 2.0,
            (rows - 1) as f64 / 2.0,
        ),
        baseline,
    })
}

/// Where each rectified pixel samples one original image.
///
/// # Arguments
///
/// * `camera` - The original camera.
/// * `rotation` - That side's rectifying rotation.
/// * `rectified` - The rectified pinhole intrinsics `[f, f, cx, cy]`.
/// * `rows` - The images' rows.
/// * `cols` - The images' columns.
///
/// # Returns
///
/// `(x, y)` per rectified pixel, row by row, for `image::remap`; [`OUTSIDE`] where the ray
/// points behind the camera.
///
/// # Errors
///
/// `InvalidInput` for bad intrinsics, a rotation that is not one, or a bad shape.
#[allow(
    clippy::cast_precision_loss,
    reason = "pixel indices are at most 2^20, exact in f64"
)]
pub fn rectification_map(
    camera: Camera<'_>,
    rotation: &Matrix3<f64>,
    rectified: &Vector4<f64>,
    rows: usize,
    cols: usize,
) -> Result<Vec<Vector2<f64>>> {
    camera::check_intrinsics(camera.model, camera.intrinsics)?;
    camera::check_intrinsics(Model::Pinhole, rectified.as_slice())?;
    so3::check_rotation(rotation)?;
    check_shape(rows, cols)?;
    let (f, cx, cy) = (rectified[0], rectified[2], rectified[3]);
    let r = rotation;
    let mut out = Vec::with_capacity(rows * cols);
    for v in 0..rows {
        for u in 0..cols {
            let x = (u as f64 - cx) / f;
            let y = (v as f64 - cy) / f;
            // R^T (x, y, 1), each product and sum in the Python reference's order.
            let ray = Vector3::new(
                r[(0, 0)] * x + r[(1, 0)] * y + r[(2, 0)],
                r[(0, 1)] * x + r[(1, 1)] * y + r[(2, 1)],
                r[(0, 2)] * x + r[(1, 2)] * y + r[(2, 2)],
            );
            out.push(if ray.z > camera::MIN_DEPTH {
                camera::project(camera.model, camera.intrinsics, &ray)?
            } else {
                Vector2::new(OUTSIDE, OUTSIDE)
            });
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/stereo.json`, which
    //! the Python and C++ tests share, and the core's own input checks.

    use super::*;
    use nalgebra::DMatrix;
    use serde_json::Value;

    /// Reads the shared fixture's functions.
    fn functions() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/stereo.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        let fixture: Value = serde_json::from_str(&text).expect("the fixture is valid JSON");
        fixture["functions"].clone()
    }

    /// Reads a list of floats.
    fn floats(value: &Value) -> Vec<f64> {
        value
            .as_array()
            .expect("a list")
            .iter()
            .map(|v| v.as_f64().expect("a number"))
            .collect()
    }

    /// Reads a row-major matrix from a list of rows.
    fn matrix(value: &Value) -> DMatrix<f64> {
        let rows: Vec<Vec<f64>> = value.as_array().expect("rows").iter().map(floats).collect();
        DMatrix::from_fn(rows.len(), rows[0].len(), |r, c| rows[r][c])
    }

    /// Reads a model from its name.
    fn model(value: &Value) -> Model {
        match value.as_str().expect("a name") {
            "radtan" => Model::RadTan,
            "radial" => Model::Radial,
            _ => Model::Pinhole,
        }
    }

    /// Reads a non-negative integer.
    fn count(value: &Value) -> usize {
        usize::try_from(value.as_u64().expect("a count")).expect("fits a usize")
    }

    /// The largest absolute difference between two equal-sized slices.
    fn worst(a: &[f64], b: &[f64]) -> f64 {
        assert_eq!(a.len(), b.len());
        a.iter()
            .zip(b)
            .map(|(x, y)| (x - y).abs())
            .fold(0.0, f64::max)
    }

    /// `rectify` matches the fixture within its tolerance.
    #[test]
    fn rectify_matches_the_fixture() {
        let spec = &functions()["rectify"];
        let tolerance = spec["tolerance"]["atol"].as_f64().expect("a tolerance");
        for case in spec["cases"].as_array().expect("cases") {
            let input = &case["input"];
            let (left_k, right_k) = (floats(&input[1]), floats(&input[3]));
            let t = matrix(&input[4]);
            let got = rectify(
                Camera {
                    model: model(&input[0]),
                    intrinsics: &left_k,
                },
                Camera {
                    model: model(&input[2]),
                    intrinsics: &right_k,
                },
                &Matrix4::from_fn(|r, c| t[(r, c)]),
                count(&input[5][0]),
                count(&input[5][1]),
            )
            .unwrap();
            let e = &case["expected"];
            assert!(
                worst(
                    got.rotation_left.transpose().as_slice(),
                    matrix(&e["rotation_left"]).transpose().as_slice()
                ) <= tolerance
            );
            assert!(
                worst(
                    got.rotation_right.transpose().as_slice(),
                    matrix(&e["rotation_right"]).transpose().as_slice()
                ) <= tolerance
            );
            assert!(worst(got.intrinsics.as_slice(), &floats(&e["intrinsics"])) <= tolerance);
            assert!((got.baseline - e["baseline"].as_f64().unwrap()).abs() <= tolerance);
        }
    }

    /// `rectification_map` matches the fixture within its tolerance.
    #[test]
    fn rectification_map_matches_the_fixture() {
        let maps = &functions()["rectification_map"];
        let tolerance = maps["tolerance"]["atol"].as_f64().expect("a tolerance");
        for case in maps["cases"].as_array().expect("cases") {
            let fields = &case["input"][2];
            let (rows, cols) = (count(&fields["shape"][0]), count(&fields["shape"][1]));
            let pinhole = floats(&fields["intrinsics"]);
            for (side, key) in [(0, "rotation_left"), (1, "rotation_right")] {
                let camera = &case["input"][side];
                let k = floats(&camera[1]);
                let r = matrix(&fields[key]);
                let got = rectification_map(
                    Camera {
                        model: model(&camera[0]),
                        intrinsics: &k,
                    },
                    &Matrix3::from_fn(|i, j| r[(i, j)]),
                    &Vector4::from_row_slice(&pinhole),
                    rows,
                    cols,
                )
                .unwrap();
                let expected: Vec<f64> = case["expected"][side]
                    .as_array()
                    .expect("rows")
                    .iter()
                    .flat_map(|row| row.as_array().expect("a row").iter().flat_map(floats))
                    .collect();
                let flat: Vec<f64> = got.iter().flat_map(|p| [p.x, p.y]).collect();
                assert!(worst(&flat, &expected) <= tolerance);
            }
        }
    }

    /// Vertical, coincident and malformed pairs are refused.
    #[test]
    fn rejects_bad_pairs() {
        let k = [500.0, 500.0, 320.0, 240.0];
        let camera = Camera {
            model: Model::Pinhole,
            intrinsics: &k,
        };
        let mut vertical = Matrix4::identity();
        vertical[(1, 3)] = 0.1;
        assert_eq!(
            rectify(camera, camera, &vertical, 480, 640)
                .unwrap_err()
                .kind,
            ErrorKind::InvalidInput
        );
        assert_eq!(
            rectify(camera, camera, &Matrix4::identity(), 480, 640)
                .unwrap_err()
                .kind,
            ErrorKind::Degenerate
        );
        let mut side = Matrix4::identity();
        side[(0, 3)] = -0.1;
        assert!(rectify(camera, camera, &side, 0, 640).is_err());
        assert!(rectify(camera, camera, &side, 480, 640).is_ok());
    }
}
