//! Bundle adjustment's solve: the problem centred on its cameras, laid out, adjusted, and
//! shifted back.
//!
//! Mirrors `slam_it.foundations.ba.solve`. It sits above every other module of the package
//! (the layout, the bundle, the gauge and the Schur system), which is why it is not in the
//! package's facade: `super::solve` from inside `ba` names this module, and the layer check
//! holds it to its place.

use nalgebra::{Matrix4, Vector3};

use super::{Gauge, Options, Problem, Solution, at, bundle, check_problem, layout};
use crate::error::{ErrorKind, Result, fail};
use crate::se3;

/// The point a solve is centred on.
///
/// The mean of the centres of the cameras with a positive-weight observation (all of them if
/// none is), summed in index order. The solve shifts the world by its negative and back.
///
/// # Arguments
///
/// * `problem` - The problem (checked).
///
/// # Returns
///
/// The origin.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "a camera count is far below 2^52"
)]
pub fn centring_origin(problem: &Problem) -> Vector3<f64> {
    // The cameras with a positive-weight observation: an unobserved placeholder (an
    // unregistered camera left at the identity) must not pull the origin off the scene.
    let mut seen = vec![false; problem.poses.len()];
    for (m, &w) in problem.weights.iter().enumerate() {
        if w > 0.0 {
            seen[at(problem.cam_idx[m])] = true;
        }
    }
    let any_seen = seen.contains(&true);
    let mut sum = Vector3::zeros();
    let mut count = 0;
    for (pose, &observed) in problem.poses.iter().zip(&seen) {
        if observed || !any_seen {
            sum += se3::translation(pose); // in index order, as Python sums
            count += 1;
        }
    }
    if count > 0 {
        sum / f64::from(count)
    } else {
        sum
    }
}

/// The problem shifted by `-origin`: camera centres, points and prior centres.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `origin` - The shift.
///
/// # Returns
///
/// The shifted problem.
fn shifted(problem: &Problem, origin: &Vector3<f64>) -> Problem {
    let shift = |pose: &Matrix4<f64>| {
        let mut out = *pose;
        out.fixed_view_mut::<3, 1>(0, 3)
            .copy_from(&(se3::translation(pose) - origin));
        out
    };
    Problem {
        poses: problem.poses.iter().map(shift).collect(),
        pose_prior: problem.pose_prior.iter().map(shift).collect(),
        points: problem.points.iter().map(|p| p - origin).collect(),
        ..problem.clone()
    }
}

/// Refines a problem's poses, intrinsics and points.
///
/// # Arguments
///
/// * `problem` - The problem.
/// * `options` - Solver settings, gauge and degeneracy threshold.
///
/// # Returns
///
/// The solution.
///
/// # Errors
///
/// `InvalidInput` for an invalid problem or options; `NotConverged` if
/// `options.solver.strict` and the solve stopped without converging.
pub fn solve(problem: &Problem, options: &Options) -> Result<Solution> {
    check_problem(problem)?;
    if !options.min_parallax_deg.is_finite()
        || options.min_parallax_deg < 0.0
        || !options.focal_std_ratio.is_finite()
        || options.focal_std_ratio < 0.0
    {
        return fail(
            ErrorKind::InvalidInput,
            "min_parallax_deg, focal_std_ratio: must be non-negative and finite",
        );
    }
    if options.gauge == Gauge::Partial {
        return fail(
            ErrorKind::InvalidInput,
            "gauge: 'partial' is reported, not requested",
        );
    }
    if !options.solver.block_loss.losses.is_empty() {
        return fail(
            ErrorKind::InvalidInput,
            "solver.block_loss: set by bundle adjustment, not the caller",
        );
    }
    // Solve in a world centred on the cameras: exact (every residual is unchanged by a
    // shift), and it keeps the steps well conditioned far from the origin, where a step's
    // rotation about the world origin would swing a camera by about |θ|² |C| / 2.
    let origin = centring_origin(problem);
    let centred = shifted(problem, &origin);
    let layout = layout::make_layout(&centred, options)?;
    let mut solution = bundle::adjust(&centred, options, &layout)?;
    // Shift back; what did not move (fixed or unobserved poses, fixed or dropped points) is
    // returned as given, bit for bit.
    for ((pose, centred_pose), original) in solution
        .poses
        .iter_mut()
        .zip(&centred.poses)
        .zip(&problem.poses)
    {
        if pose == centred_pose {
            *pose = *original;
        } else {
            let moved = se3::translation(pose) + origin;
            pose.fixed_view_mut::<3, 1>(0, 3).copy_from(&moved);
        }
    }
    for ((point, centred_point), original) in solution
        .points
        .iter_mut()
        .zip(&centred.points)
        .zip(&problem.points)
    {
        if point == centred_point {
            *point = *original;
        } else {
            *point += origin;
        }
    }
    Ok(solution)
}

#[cfg(test)]
mod tests {
    //! Native tests of bundle adjustment, runnable without Python: the problem checks, and
    //! small synthetic scenes solved back to their truth. The Python reference holds every
    //! value to its parity fixture.

    use nalgebra::Vector6;

    use nalgebra::Vector2;

    use super::*;
    use crate::ba::Intrinsics;
    use crate::camera::{self, Model};
    use crate::error::ErrorKind;
    use crate::nls;

    /// Pinhole intrinsics.
    const PINHOLE: [f64; 4] = [500.0, 500.0, 320.0, 240.0];

    /// A noise-free scene: three cameras along x looking down z at a cloud of points, every
    /// point seen by every camera, with the first two poses fixed.
    fn scene() -> Problem {
        let poses: Vec<Matrix4<f64>> = [-0.6, 0.0, 0.7]
            .iter()
            .enumerate()
            .map(|(c, &x)| {
                let angle = 0.05 * f64::from(u8::try_from(c).unwrap());
                se3::exp(&Vector6::new(x, 0.1 * angle, 0.0, 0.0, angle, -angle))
            })
            .collect();
        let points: Vec<Vector3<f64>> = (0..20)
            .map(|i| {
                let t = f64::from(i);
                Vector3::new((1.3 * t).sin(), (0.7 * t).cos(), 5.0 + (0.9 * t).sin())
            })
            .collect();
        let mut problem = every_point_seen(poses, points, |_, _, in_camera| {
            camera::project(Model::Pinhole, &PINHOLE, in_camera).unwrap()
        });
        problem.fixed_poses = vec![true, true, false];
        problem
    }

    /// A pinhole problem in which every camera sees every point.
    ///
    /// # Arguments
    ///
    /// * `poses` - The cameras' `T_world_cam`.
    /// * `points` - The points.
    /// * `pixel` - Each observation's pixel, from its camera's index, its point's and the point
    ///   in the camera's frame.
    ///
    /// # Returns
    ///
    /// The problem, every other array at its default.
    fn every_point_seen(
        poses: Vec<Matrix4<f64>>,
        points: Vec<Vector3<f64>>,
        pixel: impl Fn(usize, usize, &Vector3<f64>) -> Vector2<f64>,
    ) -> Problem {
        let mut observations = Vec::new();
        let (mut cam_idx, mut pt_idx) = (Vec::new(), Vec::new());
        for (c, pose) in poses.iter().enumerate() {
            let t_cam_world = se3::inverse(pose).unwrap();
            for (p, point) in points.iter().enumerate() {
                observations.push(pixel(c, p, &se3::act(&t_cam_world, point).unwrap()));
                cam_idx.push(i64::try_from(c).unwrap());
                pt_idx.push(i64::try_from(p).unwrap());
            }
        }
        let intrinsics = Intrinsics {
            values: PINHOLE.to_vec(),
            params: 4,
        };
        Problem::with_defaults(
            Model::Pinhole,
            intrinsics,
            poses,
            points,
            observations,
            cam_idx,
            pt_idx,
        )
    }

    /// Tight settings, so a noise-free solve reaches the truth.
    fn tight() -> Options {
        Options {
            solver: nls::Options {
                max_iterations: 100,
                function_tolerance: 0.0,
                gradient_tolerance: 1e-12,
                step_tolerance: 1e-14,
                ..nls::Options::default()
            },
            ..Options::default()
        }
    }

    /// From a perturbed start, the free pose and the points return to the truth; the fixed
    /// poses come back bit for bit. This runs the layout, the Schur system and the bundle.
    #[test]
    fn noise_free_scene_recovers_truth() {
        let truth = scene();
        let mut start = truth.clone();
        start.poses[2] = se3::compose(
            &truth.poses[2],
            &se3::exp(&Vector6::new(0.02, -0.01, 0.03, 0.01, -0.02, 0.01)),
        )
        .unwrap();
        for (p, point) in start.points.iter_mut().enumerate() {
            let t = f64::from(u8::try_from(p).unwrap());
            *point += Vector3::new(0.01 * t.sin(), 0.02 * t.cos(), -0.03 * (2.0 * t).sin());
        }
        let solved = solve(&start, &tight()).unwrap();
        assert!(solved.summary.solve.termination.is_converged());
        assert!(crate::solve::Summary::converged(&solved.summary));
        assert_eq!(
            crate::solve::Summary::iterations(&solved.summary),
            solved.summary.solve.trace.len()
        );
        assert_eq!(solved.summary.gauge, Gauge::None);
        assert_eq!(solved.poses[0], truth.poses[0]);
        assert!((solved.poses[2] - truth.poses[2]).amax() < 1e-9);
        for (got, want) in solved.points.iter().zip(&truth.points) {
            assert!((got - want).amax() < 1e-8);
        }
    }

    /// A point seen by one camera only is dropped and counted, with its observation.
    #[test]
    fn a_point_seen_once_is_dropped() {
        let mut problem = scene();
        problem.points.push(Vector3::new(0.2, -0.1, 4.0));
        problem.fixed_points.push(false);
        problem.point_anchor.push(-1);
        problem.observations.push(Vector2::new(330.0, 230.0));
        problem.cam_idx.push(0);
        problem.pt_idx.push(20);
        problem.weights.push(1.0);
        problem.depth_prior.push(f64::NAN);
        problem.depth_prior_weights.push(1.0);
        let solved = solve(&problem, &tight()).unwrap();
        assert_eq!(solved.summary.dropped_points, 1);
        assert_eq!(solved.summary.dropped_observations, 1);
        assert_eq!(solved.points[20], problem.points[20]);
    }

    /// Points 1e-9 m in front of a held camera, at its centre (advio-01's local BA, B1): camera
    /// 0, held, turned and off the origin, sees 101 points within 1e-16 m of 1e-9 m in front of
    /// it (`camera::MIN_DEPTH`) and 12 more two to three metres out; cameras 1 and 2, free, a
    /// metre behind it and 0.2 m to either side, see all of them; every point is held. The
    /// in-front test and the residuals take a point's depth from one computation, so an
    /// observation the projection would refuse is left out at the start: the solve no longer
    /// meets an infinite starting cost.
    #[test]
    fn a_point_at_a_cameras_centre_is_left_out() {
        let rotation = crate::so3::exp(&Vector3::new(0.3, -0.5, 0.7));
        let centre = Vector3::new(-0.0145, -0.0038, -0.0345);
        let mut local: Vec<Vector3<f64>> = (0..101_i32)
            .map(|k| Vector3::new(3.49e-11, 1.458e-10, 1e-9 + f64::from(k - 50) * 2e-18))
            .collect();
        for x in [-1.0, 0.0, 1.0] {
            for y in [-0.5, 0.5] {
                for z in [2.0, 3.0] {
                    local.push(Vector3::new(x, y, z));
                }
            }
        }
        let points: Vec<Vector3<f64>> = local.iter().map(|l| centre + rotation * l).collect();
        let poses: Vec<Matrix4<f64>> = [
            centre,
            centre - rotation * Vector3::new(-0.2, 0.0, 1.0),
            centre - rotation * Vector3::new(0.2, 0.0, 1.0),
        ]
        .iter()
        .map(|c| se3::from_parts(&rotation, c))
        .collect();
        // Camera 0's as meant; by hand, as `camera::project` refuses those behind 1e-9 m.
        let mut problem = every_point_seen(poses, points, |c, p, in_camera| {
            let x = if c == 0 { local[p] } else { *in_camera };
            Vector2::new(
                PINHOLE[0] * x.x / x.z + PINHOLE[2],
                PINHOLE[1] * x.y / x.z + PINHOLE[3],
            )
        });
        problem.fixed_poses = vec![true, false, false];
        problem.fixed_points = vec![true; problem.points.len()];
        let solved = solve(&problem, &Options::default()).unwrap();
        assert!((1..=101).contains(&solved.summary.dropped_observations));
        assert!(solved.summary.solve.final_cost.is_finite());
    }

    /// Each malformed problem is `InvalidInput`, naming what is wrong.
    #[test]
    fn invalid_problems_are_rejected() {
        type Change = Box<dyn Fn(&mut Problem)>;
        let cases: Vec<(&str, Change)> = vec![
            ("cam_idx", Box::new(|p| p.cam_idx[0] = 9)),
            ("pt_idx", Box::new(|p| p.pt_idx[0] = -1)),
            ("weights", Box::new(|p| p.weights[1] = -1.0)),
            (
                "points, observations",
                Box::new(|p| p.points[0].x = f64::NAN),
            ),
            (
                "fixed masks",
                Box::new(|p| {
                    p.fixed_points.pop();
                }),
            ),
            ("depth_prior", Box::new(|p| p.depth_prior[0] = -0.5)),
            ("transform", Box::new(|p| p.poses[1][(3, 0)] = 1.0)),
            ("intrinsics", Box::new(|p| p.intrinsics.values[0] = 0.0)),
            (
                "fixed point cannot be anchored",
                Box::new(|p| {
                    p.point_anchor[0] = 0;
                    p.fixed_points[0] = true;
                }),
            ),
        ];
        for (message, change) in cases {
            let mut problem = scene();
            change(&mut problem);
            let err = solve(&problem, &tight()).unwrap_err();
            assert_eq!(err.kind, ErrorKind::InvalidInput, "{message}");
            assert!(err.message.contains(message), "{message}: {}", err.message);
        }
        let partial = Options {
            gauge: Gauge::Partial,
            ..tight()
        };
        assert!(
            solve(&scene(), &partial)
                .unwrap_err()
                .message
                .contains("partial")
        );
    }
}
