//! Bundle adjustment internals: the blocks, Jacobians, retraction and solve.

use std::rc::Rc;

use nalgebra::{
    DMatrix, DVector, Dyn, Matrix2, Matrix3, Matrix4, OMatrix, SMatrix, SVector, U3, Vector1,
    Vector2, Vector3, Vector6,
};

use super::gauge;
use super::layout::Layout;
use super::schur::{Blocks, Coupling, CouplingLayout, Reduction, SchurSystem};
use super::{DenseCholesky, Gauge, Intrinsics, Options, POSE_DIM, Problem, Solution, Summary, at};
use crate::camera::{self, Model};
use crate::error::Result;
use crate::nls;
use crate::{se3, so3};

/// The most intrinsics a camera model has: radtan's nine.
const MAX_INTRINSICS: usize = Model::RadTan.intrinsics_len();
/// The most camera-side columns a residual block touches: its camera's pose and intrinsics, and
/// an anchored point's host's.
const MAX_COLUMNS: usize = 2 * (POSE_DIM + MAX_INTRINSICS);

/// A camera's centre `-Rᵀ t`, from its world-to-camera pose.
///
/// # Arguments
///
/// * `t_cam_world` - The pose.
///
/// # Returns
///
/// The centre in the world frame.
fn centre(t_cam_world: &Matrix4<f64>) -> Vector3<f64> {
    -se3::rotation(t_cam_world).transpose() * se3::translation(t_cam_world)
}

/// Inverts a rigid pose (world-to-camera and back).
///
/// # Arguments
///
/// * `pose` - The pose (already checked).
///
/// # Returns
///
/// The inverse.
fn invert(pose: &Matrix4<f64>) -> Matrix4<f64> {
    let rt = se3::rotation(pose).transpose();
    se3::from_parts(&rt, &(-rt * se3::translation(pose)))
}

/// The variables: world-to-camera poses, intrinsics, points, affines, inverse depths.
#[derive(Debug, Clone)]
pub struct BundleState {
    /// World-to-camera poses.
    poses: Vec<Matrix4<f64>>,
    /// Intrinsics groups.
    intrinsics: Intrinsics,
    /// Plain 3D points (anchored rows unused).
    points: Vec<Vector3<f64>>,
    /// Depth affines `(scale, shift)`.
    depth_affine: Vec<Vector2<f64>>,
    /// Anchored points' inverse depths (NaN for the others).
    inverse_depth: Vec<f64>,
}

/// One residual block of `R` rows linearised at a state (rows already scaled by `sqrt(w)`):
/// fixed-size by its kind (a reprojection's 2 rows, a depth prior's 1, a pose prior's 6), so it
/// lives on the stack and its small products unroll. Its camera-side coordinates are fixed for
/// the problem (`Bundle::block_columns`).
#[derive(Debug, Clone)]
struct LinearBlock<const R: usize> {
    /// The weighted residual.
    residual: SVector<f64, R>,
    /// d residual / d(camera-side variables), its first `width` columns in `block_columns`'
    /// order.
    camera_side: SMatrix<f64, R, MAX_COLUMNS>,
    /// The number of camera-side columns.
    width: usize,
    /// d residual / d(point variables): xyz, or (rho, 0, 0) for an anchored point (zero for
    /// a block that touches no point).
    point: SMatrix<f64, R, 3>,
    /// The point's free slot.
    slot: Option<usize>,
}

impl<const R: usize> LinearBlock<R> {
    /// A block observing a point: its camera's pose columns from the world-point Jacobian,
    /// `(J, -J hat(X))`; the rest of its `width` columns are the caller's to fill.
    ///
    /// # Arguments
    ///
    /// * `residual` - The weighted residual.
    /// * `j_world` - d residual / d world point.
    /// * `point` - The world point.
    /// * `width` - The number of camera-side columns.
    /// * `slot` - The point's free slot.
    ///
    /// # Returns
    ///
    /// The block.
    fn observing(
        residual: SVector<f64, R>,
        j_world: SMatrix<f64, R, 3>,
        point: &Vector3<f64>,
        width: usize,
        slot: Option<usize>,
    ) -> Self {
        let mut camera_side = SMatrix::<f64, R, MAX_COLUMNS>::zeros();
        camera_side.fixed_view_mut::<R, 3>(0, 0).copy_from(&j_world);
        camera_side
            .fixed_view_mut::<R, 3>(0, 3)
            .copy_from(&(-j_world * so3::hat(point)));
        Self {
            residual,
            camera_side,
            width,
            point: j_world,
            slot,
        }
    }
}

/// An anchored point's world position and its Jacobians at a state.
#[derive(Debug, Clone)]
struct AnchorJacobian {
    /// d world / d rho.
    rho: Vector3<f64>,
    /// d world / d(host pose).
    pose: OMatrix<f64, U3, nalgebra::U6>,
    /// d world / d(host intrinsics).
    intrinsics: OMatrix<f64, U3, Dyn>,
}

/// The bundle-adjustment problem as an [`nls::Problem`] over [`BundleState`].
struct Bundle<'a> {
    /// The problem.
    problem: &'a Problem,
    /// Its layout.
    layout: &'a Layout,
    /// Where the intrinsics start in the full camera-side vector.
    intrinsics_offset: usize,
    /// Where the affines start.
    affine_offset: usize,
    /// The full camera-side vector's length.
    full_size: usize,
    /// The free full entries.
    free: Vec<usize>,
    /// The confined poses' full entries.
    confined_rows: Vec<usize>,
    /// The anchored points.
    anchored_points: Vec<usize>,
    /// Each point's index among the anchored points.
    anchor_index: Vec<Option<usize>>,
    /// Each free point's number of variables.
    point_dims: Vec<usize>,
    /// The number of kept camera-side coordinates (`lay_out_blocks`).
    n_kept: usize,
    /// Each block's columns as positions among the kept coordinates (`None` where dropped).
    block_kept: Vec<Vec<Option<usize>>>,
    /// Each block's columns' rows in its point's coupling.
    block_rows: Vec<Vec<Option<usize>>>,
    /// Each free point's coupling rows.
    coupling_layout: Rc<CouplingLayout>,
}

impl<'a> Bundle<'a> {
    /// Indexes the anchored points, the free coordinates and every block's columns.
    ///
    /// # Arguments
    ///
    /// * `problem` - The problem.
    /// * `layout` - Its layout.
    ///
    /// # Returns
    ///
    /// The bundle.
    fn new(problem: &'a Problem, layout: &'a Layout) -> Self {
        let n_cams = problem.poses.len();
        let intrinsics_offset = POSE_DIM * n_cams;
        let affine_offset = intrinsics_offset + problem.intrinsics.values.len();
        let full_size = affine_offset + 2 * n_cams;
        let mut free = Vec::new();
        for c in 0..n_cams {
            if layout.pose_free[c] {
                free.extend(POSE_DIM * c..POSE_DIM * (c + 1));
            }
        }
        for (k, &is_free) in layout.intrinsics_free.iter().enumerate() {
            if is_free {
                free.push(intrinsics_offset + k);
            }
        }
        for c in 0..n_cams {
            if layout.affine_free[c] {
                free.extend(affine_offset + 2 * c..affine_offset + 2 * c + 2);
            }
        }
        let confined_rows = layout
            .gauge_plan
            .confined
            .iter()
            .flat_map(|&c| POSE_DIM * c..POSE_DIM * (c + 1))
            .collect();
        let mut anchor_index = vec![None; layout.anchored.len()];
        let mut anchored_points = Vec::new();
        let mut point_dims = vec![3; layout.free_points];
        for (p, &anchored) in layout.anchored.iter().enumerate() {
            if !anchored {
                continue;
            }
            anchor_index[p] = Some(anchored_points.len());
            anchored_points.push(p);
            if let Some(slot) = layout.point_slot[p] {
                point_dims[slot] = 1;
            }
        }
        let mut bundle = Self {
            problem,
            layout,
            intrinsics_offset,
            affine_offset,
            full_size,
            free,
            confined_rows,
            anchored_points,
            anchor_index,
            point_dims,
            n_kept: 0,
            block_kept: Vec::new(),
            block_rows: Vec::new(),
            coupling_layout: Rc::default(),
        };
        bundle.lay_out_blocks();
        bundle
    }

    /// Each block's loss: 0 reprojection, 1 depth prior, 2 pose prior.
    ///
    /// # Returns
    ///
    /// The loss index of every block, in `squared_norms` order.
    fn loss_index(&self) -> Vec<usize> {
        let mut index = vec![0; self.layout.reprojection.len()];
        index.resize(index.len() + self.layout.depth.len(), 1);
        index.resize(index.len() + self.layout.prior_cams.len(), 2);
        index
    }

    /// The starting state: anchored points placed on their rays at their depth.
    ///
    /// # Returns
    ///
    /// The state.
    fn start(&self) -> BundleState {
        let problem = self.problem;
        let mut state = BundleState {
            poses: problem.poses.iter().map(invert).collect(),
            intrinsics: problem.intrinsics.clone(),
            points: problem.points.clone(),
            depth_affine: problem.depth_affine.clone(),
            inverse_depth: vec![f64::NAN; problem.points.len()],
        };
        for &p in &self.anchored_points {
            let host = &state.poses[self.host_of(p)];
            let in_host = se3::rotation(host) * problem.points[p] + se3::translation(host);
            state.inverse_depth[p] = 1.0 / in_host.z;
        }
        state
    }

    /// The number of weighted residual rows: 2 per reprojection, 1 per depth prior, and 3 per
    /// weighted half of a pose prior.
    ///
    /// # Returns
    ///
    /// The row count.
    fn residual_rows(&self) -> usize {
        let mut rows = 2 * self.layout.reprojection.len() + self.layout.depth.len();
        for &c in &self.layout.prior_cams {
            // 3 per weighted half: a translation-only prior's rotation rows carry no
            // information.
            let weights = &self.problem.pose_prior_weights[c];
            rows += if weights[0] > 0.0 { 3 } else { 0 } + if weights[1] > 0.0 { 3 } else { 0 };
        }
        rows
    }

    /// The number of free parameters (the solver's step length) at `state`.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    ///
    /// # Returns
    ///
    /// The count.
    fn free_size(&self, state: &BundleState) -> usize {
        self.reduction(state).size() + self.point_dims.iter().sum::<usize>()
    }

    /// The free intrinsics parameters and their free coordinates.
    ///
    /// # Returns
    ///
    /// Each free parameter's row-major index in the intrinsics, and its position.
    fn intrinsics_positions(&self) -> (Vec<usize>, Vec<usize>) {
        let mut entries = Vec::new();
        let mut positions = Vec::new();
        for (k, &is_free) in self.layout.intrinsics_free.iter().enumerate() {
            if is_free {
                entries.push(k);
                let full = self.intrinsics_offset + k;
                positions.push(self.free.partition_point(|&f| f < full));
            }
        }
        (entries, positions)
    }

    /// The free camera-side coordinates at `state`: each confined pose's rows map to a basis
    /// of the steps that leave the gauge alone, rebuilt as the poses move.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    ///
    /// # Returns
    ///
    /// The reduction.
    fn reduction(&self, state: &BundleState) -> Reduction {
        let centres: Vec<Vector3<f64>> = state.poses.iter().map(centre).collect();
        let pose_bases = gauge::confinement(
            &self.layout.gauge_anchors,
            &centres,
            &self.layout.gauge_plan,
        );
        let columns: usize = pose_bases.iter().map(DMatrix::ncols).sum();
        let mut basis = DMatrix::zeros(POSE_DIM * pose_bases.len(), columns);
        let mut column = 0;
        for (i, block) in pose_bases.iter().enumerate() {
            basis
                .view_mut((POSE_DIM * i, column), (POSE_DIM, block.ncols()))
                .copy_from(block);
            column += block.ncols();
        }
        Reduction::new(
            self.free.clone(),
            self.confined_rows.clone(),
            basis,
            self.full_size,
        )
    }

    /// Every point in the world frame (`None` where an anchored point is invalid).
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    ///
    /// # Returns
    ///
    /// The points.
    fn world_points(&self, state: &BundleState) -> Vec<Option<Vector3<f64>>> {
        let mut out: Vec<Option<Vector3<f64>>> = state.points.iter().copied().map(Some).collect();
        for &p in &self.anchored_points {
            let rho = state.inverse_depth[p];
            out[p] = match self.anchor_ray(state, p) {
                Some(ray) if rho > 0.0 => {
                    let host = &state.poses[self.host_of(p)];
                    Some(se3::rotation(host).transpose() * (ray / rho - se3::translation(host)))
                }
                _ => None,
            };
        }
        out
    }

    /// An anchored point's host camera.
    ///
    /// # Arguments
    ///
    /// * `p` - The point.
    ///
    /// # Returns
    ///
    /// The camera of its anchor observation.
    fn host_of(&self, p: usize) -> usize {
        at(self.problem.cam_idx[at(self.problem.point_anchor[p])])
    }

    /// A camera's intrinsics group at a state.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `c` - The camera.
    ///
    /// # Returns
    ///
    /// The intrinsics.
    fn intrinsics_of<'s>(&self, state: &'s BundleState, c: usize) -> &'s [f64] {
        state.intrinsics.group(at(self.problem.intrinsics_idx[c]))
    }

    /// An anchored point's ray on its host's normalised plane, `(x, y, 1)`.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `p` - The point.
    ///
    /// # Returns
    ///
    /// The ray, or `None` if the anchor pixel cannot be undistorted.
    fn anchor_ray(&self, state: &BundleState, p: usize) -> Option<Vector3<f64>> {
        let a = at(self.problem.point_anchor[p]);
        let k = self.intrinsics_of(state, self.host_of(p));
        let bearing =
            camera::unproject(self.problem.model, k, &self.problem.observations[a]).ok()?;
        Some(bearing / bearing.z)
    }

    /// Every anchored point's Jacobians at a state.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `world` - The world points at `state`.
    ///
    /// # Returns
    ///
    /// One entry per anchored point (`None` where invalid).
    fn anchors(
        &self,
        state: &BundleState,
        world: &[Option<Vector3<f64>>],
    ) -> Vec<Option<AnchorJacobian>> {
        self.anchored_points
            .iter()
            .map(|&p| {
                let ray = self.anchor_ray(state, p)?;
                let point = world[p]?;
                let k = self.intrinsics_of(state, self.host_of(p));
                let jacobian = camera::project_jacobian(self.problem.model, k, &ray).ok()?;
                let rho = state.inverse_depth[p];
                let rotation_t = se3::rotation(&state.poses[self.host_of(p)]).transpose();
                let n_params = self.problem.intrinsics.params;
                let d_pixel_d_ray: Matrix2<f64> = jacobian.fixed_columns::<2>(0).into_owned();
                // An exactly singular pixel Jacobian (practically unreachable at a converged
                // undistortion) leaves the point without a Jacobian, so its blocks drop out;
                // C++ would carry inf or NaN, and the reference would raise.
                let d_xy = -d_pixel_d_ray.try_inverse()? * jacobian.columns(3, n_params);
                let mut d_ray = OMatrix::<f64, U3, Dyn>::zeros(n_params);
                d_ray.rows_mut(0, 2).copy_from(&d_xy);
                let mut d_pose = OMatrix::<f64, U3, nalgebra::U6>::zeros();
                d_pose
                    .fixed_columns_mut::<3>(0)
                    .copy_from(&-Matrix3::identity());
                d_pose
                    .fixed_columns_mut::<3>(3)
                    .copy_from(&so3::hat(&point));
                Some(AnchorJacobian {
                    rho: -rotation_t * ray / (rho * rho),
                    pose: d_pose,
                    intrinsics: rotation_t * d_ray / rho,
                })
            })
            .collect()
    }

    /// Chains a block's world-point Jacobian through an anchored point: the point's Jacobian
    /// becomes `(d/d rho, 0, 0)` and the host's pose and intrinsics columns follow the block's.
    ///
    /// # Arguments
    ///
    /// * `block` - The block, whose `point` holds d residual / d world.
    /// * `p` - The block's point.
    /// * `anchor_jacobians` - Every anchored point's Jacobians.
    ///
    /// # Returns
    ///
    /// The block, or `None` if the anchored point is invalid.
    fn couple<const R: usize>(
        &self,
        mut block: LinearBlock<R>,
        p: usize,
        anchor_jacobians: &[Option<AnchorJacobian>],
    ) -> Option<LinearBlock<R>> {
        let Some(a) = self.anchor_index[p] else {
            return Some(block);
        };
        let anchor = anchor_jacobians[a].as_ref()?;
        let j_world = block.point;
        let n_params = self.problem.intrinsics.params;
        let width = block.width;
        block
            .camera_side
            .fixed_view_mut::<R, POSE_DIM>(0, width)
            .copy_from(&(j_world * anchor.pose));
        block
            .camera_side
            .view_mut((0, width + POSE_DIM), (R, n_params))
            .copy_from(&(j_world * &anchor.intrinsics));
        block.width = width + POSE_DIM + n_params;
        block.point = SMatrix::zeros();
        block.point.set_column(0, &(j_world * anchor.rho));
        Some(block)
    }

    /// An observation's point in its camera's frame.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `point` - The point in the world frame.
    /// * `m` - The observation.
    ///
    /// # Returns
    ///
    /// The point in the camera frame.
    fn in_camera(&self, state: &BundleState, point: &Vector3<f64>, m: usize) -> Vector3<f64> {
        let pose = &state.poses[at(self.problem.cam_idx[m])];
        se3::rotation(pose) * point + se3::translation(pose)
    }

    /// An observation's world point.
    ///
    /// # Arguments
    ///
    /// * `world` - The world points.
    /// * `m` - The observation.
    ///
    /// # Returns
    ///
    /// The point, or `None` if it is invalid.
    fn point_of(&self, world: &[Option<Vector3<f64>>], m: usize) -> Option<Vector3<f64>> {
        world[at(self.problem.pt_idx[m])]
    }

    /// A reprojection block's weighted error.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `world` - The world points.
    /// * `i` - The block.
    ///
    /// # Returns
    ///
    /// The residual, or `None` if the point is behind the camera or invalid.
    fn reprojection(
        &self,
        state: &BundleState,
        world: &[Option<Vector3<f64>>],
        i: usize,
    ) -> Option<Vector2<f64>> {
        let m = self.layout.obs[self.layout.reprojection[i]];
        let point = self.point_of(world, m)?;
        let k = self.intrinsics_of(state, at(self.problem.cam_idx[m]));
        let pixel =
            camera::project(self.problem.model, k, &self.in_camera(state, &point, m)).ok()?;
        Some((pixel - self.problem.observations[m]) * self.problem.weights[m].sqrt())
    }

    /// A depth prior's weighted inverse-depth error.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `world` - The world points.
    /// * `d` - The depth block.
    ///
    /// # Returns
    ///
    /// The residual, or `None` if the point is behind the camera or invalid.
    fn depth(&self, state: &BundleState, world: &[Option<Vector3<f64>>], d: usize) -> Option<f64> {
        let m = self.layout.obs[self.layout.depth[d]];
        let point = self.point_of(world, m)?;
        let z = self.in_camera(state, &point, m).z;
        if z <= camera::MIN_DEPTH {
            return None;
        }
        let affine = &state.depth_affine[at(self.problem.cam_idx[m])];
        let predicted = affine[0] * self.problem.depth_prior[m] + affine[1];
        Some(self.problem.depth_prior_weights[m].sqrt() * (1.0 / z - predicted))
    }

    /// A pose prior's unweighted error `(C - C_prior, log(R_priorᵀ R_world_cam))`: decoupled,
    /// so the translation half pins the world centre whatever rotation the prior stores (a
    /// GNSS-style prior).
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `k` - The prior.
    ///
    /// # Returns
    ///
    /// The error.
    fn pose_error(&self, state: &BundleState, k: usize) -> Option<Vector6<f64>> {
        let c = self.layout.prior_cams[k];
        let prior = &self.problem.pose_prior[c];
        let r_world_cam = se3::rotation(&state.poses[c]).transpose();
        let rotation_error = so3::log(&(se3::rotation(prior).transpose() * r_world_cam)).ok()?;
        let centre_error = centre(&state.poses[c]) - se3::translation(prior);
        Some(Vector6::new(
            centre_error.x,
            centre_error.y,
            centre_error.z,
            rotation_error.x,
            rotation_error.y,
            rotation_error.z,
        ))
    }

    /// A pose prior's weights, square-rooted, per tangent component.
    ///
    /// # Arguments
    ///
    /// * `k` - The prior.
    ///
    /// # Returns
    ///
    /// `(sqrt w_t) x3, (sqrt w_r) x3`.
    fn pose_sqrt_weights(&self, k: usize) -> Vector6<f64> {
        let weights = &self.problem.pose_prior_weights[self.layout.prior_cams[k]];
        let (t, r) = (weights[0].sqrt(), weights[1].sqrt());
        Vector6::new(t, t, t, r, r, r)
    }

    /// A pose prior's weighted error.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `k` - The prior.
    ///
    /// # Returns
    ///
    /// The residual.
    fn pose_residual(&self, state: &BundleState, k: usize) -> Option<Vector6<f64>> {
        Some(
            self.pose_sqrt_weights(k)
                .component_mul(&self.pose_error(state, k)?),
        )
    }

    /// Linearises a reprojection block.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `world` - The world points.
    /// * `anchor_jacobians` - Every anchored point's Jacobians.
    /// * `i` - The block.
    ///
    /// # Returns
    ///
    /// The block, or `None` if its point is behind or invalid.
    fn reprojection_block(
        &self,
        state: &BundleState,
        world: &[Option<Vector3<f64>>],
        anchor_jacobians: &[Option<AnchorJacobian>],
        i: usize,
    ) -> Option<LinearBlock<2>> {
        let m = self.layout.obs[self.layout.reprojection[i]];
        let c = at(self.problem.cam_idx[m]);
        let p = at(self.problem.pt_idx[m]);
        let g = at(self.problem.intrinsics_idx[c]);
        let n_params = self.problem.intrinsics.params;
        let point = self.point_of(world, m)?;
        let residual = self.reprojection(state, world, i)?;
        let rotation = se3::rotation(&state.poses[c]);
        let k = state.intrinsics.group(g);
        let jacobian =
            camera::project_jacobian(self.problem.model, k, &self.in_camera(state, &point, m))
                .ok()?;
        let scale = self.problem.weights[m].sqrt();
        let j_world = jacobian.fixed_columns::<3>(0) * rotation * scale;
        let mut block = LinearBlock::observing(
            residual,
            j_world,
            &point,
            POSE_DIM + n_params,
            self.layout.point_slot[p],
        );
        block
            .camera_side
            .view_mut((0, POSE_DIM), (2, n_params))
            .copy_from(&(jacobian.columns(3, n_params) * scale));
        self.couple(block, p, anchor_jacobians)
    }

    /// Linearises a depth-prior block: `d(1/z) = -dz / z²`.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `world` - The world points.
    /// * `anchor_jacobians` - Every anchored point's Jacobians.
    /// * `d` - The depth block.
    ///
    /// # Returns
    ///
    /// The block, or `None` if its point is behind or invalid.
    fn depth_block(
        &self,
        state: &BundleState,
        world: &[Option<Vector3<f64>>],
        anchor_jacobians: &[Option<AnchorJacobian>],
        d: usize,
    ) -> Option<LinearBlock<1>> {
        let m = self.layout.obs[self.layout.depth[d]];
        let c = at(self.problem.cam_idx[m]);
        let p = at(self.problem.pt_idx[m]);
        let residual = self.depth(state, world, d)?;
        let point = self.point_of(world, m)?;
        let z = self.in_camera(state, &point, m).z;
        let scale = self.problem.depth_prior_weights[m].sqrt();
        let j_world = state.poses[c].fixed_view::<1, 3>(2, 0) * (-scale / (z * z));
        let mut block = LinearBlock::observing(
            Vector1::new(residual),
            j_world,
            &point,
            POSE_DIM + 2,
            self.layout.point_slot[p],
        );
        block.camera_side[(0, POSE_DIM)] = -scale * self.problem.depth_prior[m];
        block.camera_side[(0, POSE_DIM + 1)] = -scale;
        self.couple(block, p, anchor_jacobians)
    }

    /// Linearises a pose-prior block. Under `T_cw exp(δ)`, `δ = (ρ, θ)`, the centre moves by
    /// `-ρ + hat(C) θ` and the rotation error by `-Jr⁻¹(e_r) R_cw θ`.
    ///
    /// # Arguments
    ///
    /// * `state` - The state.
    /// * `k` - The prior.
    ///
    /// # Returns
    ///
    /// The block (touching no point), or `None` if its error cannot be evaluated.
    fn pose_prior_block(&self, state: &BundleState, k: usize) -> Option<LinearBlock<POSE_DIM>> {
        let error = self.pose_error(state, k)?;
        let inverse = so3::right_jacobian_inverse(&error.fixed_rows::<3>(3).into_owned()).ok()?;
        let c = self.layout.prior_cams[k];
        let t_cam_world = &state.poses[c];
        let mut jacobian = nalgebra::Matrix6::<f64>::zeros();
        jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&-Matrix3::identity());
        jacobian
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&so3::hat(&centre(t_cam_world)));
        jacobian
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(-inverse * se3::rotation(t_cam_world)));
        let weights = self.pose_sqrt_weights(k);
        let mut camera_side = SMatrix::<f64, POSE_DIM, MAX_COLUMNS>::zeros();
        camera_side
            .fixed_view_mut::<POSE_DIM, POSE_DIM>(0, 0)
            .copy_from(&(nalgebra::Matrix6::from_diagonal(&weights) * jacobian));
        Some(LinearBlock {
            residual: weights.component_mul(&error),
            camera_side,
            width: POSE_DIM,
            point: SMatrix::zeros(),
            slot: None,
        })
    }

    /// A block's observation.
    ///
    /// # Arguments
    ///
    /// * `b` - The block, in `squared_norms` order.
    ///
    /// # Returns
    ///
    /// The observation, or `None` for a pose prior.
    fn block_observation(&self, b: usize) -> Option<usize> {
        let n_obs = self.layout.reprojection.len();
        let entry = if b < n_obs {
            self.layout.reprojection.get(b)
        } else {
            self.layout.depth.get(b - n_obs)
        };
        entry.map(|&o| self.layout.obs[o])
    }

    /// A block's camera-side coordinates in the full vector, in its Jacobian's column order:
    /// its camera's pose, then its intrinsics (a reprojection) or depth affine (a depth prior),
    /// then an anchored point's host's pose and intrinsics; a pose prior's camera's pose.
    ///
    /// # Arguments
    ///
    /// * `b` - The block, in `squared_norms` order.
    ///
    /// # Returns
    ///
    /// The coordinates (repeated where an anchored point's host is the observer).
    fn block_columns(&self, b: usize) -> Vec<usize> {
        let n_params = self.problem.intrinsics.params;
        let intrinsics_of = |c: usize| {
            let start = self.intrinsics_offset + n_params * at(self.problem.intrinsics_idx[c]);
            start..start + n_params
        };
        let Some(m) = self.block_observation(b) else {
            let n_observing = self.layout.reprojection.len() + self.layout.depth.len();
            let c = self.layout.prior_cams[b - n_observing];
            return (POSE_DIM * c..POSE_DIM * (c + 1)).collect();
        };
        let c = at(self.problem.cam_idx[m]);
        let mut out: Vec<usize> = (POSE_DIM * c..POSE_DIM * (c + 1)).collect();
        if b < self.layout.reprojection.len() {
            out.extend(intrinsics_of(c));
        } else {
            out.extend(self.affine_offset + 2 * c..self.affine_offset + 2 * c + 2);
        }
        let p = at(self.problem.pt_idx[m]);
        if self.anchor_index[p].is_some() {
            let host = self.host_of(p);
            out.extend(POSE_DIM * host..POSE_DIM * (host + 1));
            out.extend(intrinsics_of(host));
        }
        out
    }

    /// A block's free point slot.
    ///
    /// # Arguments
    ///
    /// * `b` - The block, in `squared_norms` order.
    ///
    /// # Returns
    ///
    /// The slot, or `None` for a pose prior or a fixed point.
    fn block_slot(&self, b: usize) -> Option<usize> {
        let m = self.block_observation(b)?;
        self.layout.point_slot[at(self.problem.pt_idx[m])]
    }

    /// Lays out what every linearisation of the problem shares: the kept camera-side
    /// coordinates (the free and confined ones, the only ones the reduced system reads), each
    /// block's columns among them, and each free point's coupling rows (its distinct kept
    /// coordinates, ascending), with each block's row in its point's coupling.
    fn lay_out_blocks(&mut self) {
        let kept = Reduction::new(
            self.free.clone(),
            self.confined_rows.clone(),
            DMatrix::zeros(0, 0),
            self.full_size,
        )
        .kept();
        self.n_kept = kept.len();
        let mut kept_position = vec![None; self.full_size];
        for (k, &c) in kept.iter().enumerate() {
            kept_position[c] = Some(k);
        }
        let n_blocks =
            self.layout.reprojection.len() + self.layout.depth.len() + self.layout.prior_cams.len();
        self.block_kept = (0..n_blocks)
            .map(|b| {
                self.block_columns(b)
                    .iter()
                    .map(|&c| kept_position[c])
                    .collect()
            })
            .collect();
        let mut point_columns: Vec<Vec<usize>> = vec![Vec::new(); self.layout.free_points];
        for (b, columns) in self.block_kept.iter().enumerate() {
            let Some(s) = self.block_slot(b) else {
                continue;
            };
            for &c in columns.iter().flatten() {
                if !point_columns[s].contains(&c) {
                    point_columns[s].push(c);
                }
            }
        }
        let mut coupling = CouplingLayout {
            offset: vec![0],
            column: Vec::new(),
        };
        for coupled in &mut point_columns {
            coupled.sort_unstable(); // ascending: the elimination walks memory in order
            coupling.column.extend_from_slice(coupled);
            coupling.offset.push(coupling.column.len());
        }
        self.block_rows = self
            .block_kept
            .iter()
            .enumerate()
            .map(|(b, columns)| {
                let slot = self.block_slot(b);
                columns
                    .iter()
                    .map(|&c| {
                        let (s, c) = (slot?, c?);
                        Some(coupling.offset[s] + point_columns[s].partition_point(|&k| k < c))
                    })
                    .collect()
            })
            .collect();
        self.coupling_layout = Rc::new(coupling);
    }

    /// Adds one block's weighted contribution to the normal equations: over its kept
    /// camera-side columns only, and its coupling straight into its point's rows.
    ///
    /// # Arguments
    ///
    /// * `b` - The block, in `squared_norms` order.
    /// * `block` - The linearised block, or `None` (unreachable: accepted states keep every
    ///   point valid).
    /// * `w` - Its IRLS weight.
    /// * `blocks` - The normal equations' blocks.
    fn accumulate<const R: usize>(
        &self,
        b: usize,
        block: Option<&LinearBlock<R>>,
        w: f64,
        blocks: &mut Blocks,
    ) {
        let Some(block) = block else {
            return;
        };
        let jacobian = &block.camera_side;
        let kept = &self.block_kept[b];
        for (j, &cj) in kept.iter().enumerate() {
            let Some(cj) = cj else {
                continue;
            };
            blocks.gradient_c[cj] += w * jacobian.column(j).dot(&block.residual);
            for (i, &ci) in kept.iter().enumerate() {
                if let Some(ci) = ci {
                    blocks.hessian_cc[(ci, cj)] += w * jacobian.column(i).dot(&jacobian.column(j));
                }
            }
        }
        let Some(slot) = block.slot else {
            return;
        };
        let point = &block.point;
        blocks.hessian_pp[slot] += (point.transpose() * point) * w;
        blocks.gradient_p[slot] += (point.transpose() * block.residual) * w;
        for (j, &row) in self.block_rows[b].iter().enumerate() {
            if let Some(r) = row {
                let mut target = blocks.coupling.row_mut(r);
                target += (jacobian.column(j).transpose() * point) * w;
            }
        }
    }
}

impl nls::Problem for Bundle<'_> {
    type State = BundleState;
    type System = SchurSystem<DenseCholesky>;

    fn squared_norms(&self, state: &BundleState) -> DVector<f64> {
        let world = self.world_points(state);
        let reprojection = (0..self.layout.reprojection.len()).map(|i| {
            self.reprojection(state, &world, i)
                .map(|r| r.norm_squared())
        });
        let depth =
            (0..self.layout.depth.len()).map(|d| self.depth(state, &world, d).map(|r| r * r));
        let prior = (0..self.layout.prior_cams.len())
            .map(|k| self.pose_residual(state, k).map(|r| r.norm_squared()));
        let norms: Vec<f64> = reprojection
            .chain(depth)
            .chain(prior)
            .map(|s| s.unwrap_or(f64::INFINITY))
            .collect();
        DVector::from_vec(norms)
    }

    fn linearize(&self, state: &BundleState, weights: &DVector<f64>) -> Self::System {
        let n_free = self.layout.free_points;
        let mut blocks = Blocks {
            hessian_cc: DMatrix::zeros(self.n_kept, self.n_kept),
            gradient_c: DVector::zeros(self.n_kept),
            hessian_pp: vec![Matrix3::zeros(); n_free],
            gradient_p: vec![Vector3::zeros(); n_free],
            coupling: Coupling::zeros(self.coupling_layout.column.len()),
            layout: Rc::clone(&self.coupling_layout),
            point_dims: self.point_dims.clone(),
        };
        let world = self.world_points(state);
        let anchor_jacobians = self.anchors(state, &world);
        let mut b = 0; // the block, in `squared_norms` order (its weight's row)
        for i in 0..self.layout.reprojection.len() {
            let block = self.reprojection_block(state, &world, &anchor_jacobians, i);
            self.accumulate(b, block.as_ref(), weights[b], &mut blocks);
            b += 1;
        }
        for d in 0..self.layout.depth.len() {
            let block = self.depth_block(state, &world, &anchor_jacobians, d);
            self.accumulate(b, block.as_ref(), weights[b], &mut blocks);
            b += 1;
        }
        for k in 0..self.layout.prior_cams.len() {
            let block = self.pose_prior_block(state, k);
            self.accumulate(b, block.as_ref(), weights[b], &mut blocks);
            b += 1;
        }
        // An anchored point's two unused dimensions: inert, so their step is exactly zero.
        for (slot, &dims) in self.point_dims.iter().enumerate() {
            if dims == 1 {
                blocks.hessian_pp[slot][(1, 1)] = 1.0;
                blocks.hessian_pp[slot][(2, 2)] = 1.0;
            }
        }
        SchurSystem::new(self.reduction(state).compacted(), blocks, DenseCholesky)
    }

    fn retract(&self, state: &BundleState, delta: &DVector<f64>) -> BundleState {
        let reduction = self.reduction(state);
        let full = reduction.expand(&delta.rows(0, reduction.size()).into_owned());
        let mut out = state.clone();
        for (c, pose) in out.poses.iter_mut().enumerate() {
            let twist: Vector6<f64> = full.fixed_rows::<6>(POSE_DIM * c).into_owned();
            if twist.iter().any(|&v| v != 0.0)
                && let Ok(moved) = se3::compose(&state.poses[c], &se3::exp(&twist))
            {
                *pose = moved;
            }
        }
        let n_intrinsics = self.affine_offset - self.intrinsics_offset;
        for (value, step) in out
            .intrinsics
            .values
            .iter_mut()
            .zip(full.rows(self.intrinsics_offset, n_intrinsics).iter())
        {
            *value += step;
        }
        for (c, affine) in out.depth_affine.iter_mut().enumerate() {
            affine[0] += full[self.affine_offset + 2 * c];
            affine[1] += full[self.affine_offset + 2 * c + 1];
        }
        let mut offset = reduction.size();
        for (p, slot) in self.layout.point_slot.iter().enumerate() {
            if slot.is_none() {
                continue;
            }
            if self.layout.anchored[p] {
                out.inverse_depth[p] += delta[offset];
                offset += 1;
            } else {
                out.points[p] += delta.fixed_rows::<3>(offset);
                offset += 3;
            }
        }
        out
    }
}

/// The a-posteriori variance factor `Σ(w s) / (m - n)` at a state: the blocks' squared norms
/// under their final IRLS weights, the numerator that pairs with the information `JᵀWJ` the
/// deviations come from (`2 cost` under the trivial loss).
///
/// # Arguments
///
/// * `bundle` - The problem.
/// * `state` - The state.
/// * `solver` - The settings whose per-block loss gives the weights.
///
/// # Returns
///
/// The factor, or NaN without redundancy (m <= n).
#[allow(
    clippy::cast_precision_loss,
    reason = "a residual count is far below 2^52"
)]
fn variance_factor(bundle: &Bundle<'_>, state: &BundleState, solver: &nls::Options) -> f64 {
    let rows = bundle.residual_rows();
    let free = bundle.free_size(state);
    if rows <= free {
        return f64::NAN;
    }
    let squared = nls::Problem::squared_norms(bundle, state);
    let weights = nls::evaluate_blocks(solver, &squared).weights;
    weights.dot(&squared) / (rows - free) as f64
}

/// The free intrinsics' marginal standard deviations at a state (NaN where fixed).
///
/// # Arguments
///
/// * `bundle` - The problem.
/// * `state` - The state.
/// * `solver` - The settings whose per-block loss weights the blocks.
/// * `scaling` - The factor the variances are multiplied by (the variance factor, or 1).
///
/// # Returns
///
/// The deviations.
fn intrinsics_std(
    bundle: &Bundle<'_>,
    state: &BundleState,
    solver: &nls::Options,
    scaling: f64,
) -> Intrinsics {
    let mut out = Intrinsics {
        values: vec![f64::NAN; state.intrinsics.values.len()],
        params: state.intrinsics.params,
    };
    let (entries, positions) = bundle.intrinsics_positions();
    if entries.is_empty() {
        return out;
    }
    let squared = nls::Problem::squared_norms(bundle, state);
    let weights = nls::evaluate_blocks(solver, &squared).weights;
    let system = nls::Problem::linearize(bundle, state, &weights);
    for (&entry, variance) in entries.iter().zip(system.camera_variances(&positions)) {
        out.values[entry] = (scaling * variance).sqrt();
    }
    out
}

/// Counts the groups whose focal length is uncertain by more than `ratio` of itself.
///
/// # Arguments
///
/// * `model` - The camera model (radial has one focal length; pinhole and radtan two).
/// * `std_dev` - The intrinsics' standard deviations (NaN where fixed).
/// * `intrinsics` - The intrinsics.
/// * `ratio` - The relative deviation above which a focal length is unobservable.
///
/// # Returns
///
/// The count.
fn unobservable_focal(
    model: Model,
    std_dev: &Intrinsics,
    intrinsics: &Intrinsics,
    ratio: f64,
) -> usize {
    let n_focal = if model == Model::Radial { 1 } else { 2 };
    (0..std_dev.groups())
        .filter(|&g| (0..n_focal).any(|k| std_dev.group(g)[k] > ratio * intrinsics.group(g)[k]))
        .count()
}

/// Every point in the world frame at a state, as the solution reports it.
///
/// # Arguments
///
/// * `bundle` - The problem.
/// * `state` - The state.
///
/// # Returns
///
/// The points: plain ones as they are, anchored ones placed on their rays. An anchored point
/// with no world position (rho <= 0, or a ray the lens cannot invert) is a NaN row, as in
/// the Python reference.
fn final_points(bundle: &Bundle<'_>, state: &BundleState) -> Vec<Vector3<f64>> {
    let world = bundle.world_points(state);
    state
        .points
        .iter()
        .enumerate()
        .map(|(p, point)| {
            if bundle.layout.anchored[p] {
                world[p].unwrap_or_else(|| Vector3::repeat(f64::NAN))
            } else {
                *point
            }
        })
        .collect()
}

/// Solves a checked problem over its layout.
///
/// # Arguments
///
/// * `problem` - The problem (already checked and centred).
/// * `options` - The settings (already checked).
/// * `layout` - The problem's layout.
///
/// # Returns
///
/// The solution.
///
/// # Errors
///
/// `NotConverged` in strict mode.
pub fn adjust(problem: &Problem, options: &Options, layout: &Layout) -> Result<Solution> {
    let bundle = Bundle::new(problem, layout);
    let mut settings = options.solver.clone();
    settings.block_loss = nls::BlockLoss {
        losses: vec![
            options.solver.loss,
            options.depth_loss,
            nls::Loss::default(),
        ],
        index: bundle.loss_index(),
    };
    let solution = nls::solve(&bundle, bundle.start(), &settings)?;
    let state = solution.state;
    let start_poses: Vec<Matrix4<f64>> = problem.poses.iter().map(invert).collect();
    let poses: Vec<Matrix4<f64>> = state
        .poses
        .iter()
        .zip(&start_poses)
        .zip(&problem.poses)
        .map(|((now, start), original)| {
            // Bit-exact, not a round trip through the inverse.
            if now == start { *original } else { invert(now) }
        })
        .collect();
    let factor = variance_factor(&bundle, &state, &settings);
    let scaling = if options.scale_deviations && factor.is_finite() {
        factor
    } else {
        1.0
    };
    let std_dev = intrinsics_std(&bundle, &state, &settings, scaling);
    let unobservable_focal = unobservable_focal(
        problem.model,
        &std_dev,
        &state.intrinsics,
        options.focal_std_ratio,
    );
    let points = final_points(&bundle, &state);
    let counts = &layout.counts;
    Ok(Solution {
        poses,
        intrinsics: state.intrinsics,
        points,
        inverse_depth: state.inverse_depth,
        depth_affine: state.depth_affine,
        intrinsics_std: std_dev,
        summary: Summary {
            solve: solution.summary,
            gauge: counts.gauge.unwrap_or(Gauge::None),
            gauge_pose: counts.gauge_pose,
            confined_pose: counts.confined_pose,
            free_gauge: counts.free_gauge,
            dropped_points: counts.dropped_points,
            dropped_observations: counts.dropped_observations,
            unobserved_poses: counts.unobserved_poses,
            unobserved_intrinsics: counts.unobserved_intrinsics,
            unobserved_depth_affines: counts.unobserved_depth_affines,
            unobservable_focal,
            variance_factor: factor,
        },
    })
}
