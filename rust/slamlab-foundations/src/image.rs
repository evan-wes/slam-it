//! Grayscale images in fixed point: the Rust core's mirror of `slamlab.foundations.image`.
//!
//! Colour conversion, a separable integer blur, bilinear resizing, pyramids and remapping,
//! all in integer arithmetic so the three languages' images are bit-identical (the Python
//! module's docstring defines each operation and its rounding). Pixel `(x, y)` is
//! `pixels[y * cols + x]`; the centre of the top-left pixel is `(0, 0)`.

use nalgebra::Vector2;

use crate::error::{ErrorKind, Result, check_range, fail, fail_setting};

/// BT.601's red, green and blue weights over `2^GRAY_BITS` (OpenCV's `cvtColor`).
pub const GRAY_WEIGHTS: [i64; 3] = [9798, 19235, 3735];
/// The fixed-point bits of [`GRAY_WEIGHTS`].
pub const GRAY_BITS: u32 = 15;
/// The fixed-point bits of a bilinear resize's weights.
pub const RESIZE_BITS: u32 = 11;
/// The fixed-point bits of a remap's coordinates: a 32nd of a pixel.
pub const REMAP_BITS: u32 = 5;
/// A blur kernel's weights sum to `2^b` with `b` at most this.
pub const MAX_KERNEL_BITS: u32 = 15;
/// The largest image side, in pixels (`slamlab.foundations.arrays.MAX_IMAGE_SIDE`): the
/// integer arithmetic's intermediate values then fit 64 bits.
pub const MAX_SIDE: usize = 1 << 20;

/// Checks a pyramid's scale factor (`slamlab.foundations.image.as_scale_factor`).
///
/// # Arguments
///
/// * `scale_factor` - The factor.
/// * `name` - Its name, for the message.
///
/// # Errors
///
/// `InvalidInput` unless it is finite and above 1.
pub fn check_scale_factor(scale_factor: f64, name: &str) -> Result<()> {
    if scale_factor.is_finite() && scale_factor > 1.0 {
        Ok(())
    } else {
        fail_setting(name, "must be finite and above 1", scale_factor)
    }
}

/// A pyramid-level position's coordinate in the full image, along one axis
/// (`slamlab.foundations.image.full_image_coordinate`): the resizes' aligned-centre maps
/// composed, `((2 x + 1) n_0 - n_l) / (2 n_l)`, computed in that order. For an integer `x`
/// only the division rounds, and level 0 maps to itself.
///
/// # Arguments
///
/// * `x` - The position on the level, in its pixels.
/// * `level_size` - The level's size along the axis, `n_l`.
/// * `size` - The full image's size along it, `n_0`.
///
/// # Returns
///
/// The full-image coordinate.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "sizes are at most MAX_SIDE, exact in f64"
)]
pub fn full_image_coordinate(x: f64, level_size: usize, size: usize) -> f64 {
    let level = level_size as f64;
    ((2.0 * x + 1.0) * size as f64 - level) / (2.0 * level)
}

/// A full-image position's coordinate on a pyramid level, along one axis
/// (`slamlab.foundations.image.level_coordinate`): [`full_image_coordinate`]'s inverse,
/// `((2 u + 1) n_l - n_0) / (2 n_0)`, computed in that order. Level 0 maps to itself.
///
/// # Arguments
///
/// * `u` - The position in the full image, in its pixels.
/// * `level_size` - The level's size along the axis, `n_l`.
/// * `size` - The full image's size along it, `n_0`.
///
/// # Returns
///
/// The level coordinate.
#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "sizes are at most MAX_SIDE, exact in f64"
)]
pub fn level_coordinate(u: f64, level_size: usize, size: usize) -> f64 {
    let full = size as f64;
    ((2.0 * u + 1.0) * level_size as f64 - full) / (2.0 * full)
}

/// A grayscale image, row-major.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Image {
    /// The number of rows (the height), at least 1.
    pub rows: usize,
    /// The number of columns (the width), at least 1.
    pub cols: usize,
    /// The intensities, `rows * cols` of them, row by row.
    pub pixels: Vec<u8>,
}

impl Image {
    /// Creates an image of zeros.
    ///
    /// # Arguments
    ///
    /// * `rows` - Its rows.
    /// * `cols` - Its columns.
    ///
    /// # Returns
    ///
    /// The image.
    fn blank(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            pixels: vec![0; rows * cols],
        }
    }

    /// Reads one pixel.
    ///
    /// # Arguments
    ///
    /// * `row` - Its row, in `[0, rows)`.
    /// * `col` - Its column, in `[0, cols)`.
    ///
    /// # Returns
    ///
    /// Its intensity.
    ///
    /// # Panics
    ///
    /// If the pixel is outside the image.
    #[must_use]
    pub fn at(&self, row: usize, col: usize) -> u8 {
        self.pixels[row * self.cols + col]
    }
}

/// Checks an image's shape and its number of values.
///
/// # Arguments
///
/// * `rows` - Its rows, from 1 to [`MAX_SIDE`].
/// * `cols` - Its columns, from 1 to [`MAX_SIDE`].
/// * `channels` - Its values per pixel.
/// * `len` - Its number of values, `rows * cols * channels`.
/// * `name` - Its name, for the message.
///
/// # Errors
///
/// `InvalidInput` for a side out of range or another number of values.
fn check_values(rows: usize, cols: usize, channels: usize, len: usize, name: &str) -> Result<()> {
    if !(1..=MAX_SIDE).contains(&rows) || !(1..=MAX_SIDE).contains(&cols) {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: sides must be from 1 to {MAX_SIDE}, got {rows}x{cols}"),
        );
    }
    if len != rows * cols * channels {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{name}: expected {} values, got {len}",
                rows * cols * channels
            ),
        );
    }
    Ok(())
}

/// Checks an image: sides from 1 to [`MAX_SIDE`], and `rows * cols` pixels.
///
/// # Arguments
///
/// * `image` - The image.
/// * `name` - The argument's name, for the message.
///
/// # Errors
///
/// `InvalidInput` if either fails.
pub fn check_image(image: &Image, name: &str) -> Result<()> {
    check_values(image.rows, image.cols, 1, image.pixels.len(), name)
}

/// Checks an output side or a count: from 1 to [`MAX_SIDE`].
///
/// # Arguments
///
/// * `value` - The value.
/// * `name` - Its name, for the message.
///
/// # Errors
///
/// `InvalidInput` if it is out of range.
fn check_side(value: usize, name: &str) -> Result<()> {
    check_range(name, value, 1, MAX_SIDE)
}

/// Checks a blur kernel and finds `b` with its weights summing to `2^b`.
///
/// # Arguments
///
/// * `kernel` - The weights.
///
/// # Returns
///
/// `b`, from 0 to [`MAX_KERNEL_BITS`].
///
/// # Errors
///
/// `InvalidInput` for an even count, or a sum that is not such a power of two.
pub fn kernel_bits(kernel: &[u32]) -> Result<u32> {
    if kernel.len() % 2 != 1 {
        return fail(
            ErrorKind::InvalidInput,
            "kernel: must be an odd number of non-negative integers",
        );
    }
    let total: u64 = kernel.iter().map(|&w| u64::from(w)).sum();
    if total.is_power_of_two() && total.trailing_zeros() <= MAX_KERNEL_BITS {
        Ok(total.trailing_zeros())
    } else {
        fail(
            ErrorKind::InvalidInput,
            format!("kernel: must sum to a power of two up to 2^{MAX_KERNEL_BITS}, got {total}"),
        )
    }
}

/// Rounds a fixed-point value half up and drops its fraction bits.
///
/// # Arguments
///
/// * `value` - The value, from 0 to `255 * 2^bits`.
/// * `bits` - Its fraction bits.
///
/// # Returns
///
/// `round(value / 2^bits)`, as an intensity.
fn round_shift(value: i64, bits: u32) -> u8 {
    let half = (1_i64 << bits) >> 1;
    u8::try_from((value + half) >> bits).expect("a weighted mean of intensities fits a u8")
}

/// Maps an index outside `[0, size)` back inside, reflecting without repeating the edge
/// (OpenCV's `BORDER_REFLECT_101`).
///
/// # Arguments
///
/// * `index` - The index.
/// * `size` - The axis' length, at least 1.
///
/// # Returns
///
/// The reflected index.
fn reflect101(index: i64, size: usize) -> usize {
    if size == 1 {
        return 0;
    }
    let size = to_i64(size);
    let period = 2 * size - 2;
    let wrapped = index.rem_euclid(period);
    to_usize(if wrapped >= size {
        period - wrapped
    } else {
        wrapped
    })
}

/// Converts a length or index to `i64` (every side is at most [`MAX_SIDE`]).
///
/// # Arguments
///
/// * `value` - The value.
///
/// # Returns
///
/// It as an `i64`.
fn to_i64(value: usize) -> i64 {
    i64::try_from(value).expect("image sides fit an i64")
}

/// Converts a non-negative index to `usize`.
///
/// # Arguments
///
/// * `value` - The value, non-negative.
///
/// # Returns
///
/// It as a `usize`.
fn to_usize(value: i64) -> usize {
    usize::try_from(value).expect("a non-negative index")
}

/// Where one destination index of a resized axis samples its source.
#[derive(Debug, Clone, Copy)]
struct AxisSample {
    /// The first source index.
    first: usize,
    /// The second source index (the first's neighbour, clamped).
    second: usize,
    /// The second index's weight, over `2^RESIZE_BITS`.
    fraction: i64,
}

/// Sample positions along one axis: destination `d` samples
/// `s = ((2d + 1) n_in - n_out) / (2 n_out)`, clamped to `[0, n_in - 1]`, its fraction
/// rounded half up to [`RESIZE_BITS`] bits, all in integers.
///
/// # Arguments
///
/// * `n_in` - The source axis' length.
/// * `n_out` - The destination axis' length.
///
/// # Returns
///
/// One sample per destination index.
fn axis_samples(n_in: usize, n_out: usize) -> Vec<AxisSample> {
    let one = 1_i64 << RESIZE_BITS;
    let (n_in, n_out) = (to_i64(n_in), to_i64(n_out));
    let denominator = 2 * n_out;
    (0..n_out)
        .map(|d| {
            let numerator = (2 * d + 1) * n_in - n_out;
            let (mut whole, mut fraction) = (0, 0);
            if numerator > 0 {
                // s <= 0 clamps to the first pixel
                whole = numerator / denominator;
                let remainder = numerator - whole * denominator;
                fraction = (2 * remainder * one + denominator) / (2 * denominator);
                if fraction == one {
                    whole += 1;
                    fraction = 0;
                }
            }
            if whole >= n_in - 1 {
                // s >= n_in - 1 clamps to the last pixel
                whole = n_in - 1;
                fraction = 0;
            }
            AxisSample {
                first: to_usize(whole),
                second: to_usize((whole + 1).min(n_in - 1)),
                fraction,
            }
        })
        .collect()
}

/// Resizes a checked image to a checked shape.
///
/// # Arguments
///
/// * `image` - The image.
/// * `rows` - The output's rows.
/// * `cols` - The output's columns.
///
/// # Returns
///
/// The resized image.
fn resize_checked(image: &Image, rows: usize, cols: usize) -> Image {
    let one = 1_i32 << RESIZE_BITS;
    let xs = axis_samples(image.cols, cols);
    let ys = axis_samples(image.rows, rows);
    // Every sum fits 32 bits: across a row at most 255 * 2^11, down a column 255 * 2^22.
    let fraction =
        |sample: &AxisSample| i32::try_from(sample.fraction).expect("an 11-bit fraction");
    let mut across = vec![0_i32; image.rows * cols];
    for (row, sums) in image
        .pixels
        .chunks_exact(image.cols)
        .zip(across.chunks_exact_mut(cols))
    {
        for (sum, x) in sums.iter_mut().zip(&xs) {
            let fx = fraction(x);
            *sum = i32::from(row[x.first]) * (one - fx) + i32::from(row[x.second]) * fx;
        }
    }
    let shift = 2 * RESIZE_BITS;
    let half = 1_i32 << (shift - 1);
    let mut out = Image::blank(rows, cols);
    for (line, y) in out.pixels.chunks_exact_mut(cols).zip(&ys) {
        let fy = fraction(y);
        let top = &across[y.first * cols..(y.first + 1) * cols];
        let bottom = &across[y.second * cols..(y.second + 1) * cols];
        for ((pixel, &a), &b) in line.iter_mut().zip(top).zip(bottom) {
            let total = a * (one - fy) + b * fy;
            *pixel = u8::try_from((total + half) >> shift).unwrap_or(u8::MAX);
        }
    }
    out
}

/// Each pyramid level's `(rows, cols)`: `round(n / s^l)`, half up, `s^l` by repeated
/// multiplication.
///
/// # Arguments
///
/// * `rows` - Level 0's rows.
/// * `cols` - Level 0's columns.
/// * `levels` - The number of levels, at least 1.
/// * `scale_factor` - Each level's downscale from the previous one, above 1.
///
/// # Returns
///
/// One shape per level.
///
/// # Errors
///
/// `InvalidInput` if a setting is out of range or a level would be empty.
#[allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "sides are at most 2^20, exact in f64; the rounded quotient is non-negative"
)]
pub fn level_shapes(
    rows: usize,
    cols: usize,
    levels: usize,
    scale_factor: f64,
) -> Result<Vec<(usize, usize)>> {
    check_side(levels, "levels")?;
    check_scale_factor(scale_factor, "scale_factor")?;
    let mut shapes = vec![(rows, cols)];
    let mut scale = 1.0;
    for level in 1..levels {
        scale *= scale_factor;
        let shape = (
            (rows as f64 / scale + 0.5).floor() as usize,
            (cols as f64 / scale + 0.5).floor() as usize,
        );
        if shape.0 < 1 || shape.1 < 1 {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "pyramid: level {level} of a {rows}x{cols} image at scale factor \
                     {scale_factor} would be empty"
                ),
            );
        }
        shapes.push(shape);
    }
    Ok(shapes)
}

/// Converts an RGB image to gray: `Y = 0.299 R + 0.587 G + 0.114 B` in 15-bit integers.
///
/// # Arguments
///
/// * `rows` - The image's rows.
/// * `cols` - The image's columns.
/// * `rgb` - Interleaved red, green and blue, `rows * cols * 3` values, row by row.
///
/// # Returns
///
/// The gray image, rounded half up.
///
/// # Errors
///
/// `InvalidInput` for a bad shape.
pub fn to_gray(rows: usize, cols: usize, rgb: &[u8]) -> Result<Image> {
    check_values(rows, cols, 3, rgb.len(), "rgb")?;
    let [r, g, b] = GRAY_WEIGHTS;
    let pixels = rgb
        .as_chunks::<3>()
        .0
        .iter()
        .map(|p| {
            let total = r * i64::from(p[0]) + g * i64::from(p[1]) + b * i64::from(p[2]);
            round_shift(total, GRAY_BITS)
        })
        .collect();
    Ok(Image { rows, cols, pixels })
}

/// Filters separably with integer weights summing to a power of two, borders reflected
/// without repeating the edge.
///
/// # Arguments
///
/// * `image` - The image.
/// * `kernel` - An odd number of weights, applied along rows, then along columns.
///
/// # Returns
///
/// The filtered image, rounded half up.
///
/// # Errors
///
/// `InvalidInput` for a bad image or kernel.
pub fn blur(image: &Image, kernel: &[u32]) -> Result<Image> {
    check_image(image, "image")?;
    let bits = kernel_bits(kernel)?;
    let radius = to_i64(kernel.len() / 2);
    let (rows, cols) = (image.rows, image.cols);
    let column_index: Vec<usize> = (-radius..to_i64(cols) + radius)
        .map(|c| reflect101(c, cols))
        .collect();
    let row_index: Vec<usize> = (-radius..to_i64(rows) + radius)
        .map(|r| reflect101(r, rows))
        .collect();
    // The horizontal sums reach 255 * 2^bits, bits at most MAX_KERNEL_BITS: 32 bits hold them.
    // The interior reads its neighbours directly, a tap at a time, so the loops vectorize; the
    // border reflects. Integer sums are exact in any order.
    let mut across = vec![0_u32; rows * cols];
    let reach = kernel.len() / 2;
    let first = reach.min(cols);
    let last = first.max(cols.saturating_sub(reach));
    for (row, sums) in image
        .pixels
        .chunks_exact(cols)
        .zip(across.chunks_exact_mut(cols))
    {
        if last > first {
            for (k, &weight) in kernel.iter().enumerate() {
                for (sum, &value) in sums[first..last].iter_mut().zip(&row[k..]) {
                    *sum += weight * u32::from(value);
                }
            }
        }
        for c in (0..first).chain(last..cols) {
            sums[c] = kernel
                .iter()
                .zip(&column_index[c..])
                .map(|(&weight, &source)| weight * u32::from(row[source]))
                .sum();
        }
    }
    // The vertical sums reach 255 * 2^(2 bits): 32 bits hold them up to 12-bit kernels
    // (ORB's), which lets the loop vectorize; 64 bits above.
    let mut out = Image::blank(rows, cols);
    if 2 * bits + 8 <= 32 {
        blur_columns::<u32>(&across, kernel, &row_index, bits, &mut out);
    } else {
        blur_columns::<u64>(&across, kernel, &row_index, bits, &mut out);
    }
    Ok(out)
}

/// A separable blur's vertical pass, rounded to intensities.
///
/// # Arguments
///
/// * `across` - The horizontal pass's sums, row-major, `out`'s shape.
/// * `kernel` - The weights.
/// * `row_index` - Each padded row's reflected row.
/// * `bits` - The kernel's bits.
/// * `out` - The blurred image, its pixels written.
fn blur_columns<T>(across: &[u32], kernel: &[u32], row_index: &[usize], bits: u32, out: &mut Image)
where
    T: Copy
        + From<u32>
        + std::ops::Add<Output = T>
        + std::ops::AddAssign
        + std::ops::Mul<Output = T>
        + std::ops::Shl<u32, Output = T>
        + std::ops::Shr<u32, Output = T>,
    u8: TryFrom<T>,
{
    let cols = out.cols;
    let zero = T::from(0);
    let half = (T::from(1) << (2 * bits)) >> 1; // 0 for a 0-bit kernel, as round_shift's
    let mut sums = vec![zero; cols];
    for (r, row) in out.pixels.chunks_exact_mut(cols).enumerate() {
        sums.fill(zero);
        for (&weight, &source) in kernel.iter().zip(&row_index[r..]) {
            let weight = T::from(weight);
            let line = &across[source * cols..(source + 1) * cols];
            for (sum, &value) in sums.iter_mut().zip(line) {
                *sum += weight * T::from(value);
            }
        }
        // A weighted mean of intensities: it fits a byte (the fallback never applies).
        for (pixel, &sum) in row.iter_mut().zip(&sums) {
            *pixel = u8::try_from((sum + half) >> (2 * bits)).unwrap_or(u8::MAX);
        }
    }
}

/// Resizes bilinearly, pixel centres aligned, in 11-bit fixed point.
///
/// # Arguments
///
/// * `image` - The image.
/// * `rows` - The output's rows, from 1 to [`MAX_SIDE`].
/// * `cols` - The output's columns, from 1 to [`MAX_SIDE`].
///
/// # Returns
///
/// The resized image.
///
/// # Errors
///
/// `InvalidInput` for a bad image or shape.
pub fn resize(image: &Image, rows: usize, cols: usize) -> Result<Image> {
    check_image(image, "image")?;
    check_side(rows, "rows")?;
    check_side(cols, "cols")?;
    Ok(resize_checked(image, rows, cols))
}

/// The image and its successively downscaled levels, each resized from the previous.
///
/// # Arguments
///
/// * `image` - Level 0.
/// * `levels` - The number of levels, at least 1.
/// * `scale_factor` - Each level's downscale, above 1.
///
/// # Returns
///
/// The levels.
///
/// # Errors
///
/// `InvalidInput` for a bad image, or as [`level_shapes`].
pub fn pyramid(image: &Image, levels: usize, scale_factor: f64) -> Result<Vec<Image>> {
    check_image(image, "image")?;
    let shapes = level_shapes(image.rows, image.cols, levels, scale_factor)?;
    let mut out = Vec::with_capacity(shapes.len());
    let mut level = image.clone();
    for &(rows, cols) in &shapes[1..] {
        let next = resize_checked(&level, rows, cols);
        out.push(std::mem::replace(&mut level, next));
    }
    out.push(level);
    Ok(out)
}

/// Checks a pyramid already built (`slamlab.foundations.image.as_pyramid`): `count` images,
/// each a valid image of the shape [`level_shapes`] gives from level 0's.
///
/// # Arguments
///
/// * `levels` - The levels, level 0 first.
/// * `count` - The number of levels expected.
/// * `scale_factor` - The pyramid's scale factor.
/// * `name` - The argument's name, for the message.
///
/// # Errors
///
/// `InvalidInput` for another number of levels, a malformed level, one of another shape, or
/// settings [`level_shapes`] refuses.
pub fn check_pyramid(levels: &[Image], count: usize, scale_factor: f64, name: &str) -> Result<()> {
    if levels.len() != count {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected {count} levels, got {}", levels.len()),
        );
    }
    for level in levels {
        check_image(level, name)?;
    }
    let Some(first) = levels.first() else {
        return fail(
            ErrorKind::InvalidInput,
            format!("{name}: expected at least one level"),
        );
    };
    let shapes = level_shapes(first.rows, first.cols, count, scale_factor)?;
    for (k, (level, &(rows, cols))) in levels.iter().zip(&shapes).enumerate() {
        if (level.rows, level.cols) != (rows, cols) {
            return fail(
                ErrorKind::InvalidInput,
                format!(
                    "{name}: level {k} is {}x{}, expected {rows}x{cols}",
                    level.rows, level.cols
                ),
            );
        }
    }
    Ok(())
}

/// Samples an image bilinearly at a map of coordinates, rounded to a 32nd of a pixel;
/// neighbours outside the image count as 0.
///
/// # Arguments
///
/// * `image` - The image.
/// * `coordinates` - Each output pixel's source `(x, y)`, row by row (`rows * cols` of them).
/// * `rows` - The output's rows.
/// * `cols` - The output's columns.
///
/// # Returns
///
/// The sampled image.
///
/// # Errors
///
/// `InvalidInput` for a bad image or shape, or a non-finite coordinate.
#[allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    reason = "coordinates are clamped to a few pixels past the image, so the floor fits an i64"
)]
pub fn remap(
    image: &Image,
    coordinates: &[Vector2<f64>],
    rows: usize,
    cols: usize,
) -> Result<Image> {
    check_image(image, "image")?;
    check_side(rows, "rows")?;
    check_side(cols, "cols")?;
    if coordinates.len() != rows * cols {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "coordinates: expected rows * cols coordinates, got {}",
                coordinates.len()
            ),
        );
    }
    if coordinates
        .iter()
        .any(|p| !(p.x.is_finite() && p.y.is_finite()))
    {
        return fail(
            ErrorKind::InvalidInput,
            "coordinates: expected finite values",
        );
    }
    let one = 1_i64 << REMAP_BITS;
    let (height, width) = (to_i64(image.rows), to_i64(image.cols));
    let sample = |r: i64, c: i64| -> i64 {
        if (0..height).contains(&r) && (0..width).contains(&c) {
            i64::from(image.at(to_usize(r), to_usize(c)))
        } else {
            0
        }
    };
    let pixels = coordinates
        .iter()
        .map(|p| {
            // Clamping keeps the fixed-point values small; past it every neighbour is outside
            // anyway. x * 2^bits is exact, so the rounding is the same whether or not the add
            // is fused.
            let x = p.x.clamp(-2.0, image.cols as f64 + 1.0);
            let y = p.y.clamp(-2.0, image.rows as f64 + 1.0);
            let qx = (x * one as f64 + 0.5).floor() as i64;
            let qy = (y * one as f64 + 0.5).floor() as i64;
            let (ix, fx) = (qx >> REMAP_BITS, qx & (one - 1));
            let (iy, fy) = (qy >> REMAP_BITS, qy & (one - 1));
            let total = sample(iy, ix) * (one - fx) * (one - fy)
                + sample(iy, ix + 1) * fx * (one - fy)
                + sample(iy + 1, ix) * (one - fx) * fy
                + sample(iy + 1, ix + 1) * fx * fy;
            round_shift(total, 2 * REMAP_BITS)
        })
        .collect();
    Ok(Image { rows, cols, pixels })
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/image.json`, which
    //! the Python and C++ tests share, and the cores' own input checks.

    use super::*;
    use serde_json::Value;

    /// The two coordinate maps are inverses, level 0 maps to itself, and their composition is
    /// the level-to-level map stereo matching uses.
    #[test]
    #[allow(clippy::cast_precision_loss, reason = "small integers, exact in f64")]
    fn coordinate_maps_invert_and_compose() {
        assert_eq!(level_coordinate(7.0, 640, 640).to_bits(), 7.0_f64.to_bits());
        let levels = [640_usize, 533, 444, 370, 309, 257, 214, 179];
        for a in levels {
            for b in levels {
                for x in (0..a).step_by(7) {
                    let u = full_image_coordinate(x as f64, a, 640);
                    assert!((level_coordinate(u, a, 640) - x as f64).abs() < 1e-12);
                    let on_b = ((2 * x + 1) * b) as f64 - a as f64;
                    let on_b = on_b / (2 * a) as f64;
                    assert!((level_coordinate(u, b, 640) - on_b).abs() < 1e-12);
                }
            }
        }
    }

    /// Reads the shared fixture.
    fn fixture() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/image.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        serde_json::from_str(&text).expect("the fixture is valid JSON")
    }

    /// Reads a non-negative integer.
    fn count(value: &Value) -> usize {
        usize::try_from(value.as_u64().expect("a non-negative integer")).expect("fits a usize")
    }

    /// Reads a nested list of intensities as an image.
    fn image(value: &Value) -> Image {
        let rows: Vec<&Vec<Value>> = value
            .as_array()
            .expect("a list of rows")
            .iter()
            .map(|row| row.as_array().expect("a row"))
            .collect();
        let pixels = rows
            .iter()
            .flat_map(|row| row.iter())
            .map(|v| u8::try_from(count(v)).expect("an intensity"))
            .collect();
        Image {
            rows: rows.len(),
            cols: rows[0].len(),
            pixels,
        }
    }

    /// Every fixture case of a function, as its inputs and expected value.
    fn cases(name: &str) -> Vec<(Vec<Value>, Value)> {
        fixture()["functions"][name]["cases"]
            .as_array()
            .expect("cases")
            .iter()
            .map(|case| {
                let input = case["input"].as_array().expect("an argument list").clone();
                (input, case["expected"].clone())
            })
            .collect()
    }

    /// `to_gray` matches the fixture bit for bit.
    #[test]
    fn to_gray_matches_the_fixture() {
        for (input, expected) in cases("to_gray") {
            let rows = input[0].as_array().expect("rows");
            let cols = rows[0].as_array().expect("a row").len();
            let rgb: Vec<u8> = rows
                .iter()
                .flat_map(|row| row.as_array().expect("a row").iter())
                .flat_map(|p| p.as_array().expect("a pixel").iter())
                .map(|v| u8::try_from(count(v)).expect("an intensity"))
                .collect();
            assert_eq!(to_gray(rows.len(), cols, &rgb).unwrap(), image(&expected));
        }
    }

    /// `blur` matches the fixture bit for bit.
    #[test]
    fn blur_matches_the_fixture() {
        for (input, expected) in cases("blur") {
            let kernel: Vec<u32> = input[1]
                .as_array()
                .expect("weights")
                .iter()
                .map(|w| u32::try_from(count(w)).expect("a weight"))
                .collect();
            assert_eq!(blur(&image(&input[0]), &kernel).unwrap(), image(&expected));
        }
    }

    /// `resize` matches the fixture bit for bit.
    #[test]
    fn resize_matches_the_fixture() {
        for (input, expected) in cases("resize") {
            let got = resize(&image(&input[0]), count(&input[1]), count(&input[2])).unwrap();
            assert_eq!(got, image(&expected));
        }
    }

    /// `pyramid` matches the fixture bit for bit.
    #[test]
    fn pyramid_matches_the_fixture() {
        for (input, expected) in cases("pyramid") {
            let scale = input[2].as_f64().expect("a scale factor");
            let got = pyramid(&image(&input[0]), count(&input[1]), scale).unwrap();
            let expected: Vec<Image> = expected
                .as_array()
                .expect("levels")
                .iter()
                .map(image)
                .collect();
            assert_eq!(got, expected);
        }
    }

    /// `remap` matches the fixture bit for bit.
    #[test]
    fn remap_matches_the_fixture() {
        for (input, expected) in cases("remap") {
            let map = input[1].as_array().expect("a map");
            let cols = map[0].as_array().expect("a row").len();
            let coordinates: Vec<Vector2<f64>> = map
                .iter()
                .flat_map(|row| row.as_array().expect("a row").iter())
                .map(|p| Vector2::new(p[0].as_f64().unwrap(), p[1].as_f64().unwrap()))
                .collect();
            let got = remap(&image(&input[0]), &coordinates, map.len(), cols).unwrap();
            assert_eq!(got, image(&expected));
        }
    }

    /// A blank image of the given shape, for the input checks.
    fn gray(rows: usize, cols: usize) -> Image {
        Image::blank(rows, cols)
    }

    /// Bad images, kernels, shapes and settings are `InvalidInput`.
    #[test]
    fn rejects_invalid_input() {
        let invalid = |result: Result<Image>| {
            assert_eq!(result.unwrap_err().kind, ErrorKind::InvalidInput);
        };
        let mut short = gray(3, 4);
        short.pixels.pop();
        invalid(blur(&short, &[1]));
        invalid(blur(&gray(0, 4), &[1]));
        invalid(blur(&gray(MAX_SIDE + 1, 1), &[1]));
        invalid(blur(&gray(3, 4), &[1, 1]));
        invalid(blur(&gray(3, 4), &[1, 1, 1]));
        invalid(blur(&gray(3, 4), &[1 << 16]));
        invalid(resize(&gray(3, 4), 0, 4));
        invalid(resize(&gray(3, 4), 3, MAX_SIDE + 1));
        invalid(to_gray(2, 2, &[0; 11]));
        invalid(remap(&gray(3, 4), &[Vector2::new(f64::NAN, 0.0)], 1, 1));
        invalid(remap(&gray(3, 4), &[Vector2::zeros(); 2], 1, 1));
        for (levels, scale) in [(0, 1.2), (2, 1.0), (2, f64::INFINITY), (3, 4.0)] {
            assert_eq!(
                pyramid(&gray(3, 4), levels, scale).unwrap_err().kind,
                ErrorKind::InvalidInput
            );
        }
    }

    /// Reflection repeats no edge pixel, also far outside and on a one-pixel axis.
    #[test]
    fn reflect101_mirrors_without_the_edge() {
        let got: Vec<usize> = (-5..9).map(|i| reflect101(i, 4)).collect();
        assert_eq!(got, [1, 2, 3, 2, 1, 0, 1, 2, 3, 2, 1, 0, 1, 2]);
        assert_eq!(reflect101(-7, 1), 0);
    }
}
