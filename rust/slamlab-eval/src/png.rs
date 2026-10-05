//! Reading PNG files natively: the Rust side of `slamlab.eval.images`.
//!
//! Decoded by the `png` crate. A file reads the way Python's Pillow-based reader reads it,
//! under the same names: [`read_gray`] gives gray from an 8-bit gray file as stored (without
//! its alpha), or from an 8-bit colour file (RGB, RGBA, or a palette expanded to RGB) through
//! `image::to_gray`; [`read_rgb`] gives an 8-bit file's colour (gray repeated to three
//! channels, alpha dropped); [`read_depth`] a 16-bit single-channel file as stored.
//!
//! Every failure is `InvalidInput`, its message naming the file: a missing or unreadable file
//! ("no such file, or not readable"), one the decoder refuses ("not a readable PNG", a bad
//! checksum included), or the wrong bit depth. Python raises `FileNotFoundError` for a missing
//! file instead; the cores have no I/O error kind.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use png::{BitDepth, ColorType, Decoder, Transformations};
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::image::{self, Image};

/// An 8-bit image as stored: 1 (gray), 2 (gray, alpha), 3 (RGB) or 4 (RGBA) channels.
struct Raster8 {
    /// Rows.
    rows: usize,
    /// Columns.
    cols: usize,
    /// Channels per pixel.
    channels: usize,
    /// The values, row by row, the channels of a pixel together.
    values: Vec<u8>,
}

/// An 8-bit colour image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RgbImage {
    /// Rows.
    pub rows: usize,
    /// Columns.
    pub cols: usize,
    /// The values, row by row, each pixel's red, green and blue together.
    pub values: Vec<u8>,
}

/// A 16-bit single-channel image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DepthImage {
    /// Rows.
    pub rows: usize,
    /// Columns.
    pub cols: usize,
    /// The values, row by row.
    pub values: Vec<u16>,
}

/// A decoded frame: its shape, colour type, bit depth and bytes.
struct Decoded {
    /// Rows.
    rows: usize,
    /// Columns.
    cols: usize,
    /// The colour type after expansion.
    color: ColorType,
    /// The bit depth after expansion.
    depth: BitDepth,
    /// The bytes, row by row (16-bit values big-endian).
    bytes: Vec<u8>,
}

/// Decodes a PNG, palettes and low bit depths expanded.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The decoded frame.
///
/// # Errors
///
/// `InvalidInput` if the file cannot be opened or decoded.
fn decode(path: &Path) -> Result<Decoded> {
    let unreadable = |reason: String| {
        fail::<Decoded>(
            ErrorKind::InvalidInput,
            format!("{}: not a readable PNG ({reason})", path.display()),
        )
    };
    let Ok(file) = File::open(path) else {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: no such file, or not readable", path.display()),
        );
    };
    let mut decoder = Decoder::new(BufReader::new(file));
    decoder.set_transformations(Transformations::EXPAND);
    let mut reader = match decoder.read_info() {
        Ok(reader) => reader,
        Err(error) => return unreadable(error.to_string()),
    };
    let Some(size) = reader.output_buffer_size() else {
        return unreadable("the image is too large".to_owned());
    };
    let mut bytes = vec![0; size];
    let info = match reader.next_frame(&mut bytes) {
        Ok(info) => info,
        Err(error) => return unreadable(error.to_string()),
    };
    bytes.truncate(info.buffer_size());
    Ok(Decoded {
        rows: usize::try_from(info.height).expect("a u32 fits a usize"),
        cols: usize::try_from(info.width).expect("a u32 fits a usize"),
        color: info.color_type,
        depth: info.bit_depth,
        bytes,
    })
}

/// Reads an 8-bit PNG as stored (a palette expanded to RGB or RGBA).
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The raster.
///
/// # Errors
///
/// `InvalidInput` if the file cannot be read or is not 8-bit.
fn read8(path: &Path) -> Result<Raster8> {
    let decoded = decode(path)?;
    if decoded.depth != BitDepth::Eight {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: expected an 8-bit image", path.display()),
        );
    }
    Ok(Raster8 {
        rows: decoded.rows,
        cols: decoded.cols,
        channels: decoded.color.samples(),
        values: decoded.bytes,
    })
}

/// The colour of an 8-bit raster: gray repeated to three channels, alpha dropped.
///
/// # Arguments
///
/// * `raster` - The raster.
///
/// # Returns
///
/// Three values per pixel, red first.
fn rgb_values(raster: &Raster8) -> Vec<u8> {
    raster
        .values
        .chunks_exact(raster.channels)
        .flat_map(|pixel| {
            // Gray (with or without alpha) repeats channel 0; colour takes channels 0 to 2.
            let gray = raster.channels <= 2;
            (0..3).map(move |c| pixel[if gray { 0 } else { c }])
        })
        .collect()
}

/// Reads an 8-bit PNG as colour, as `slamlab.eval.images.read_rgb` does.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The colour image.
///
/// # Errors
///
/// `InvalidInput` if the file cannot be read or is not 8-bit.
pub fn read_rgb(path: &Path) -> Result<RgbImage> {
    let raster = read8(path)?;
    Ok(RgbImage {
        rows: raster.rows,
        cols: raster.cols,
        values: rgb_values(&raster),
    })
}

/// Reads a 16-bit single-channel PNG, as `slamlab.eval.images.read_depth` does.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The image.
///
/// # Errors
///
/// `InvalidInput` if the file cannot be read or is not 16-bit gray.
pub fn read_depth(path: &Path) -> Result<DepthImage> {
    let decoded = decode(path)?;
    if decoded.depth != BitDepth::Sixteen || decoded.color != ColorType::Grayscale {
        return fail(
            ErrorKind::InvalidInput,
            format!("{}: expected a 16-bit single-channel image", path.display()),
        );
    }
    let values = decoded
        .bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&pair| u16::from_be_bytes(pair))
        .collect();
    Ok(DepthImage {
        rows: decoded.rows,
        cols: decoded.cols,
        values,
    })
}

/// Reads an 8-bit PNG as gray, as `slamlab.eval.images.read_gray` does.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Returns
///
/// The gray image.
///
/// # Errors
///
/// `InvalidInput` if the file cannot be read or is not 8-bit.
pub fn read_gray(path: &Path) -> Result<Image> {
    let raster = read8(path)?;
    let channels = raster.channels;
    if channels <= 2 {
        // gray, perhaps with alpha: the gray channel as stored
        let pixels = raster.values.iter().step_by(channels).copied().collect();
        return Ok(Image {
            rows: raster.rows,
            cols: raster.cols,
            pixels,
        });
    }
    image::to_gray(raster.rows, raster.cols, &rgb_values(&raster))
}

#[cfg(test)]
mod tests {
    //! Native tests against the shared fixtures `tests/fixtures/eval/png/` and their expected
    //! values `png.json`, which the Python and C++ tests read too.

    use super::*;
    use serde_json::Value;
    use std::path::PathBuf;

    /// The fixtures' folder.
    fn folder() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/eval")
    }

    /// Reads a nested list of numbers, flattened.
    fn flat(value: &Value) -> Vec<u64> {
        value
            .as_array()
            .expect("rows")
            .iter()
            .flat_map(|row| row.as_array().expect("a row").iter())
            .map(|v| v.as_u64().expect("a non-negative integer"))
            .collect()
    }

    /// Every file reads as its expected values, and the wrong bit depth is refused.
    #[test]
    fn readers_match_the_fixture() {
        let text = std::fs::read_to_string(folder().join("png.json")).expect("the fixture");
        let fixture: Value = serde_json::from_str(&text).expect("valid JSON");
        for (name, expected) in fixture["cases"].as_object().expect("cases") {
            let path = folder().join("png").join(name);
            if let Some(gray) = expected.get("gray") {
                let got = read_gray(&path).unwrap();
                let want: Vec<u8> = flat(gray)
                    .into_iter()
                    .map(|v| u8::try_from(v).unwrap())
                    .collect();
                assert_eq!(got.pixels, want, "{name}");
                let rgb: Vec<u8> = expected["rgb"]
                    .as_array()
                    .expect("rows")
                    .iter()
                    .flat_map(flat)
                    .map(|v| u8::try_from(v).unwrap())
                    .collect();
                assert_eq!(read_rgb(&path).unwrap().values, rgb, "{name}");
                assert!(
                    message(read_depth(&path)).contains("expected a 16-bit"),
                    "{name}"
                );
            } else {
                let got = read_depth(&path).unwrap();
                let want: Vec<u16> = flat(&expected["depth"])
                    .into_iter()
                    .map(|v| u16::try_from(v).unwrap())
                    .collect();
                assert_eq!(got.values, want, "{name}");
                assert!(
                    message(read_gray(&path)).contains("expected an 8-bit"),
                    "{name}"
                );
            }
        }
    }

    /// The failure's message, for an `InvalidInput` error.
    fn message<T: std::fmt::Debug>(result: Result<T>) -> String {
        let error = result.unwrap_err();
        assert_eq!(error.kind, ErrorKind::InvalidInput);
        error.message
    }

    /// Missing, truncated and non-PNG files are refused, each named.
    #[test]
    fn readers_refuse_bad_files() {
        let missing = folder().join("png/missing.png");
        assert!(message(read_gray(&missing)).contains("no such file"));
        assert!(message(read_depth(&missing)).contains("no such file"));
        let whole = std::fs::read(folder().join("png/rgb.png")).expect("the fixture");
        let truncated = std::env::temp_dir().join("slamlab_eval_png_truncated.png");
        std::fs::write(&truncated, &whole[..whole.len() / 2]).expect("a temporary file");
        assert!(message(read_rgb(&truncated)).contains("not a readable PNG"));
        std::fs::remove_file(&truncated).expect("removed");
        assert!(message(read_depth(&folder().join("png.json"))).contains("not a readable PNG"));
    }
}
