//! Reading PNG files natively: the Rust side of `slamlab.eval.images`.
//!
//! Decoded by the `png` crate. A file reads the way Python's Pillow-based reader reads it:
//! gray from an 8-bit gray file as stored (without its alpha), from an 8-bit colour file (RGB,
//! RGBA, or a palette expanded to RGB) through `image::to_gray`; depth from a 16-bit
//! single-channel file as stored.

use std::fs::File;
use std::io::BufReader;
use std::path::Path;

use png::{BitDepth, ColorType, Decoder, Transformations};
use slamlab_foundations::error::{ErrorKind, Result, fail};
use slamlab_foundations::image::{self, Image};

/// An 8-bit image as stored: 1 (gray), 2 (gray, alpha), 3 (RGB) or 4 (RGBA) channels.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Raster8 {
    /// Rows.
    pub rows: usize,
    /// Columns.
    pub cols: usize,
    /// Channels per pixel.
    pub channels: usize,
    /// The values, row by row, the channels of a pixel together.
    pub values: Vec<u8>,
}

/// A 16-bit single-channel image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Raster16 {
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
    let file = match File::open(path) {
        Ok(file) => file,
        Err(error) => return unreadable(error.to_string()),
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
pub fn read8(path: &Path) -> Result<Raster8> {
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

/// Reads a 16-bit single-channel PNG.
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
/// `InvalidInput` if the file cannot be read or is not 16-bit gray.
pub fn read16(path: &Path) -> Result<Raster16> {
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
    Ok(Raster16 {
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
/// `InvalidInput` as [`read8`].
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
    let rgb: Vec<u8> = raster
        .values
        .chunks_exact(channels)
        .flat_map(|pixel| pixel[..3].iter().copied())
        .collect();
    image::to_gray(raster.rows, raster.cols, &rgb)
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
                assert!(read16(&path).is_err(), "{name}");
            } else {
                let got = read16(&path).unwrap();
                let want: Vec<u16> = flat(&expected["depth"])
                    .into_iter()
                    .map(|v| u16::try_from(v).unwrap())
                    .collect();
                assert_eq!(got.values, want, "{name}");
                assert!(read8(&path).is_err(), "{name}");
            }
        }
        let rgb = read8(&folder().join("png/rgb.png")).unwrap();
        assert_eq!((rgb.rows, rgb.cols, rgb.channels), (6, 8, 3));
        assert!(read_gray(&folder().join("png/missing.png")).is_err());
    }
}
