//! A vocabulary's folder, read: the Rust mirror of `slam_it.eval.vocabularies.read` (the
//! module's docstring states the folder's files).
//!
//! The tree is rung 0's (`slam_it_foundations::vocabulary`), which takes arrays, never files;
//! its folder is read here, for the native programs. The header is checked against this crate's
//! format version and this ORB's descriptor version, and the tree as rung 0 checks it. The
//! arrays' SHA-256 are read but not checked: Python checks them (the module docstring's one
//! exception), and they are what a relocalising run records.

use std::path::Path;

use slam_it_foundations::error::{ErrorKind, Result, fail};
use slam_it_foundations::orb::{DESCRIPTOR_BYTES, DESCRIPTOR_VERSION};
use slam_it_foundations::vocabulary::{Vocabulary, check};

/// The folder's format version (`slam_it.eval.vocabularies.VERSION`).
pub const VERSION: i64 = 2;

/// A SHA-256 digest's bytes.
pub const SHA256_BYTES: usize = 32;

/// The tree's arrays, each a `<name>.npy` (`slam_it.foundations.vocabulary.ARRAYS`).
pub const ARRAYS: [&str; 5] = ["descriptors", "parents", "children", "words", "weights"];

/// A folder's header and checksums: what a relocalising run records of its vocabulary.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Header {
    /// The folder's format version.
    pub version: i64,
    /// The version of the descriptor its words were trained on.
    pub descriptor_version: i64,
    /// The most children a node has.
    pub k: i64,
    /// The deepest a leaf lies.
    pub levels: i64,
    /// The number of words.
    pub words: i64,
    /// Each array's SHA-256, in [`ARRAYS`]' order.
    pub sha256: [[u8; SHA256_BYTES]; ARRAYS.len()],
}

/// A folder as read: the vocabulary, and its header.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Folder {
    /// The vocabulary, checked.
    pub vocabulary: Vocabulary,
    /// Its header and checksums.
    pub header: Header,
}

/// A `.npy` array: its type, shape and raw little-endian values.
struct Npy {
    /// numpy's `descr`, e.g. `|u1`, `<i4`, `<i8`, `<f8`.
    descr: String,
    /// The shape.
    shape: Vec<usize>,
    /// The values' bytes, C order.
    data: Vec<u8>,
}

/// The value of a key in a `.npy` header's dict literal, up to its comma or brace.
///
/// # Arguments
///
/// * `header` - The header.
/// * `key` - The key, e.g. `'descr'`.
///
/// # Returns
///
/// The value's text, or `None` if the key is absent.
fn header_value<'a>(header: &'a str, key: &str) -> Option<&'a str> {
    let after = &header[header.find(key)? + key.len()..];
    let value = after[after.find(':')? + 1..].trim_start();
    if value.starts_with('(') {
        return Some(&value[..=value.find(')')?]);
    }
    Some(&value[..value.find([',', '}'])?])
}

/// Where a `.npy` file's header lies, from its magic and version.
///
/// # Arguments
///
/// * `bytes` - The file.
///
/// # Returns
///
/// The header's offset and length, or `None` for another file or version.
fn header_span(bytes: &[u8]) -> Option<(usize, usize)> {
    if bytes.len() < 10 || &bytes[..6] != b"\x93NUMPY" {
        return None;
    }
    match bytes[6] {
        1 => Some((10, usize::from(u16::from_le_bytes([bytes[8], bytes[9]])))),
        2 if bytes.len() >= 12 => {
            let raw = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
            Some((12, usize::try_from(raw).ok()?))
        }
        _ => None,
    }
}

/// A `.npy` header's shape tuple, e.g. `(111070, 32)` or `(99961,)`.
///
/// # Arguments
///
/// * `text` - The tuple's text.
///
/// # Returns
///
/// The shape, or `None` if a part is not a whole number a `usize` holds.
fn parse_shape(text: &str) -> Option<Vec<usize>> {
    text.trim_matches(|c| c == '(' || c == ')')
        .split(',')
        .map(str::trim)
        .filter(|p| !p.is_empty())
        .map(|p| p.parse::<usize>().ok())
        .collect()
}

/// The bytes one value of a `.npy` type takes, for the types a vocabulary's folder holds.
///
/// # Arguments
///
/// * `descr` - numpy's `descr`.
///
/// # Returns
///
/// 1, 4 or 8; or `None` for another type.
fn item_size(descr: &str) -> Option<usize> {
    match descr {
        "|u1" => Some(1),
        "<i4" => Some(4),
        "<i8" | "<f8" => Some(8),
        _ => None,
    }
}

/// Reads a `.npy` file of version 1 or 2, C order.
///
/// # Arguments
///
/// * `path` - The file.
///
/// # Errors
///
/// `InvalidInput` for a missing or malformed file.
fn read_npy(path: &Path) -> Result<Npy> {
    let name = path.display();
    let invalid = |why: &str| fail(ErrorKind::InvalidInput, format!("{name}: {why}"));
    let Ok(bytes) = std::fs::read(path) else {
        return invalid("cannot be read");
    };
    let Some((offset, length)) = header_span(&bytes) else {
        return invalid("not a .npy file of version 1 or 2");
    };
    if length > bytes.len() - offset {
        return invalid("a .npy header past its end");
    }
    let Ok(header) = std::str::from_utf8(&bytes[offset..offset + length]) else {
        return invalid("a .npy header not text");
    };
    if header_value(header, "'fortran_order'") != Some("False") {
        return invalid("expected a C-order .npy array");
    }
    let (Some(descr), Some(shape)) = (
        header_value(header, "'descr'"),
        header_value(header, "'shape'").and_then(parse_shape),
    ) else {
        return invalid("a malformed .npy header");
    };
    let descr = descr.trim_matches('\'').to_owned();
    let Some(item) = item_size(&descr) else {
        return invalid(&format!("unexpected .npy type {descr:?}"));
    };
    let data = bytes[offset + length..].to_vec();
    let size = shape
        .iter()
        .try_fold(item, |count: usize, &extent| count.checked_mul(extent));
    if size != Some(data.len()) {
        return invalid("its data do not fill its shape");
    }
    Ok(Npy { descr, shape, data })
}

/// Reads one `.npy` file of a folder, checked against its type and shape (a 0 accepts any
/// length on that axis).
///
/// # Arguments
///
/// * `root` - The folder.
/// * `name` - The array's name: the file is `<name>.npy`.
/// * `descr` - The type it must have.
/// * `shape` - The shape it must have.
///
/// # Errors
///
/// `InvalidInput` for a missing or malformed file, or another type or shape.
fn read_array(root: &Path, name: &str, descr: &str, shape: &[usize]) -> Result<Npy> {
    let array = read_npy(&root.join(format!("{name}.npy")))?;
    let ok = array.descr == descr
        && array.shape.len() == shape.len()
        && array
            .shape
            .iter()
            .zip(shape)
            .all(|(a, s)| *s == 0 || a == s);
    if ok {
        Ok(array)
    } else {
        fail(
            ErrorKind::InvalidInput,
            format!("{name}: unexpected type or shape"),
        )
    }
}

/// An array's little-endian values, as `N`-byte chunks each made a value.
///
/// # Arguments
///
/// * `data` - The bytes.
/// * `value` - Makes a value of a chunk.
///
/// # Returns
///
/// The values.
fn values<const N: usize, T>(data: &[u8], value: impl Fn([u8; N]) -> T) -> Vec<T> {
    data.as_chunks::<N>().0.iter().map(|c| value(*c)).collect()
}

/// Reads a folder's header and checksums: this format's version, this ORB's descriptor.
///
/// # Arguments
///
/// * `root` - The folder.
///
/// # Errors
///
/// `InvalidInput` for a missing or malformed file, or a header of another format version or
/// descriptor.
fn read_header(root: &Path) -> Result<Header> {
    let numbers = values(
        &read_array(root, "header", "<i8", &[5])?.data,
        i64::from_le_bytes,
    );
    let [version, descriptor_version, k, levels, words] = numbers[..] else {
        return fail(ErrorKind::InvalidInput, "header: unexpected type or shape");
    };
    if version != VERSION {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{}: a vocabulary of format version {version}, expected {VERSION}",
                root.display()
            ),
        );
    }
    if descriptor_version != DESCRIPTOR_VERSION {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{}: trained on descriptor version {descriptor_version}, this ORB's is \
                 {DESCRIPTOR_VERSION}: its words mean nothing here",
                root.display()
            ),
        );
    }
    let sums = read_array(root, "sha256", "|u1", &[ARRAYS.len(), SHA256_BYTES])?;
    let mut sha256 = [[0; SHA256_BYTES]; ARRAYS.len()];
    for (out, digest) in sha256
        .iter_mut()
        .zip(sums.data.as_chunks::<SHA256_BYTES>().0)
    {
        *out = *digest;
    }
    Ok(Header {
        version,
        descriptor_version,
        k,
        levels,
        words,
        sha256,
    })
}

/// Reads a vocabulary's folder.
///
/// # Arguments
///
/// * `folder` - The folder `slam_it.eval.vocabularies.write` wrote.
///
/// # Errors
///
/// `InvalidInput` for a missing or malformed file, a header of another format version or
/// descriptor, a malformed tree, or a header whose words are not the tree's.
pub fn read(folder: &Path) -> Result<Folder> {
    let header = read_header(folder)?;
    let parents = read_array(folder, "parents", "<i4", &[0])?;
    let n = parents.shape[0];
    let descriptors = read_array(folder, "descriptors", "|u1", &[n, DESCRIPTOR_BYTES])?;
    let children = read_array(folder, "children", "<i4", &[n, 2])?;
    let words = read_array(folder, "words", "<i4", &[n])?;
    let weights = read_array(folder, "weights", "<f8", &[0])?;
    let children_flat = values(&children.data, i32::from_le_bytes);
    let (Ok(k), Ok(levels)) = (usize::try_from(header.k), usize::try_from(header.levels)) else {
        return fail(ErrorKind::InvalidInput, "header: a negative k or levels");
    };
    let vocabulary = Vocabulary {
        k,
        levels,
        descriptors: descriptors.data.as_chunks::<DESCRIPTOR_BYTES>().0.to_vec(),
        parents: values(&parents.data, i32::from_le_bytes),
        children: children_flat.as_chunks::<2>().0.to_vec(),
        words: values(&words.data, i32::from_le_bytes),
        weights: values(&weights.data, f64::from_le_bytes),
    };
    check(&vocabulary)?;
    if i64::try_from(vocabulary.weights.len()).ok() != Some(header.words) {
        return fail(
            ErrorKind::InvalidInput,
            format!(
                "{}: the header counts {} words, the tree {}",
                folder.display(),
                header.words,
                vocabulary.weights.len()
            ),
        );
    }
    Ok(Folder { vocabulary, header })
}

#[cfg(test)]
mod tests {
    //! Native tests against the shared fixture `tests/fixtures/eval/vocabulary/` and its cases
    //! `vocabulary_cases.json`, which the Python and C++ tests run too: the tree and its header
    //! as read, and every case the Rust reader reads or refuses.

    use super::*;
    use serde_json::Value;
    use std::fmt::Write as _;
    use std::path::PathBuf;

    /// The fixtures' folder.
    fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tests/fixtures/eval")
    }

    /// The fixture.
    fn fixture() -> Value {
        let text = std::fs::read_to_string(fixtures().join("vocabulary_cases.json")).unwrap();
        serde_json::from_str(&text).unwrap()
    }

    /// A JSON list of integers.
    fn integers<T: TryFrom<i64>>(value: &Value) -> Vec<T>
    where
        T::Error: std::fmt::Debug,
    {
        value
            .as_array()
            .unwrap()
            .iter()
            .map(|v| T::try_from(v.as_i64().unwrap()).unwrap())
            .collect()
    }

    #[test]
    fn the_fixture_folder_reads_as_its_tree_with_its_header() {
        let fixture = fixture();
        let (tree, record) = (&fixture["tree"], &fixture["record"]);
        let read = read(&fixtures().join("vocabulary")).unwrap();
        let v = &read.vocabulary;
        assert_eq!(
            (v.k, v.levels),
            (
                usize::try_from(tree["k"].as_u64().unwrap()).unwrap(),
                usize::try_from(tree["levels"].as_u64().unwrap()).unwrap()
            )
        );
        let descriptors: Vec<u8> = tree["descriptors"]
            .as_array()
            .unwrap()
            .iter()
            .flat_map(integers::<u8>)
            .collect();
        assert_eq!(v.descriptors.concat(), descriptors);
        assert_eq!(v.parents, integers::<i32>(&tree["parents"]));
        let children: Vec<i32> = tree["children"]
            .as_array()
            .unwrap()
            .iter()
            .flat_map(integers::<i32>)
            .collect();
        assert_eq!(v.children.concat(), children);
        assert_eq!(v.words, integers::<i32>(&tree["words"]));
        let weights: Vec<f64> = tree["weights"]
            .as_array()
            .unwrap()
            .iter()
            .map(|w| w.as_f64().unwrap())
            .collect();
        assert_eq!(v.weights, weights);
        let h = &read.header;
        let number = |key: &str| record[key].as_i64().unwrap();
        assert_eq!(
            (h.version, h.descriptor_version, h.k, h.levels, h.words),
            (
                number("version"),
                number("descriptor_version"),
                number("k"),
                number("levels"),
                number("words")
            )
        );
        for (digest, name) in h.sha256.iter().zip(ARRAYS) {
            let hex = digest.iter().fold(String::new(), |mut hex, b| {
                let _ = write!(hex, "{b:02x}");
                hex
            });
            assert_eq!(hex, record["sha256"][name].as_str().unwrap(), "{name}");
        }
    }

    #[test]
    fn every_case_is_read_or_refused_with_its_message() {
        let fixture = fixture();
        // Named for this process, emptied before each case and removed at the end; a failed
        // case leaves it to the next run, which empties it first.
        let scratch =
            std::env::temp_dir().join(format!("slam_it_vocabularies_rust_{}", std::process::id()));
        for case in fixture["cases"].as_array().unwrap() {
            let name = case["name"].as_str().unwrap();
            let _ = std::fs::remove_dir_all(&scratch);
            std::fs::create_dir_all(&scratch).unwrap();
            for entry in std::fs::read_dir(fixtures().join("vocabulary")).unwrap() {
                let path = entry.unwrap().path();
                std::fs::copy(&path, scratch.join(path.file_name().unwrap())).unwrap();
            }
            for (file, hex) in case["files"].as_object().unwrap() {
                let hex = hex.as_str().unwrap();
                let bytes: Vec<u8> = (0..hex.len())
                    .step_by(2)
                    .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
                    .collect();
                std::fs::write(scratch.join(file), bytes).unwrap();
            }
            let refused = case["refused_by"]
                .as_array()
                .unwrap()
                .iter()
                .any(|l| l == "rust");
            match read(&scratch) {
                Err(error) if refused => {
                    assert_eq!(error.kind, ErrorKind::InvalidInput, "{name}");
                    let wanted = case["match"].as_str().unwrap();
                    assert!(error.message.contains(wanted), "{name}: {}", error.message);
                }
                Ok(_) if !refused => {}
                other => panic!(
                    "{name}: expected {}, got {other:?}",
                    if refused { "a refusal" } else { "a read" }
                ),
            }
        }
        let _ = std::fs::remove_dir_all(&scratch);
    }

    #[test]
    fn a_missing_folder_is_invalid_input() {
        let error = read(&fixtures().join("missing")).unwrap_err();
        assert_eq!(error.kind, ErrorKind::InvalidInput);
    }
}
