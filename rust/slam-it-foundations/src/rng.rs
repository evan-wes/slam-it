//! PCG32, the random generator every backend shares, so a seed draws the same samples
//! everywhere.
//!
//! Mirrors `slam_it.foundations.rng`, whose docstring states the algorithm: O'Neill's PCG32
//! (XSH RR), its unbiased bounded draw, and Floyd's algorithm for distinct indices. RANSAC
//! draws its samples from it, so every backend tests the same hypotheses. It is not bound to
//! Python itself; the shared fixture `tests/fixtures/foundations/rng.json` holds every
//! language to the same draws.

use crate::error::{ErrorKind, Result, fail, fail_setting};

/// The LCG multiplier of the reference library.
const MULTIPLIER: u64 = 6_364_136_223_846_793_005;

/// O'Neill's PCG32 (XSH RR, 64-bit state, 32-bit output).
#[derive(Debug, Clone)]
pub struct Pcg32 {
    /// The 64-bit state.
    state: u64,
    /// The increment, odd: `2 * stream + 1`.
    increment: u64,
}

impl Pcg32 {
    /// Seeds a generator, as the reference library's `pcg32_srandom_r` does.
    ///
    /// # Arguments
    ///
    /// * `seed` - The initial state.
    /// * `stream` - The stream (sequence) selector; its top bit is dropped, as in the
    ///   reference library.
    ///
    /// # Returns
    ///
    /// The generator.
    #[must_use]
    pub fn new(seed: u64, stream: u64) -> Self {
        let mut rng = Self {
            state: 0,
            increment: (stream << 1) | 1,
        };
        rng.next_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.next_u32();
        rng
    }

    /// Draws the next 32-bit output.
    ///
    /// # Returns
    ///
    /// A uniform integer in `[0, 2³²)`.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "XSH RR keeps the low 32 bits of the shifted state by design"
    )]
    pub fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old.wrapping_mul(MULTIPLIER).wrapping_add(self.increment);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rotation = (old >> 59) as u32;
        xorshifted.rotate_right(rotation)
    }

    /// Draws a uniform integer below `bound`, without modulo bias: outputs below
    /// `2³² mod bound` are rejected.
    ///
    /// # Arguments
    ///
    /// * `bound` - The exclusive upper bound, at least 1.
    ///
    /// # Returns
    ///
    /// A uniform integer in `[0, bound)`.
    ///
    /// # Errors
    ///
    /// `InvalidInput` if `bound` is 0.
    pub fn bounded(&mut self, bound: u32) -> Result<u32> {
        if bound == 0 {
            return fail_setting("bound", "expected 1 <= bound < 2**32", bound);
        }
        Ok(self.draw_below(bound))
    }

    /// Draws `count` distinct indices from `[0, population)` (Floyd's algorithm), with
    /// exactly `count` bounded draws.
    ///
    /// # Arguments
    ///
    /// * `population` - The number of indices to choose from.
    /// * `count` - How many to draw.
    ///
    /// # Returns
    ///
    /// The indices, in the order Floyd's algorithm adds them.
    ///
    /// # Errors
    ///
    /// `InvalidInput` if `count` exceeds `population`.
    pub fn sample(&mut self, population: u32, count: u32) -> Result<Vec<u32>> {
        if count > population {
            return fail(
                ErrorKind::InvalidInput,
                format!("sample: expected 0 <= count <= population, got {count} of {population}"),
            );
        }
        let mut chosen = Vec::with_capacity(count as usize);
        for j in population - count..population {
            let pick = self.draw_below(j + 1);
            chosen.push(if chosen.contains(&pick) { j } else { pick });
        }
        Ok(chosen)
    }

    /// The bounded draw, for a bound already known to be at least 1.
    ///
    /// # Arguments
    ///
    /// * `bound` - The exclusive upper bound, at least 1.
    ///
    /// # Returns
    ///
    /// A uniform integer in `[0, bound)`.
    fn draw_below(&mut self, bound: u32) -> u32 {
        let threshold = bound.wrapping_neg() % bound;
        loop {
            let value = self.next_u32();
            if value >= threshold {
                return value % bound;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Native tests against the parity fixture `tests/fixtures/foundations/rng.json`, which
    //! the Python and C++ tests share.

    use super::*;
    use serde_json::Value;

    /// Reads the shared fixture.
    fn fixture() -> Value {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../tests/fixtures/foundations/rng.json"
        );
        let text = std::fs::read_to_string(path).expect("the fixture is in the repository");
        serde_json::from_str(&text).expect("the fixture is valid JSON")
    }

    /// Reads an unsigned integer field.
    fn number(value: &Value) -> u64 {
        value.as_u64().expect("a non-negative integer")
    }

    /// Reads an unsigned 32-bit integer field.
    fn number32(value: &Value) -> u32 {
        u32::try_from(number(value)).expect("fits in 32 bits")
    }

    /// The output stream is the reference library's, bit for bit.
    #[test]
    fn matches_the_reference_library() {
        let reference = &fixture()["reference"];
        let mut rng = Pcg32::new(number(&reference["seed"]), number(&reference["stream"]));
        for expected in reference["next_u32"].as_array().unwrap() {
            assert_eq!(rng.next_u32(), number32(expected));
        }
    }

    /// Floyd samples from one generator are the fixture's.
    #[test]
    fn samples_match_the_fixture() {
        let samples = &fixture()["samples"];
        let mut rng = Pcg32::new(number(&samples["seed"]), number(&samples["stream"]));
        for draw in samples["draws"].as_array().unwrap() {
            let drawn = rng
                .sample(number32(&draw["population"]), number32(&draw["count"]))
                .unwrap();
            let expected: Vec<u32> = draw["expected"]
                .as_array()
                .unwrap()
                .iter()
                .map(number32)
                .collect();
            assert_eq!(drawn, expected);
        }
    }

    /// A count above the population and a bound of 0 are rejected; a bound of 1 gives 0.
    #[test]
    fn invalid_input_is_rejected() {
        let mut rng = Pcg32::new(7, 0);
        assert_eq!(rng.sample(3, 4).unwrap_err().kind, ErrorKind::InvalidInput);
        assert_eq!(rng.bounded(0).unwrap_err().kind, ErrorKind::InvalidInput);
        assert_eq!(rng.bounded(1).unwrap(), 0);
    }
}
