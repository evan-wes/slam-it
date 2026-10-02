"""SO(3) helpers -- the Python reference implementation.

A placeholder proving the three-language pipeline; rung 0's real SO(3) work
arrives with milestone 1.
"""

from __future__ import annotations

import numpy as np


def hat(v) -> np.ndarray:
    """Skew-symmetric matrix [v]x of a 3-vector, so that hat(v) @ u == cross(v, u)."""
    x, y, z = np.asarray(v, dtype=np.float64)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ]
    )
