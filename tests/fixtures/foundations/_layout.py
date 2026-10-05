"""How the generators lay out a fixture's JSON: indented, each innermost list on one line."""

from __future__ import annotations

import json
import re
from typing import Any


def dumps(doc: dict[str, Any]) -> str:
    """Serialise a fixture, one line per innermost list of numbers.

    Parameters
    ----------
    doc : dict
        The fixture.

    Returns
    -------
    str
        The JSON text with a final newline. ``indent`` alone would give every number its own
        line; an image's rows then read as rows.
    """
    text = json.dumps(doc, indent=1)
    compact = re.sub(
        r"\[\s*([-+0-9.eE,\snul]*?)\s*\]",
        lambda match: "[" + re.sub(r"\s+", " ", match.group(1)).replace(" ,", ",") + "]",
        text,
    )
    return compact + "\n"
