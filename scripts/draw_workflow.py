"""Draw the README's workflow diagram, ``docs/workflow.svg``: how SLAM It! is built.

    python scripts/draw_workflow.py [OUT.svg]

The layout is drawn by hand, not by a graph engine (Mermaid, Graphviz), so the picture keeps what
an engine would not: one gap between every two steps, the four stages centred on the line that
joins them, and the review loop's arrows on chosen sides (out of Layer 1's left side and into its
right; into the owner's hexagon at its left point and out at its right). To change the diagram,
edit :func:`stages` and run this again.

Text is measured with Liberation Sans (``fonts-liberation`` on Debian and Ubuntu), which has
Arial's metrics; the SVG names Arial first. Each shape's text is placed by its glyphs' outlines,
so its ink is centred in the shape whatever its descenders (:func:`centred_text`). The colours
are the run viewer's dark theme, with the owner's steps in amber.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path
from xml.sax.saxutils import escape

from PIL import ImageFont

FAMILY = "Arial, 'Liberation Sans', Helvetica, sans-serif"
BG, PANEL, NODE, NODE_EDGE, TEXT = "#101416", "#202729", "#181d20", "#3a464b", "#e6ecec"
LINE, AMBER, INK = "#8d9a9f", "#f2c14e", "#08181b"

SIZE, LH, PADX, PADY = 15, 19, 18, 11
NODE_H = 2 * LH + 2 * PADY  # every step is two lines, so every step is one height
GAP = 46  # the arrow between two steps, everywhere
TITLE, TITLE_TOP, TITLE_GAP = 21, 16, 18
STAGE_PAD, STAGE_GAP, MARGIN = 26, 64, 24
LABEL = 13
TAG_H = LABEL + 11  # a coloured label's block
SLANT = 17  # how far the hexagon's points stand out
RADIUS = 12  # rounded corners on bent arrows
HEAD = 10  # arrowhead length
STROKE = 2.2
# The review loop's two ways out of the owner's decision: back to the checks, or on.
FIXES, FIXES_FILL = "apply selected fixes", "#f0884a"
RESOLVED, RESOLVED_FILL = "all issues resolved", "#5fd38d"

_fonts: dict[tuple[bool, int], ImageFont.FreeTypeFont] = {}

Point = tuple[float, float]


FINE = 8  # ink is measured at this many times the size, for sub-pixel extents


def font(bold: bool, size: int) -> ImageFont.FreeTypeFont:
    """Return Liberation Sans, regular or bold, at ``size`` pixels.

    Parameters
    ----------
    bold : bool
        Whether the face is bold.
    size : int
        The font size, in pixels.

    Returns
    -------
    ImageFont.FreeTypeFont
        The font, loaded once.
    """
    if (bold, size) not in _fonts:
        name = "LiberationSans-Bold.ttf" if bold else "LiberationSans-Regular.ttf"
        _fonts[(bold, size)] = ImageFont.truetype(name, size)
    return _fonts[(bold, size)]


def text_width(text: str, bold: bool = False, size: int = SIZE) -> float:
    """Return the advance width of one line of ``text``, in pixels.

    Parameters
    ----------
    text : str
        One line of text.
    bold : bool
        Whether the line is bold.
    size : int
        The font size, in pixels.

    Returns
    -------
    float
        The line's advance width.
    """
    return font(bold, size).getlength(text)


@dataclass(frozen=True)
class TextStyle:
    """How a piece of text is set.

    Attributes
    ----------
    size : int
        The font size, in pixels.
    fill : str
        The text's colour.
    bold : bool
        Whether the text is bold.
    caps : bool
        Centre vertically by the capitals' height (top of an H to the baseline) rather than by the
        ink, so that titles side by side share one rule whatever their letters.
    """

    size: int
    fill: str
    bold: bool = False
    caps: bool = False


def centred_text(centre: Point, lines: list[str], style: TextStyle) -> str:
    """Return ``lines`` as SVG text whose ink is centred on ``centre``.

    SVG's own centring (``dominant-baseline``) centres a fixed line box, which leaves a line with
    descenders sitting low. Here each line's glyph outlines are measured instead: the block's ink,
    from its highest glyph's top to its lowest glyph's bottom, is centred vertically, and each
    line's ink horizontally. The baselines are written out, so no renderer has to place them.

    Parameters
    ----------
    centre : Point
        Where the ink's centre goes.
    lines : list[str]
        The lines, top to bottom, ``LH`` apart.
    style : TextStyle
        How the text is set.

    Returns
    -------
    str
        One SVG ``text`` element per line.
    """
    fine = font(style.bold, style.size * FINE)
    # ink boxes (left, top, right, bottom) relative to each line's left end on its baseline
    boxes = [tuple(v / FINE for v in fine.getbbox(t, anchor="ls")) for t in lines]
    if style.caps:
        top = fine.getbbox("H", anchor="ls")[1] / FINE
        bottom = float((len(lines) - 1) * LH)
    else:
        top = min(b[1] + i * LH for i, b in enumerate(boxes))
        bottom = max(b[3] + i * LH for i, b in enumerate(boxes))
    first = centre[1] - (top + bottom) / 2
    weight = ' font-weight="700"' if style.bold else ""
    out = []
    for i, (line, b) in enumerate(zip(lines, boxes, strict=True)):
        # text-anchor centres the advance width; move it so the ink is centred instead
        x = centre[0] + fine.getlength(line) / FINE / 2 - (b[0] + b[2]) / 2
        out.append(
            f'<text x="{x:.2f}" y="{first + i * LH:.2f}" font-size="{style.size}" '
            f'fill="{style.fill}"{weight} text-anchor="middle">{escape(line)}</text>'
        )
    return "".join(out)


@dataclass
class Step:
    """One step: two lines of text in a box, a hexagon (an owner's decision) or a pill.

    Attributes
    ----------
    lines : list[str]
        The step's text, one entry per line.
    shape : str
        ``"box"``, ``"hex"`` (the owner decides) or ``"pill"`` (the owner starts).
    x, y : float
        The step's centre, once placed.
    w : float
        The step's width, from its text.
    """

    lines: list[str]
    shape: str = "box"
    x: float = 0.0
    y: float = 0.0
    w: float = field(init=False)

    def __post_init__(self) -> None:
        """Size the step to its text."""
        self.w = max(text_width(t, self.owner) for t in self.lines) + 2 * PADX
        if self.shape == "hex":
            self.w += 2 * SLANT - 8

    @property
    def owner(self) -> bool:
        """Whether the owner takes this step (drawn in amber)."""
        return self.shape in ("hex", "pill")

    def left(self) -> Point:
        """Return the middle of the step's left side."""
        return self.x - self.w / 2, self.y

    def right(self) -> Point:
        """Return the middle of the step's right side."""
        return self.x + self.w / 2, self.y

    def top(self) -> Point:
        """Return the middle of the step's top."""
        return self.x, self.y - NODE_H / 2

    def bottom(self) -> Point:
        """Return the middle of the step's bottom."""
        return self.x, self.y + NODE_H / 2


@dataclass
class Tag:
    """A coloured label on the arrow between two steps, given a row of its own in the column.

    Attributes
    ----------
    text : str
        The label.
    fill : str
        Its block's colour.
    x, y : float
        Its centre, once placed.
    """

    text: str
    fill: str
    x: float = 0.0
    y: float = 0.0


Row = Step | Tag | None


@dataclass
class Stage:
    """One stage: a titled column of steps, top to bottom.

    Attributes
    ----------
    title : str
        The stage's title.
    colour : str
        Its outline and title colour.
    steps : list[Row]
        The column, one gap between every two rows. ``None`` marks the review loop's row: the panel
        on the left of the column, the fixes on the right. A :class:`Tag` labels the arrow that
        passes through its row.
    panel : Step | None
        The review panel, in the loop's row.
    """

    title: str
    colour: str
    steps: list[Row]
    panel: Step | None = None


def step(text: str, shape: str = "box") -> Step:
    """Return a step whose lines are ``text`` split at newlines.

    Parameters
    ----------
    text : str
        The step's two lines, separated by a newline.
    shape : str
        As :class:`Step`'s ``shape``.

    Returns
    -------
    Step
        The step, sized and not yet placed.
    """
    return Step(text.split("\n"), shape)


def stages() -> tuple[Step, list[Stage]]:
    """Return the diagram's content: the owner's start, and the four stages.

    Returns
    -------
    tuple[Step, list[Stage]]
        The start, then the stages left to right.
    """
    goal = step("The owner sets\na slice's goal", "pill")
    return goal, [
        Stage(
            "1 · Design",
            "#a596ff",
            [
                step("Research agents read\nthe reference systems"),
                step("Design drafted: each fork\ngrounded in a reference"),
                step("Owner decides\neach fork", "hex"),
                step("Design committed\nto develop"),
            ],
        ),
        Stage(
            "2 · Build",
            "#f0884a",
            [
                step("A git worktree\nper branch"),
                step("Python\nreference"),
                step("C++ and Rust ports,\nheld to Python by parity tests"),
                step("A golden record per language;\none bar all three must meet"),
                step("Touched checks per commit;\nmessages checked claim by claim"),
            ],
        ),
        Stage(
            "3 · Verify",
            "#5fd38d",
            [
                step("Measured on public data\nagainst published figures"),
                step("Layer 1: automated checks\nlint, types, docs, tests in every language"),
                None,
                step("Owner decides\nevery finding", "hex"),
                Tag(RESOLVED, RESOLVED_FILL),
                step("Panel report saved:\nfindings, decisions, resolutions"),
                step("Offline CI from\na clean checkout"),
            ],
            panel=step("Layer 2: review panel\nhygiene · architecture · algorithm"),
        ),
        Stage(
            "4 · Ship",
            "#3fb6c6",
            [
                step("Pull request into develop;\n'How this was built' links the report"),
                step("Owner's\nmerge Yes", "hex"),
                step("Commits verified;\nversion tag made"),
                step("Owner's release call:\ndevelop to main", "hex"),
            ],
        ),
    ]


def loop_offset(stage: Stage) -> float:
    """Return how far the review loop's two sides sit from the stage's centre line.

    Parameters
    ----------
    stage : Stage
        A stage with a loop row.

    Returns
    -------
    float
        The offset: clear of the step above the loop, which the loop leaves and re-enters.
    """
    above = stage.steps[stage.steps.index(None) - 1]
    assert isinstance(above, Step)
    return above.w / 2 + 42


def stage_size(stage: Stage) -> tuple[float, float]:
    """Return a stage's width and height.

    Parameters
    ----------
    stage : Stage
        The stage.

    Returns
    -------
    tuple[float, float]
        Width and height, symmetric about the stage's centre line.
    """
    half = max(s.w / 2 for s in stage.steps if isinstance(s, Step))
    if stage.panel:
        dx = loop_offset(stage)
        half = max(half, dx + stage.panel.w / 2, dx + text_width(FIXES, True, LABEL) / 2 + 8)
    half = max(half, text_width(stage.title, True, TITLE) / 2)
    rows = sum(row_height(r) for r in stage.steps) + (len(stage.steps) - 1) * GAP
    return 2 * (half + STAGE_PAD), TITLE_TOP + TITLE + TITLE_GAP + rows + STAGE_PAD


def row_height(row: Row) -> float:
    """Return a row's height: a tag's block, or a step's.

    Parameters
    ----------
    row : Row
        The row.

    Returns
    -------
    float
        Its height.
    """
    return TAG_H if isinstance(row, Tag) else NODE_H


def rounded(points: list[Point]) -> str:
    """Return SVG path data for an orthogonal polyline with rounded corners.

    The end is pulled back by the arrowhead's length, so the head's tip lands on the last point.

    Parameters
    ----------
    points : list[Point]
        The polyline's corners, at least two.

    Returns
    -------
    str
        The path's ``d`` attribute.
    """
    pts = list(points)
    (x0, y0), (x1, y1) = pts[-2], pts[-1]
    d = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
    pts[-1] = (x1 - (x1 - x0) / d * HEAD, y1 - (y1 - y0) / d * HEAD)
    out = [f"M{pts[0][0]:.1f},{pts[0][1]:.1f}"]
    for (ax, ay), (bx, by), (cx, cy) in zip(pts, pts[1:], pts[2:], strict=False):
        nin = ((bx - ax) ** 2 + (by - ay) ** 2) ** 0.5
        nout = ((cx - bx) ** 2 + (cy - by) ** 2) ** 0.5
        r = min(RADIUS, nin / 2, nout / 2)
        p = (bx - (bx - ax) / nin * r, by - (by - ay) / nin * r)
        q = (bx + (cx - bx) / nout * r, by + (cy - by) / nout * r)
        out.append(f"L{p[0]:.1f},{p[1]:.1f} Q{bx:.1f},{by:.1f} {q[0]:.1f},{q[1]:.1f}")
    out.append(f"L{pts[-1][0]:.1f},{pts[-1][1]:.1f}")
    return " ".join(out)


def arrow(points: list[Point]) -> str:
    """Return an arrow along ``points``, its head at the last one.

    Parameters
    ----------
    points : list[Point]
        The arrow's corners.

    Returns
    -------
    str
        An SVG ``path`` element.
    """
    return (
        f'<path d="{rounded(points)}" fill="none" stroke="{LINE}" stroke-width="{STROKE}" '
        f'stroke-linejoin="round" marker-end="url(#head)"/>'
    )


def label(x: float, y: float, text: str, fill: str) -> str:
    """Return a label in a coloured block, centred on a point of an arrow.

    Parameters
    ----------
    x, y : float
        The label's centre.
    text : str
        The label.
    fill : str
        The block's colour; the text is dark and bold on it.

    Returns
    -------
    str
        SVG ``rect`` and ``text`` elements.
    """
    w = text_width(text, True, LABEL) + 16
    return (
        f'<rect x="{x - w / 2:.1f}" y="{y - TAG_H / 2:.1f}" width="{w:.1f}" height="{TAG_H}" '
        f'rx="4" fill="{fill}"/>' + centred_text((x, y), [text], TextStyle(LABEL, INK, bold=True))
    )


def draw_step(s: Step) -> str:
    """Return a placed step's shape and text.

    Parameters
    ----------
    s : Step
        The step.

    Returns
    -------
    str
        SVG elements.
    """
    x0, y0, x1, y1 = s.x - s.w / 2, s.y - NODE_H / 2, s.x + s.w / 2, s.y + NODE_H / 2
    fill, stroke, ink = (AMBER, AMBER, INK) if s.owner else (NODE, NODE_EDGE, TEXT)
    if s.shape == "hex":
        corners = [
            (x0, s.y),
            (x0 + SLANT, y0),
            (x1 - SLANT, y0),
            (x1, s.y),
            (x1 - SLANT, y1),
            (x0 + SLANT, y1),
        ]
        body = f'<polygon points="{" ".join(f"{a:.1f},{b:.1f}" for a, b in corners)}"'
    else:
        rx = NODE_H / 2 if s.shape == "pill" else 5
        body = f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{s.w:.1f}" height="{NODE_H}" rx="{rx}"'
    shape = f'{body} fill="{fill}" stroke="{stroke}" stroke-width="1.2"/>'
    return shape + centred_text((s.x, s.y), s.lines, TextStyle(SIZE, ink, bold=s.owner))


@dataclass
class Layers:
    """The drawing's elements, by layer, back to front.

    Attributes
    ----------
    boxes, edges, steps, labels : list[str]
        Stage boxes and titles; arrows; steps; labels on arrows.
    """

    boxes: list[str] = field(default_factory=list)
    edges: list[str] = field(default_factory=list)
    steps: list[str] = field(default_factory=list)
    labels: list[str] = field(default_factory=list)


def draw_stage(stage: Stage, left: float, mid: float, layers: Layers) -> float:
    """Place a stage with its left side at ``left``, centred on ``mid``, and draw it.

    Parameters
    ----------
    stage : Stage
        The stage.
    left : float
        Its left side.
    mid : float
        The line joining the stages.
    layers : Layers
        Where its elements go.

    Returns
    -------
    float
        Its right side.
    """
    w, h = stage_size(stage)
    top, cx = mid - h / 2, left + w / 2
    title = (cx, top + TITLE_TOP + TITLE / 2)
    layers.boxes.append(
        f'<rect x="{left:.1f}" y="{top:.1f}" width="{w:.1f}" height="{h:.1f}" rx="10" '
        f'fill="{PANEL}" stroke="{stage.colour}" stroke-width="2"/>'
        + centred_text(title, [stage.title], TextStyle(TITLE, stage.colour, bold=True, caps=True))
    )
    y = top + TITLE_TOP + TITLE + TITLE_GAP
    for row in stage.steps:
        if row is not None:
            row.x, row.y = cx, y + row_height(row) / 2
        y += row_height(row) + GAP
    # one arrow between two steps, passing behind any tag between them; none across the loop row
    column = [r for r in stage.steps if not isinstance(r, Tag)]
    for a, b in pairwise(column):
        if a and b:
            layers.edges.append(arrow([a.bottom(), b.top()]))
    layers.labels.extend(label(t.x, t.y, t.text, t.fill) for t in stage.steps if isinstance(t, Tag))
    if stage.panel:
        draw_loop(stage, cx, layers)
    layers.steps.extend(draw_step(s) for s in stage.steps if isinstance(s, Step))
    return left + w


def draw_loop(stage: Stage, cx: float, layers: Layers) -> None:
    """Draw the review loop: Layer 1, the panel, the owner's decision, and back.

    Parameters
    ----------
    stage : Stage
        A placed stage with a loop row.
    cx : float
        The stage's centre line.
    layers : Layers
        Where the loop's elements go.
    """
    assert stage.panel is not None
    row = stage.steps.index(None)
    above, below = stage.steps[row - 1], stage.steps[row + 1]
    assert isinstance(above, Step)
    assert isinstance(below, Step)
    dx = loop_offset(stage)
    panel = stage.panel
    panel.x, panel.y = cx - dx, (above.y + below.y) / 2
    layers.steps.append(draw_step(panel))
    # out of Layer 1's left side into the panel; the panel into the owner's left point
    layers.edges.append(arrow([above.left(), (cx - dx, above.y), panel.top()]))
    layers.edges.append(arrow([panel.bottom(), (cx - dx, below.y), below.left()]))
    # out of the owner's right point, up through the fixes, into Layer 1's right side
    layers.edges.append(
        arrow([below.right(), (cx + dx, below.y), (cx + dx, above.y), above.right()])
    )
    layers.labels.append(label(cx + dx, panel.y, FIXES, FIXES_FILL))


def render() -> str:
    """Return the diagram as an SVG document.

    Returns
    -------
    str
        The SVG.
    """
    goal, columns = stages()
    height = max(stage_size(s)[1] for s in columns) + 2 * MARGIN
    mid = height / 2
    goal.x, goal.y = MARGIN + goal.w / 2, mid
    layers = Layers(steps=[draw_step(goal)])
    x = MARGIN + goal.w + STAGE_GAP
    layers.edges.append(arrow([goal.right(), (x, mid)]))
    for i, stage in enumerate(columns):
        x = draw_stage(stage, x, mid, layers)
        if i < len(columns) - 1:
            layers.edges.append(arrow([(x + 1, mid), (x + STAGE_GAP - 1, mid)]))
            x += STAGE_GAP
    width = x + MARGIN
    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0f}" height="{height:.0f}" '
        f'viewBox="0 0 {width:.0f} {height:.0f}" font-family="{FAMILY}">'
        f"<title>How SLAM It! is built: design, build, verify, ship</title>"
        f'<defs><marker id="head" viewBox="0 0 10 10" refX="0" refY="5" markerWidth="{HEAD}" '
        f'markerHeight="{HEAD}" markerUnits="userSpaceOnUse" orient="auto">'
        f'<path d="M0,0.5 L10,5 L0,9.5 z" fill="{LINE}"/></marker></defs>'
        f'<rect width="100%" height="100%" fill="{BG}"/>'
        + "".join(layers.boxes + layers.edges + layers.steps + layers.labels)
        + "</svg>\n"
    )


def main(argv: list[str]) -> int:
    """Write the diagram to ``argv[0]``, or to ``docs/workflow.svg``.

    Parameters
    ----------
    argv : list[str]
        The command line, without the program's name.

    Returns
    -------
    int
        The exit status.
    """
    out = Path(argv[0]) if argv else Path(__file__).resolve().parents[1] / "docs" / "workflow.svg"
    out.write_text(render(), encoding="utf-8")
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
