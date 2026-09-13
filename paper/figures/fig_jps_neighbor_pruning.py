"""JPS neighbor-pruning schematic for manuscript Fig. X.

Panel (a): cardinal (straight) expansion keeps natural N and forced F.
Panel (b): diagonal expansion keeps natural N1 (diag), N2 (horiz), N3 (vert) and forced F.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle
from matplotlib.lines import Line2D

SKILL_SCRIPTS = Path(
    r"D:\Mino-File\shiyan1\.claude\skills\nature-figure\scripts"
)
if str(SKILL_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SKILL_SCRIPTS))

from audit_panel_alignment import require_matplotlib_panel_alignment  # noqa: E402

OUT_DIR = Path(__file__).resolve().parent
STEM = OUT_DIR / "fig-jps-neighbor-pruning"

C_CURRENT = "#0F4D92"
C_NATURAL = "#8BCF8B"
C_NATURAL_EDGE = "#2E7D4F"
C_FORCED = "#F0B7B2"
C_FORCED_EDGE = "#B64342"
C_OBSTACLE = "#3A3A3A"
C_OBSTACLE_MARK = "#6A6A6A"
C_PRUNED = "#EFEFEF"
C_PARENT_FACE = "#D9D9D9"
C_GRID_GAP = "#FFFFFF"
C_ARROW = "#0F4D92"
C_TEXT = "#272727"
C_NOTE = "#555555"

mpl.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": [
            "Microsoft YaHei",
            "SimHei",
            "Arial",
            "Helvetica",
            "DejaVu Sans",
        ],
        "axes.unicode_minus": False,
        "svg.fonttype": "none",
        "pdf.fonttype": 42,
        "font.size": 8,
        "axes.linewidth": 0.6,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
    }
)

FILL = {
    "obstacle": C_OBSTACLE,
    "forced": C_FORCED,
    "natural": C_NATURAL,
    "pruned": C_PRUNED,
    "current": C_CURRENT,
    "parent": C_PARENT_FACE,
    "empty": C_GRID_GAP,
}


def cell_origin(r: int, c: int) -> tuple[float, float]:
    return float(c), float(3 - r - 1)


def draw_cell(ax, r: int, c: int, kind: str, label: str | None = None):
    x, y = cell_origin(r, c)
    # Inset face only — white gutters between cells, no stroke near glyphs.
    inset = 0.05
    ax.add_patch(
        Rectangle(
            (x + inset, y + inset),
            1 - 2 * inset,
            1 - 2 * inset,
            facecolor=FILL[kind],
            edgecolor="none",
            zorder=3,
        )
    )
    if label:
        color = {
            "current": "white",
            "forced": C_FORCED_EDGE,
            "natural": C_NATURAL_EDGE,
        }.get(kind, C_TEXT)
        ax.text(
            x + 0.5,
            y + 0.5,
            label,
            ha="center",
            va="center",
            fontsize=8.5,
            fontweight="bold",
            color=color,
            zorder=5,
        )


def draw_obstacle_marks(ax, r: int, c: int):
    """Three short diagonal strokes inside an obstacle cell (no hatch near labels)."""
    x, y = cell_origin(r, c)
    for t in (0.25, 0.5, 0.75):
        ax.plot(
            [x + t - 0.18, x + t + 0.18],
            [y + 0.18, y + 0.82],
            color=C_OBSTACLE_MARK,
            lw=0.9,
            solid_capstyle="butt",
            zorder=4,
        )


def setup_panel(ax, panel_tag: str, panel_title: str):
    ax.set_xlim(-0.3, 3.3)
    ax.set_ylim(-0.25, 3.95)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.text(
        -0.3,
        3.85,
        panel_tag,
        ha="left",
        va="top",
        fontsize=8.5,
        fontweight="bold",
        color=C_TEXT,
        zorder=8,
    )
    ax.text(
        -0.3,
        3.45,
        panel_title,
        ha="left",
        va="top",
        fontsize=8,
        color=C_TEXT,
        zorder=8,
    )


def cell_center(r: int, c: int) -> tuple[float, float]:
    x, y = cell_origin(r, c)
    return x + 0.5, y + 0.5


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def draw_label(ax, r: int, c: int, kind: str, label: str):
    color = {
        "current": "white",
        "forced": C_FORCED_EDGE,
        "natural": C_NATURAL_EDGE,
    }.get(kind, C_TEXT)
    x, y = cell_center(r, c)
    ax.text(
        x,
        y,
        label,
        ha="center",
        va="center",
        fontsize=8.5,
        fontweight="bold",
        color=color,
        zorder=8,
    )


def arrow(ax, start, end, *, color=C_ARROW, zorder=6, lw=1.4):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=11,
            linewidth=lw,
            color=color,
            zorder=zorder,
            shrinkA=0,
            shrinkB=0,
        )
    )


def expansion_arrow(ax, src: tuple[float, float], dst: tuple[float, float], *, t0=0.18, t1=0.78):
    """Arrow from current-node center toward a kept neighbor, clear of glyphs."""
    x0, y0 = src
    x1, y1 = dst
    arrow(
        ax,
        (lerp(x0, x1, t0), lerp(y0, y1, t0)),
        (lerp(x0, x1, t1), lerp(y0, y1, t1)),
    )


def draw_frame(ax):
    ax.add_patch(
        Rectangle((0, 0), 3, 3, facecolor="none", edgecolor="#9A9A9A", lw=0.7, zorder=5)
    )


def draw_cardinal_panel(ax):
    setup_panel(ax, "(a)", "直线移动")

    layout = {
        (0, 0): ("pruned", None),
        (0, 1): ("obstacle", None),
        (0, 2): ("forced", "F"),
        (1, 0): ("parent", None),
        (1, 1): ("current", "x"),
        (1, 2): ("natural", "N"),
        (2, 0): ("pruned", None),
        (2, 1): ("pruned", None),
        (2, 2): ("pruned", None),
    }
    # Faces first, arrows next, labels last so glyphs stay on top
    for (r, c), (kind, lab) in layout.items():
        draw_cell(ax, r, c, kind, None)
        if kind == "obstacle":
            draw_obstacle_marks(ax, r, c)

    draw_frame(ax)

    # On-grid expansion: current x → natural N
    expansion_arrow(ax, cell_center(1, 1), cell_center(1, 2))

    for (r, c), (kind, lab) in layout.items():
        if lab:
            draw_label(ax, r, c, kind, lab)


def draw_diagonal_panel(ax):
    setup_panel(ax, "(b)", "对角线移动")

    layout = {
        (0, 0): ("forced", "F"),
        (0, 1): ("natural", "N3"),
        (0, 2): ("natural", "N1"),
        (1, 0): ("obstacle", None),
        (1, 1): ("current", "x"),
        (1, 2): ("natural", "N2"),
        (2, 0): ("parent", None),
        (2, 1): ("pruned", None),
        (2, 2): ("pruned", None),
    }
    for (r, c), (kind, lab) in layout.items():
        draw_cell(ax, r, c, kind, None)
        if kind == "obstacle":
            draw_obstacle_marks(ax, r, c)

    draw_frame(ax)

    # On-grid expansion fan: x → N1 (diag), N2 (horiz), N3 (vert)
    src = cell_center(1, 1)
    expansion_arrow(ax, src, cell_center(0, 2))  # N1
    expansion_arrow(ax, src, cell_center(1, 2))  # N2
    expansion_arrow(ax, src, cell_center(0, 1))  # N3

    for (r, c), (kind, lab) in layout.items():
        if lab:
            draw_label(ax, r, c, kind, lab)


def add_shared_legend(fig):
    handles = [
        Rectangle((0, 0), 1, 1, facecolor=C_CURRENT, edgecolor="none", label="当前节点 x"),
        Rectangle((0, 0), 1, 1, facecolor=C_NATURAL, edgecolor="none", label="自然邻居 N"),
        Rectangle((0, 0), 1, 1, facecolor=C_FORCED, edgecolor="none", label="被迫邻居 F"),
        Rectangle((0, 0), 1, 1, facecolor=C_OBSTACLE, edgecolor="none", label="障碍物"),
        Rectangle((0, 0), 1, 1, facecolor=C_PRUNED, edgecolor="none", label="剪枝邻居"),
        Rectangle((0, 0), 1, 1, facecolor=C_PARENT_FACE, edgecolor="none", label="父节点"),
        Line2D([0], [0], color=C_ARROW, lw=1.3, marker=">", markersize=5, label="扩展方向"),
    ]
    fig.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.0),
        ncol=7,
        frameon=False,
        fontsize=7.5,
        handlelength=1.3,
        handleheight=0.85,
        columnspacing=1.05,
        handletextpad=0.4,
    )


def main():
    fig = plt.figure(figsize=(7.1, 2.95))
    gs = fig.add_gridspec(
        1,
        2,
        left=0.05,
        right=0.98,
        top=0.97,
        bottom=0.28,
        wspace=0.18,
    )
    ax_a = fig.add_subplot(gs[0, 0])
    ax_b = fig.add_subplot(gs[0, 1])

    draw_cardinal_panel(ax_a)
    draw_diagonal_panel(ax_b)
    add_shared_legend(fig)

    fig.text(
        0.5,
        0.18,
        "仅保留自然邻居与因障碍物产生的被迫邻居，其余邻居被剪枝",
        ha="center",
        va="center",
        fontsize=7.5,
        color=C_NOTE,
    )

    require_matplotlib_panel_alignment(
        fig,
        json_out=f"{STEM}.alignment.json",
        overlay_svg=f"{STEM}.alignment.svg",
        tolerance_pt=1.5,
        gutter_tolerance_pt=1.5,
        strict=True,
    )

    fig.savefig(f"{STEM}.svg", bbox_inches="tight")
    fig.savefig(f"{STEM}.png", dpi=300, bbox_inches="tight")
    fig.savefig(f"{STEM}.tiff", dpi=600, bbox_inches="tight")
    # Write PDF via a temp name so a locked preview of the previous PDF cannot block export
    tmp_pdf = f"{STEM}.tmp.pdf"
    fig.savefig(tmp_pdf, bbox_inches="tight")
    try:
        Path(tmp_pdf).replace(f"{STEM}.pdf")
    except PermissionError:
        Path(tmp_pdf).replace(f"{STEM}-ongrid.pdf")
        print(f"PDF locked; wrote {STEM}-ongrid.pdf instead")
    plt.close(fig)
    print(f"Wrote {STEM}.pdf / .svg / .png / .tiff")


if __name__ == "__main__":
    main()
