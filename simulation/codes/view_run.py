#!/usr/bin/env python3
"""Top-down visualiser for a discrete MockWorld run.

Produces, in simulation/runs/:
    <mission>_seed<N>.png   static storyboard: arena, zones, bricks, path, step table
    <mission>_seed<N>.gif   animated robot moving along the named points

This visualises the *discrete* model (Level 0A): positions are named points, motion is
interpolated for display only. It is not physics and not the spatial world (Level 0B).

Usage:
    python3 view_run.py --seed 3
    python3 view_run.py --seed 5 --no-gif
"""

from __future__ import annotations

import argparse
import io
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from PIL import Image

from mock_adapter import run

TYPE_COLORS = {
    "doll": (0.55, 0.35, 0.20),
    "clock": (0.85, 0.85, 0.85),
    "banana": (0.95, 0.85, 0.15),
    "cake": (0.90, 0.30, 0.40),
}
ZONE_FILL = {"A": "#fff3c4", "D": "#fff3c4", "B": "#dcf2dc", "C": "#dcf2dc"}
ARENA = 1.5


# ---------------------------------------------------------------------------
# Data extraction
# ---------------------------------------------------------------------------


def event_sequence(mock, scenario):
    """Reconstruct the robot's positions from the mock's own event log."""
    sp = scenario["start_pose"]
    x, y, yaw = float(sp["x"]), float(sp["y"]), float(sp["yaw"])
    points = {k: (float(v[0]), float(v[1])) for k, v in scenario["points"].items()}
    seq = [{"t": 0.0, "what": "start", "name": "start in A", "x": x, "y": y, "yaw": yaw}]
    for e in mock.log:
        what = e["what"]
        if what == "goto_local" and e.get("ok") and e.get("name") in points:
            nx, ny = points[e["name"]]
            if (nx, ny) != (x, y):
                yaw = math.atan2(ny - y, nx - x)
            x, y = nx, ny
            seq.append({"t": e["t"], "what": "goto", "name": e["name"], "x": x, "y": y, "yaw": yaw})
        elif what == "pick":
            seq.append({"t": e["t"], "what": "pick", "name": e.get("ty"), "ok": e.get("ok"),
                        "x": x, "y": y, "yaw": yaw})
        elif what == "drop":
            seq.append({"t": e["t"], "what": "drop", "name": e.get("zone"), "ty": e.get("ty"),
                        "x": x, "y": y, "yaw": yaw})
        elif what == "read_board":
            seq.append({"t": e["t"], "what": "read_board", "name": "", "x": x, "y": y, "yaw": yaw})
    return seq


# ---------------------------------------------------------------------------
# Static scene
# ---------------------------------------------------------------------------


def draw_static(ax, scenario):
    ax.add_patch(Rectangle((-ARENA, -ARENA), 2 * ARENA, 2 * ARENA,
                           fill=False, lw=2.0, color="black", zorder=6))

    for zone, info in (scenario.get("zones") or {}).items():
        tx, ty = info["transport_center"]
        cx, cy = info["collection_center"]
        ax.add_patch(Rectangle((tx - 0.5, ty - 0.5), 1.0, 1.0,
                               facecolor=ZONE_FILL.get(zone, "#f4f4f4"),
                               edgecolor="#bcbcbc", lw=0.8, zorder=0))
        # every collection square is the red scoring square
        ax.add_patch(Rectangle((cx - 0.225, cy - 0.225), 0.45, 0.45, facecolor="none",
                               edgecolor="#d62728", lw=1.6, zorder=2))
        ax.text(cx, cy - 0.33, "Zone " + zone, ha="center", fontsize=8, color="#444444")

    # sorting table on the west wall + the three board cards
    ax.add_patch(Rectangle((-1.5, -0.20), 0.33, 0.40, facecolor="#d0d0d0",
                           edgecolor="#666666", lw=0.8, zorder=1))
    ax.text(-1.335, 0.26, "table", ha="center", fontsize=7)
    for i, ty in enumerate(scenario.get("board_left_to_right") or []):
        y = (i - 1) * 0.12
        ax.add_patch(Rectangle((-1.385, y - 0.05), 0.10, 0.10,
                               facecolor=TYPE_COLORS.get(ty, (0.5, 0.5, 0.5)),
                               edgecolor="black", lw=0.6, zorder=3))
    ax.text(-1.335, -0.30, "board L→R", ha="center", fontsize=7)

    # first-tier bricks; the cake sits on top of the centre brick
    for brick in scenario.get("bricks") or []:
        if brick.get("layer") == 2:
            continue
        ax.add_patch(Rectangle((brick["x"] - 0.05, brick["y"] - 0.05), 0.10, 0.10,
                               facecolor=TYPE_COLORS.get(brick["type"], (0.5, 0.5, 0.5)),
                               edgecolor="black", lw=0.5, zorder=3))
    cake = next((b for b in scenario.get("bricks") or [] if b["type"] == "cake"), None)
    if cake:
        ax.scatter([cake["x"]], [cake["y"]], marker="*", s=170, color=TYPE_COLORS["cake"],
                   edgecolor="black", linewidth=0.6, zorder=4)
        ax.text(0.0, -0.30, "cake (L2)", ha="center", fontsize=7)

    for name, (px, py) in (scenario.get("points") or {}).items():
        if name.endswith("_approach"):
            ax.plot(px, py, "o", ms=3.0, mfc="none", mec="#1f77b4", zorder=2)
            continue
        ax.plot(px, py, "o", ms=3.0, color="#1f77b4", zorder=2)
        ax.text(px, py + 0.07, name.replace("_", " "), ha="center", fontsize=6,
                color="#1f77b4")

    sp = scenario.get("start_pose") or {}
    if sp:
        ax.add_patch(Circle((sp["x"], sp["y"]), 0.07, facecolor="#ff7f0e",
                            edgecolor="black", zorder=5))
        ax.text(sp["x"], sp["y"] - 0.16, "START", ha="center", fontsize=7,
                color="#ff7f0e", weight="bold")

    ax.set_xlim(-1.62, 1.62)
    ax.set_ylim(-1.62, 1.62)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.25, linestyle=":")


def draw_path(ax, seq, alpha=0.9):
    distinct = [e for e in seq if e["what"] in ("start", "goto")]
    for i in range(1, len(distinct)):
        a, b = distinct[i - 1], distinct[i]
        if (a["x"], a["y"]) == (b["x"], b["y"]):
            continue
        ax.annotate("", xy=(b["x"], b["y"]), xytext=(a["x"], a["y"]),
                    arrowprops=dict(arrowstyle="->", color="#9467bd", lw=1.2,
                                    linestyle="--", alpha=alpha))
        mx = a["x"] + (b["x"] - a["x"]) * 0.38
        my = a["y"] + (b["y"] - a["y"]) * 0.38
        ax.text(mx, my, str(i), fontsize=7, color="#9467bd", alpha=alpha,
                ha="center", va="center",
                bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec="none", alpha=0.7))


def draw_events(ax, seq):
    """Mark pick and drop events on the storyboard."""
    for event in seq:
        if event["what"] == "pick":
            ok = "ok" if event.get("ok") else "FAILED"
            ax.text(event["x"], event["y"] + 0.13, "PICK %s (%s)" % (event.get("name"), ok),
                    ha="center", fontsize=7, color="#d62728", weight="bold")
        elif event["what"] == "drop":
            ax.text(event["x"], event["y"] - 0.16,
                    "DROP %s -> %s" % (event.get("ty"), event.get("name")),
                    ha="center", fontsize=7, color="#2ca02c", weight="bold")


# ---------------------------------------------------------------------------
# Report panel
# ---------------------------------------------------------------------------


def draw_report(tx, report, mock, mission_name):
    tx.axis("off")
    tx.set_xlim(0, 1)
    tx.set_ylim(0, 1)

    score, detail = mock.score(overtime_s=max(0.0, mock.t - 600))
    lines = [
        "MISSION  %s" % mission_name,
        "state    %s %s" % (report.state, report.reason),
        "clock    %.0f s left" % report.clock_remaining,
        "board    %s  (L -> R)" % ", ".join(mock.sc["board_left_to_right"]),
        "mapping  %s" % ", ".join("%s->%s" % kv for kv in sorted(report.mapping.items())),
        "score    %d   (mock time %.0f s)" % (score, mock.t),
        "",
        "STEPS",
    ]
    for r in report.records:
        lines.append(" %d. %-10s %-10s %s" % (r.index + 1, r.kind, r.outcome.value, r.reason))
    lines.append("")
    lines.append("VISUALISATION NOTE")
    lines.append("Discrete model: motion between named points is")
    lines.append("interpolated for display only. No physics, no")
    lines.append("lidar, no camera. Level 0B adds those.")
    lines.append("")
    lines.append("brown=doll grey=clock yellow=banana pink=cake")
    lines.append("red square = collection zone")
    if detail:
        lines.append("")
        lines.append("scoring detail")
        for zone, ty, pts in detail:
            lines.append("  %s %-8s %d" % (zone, ty, pts))

    tx.text(0.0, 0.98, "\n".join(lines), va="top", ha="left",
            family="monospace", fontsize=8.5, transform=tx.transAxes)


# ---------------------------------------------------------------------------
# Outputs
# ---------------------------------------------------------------------------


def render_png(path, scenario, seq, report, mock, mission_name):
    fig, (ax, tx) = plt.subplots(
        1, 2, figsize=(13.0, 7.0), gridspec_kw={"width_ratios": [1.0, 1.05]}
    )
    draw_static(ax, scenario)
    draw_path(ax, seq)
    draw_events(ax, seq)
    draw_report(tx, report, mock, mission_name)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def render_gif(path, scenario, seq, report, mission_name, fps=8):
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    draw_static(ax, scenario)
    draw_path(ax, seq, alpha=0.30)

    trail_x, trail_y = [], []
    (trail,) = ax.plot([], [], "-", color="#2ca02c", lw=1.5, alpha=0.8, zorder=7)
    robot = Circle((0, 0), 0.06, facecolor="#ff7f0e", edgecolor="black", zorder=9)
    ax.add_patch(robot)
    (heading,) = ax.plot([], [], "-", color="black", lw=1.2, zorder=10)
    label = ax.text(0, 1.57, "", ha="center", fontsize=8, family="monospace")

    frames = []
    last_yaw = seq[0]["yaw"]

    def snapshot(x, y, yaw, text):
        robot.center = (x, y)
        heading.set_data([x, x + 0.13 * math.cos(yaw)], [y, y + 0.13 * math.sin(yaw)])
        trail.set_data(trail_x, trail_y)
        label.set_text(text)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=90)
        buf.seek(0)
        frames.append(Image.open(buf).convert("P", palette=Image.ADAPTIVE))

    for i, event in enumerate(seq):
        prev = seq[i - 1] if i > 0 else seq[0]
        is_move = event["what"] == "goto" and (event["x"], event["y"]) != (prev["x"], prev["y"])
        if is_move:
            steps = 8
            for k in range(1, steps + 1):
                x = prev["x"] + (event["x"] - prev["x"]) * k / steps
                y = prev["y"] + (event["y"] - prev["y"]) * k / steps
                yaw = math.atan2(event["y"] - prev["y"], event["x"] - prev["x"])
                snapshot(x, y, yaw, "t=%3.0fs  goto %s" % (event["t"], event["name"]))
            trail_x.append(event["x"])
            trail_y.append(event["y"])
            last_yaw = yaw
        else:
            for _ in range(3):
                what = event["what"]
                if what == "pick":
                    text = "t=%3.0fs  pick %s  (%s)" % (
                        event["t"], event["name"], "ok" if event.get("ok") else "failed")
                elif what == "drop":
                    text = "t=%3.0fs  drop %s -> Zone %s" % (
                        event["t"], event.get("ty"), event["name"])
                elif what == "read_board":
                    text = "t=%3.0fs  read board" % event["t"]
                else:
                    text = "t=%3.0fs  start" % event["t"]
                snapshot(event["x"], event["y"], last_yaw, text)

    if frames:
        frames[0].save(path, save_all=True, append_images=frames[1:],
                       duration=int(1000 / fps), loop=0, optimize=True)
    plt.close(fig)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--mission", default=os.path.join(here, "..", "config", "mission_demo.yaml"))
    ap.add_argument("--scenario", default=os.path.join(here, "scenario.json"))
    ap.add_argument("--out", default=os.path.join(here, "..", "runs"))
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()

    report, mock = run(args.mission, args.scenario, args.seed)
    with open(args.scenario) as f:
        import json

        scenario = json.load(f)
    mission_name = os.path.splitext(os.path.basename(args.mission))[0]
    seq = event_sequence(mock, scenario)

    os.makedirs(args.out, exist_ok=True)
    stem = "%s_seed%d" % (mission_name, args.seed)
    png_path = os.path.join(args.out, stem + ".png")
    render_png(png_path, scenario, seq, report, mock, mission_name)
    print("wrote %s" % png_path)

    if not args.no_gif:
        gif_path = os.path.join(args.out, stem + ".gif")
        render_gif(gif_path, scenario, seq, report, mission_name)
        print("wrote %s" % gif_path)


if __name__ == "__main__":
    main()
