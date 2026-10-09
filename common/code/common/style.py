"""Shared visual style for every figure and the animation.

One semantic palette and one world base layer: every figure draws the
world identically (same colors, same z-order) and adds only its own
layer on top. Robot i keeps the same color in every figure and video.

Layer order (zorder): roadmap 0.5 < samples 1 < regions 2-3 < obstacles 4
< goal regions 5 < goal samples 6 < paths 7 < robots 8 < labels 9.
"""
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon as MplPolygon

# Colorblind-safe categorical palette (matplotlib tab10): robot i is the
# same color everywhere - figures, filmstrips, and the animation.
ROBOT_COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

# Semantic colors: one meaning, one color, in every figure.
C = {
    'obstacle':   '#5a5a5a',
    'obstacle_e': '#3d3d3d',
    'goal_face':  '#ffffff',
    'goal_edge':  '#2f2f2f',
    'goal_dot':   '#2f2f2f',
    'roadmap':    '#e0e0e0',
    'sample':     '#cccccc',
    'waypoint':   '#b0b0b0',
    'narrow':     '#d62728',   # structural narrow region (carved, token)
    'normal':     '#ff9214',   # normal collision region (carved)
    'transient':  '#1f77b4',   # transient crossing (NOT carved - dashed)
    'entry':      '#7b2d8b',   # region entry / waiting waypoints
    'text':       '#333333',
}

TASK_MARKERS = ['s', '*', 'o', '^', 'D', 'v', 'P', 'X']  # task k -> marker


def robot_color(i):
    return ROBOT_COLORS[i % len(ROBOT_COLORS)]


def task_marker(k):
    return TASK_MARKERS[k % len(TASK_MARKERS)]


def shapely_patch(geom, fc, ec=None, alpha=1.0, zorder=1, lw=0.8, hatch=None):
    """matplotlib patch from a shapely polygon (descartes replacement)."""
    return MplPolygon(np.asarray(geom.exterior.coords), closed=True,
                      facecolor=fc, edgecolor=ec or fc, alpha=alpha,
                      zorder=zorder, linewidth=lw, hatch=hatch)


def new_fig(name, env, pad=0.0):
    """Figure sized from the world bounds; clean axes (no ticks)."""
    minx, miny, maxx, maxy = env.bounds
    aspect = (maxx - minx) / max(maxy - miny, 1e-9)
    fig = plt.figure(name, figsize=(7 * aspect, 7))
    ax = fig.add_subplot(111)
    ax.clear()
    ax.set_xlim(minx - pad, maxx + pad)
    ax.set_ylim(miny - pad, maxy + pad)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color('#bbbbbb')
    return fig, ax


def draw_world(ax, env, goals=None, labels=False):
    """Base layer: obstacles, goal regions, goal samples, robots."""
    for name, obs in env.obstacles_map.items():
        ax.add_patch(shapely_patch(obs, C['obstacle'], C['obstacle_e'],
                                   zorder=4))
    for gi, (name, goal) in enumerate(env.goals_map.items()):
        if goal['based'] == 'region':
            ax.add_patch(shapely_patch(goal['shape'], C['goal_face'],
                                       C['goal_edge'], zorder=5, lw=1.2))
            gminx, gminy, gmaxx, gmaxy = goal['shape'].bounds
            ax.text(gminx + 0.08, gmaxy - 0.08, 'g%d' % gi, ha='left',
                    va='top', fontsize=7, color=C['goal_edge'], zorder=9)
    if goals:
        for ele in goals:
            for g in ele:
                ax.add_patch(Circle(g, 0.09, facecolor='white',
                                    edgecolor=C['goal_dot'], lw=0.9, zorder=6))
    for name, robot in env.robots_map.items():
        col = robot_color(robot.index)
        ax.add_patch(Circle(robot.pos, robot.robot_radius, facecolor=col,
                            edgecolor='#222222', lw=0.8, zorder=8))
        if labels:
            ax.text(robot.pos[0], robot.pos[1] - robot.robot_radius - 0.25,
                    'r%d' % robot.index, ha='center', va='top', fontsize=8,
                    color=col, zorder=9)
    return ax


def draw_roadmap(ax, samples, roadmap):
    """Hairline roadmap behind everything."""
    segs_x, segs_y = [], []
    for i, nbrs in enumerate(roadmap):
        for j in nbrs:
            segs_x += [samples[i][0], samples[j][0], None]
            segs_y += [samples[i][1], samples[j][1], None]
    if segs_x:
        ax.plot(segs_x, segs_y, color=C['roadmap'], lw=0.5, zorder=0.5)
    if samples:
        xs, ys = zip(*samples)
        ax.plot(xs, ys, '.', color=C['sample'], markersize=2, zorder=1)


def draw_regions(ax, col_env):
    """Carved collision regions: narrow red / normal orange, translucent
 sample blobs; entry and waiting points as outlined circles."""
    for col in col_env:
        entry, samples, col_type = col[0], col[1], col[2]
        color = C['narrow'] if col_type == 0 else C['normal']
        for s in samples:
            ax.add_patch(Circle(s, 0.22, facecolor=color, edgecolor='none',
                                alpha=0.25, zorder=2))
        for ele in entry:
            ax.add_patch(Circle(ele[0], 0.16, facecolor='none',
                                edgecolor=color, lw=1.4, zorder=3))
            for p in ele[1:]:
                ax.add_patch(Circle(p, 0.16, facecolor='none',
                                    edgecolor=C['entry'], lw=1.4, zorder=3))


def draw_transients(ax, transients):
    """Transient crossings (not carved): dashed blue circles at centers."""
    for m in transients:
        r = 0.25 + 0.05 * math.sqrt(max(len(m.get('samples', [])), 1))
        ax.add_patch(Circle(m['center'], r, facecolor='none',
                            edgecolor=C['transient'], lw=1.4,
                            linestyle='--', zorder=3))


def draw_path(ax, path, robot_index, lw=2.2, alpha=0.95):
    if len(path) < 2:
        return
    xs, ys = zip(*[(p[0], p[1]) for p in path])
    ax.plot(xs, ys, color=robot_color(robot_index), lw=lw, alpha=alpha,
            solid_capstyle='round', zorder=7)


def world_legend(ax, narrow=False, normal=False, transient=False,
                 entry=False, roadmap=False):
    """Compact legend for the collision layers actually present."""
    items = []
    if narrow:
        items.append((Line2D([], [], marker='o', color='none',
                             markerfacecolor=C['narrow'], alpha=0.5,
                             markersize=9), 'narrow region'))
    if normal:
        items.append((Line2D([], [], marker='o', color='none',
                             markerfacecolor=C['normal'], alpha=0.5,
                             markersize=9), 'normal region'))
    if transient:
        items.append((Line2D([], [], marker='o', color=C['transient'],
                             linestyle='--', markerfacecolor='none',
                             markersize=9), 'transient (not carved)'))
    if entry:
        items.append((Line2D([], [], marker='o', color=C['entry'],
                             linestyle='none', markerfacecolor='none',
                             markersize=9), 'entry / waiting'))
    if roadmap:
        items.append((Line2D([], [], color=C['roadmap'], lw=1), 'roadmap'))
    if items:
        ax.legend(*zip(*[(h, l) for h, l in items]), loc='upper right',
                  fontsize=7, framealpha=0.9, borderpad=0.4,
                  handletextpad=0.5)


def save(fig, path, dpi=140):
    fig.savefig(path, dpi=dpi, bbox_inches='tight', pad_inches=0.05)
