#!/usr/bin/env python3
"""Debug and paper figures for the planning loop.

Every figure shares the base layers from common.style (draw_world /
draw_roadmap / draw_regions), so obstacles, goals, robots, and collision
regions look identical everywhere and robot i keeps one color across
all figures and the animation. Each function adds only its own layer.

Figure files keep their historical names under figure/ so existing
habits (and scripts) still find them.
"""
from pathlib import Path

import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point

from common.animation import Animation  # re-export (public API)
from common.style import (C, draw_path, draw_regions, draw_roadmap,
                          draw_transients, draw_world, new_fig, robot_color,
                          save, shapely_patch, world_legend)

__all__ = ['Animation', 'plot_environment', 'plot_prm',
           'plot_prm_one', 'plot_plan', 'plot_vrp_plan', 'plot_best_plan',
           'plot_collision', 'plot_collision_update', 'plot_obj_check',
           'plot_obj_final']


def _fig_path(name):
    Path('figure').mkdir(exist_ok=True)
    return 'figure/%s' % name


def plot_environment(env, t, figsize=None):
    fig, ax = new_fig('Environment', env)
    draw_world(ax, env, labels=True)
    save(fig, _fig_path('%d_environment.png' % t))
    return ax


def plot_prm(env, samples, roadmap, goals, figsize=None):
    """One figure per distinct robot radius (dict keyed by radius)."""
    axes = {}
    for key, val in env.robots_map.items():
        rr = val.robot_radius
        if rr in axes:
            continue
        fig, ax = new_fig('PRM %.2f' % rr, env)
        draw_roadmap(ax, samples[rr], roadmap[rr])
        draw_world(ax, env, goals, labels=True)
        world_legend(ax, roadmap=True)
        save(fig, _fig_path('init_PRM_%0.2f.png' % rr))
        axes[rr] = ax
    return axes


def plot_prm_one(env, samples, roadmap, goals, figsize=None):
    fig, ax = new_fig('PRM', env)
    draw_roadmap(ax, samples, roadmap)
    draw_world(ax, env, labels=True)
    save(fig, _fig_path('PRM1.png'))
    return ax


_PLAN_NAMES = {0: 'init', 1: 'early', 2: 'late', 3: 'const'}
_VRP_NAMES = {0: 'normal', 1: 'way', 2: 'act', 3: 'order', 4: 'help',
              5: 'way_ord', 6: 'act_ord', 7: 'all'}


def _plot_paths(figname, env, goals, robot_path, filename, roadmap=None,
                waypoint_labels=None):
    fig, ax = new_fig(figname, env)
    if roadmap is not None:
        draw_roadmap(ax, *roadmap)
    draw_world(ax, env, goals, labels=True)
    if waypoint_labels:
        _draw_waypoints(ax, waypoint_labels)
    for key, val in env.robots_map.items():
        if key in robot_path:
            path = robot_path[key]['path_only']
            draw_path(ax, path, val.index)
    save(fig, _fig_path(filename))
    return ax


def plot_plan(env, makeprob, goals, robot_path, count, early, figsize=None):
    rr = max(v.robot_radius for v in env.robots_map.values())
    name = _PLAN_NAMES.get(early, str(early))
    ax = _plot_paths('Plan', env, goals, robot_path,
                     '%dth_4_%s_plan_%0.2f.png' % (count, name, rr),
                     roadmap=(makeprob.samples[rr], makeprob.roadmaps[rr]),
                     waypoint_labels=makeprob.obj_ins)
    return {rr: ax}


def plot_vrp_plan(env, makeprob, goals, robot_path, count, early, figsize=None):
    rr = max(v.robot_radius for v in env.robots_map.values())
    name = _VRP_NAMES.get(early, str(early))
    ax = _plot_paths('Plan', env, goals, robot_path,
                     '%dth_4_%s_plan_%0.2f.png' % (count, name, rr),
                     roadmap=(makeprob.samples[rr], makeprob.roadmaps[rr]),
                     waypoint_labels=makeprob.obj_ins)
    return {rr: ax}


def plot_best_plan(env, obj, goals, robot_path, count, figsize=None):
    rr = max(v.robot_radius for v in env.robots_map.values())
    ax = _plot_paths('Best Plan', env, goals, robot_path,
                     '%dth_4_best_plan_%0.2f.png' % (count, rr),
                     waypoint_labels=obj)
    return {rr: ax}


def plot_collision(env, prm, collision, goals, count, figsize=None):
    """Raw collision report of one iteration: where the paths clashed."""
    fig, ax = new_fig('Collisions', env)
    draw_roadmap(ax, prm.samples[prm.max_rr], prm.roadmaps[prm.max_rr])
    draw_world(ax, env, goals, labels=True)
    narrow = normal = False
    for col in collision:
        col_type = col[-1] if col and isinstance(col[-1], int) else 1
        color = C['narrow'] if col_type == 0 else C['normal']
        narrow |= col_type == 0
        normal |= col_type != 0
        for s in col[0]:
            ax.add_patch(plt.Circle(s, 0.2, facecolor=color,
                                    edgecolor='none', alpha=0.35, zorder=2))
    world_legend(ax, narrow=narrow, normal=normal, roadmap=True)
    save(fig, _fig_path('%dth_1_col_check.png' % count))
    return ax


def plot_collision_update(env, prm, col_env, goals, count, figsize=None,
                          transients=None):
    """Accumulated collision regions after merging; optionally the
 transient crossings that were deliberately NOT carved."""
    fig, ax = new_fig('Collision Update', env)
    draw_world(ax, env, goals, labels=True)
    draw_regions(ax, col_env)
    if transients:
        draw_transients(ax, transients)
    world_legend(ax,
                 narrow=any(c[2] == 0 for c in col_env),
                 normal=any(c[2] != 0 for c in col_env),
                 transient=bool(transients),
                 entry=any(len(e) > 1 for c in col_env for e in c[0]))
    save(fig, _fig_path('%dth_2_col_update.png' % count))
    return ax


def _draw_waypoints(ax, obj_ins):
    """Task abstraction layer: waypoints with type-coded styling."""
    for ele in obj_ins.get('waypoint', []):
        if not ele.loc:
            continue
        if getattr(ele, 'instype', '') == 'col_way':
            ax.add_patch(plt.Circle(ele.loc, 0.07, facecolor=C['entry'],
                                    edgecolor='none', zorder=3))
        ax.text(ele.loc[0], ele.loc[1] + 0.18, ele.ind, ha='center',
                va='center', fontsize=6, color=C['text'], zorder=9)
    for ele in obj_ins.get('robot', []):
        if ele.loc:
            ax.text(ele.loc[0], ele.loc[1] - 0.45, ele.ind, ha='center',
                    va='center', fontsize=7,
                    color=robot_color(int(ele.ind[-2:])
                                      if ele.ind[-2:].isdigit() else 0),
                    zorder=9)


def _obj_graph(ax, way_list):
    segs_x, segs_y = [], []
    for i in range(len(way_list) - 1):
        connect = way_list[i].connected
        for j in range(i + 1, len(way_list)):
            if way_list[j].ind in connect:
                segs_x += [way_list[i].loc[0], way_list[j].loc[0], None]
                segs_y += [way_list[i].loc[1], way_list[j].loc[1], None]
    if segs_x:
        ax.plot(segs_x, segs_y, color=C['waypoint'], lw=0.7, zorder=1)


def plot_obj_check(env, makeprob, goals, count, figsize=None):
    fig, ax = new_fig('Task Domain', env)
    draw_world(ax, env, goals, labels=False)
    _obj_graph(ax, makeprob.obj_ins['waypoint'])
    _draw_waypoints(ax, makeprob.obj_ins)
    save(fig, _fig_path('%dth_3_obj_check.png' % count))
    return ax


def plot_obj_final(env, instant, goals, count, figsize=None):
    fig, ax = new_fig('Task Domain', env)
    draw_world(ax, env, goals, labels=False)
    _obj_graph(ax, instant['waypoint'])
    _draw_waypoints(ax, instant)
    save(fig, _fig_path('%dth_3_obj_check.png' % count))
    return ax
