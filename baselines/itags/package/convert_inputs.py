#!/usr/bin/env python3
"""Conservative MRMG YAML -> official static ITAGS JSON smoke-test adapter.

Preserves capability requirements, co-located task identity, static service
durations and task precedence. Uses one polygon centroid per region and a new
OMPL planner, so these are integration probes, not matched PRM benchmarks.
Supports robot-dependent service durations through an explicit C++ Task extension.
Refuses portable-tool problems.
"""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def csv(value):
    return [s.strip() for s in value.split(',')]


def polygon_centroid(corners):
    points = [tuple(map(float, xy)) for xy in corners]
    if points[0] == points[-1]:
        points = points[:-1]
    twice_area = cx = cy = 0.0
    for a, b in zip(points, points[1:] + points[:1]):
        cross = a[0]*b[1] - b[0]*a[1]
        twice_area += cross
        cx += (a[0]+b[0])*cross
        cy += (a[1]+b[1])*cross
    if abs(twice_area) < 1e-12:
        raise ValueError('Degenerate polygon')
    return cx/(3*twice_area), cy/(3*twice_area)


def configuration(xy):
    return dict(configuration_type='ompl', goal_type='state',
                state_space_type='se2', x=float(xy[0]), y=float(xy[1]), yaw=0.0)


def convert(source, outdir):
    source, outdir = Path(source), Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    raw = yaml.safe_load(source.read_text())
    robots, goals = raw['robots']['robot'], raw['goals']['goal']
    services = sorted({s for g in goals.values() for s in csv(g['service'])})
    costs = {r: dict(zip(csv(v['service']), v['action_cost'], strict=True))
             for r, v in robots.items()}
    report = {'source': source.name,
              'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'robots': len(robots), 'tasks': sum(len(csv(g['service'])) for g in goals.values()),
              'limitations': ['Region replaced by its polygon centroid.',
                              'OMPL builds its own paths; not the original PRM.',
                              'Official static ITAGS does not enforce inter-robot path conflicts in this pipeline.',
                              'No tool possession/transfer or extra terminal/retreat semantics added.'],
              'blockers': [], 'extensions': []}
    durations = {}
    for s in services:
        by_robot = {r: float(d[s]) for r, d in costs.items() if s in d}
        if not by_robot:
            report['blockers'].append({'kind': 'no_eligible_robot', 'service': s})
        elif len(set(by_robot.values())) != 1:
            report['extensions'].append({'kind': 'robot_dependent_service_duration',
                                          'service': s, 'duration_by_robot': by_robot})
            durations[s] = min(by_robot.values())
        else:
            durations[s] = next(iter(by_robot.values()))
    if raw.get('tools') or raw.get('tool') or raw.get('tool_config'):
        report['blockers'].append({'kind': 'portable_tools_require_extension'})
    if raw['environment'].get('obstacles'):
        report['blockers'].append({'kind': 'this_probe_adapter_only_handles_empty_maps'})
    for g, val in goals.items():
        if val.get('based') != 'region' or val.get('shape') != 'polygon':
            report['blockers'].append({'kind': 'unsupported_goal_geometry', 'goal': g})
        if any(s != 'single_task' for s in csv(val.get('service_type', 'single_task'))):
            report['blockers'].append({'kind': 'non_single_task_service', 'goal': g})
    report['conversion_status'] = 'refused_inexact_model' if report['blockers'] else 'converted_probe'
    report_path = outdir / (source.stem + '.assessment.json')
    if report['blockers']:
        report_path.write_text(json.dumps(report, indent=2) + '\n')
        return report

    bounds = [float(x) for x in csv(raw['environment']['bounds'])]
    xmin, ymin, xmax, ymax = bounds
    resolution = 0.05
    width, height = round((xmax-xmin)/resolution), round((ymax-ymin)/resolution)
    if abs(width*resolution - (xmax-xmin)) > 1e-8 or abs(height*resolution - (ymax-ymin)) > 1e-8:
        raise ValueError('Map bounds are not aligned with probe raster resolution')
    map_name = source.stem + '_map'
    (outdir / (map_name + '.pgm')).write_bytes(f'P5\n{width} {height}\n255\n'.encode() + bytes([255])*(width*height))
    map_yaml = outdir / (map_name + '.yaml')
    map_yaml.write_text(yaml.safe_dump(dict(image=map_name+'.pgm', resolution=resolution,
                                          origin=[xmin, ymin, 0.0], negate=0,
                                          occupied_thresh=0.65, free_thresh=0.196)))
    j = {'alpha': 0.5,
         'motion_planners': [{'environment_parameters': {
             'configuration_type': 'ompl', 'environment_type': 'pgm',
             'yaml_filepath': str(map_yaml.resolve())},
             'mp_parameters': {'timeout': 0.1, 'simplify_path': True,
                               'simplify_path_timeout': 0.01, 'connection_range': 0.1,
                               'configuration_type': 'ompl'}, 'mp_type': 'lazy_prm'}],
         'species': [], 'robots': [], 'tasks': [], 'precedence_constraints': [],
         'itags_parameters': {'has_timeout': True, 'timeout': 100.0, 'timer_name': 'itags',
                              'save_pruned_nodes': False, 'save_closed_nodes': False},
         'scheduler_parameters': {'scheduler_type': 'cpsat', 'time_scale': 1000, 'relative_gap': 0.1,
                                  'timeout': 10.0, 'threads': 4,
                                  'compute_transition_duration_heuristic': False,
                                  'use_hierarchical_objective': True}}
    for name, r in robots.items():
        j['species'].append({'name': name+'_species', 'traits': [float(s in costs[name]) for s in services],
                             'bounding_radius': float(r['robot_radius']), 'speed': 1.0, 'mp_index': 0})
        j['robots'].append({'name': name, 'species': name+'_species',
                            'initial_configuration': configuration(r['state'][:2])})
    indices = {}
    for name, g in goals.items():
        loc = configuration(polygon_centroid(g['corners']))
        for s in csv(g['service']):
            indices[name, s] = len(j['tasks'])
            j['tasks'].append({'name': name+'_'+s, 'duration': durations[s],
                               'desired_traits': [float(t == s) for t in services],
                               'initial_configuration': loc.copy(), 'terminal_configuration': loc.copy()})
    for name, g in goals.items():
        for edge in g.get('order', []) or []:
            before, after = [s.strip() for s in edge.split('-')]
            j['precedence_constraints'].append([indices[name, before], indices[name, after]])
    for (goal_name, service), ti in indices.items():
        by_robot = {r: float(d[service]) for r, d in costs.items() if service in d}
        if len(set(by_robot.values())) > 1:
            j['tasks'][ti]['service_durations_by_robot'] = by_robot
    # Same serial upper-normalization construction used by the upstream empty-map
    # model, with max eligible service durations for the GEN extension.
    perimeter = 2*((xmax-xmin)+(ymax-ymin))
    j['worst_makespan'] = sum(2*perimeter + max(float(d[s]) for d in costs.values() if s in d)
                               for g in goals.values() for s in csv(g['service']))
    j['plan_task_indices'] = list(range(len(j['tasks'])))
    output = outdir / (source.stem + '.json')
    output.write_text(json.dumps(j, indent=2) + '\n')
    report['input_json'] = output.name
    report['precedence_edges'] = len(j['precedence_constraints'])
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('--outdir', type=Path, default=Path('inputs'))
    args = ap.parse_args()
    report = convert(args.source, args.outdir)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['conversion_status'] == 'converted_probe' else 2)
