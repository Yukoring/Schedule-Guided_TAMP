#!/usr/bin/env python3
"""Sequential native C++ runs. Python only prepares inputs and captures output."""
import argparse
import json
from pathlib import Path
import subprocess
from convert_inputs import convert

ROOT = Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cases', nargs='+', default=['SP_s0', 'MP_s0'])
    ap.add_argument('--budget', type=float, default=100.0)
    ap.add_argument('--outdir', type=Path, default=ROOT/'results')
    args = ap.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=True)
    summary = []
    for case in args.cases:
        source = ROOT/'source_inputs'/(case+'.yaml')
        if source.exists():
            assessment = convert(source, ROOT/'inputs')
            if assessment['blockers']:
                raise ValueError(assessment)
        input_file = ROOT/'inputs'/(case+'.json')
        # Rehome maps for packaged diagnostic JSON inputs as well as YAML cases.
        payload = json.loads(input_file.read_text())
        for planner in payload['motion_planners']:
            env = planner['environment_parameters']
            local_map = ROOT/'inputs'/Path(env['yaml_filepath']).name
            if not local_map.exists():
                raise FileNotFoundError(local_map)
            env['yaml_filepath'] = str(local_map)
        input_file.write_text(json.dumps(payload, indent=2)+'\n')
        result_file = args.outdir/(case+'.json')
        log_file = args.outdir/(case+'.log')
        result_file.unlink(missing_ok=True)
        try:
            with log_file.open('w') as log:
                run = subprocess.run([str(ROOT/'build/itags_cpsat'), str(input_file),
                                      str(result_file), str(args.budget)], cwd=ROOT,
                                     stdout=log, stderr=subprocess.STDOUT, timeout=args.budget+5)
            result = json.loads(result_file.read_text()) if result_file.exists() else {'status': 'exited_without_result'}
            result['returncode'] = run.returncode
        except subprocess.TimeoutExpired:
            result = {'status': 'external_timeout'}
        row = {key: result[key] for key in ['status', 'returncode', 'makespan', 'elapsed_seconds',
                                           'scheduler_statistics', 'search_statistics', 'error'] if key in result}
        row['case'] = case
        summary.append(row)
        print(case, row['status'], row.get('makespan'), row.get('elapsed_seconds'), flush=True)
        (args.outdir/'run_summary.json').write_text(json.dumps(summary, indent=2)+'\n')


if __name__ == '__main__':
    main()
