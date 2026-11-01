"""Summarize archived gazebo_speedup results, with later runs overriding keys."""
import argparse
import csv
import json
from pathlib import Path
import statistics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('directories',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();runs={}
    for directory in args.directories:
        with (directory/'metrics.csv').open() as stream:
            for row in csv.DictReader(stream):runs[row['label'],row['case'],row['run']]=row
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/'metrics.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(next(iter(runs.values()))))
        writer.writeheader();writer.writerows(runs.values())
    summary={}
    for label,case in sorted({key[:2] for key in runs}):
        rows=[row for key,row in runs.items() if key[:2]==(label,case)]
        successes=[row for row in rows if row['success']=='True']
        result={'runs':len(rows),'failures':len(rows)-len(successes)}
        for field in ('sim_seconds','wall_seconds','distance','max_speed','stopped_sim_seconds',
                      'rotation_episodes','total_yaw_degrees','min_wall_circle_clearance','waypoints','planned_length'):
            result[field]=statistics.median(float(row[field]) for row in successes) if successes else None
        summary.setdefault(case,{})[label]=result
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    for case,labels in summary.items():
        before=labels.get('before');after=labels.get('after')
        if before and after:
            saving=100*(1-after['sim_seconds']/before['sim_seconds'])
            print(f'{case}: {before["sim_seconds"]:.3f}s -> {after["sim_seconds"]:.3f}s '
                  f'({saving:.1f}% shorter), failures={after["failures"]}/{after["runs"]}')


if __name__=='__main__':main()
