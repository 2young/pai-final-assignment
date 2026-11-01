"""Summarize explicit Gazebo pose-navigation trials without re-running them."""
import argparse
import csv
import json
from pathlib import Path
import re
import statistics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('root',type=Path)
    args=parser.parse_args();root=args.root;rows=[]
    for directory in ('circle','auto','reverse'):
        path=root/directory/'summary.json'
        if not path.exists():continue
        trials=json.loads(path.read_text())
        for row in trials:
            prefix=f'{row["planner"]}-{row["case"]}-{row["run"]}'
            text=(root/directory/f'{prefix}.txt').read_text()
            arrived=next((s for s in text.splitlines() if s.startswith('ARRIVED:')),'')
            for key in ('minimum_footprint_clearance','max_cross_track','max_heading_error','max_control_ms'):
                match=re.search(rf'{key}=([0-9.]+)',arrived)
                row[key]=float(match[1]) if match else None
            row['pose_follower']='controller=pose_pursuit' in text
            row['log']=str(Path(directory)/f'{prefix}.txt')
            rows.append(row)
    if not rows:raise SystemExit('No trials found')
    with (root/'metrics.csv').open('w',newline='') as output:
        writer=csv.DictWriter(output,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    grouped={}
    for row in rows:grouped.setdefault((row['planner'],row['case']),[]).append(row)
    summary={}
    for (planner,case),trials in grouped.items():
        values={'runs':len(trials),'all_successful':all(t['success'] for t in trials),
                'median_sim_seconds':statistics.median(t['sim_seconds'] for t in trials),
                'median_wall_seconds':statistics.median(t['wall_seconds'] for t in trials),
                'max_actual_speed':max(t['max_actual_speed'] for t in trials),
                'pose_follower_runs':sum(t['pose_follower'] for t in trials)}
        for metric in ('minimum_footprint_clearance','max_cross_track','max_heading_error','max_control_ms'):
            present=[t[metric] for t in trials if t[metric] is not None]
            values[metric]=(min(present) if metric=='minimum_footprint_clearance' else max(present)) if present else None
        summary[f'{planner}/{case}']=values
    for case in ('open_straight','maze'):
        before=summary[f'circle/{case}']['median_sim_seconds'];after=summary[f'auto/{case}']['median_sim_seconds']
        summary[f'auto/{case}']['time_change_percent']=100*(after/before-1)
    (root/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
