"""Score against approximate bag-derived reference positions, not survey truth."""
import argparse
import collections
import datetime as dt
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
KST = dt.timezone(dt.timedelta(hours=9))


def score(data, reference):
    ref = reference.reshape(-1, 2)
    mids = reference.mean(1)
    axis = mids[-1]-mids[0]
    axis /= np.linalg.norm(axis)
    across = np.array([-axis[1], axis[0]])
    rs, ry = (mids-mids[0])@axis, (mids-mids[0])@across
    stats = collections.defaultdict(lambda: collections.defaultdict(list))
    sources = collections.defaultdict(lambda: dict(frames=0, count=0, matched=0, errors=[], observed=set()))
    baseline = dict(frames=0, follow_frames=0)
    for snap in data['snapshots']:
        boat = np.array(snap['boat'])
        heading = np.array([np.cos(snap['yaw']), np.sin(snap['yaw'])])
        # Report only forward-facing snapshots physically in the reference channel.
        station = (boat-mids[0])@axis
        if not (-2. <= station <= rs[-1]+1. and heading@axis > .25):
            continue
        baseline['frames'] += 1
        baseline['follow_frames'] += int((snap['baseline_status'] or '').startswith('FOLLOW_GATE'))
        for source, points in snap['sources'].items():
            pts = np.asarray(points, float).reshape(-1, 2)
            sm = sources[source]
            sm['frames'] += 1
            sm['count'] += len(pts)
            if len(pts):
                d = np.linalg.norm(pts[:, None]-ref[None, :], axis=2)
                nearest, err = d.argmin(1), d.min(1)
                match = err <= .8
                sm['matched'] += int(match.sum())
                sm['observed'].update(int(v) for v in nearest[match])
                sm['errors'].extend(err[match].tolist())
            for mode, w in snap['walls'][source].items():
                key = (source, mode)
                st = stats[key]
                st['track'].append(w['status'] == 'TRACK')
                st['trial'].append(snap['trial'])
                if not w['xy']:
                    continue
                xy, labels = np.array(w['xy']), np.array(w['labels'])
                d = np.linalg.norm(xy[:, None]-ref[None, :], axis=2)
                nearest, err = d.argmin(1), d.min(1)
                match = err <= .8
                accepted = labels >= 0
                valid = match & accepted
                st['known'] += (labels[valid] == nearest[valid]%2).tolist()
                st['clutter'] += (~match[accepted]).tolist()
                raw = np.array(w['raw_labels'])
                for label in set(raw):
                    bank = (nearest%2)[match & (raw == label)]
                    if len(bank) >= 2:
                        st['raw_mixed'].append(len(set(bank)) > 1)
                if w['status'] != 'TRACK':
                    continue
                # Only score the forward part supported by both observed banks.
                center = np.array(w['center'])
                u = np.linspace(-3, 6, len(center))
                keep = (u >= 0) & (u <= w['supported_end']-w['boat_s'])
                center = center[keep]
                if len(center):
                    s = (center-mids[0])@axis
                    y = (center-mids[0])@across
                    inside = (rs[0] <= s) & (s <= rs[-1])
                    errors = abs(y[inside]-np.interp(s[inside], rs, ry))
                    st['center_error'] += errors.tolist()
    rows = []
    for (source, mode), s in stats.items():
        rows.append(dict(source=source, direction=mode, frames=len(s['track']),
            supported_percent=round(100*np.mean(s['track']), 1),
            known_assignment_count=len(s['known']),
            known_side_accuracy_percent=round(100*np.mean(s['known']), 1) if s['known'] else None,
            assigned_unmatched_percent=round(100*np.mean(s['clutter']), 1) if s['clutter'] else None,
            raw_component_mixed_percent=round(100*np.mean(s['raw_mixed']), 1) if s['raw_mixed'] else None,
            center_median_m=round(float(np.median(s['center_error'])), 3) if s['center_error'] else None,
            center_p90_m=round(float(np.percentile(s['center_error'], 90)), 3) if s['center_error'] else None))
    src = {}
    for k, s in sources.items():
        src[k] = dict(frames=s['frames'], point_observations=s['count'], matched=s['matched'],
                      reference_posts_seen=len(s['observed']),
                      unmatched_percent=round(100*(1-s['matched']/max(s['count'], 1)), 1),
                      matched_position_median_m=round(float(np.median(s['errors'])), 3) if s['errors'] else None,
                      matched_position_p90_m=round(float(np.percentile(s['errors'], 90)), 3) if s['errors'] else None)
    return dict(warning='References are approximate positions from the same bag; snapshots are correlated. This is open-loop replay, not navigation success.',
                reference_match_radius_m=.8, sources=src, walls=rows, baseline=baseline)


def draw(data, reference, out):
    frames = [s for s in data['snapshots'] if s['trial'] == 4]
    # Fixed recorded positions, selected independently of method fit error.
    checkpoints = [min(frames, key=lambda s: abs(s['boat'][0]-x)+max(0, -np.cos(s['yaw']))*100)
                   for x in (113., 118., 123., 128.)]
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.subplots_adjust(top=.87, bottom=.13, hspace=.36, wspace=.25)
    bankcols = ['#c64eb1', '#00a6b2']
    for ax, snap in zip(axes.ravel(), checkpoints):
        for k in (0, 1):
            ax.plot(reference[:, k, 0], reference[:, k, 1], ':', color='#aaaaaa', lw=1)
            ax.scatter(reference[:, k, 0], reference[:, k, 1], marker='o', facecolors='none',
                       edgecolors='#888888', s=80, zorder=1)
        pts = np.array(snap['sources']['grid']).reshape(-1, 2)
        if len(pts): ax.scatter(*pts.T, c='#bbbbbb', s=12, alpha=.65, label='Grid candidates')
        for source, style, alpha in [('grid', '-', .9), ('detector', '--', .9)]:
            w = snap['walls'][source]['adaptive']
            for side, key in enumerate(('left', 'right')):
                line = np.array(w[key])
                if len(line): ax.plot(*line.T, style, color=bankcols[side],
                                     alpha=alpha if w['status'] == 'TRACK' else .35,
                                     lw=2, label=f'{source} {key}')
        path = np.array(snap['baseline_path'])
        if len(path): ax.plot(*path.T, color='#222222', lw=1.5, label='Competition path')
        x, y = snap['boat']
        ax.scatter([x], [y], c='#214bdb', marker='s', s=65, zorder=5)
        ax.arrow(x, y, 1.2*np.cos(snap['yaw']), 1.2*np.sin(snap['yaw']), width=.08, color='#214bdb', zorder=5)
        stamp = dt.datetime.fromtimestamp(snap['t'], KST).strftime('%H:%M:%S')
        gs, ds = (snap['walls'][source]['adaptive']['status'] for source in ('grid', 'detector'))
        ax.set_title(f'{stamp} | grid: {gs}\ndetector: {ds}', fontsize=10)
        ax.set_xlim(x-5, x+8); ax.set_ylim(y-5, y+5)
        ax.set_aspect('equal'); ax.grid(alpha=.2)
        ax.set_xlabel('odom X (m)'); ax.set_ylabel('odom Y (m)')
    handles, labels = axes.ravel()[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.5, .015), ncol=4, fontsize=9)
    fig.suptitle('Recorded gate run: two-wall hypotheses vs competition path\nSolid = grid, dashed = buoy detector. Dotted references used for evaluation only.', fontsize=13)
    fig.savefig(out/'wall_comparison.png', dpi=150)
    plt.close(fig)
    json.dump(checkpoints, open(out/'checkpoints.json', 'w'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('replay')
    ap.add_argument('out')
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    data = json.load(gzip.open(a.replay, 'rt'))
    reference = np.array(json.load(open(ROOT/'scripts/gate_sim/gates_20261112.json'))['gates'])
    report = score(data, reference)
    json.dump(report, open(out/'metrics.json', 'w'), indent=2)
    print(json.dumps(report, indent=2), flush=True)
    draw(data, reference, out)
    preview = []
    for frame in data['snapshots']:
        if frame['trial'] != 4:
            continue
        frame = dict(frame)
        frame['walls'] = {source: {'adaptive': frame['walls'][source]['adaptive']}
                          for source in ('grid', 'detector')}
        preview.append(frame)
    with gzip.open(out/'preview.json.gz', 'wt') as f:
        json.dump(dict(snapshots=preview), f)


if __name__ == '__main__':
    main()
