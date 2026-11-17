#!/usr/bin/env python3
"""Render saved simulation evidence without changing any ROS state."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory.resolve()
    metadata = yaml.safe_load((root/'map/occupancy.yaml').read_text())
    image = np.array(Image.open(root/'map'/metadata['image']))
    x, y, _ = metadata['origin']
    resolution = metadata['resolution']
    extent = (x, x+image.shape[1]*resolution, y, y+image.shape[0]*resolution)
    for stage in ['mapping', 'navigation']:
        path = root/(stage+'_metrics.json')
        if not path.exists():
            continue
        report = json.loads(path.read_text())
        rows = report['samples']
        fig = plt.figure(figsize=(13, 8), constrained_layout=True)
        grid = fig.add_gridspec(2, 2, width_ratios=[1.4, 1])
        ax = fig.add_subplot(grid[:, 0])
        ax.imshow(image, cmap='gray', vmin=0, vmax=255, extent=extent, origin='upper')
        for key, label, color in [('truth', 'Gazebo truth (diagnostic)', '#d64b40'),
                                  ('map', 'Estimated map pose', '#1268b2')]:
            values = np.array([r[key][:2] for r in rows if key in r])
            if len(values):
                ax.plot(values[:, 0], values[:, 1], color=color, linewidth=1.2, label=label)
        goal_paths = ([root/'mapping_nav_goals_northwest.json', root/'mapping_nav_goals_east_south.json']
                      if stage == 'mapping' else [root/'navigation_goals.json'])
        goals = [v for p in goal_paths if p.exists() for v in json.loads(p.read_text())]
        positions = {}
        for i, item in enumerate(goals, 1):
            key = tuple(item['goal'][:2])
            positions.setdefault(key, []).append((i, item.get('succeeded', False)))
        for (gx, gy), visits in positions.items():
            ax.scatter([gx], [gy], marker='x', color='#188447' if all(v[1] for v in visits) else '#ae1919', zorder=5)
            ax.annotate(','.join(str(v[0]) for v in visits), (gx, gy), xytext=(4, 4), textcoords='offset points', fontsize=8)
        ax.set(title=stage.title()+' trajectory on saved occupancy map', xlabel='Map x [m]', ylabel='Map y [m]', aspect='equal')
        ax.legend(loc='lower right', fontsize=8)
        ax.grid(alpha=.15)
        for index, (key, factor, title, unit) in enumerate([
                ('xy_error_m', 100, 'Latest-pose XY difference', 'cm'),
                ('yaw_error_rad', 180/math.pi, 'Latest-pose yaw difference', 'deg')]):
            error_ax = fig.add_subplot(grid[index, 1])
            values = [(r['wall_seconds'], r[key]*factor) for r in rows if key in r]
            if values:
                a = np.array(values)
                error_ax.plot(a[:, 0], a[:, 1], linewidth=.8, color='#1268b2')
            error_ax.set(title=title, xlabel='Wall time [s]', ylabel=unit)
            error_ax.grid(alpha=.25)
        fig.suptitle('Baobao / JUNIOR custom DWB validation — '+stage.title(), fontsize=14)
        fig.savefig(root/(stage+'_trajectory.png'), dpi=160)
        plt.close(fig)
        print(root/(stage+'_trajectory.png'))

    # PCL's saved XYZI binary map; height coloring is for this preview only.
    with (root/'map/full_map.pcd').open('rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Incomplete PCD header')
            header.append(line.strip())
            if line.startswith(b'DATA'):
                break
        if b'FIELDS x y z intensity' not in header or header[-1] != b'DATA binary':
            raise ValueError('Expected PCL binary XYZI map')
        points = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 4)
    points = points[np.isfinite(points[:, :3]).all(axis=1)]
    preview = points[(points[:, 2] > -.1) & (points[:, 2] < 2.5)][::3]
    fig = plt.figure(figsize=(12, 8), constrained_layout=True)
    ax = fig.add_subplot(111, projection='3d')
    cloud = ax.scatter(preview[:, 0], preview[:, 1], preview[:, 2], c=preview[:, 2],
                       cmap='viridis', s=.6, linewidths=0, rasterized=True)
    ax.set(xlabel='Map x [m]', ylabel='Map y [m]', zlabel='Height [m]',
           title=f'Saved 3D map: {len(points):,} XYZI points (height-colored preview)')
    ax.set_box_aspect((16, 12, 3))
    ax.view_init(elev=55, azim=-70)
    fig.colorbar(cloud, ax=ax, label='Height [m]', shrink=.5, pad=.05)
    fig.savefig(root/'pointcloud_preview.png', dpi=160)
    plt.close(fig)
    print(root/'pointcloud_preview.png')


if __name__ == '__main__':
    main()
