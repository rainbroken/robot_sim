#!/usr/bin/env python3
"""Read-only house-map checks against the shipped collision geometry.

Ground truth is used ONLY for scoring, never to build or modify a SLAM map.
Assumes the house spawn-aligned map frame; does not fit/align the result to truth.
Unknown is kept separate from occupied and free. Surface recall includes unseen
surfaces, so inspect the per-object results rather than treating it as accuracy.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy import ndimage
import yaml


def evaluate(directory, layout_path, output):
    directory, output = Path(directory), Path(output)
    meta = yaml.safe_load((directory / 'occupancy.yaml').read_text())
    pixels = np.array(Image.open(directory / meta['image']).convert('L'))
    res = float(meta['resolution'])
    if meta['origin'][2] != 0 or meta.get('negate', 0) != 0:
        raise ValueError('This evaluator requires an unrotated, non-negated map')
    # Work in map/world coordinates with y increasing upwards.
    pixels = np.flipud(pixels)
    probability = 1.0 - pixels / 255.0
    occupied = probability > float(meta['occupied_thresh'])
    free = probability < float(meta['free_thresh'])
    unknown = ~(occupied | free)
    xs = meta['origin'][0] + (np.arange(pixels.shape[1]) + .5) * res
    ys = meta['origin'][1] + (np.arange(pixels.shape[0]) + .5) * res
    x, y = np.meshgrid(xs, ys)
    layout = json.loads(Path(layout_path).read_text())
    truth = np.zeros(pixels.shape, bool)
    furniture = np.zeros(pixels.shape, bool)
    objects = []
    for part in layout['parts']:
        if not part['collision']:
            continue
        px, py, pz = part['pose']
        sx, sy, sz = part['size']
        if pz + sz / 2 < .08 or pz - sz / 2 > .85:
            continue
        mask = (abs(x - px) <= sx / 2) & (abs(y - py) <= sy / 2)
        truth |= mask
        is_furniture = not part['name'].startswith(
            ('outer_', 'north_', 'south_', 'floor'))
        if is_furniture:
            furniture |= mask
            objects.append((part['name'], mask))
    truth_distance = ndimage.distance_transform_edt(~truth) * res
    observed_distance = ndimage.distance_transform_edt(~occupied) * res
    boundary = truth & ~ndimage.binary_erosion(truth)
    furniture_boundary = furniture & ~ndimage.binary_erosion(furniture)
    # Tolerance accommodates grid quantization and modest SLAM drift; it is
    # explicitly NOT a measurement of sub-cell registration accuracy.
    tolerance = .15
    def fraction(mask, condition):
        return float(condition[mask].mean()) if mask.any() else None
    core = ndimage.binary_erosion(furniture, iterations=3)
    corridor = (abs(x) < 7.4) & (abs(y) < .65)
    clearance = ndimage.distance_transform_edt(free) * res
    labels, _ = ndimage.label(clearance >= .375)
    def label_at(px, py):
        ix = int(np.floor((px - meta['origin'][0]) / res))
        iy = int(np.floor((py - meta['origin'][1]) / res))
        if 0 <= iy < labels.shape[0] and 0 <= ix < labels.shape[1]:
            return int(labels[iy, ix])
        return 0
    origin_label = label_at(0, 0)
    room_reachable = {room['name']: bool(origin_label and
        label_at(*room['goal']) == origin_label) for room in layout['rooms']}
    report = {
        'map': str(directory), 'reference': str(layout_path),
        'assumption': 'spawn-aligned house frame, no best-fit alignment',
        'resolution_m': res, 'width': pixels.shape[1], 'height': pixels.shape[0],
        'area_m2': {'occupied': float(occupied.sum() * res**2),
                    'free': float(free.sum() * res**2),
                    'unknown': float(unknown.sum() * res**2)},
        'surface_tolerance_m': tolerance,
        'all_surface_recall': fraction(boundary, observed_distance <= tolerance),
        'furniture_surface_recall': fraction(furniture_boundary, observed_distance <= tolerance),
        'occupied_outside_reference_tolerance_fraction': fraction(occupied, truth_distance > tolerance),
        'furniture_core_false_free_fraction': fraction(core, free),
        'furniture_core_unknown_fraction': fraction(core, unknown),
        'corridor_free_fraction': fraction(corridor, free),
        'room_goals_connected_at_radius_0375m': room_reachable,
        'objects': {name: {
            'surface_recall': fraction(mask & ~ndimage.binary_erosion(mask),
                                       observed_distance <= tolerance),
            'false_free_fraction': fraction(mask, free),
            'unknown_fraction': fraction(mask, unknown),
        } for name, mask in objects},
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharex=True, sharey=True)
    extent = [xs[0]-res/2, xs[-1]+res/2, ys[0]-res/2, ys[-1]+res/2]
    axes[0].imshow(pixels, origin='lower', cmap='gray', vmin=0, vmax=255, extent=extent)
    axes[0].set_title('Saved SLAM occupancy (gray = unknown)')
    axes[1].imshow(pixels, origin='lower', cmap='gray', vmin=0, vmax=255, extent=extent)
    axes[1].contour(x, y, truth.astype(float), [.5], colors=['tab:blue'], linewidths=.6)
    axes[1].set_title('Blue: reference collision geometry (evaluation only)')
    for ax in axes:
        ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    fig.tight_layout(); fig.savefig(output.with_suffix('.png'), dpi=150); plt.close(fig)
    print(json.dumps({k: v for k, v in report.items() if k != 'objects'}, indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_directory')
    parser.add_argument('--layout', default='src/robot_sim/worlds/family_house.layout.json')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    evaluate(args.map_directory, args.layout, args.output)
