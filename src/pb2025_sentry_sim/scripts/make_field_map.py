#!/usr/bin/env python3
"""Conservative 2D navigation map from the pinned RMUC 2025 collision STL.

Project surfaces above ground clearance through chassis height. Ramps are
deliberately obstacles: this baseline validates planar navigation only.
"""
import argparse
from pathlib import Path
import struct
import numpy as np
import yaml


def generate(mesh, output):
    data = Path(mesh).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
    # model.sdf: body pose = (14.5, 8, 0.2), mesh scale = 1.
    triangles += [14.5, 8.0, 0.2]
    resolution = 0.05
    origin = np.array([-1., -1.])
    width, height = 620, 360
    grid = np.full((height, width), 254, dtype=np.uint8)
    for triangle in triangles:
        if triangle[:, 2].max() <= 0.27 or triangle[:, 2].min() >= 0.8:
            continue
        xy = (triangle[:, :2] - origin) / resolution
        # Rasterize triangle using barycentric samples, including vertical edges.
        steps = max(1, int(np.ceil(max(np.linalg.norm(xy[i] - xy[j]) for i, j in [(0, 1), (1, 2), (2, 0)]) * 2)))
        for i in range(steps + 1):
            a = i / steps
            n = max(1, steps - i)
            b = np.linspace(0., 1. - a, n + 1)
            points = xy[0] * a + b[:, None] * xy[1] + (1. - a - b)[:, None] * xy[2]
            cells = np.floor(points).astype(int)
            valid = (cells[:, 0] >= 0) & (cells[:, 0] < width) & (cells[:, 1] >= 0) & (cells[:, 1] < height)
            cells = cells[valid]
            grid[cells[:, 1], cells[:, 0]] = 0
    # Outside the field footprint is never navigable.
    grid[:20, :] = grid[340:, :] = 0
    grid[:, :20] = grid[:, 600:] = 0
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / 'rmuc_2025.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode() + grid[::-1].tobytes())
    (output / 'rmuc_2025.yaml').write_text(yaml.safe_dump(dict(
        image='rmuc_2025.pgm', mode='trinary', resolution=resolution,
        origin=[-1., -1., 0.], negate=0, occupied_thresh=0.65, free_thresh=0.25)), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mesh')
    parser.add_argument('output')
    args = parser.parse_args()
    generate(args.mesh, args.output)
