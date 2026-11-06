#!/usr/bin/env python3
"""Stack two runs' overlay frames: run A (top row) above run B (bottom row).

Each row is the side-by-side depth | lidar image from export_detection_overlays.py.
Frames are paired by time; the runs process slightly different camera frames, so the
nearest frame within --max-dt seconds (default 0.15) is used.

Usage:
    python3 scripts/compare_runs.py ~/bags/eval_run3_detections ~/bags/eval_run4_detections \
        [out_dir] [--names "run 3" "run 4"]
"""

import bisect
import os
import re
import sys

import cv2
import numpy as np


def frames(folder):
    found = []
    for name in os.listdir(folder):
        match = re.fullmatch(r't(\d+\.\d)\.jpg', name)
        if match:
            found.append((float(match.group(1)), os.path.join(folder, name)))
    return sorted(found)


def banner(text, width):
    strip = np.zeros((30, width, 3), np.uint8)
    cv2.putText(strip, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
                cv2.LINE_AA)
    return strip


def main():
    args = sys.argv[1:]
    names = ['A', 'B']
    if '--names' in args:
        i = args.index('--names')
        names, args = args[i + 1:i + 3], args[:i] + args[i + 3:]
    max_dt = 0.15
    if '--max-dt' in args:
        i = args.index('--max-dt')
        max_dt, args = float(args[i + 1]), args[:i] + args[i + 2:]
    run_a, run_b = (os.path.expanduser(a) for a in args[:2])
    out = os.path.expanduser(args[2]) if len(args) > 2 else os.path.join(
        os.path.dirname(run_a.rstrip('/')),
        'compare_%s_vs_%s' % (os.path.basename(run_a.rstrip('/')),
                              os.path.basename(run_b.rstrip('/'))))
    os.makedirs(out, exist_ok=True)
    frames_a, frames_b = frames(run_a), frames(run_b)
    times_b = [t for t, _ in frames_b]
    written = 0
    for t, path_a in frames_a:
        i = bisect.bisect_left(times_b, t)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(frames_b)]
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(times_b[k] - t))
        if abs(times_b[j] - t) > max_dt:
            continue
        top, bottom = cv2.imread(path_a), cv2.imread(frames_b[j][1])
        if bottom.shape != top.shape:
            bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))
        width = top.shape[1]
        grid = np.vstack([banner('%s   (t=%.1fs)' % (names[0], t), width), top,
                          banner('%s   (t=%.1fs)' % (names[1], times_b[j]), width), bottom])
        cv2.imwrite(os.path.join(out, 't%05.1f.jpg' % t), grid, [cv2.IMWRITE_JPEG_QUALITY, 90])
        written += 1
    print('%d paired frames -> %s' % (written, out))


if __name__ == '__main__':
    main()
