"""Offline robustness evaluation: run several detection presets over saved images.

No ROS needed. Example:
    python3 -m l2s1_marker_detection.evaluate runs/marker_captures \
        --presets baseline clahe upscale tuned improved --expect APRILTAG_36h11:3 --out runs/eval

Writes <out>/detections.csv (one row per image x preset), annotated images per
preset in <out>/annotated/<preset>/, and prints a per-preset summary table.
"""

import argparse
import csv
from dataclasses import replace
from pathlib import Path
import statistics
import sys

import cv2

from . import aruco_compat as compat
from .camera_model import CameraModel
from .detection import DEFAULT_FAMILIES, PRESETS, MarkerDetector, draw, short_family

IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.bmp'}


def collect_images(paths):
    images = []
    for path in map(Path, paths):
        if path.is_dir():
            images += sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES
                             and not p.stem.endswith('_annotated'))
        elif path.suffix.lower() in IMAGE_SUFFIXES:
            images.append(path)
    return images


def parse_expected(items):
    expected = set()
    for item in items:
        family, _, marker_id = item.rpartition(':')
        if not family:
            raise SystemExit(f'--expect needs FAMILY:ID, got {item!r}')
        expected.add((compat.normalize_family(family), int(marker_id)))
    return expected


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('images', nargs='+', help='image files and/or directories')
    parser.add_argument('--presets', nargs='+', default=list(PRESETS), choices=list(PRESETS))
    parser.add_argument('--families', nargs='+', default=list(DEFAULT_FAMILIES))
    parser.add_argument('--camera-info', help='camera_info YAML (needed for undistort)')
    parser.add_argument('--expect', nargs='*', default=[],
                        help='markers that are in every image, as FAMILY:ID')
    parser.add_argument('--repeat', type=int, default=3, help='timing repetitions per image')
    parser.add_argument('--out', default='marker_eval')
    args = parser.parse_args(argv)

    images = collect_images(args.images)
    if not images:
        raise SystemExit('No images found')
    expected = parse_expected(args.expect)
    camera = CameraModel.from_yaml(args.camera_info) if args.camera_info else None

    detectors = {}
    for name in args.presets:
        config = replace(PRESETS[name], families=tuple(args.families))
        if config.undistort and camera is None:
            print(f'Skipping preset {name}: needs --camera-info', file=sys.stderr)
            continue
        detectors[name] = MarkerDetector(config, camera)

    out = Path(args.out)
    for name in detectors:
        (out / 'annotated' / name).mkdir(parents=True, exist_ok=True)
    rows = []
    for image_path in images:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            print(f'Cannot read {image_path}', file=sys.stderr)
            continue
        for name, detector in detectors.items():
            results = [detector.detect(image) for _ in range(max(1, args.repeat))]
            result = results[0]
            found = {(d.family, d.marker_id) for d in result.detections}
            markers = ' '.join(f'{short_family(f)}:{i}' for f, i in sorted(found))
            rows.append({
                'image': image_path.name, 'preset': name, 'count': len(result.detections),
                'markers': markers,
                'expected_found': (f'{len(found & expected)}/{len(expected)}'
                                   if expected else ''),
                'unexpected': len(found - expected) if expected else '',
                'mean_side_px': (round(statistics.mean(d.side_px for d in result.detections), 1)
                                 if result.detections else ''),
                'runtime_ms': round(statistics.median(r.runtime_ms for r in results), 2),
            })
            status = f'{name} n={len(result.detections)} {markers}'
            cv2.imwrite(str(out / 'annotated' / name / (image_path.stem + '.jpg')),
                        draw(image, result.detections, status))

    with open(out / 'detections.csv', 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print_summary(rows, detectors, expected, len(images))
    print(f'\nWrote {out / "detections.csv"} and annotated images in {out / "annotated"}')
    return 0


def print_summary(rows, detectors, expected, image_count):
    header = f'{"preset":10} {"images>=1":>10} {"detections":>11}'
    if expected:
        header += f' {"expected hit":>13} {"unexpected":>11}'
    print(header + f' {"median ms":>10}')
    for name in detectors:
        mine = [row for row in rows if row['preset'] == name]
        line = (f'{name:10} {sum(r["count"] > 0 for r in mine):>4}/{image_count:<5} '
                f'{sum(r["count"] for r in mine):>11}')
        if expected:
            hits = sum(int(r['expected_found'].split('/')[0]) for r in mine)
            line += (f' {hits:>6}/{len(expected) * len(mine):<6}'
                     f' {sum(r["unexpected"] for r in mine):>11}')
        line += f' {statistics.median(r["runtime_ms"] for r in mine):>10.1f}'
        print(line)


def console_main():
    sys.exit(main())


if __name__ == '__main__':
    console_main()
