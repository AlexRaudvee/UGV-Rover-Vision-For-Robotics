#!/usr/bin/env python3
"""Export the same free-carpet map published by football_field (no ROS needed)."""

import argparse
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'src/ugv_main/ugv_bringup'))
from ugv_bringup.field_model import export_map, load_config  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--config', type=Path,
                    default=root / 'src/ugv_main/ugv_bringup/param/football_field.yaml')
parser.add_argument('--out', type=Path, default=root / 'maps/robocup')
args = parser.parse_args()
export_map(load_config(args.config), args.out)
print(f'Wrote {args.out}/robocup_field.yaml and robocup_field.pgm')
