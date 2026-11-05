#!/usr/bin/env python3
"""Remove the obsolete placeholder metadata that shadows the actual detector entry point."""

from pathlib import Path
import shutil
import sys


def main():
    """Clean only the known 0.0.0 placeholder inside the requested install prefix."""
    prefix = Path(sys.argv[1] if len(sys.argv) > 1 else 'install')
    site_root = prefix / 'object_detection' / 'lib'
    for path in site_root.glob('python*/site-packages/object_detection-0.0.0*.egg-info'):
        if path.is_symlink() or not path.is_dir():
            continue
        metadata = (path / 'PKG-INFO').read_text()
        names = ('Name: object-detection\n', 'Name: object_detection\n')
        if not any(name in metadata for name in names):
            continue
        if 'Version: 0.0.0\n' not in metadata:
            continue
        shutil.rmtree(path)
        print('Removed obsolete detector placeholder metadata: ' + str(path))


if __name__ == '__main__':
    main()
