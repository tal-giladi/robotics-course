#!/usr/bin/env python3
"""Check that karmel_description's copy of the robot config matches labs/config/karmel.yaml.

labs/config/karmel.yaml is the single source of truth for the robot's dimensions. ROS packages
cannot reach outside the workspace once installed, so karmel_description ships a copy at
src/karmel_description/config/robot.yaml. This script tells you when the two drift apart.

    python3 check_config_sync.py          # exit 0 if in sync, 1 if not (prints the differences)
    python3 check_config_sync.py --fix    # copy labs/config/karmel.yaml over the package copy
"""
import argparse
import shutil
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / 'config' / 'karmel.yaml'
COPY = HERE / 'src' / 'karmel_description' / 'config' / 'robot.yaml'


def diff(a, b, path=''):
    """Yield human-readable differences between two nested YAML structures."""
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b), key=str):
            sub = f'{path}.{key}' if path else str(key)
            if key not in a:
                yield f'{sub}: missing in labs/config/karmel.yaml'
            elif key not in b:
                yield f'{sub}: missing in the package copy'
            else:
                yield from diff(a[key], b[key], sub)
    elif a != b:
        yield f'{path}: karmel.yaml={a!r}  copy={b!r}'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--fix', action='store_true', help='overwrite the package copy with labs/config/karmel.yaml')
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--copy', type=Path, default=COPY)
    args = parser.parse_args()

    if not args.source.is_file():
        print(f'source not found: {args.source}', file=sys.stderr)
        return 2
    if args.fix:
        shutil.copyfile(args.source, args.copy)
        print(f'copied {args.source} -> {args.copy}')
        return 0

    with open(args.source, encoding='utf-8') as f:
        src = yaml.safe_load(f)
    with open(args.copy, encoding='utf-8') as f:
        cpy = yaml.safe_load(f)
    problems = list(diff(src, cpy))
    if problems:
        print('robot config OUT OF SYNC (run with --fix to copy):')
        for p in problems:
            print('  ' + p)
        return 1
    print('robot config in sync')
    return 0


if __name__ == '__main__':
    sys.exit(main())
