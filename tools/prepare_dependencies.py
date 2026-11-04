"""Fetch exact upstream revisions and apply this repository's small patches."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def run(*args, cwd=None):
    subprocess.run(args, cwd=cwd, check=True)


def main():
    pins = json.loads((ROOT / 'patches/upstreams.json').read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', choices=list(pins), default='python-roborock')
    args = parser.parse_args()
    pin = pins[args.only]
    dest = ROOT / '.deps' / args.only
    if dest.exists():
        parser.error(f'{dest} already exists; refusing to overwrite it')
    dest.mkdir(parents=True)
    run('git', 'init', str(dest))
    run('git', 'remote', 'add', 'origin', pin['url'], cwd=dest)
    run('git', 'fetch', '--depth=1', 'origin', pin['revision'], cwd=dest)
    run('git', 'checkout', '--detach', pin['revision'], cwd=dest)
    actual = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=dest, text=True).strip()
    if actual != pin['revision']:
        raise RuntimeError('Upstream revision mismatch')
    for rel in pin['patches']:
        patch = str(ROOT / rel)
        run('git', 'apply', '--check', patch, cwd=dest)
        run('git', 'apply', patch, cwd=dest)
    print(f'Ready: {dest} at {actual} + {len(pin["patches"])} patch(es)')


if __name__ == '__main__':
    main()
