# Development

The public tree is independent of the running personal deployment. No deployment configuration or data is imported.

## Tests

```sh
python3 -m unittest discover -s tests -v
python3 tools/prepare_dependencies.py --only python-roborock
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt ./.deps/python-roborock
PYTHONPATH=.deps/python-roborock .venv/bin/python -m unittest discover -s tests -v
```

The patrol tests use a fake robot, never hardware. Patch tests use artificial payloads. The server patch carries its own mocked Cloudflare tests; run them in a separate upstream server environment (the server requires a different Python client version).

## Rebuild the screenshots

```sh
npm ci
npx playwright install --with-deps chromium
npm run screenshots
```

The capture script serves the real UI with an invented apartment, room names, identifiers, and zone coordinates. Only the preset display names are borrowed from the personal panel. Every browser request is intercepted. It never reads production files, calls a robot, opens a camera, or uses live audio. The floor plan is a generated occupancy grid with almost clean pixelated walls and very slight edge irregularities. It includes no door-opening symbols. The combined-preset capture selects two of its five rooms. The two screenshots show combined presets and zone presets.

Screenshots are written to `docs/media/`. To add a real demonstration video later, replace the placeholder in the root README. Check the camera view and background before publishing it.
