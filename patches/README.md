# Small patches, not forks

The exact upstream URLs and commits are in [upstreams.json](upstreams.json). Patches apply to those revisions, not an arbitrary future release.

| Project | Base | What is included |
| --- | --- | --- |
| python-roborock | [`e8d5466` / v6.2.1](https://github.com/Python-roborock/python-roborock/commit/e8d546615720d7792788378f49bf292c3f0e499c) | [Panel client support](python-roborock/0001-panel-client-support.patch) |
| local_roborock_server | [`206d560`](https://github.com/Python-roborock/local_roborock_server/commit/206d560f622836fc0a58f221e042858c475564cc) | [Optional Cloudflare TURN](local-roborock-server/0001-optional-cloudflare-turn.patch) |

## Python client — included in the panel build

The client patch contains only the changes used by the panel:

- Water-slider codes and conversion helpers; robot dust-bin state.
- `get_clean_detail` for completed-job timelines.
- Reassembly of fragmented obstacle-photo blobs and per-request timeouts.
- `RR_MQTT_URL` for a local broker, including anonymous internal MQTT if configured.
- `RR_NO_LOCAL=1` to use MQTT without retrying an unreachable local robot socket.

The exploratory blob-cipher switch is deliberately omitted. No test captures, home data, or unrelated client files are copied into the patch.

The Docker build applies this automatically. For a development checkout:

```sh
python3 tools/prepare_dependencies.py --only python-roborock
```

## Local server — only for the optional Cloudflare relay

Normal cleaning and camera support with a standard TURN relay already exist at the pinned upstream server revision. **Do not apply a server patch just to use this panel.** Follow upstream’s [camera documentation at the pinned revision](https://github.com/Python-roborock/local_roborock_server/blob/206d560f622836fc0a58f221e042858c475564cc/docs/camera.md) for onboarding, remote-view activation, and provided/external TURN.

This optional patch reproduces the Cloudflare relay integration used in my setup. It adds credential fetching/caching, the narrow `createca` response understood by the robot, configuration wiring, and mocked tests. It contains none of the local Android-app stubs, APK modifications, media endpoints, TLS/routing customization, or packet tracing from my deployment.

```sh
python3 tools/prepare_dependencies.py --only local-roborock-server
# This prepares .deps/local-roborock-server; it does not touch a running server.
```

Build/install that checkout using the upstream instructions, then set these fields in its **private** `config.toml`:

```toml
[turn]
mode = "cloudflare"
cloudflare_key_id = "YOUR_TURN_KEY_ID"
cloudflare_api_token = "YOUR_TURN_API_TOKEN"
ttl = 86400
```

Create the TURN key in your own Cloudflare account. Credentials and any provider charges are yours; no shared relay account is included. Restart the server after configuration. This was used with a Saros 10R; support on other models is unverified.

## Applying by hand

In a clean checkout of the listed commit:

```sh
git apply --check /path/to/0001-panel-client-support.patch
git apply /path/to/0001-panel-client-support.patch
```

The preparation script checks out the exact SHA, validates the patch, and applies it. It refuses an existing destination so it cannot overwrite your changes. Remove or rename your own `.deps` checkout when you deliberately want a fresh one.

Upstream code remains under each upstream project’s license. These patches are distributed under the corresponding upstream license; see [third-party notices](../THIRD_PARTY_NOTICES.md).
