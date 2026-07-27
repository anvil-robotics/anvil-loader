#!/usr/bin/env python3
"""Start/stop an anvil-loader teleop recording via the webapp tRPC API.

Runs inside the `ros2` container (or anywhere that can reach the webapp on
127.0.0.1:3000 / http://webapps:3000). Wraps the same recording.start /
recording.stop calls the Pico 4 controller sends on Right-A / Right-B, so
episodes land in data/recordings/<dataset>/ exactly like a normal session
(same metadata.yaml / metadata.json, same required-topics guarantee since
the anvil_recorder node — not this script — does the actual MCAP writing).

Usage:
    python3 record_motion.py start [--session-id ID] [--webapp-url URL]
    python3 record_motion.py stop  [--webapp-url URL]

If --session-id is omitted, the current default session is looked up from
the webapp (same as the teleop controller does before an A-button press).
"""

import argparse
import json
import sys
import urllib.parse
import urllib.request


def get_default_session_id(base_url: str) -> int | None:
    url = f"{base_url.rstrip('/')}/api/default-session"
    with urllib.request.urlopen(url, timeout=5) as resp:
        data = json.loads(resp.read())
        return data["id"] if data and "id" in data else None


def trpc_call(base_url: str, method: str, procedure: str, payload: dict | None = None) -> str:
    url = f"{base_url.rstrip('/')}/trpc/{procedure}"
    if method == "GET":
        data = None
        if payload is not None:
            url = f"{url}?input={urllib.parse.quote(json.dumps(payload))}"
    else:
        data = json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"} if data else {},
        method=method,
    )
    with urllib.request.urlopen(req, timeout=10) as resp:
        return resp.read().decode()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["start", "stop"])
    parser.add_argument("--session-id", type=int, default=None)
    parser.add_argument("--webapp-url", default="http://127.0.0.1:3000")
    args = parser.parse_args()

    if args.action == "start":
        session_id = args.session_id
        if session_id is None:
            session_id = get_default_session_id(args.webapp_url)
            if session_id is None:
                sys.exit("No session-id given and no default session set — open a session in the web UI first.")
        result = trpc_call(args.webapp_url, "POST", "recording.start", {"sessionId": session_id})
        print(f"recording.start (session {session_id}): {result}")
    else:
        result = trpc_call(args.webapp_url, "POST", "recording.stop")
        print(f"recording.stop: {result}")


if __name__ == "__main__":
    main()
