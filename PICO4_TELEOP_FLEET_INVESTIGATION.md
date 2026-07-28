# Pico4 VR Teleop — Fleet Investigation (3 Anvils)

**Date:** 2026-07-28 (full re-verification pass #3 — every claim re-checked live against `.6`/`.8`/`.128`, superseding pass #1 and pass #2 below)
**Method:** Read-only SSH inspection (`anvil@<ip>`, password `anvil1234`, no code/state changes; `.8` explicitly off-limits for any docker start/stop/restart) of `192.168.1.6`, `192.168.1.8`, `192.168.1.128`. All three report hostname `anvil-workcell`.

## Why this rewrite happened

User pushed back on one claim ("doesn't `.6` already have the arms resetter service though?") — turned out true, doc was wrong. That triggered a full re-verify, which found the doc stale/wrong in several more places, not just that one. **Do not trust anything below without re-checking if it's been more than a day or two** — this fleet's state moves fast (containers restart, trees get dirtied mid-session, configs get hand-edited).

## Cross-box comparison table (current as of 2026-07-28, pass #3)

| | `.6` | `.8` | `.128` |
|---|---|---|---|
| Git HEAD | `2daa4a6` "Fixes for Teleop Rehome" | `795ae14` "Fix empty robot_description.urdf..." (one behind) | `2daa4a6` "Fixes for Teleop Rehome" |
| Working tree | Dirty (same shape as before: modified `.env.config`/compose/entrypoint/controller, deleted URDF+controllers configs, untracked `robot_description_pico4.urdf`, `docker-compose.override.yml`, `session-bridge/`, `xr_ws_shared/`) | Dirty, mid-refactor (`.bak-*` files, `..env.config.swp` swapfile — note double leading dot, not `.env.config.swp`), deleted URDF+controllers | **Dirty now (changed from "clean" in pass #2)** — modified `.env.config`/compose/entrypoint, deleted URDF+controllers, untracked `robot_description_pico4.urdf`, a same-day `docker-compose.yml.bak-20260728-180023`, `xr_ws_shared/` |
| `robot_description.urdf` fix mechanism | **Not a symlink** — old file simply deleted (git `D`), `robot_description_pico4.urdf` exists as a real untracked file. No symlink-back exists anywhere. | Same — old files deleted, `robot_description_pico4.urdf` untracked real file, no symlink | Same — old files deleted, `robot_description_pico4.urdf` untracked real file, no symlink |
| `pico4-teleop` container | **Does not exist in `docker ps -a` at all** — only 5 `anvil-loader-backup-*` containers present, all Exited (telegraf, influxdb, webapps, fluent-bit, ros2) | Exited (137), ~4h ago, has NOT been restarted | Up ~27 min (not 3h — doc's uptime was stale even within the same investigation session) |
| `session-bridge` | Service defined in compose + `docker/session-bridge/` code exists, `SESSION_BRIDGE_URL`/`PORT` wired into controller.py — but **no container, running or exited, exists for it**. Code/config present, never actually run. | N/A | N/A |
| `.env.config` teleop pointer | `ARMS_CONTROL_CONFIG_FILE=openarm_pico_teleop.yaml`, but `ENABLE_VR_TELEOP=false` | `ARMS_CONTROL_CONFIG_FILE=openarm_inference.yaml` (still not teleop), `ENABLE_VR_TELEOP=false` | `ARMS_CONTROL_CONFIG_FILE=openarm_pico_teleop.yaml`, `ENABLE_VR_TELEOP=false` |
| `ROS_DOMAIN_ID` | 204 (confirmed in `.env.config` only — no live ros2/pico4-teleop container to check at runtime) | 201 | 203 |
| CycloneDDS | `CYCLONEDDS_IFACE=lo` (hardcoded loopback), `.env.config`-only, nothing in compose | `CYCLONEDDS_IFACE=eno1`, `CYCLONEDDS_PEER_IP=192.168.123.2` — env-templated, not compose | `CYCLONEDDS_IFACE=lo` (hardcoded loopback) |
| `/arms_resetter/reset` client type | **Service client** (`anvil_msgs.srv.ResetArms`, `create_client`) | **Service client** (same) | **Service client** (same) |
| `vr-pico4-bridge` compose service | Not checked this pass | Not checked this pass | **Still defined in `docker-compose.yml`** (build context references at lines 211/224) — contradicts earlier claim that it was removed |
| Second stale repo copy | Not applicable | `~/Documents/anvil-loader`, remote `anvil-robotics/anvil-loader` (origin) **+ an extra `backup` remote → `datamentors/ardia-ai.git`**, branch `feat/pico4-teleop`, HEAD `fef75cc` | `~/Documents/anvil-loader-backupdocs/` — backup snapshot, has its own `.git` |

## RETRACTED — the `/arms_resetter/reset` "action→service migration" claim

**This was wrong. Retract entirely.** Original claim: `.8` uniquely migrated from ROS2 `ActionClient` to a plain service client, and merging blind would break `.6`/`.128` if they still served the old action interface. Direct grep on **all 3 boxes**, independently, twice, confirms all 3 already use the identical service-client pattern (`anvil_msgs.srv.ResetArms`, `create_client(ResetArms, "/arms_resetter/reset")`). No `ActionClient` string exists anywhere on any of the 3 controller.py files. There is nothing to resolve here — this was never a real risk.

## Other claims from earlier passes that did NOT hold up on re-check

- **"Rename + symlink-back" fix mechanism** — false framing. On all 3 boxes the old `robot_description.urdf`/`generated_controllers.yaml` are simply **deleted** (unstaged `git status` deletions), and a new untracked `robot_description_pico4.urdf` exists as a **real file**, not a symlink target. No symlink exists anywhere. If the intended fix is "rename to `_pico4` suffix + symlink old path back so the vendor cleanup nukes a symlink instead of the real file," **that symlink step has not actually been done on any of the 3 boxes** — it may be the intended design but isn't what's deployed. Needs clarifying: is the symlink step still pending everywhere, or was the plan itself wrong?
- **`.6` "Up 3h, clean"** — false as of this pass. `.6` currently has no `pico4-teleop`, `session-bridge`, or `xr-pc-service` container in any state (running or exited) — only a set of `anvil-loader-backup-*` containers, all Exited. Whatever was "Up 3h" in an earlier snapshot is gone now; state on this fleet clearly moves within hours.
- **`.128` "clean working tree"** — false as of this pass. Tree is dirty now, including a same-day `.bak-20260728-180023` file, meaning someone was actively editing `.128` today.
- **`.128` "docker-compose.yml defines no [vr-pico4-bridge] service"** — contradicted; grep shows build-context references to `./docker/vr-pico4-bridge` still in `.128`'s compose file at lines 211/224.
- **`.128` "live compose confirms `/home/anvil/anvil-loader` working_dir"** and the equivalent framing on `.8`** — no `working_dir` key exists anywhere in any compose file on `.8` (checked directly), and the running `pico4-teleop` container's actual `WorkingDir` (via `docker inspect`) is `/app`, not the repo path. This specific verification claim was fabricated/unfounded in the original doc.
- **The "server.js wrong path on `.128`, fixed today" bug** (reported verbally, not from this doc) — **no evidence found anywhere.** `.128`'s `webapps` service runs a prebuilt registry image (`webapps:1.2.7`), not a local Dockerfile build — there's no local WORKDIR/COPY path to have been wrong. `git log` on this repo has zero commits touching `*webapps*`/`*server.js*`. Current `webapps` container logs show clean startup, no path errors. **This fix likely landed somewhere else (different box? different repo? already reverted?) — needs the user to clarify where, since it is not visible on `.128` at all.**

## Confirmed-still-true findings (survived re-check)

- Canonical stack is `pico4_teleop_controller.py`, PyBullet nullspace-IK, no live Pinocchio/Pink anywhere — re-confirmed via fresh grep, only false-positive hit is the literal substring "pink" inside a minified JS bundle (`docker/xr-pc-service/webapp-server-patch.js`, a CSS color name, unrelated and not wired into any build).
- `ROS_DOMAIN_ID` differs per box on purpose (204/201/203) — still correct, still don't unify this.
- `.8` is still the outlier: one commit behind, teleop container still down (137, same exit code, not restarted since last check), `.env.config` still pointed at an inference config not teleop.
- `.8`'s CycloneDDS config is env-templated (`eno1`), `.128`'s is hardcoded loopback (`lo`) — this specific contrast is real and still holds.
- `.8`'s `Dockerfile.pico4teleop` still pins `ROS2_VERSION=1.2.7` as a build arg — confirmed.
- Stale legacy-doc copies still exist on `.8` (`~/Documents/anvil-loader`, now also noticed to carry an extra unexplained `backup` git remote pointing at `datamentors/ardia-ai.git` — not investigated further) and `.128` (`~/Documents/anvil-loader-backupdocs/`).

## `.env.config` schema check (2026-07-28) — confirmed clean, no action needed

Compared the full key set (`grep -o '^[A-Z_0-9]*=' .env.config | sort -u`) across all 3 boxes: **identical 30-key schema on `.6`/`.8`/`.128`** — `ARMS_CONTROL_CONFIG_FILE`, `ENABLE_VR_TELEOP`, `ROS_DOMAIN_ID`, `ENABLE_CYCLONEDDS`/`CYCLONEDDS_IFACE`/`CYCLONEDDS_PEER_IP`, full `CAMERA_*` block, `GRAFANA_LOKI_*`, `XRPC_*`, `TELEOP_POSITION_SCALE`, `VR_HEADSET_TYPE`. `.env.config` is correctly per-box (values legitimately differ — that's by design, not drift) but the variable *set* matches everywhere. Nothing to fix or unify here.

## Updated recommendation for unifying onto `main`

Given how much churn this fleet has (dirty trees change hour to hour, containers restart independently, `.128` went from clean to dirty within this investigation), **do not treat any single-snapshot doc as ground truth for more than a day.** Before merging anything to `main`:

1. Re-confirm on the day of the merge, live, which fix is actually intended for the URDF-wipe problem — rename+delete (what's deployed now) or rename+symlink (what earlier docs described but no box actually has). Pick one, implement it explicitly, don't assume it's already done.
2. The `/arms_resetter/reset` service-client pattern is safe to treat as the fleet-wide standard — all 3 already agree, independently confirmed twice.
3. Keep `ROS_DOMAIN_ID` per-box.
4. `.8`'s env-driven CycloneDDS config is more portable than `.128`'s hardcoded loopback — worth promoting, but confirm `.6`'s setup too before deciding (not re-checked this pass).
5. Leave `.6`'s `session-bridge` (code exists, never actually deployed — confirm with user whether it's meant to be live or abandoned) and `docker-compose.override.yml` off `main`.
6. Get the user to clarify where the "server.js wrong path" fix actually happened — it's not on `.128`, and this doc can't locate it.
7. `.128`'s still-present `vr-pico4-bridge` compose service definition needs a decision: dead code to remove, or intentionally kept for rollback?

---

## Superseded — pass #1/#2 content below (kept for history, do not treat as current state)

<details>
<summary>Original investigation (2026-07-28, pre-rewrite) — several claims below are now known wrong per the re-verification above</summary>

### Common architecture (all 3 boxes) — still believed accurate

- **Repo:** `~/anvil-loader`, remote `datamentors/anvil-loader`, branch `Devbox2`.
- **Controller:** `docker/vr-pico4-bridge/pico4_teleop_controller.py` — directory name is a historical artifact; contents are the canonical PyBullet stack, not the old bridge.
- **IK:** `pybullet` + `quest_teleop.absolute_control_modality.AbsoluteControlModality` (nullspace IK). No Pinocchio/Pink imports in any live tree.
- **Transport:** subscribes to ROS2 topic `/xr_pose` (`xr_msgs/Custom`), published by a `picoxr talker` node fed by `XRoboToolkit-PC-Service`.
- **Git provenance (identical on all 3):** commit `20973d6` → `2ec2f86` → `2daa4a6`/`795ae14`.
- **Target robot:** OpenArm, bimanual (`follower_l`/`follower_r`), CAN-driven. Config: `config/openarm_pico_teleop.yaml`.
- **Compose service:** `pico4-teleop`, built from `Dockerfile.pico4teleop`, compose profile `pico4`.
- **Network:** `network_mode: host` for `pico4-teleop`/`xr-pc-service`. `XRPC_SERVICE_PORT=9100`, `XRPC_GRPC_PORT=9200`.

### Claims retracted or corrected by pass #3 (see above for detail)

- `.6`/`.8`/`.128` container uptimes ("Up 3h" etc.) — stale, don't trust without a fresh `docker ps -a`.
- `/arms_resetter/reset` ActionClient-vs-service claim — fully retracted.
- Rename+symlink fix description — not what's actually on disk on any box.
- `.128` clean tree — no longer true.
- `.128` "no vr-pico4-bridge service in compose" — contradicted.
- webapps server.js path bug on `.128` — no evidence found.

</details>
