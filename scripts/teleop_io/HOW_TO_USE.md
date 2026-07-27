# Anvil Teleop — Record / Replay Guide

Everything below runs on the anvil workcell PC (`192.168.1.128`, user `anvil`).

Scripts live on the host at:
```
/home/anvil/anvil-loader/scripts/teleop_io/
  record_motion_standalone.py   # record a motion to an .mcap file
  replay_motion.py              # replay an .mcap file (with ramp-in / ramp-out)
  snapshot_pose.py              # save current arm pose as an .mcap "pose" file
```
This directory is bind-mounted read-only into the `ros2` container at `/scripts`
(`docker-compose.yml` → `ros2.volumes`). Editing a file on the host
takes effect in the running container immediately — no `docker cp`, no
rebuild, no restart needed. Surviving a full `docker compose down && up` is
guaranteed since it's a host mount, not container state.

Recordings live at `/home/anvil/anvil-loader/data/`, mounted at
`/data` inside the container — also a host mount, also survives container
restarts.

---

## 1. Start the stack

```bash
cd /home/anvil/anvil-loader
docker compose --profile pico4 up -d
```

> **Warning:** arms auto-home on startup. Clear the workspace first.

Wait ~30s for controllers to activate. Check everything is up:
```bash
docker ps --format "{{.Names}}\t{{.Status}}"
```
Expect: `ros2-1`, `pico4-teleop-1`, `xr-pc-service-1`, `webapps-1`, `influxdb-1`,
`telegraf-1`, `fluent-bit-1` all `Up`.

## 2. One-time per container start: install mcap support

The `ros2` image doesn't ship `mcap-ros2-support` by default. It does **not**
persist across `down`/`up` (it's installed into the container's own
filesystem, not a mount), so run this once after every fresh `up`:

```bash
docker exec anvil-loader-ros2-1 pip3 install --break-system-packages -q mcap-ros2-support
```

## 3. Enter the container

```bash
docker exec -it anvil-loader-ros2-1 bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=199
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Run everything below from this shell.

---

## Record a motion

Connect the Pico 4 first (XRoboToolkit app → enter PC IP → tick Head/
Controller/Hand → Data/Control: Send → press **A** on right controller to
stream). Hold both side triggers to teleop.

```bash
python3 /scripts/teleop_io/record_motion_standalone.py /data/my_motion.mcap
```
Teleop the motion, then Ctrl+C to stop and save. No session/webapp needed —
this writes the MCAP directly from `/follower_l|r_forward_position_controller/commands`.

## Replay a motion

```bash
# preview the command stream without moving anything
python3 /scripts/teleop_io/replay_motion.py /data/my_motion.mcap --dry-run

# actual replay
python3 /scripts/teleop_io/replay_motion.py /data/my_motion.mcap
```

Every replay automatically:
1. Ramps from the arms' **current** position to the episode's first pose
   (2s default — `--lead-in <seconds>`, `0` to disable).
2. Plays the recorded motion at original timing (`--rate 0.5` for half speed).
3. Ramps back to the saved **stand pose** (2s default — `--lead-out <seconds>`,
   `0` to disable; `--stand-pose <path>` to use a different pose file).

The stand pose defaults to `/data/initial_stand.mcap`.

## Save/refresh the stand pose

Move the arms to whatever pose you want as "home" (e.g. via teleop, then
release), then:

```bash
python3 /scripts/teleop_io/snapshot_pose.py /data/initial_stand.mcap
```
Overwrites the file with the arms' current `/joint_states`. Any replay run
after this will ramp back to the new pose.

---

## Stop the stack

```bash
docker compose --profile pico4 down
```
Recordings in `/home/anvil/anvil-loader/data/*.mcap` and the
scripts in `scripts/teleop_io/` are on host disk — unaffected by this.

---

## Troubleshooting

- **`ModuleNotFoundError: mcap_ros2`** → step 2 wasn't run after this
  container start; re-run the `pip3 install`.
- **`Timed out waiting for /joint_states`** → `ros2` container's controllers
  haven't finished activating yet; wait and retry, or check
  `docker logs anvil-loader-ros2-1 | tail -50`.
- **Harmless `ExternalShutdownException` traceback after Ctrl+C or
  `timeout`** → cosmetic rclpy shutdown race, the `.mcap` file is still
  saved correctly beforehand.
