# Hexapod kinematics (KOMPAS extract + offline motion)

Python CLI that reads local coordinate systems (LCS) from KOMPAS-3D models under
`spider_body\996` and exports link lengths / body mounts for
`P:\Arduino\hexapod`. Also includes an **offline motion calculator** (pulse
replay + IK gait, matplotlib animation, CSV/JSONL logs).

Package name: **`hexapod_kinematics`** (installable as `hexapod-kinematics`).

## Conventions

- `LCS_in` — joint axis start; `LCS_out` — end; **Z = servo rotation axis**
- Femur housing without `LCS_out`: synthesize along Z by **MG996R height 42.9 mm**
  (datasheet [`DOC017151299.pdf`](DOC017151299.pdf): 40.7 × 19.7 × 42.9 mm)
- Override femur with `servo.femur_override_length_mm` if brackets change the real length
- **Links**: leaf `.m3d` only
- **Body**: `.a3d` + component placements → mounts in assembly frame

## KOMPAS → robot axes (996)

Configured in `extractor_config.yml` → `body_frame`:

| Robot | KOMPAS assembly |
|-------|-----------------|
| forward | +Z (long body axis) |
| up | +Y |
| right | forward × up = −X |

**Calculator world frame** (motion sim): **X forward, Y left, Z up**, via
`cad_to_world_rotation` in config / JSON meta. See [`science/NOTES.md`](science/NOTES.md).

Yaw is the angle of the mount origin in the horizontal plane (⊥ up), 0 along
forward, positive toward right. Unit: `yaw_unit` (`deg` default).

For **body mounts**, coxa servo Z is expected **parallel to up** (yaw joint).
Transforms use full `axis_x/y/z`, not yaw alone.

## Setup

```bat
cd /d P:\stl_reader
py -3 -m pip install -r requirements.txt
```

Requires KOMPAS-3D v21 (`KOMPAS.Application.7`) **only for extract**.

## Extract usage

```bat
py -3 -m hexapod_kinematics extract --config extractor_config.yml --export-dir export
py -3 -m hexapod_kinematics extract --json --header
py -3 -m hexapod_kinematics extract --gabarit
py -3 -m hexapod_kinematics extract --mount-map mount_map.yml
```

- Logs (including auto-mount decisions) → **stderr**
- **stdout** → output paths

Artifacts:

- `export/kinematics_996.json`
- `export/generated_kinematics_996.h` (`COXA/FEMUR/TIBIA`, `LEG_MOUNT_ORIGIN`,
  `LEG_MOUNT_YAW_*`, `LEG_MOUNT_AXES[6][3][3]`)

## Motion calculator (offline)

Gait params + firmware pulse traj: [`config/hexapod_gait.yml`](config/hexapod_gait.yml).

```bat
py -3 -m hexapod_kinematics simulate --mode pulse --cycles 2 --log-dir export/logs
py -3 -m hexapod_kinematics simulate --mode ik --animate --out-gif export/motion_ik.gif
py -3 -m hexapod_kinematics simulate --mode compare --cycles 2
py -3 -m hexapod_kinematics simulate --mode pulse --scale-pulse-to-stride 40
```

Modes:

| Mode | Meaning |
|------|---------|
| `pulse` | Replay `TRANSFER_TRAJ` / `SUPPORT_TRAJ` (as-is firmware µs) |
| `ik` | Alternating tripod foot targets → IK (CAD lengths / mounts) |
| `compare` | Pulse first; IK `stride_mm` set to **measured** pulse stride |

Logs (two files, shared `frame_idx`):

- `export/logs/motion_<mode>_<stamp>_legs.csv` (+ `.jsonl`)
- `export/logs/motion_<mode>_<stamp>_summary.csv` (`support_ok`, `phase_err_*`, `ik_fail_ratio`, …)

Without `--scale-pulse-to-stride` / `compare`, raw pulse vs arbitrary IK stride is **not** comparable.

Static preview:

```bat
py -3 -m hexapod_kinematics.presentation.visualize_kinematics
```

## Body mount assignment priority

1. Explicit `mount_leg_*` / `leg_mount_map`
2. External `--mount-map` / `mount_map_path` (keys: `component_index:LCS_name` or name)
3. Auto: radius filter + CW/CCW sweep + `start_anchor` + `leg_order_from_anchor`

### Example `mount_map.yml`

```yaml
"1:LCS_in": 0
"2:LCS_in": 1
```

## Foot pad

Tip: **RF-F12050** silicone 12×5 mm, recessed **3 mm** into the tibia print →
**+2 mm** protrusion. Exported as `FOOT_PAD_PROTRUSION_MM` and
`TIBIA_EFFECTIVE_LENGTH = TIBIA_LENGTH + 2`.

## Tests

```bat
py -3 -m pytest tests -q
py -3 -m pytest tests -m kompas -q
```

`kompas` tests are excluded by default (`addopts = -m "not kompas"`).
