# Hexapod kinematics

Python toolkit that:

1. **Extracts** link lengths and body mounts from KOMPAS-3D CAD (`spider_body\996`) via LCS.
2. **Exports** JSON + C headers for firmware (`P:\Arduino\hexapod`).
3. **Simulates** offline gait (firmware pulse replay + IK), writes CSV/JSONL logs, GIFs, and markdown reports.
4. **Visualizes** a static L-stance stick-figure preview from the exported JSON.
5. **Syncs** CAD lengths into Arduino `Config.h`.

Package: **`hexapod_kinematics`** (PyPI / install name: `hexapod-kinematics`).  
CLI entry: `py -3 -m hexapod_kinematics …` (also console scripts `hexapod-kinematics`, `kinematics-extract`).

---

## Gallery

Neutral **L-stance** (1500 µs / 90°): femur horizontal outward, tibia vertical down (⊥ floor), knee 90°. Body height uses **tibia_effective** (print + foot pad). Motion GIFs use fixed axis limits (do not rescale while walking).

### Static preview

![L-stance kinematics preview](export/robot_kinematics_preview.png)

[`export/robot_kinematics_preview.png`](export/robot_kinematics_preview.png) — top view of body mounts + 3D L-stance stick figure.

### Pulse replay (aligned)

![Pulse-aligned gait GIF](export/pulse_aligned.gif)

[`export/pulse_aligned.gif`](export/pulse_aligned.gif) — firmware `TRANSFER_TRAJ` / `SUPPORT_TRAJ` replayed with L-stance servo null; stance tips aligned to ground (`pulse_aligned`).

### IK gait

![IK gait GIF](export/motion_ik.gif)

[`export/motion_ik.gif`](export/motion_ik.gif) — alternating tripod foot targets → IK (continuous elbow branch). Prefer this GIF for geometry-faithful gait visualization.

Regenerate:

```bat
py -3 -m hexapod_kinematics visualize --json export/kinematics_996.json --out export/robot_kinematics_preview.png
py -3 -m hexapod_kinematics simulate --mode pulse_aligned --cycles 2 --animate --out-gif export/pulse_aligned.gif
py -3 -m hexapod_kinematics simulate --mode ik --cycles 2 --animate --out-gif export/motion_ik.gif
```

---

## Repository layout

```
hexapod_kinematics/          # installable Python package
  presentation/              # CLI, matplotlib animate / visualize
  application/               # extract pipeline, simulate, sync, reports
  domain/                    # frames, IK, pulse gait, neutral L-pose, metrics
  infrastructure/kompas/     # COM session, documents, LCS, matrices
  core/                      # config loader, constants, logging
config/
  hexapod_gait.yml           # pulse traj, servo_neutral_angles, IK planner
  masses_996.yml             # mass model (COM / torques)
extractor_config.yml         # CAD roots, LCS maps, body_frame, servo, foot pad
export/                      # generated artifacts (+ showcase PNG/GIFs above)
science/                     # papers + NOTES.md (traceability to code)
tests/                       # pytest (kompas marker excluded by default)
```

Layering: `presentation` → `application` → `domain`; COM lives only under `infrastructure/kompas/`.

---

## Requirements

| Need | When |
|------|------|
| Python **≥ 3.12** | always |
| `pip install -r requirements.txt` | always |
| **KOMPAS-3D v21** (`KOMPAS.Application.7`) | **`extract` only** |
| CAD tree under `roots.cad` (see config) | extract |

Simulate / visualize / sync need only an existing `export/kinematics_996.json` (no KOMPAS).

### Setup

```bat
cd /d P:\hexapod_kinematics
py -3 -m pip install -r requirements.txt
py -3 -m pip install -e .
```

Editable install is optional if you always run from the repo root with `py -3 -m hexapod_kinematics`.

---

## Conventions (CAD)

- **`LCS_in`** — joint axis start; **`LCS_out`** — end; **Z = servo rotation axis**.
- **Links**: lengths from leaf **`.m3d` only** (coxa / femur / tibia folders).
- **Body mounts**: assembly **`.a3d`** + component placements → origins/axes in assembly frame.
- **Femur** without `LCS_out`: length synthesized along Z as **MG996R height 42.9 mm**  
  (datasheet [`DOC017151299.pdf`](DOC017151299.pdf): 40.7 × 19.7 × 42.9 mm).  
  Override with `servo.femur_override_length_mm` in `extractor_config.yml` if brackets change the real length.
- **Foot pad** RF-F12050 (12×5 mm silicone), recessed 3 mm → **+2 mm** tip protrusion.  
  Exported as `FOOT_PAD_PROTRUSION_MM` and `TIBIA_EFFECTIVE_LENGTH = TIBIA_LENGTH + 2`.

Folder → role mapping is in `extractor_config.yml` → `folder_roles` (e.g. `coxa_A_996`, `tiba_A_996`, `body_996`).

### Servo null / math angles

At **1500 µs (90°)** the calculator uses L-stance math angles from
[`config/hexapod_gait.yml`](config/hexapod_gait.yml) → `servo_neutral_angles: [0, 0, −π/2]`:

| Joint | Math at 1500 µs | Meaning |
|-------|-----------------|---------|
| coxa | 0 | radial outward |
| femur | 0 | horizontal |
| tibia | −π/2 | vertical down; knee \|tibia\| = 90° |

Pulse I/O: `math = pulse_to_rad(pulse − 1500) + servo_neutral`. Core FK/IK use **math angles only**.  
`body_height` (when null) = world `hip_z − tip_z` for that pose with **tibia_effective**.

---

## Coordinate frames (996)

Configured in `extractor_config.yml` → `body_frame`:

| Robot concept | KOMPAS assembly |
|---------------|-----------------|
| forward | +Z (long body axis) |
| up | +Y |
| right | forward × up = −X |

**Motion calculator world frame:** **X forward, Y left, Z up**, via `cad_to_world_rotation` (also stored in JSON `meta`). Details: [`science/NOTES.md`](science/NOTES.md).

**Yaw** (mount): angle of mount origin in the horizontal plane (⊥ up), 0 along forward, positive toward right. Unit: `yaw_unit` (`deg` default).

For body mounts, coxa servo **Z** is expected **parallel to up** (yaw joint). Transforms use full `axis_x/y/z`, not yaw alone.

---

## Configuration

| File | Role |
|------|------|
| [`extractor_config.yml`](extractor_config.yml) | KOMPAS progid/bin, CAD root, folder roles, LCS maps, servo, foot pad, body_frame, auto-mount |
| [`config/hexapod_gait.yml`](config/hexapod_gait.yml) | Firmware pulse µs arrays, `servo_neutral_angles`, `ik_reach_margin_mm`, IK planner (`stride_mm`, `step_height_mm`, …) |
| [`config/masses_996.yml`](config/masses_996.yml) | Link/servo/body masses (g), COM fraction, total 1800 g |

YAML overrides defaults from `hexapod_kinematics/core/constants/`.

---

## CLI overview

```bat
py -3 -m hexapod_kinematics extract|simulate|visualize|sync-hexapod -h
```

Logging goes to **stderr**. Command results / paths / reports go to **stdout**.

---

## 1. Extract (KOMPAS → export)

Requires KOMPAS-3D. Scans CAD under `roots.cad`, builds the kinematics model, writes artifacts.

```bat
cd /d P:\hexapod_kinematics
py -3 -m hexapod_kinematics extract --config extractor_config.yml --export-dir export
```

If neither `--json` nor `--header` is set, **both** are written.

| Flag | Meaning |
|------|---------|
| `--config` | Path to `extractor_config.yml` (default: `./extractor_config.yml`) |
| `--export-dir` | Output directory (default: `export`) |
| `--json` | Write only JSON (unless combined with `--header`) |
| `--header` | Write only headers |
| `--gabarit` | Also collect bounding boxes into the model |
| `--mount-map` | External mount map YAML (overrides auto / config map) |
| `-v` | Debug logging |

### Extract artifacts

| Path | Contents |
|------|----------|
| `export/kinematics_996.json` | Full model: `lengths_mm`, `body_mounts[6]`, `links`, `foot_pad`, `meta`, warnings, … |
| `export/generated_kinematics_996.h` | `COXA/FEMUR/TIBIA_*`, `FOOT_PAD_*`, `LEG_MOUNT_ORIGIN`, `LEG_MOUNT_YAW_*`, `LEG_MOUNT_AXES[6][3][3]` |
| `export/generated_link_lengths.h` | Legacy filename (same header content) |

Example lengths from a fresh extract (order of magnitude): coxa ≈ 52.5 mm, femur ≈ 42.9 mm (synth), tibia ≈ 52.5 mm, tibia effective ≈ 54.5 mm.

---

## 2. Visualize (static stick figure)

No KOMPAS. Needs `kinematics_996.json`. Draws an **L-stance** stick figure:
coxa/femur horizontal outward (signed radial for left/right), tibia straight
down (⊥ floor), foot-pad segment below the print tibia tip.

```bat
py -3 -m hexapod_kinematics visualize --json export/kinematics_996.json --out export/robot_kinematics_preview.png
```

![Preview](export/robot_kinematics_preview.png)

| Flag | Meaning |
|------|---------|
| `--json` | Kinematics JSON (default: `export/kinematics_996.json`) |
| `--out` | PNG path; if omitted, opens an interactive window |
| `--show` | Force window even when `--out` is set |
| `--body-height` | Place hips at this CAD-Y (mm) for display |

---

## 3. Simulate (offline motion)

No KOMPAS. Replays pulse and/or runs IK using CAD lengths/mounts + gait YAML.

```bat
py -3 -m hexapod_kinematics simulate --mode pulse_aligned --cycles 2 --log-dir export/logs
```

### Modes

| Mode | Meaning |
|------|---------|
| `pulse` | Alias of **`pulse_aligned`** |
| `pulse_raw` | Firmware angles / FK as-is (no ground alignment) |
| `pulse_aligned` | Same joint angles; rigid body ΔZ so min stance tip **z → 0** each frame |
| `ik` | Alternating tripod foot targets → IK (two elbows; pick branch nearest to previous / L-start) |
| `compare` | Run pulse_aligned first; set IK `stride_mm` to **measured** pulse stride, then IK |

**Pulse-mode caveat:** `TRANSFER_TRAJ` / `SUPPORT_TRAJ` are still the firmware µs snapshot and were **not** rewritten for L-zero. Pulse GIFs show motion **relative to L-stance** and can look unnatural until firmware trajs are re-tuned. Prefer **`ik`** for geometry-faithful gait viz. `ik_reach_margin_mm` (default **5**) pulls IK feet slightly inward of full L-reach.

Without `--scale-pulse-to-stride` or `compare`, raw pulse stride vs arbitrary IK `stride_mm` is **not** a fair comparison.

GIF playback defaults to **half simulated realtime** (pulse and IK finish a cycle in similar wall time). Plot axes are **fixed** for the whole animation (limits from all frames).

### Useful flags

| Flag | Meaning |
|------|---------|
| `--json` | Default `export/kinematics_996.json` |
| `--gait` | Default `config/hexapod_gait.yml` |
| `--masses` | Default `config/masses_996.yml` |
| `--direction` | `forward` / `backward` / `turn_left` / `turn_right` |
| `--cycles` | Gait cycles (default 2) |
| `--body-height` | Override body height (mm) for mounts / COM / support / viz |
| `--scale-pulse-to-stride N` | Scale coxa pulse offsets toward target stride (mm) |
| `--animate` | Render matplotlib animation |
| `--out-gif PATH` | Save GIF (implies animation write) |
| `--fps` | GIF/playback FPS (default ≈ half realtime from frame dt) |
| `--interactive` | Slider over cached frames |
| `--show` | Show matplotlib window |
| `--torques` | Write Lagrange torque CSV under `log-dir` |
| `--no-report` | Skip writing markdown report file (stdout report still prints) |
| `-v` | Debug logging |

### GIF examples

```bat
:: Pulse (aligned)
py -3 -m hexapod_kinematics simulate --mode pulse_aligned --cycles 2 --animate --out-gif export/pulse_aligned.gif

:: IK
py -3 -m hexapod_kinematics simulate --mode ik --cycles 2 --animate --out-gif export/motion_ik.gif

:: Compare → two GIFs: <stem>_pulse_aligned.gif and <stem>_ik.gif
py -3 -m hexapod_kinematics simulate --mode compare --cycles 2 --animate --out-gif export/motion_compare.gif
```

![Pulse aligned](export/pulse_aligned.gif)

![IK motion](export/motion_ik.gif)

With `--out-gif export/motion_compare.gif`, compare writes:

- `export/motion_compare_pulse_aligned.gif`
- `export/motion_compare_ik.gif`

### Simulation logs & report

Under `export/logs/` (timestamped):

| Pattern | Contents |
|---------|----------|
| `motion_<mode>_<stamp>_legs.csv` (+ `.jsonl`) | Per-leg / per-frame angles, tips, roles, `pulse_*`, … |
| `motion_<mode>_<stamp>_summary.csv` | Frame metrics: `support_ok`, `phase_err_*`, `ik_fail_ratio`, … |
| `motion_<mode>_<stamp>_report.md` | Human-readable run report (also printed to stdout) |
| `motion_<branch>_torques.csv` | With `--torques`: `M_femur_nm`, `M_tibia_nm`, … |

Stdout always includes the markdown report and a `logs: …` path line.

Science / metrics mapping: [`science/NOTES.md`](science/NOTES.md).

---

## 4. Sync Arduino `Config.h`

Patches link lengths (and related defines) into the firmware config; prints a JSON diff of CAD vs current file.

```bat
py -3 -m hexapod_kinematics sync-hexapod --json export/kinematics_996.json --config-h P:\Arduino\hexapod\src\core\Config.h
```

Default `--config-h` is `P:\Arduino\hexapod\src\core\Config.h`. Review the diff/warnings before committing firmware.

---

## Body mount assignment

Priority:

1. Explicit LCS names `mount_leg_*` / `leg_mount_map` in `extractor_config.yml`
2. External `--mount-map` or `mount_map_path` (keys: `component_index:LCS_name` or LCS name → leg id 0..5)
3. Auto: radius filter + CW/CCW sweep + `start_anchor` + `leg_order_from_anchor` (`body_mount_auto`)

### Example `mount_map.yml`

```yaml
"1:LCS_in": 0
"2:LCS_in": 1
"3:LCS_in": 2
"4:LCS_in": 3
"5:LCS_in": 4
"6:LCS_in": 5
```

Auto-mount decisions are logged on stderr (`mount_assigned …`).

---

## Typical full refresh of `export/`

```bat
cd /d P:\hexapod_kinematics

:: 1) Clear old artifacts (optional; keeps showcase media if you only delete logs)
rmdir /s /q export\logs
mkdir export\logs

:: 2) CAD extract (needs KOMPAS)
py -3 -m hexapod_kinematics extract --config extractor_config.yml --export-dir export -v

:: 3) Static preview
py -3 -m hexapod_kinematics visualize --json export/kinematics_996.json --out export/robot_kinematics_preview.png

:: 4) Motions + GIFs + logs
py -3 -m hexapod_kinematics simulate --mode pulse_aligned --cycles 2 --log-dir export/logs --animate --out-gif export/pulse_aligned.gif --torques
py -3 -m hexapod_kinematics simulate --mode pulse_raw --cycles 2 --log-dir export/logs
py -3 -m hexapod_kinematics simulate --mode ik --cycles 2 --log-dir export/logs --animate --out-gif export/motion_ik.gif
py -3 -m hexapod_kinematics simulate --mode compare --cycles 2 --log-dir export/logs --animate --out-gif export/motion_compare.gif
```

Expected top-level export files after that:

- `kinematics_996.json`
- `generated_kinematics_996.h`, `generated_link_lengths.h`
- `robot_kinematics_preview.png` *(tracked for README)*
- `pulse_aligned.gif`, `motion_ik.gif` *(tracked for README)*
- `motion_compare_pulse_aligned.gif`, `motion_compare_ik.gif`
- `logs/…`

---

## Tests

```bat
py -3 -m pytest tests -q
py -3 -m pytest tests -m kompas -q
```

`@pytest.mark.kompas` tests need installed KOMPAS-3D and are **excluded by default** (`addopts = -m "not kompas"` in `pyproject.toml`).

Optional lint:

```bat
py -3 -m ruff check hexapod_kinematics tests
```

---

## Related docs

- [`science/NOTES.md`](science/NOTES.md) — papers → calculator parameters, L-stance / pulse null, modes, log fields, mass table
- [`extractor_config.yml`](extractor_config.yml) — CAD / frame / auto-mount policy
- [`config/hexapod_gait.yml`](config/hexapod_gait.yml) — pulse arrays, `servo_neutral_angles`, IK planner
- [`DOC017151299.pdf`](DOC017151299.pdf) — MG996R datasheet (servo envelope)
