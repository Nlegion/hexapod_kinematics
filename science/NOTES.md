# Science notes → motion calculator traceability

This file maps the three papers under `science/` to concrete calculator
parameters, modules, and log fields.

---

## Массы hexapod 996 (г)

Источник: взвешивание / оценка сборки. Конфиг: `config/masses_996.yml`.

| Элемент | Масса | Итого |
|---------|------:|------:|
| coxa / femur / tibia | 15 / 21 / 26 | ×6 → 372 |
| body parts `2+18×6+25+30+49+45` | | 259 |
| MG996R servo | 55 | ×18 → 990 |
| battery_misc (остаток) | 179 | 179 |
| **Всего** | | **1800** |

CoM звеньев v1: mid-link (`com_link_fraction: 0.5`). Mass-weighted COM → `com_x/y`, `com_source=mass_model`.  
Динамика: `domain/dynamics_leg.py` → `*_torques.csv` (`M_femur_nm`, `M_tibia_nm`, …).

---

## Рукавицын А.Н., Чжо Пью Вей (2023)

**File:** `issledovanie-dinamiki-dvizheniya-opornoy-konechnosti-shagayuschego-robota-bionicheskogo-tipa.pdf`  
**Venue:** Транспортное машиностроение, 2023, №1(13), pp. 14–23.

### Переносится в калькулятор

| Из статьи | Формула / смысл | Куда в коде | YAML / лог / report |
|-----------|-----------------|-------------|---------------------|
| Опорный треугольник, ЦТ внутри | COM ∈ triangle(stance) | `gait_metrics.support_margin_mm`, `support_ok` | `support_ok`, `support_ok_rate` |
| СК корпуса + СК крепления | `p_world = R @ p_cad` | `frames_world`, `body_frame_kin` | `cad_to_world_rotation` |
| Траектория опоры → ОЗК | foot target → IK | `gait_ik` → `inverse_kinematics` | `stride_mm`, `step_height_mm` |
| Полиномы 5-го порядка | swing law | `gait_ik.poly5` | `swing_profile: poly5` |
| Лагранж II рода, моменты | M = d/dt(∂L/∂q̇) − ∂L/∂q | `dynamics_leg.torques_planar_femur_tibia` | `M_*_nm`, masses.yml |
| Углы во времени | q(t) | legs CSV | `angle_*_rad`, `t_ms` |

### Режимы симуляции (честное сравнение)

| Mode | Смысл |
|------|--------|
| `pulse_raw` | углы/FK как в firmware |
| `pulse_aligned` | те же углы; жёсткий ΔZ до контакта stance с z=0 |
| `ik` | полноценная IK-траектория |

---

## Ramdya et al. (2017) — Nature Communications 8:14494

**File:** `ncomms14494.pdf`  
**Title:** Climbing favours the tripod gait over alternative faster insect gaits.

### Переносится в калькулятор

| Из статьи | Формула / смысл | Куда в коде | YAML / лог |
|-----------|-----------------|-------------|------------|
| Alternating tripod 0° / 180° | Δphase = 0.5 | `tripod_group_1/2`; `phase_errors_deg` | `phase_err_mean_deg`, `phase_err_max_deg` |
| Footfall / gait diagram | stance vs swing | Hildebrand panel | roles `support`/`transfer` |
| COM in support polygon | static stability | `gait_metrics` + mass COM | `support_ok`, `support_margin_mm` |
| TCS (Tripod Coordination Strength) | не полный TCS | аппроксимация phase error | report `phase_err_*` |

### Не переносится

- Полный TCS / PSO gait discovery — избыточно для v1.
- Bipod / climbing adhesion — вне scope.

---

## O’Neil et al. (2024) — bioRxiv 10.1101/2024.04.02.587757

**File:** `nihpp-2024.04.02.587757v1.pdf`

### Переносится в калькулятор

| Из статьи | Куда в коде | YAML / лог |
|-----------|-------------|------------|
| Alternating tripod | `tripod_group_*` / modes | — |
| Роли поясов (front/mid/hind) | `LEG_BAND` / `band_of` | `band`, `stride_by_band_mm` |
| Stride length logging | measure pulse stride; compare | `stride_mm`, report bands |

### Не переносится

- Водная / duckweed физика; per-band stride YAML — заготовка (фаза research).

---

## Firmware pulse source

Pulse arrays are **not** from the papers; they are a snapshot of
`P:\Arduino\hexapod\src\core\Config.h` (`TRANSFER_TRAJ` / `SUPPORT_TRAJ`)
stored in `config/hexapod_gait.yml` for faithful offline replay.

Fail-fast: `domain/gait_config.load_gait_config` требует µs `[coxa,femur,tibia]` массивы.

---

## Rename note

Пакет переименован: `src` / `stl_reader` → **`hexapod_kinematics`**
(`pyproject` name: `hexapod-kinematics`). CLI: `py -3 -m hexapod_kinematics …`.
