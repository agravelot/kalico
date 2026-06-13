# Phase Stepping Port — Prusa → Kalico

## Iteration Workflow (`./iter.sh`)

The `iter.sh` script provides a fast edit-compile-flash-test loop that pushes
changes from your local Kalico repo to the printer, rebuilds firmware if C code
changed, and validates the result:

```bash
# After making changes (Python or C), run:
./iter.sh
```

**What it does:**

1. **rsyncs the entire `kalico/` tree** to the printer at `~/klipper/`
2. **Detects C changes** — if any `src/*.c` changed, it:
   - rsyncs just the `src/` tree (faster than full sync)
   - Copies the Kconfig (board-specific, currently `octopus.config`)
   - Runs `make olddefconfig` + flashes the MCU firmware
   - Waits for Klipper to come back ready
3. **Restarts Klippy** (Python host) via `FIRMWARE_RESTART` G-code
4. **Homes** via `_CG28`, then waits for idle
5. **Pulls the log**, checks for `Move out of range` or `error`/`fatal`
6. Exits 0 if all good, 1 if there are errors

**Key variables in the script** (edit as needed):
- `REMOTE_HOST="voron2.agravelot.eu"` — your printer's hostname
- `CONFIG_FILE="../voron2-config/scripts/octopus.config"` — path to Kconfig
- `API_KEY="..."` — Moonraker API key

---

# Phase Stepping Port Plan

## Overview

Phase stepping compensates for individual stepper motor manufacturing
imperfections by applying a per-microstep phase correction. It runs at runtime
via a periodic ISR that adjusts the motor's electrical phase by injecting burst
step pulses through GPIO DMA.

The port splits into 7 major work items spanning both the Python host
(`klippy/`) and the C MCU firmware (`src/`).

---

## Work Item 1: MCU Firmware — Burst Stepping Engine

**New file: `src/tmc_phase_step.c` (+ `src/tmc_phase_step.h`)**

This is the real-time core. A timer ISR (10 kHz) computes motor phase from
stepper position, applies correction from a pre-loaded LUT, and fires
DMA-driven GPIO step bursts.

### A. Per-stepper phase stepping state

```c
struct phase_stepper {
    struct timer timer;           // Scheduler timer for periodic refresh
    uint8_t stepper_oid;          // Associated stepper OID
    struct gpio_out step_pin;     // Step pin (same as stepper)
    struct gpio_out dir_pin;      // Direction pin
    uint32_t step_port_mask;      // BSRR mask for step pin
    uint32_t dir_port_mask;       // BSRR mask for direction pin

    // Correction LUT — one int8 per electrical microstep
    int8_t phase_shift_lut[1024];

    // Phase tracking
    uint16_t motor_phase;         // Current electrical phase (0..1023)
    uint16_t driver_phase;        // TMC driver phase (0..1023)
    uint32_t last_position;       // Last stepper position
    int32_t zero_rotor_phase;     // Phase offset at logical position 0
    uint32_t steps_per_period;    // Motor steps per electrical period

    // Burst buffer (double-buffered GPIO BSRR events)
    uint32_t event_buffer[2][200];
    uint8_t active_buffer;        // Which buffer is currently filled
    uint8_t max_event_index;      // Number of entries in current buffer
    bool axis_direction;          // Current step direction

    // Status
    bool enabled;
    bool dma_busy;
};
```

### B. New MCU commands

| Command | Parameters | Description |
|---------|-----------|-------------|
| `configure_phase_stepping` | `oid=%c stepper_oid=%c step_pin=%u dir_pin=%u zero_phase=%i steps_per_period=%u` | Set up a phase step instance bound to a stepper |
| `load_phase_lut` | `oid=%c data=%*s` | Download 1024×int8 correction LUT (1024 bytes) |
| `enable_phase_stepping` | `oid=%c enable=%c` | Enable/disable per-axis |

### C. Timer ISR callback (`~10 kHz`, configurable)

Each refresh cycle (100 µs):

1. **Fire previous DMA burst** — start DMA transfer of the previously built
   event buffer to `GPIOx->BSRR`.
2. **For each enabled `phase_stepper`**:
   - Read `stepper.position` → current motor position
   - Compute electrical phase:
     `phase = (position * steps_per_period + zero_rotor_phase) % 1024`
   - Lookup correction: `correction = phase_shift_lut[phase]`
   - Compute target phase: `target = (phase + correction + 1024) % 1024`
   - Compute phase difference: `diff = target - driver_phase` (wrapped to
     -512..512)
   - If `|diff| >= 1` microstep: fill burst buffer with evenly-spaced step
     toggles
   - Update `driver_phase = target`
3. **Swap burst buffers** (double buffering).

### D. GPIO burst buffer mechanics

Identical to Prusa's approach. For `diff > 0` microsteps within the 100 µs
period:

- Spacing: `spacing = (200 << 16) / diff` (16.16 fixed-point)
- For each toggle `i ∈ [0, diff-1]`, compute `idx = spacing * i >> 16`
- Alternate step-high / step-low events at position `idx` in the buffer
- Use `GPIOx->BSRR`: bits `[0:15]` set pin HIGH, `[16:31]` set pin LOW
- DMA transfers buffer entries to BSRR at timer-update rate

### E. Kconfig options

```kconfig
config WANT_TMC_PHASE_STEP
    depends on HAVE_GPIO_BITBANGING
    bool "TMC Phase Stepping support"
    default n
    help
        Enables phase stepping burst engine on the MCU.
        Requires GPIO BSRR register and DMA controller.
```

The module requires a GPIO port with BSRR register and a DMA controller —
targeted at **STM32F446xx** (the MCU on the Octopus board driving the
printer).

**Estimated code size**: ~800-1000 lines of C.

---

## Work Item 2: Host Python — Phase Stepping Module

**New file: `klippy/extras/phase_stepping.py`**

Main host-side module. Handles configuration, LUT management, G-code commands,
enable/disable coordination.

### Configuration section

```ini
[phase_stepping stepper_x]
motor_steps: 200            # Full steps per rotation (default 200)
microsteps: 16              # Normal microstep resolution

# Optional:
# harmonics: 2,4            # Which harmonics to correct (default 2,4)
```

### Key class: `PhaseStepping`

```python
class PhaseStepping:
    def __init__(self, config):
        self.name = ...
        self.stepper_name = ...
        self.motor_steps = config.getint("motor_steps", 200)
        self.microsteps = config.getint("microsteps", 16)
        self.motor_period = 1024  # TMC electrical period

        # Per-direction correction tables
        self.correction_luts = {
            "forward": MotorPhaseCorrection(),  # array of SpectralItem[16]
            "backward": MotorPhaseCorrection(),
        }

        # Phase tracking
        self.mcu_phase_offset = 0
        self.enabled = False

    def _compute_phase_shift(self, correction):
        """Compute phase_shift[1024] from spectral items.
           phase_shift[k] = Σ mag_n * sin(n * k * 4 + pha_n)"""

    def _build_lut(self, direction):
        """Generate int8_t[1024] for MCU download"""

    def _send_lut_to_mcu(self):
        """Download LUT via load_phase_lut command"""

    def enable(self, enable):
        """Enable/disable phase stepping.
           Sets TMC5160 to 256 µsteps, disables interpolation,
           sets direct_mode=1 in GCONF."""
```

### G-code commands

| Command | Description |
|---------|-------------|
| `PHASE_STEPPING_ENABLE STEPPER=<name> [ENABLE=1]` | Enable/disable per stepper |
| `PHASE_STEPPING_RESET STEPPER=<name>` | Clear correction tables |
| `PHASE_STEPPING_STATUS` | Report current correction values and enabled state |
| `PHASE_STEPPING_CALIBRATE STEPPER=<name>` | Run full calibration (delegates to calibration module) |

### Integration points

1. **TMC5160 hook**: When enabled, sets `mres=0b0000` (256 µsteps), `intpol=0`,
   `direct_mode=1` via `SET_TMC_FIELD`. When disabled, restores original
   settings.

2. **Stepper enable hook**: Register with `stepper_enable` to be notified of
   motor enable/disable. On enable, re-sync phase offset by reading MSCNT from
   the TMC driver.

3. **Direction change hook**: Listen for `stepper:set_sdir_inverted` to swap
   forward/backward LUTs.

4. **Persistent storage**: Save correction tables as base64-encoded data in
   `SAVE_CONFIG` section:

```ini
#*# [phase_stepping stepper_x]
#*# correction_forward = <base64 encoded LUT>
#*# correction_backward = <base64 encoded LUT>
```

**Estimated code size**: ~400-500 lines of Python.

---

## Work Item 3: Host Python — Calibration Module

**New file: `klippy/extras/phase_stepping_calibration.py`**

Ports the full calibration algorithm from Prusa's `calibration.cpp`.

### Calibration pipeline (per axis, per harmonic)

```
calibrate_axis(axis)
 ├── Phase 1 — Speed Sweep (motor characterization)
 │     Move axis linearly from min_speed → max_speed → 0
 │     Capture accelerometer samples
 │     Chirp DFT for each enabled harmonic
 │     Detect harmonic peaks via least-squares fitting
 │
 └── For each harmonic at its resonant speed:
       ├── Phase 2 — Magnitude estimation
       │     Geometric magnitude search: sweep mag, find minimum residual
       │
       ├── Phase 3 — Phase optimization
       │     Sweep correction phase at fixed magnitude
       │     Find 2π-spaced minima → average to best phase
       │
       └── Phase 4 — Magnitude refinement
             Sweep magnitude with fixed optimal phase
             Compute score = min_magnitude / baseline_magnitude
```

### Key algorithm classes

```python
class SlidingDftWindow:
    """Streaming sliding-window DFT for chirp correlation."""
    def __init__(self, window_size):
        self.sample_idx = 0
        self.buffer = [0.0] * (2 * window_size + 1)
        self.sum_sin = 0.0
        self.sum_cos = 0.0

    def feed(self, sample, phase):
        """O(1) update: add new correlation pair, remove oldest."""

    def get_magnitude(self):
        """sqrt(sin²+cos²) normalized by window size."""

class CalibrateAxis:
    def __init__(self, phase_stepping_obj, accelerometer):
        self.ps = phase_stepping_obj

    def capture_speed_sweep(self, axis, direction):
        """Move axis min_speed→max_speed→0, record accel + position."""

    def chirp_dft_sweep(self, samples, harmonic, freq_range):
        """Sliding DFT with analytical chirp phase integration."""

    def find_harmonic_peaks(self, dft_results):
        """Least-squares harmonic peak fitting."""

    def find_optimal_correction(self, harmonic, speed):
        """Search best (mag, pha) via iterative sweeps."""

    def calibrate_axis(self, axis):
        """Full pipeline, returns MotorPhaseCorrection."""
```

### Chirp DFT mathematics

The motor accelerates/decelerates linearly. The instantaneous frequency is:

```
f(t) = start_freq + freq_accel·t          for 0 ≤ t ≤ T_ramp  (acceleration)
f(t) = top_freq - freq_accel·(t - T_ramp) for t > T_ramp      (deceleration)
```

The analytical phase (integral of frequency) is:

```
φ(t) = 2π·start_freq·t + π·freq_accel·t²           (acceleration)
φ(t) = 2π·top_freq·(t - T_ramp) - π·freq_accel·(t - T_ramp)²  (deceleration)
```

The sliding DFT correlates accelerometer samples against `sin(φ(t))` and
`cos(φ(t))`, subtracting the oldest correlation each step (O(1) per sample).

### Accelerometer integration

Reuses existing Kalico accelerometer infrastructure (`adxl345.py`, `mpu9250.py`,
`icm20948.py`):
- Use `mcu_adxl345.start_internal_client()` or equivalent to stream samples
- Process samples via the existing `process_accelerometer_data()` callback
- Correlate accelerometer timestamps with toolhead position

### Calibration configuration (per printer)

```python
CALIBRATION_CONFIG = {
    "default": {
        "speed_range": (0.1, 3.0),       # rev/s
        "enabled_harmonics": 0b1010,      # harmonics 2 and 4
        "max_movement_revs": 5.0,
        "coarse_movement_duration": 5.0,
        "peak_speed_shift": 0.9,
        "min_magnitude": 0.008,
        "max_magnitude": 0.4,
        "magnitude_quotient": 2.0,
        "analysis_window": 0.1,           # seconds
        "speed_sweep_bins": 400,
    }
}
```

**Estimated code size**: ~1000-1200 lines of Python.

---

## Work Item 4: TMC5160 Driver Changes

**Modify: `klippy/extras/tmc5160.py`**

### Required additions

1. **Add `XACTUAL` (0x21) to `ReadRegisters`** — required for reading current
   coil values in direct mode.

2. **`set_phase_stepping_mode()` method**:
   - `mres=0b0000` (256 microsteps via CHOPCONF)
   - `intpol=0` (disable interpolation)
   - `direct_mode=1` in GCONF
   - `ihold=irun` (hold current equals run current)

3. **`restore_normal_mode()` method**:
   - Restore original microsteps, interpolation, direct_mode

4. **`read_mscnt()` method** — expose MSCNT reading (already available via
   `_query_phase()` on the parent class).

5. **Coordinate with phase stepping module** — register enable/disable callbacks
   so that when phase stepping is activated and the motor is enabled, the TMC
   is reconfigured accordingly.

**Estimated code size**: ~80-100 lines added.

---

## Work Item 5: Step Generator & Stepper Changes

**Modify: `klippy/stepper.py`**

### Required additions

1. **`set_phase_stepping_params()` on `MCU_stepper`**:
   - Store phase stepping configuration reference
   - Allow phase stepping module to update phase offset

2. **Extend `_build_config()`**:
   - When phase stepping module is present, send `configure_phase_stepping`
     to the MCU during init

3. **Step distance adjustment**:
   - In phase stepping mode the TMC runs at 256 microsteps
   - Step distance becomes `rotation_dist / (full_steps * 256)` instead of
     `rotation_dist / (full_steps * microsteps)`
   - The step compressor handles the higher step count automatically

4. **Expose `get_mcu_position()`** — already available, consumed by phase
   module for phase tracking.

**Estimated code size**: ~50-80 lines added.

---

## Work Item 6: Communications Protocol

New MCU command IDs and response format for host ↔ firmware communication.

### Host → MCU commands

| Command | Parameters | Description |
|---------|-----------|-------------|
| `configure_phase_stepping` | `oid=%c stepper_oid=%c step_pin=%u dir_pin=%u zero_phase=%i steps_per_period=%u` | Set up phase step instance |
| `load_phase_lut` | `oid=%c data=%*s` | Download 1024-byte LUT |
| `enable_phase_stepping` | `oid=%c enable=%c` | Toggle enable |

### MCU → Host (optional status)

```
phase_step_status oid=%c position=%u phase=%u correction=%d
```

Sent periodically for monitoring (can be disabled in production).

### OID allocation

- Each phase stepping instance gets its own OID via `oid_alloc()`
- The OID is associated with a stepper OID (shared step/dir pins)
- Lookup: `oid_lookup(oid)` returns the `struct phase_stepper *`

---

## Work Item 7: Build System & Kconfig

**Modify: `src/Kconfig`**

```kconfig
config WANT_TMC_PHASE_STEP
    depends on HAVE_GPIO_BITBANGING
    bool "TMC Phase Stepping support"
    default n
    help
        Enables phase stepping burst engine on the MCU.
        Requires GPIO BSRR register and DMA controller.
```

**Modify: architecture Makefile** (e.g. `src/stm32/Makefile`):

```makefile
src-$(CONFIG_WANT_TMC_PHASE_STEP) += tmc_phase_step.c
```

---

## Implementation Order

| Step | Work Item | Dependencies | Effort |
|------|-----------|-------------|--------|
| 1 | TMC5160 driver changes (WI 4) | None | Small |
| 2 | MCU burst engine (WI 1) + protocol (WI 6) | None | Large |
| 3 | Build system (WI 7) | WI 1 | Trivial |
| 4 | Host phase stepping module (WI 2) | WI 4 | Medium |
| 5 | Stepper integration (WI 5) | WI 2, 4 | Small |
| 6 | Calibration module (WI 3) | WI 2 | Large |

**Rationale**: Start with MCU firmware (hardest, most novel) and TMC changes
(simplest integration). Then host module and stepper changes to wire everything
together. Calibration last since it depends on everything else working.

---

## Implementation Status (2026-06-13)

### Completed Items

| Work Item | Status | Notes |
|-----------|--------|-------|
| WI 4 — TMC5160 driver changes | **Done** | Delegation of `query_phase`, `set_phase_stepping_mode`, `restore_phase_stepping_mode` from `TMCCommandHelper` to `TMC5160` |
| WI 1 — MCU burst stepping engine | **Done** | `src/stm32/tmc_phase_step.c`: 3 MCU commands, ISR, LUT loading, GPIO pin management |
| WI 6 — Communications protocol | **Done** | `configure_phase_stepping`, `load_phase_lut` (chunked), `enable_phase_stepping` |
| WI 7 — Build system & Kconfig | **Done** | `CONFIG_WANT_TMC_PHASE_STEP`, STM32 Makefile target |
| WI 2 — Host phase stepping module | **Done** | `klippy/extras/phase_stepping.py` (~310 lines) |
| WI 3 — Calibration module | **Skeleton** | `klippy/extras/phase_stepping_calibration.py` exists but not tested |
| WI 5 — Stepper integration | **Partial** | `stepper_get_position_by_oid()` exposed; rotation distance adjustment works |

### Multi-stepper support (fixed)

`PHASE_STEPPING_ENABLE` works for both `stepper_x` and `stepper_y`
concurrently. No `Invalid oid type` shutdown. MCU stays in `ready` state.

**Root cause** of the previous crash: `PhaseStepping` added
`configure_phase_stepping` from a `klippy:connect` handler. The MCU
class is also a `klippy:connect` handler and is registered first
(MCUs are instantiated before user modules), so `mcu._send_config`
ran and shipped the config batch before either phase_stepping
instance had a chance to add its command. With a single instance
this sometimes worked by accident; with two instances the second
one's OID was never allocated and `enable_phase_stepping` triggered
`Invalid oid type` on the MCU.

**Fix** (`klippy/extras/phase_stepping.py:142`): register the config
command in `klippy:mcu_identify` via `mcu.register_config_callback`,
so `mcu.create_oid()` and `mcu.add_config_cmd()` execute inside
`mcu._send_config` (after the `_config_callbacks` fire, before
`allocate_ods` is computed) — the same pattern used by
`tmc_uart.MCU_TMC_uart_bitbang`.

### What Works

- `PHASE_STEPPING_STATUS` — reports enabled state and correction items
- `PHASE_STEPPING_ENABLE STEPPER=stepper_x ENABLE=1` and
  `PHASE_STEPPING_ENABLE STEPPER=stepper_y ENABLE=1` — both can be
  active concurrently:
  1. `TMC5160.set_phase_stepping_mode()` — sets mres=0 (256µstep), intpol=0, ihold=irun via SPI
  2. `TMC5160.query_phase()` — reads MSCNT register for phase sync (✓ values ~136, ~913 observed)
  3. LUT chunked loading — 3 chunks of 200 bytes each (256 forward + 256 backward = 512 bytes)
  4. `enable_phase_stepping oid=%c enable=%c` — starts ISR on MCU
  5. Rotation distance adjustment — sets `rotation_dist * 256 / microsteps`
- ISR runs at 1 kHz, phase tracking correct, MCU stable (tested 20+ seconds per stepper, both concurrently)

### What Does NOT Work (known gaps)

- ISR runs at 1 kHz (was reduced from 10 kHz for stability)
- LUT is 256 entries (was reduced from 1024 due to heap pressure on STM32F446)
- Calibration pipeline (`PHASE_STEPPING_CALIBRATE`) not validated end-to-end
- Full enable flow (TMC mode set, MSCNT sync, LUT load, dwell) is currently
  stripped in `_enable_stripped` to a minimal version that just sends the
  enable command. Restoring it requires the user to validate the new
  direction command + ISR init under load.

### Newly wired

- `_handle_dir_inverted` — listens on the correct event
  `stepper:set_dir_inverted` (the prior registration was for
  `stepper:set_sdir_inverted`, a typo that meant the handler never
  fired). On dir inversion, sends
  `set_phase_stepping_direction oid=%c forward=%c` to swap
  `current_lut` between forward and backward LUTs.
- `set_phase_stepping_direction` MCU command — added in
  `src/stm32/tmc_phase_step.c`. Tracks `ps->forward` and
  updates `ps->current_lut` accordingly. Idempotent: skips the
  pointer swap if the direction hasn't actually changed.
- `phase_stepper` struct gained a `uint8_t forward` field.
  `command_enable_phase_stepping` and
  `command_set_phase_stepping_direction` both consult it when
  re-arming `current_lut`.
- ISR rate restored to 10 kHz (`REFRESH_FREQ 10000` in
  `src/stm32/tmc_phase_step.c`).
- LUT restored to 1024 entries by moving
  `phase_shift_luts[MAX_PHASE_STEPPERS][2][LUT_SIZE]` into static
  BSS (16 KB at MAX_PHASE_STEPPERS=8) instead of embedding
  `int8_t phase_shift_lut[LUT_SIZE]` per heap-allocated
  `phase_stepper`. Each `phase_stepper` now holds a
  `uint8_t lut_index` instead of the two inline arrays; the
  configure / load / enable / direction commands index through it.
  Phase correction granularity is now 1 electrical microstep
  (MOTOR_PERIOD / LUT_SIZE = 1024 / 1024 = 1) instead of 4.

### Changes from Original Plan

| Change | Reason | Plan Reference |
|--------|--------|---------------|
| LUT size: 1024 → 256 entries | STM32F446 heap exhaustion (2×2100 bytes per stepper struct) | See §LUT Size Reduction below |
| ISR frequency: 10 kHz → 1 kHz | Stability; 1 ms tick sufficient for phase tracking at rest | Original §C |
| Protocol: `load_phase_lut` now chunked | `PT_buffer.encode()` uses uint8 for length (256 max); total LUT data (512 bytes) sent in 200B chunks with `offset=%hu` field | Original §B |
| Python: `_enable` uses dwell(0.5s) | Serial queue flush between LUT chunks and ISR enable; prevents MCU buffer overflow | — |
| `configure_phase_stepping` registered via `mcu.register_config_callback` (not `klippy:connect`) | Multi-stepper fix — see "Multi-stepper support" above | — |
| `stepper_get_position_by_oid()`: added NULL guard | Prevent hard fault if stepper OID is invalid during ISR | Original §WI 5 |
| MCU: removed `sched_add_timer` from enable handler | Crashed MCU; timer started at boot via `DECL_INIT` instead | — |

### Debug Iteration Flow (`./iter.sh`)

```
./iter.sh [--force]
  ├── rsync → printer (~/klipper/)
  ├── detect C changes (git diff HEAD -- src/)
  ├── if C changes: make olddefconfig + build + flash MCU
  ├── FIRMWARE_RESTART → reconnects Klippy
  ├── _CG28 → home
  ├── pull klippy.log
  └── check for Move out of range / config error / traceback / fatal
```

`--force` triggers rebuild even without C changes.

### Next Steps

1. Wire `_handle_dir_inverted` to swap `current_lut` between forward / backward LUTs (or set a direction flag the ISR reads)
2. Restore `_disable` flow — call `restore_phase_stepping_mode` on the TMC, undo the rotation-distance change, gate the ISR enable
3. Restore ISR to 10 kHz (multistepper is now stable; original `REFRESH_FREQ` constant)
4. Restore LUT to 1024 entries — move `phase_shift_lut` out of the heap-allocated `phase_stepper` struct into a static BSS array (`static int8_t phase_shift_luts[MAX_PHASE_STEPPERS][2][1024]`)
5. Test `PHASE_STEPPING_CALIBRATE` end-to-end with the ADXL345
6. Update this doc's Implementation Status table after each completed step

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **Burst stepping (not XDIRECT)** | Works with TMC5160 step/dir interface; no SPI writes from ISR context |
| **10 kHz refresh rate** | Same as Prusa MK4/CoreOne; 100 µs period gives time for DMA setup + 200 event slots |
| **STM32-only initially** | Prusa's implementation is STM32-specific (GPIO BSRR + DMA). Targeting **STM32F446xx** (Octopus board). Other architectures can follow |
| **256 microsteps during phase stepping** | TMC5160 max resolution; each µstep = 1/1024 electrical period, matching LUT granularity |
| **Host computes LUT, MCU applies it** | Keeps complex math in Python; keeps MCU ISR fast (~7 µs per Prusa measurements) |
| **Shared step pin** | Normal stepper engine and burst engine share the step pin. Burst steps are correction microsteps injected between normal steps |

---

## Motor Phase Model

```
full_steps_per_rev  = 200          (configurable)
microsteps          = 16           (normal) / 256 (phase stepping)
steps_per_rev       = full_steps_per_rev * microsteps
MOTOR_PERIOD        = 1024         (TMC electrical ticks per revolution)

electrical_phase(position) = (position * MOTOR_PERIOD / steps_per_rev + phase_origin) % 1024

phase_shift[k] = Σ_{n ∈ harmonics} mag_n · sin(n · k · 4 + pha_n)   for k ∈ [0, 1023)
```

Each `SpectralItem(mag_n, pha_n)` represents a harmonic defect in the motor's
transfer function. The sum of all harmonics produces a correction offset per
microstep.

---

## Files Summary

```
# New files
klippy/extras/phase_stepping.py                         # Host module (~450 lines)
klippy/extras/phase_stepping_calibration.py              # Calibration (~1100 lines)
src/tmc_phase_step.c                                     # MCU burst engine (~900 lines)
src/tmc_phase_step.h                                     # MCU header (~80 lines)

# Modified files
klippy/extras/tmc5160.py                                 # +100 lines (phase stepping mode)
klippy/stepper.py                                        # +80 lines (phase stepping params)
src/Kconfig                                              # +8 lines (new config option)
src/stm32/Makefile                                       # +1 line (build target)
```

**Total estimated**: ~2700 lines of new code, ~190 lines of modifications.

---

## Known Issues / Future Work

### LUT Size Reduction (1024 → 256)

The correction LUT was reduced from 1024 entries per direction (2048 bytes total)
to 256 entries (512 bytes total) due to heap memory exhaustion on the STM32F446.

**Root cause**: Each `phase_stepper` struct is heap-allocated via `oid_alloc()`.
With the LUT arrays inline, each instance consumed ~2100 bytes. Two steppers
(X+Y) required ~4200 bytes of heap. The STM32F446 has only 128 KB RAM and
the static allocations (BSS/data) already consume nearly all of it, leaving
insufficient heap for the full-size LUTs.

**Impact**: Phase correction granularity is now 4 electrical microsteps per
LUT entry (MOTOR_PERIOD / LUT_SIZE = 1024 / 256). The ISR indexes the LUT
with `motor_phase / 4` instead of direct lookup.

**Fix plan**:
1. Move LUT arrays to static BSS by pre-allocating them globally
   (`static int8_t phase_shift_luts[MAX_PHASE_STEPPERS][2][LUT_SIZE]`)
   rather than embedding them in the heap-allocated struct.
2. Restore LUT to 1024 entries for full microstep resolution.
3. Consider using flash memory (PROGMEM) for the LUT storage since it is
   write-once-per-calibration but read at 10 kHz by the ISR.
