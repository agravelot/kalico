# Phase Stepping Roadmap — Remaining Work

Last updated: 2026-06-13

The 7 original work items in `phase_stepping_plan.md` are complete
(see the Implementation Status table in that document). This file
captures the **remaining work** identified during a post-completion
review, broken into 4 stages (tiers) ordered by priority and ROI.

## Stage Map

| Stage | Theme | Status | Effort |
|-------|-------|--------|--------|
| **Tier 1** | Usability (must-have for production) | Pending | ~½ day |
| **Tier 2** | Quality wins | Pending | ~1-2 days |
| **Tier 3** | Technical debt (Prusa parity) | Pending | ~3-5 days |
| **Tier 4** | Coverage / robustness | Pending | ~1-2 days |

The acceptance gate for "production ready" is completing Tier 1
plus the smoke test. Tier 2 is the next round of polish. Tier 3
and 4 are long-term.

---

## Tier 1 — Usability

Make the phase stepping port usable day-to-day. After Tier 1, a
user can run calibration once, save it, and have it survive
restarts.

### T1.1 — SAVE_CONFIG persistence

**Goal**: Calibration results survive `FIRMWARE_RESTART`. After
running `PHASE_STEPPING_CALIBRATE`, the user runs `SAVE_CONFIG`
and the corrections land in `printer.cfg` under
`[phase_stepping stepper_x]`.

**File**: `klippy/extras/phase_stepping.py`

**Changes**:
1. In `PhaseStepping.__init__` (~line 75), look up the `configfile`
   object and register a status handler:
   ```python
   self.configfile = self.printer.lookup_object("configfile")
   self.configfile.register_status_handler(self._handle_configfile_status)
   ```
2. Add `_handle_configfile_status(self, status)`: when
   `status.get("phase_stepping_persist", False)` is true (or always
   if simpler), push:
   ```python
   status["phase_stepping %s" % self.name] = {
       "correction_forward": self._serialize_correction(self.correction_fwd),
       "correction_backward": self._serialize_correction(self.correction_bwd),
   }
   ```
3. In `cmd_PHASE_STEPPING_SET_HARMONIC` (line 360) and after
   `CalibrateAxis.calibrate()` returns in
   `cmd_PHASE_STEPPING_CALIBRATE` (line 351), call
   `self.configfile.set_status(...)` to push the new values to the
   in-memory config and `gcmd.respond_info` to tell the user to run
   `SAVE_CONFIG`.

**Why this works**: `_serialize_correction` and `_load_correction`
are already there. The save path is the only missing wire.

**Verify**:
1. Run `PHASE_STEPPING_CALIBRATE STEPPER=stepper_x`
2. `SAVE_CONFIG`
3. `FIRMWARE_RESTART`
4. `PHASE_STEPPING_STATUS STEPPER=stepper_x` — H1/H3 corrections
   must still be present

### T1.2 — `stepper_y` calibration parity

**Goal**: Confirm `stepper_y` calibrates and runs the same way as
`stepper_x`.

**No code changes expected.** Test pass:

1. `PHASE_STEPPING_CALIBRATE STEPPER=stepper_y` — must complete
   without crashing and return reasonable H1/H3 values
2. `PHASE_STEPPING_ENABLE STEPPER=stepper_y ENABLE=1`
3. 10 round-trips of 50mm at 9000 mm/min
4. `LOST_STEPS=0` and position stable

**Risk**: Y-axis on a Voron2 has different accelerometer axis
projection and a different rail length. The centering code
branches on `rail_name` (line 490) and `ax_idx` is in place (line
146), so we expect it to work, but unverified.

**Backup plan**: If `_chirp_dft_sweep` or `_dft_at_harmonic` fail
on Y, add an `ax_idx` per-stepper mapping (currently hardcoded
`{"stepper_x": 1, "stepper_y": 2}` in line 146 — for a CoreXY
printer, this may need to be different).

### T1.3 — Real-print verification

**Goal**: Confirm phase stepping works under accel/decel, not just
constant velocity.

**No code changes.** Test sequence:

1. Enable both `stepper_x` and `stepper_y` with calibrated
   corrections
2. `G91` (relative mode), then loop 5 times:
   ```
   G1 X100 Y100 F9000
   G1 X-100 Y-100 F9000
   ```
3. `G90` (absolute), then `M114` to check position
4. Inspect TMC `LOST_STEPS` for both X and Y steppers
5. Inspect Beacon `PHASE_STEPPING_STATUS` for both

**Acceptance**:
- LOST_STEPS = 0 for both axes throughout
- Final position back to start
- No `MCU shutdown` or `Move out of range` errors in klippy.log
- Ringing measured by accelerometer reduced vs uncorrected baseline

**Optional** (if time allows): try a circle (`G2` arc) at
9000 mm/min, or a more complex path mimicking a print move.

### T1.4 — Dead code cleanup

**Files**: `klippy/extras/phase_stepping.py`,
`klippy/extras/phase_stepping_calibration.py`

**Audit candidates** (read-only `git grep` first):
- `_move_to_start` in `phase_stepping_calibration.py:107` — was
  replaced by inline centering in `calibrate()` (line 468)
- `MotorPhaseCorrection.is_empty()` in `phase_stepping.py:56`
- `SlidingDftWindow.feed_zero()` in `phase_stepping_calibration.py:51`

**Action**: Remove any of the above that are not called. Confirm
with `git grep` first that each is dead.

**Verify**: `git grep <symbol>` should return zero hits after
removal.

### Tier 1 execution order

```
T1.1 (wire SAVE_CONFIG) ──────┐
T1.4 (cleanup, small) ────────┤
                              ├── T1.2 (test stepper_y, with save active)
                              ├── T1.3 (real-print verification)
                              └── Final: SAVE_CONFIG → FIRMWARE_RESTART → confirm
```

T1.1 and T1.4 are independent. T1.1 must land before the final
"save and restart" smoke test. T1.2 should run with T1.1 in place
so the calibration result lands in config.

---

## Tier 2 — Quality wins

Make the calibration and runtime behavior more correct. After
Tier 2, the residual error between forward and backward motion
is reduced, and the integration with Klipper's motor on/off is
safe.

### T2.1 — Bidirectional calibration (separate fwd/bwd sweeps)

**Goal**: Run a full coarse+fine phase sweep in **both** forward
and backward directions, write independent corrections to
`correction_fwd` and `correction_bwd`.

**Why this matters**: The current calibration stores the same
mag/pha in both directions. End-to-end numbers on the Voron2
show forward response ~40 (post-correction) and backward response
~120 — the asymmetry is real and significant. Prusa's approach
treats forward and backward as independent calibrations.

**File**: `klippy/extras/phase_stepping_calibration.py`

**Changes**:
1. In `CalibrateAxis.calibrate()` (line 453), wrap the per-harmonic
   loop in an outer direction loop:
   ```python
   for direction in ("forward", "backward"):
       for harmonic in enabled_harmonics:
           mag, pha = self._find_optimal(harmonic, speed, ..., direction=direction)
           if direction == "forward":
               self.ps.correction_fwd.set_harmonic(harmonic, mag, pha)
           else:
               self.ps.correction_bwd.set_harmonic(harmonic, mag, pha)
           self.ps._send_lut()
   ```
2. In `_sweep_phase` (line 276), accept a `direction` arg and flip
   the move sign in the trial move loop:
   ```python
   sign = 1 if direction == "forward" else -1
   fm.manual_move(stepper, sign * dist_mm, speed_mm_s, accel=50.0)
   ```
3. In `_dft_at_harmonic` (line 405) and `_chirp_dft_sweep` (line
   119), no change needed — the DFT correlates against `sin(nωt)`
   which is sign-invariant.

**Cost**: ~3x calibration time. Each harmonic takes a coarse
sweep (80 trials × ~200ms each = 16s) + fine sweep (80 trials ×
~100ms = 8s) per direction. Two harmonics × two directions ≈ 90s
total.

**Verify**: After calibration, run separate forward and backward
round-trips at 9000 mm/min and check that response is reduced in
both directions (should be ~40 in both, not 40 vs 120).

### T2.2 — Stepper enable coordination (M84 / motor on/off)

**Goal**: On `M84` (motor off), auto-disable phase stepping ISR.
On the next `M17` or motor-on, re-enable and re-sync MSCNT.

**Why this matters**: When the motor is unpowered, the rotor
drifts and MSCNT no longer reflects the actual rotor position.
Re-enabling with a stale MSCNT would produce a 1-time burst on
the first move (similar to the early-init bug fixed in commit
`2d1855ba`). Auto-disable + re-sync makes the integration
transparent.

**File**: `klippy/extras/phase_stepping.py`

**Changes**:
1. In `PhaseStepping.__init__`, register listeners:
   ```python
   self.printer.register_event_handler("stepper_enable:motor_off",
                                        self._handle_motor_off)
   self.printer.register_event_handler("stepper_enable:motor_on",
                                        self._handle_motor_on)
   ```
2. Add `_handle_motor_off`: if `self.enabled`, send
   `enable_cmd(0)` to disable ISR, store `_was_enabled_before_off`
   flag, do **not** touch TMC mode (motor is being powered down).
3. Add `_handle_motor_on`: if `self._was_enabled_before_off`:
   - `_sync_phase_offset` (reads MSCNT and sends zero_phase)
   - `enable_cmd(1)` to re-arm ISR
   - clear the flag

**Risk**: The `stepper_enable` events fire from a different
module; coupling between modules adds fragility. The current
implementation already has `_enable` doing the right thing
(MSCNT read → zero_phase → enable) so the risk is mostly in
correctly hooking the events.

**Verify**: Test sequence:
1. `PHASE_STEPPING_ENABLE STEPPER=stepper_x ENABLE=1`
2. `M84` (motor off) — klippy.log should show ISR disabled
3. `M17` (motor on) — klippy.log should show MSCNT re-synced
4. Round-trip move — LOST_STEPS=0

### T2.3 — Better status reporting

**Goal**: `PHASE_STEPPING_STATUS` returns both forward and
backward harmonics, last calibration timestamp, `lut_loaded`
flag, `zero_phase_synced` flag.

**File**: `klippy/extras/phase_stepping.py`

**Changes**: Extend `get_status()` (line 314):
```python
def get_status(self, eventtime=None):
    fwd = {n: self.correction_fwd.get_harmonic(n) for n in range(1, 17)}
    bwd = {n: self.correction_bwd.get_harmonic(n) for n in range(1, 17)}
    return {
        "enabled": self.enabled,
        "lut_loaded": self.phase_oid is not None,
        "zero_phase_synced": self.zero_phase != 0,
        "harmonic_forward": {k: v for k, v in fwd.items() if v[0] != 0.0},
        "harmonic_backward": {k: v for k, v in bwd.items() if v[0] != 0.0},
    }
```

Update `cmd_PHASE_STEPPING_STATUS` (line 341) to print both
directions.

---

## Tier 3 — Technical debt (Prusa parity)

Cross-cutting and firmware-level work. Not on the critical path
for production use, but eliminates workarounds and brings the
implementation closer to Prusa's.

### T3.1 — Chelper `message_fill()` overflow proper fix

**Goal**: Remove the `_send_lut` chunk=50 workaround by fixing the
underlying chelper + MCU protocol.

**Why this matters**: Currently `load_phase_lut` is split into
41 chunks per stepper, each ~55 B. A proper flexible-array
`struct queue_message` (in `klippy/chelper/msgblock.h:22`) plus a
matching MCU receive-buffer bump would allow a single chunk.
Simpler code, less risk of partial-load inconsistency.

**Files**:
- `klippy/chelper/msgblock.h:22` — change
  `uint8_t msg[MESSAGE_MAX=64]` to `uint8_t msg[]`
- `klippy/chelper/msgblock.c:137-143` — already a single `memcpy`,
  no change needed
- `src/serial.c` (or wherever USB receive ring buffer is) — bump
  buffer size to match

**Risk**: Cross-cutting change. Every `message_fill` /
`message_alloc` call site needs to be re-verified. Risk of
breaking other Klippy code that was relying on the implicit 64-B
cap.

**Cost**: 1-2 days of careful change + smoke test of every G-code
that uses large payloads (e.g. `SET_TMC_FIELD` with long
register-list values).

### T3.2 — MCU-driven phase sweep (Prusa M973-style)

**Goal**: Move the calibration sweep to the MCU. A single G-code
triggers a firmware-internal sequence: drive stepper, sample
accelerometer, integrate, decide best (mag, pha), write LUT.

**Why this matters**: ~5-10x faster calibration. Host-driven sweep
takes ~90s (T2.1) for two harmonics × two directions. MCU-driven
sweep can do it in ~10s.

**Files**:
- `src/stm32/tmc_phase_step.c` — add `phase_step_calibrate`
  command and a calibration state machine
- `klippy/extras/phase_stepping_calibration.py` — replace
  host-driven moves with a single command + status wait

**Cost**: Significant C-side work. The host-driven path is
already correct; this is for speed, not correctness.

---

## Tier 4 — Coverage

Edge cases and broader support.

### T4.1 — ADXL345 / MPU9250 testing

**Goal**: Confirm calibration works with non-Beacon accelerometers.

**No code change expected.** The `start_internal_client` /
`finish_measurements` / `get_samples` API is standard across
Kalico's accelerometer drivers (per
`klippy/extras/adxl345.py`, `klippy/extras/mpu9250.py`,
`klippy/extras/icm20948.py`).

**Test**: with an ADXL345 wired up, run
`PHASE_STEPPING_CALIBRATE STEPPER=stepper_x` and verify it
completes.

**Backup plan**: If `get_samples()` differs in shape (some
drivers may return `Accel_Measurement` namedtuples, others raw
tuples), add a normalization shim.

### T4.2 — Algorithm tuning

**Goal**: Verify `gone_worse >= 2` early termination in
`_find_optimal` (line ~244) isn't skipping the real optimum.

**Test**: Run 3-4 calibrations on the same stepper with the same
config. Compare H1/H3 values. If they vary by more than 10%, the
early-termination is too aggressive and we should run the full
sweep.

**Backup plan**: Add a `MIN_TRIALS` floor or use a smooth
interpolation across mag trials.

---

## Decision Log

| Question | Answer | Implemented in |
|----------|--------|----------------|
| Which tier to tackle first? | Tier 1 (usability) | This PR |
| Bidirectional approach (Tier 2)? | Same as Prusa — full coarse+fine sweep in both directions | T2.1 |
| M84 behavior (Tier 2)? | Auto-disable ISR on motor_off; re-enable + MSCNT re-sync on motor_on | T2.2 |
| Ship current calibration to config? | Yes — save current H1 mag=0.016 pha=5.9993, H3 mag=0.008 pha=0.7312 | T1.1 |

---

## Risks & Open Questions

1. **Calibration stability**: T1.1 wires save, but if the user
   re-runs calibration and gets different H1/H3 values, is that
   noise or algorithm variance? Suggest averaging 2 runs before
   committing to config.
2. **TMC SPI races**: During `_enable`, we read MSCNT, send LUT,
   set zero_phase, arm ISR. If a `set_dir_inverted` event fires
   between those, the LUT slot might mismatch. The current code
   only swaps `current_lut` pointer (no re-read of MSCNT), so it
   should be safe — but worth a stress test.
3. **TMC error fallback**: If TMC reports a `stall_guard` or `ot`
   event during phase stepping, do we disable phase stepping and
   surface the error? Currently nothing in `phase_stepping.py`
   listens for TMC errors. **T2.2 work will add this, but not
   Tier 1.**
4. **Calibration speed**: Tier 1's host-driven calibration takes
   ~45s per stepper for 2 harmonics. Tier 2 (T2.1) makes it
   ~90s. Acceptable for a one-shot operation. Tier 3 (T3.2) would
   bring it under 10s.
