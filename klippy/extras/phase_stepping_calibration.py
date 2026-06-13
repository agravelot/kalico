# Phase stepping calibration
#
# Calibrates per-motor phase correction tables by performing speed
# sweeps, accelerometer-based DFT analysis, and iterative optimization
# of correction magnitude and phase for each harmonic.
#
# Copyright (C) 2024  Kalico contributors
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, time

MOTOR_PERIOD = 1024
SIN_FRACTION = 4
SIN_PERIOD = SIN_FRACTION * MOTOR_PERIOD

# Default calibration parameters
DEFAULT_SPEED_RANGE = (0.1, 3.0)  # rev/s
DEFAULT_ENABLED_HARMONICS = 0b1010  # harmonics 2 and 4
DEFAULT_MAX_MOVEMENT_REVS = 5.0
DEFAULT_COARSE_DURATION = 5.0
DEFAULT_PEAK_SPEED_SHIFT = 0.9
DEFAULT_MIN_MAGNITUDE = 0.008
DEFAULT_MAX_MAGNITUDE = 0.4
DEFAULT_MAGNITUDE_QUOTIENT = 2.0
DEFAULT_WINDOW_SIZE = 0.1  # seconds
DEFAULT_SPEED_SWEEP_BINS = 400
DEFAULT_PARAM_SWEEP_BINS = 400
# Sample rate for accelerometers that don't expose get_sample_rate()
# (e.g. beacon). Computed from the first two samples when available.
DEFAULT_ACCEL_SAMPLE_RATE = 1300


class SlidingDftWindow:
    def __init__(self, window_size):
        self.window_size = window_size
        self.buf_len = 2 * window_size + 1
        self.samples_sin = [0.0] * self.buf_len
        self.samples_cos = [0.0] * self.buf_len
        self.idx = 0
        self.sum_sin = 0.0
        self.sum_cos = 0.0

    def feed(self, sample, sin_val, cos_val):
        corr_sin = sample * sin_val
        corr_cos = sample * cos_val
        self.sum_sin += corr_sin - self.samples_sin[self.idx]
        self.sum_cos += corr_cos - self.samples_cos[self.idx]
        self.samples_sin[self.idx] = corr_sin
        self.samples_cos[self.idx] = corr_cos
        self.idx = (self.idx + 1) % self.buf_len

    def feed_zero(self):
        self.sum_sin -= self.samples_sin[self.idx]
        self.sum_cos -= self.samples_cos[self.idx]
        self.samples_sin[self.idx] = 0.0
        self.samples_cos[self.idx] = 0.0
        self.idx = (self.idx + 1) % self.buf_len

    def get_magnitude(self):
        n = 2 * self.window_size + 1
        if n == 0:
            return 0.0
        return (2.0 / n) * math.sqrt(
            self.sum_sin * self.sum_sin + self.sum_cos * self.sum_cos)


class CalibrationConfig:
    def __init__(self, speed_range=None, enabled_harmonics=None,
                 max_movement_revs=None, coarse_duration=None,
                 peak_speed_shift=None, min_magnitude=None,
                 max_magnitude=None, magnitude_quotient=None,
                 window_size=None, speed_sweep_bins=None,
                 param_sweep_bins=None):
        self.speed_range = speed_range or DEFAULT_SPEED_RANGE
        self.enabled_harmonics = (enabled_harmonics
                                  if enabled_harmonics is not None
                                  else DEFAULT_ENABLED_HARMONICS)
        self.max_movement_revs = max_movement_revs or DEFAULT_MAX_MOVEMENT_REVS
        self.coarse_duration = coarse_duration or DEFAULT_COARSE_DURATION
        self.peak_speed_shift = peak_speed_shift or DEFAULT_PEAK_SPEED_SHIFT
        self.min_magnitude = min_magnitude or DEFAULT_MIN_MAGNITUDE
        self.max_magnitude = max_magnitude or DEFAULT_MAX_MAGNITUDE
        self.magnitude_quotient = (magnitude_quotient
                                   or DEFAULT_MAGNITUDE_QUOTIENT)
        self.window_size = window_size or DEFAULT_WINDOW_SIZE
        self.speed_sweep_bins = speed_sweep_bins or DEFAULT_SPEED_SWEEP_BINS
        self.param_sweep_bins = param_sweep_bins or DEFAULT_PARAM_SWEEP_BINS

    def get_enabled_harmonics(self):
        return [h for h in range(1, 17) if self.enabled_harmonics & (1 << h)]


class CalibrateAxis:
    def __init__(self, phase_stepping, config=None):
        self.ps = phase_stepping
        self.printer = phase_stepping.printer
        self.config = config or CalibrationConfig()
        self.gcode = self.printer.lookup_object("gcode")

    def _get_toolhead_and_accel(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        accel_chip_name = gcmd.get("ACCEL_CHIP", None)
        if accel_chip_name is None:
            raise gcmd.error("ACCEL_CHIP parameter required")
        accel_chip = self.printer.lookup_object(accel_chip_name.strip())
        return toolhead, accel_chip

    def _move_to_start(self, toolhead, print_time):
        toolhead.wait_moves()
        kin = toolhead.get_kinematics()
        X, Y, Z, E = toolhead.get_position()
        # Move to center-ish for sweep
        pos = [X, Y, Z, E]
        toolhead.manual_move(pos, 100.0)

    def _compute_electrical_speed(self, rev_per_s, motor_steps=200):
        # Electrical frequency in Hz = rev/s * pole_pairs
        return rev_per_s * motor_steps / 2  # 50 pole pairs for 200-step motor

    def _chirp_dft_sweep(self, samples, sample_rate, harmonic,
                         start_speed, end_speed, motor_steps,
                         bins=None):
        if bins is None:
            bins = self.config.speed_sweep_bins

        start_freq = self._compute_electrical_speed(start_speed, motor_steps)
        end_freq = self._compute_electrical_speed(end_speed, motor_steps)

        n_samples = len(samples)
        total_time = n_samples / sample_rate
        ramp_time = total_time / 2.0 if total_time > 0 else 0.0

        results = []
        window_samples = int(self.config.window_size * sample_rate)
        if window_samples < 1:
            window_samples = 1
        dft = SlidingDftWindow(window_samples)

        for _ in range(2 * window_samples + 1):
            dft.feed_zero()

        step = max(1, n_samples // bins)
        freq_accel = (end_freq - start_freq) / ramp_time if ramp_time > 0 else 0

        # Accel samples are Accel_Measurement(time, x, y, z) namedtuples.
        # Project onto the axis most relevant to this stepper.
        ax_idx = {"stepper_x": 1, "stepper_y": 2}.get(
            self.ps.stepper_name, 1)
        for i in range(0, n_samples, step):
            t = i / sample_rate
            if t <= ramp_time:
                freq = start_freq + freq_accel * t
                phase = 2.0 * math.pi * (start_freq * t
                                         + 0.5 * freq_accel * t * t)
            else:
                t_rel = t - ramp_time
                freq = end_freq - freq_accel * t_rel
                phase = 2.0 * math.pi * (end_freq * t_rel
                                         - 0.5 * freq_accel * t_rel * t_rel)

            h_freq = freq * harmonic
            h_phase = phase * harmonic

            if h_freq >= sample_rate / 2.0:
                s = 0.0
                c = 0.0
            else:
                s = math.sin(h_phase)
                c = math.cos(h_phase)

            end_idx = min(i + step, n_samples)
            for j in range(i, end_idx):
                dft.feed(samples[j][ax_idx], s, c)

            speed = freq / motor_steps * 2.0  # rev/s
            results.append((speed, dft.get_magnitude()))

        return results

    def _find_peaks(self, values, min_prominence=0.15):
        n = len(values)
        if n < 3:
            return []
        signal_range = max(values) - min(values)
        if signal_range == 0:
            return []

        # Left and right minima
        left_min = [values[0]] * n
        right_min = [values[-1]] * n
        for i in range(1, n):
            left_min[i] = min(left_min[i - 1], values[i])
        for i in range(n - 2, -1, -1):
            right_min[i] = min(right_min[i + 1], values[i])

        peaks = []
        for i in range(1, n - 1):
            if values[i] > values[i - 1] and values[i] > values[i + 1]:
                prominence = (values[i] - max(left_min[i], right_min[i])
                              ) / signal_range
                if prominence >= min_prominence:
                    peaks.append((i, values[i], prominence))

        if not peaks:
            max_idx = max(range(n), key=lambda x: values[x])
            peaks.append((max_idx, values[max_idx], 0.0))

        return peaks

    def _harmonic_fit(self, speeds_by_harmonic, peaks_by_harmonic):
        enabled = self.config.get_enabled_harmonics()
        if not enabled:
            return {}
        n_harmonics = len(enabled)
        if n_harmonics == 0:
            return {}

        # Use the most prominent peak from the lowest harmonic as anchor
        anchor_h = enabled[0]
        best_peaks = max(peaks_by_harmonic[anchor_h],
                         key=lambda p: p[2], default=None)
        if best_peaks is None:
            return {}

        anchor_idx = best_peaks[0]
        anchor_speed = speeds_by_harmonic[anchor_h][anchor_idx]

        result = {}
        for h in enabled:
            if h == anchor_h:
                result[h] = anchor_speed
                continue
            # Find closest peak to predicted speed
            predicted = anchor_speed * anchor_h / h
            speeds = speeds_by_harmonic[h]
            peaks = peaks_by_harmonic[h]
            if not peaks:
                result[h] = predicted
                continue
            best = min(peaks, key=lambda p: abs(speeds[p[0]] - predicted))
            result[h] = speeds[best[0]]

        return result

    def _find_optimal(self, harmonic, speed, motor_steps,
                      accelerometer, gcmd):
        # Iterative 2D search: at each magnitude, sweep phase and find
        # the phase that minimizes the harmonic response. Stop when
        # increasing magnitude no longer reduces the response.
        min_mag = self.config.min_magnitude
        max_mag = self.config.max_magnitude
        quotient = self.config.magnitude_quotient

        best_mag = 0.0
        best_pha = 0.0
        best_response = float("inf")
        gone_worse = 0
        mag = min_mag

        while mag <= max_mag:
            pha, response = self._sweep_phase(
                harmonic, mag, speed, motor_steps, accelerometer, gcmd)
            gcmd.respond_info("    mag=%.4f: best_pha=%.4f response=%.4f"
                              % (mag, pha, response))
            if response < best_response * 0.95:
                best_response = response
                best_mag = mag
                best_pha = pha
                gone_worse = 0
            else:
                gone_worse += 1
                if gone_worse >= 2:
                    break
            mag *= quotient
        return best_mag, best_pha

    def _sweep_phase(self, harmonic, mag, speed, motor_steps, accelerometer, gcmd):
        # Host-driven phase sweep: for a fixed magnitude, run several
        # constant-velocity moves with different correction phases and
        # measure the harmonic response at each. The response curve
        # over phase has a single minimum — that is the optimal phase.
        n_phases = max(8, int(self.config.param_sweep_bins / 5))
        toolhead = self.printer.lookup_object("toolhead")
        force_move = self.printer.lookup_object("force_move")
        stepper = force_move.lookup_stepper(self.ps.stepper_name)
        rot_dist = self._get_rotation_distance(stepper)
        if rot_dist is None or rot_dist <= 0:
            raise gcmd.error(
                "Cannot determine rotation_distance for %s"
                % self.ps.stepper_name)
        # Move distance: 0.1 rev per trial. At 1 rev/s that's 100ms,
        # enough for several 100ms DFT windows. The fine search does
        # another n_phases moves at half this distance. Total
        # displacement is bounded by alternating direction.
        revs_per_move = 0.1
        dist_mm = revs_per_move * rot_dist
        speed_mm_s = speed * rot_dist
        # Track total displacement; abort if we exceed 80mm in either
        # direction from the start position.
        start_pos = self._current_rail_pos(toolhead, stepper)
        if start_pos is None:
            start_pos = toolhead.get_position()[0]
        max_total_disp = 80.0
        total_disp = 0.0
        best_pha = 0.0
        best_resp = float("inf")
        # Coarse sweep
        for i in range(n_phases):
            pha = 2.0 * math.pi * i / n_phases
            # Alternate direction to oscillate around start
            direction = 1 if i % 2 == 0 else -1
            trial_dist = direction * dist_mm
            self._check_displacement(start_pos, start_pos + total_disp + trial_dist,
                                     max_total_disp, gcmd)
            corr = self.ps.get_correction("forward")
            corr.set_harmonic(harmonic, mag, pha)
            self.ps.update_correction()
            aclient = accelerometer.start_internal_client()
            force_move.manual_move(stepper, trial_dist, speed_mm_s, accel=0.0)
            toolhead.wait_moves()
            aclient.finish_measurements()
            samples = aclient.get_samples()
            total_disp += trial_dist
            if not samples:
                continue
            resp = self._dft_at_harmonic(
                samples, speed, motor_steps, harmonic)
            gcmd.respond_info("      pha=%.3f dir=%+d resp=%.4f"
                              % (pha, direction, resp))
            if resp < best_resp:
                best_resp = resp
                best_pha = pha
        # Fine search: ±π/n_phases around the coarse best. Half distance
        # to stay within bounds even if we're near the limit.
        fine_dist = dist_mm * 0.5
        fine_lo = best_pha - math.pi / n_phases
        fine_hi = best_pha + math.pi / n_phases
        for i in range(n_phases):
            pha = fine_lo + (fine_hi - fine_lo) * i / max(1, n_phases - 1)
            direction = 1 if i % 2 == 0 else -1
            trial_dist = direction * fine_dist
            try:
                self._check_displacement(start_pos, start_pos + total_disp + trial_dist,
                                         max_total_disp, gcmd)
            except gcmd.error:
                break
            corr.set_harmonic(harmonic, mag, pha)
            self.ps.update_correction()
            aclient = accelerometer.start_internal_client()
            force_move.manual_move(stepper, trial_dist, speed_mm_s, accel=0.0)
            toolhead.wait_moves()
            aclient.finish_measurements()
            samples = aclient.get_samples()
            total_disp += trial_dist
            if not samples:
                continue
            resp = self._dft_at_harmonic(
                samples, speed, motor_steps, harmonic)
            if resp < best_resp:
                best_resp = resp
                best_pha = pha
        return best_pha, best_resp

    def _current_rail_pos(self, toolhead, stepper):
        """Get the toolhead position projected onto this stepper's rail."""
        try:
            kin = toolhead.get_kinematics()
            for rail in kin.rails:
                for s in rail.get_steppers():
                    if s.get_name() == stepper.get_name():
                        name = rail.get_name()
                        pos = toolhead.get_position()
                        if name == 'stepper_x':
                            return pos[0]
                        elif name == 'stepper_y':
                            return pos[1]
            return None
        except Exception:
            return None

    def _check_displacement(self, start_pos, current_pos, max_disp, gcmd):
        """Raise gcmd.error if the current position would exceed the
        displacement limit from the start position."""
        if abs(current_pos - start_pos) > max_disp:
            raise gcmd.error(
                "Calibration aborted: toolhead would move %.1fmm from"
                " start (limit %.1fmm). Move closer to center and"
                " retry."
                % (current_pos - start_pos, max_disp))

    def _get_rotation_distance(self, stepper):
        try:
            rot_dist, _ = stepper.get_rotation_distance()
            return rot_dist
        except Exception:
            return None

    def _get_stepper_rotation_distance(self, stepper_name):
        try:
            fm = self.printer.lookup_object("force_move")
            stepper = fm.lookup_stepper(stepper_name)
            return self._get_rotation_distance(stepper)
        except Exception:
            return None

    def _dft_at_harmonic(self, samples, speed, motor_steps, harmonic):
        # Compute the magnitude of the sliding-window DFT at the
        # analysis frequency f = speed * motor_steps/2 * harmonic
        # (electrical frequency × harmonic number, in Hz).
        if len(samples) < 2:
            return float("inf")
        dt = samples[1][0] - samples[0][0]
        if dt <= 0:
            return float("inf")
        sample_rate = 1.0 / dt
        analysis_freq = speed * motor_steps / 2.0 * harmonic
        if analysis_freq >= sample_rate / 2.0:
            return float("inf")
        # Use a 100ms sliding window. The window should cover an
        # integer number of motor periods to avoid spectral leakage.
        window_s = self.config.window_size
        window_samples = max(3, int(window_s * sample_rate))
        if window_samples % 2 == 0:
            window_samples += 1
        half = window_samples // 2
        # Pre-multiply samples by sin/cos at the analysis frequency.
        n = len(samples)
        start_time = samples[0][0]
        sin_vals = [0.0] * n
        cos_vals = [0.0] * n
        for i, s in enumerate(samples):
            t = s[0] - start_time
            phase = 2.0 * math.pi * analysis_freq * t
            # Project onto the axis most relevant to this stepper.
            accel = s[1] if self.ps.stepper_name.endswith("_x") else s[2]
            sin_vals[i] = accel * math.sin(phase)
            cos_vals[i] = accel * math.cos(phase)
        # Sliding window sum. Skip the first and last `half` samples
        # (no full window available). For each valid center, compute
        # the integrated sin*sample and cos*sample over the window,
        # then the magnitude. Return the median to be robust to
        # transients at the start/end of the move.
        mags = []
        for center in range(half, n - half):
            sin_sum = sum(sin_vals[center - half:center + half + 1])
            cos_sum = sum(cos_vals[center - half:center + half + 1])
            mags.append(math.sqrt(sin_sum * sin_sum + cos_sum * cos_sum)
                        / window_samples)
        if not mags:
            return float("inf")
        mags.sort()
        return mags[len(mags) // 2]

    def calibrate(self, gcmd):
        toolhead, accel_chip = self._get_toolhead_and_accel(gcmd)
        motor_steps = self.ps.full_steps

        gcmd.respond_info("Phase stepping calibration starting for %s"
                          % self.ps.stepper_name)

        # Ensure all axes are homed for the move
        try:
            toolhead.get_kinematics()._check_homed_axes(
                ['x', 'y', 'z'] if 'z' in toolhead.get_kinematics().get_axes()
                else ['x', 'y'])
        except Exception:
            pass

        # SAFETY: Center the toolhead before any motion. The phase
        # sweep does ~30 small moves; if they all go in the same
        # direction, accumulated displacement can exceed the rail
        # length and crash into an endstop. Centering first gives us
        # ±175mm of headroom on a 350mm X axis.
        cur_pos = toolhead.get_position()
        kin = toolhead.get_kinematics()
        rails = kin.rails
        # Find the rail that owns this stepper so we know which axis
        # to center on.
        stepper_rail = None
        for rail in rails:
            for s in rail.get_steppers():
                if s.get_name() == self.ps.stepper_name:
                    stepper_rail = rail
                    break
            if stepper_rail is not None:
                break
        if stepper_rail is not None:
            rail_name = stepper_rail.get_name()
            r_min, r_max = stepper_rail.get_range()
            center = 0.5 * (r_min + r_max)
            if rail_name == 'stepper_x':
                center_pos = [center, cur_pos[1], cur_pos[2], cur_pos[3]]
            elif rail_name == 'stepper_y':
                center_pos = [cur_pos[0], center, cur_pos[2], cur_pos[3]]
            else:
                center_pos = None
            if center_pos is not None:
                gcmd.respond_info(
                    "  Centering toolhead: moving to %s for %s"
                    % (center_pos, rail_name))
                toolhead.manual_move(center_pos, 6000.0)
                toolhead.wait_moves()

        # Phase 1: Speed sweep. Drive the stepper (not the toolhead)
        # via force_move to avoid crashing into endstops. We do a
        # short constant-velocity move of N revolutions at the
        # configured speed range. Distance is bounded to 100mm
        # (so we don't hit an endstop even from center).
        gcmd.respond_info("Phase 1: Speed sweep...")
        aclient = accel_chip.start_internal_client()

        min_speed, max_speed = self.config.speed_range
        revs = self.config.max_movement_revs
        avg_speed = 0.5 * (min_speed + max_speed)
        rot_dist = self._get_stepper_rotation_distance(
            self.ps.stepper_name)
        if rot_dist is None or rot_dist <= 0:
            raise gcmd.error(
                "Cannot determine rotation_distance for %s"
                % self.ps.stepper_name)
        dist_mm = revs * rot_dist
        speed_mm_s = avg_speed * rot_dist

        # Hard bound: 100mm per single move. Centered toolhead has
        # at least 150mm headroom on a 350mm axis.
        MAX_MOVE_MM = 100.0
        if dist_mm > MAX_MOVE_MM:
            gcmd.respond_info(
                "  WARNING: requested move %.1fmm > safe %dmm,"
                " reducing revs" % (dist_mm, MAX_MOVE_MM))
            revs = MAX_MOVE_MM / rot_dist
            dist_mm = revs * rot_dist

        gcmd.respond_info(
            "  Moving stepper %s by %.1fmm (%.1f revs) at %.1f rev/s"
            " (%.1f mm/s)" % (self.ps.stepper_name, dist_mm, revs,
                              avg_speed, speed_mm_s))

        fm = self.printer.lookup_object("force_move")
        stepper = fm.lookup_stepper(self.ps.stepper_name)
        # Use a small acceleration (50 mm/s^2) to avoid stalling
        # the motor when starting at non-zero speed. accel=0.0
        # caused a stall in earlier tests.
        fm.manual_move(stepper, dist_mm, speed_mm_s, accel=50.0)
        toolhead.wait_moves()

        aclient.finish_measurements()
        samples = aclient.get_samples()

        if samples is None or len(samples) == 0:
            raise gcmd.error("No accelerometer data received")

        gcmd.respond_info("  Got %d accelerometer samples" % len(samples))

        # Estimate sample rate from first two samples
        sample_rate = DEFAULT_ACCEL_SAMPLE_RATE
        if len(samples) >= 2:
            dt = samples[1][0] - samples[0][0]
            if dt > 0:
                sample_rate = 1.0 / dt

        harmonics = self.config.get_enabled_harmonics()

        dft_results = {}
        peaks = {}
        speeds = {}

        for h in harmonics:
            gcmd.respond_info("  DFT for harmonic %d..." % h)
            result = self._chirp_dft_sweep(
                samples, sample_rate, h, min_speed, max_speed, motor_steps)
            if result:
                speeds[h] = [r[0] for r in result]
                vals = [r[1] for r in result]
                dft_results[h] = vals
                peaks[h] = self._find_peaks(vals)

        resonant_speeds = self._harmonic_fit(speeds, peaks)

        gcmd.respond_info("Resonant speeds: %s" % resonant_speeds)

        # Phase 2-4: For each harmonic, find optimal correction
        for h in harmonics:
            speed = resonant_speeds.get(h)
            if speed is None:
                continue

            gcmd.respond_info("Calibrating harmonic %d at %.3f rev/s..."
                              % (h, speed))

            mag, pha = self._find_optimal(
                h, speed, motor_steps, accel_chip, gcmd)

            gcmd.respond_info("  Harmonic %d: best_mag=%.4f best_pha=%.4f"
                              % (h, mag, pha))

            corr_fwd = self.ps.get_correction("forward")
            corr_bwd = self.ps.get_correction("backward")
            corr_fwd.set_harmonic(h, mag, pha)
            corr_bwd.set_harmonic(h, mag, pha)

        self.ps.update_correction()
        gcmd.respond_info("Calibration complete for %s"
                          % self.ps.stepper_name)


def calibrate_phase_stepping(phase_stepping_obj, config=None):
    return CalibrateAxis(phase_stepping_obj, config)
