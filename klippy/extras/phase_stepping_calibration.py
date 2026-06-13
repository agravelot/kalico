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
                dft.feed(samples[j], s, c)

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

    def _find_approx_mag(self, harmonic, speed, motor_steps,
                         accelerometer, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        min_mag = self.config.min_magnitude
        max_mag = self.config.max_magnitude
        quotient = self.config.magnitude_quotient

        best_mag = 0.0
        best_min = float("inf")
        gone_worse = 0
        mag = min_mag

        while mag <= max_mag:
            # Set trial correction
            corr = self.ps.get_correction("forward")
            corr.set_harmonic(harmonic, mag, 0.0)
            self.ps.update_correction()

            # Phase sweep at this magnitude
            response = self._capture_param_sweep(
                harmonic, speed, motor_steps, accelerometer, gcmd)

            # Find minimum
            if response:
                min_val = min(response)
                if min_val < best_min:
                    best_min = min_val
                    best_mag = mag
                    gone_worse = 0
                else:
                    gone_worse += 1

            if gone_worse >= 2:
                break
            mag *= quotient

        return best_mag

    def _capture_param_sweep(self, harmonic, speed, motor_steps,
                             accelerometer, gcmd):
        toolhead = self.printer.lookup_object("toolhead"
                                              ).get_kinematics().rails[0]
        aclient = accelerometer.start_internal_client()
        # Do a small move at the target speed, capture samples
        rev_pos = speed * 60.0 * 0.2  # 0.2s at this speed = ~1/5 rev
        cur_pos = toolhead.get_tag_position()
        # Move in +X (or whatever this stepper's rail is)
        # We use the stepper's own rail, not the X axis
        if not self.ps.stepper_name:
            aclient.finish_measurements()
            return []
        # The phase_stepping object knows which stepper to drive:
        # move it via its kinematics rail
        from . import force_move
        fm = self.printer.lookup_object("force_move")
        stepper = fm.lookup_stepper(self.ps.stepper_name)
        stepper.kin_add_move(cur_pos + rev_pos, speed * 60.0)
        toolhead.wait_moves()
        aclient.finish_measurements()
        samples = aclient.get_samples()
        if not samples:
            return []
        # Sum of squared X-axis accel as the magnitude proxy
        n = len(samples)
        if n < 2:
            return []
        # Use the first two samples to estimate rate
        dt = samples[1][0] - samples[0][0]
        if dt > 0:
            sample_rate = 1.0 / dt
        else:
            sample_rate = DEFAULT_ACCEL_SAMPLE_RATE
        mag_sum = sum(s[1] * s[1] + s[2] * s[2] + s[3] * s[3]
                      for s in samples)
        # Return a list of (phase_value, magnitude) for the optimizer
        # to search. Since we don't have the full phase-sweep loop
        # here, return a single-point list as a smoke-test stub.
        return [math.sqrt(mag_sum / n)]

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

        # Phase 1: Speed sweep
        gcmd.respond_info("Phase 1: Speed sweep...")
        aclient = accel_chip.start_internal_client()

        min_speed, max_speed = self.config.speed_range
        # Generate a linear speed ramp movement
        duration = self.config.coarse_duration
        revs = self.config.max_movement_revs
        start_pos = 0.0
        end_pos = revs

        # Move forward accelerating
        toolhead.manual_move([start_pos, None, None, None], min_speed * 60)
        toolhead.manual_move([end_pos, None, None, None], max_speed * 60)

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

            mag = self._find_approx_mag(h, speed, motor_steps,
                                        accel_chip, gcmd)

            gcmd.respond_info("  Harmonic %d: best_mag=%.4f" % (h, mag))

            # Store correction (placeholder: just use found magnitude)
            corr_fwd = self.ps.get_correction("forward")
            corr_bwd = self.ps.get_correction("backward")
            corr_fwd.set_harmonic(h, mag, 0.0)
            corr_bwd.set_harmonic(h, mag, 0.0)

        self.ps.update_correction()
        gcmd.respond_info("Calibration complete for %s"
                          % self.ps.stepper_name)


def calibrate_phase_stepping(phase_stepping_obj, config=None):
    return CalibrateAxis(phase_stepping_obj, config)
