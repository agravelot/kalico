# Phase stepping support for stepper motors
#
# Compensates for individual stepper motor manufacturing imperfections
# by applying a per-microstep electrical phase correction. Uses the
# MCU-side burst stepping engine (CONFIG_WANT_TMC_PHASE_STEP).
#
# Copyright (C) 2024  Kalico contributors
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, base64, struct

MOTOR_PERIOD = 1024
LUT_SIZE = 1024
SIN_FRACTION = 4
SIN_PERIOD = SIN_FRACTION * MOTOR_PERIOD
MAG_FRACTIONAL = 8


def _mag_to_fixed(m):
    return int(round(m * SIN_PERIOD * (1 << MAG_FRACTIONAL) / (2.0 * math.pi)))


def _pha_to_fixed(p):
    return int(round(p * SIN_PERIOD / (2.0 * math.pi)))


_SIN_LUT = [
    int(round((1 << 15) * math.sin(math.pi / 2.0 * i / 1024.0)))
    for i in range(1024)
]


def _sin_lut(x):
    quo = x // 1024
    rem = x % 1024
    if quo == 0:
        return _SIN_LUT[rem]
    if quo == 1:
        return _SIN_LUT[1024 - rem]
    if quo == 2:
        return -_SIN_LUT[rem]
    return -_SIN_LUT[1024 - rem]


class MotorPhaseCorrection:
    def __init__(self):
        self.items = [(0.0, 0.0)] * 17

    def set_harmonic(self, n, mag, pha):
        self.items[n] = (mag, pha)

    def get_harmonic(self, n):
        return self.items[n]

    def is_empty(self):
        return all(m == 0.0 for m, _ in self.items[1:])

    def build_phase_shift(self):
        lut = [0.0] * LUT_SIZE
        for n in range(1, len(self.items)):
            mag, pha = self.items[n]
            if mag == 0.0:
                continue
            fixed_mag = _mag_to_fixed(mag)
            fixed_pha = _pha_to_fixed(pha)
            for k in range(LUT_SIZE):
                phase = k * (MOTOR_PERIOD // LUT_SIZE)
                arg = (n * phase * SIN_FRACTION + fixed_pha) % SIN_PERIOD
                sval = _sin_lut(arg)
                lut[k] += fixed_mag * sval / (1 << (15 + MAG_FRACTIONAL))
        return [max(-128.0, min(127.0, v)) for v in lut]


class PhaseStepping:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.stepper_name = self.name
        self.stepper = None
        self.tmc_module = None
        self.phase_oid = None
        self.enabled = False
        self.load_lut_cmd = None

        self.full_steps = config.getint("motor_steps", 200, minval=1)
        self.microsteps = config.getint("microsteps", 16, minval=1)
        self.steps_per_period = (self.full_steps * 256) // 50
        self.zero_phase = 0

        self.correction_fwd = MotorPhaseCorrection()
        self.correction_bwd = MotorPhaseCorrection()

        saved = config.get("correction_forward", None)
        if saved is not None:
            self._load_correction(self.correction_fwd, saved)
        saved = config.get("correction_backward", None)
        if saved is not None:
            self._load_correction(self.correction_bwd, saved)

        self.printer.register_event_handler("klippy:mcu_identify",
                                             self._handle_mcu_identify)
        self.printer.register_event_handler("stepper:set_dir_inverted",
                                             self._handle_dir_inverted)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "PHASE_STEPPING_ENABLE", "STEPPER", self.name,
            self.cmd_PHASE_STEPPING_ENABLE,
            desc=self.cmd_PHASE_STEPPING_ENABLE_help)
        gcode.register_mux_command(
            "PHASE_STEPPING_RESET", "STEPPER", self.name,
            self.cmd_PHASE_STEPPING_RESET,
            desc=self.cmd_PHASE_STEPPING_RESET_help)
        gcode.register_mux_command(
            "PHASE_STEPPING_STATUS", "STEPPER", self.name,
            self.cmd_PHASE_STEPPING_STATUS,
            desc=self.cmd_PHASE_STEPPING_STATUS_help)
        gcode.register_mux_command(
            "PHASE_STEPPING_CALIBRATE", "STEPPER", self.name,
            self.cmd_PHASE_STEPPING_CALIBRATE,
            desc=self.cmd_PHASE_STEPPING_CALIBRATE_help)

    def _load_correction(self, corr, data):
        try:
            raw = base64.b64decode(data)
            for n in range(1, 17):
                off = (n - 1) * 8
                if off + 8 > len(raw):
                    break
                mag, pha = struct.unpack("ff", raw[off:off + 8])
                corr.set_harmonic(n, mag, pha)
        except Exception:
            logging.warning("phase_stepping: failed to load correction for %s",
                            self.name)

    def _serialize_correction(self, corr):
        raw = b""
        for n in range(1, 17):
            mag, pha = corr.get_harmonic(n)
            raw += struct.pack("ff", mag, pha)
        return base64.b64encode(raw).decode()

    def _handle_mcu_identify(self):
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        try:
            self.tmc_module = self.printer.lookup_object(
                "tmc5160 " + self.stepper_name)
        except self.printer.config_error:
            raise self.printer.config_error(
                "phase_stepping requires a TMC5160 driver "
                "for stepper '%s'" % (self.stepper_name,))
        # Register deferred config callback so the
        # configure_phase_stepping command is sent with the rest
        # of the MCU's config batch (mcu._send_config runs in
        # klippy:connect which may be ordered after this module's
        # klippy:connect handler).
        self.stepper.get_mcu().register_config_callback(
            self._build_phase_stepping_config)

    def _build_phase_stepping_config(self):
        mcu = self.stepper.get_mcu()
        self.phase_oid = mcu.create_oid()
        step_pin = self.stepper.get_step_pin()
        dir_pin = self.stepper.get_dir_pin()

        logging.info("phase_stepping %s: phase_oid=%d stepper_oid=%d",
                     self.stepper_name, self.phase_oid,
                     self.stepper.get_oid())

        mcu.add_config_cmd(
            "configure_phase_stepping oid=%d stepper_oid=%d"
            " step_pin=%s dir_pin=%s zero_phase=%i steps_per_period=%u"
            % (self.phase_oid, self.stepper.get_oid(),
               step_pin, dir_pin, 0, self.steps_per_period))

        self._lut_cq = mcu.alloc_command_queue()
        self._enable_cq = mcu.alloc_command_queue()
        self._build_phase_cmds()
        self._send_lut_init()

    def _build_phase_cmds(self):
        mcu = self.stepper.get_mcu()
        self.load_lut_cmd = mcu.lookup_command(
            "load_phase_lut oid=%c offset=%hu data=%*s", cq=self._lut_cq)
        self.enable_cmd = mcu.lookup_command(
            "enable_phase_stepping oid=%c enable=%c", cq=self._enable_cq)
        self.direction_cmd = mcu.lookup_command(
            "set_phase_stepping_direction oid=%c forward=%c", cq=self._enable_cq)

    def _send_lut_init(self):
        pass

    def _send_lut(self):
        if self.load_lut_cmd is None:
            return
        fwd = self.correction_fwd.build_phase_shift()
        bwd = self.correction_bwd.build_phase_shift()
        # Concatenate forward + backward LUTs into a single buffer.
        # MCU splits at offset=LUT_SIZE (command_load_phase_lut
        # treats the first LUT_SIZE bytes as forward, the rest as
        # backward).
        data = bytearray(LUT_SIZE * 2)
        for i in range(LUT_SIZE):
            data[i] = int(fwd[i]) & 0xFF
            data[i + LUT_SIZE] = int(bwd[i]) & 0xFF
        # PT_buffer max length is 59 bytes (MESSAGE_PAYLOAD_MAX in
        # klippy/chelper/msgblock.h). Larger chunks overflow the
        # queue_message.msg[64] buffer in serialqueue_send and corrupt
        # the heap, disconnecting the MCU. 50 bytes leaves headroom
        # for msgid(1) + oid(1) + offset(2) + len(1) = 5 byte header
        # before the 64-byte total cap.
        chunk = 50
        for off in range(0, len(data), chunk):
            self.load_lut_cmd.send(
                [self.phase_oid, off, list(data[off:off + chunk])])

    def _handle_dir_inverted(self, stepper):
        if stepper is not self.stepper or self.direction_cmd is None:
            return
        # get_dir_inverted returns (current, original). We use the
        # current setting: if inverted, kinematics-level forward
        # commands map to motor-level reverse, so the backward LUT
        # captures the correct phase correction.
        invert, _ = self.stepper.get_dir_inverted()
        self.direction_cmd.send([self.phase_oid, 0 if invert else 1])

    def _sync_phase_offset(self):
        if self.tmc_module is None or self.stepper is None:
            return
        try:
            mscnt = self.tmc_module.query_phase()
        except self.printer.command_error:
            logging.info("phase_stepping: cannot read MSCNT for %s",
                         self.stepper_name)
            return
        self.zero_phase = int(mscnt)

    def enable(self, enable):
        if enable == self.enabled:
            return
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.wait_moves()
        print_time = toolhead.get_last_move_time()

        if enable:
            self._enable(print_time)
        else:
            self._disable(print_time)

    def _enable(self, print_time):
        if not self.enable_cmd:
            return
        # 1. Set TMC into phase-stepping mode (saves _saved_mres, etc.
        #    in tmc.py so disable can restore).
        if self.tmc_module is not None:
            self.tmc_module.set_phase_stepping_mode(print_time)
        # 2. Sync MCU's zero_phase with the live TMC MSCNT so the first
        #    ISR tick computes correction from the right baseline.
        self._sync_phase_offset()
        # 3. Push the current correction LUT to the MCU.
        self._send_lut()
        # 4. Push current stepper dir inversion to the MCU so the right
        #    LUT (forward vs backward) is selected from the first tick.
        if self.direction_cmd is not None:
            invert, _ = self.stepper.get_dir_inverted()
            self.direction_cmd.send([self.phase_oid, 0 if invert else 1])
        # 5. Arm the ISR.
        self.enable_cmd.send([self.phase_oid, 1])

        # 6. Adjust the kinematic rotation distance so the existing
        # full-step move count maps to 256 microsteps (or whatever
        # the TMC was configured for).
        rot_dist, steps_per_rot = self.stepper.get_rotation_distance()
        new_steps = self.full_steps * 256
        self._saved_steps_per_rot = steps_per_rot
        self.stepper.set_rotation_distance(
            rot_dist * new_steps / steps_per_rot)
        self.enabled = True
        logging.info("phase_stepping: enabled for %s", self.stepper_name)

    def _enable_stripped(self, print_time):
        # Stripped-down enable used while debugging the multi-chunk
        # LUT load regression. Skips TMC mode set, MSCNT sync, LUT
        # upload, and dwell — only the direction + enable commands
        # are sent. Not for production use; see
        # docs/phase_stepping_plan.md "Multi-chunk LUT load: open
        # issue" for the test matrix.
        if not self.enable_cmd:
            return
        if self.direction_cmd is not None:
            invert, _ = self.stepper.get_dir_inverted()
            self.direction_cmd.send([self.phase_oid, 0 if invert else 1])
        self.enable_cmd.send([self.phase_oid, 1])

        rot_dist, steps_per_rot = self.stepper.get_rotation_distance()
        new_steps = self.full_steps * 256
        self._saved_steps_per_rot = steps_per_rot
        self.stepper.set_rotation_distance(
            rot_dist * new_steps / steps_per_rot)
        self.enabled = True
        logging.info("phase_stepping: stripped enable for %s",
                     self.stepper_name)

    def _disable(self, print_time):
        if self.phase_oid is None:
            return
        if self.enable_cmd:
            self.enable_cmd.send([self.phase_oid, 0])
        if self.tmc_module is not None:
            self.tmc_module.restore_phase_stepping_mode(print_time)
        if hasattr(self, "_saved_steps_per_rot"):
            rot_dist, _ = self.stepper.get_rotation_distance()
            self.stepper.set_rotation_distance(
                rot_dist * self._saved_steps_per_rot
                / (self.full_steps * 256))
        self.enabled = False
        logging.info("phase_stepping: disabled for %s", self.stepper_name)

    def update_correction(self):
        if not self.enabled:
            return
        self._send_lut()

    def get_correction(self, direction="forward"):
        return self.correction_fwd if direction == "forward" else self.correction_bwd

    def get_status(self, eventtime=None):
        items = {}
        for n in range(1, 17):
            mag, pha = self.correction_fwd.get_harmonic(n)
            if mag != 0.0:
                items["harmonic_%d" % n] = (mag, pha)
        return {
            "enabled": self.enabled,
            "correction_items": items,
        }

    cmd_PHASE_STEPPING_ENABLE_help = "Enable/disable phase stepping"

    def cmd_PHASE_STEPPING_ENABLE(self, gcmd):
        self.enable(bool(gcmd.get_int("ENABLE", 1)))

    cmd_PHASE_STEPPING_RESET_help = "Reset phase stepping corrections"

    def cmd_PHASE_STEPPING_RESET(self, gcmd):
        self.correction_fwd = MotorPhaseCorrection()
        self.correction_bwd = MotorPhaseCorrection()
        self._send_lut()
        gcmd.respond_info("Phase stepping corrections reset for %s"
                          % self.stepper_name)

    cmd_PHASE_STEPPING_STATUS_help = "Show phase stepping status and corrections"

    def cmd_PHASE_STEPPING_STATUS(self, gcmd):
        status = self.get_status()
        lines = ["Phase Stepping %s: enabled=%s" % (self.stepper_name,
                                                     status["enabled"])]
        for k, (mag, pha) in status["correction_items"].items():
            lines.append("  %s: mag=%.4f pha=%.4f" % (k, mag, pha))
        gcmd.respond_info("\n".join(lines))

    cmd_PHASE_STEPPING_CALIBRATE_help = "Run phase stepping calibration"

    def cmd_PHASE_STEPPING_CALIBRATE(self, gcmd):
        from . import phase_stepping_calibration
        cal = phase_stepping_calibration.CalibrateAxis(self)
        cal.calibrate(gcmd)


def load_config_prefix(config):
    return PhaseStepping(config)
