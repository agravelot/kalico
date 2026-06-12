// Phase stepping burst engine for TMC stepper drivers
//
// Injects microstep correction pulses on top of normal step generation.
// A 10 kHz timer ISR reads stepper position, computes motor electrical
// phase, applies correction from a pre-loaded LUT, and bursts correction
// step pulses via GPIO BSRR writes.
//
// Copyright (C) 2024  Kalico contributors
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"
#include "basecmd.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "internal.h"
#include "sched.h"
#include "stepper.h"

#if CONFIG_WANT_TMC_PHASE_STEP

DECL_CONSTANT("PHASE_STEPPING", 1);

// Electrical period of Trinamic drivers (MSCNT range)
#define MOTOR_PERIOD 1024
// Number of electrical periods per full mechanical revolution
// (50 pole pairs = 50 electrical periods per rev for a standard 200-step motor)
// This is configurable per-stepper via steps_per_period
#define SIN_FRACTION 4
#define SIN_PERIOD (SIN_FRACTION * MOTOR_PERIOD)  // 4096

// Refresh rate: 10 kHz = 100 µs period
#define REFRESH_FREQ 10000
// Maximum burst events per refresh cycle
#define GPIO_BUFFER_SIZE 200

struct phase_stepper {
    struct timer timer;
    uint8_t oid;
    uint8_t stepper_oid;
    // GPIO pins (same as the associated stepper)
    struct gpio_out step_pin;
    struct gpio_out dir_pin;
    uint32_t step_mask;       // BSRR bit for step SET
    uint32_t step_reset_mask; // BSRR bit for step RESET
    uint32_t dir_set_mask;    // BSRR bit for dir SET
    uint32_t dir_reset_mask;  // BSRR bit for dir RESET
    // Correction LUT: one int8_t per electrical microstep
    int8_t phase_shift_lut[MOTOR_PERIOD];
    int8_t phase_shift_lut_bwd[MOTOR_PERIOD];
    int8_t *current_lut;
    // Phase tracking
    uint32_t last_position;
    uint16_t motor_phase;     // Current electrical phase (0..1023)
    uint16_t driver_phase;    // TMC driver phase estimate (0..1023)
    int32_t zero_rotor_phase; // Phase offset at logical position 0
    uint32_t steps_per_period;// Motor steps per electrical period
    uint32_t scale_factor;    // Precomputed: POSITION_BIAS * MOTOR_PERIOD / steps_per_period
    // Burst state
    uint32_t event_buffer[GPIO_BUFFER_SIZE];
    uint8_t max_event;
    bool axis_forward;        // Current step direction
    bool enabled;
    bool pending_lut;
};

static struct phase_stepper *phase_steppers[8];
static uint8_t num_phase_steppers;
static struct timer refresh_timer;
static uint32_t refresh_period_ticks;

#define MAX_PHASE_STEPPERS 8

// Forward declaration
static uint_fast8_t phase_stepping_refresh(struct timer *t);

// ---- Helpers ----

// Compute motor electrical phase from stepper position
static inline uint16_t
pos_to_phase(struct phase_stepper *ps, uint32_t position)
{
    // phase = ((position + POSITION_BIAS) * MOTOR_PERIOD / steps_per_period
    //          + zero_rotor_phase) % MOTOR_PERIOD
    uint32_t adjusted = position + 0x40000000; // POSITION_BIAS
    uint32_t phase = ((uint64_t)adjusted * MOTOR_PERIOD) / ps->steps_per_period;
    phase = (phase + ps->zero_rotor_phase) % MOTOR_PERIOD;
    return (uint16_t)phase;
}

// Compute phase difference, wrapped to [-MOTOR_PERIOD/2, MOTOR_PERIOD/2)
static inline int32_t
phase_diff(uint16_t target, uint16_t current)
{
    int32_t diff = (int32_t)target - (int32_t)current;
    if (diff > MOTOR_PERIOD / 2)
        diff -= MOTOR_PERIOD;
    else if (diff < -MOTOR_PERIOD / 2)
        diff += MOTOR_PERIOD;
    return diff;
}

// Write direction pin for an axis
static inline void
set_direction(struct phase_stepper *ps, bool forward)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)ps->dir_pin.regs;
    if (forward)
        regs->BSRR = ps->dir_set_mask;
    else
        regs->BSRR = ps->dir_reset_mask;
}

// Burst step pulses for a phase correction difference
// 'diff' is in microsteps (signed). Positive = forward, negative = backward.
static void
burst_steps(struct phase_stepper *ps, int32_t diff)
{
    if (diff == 0)
        return;

    bool forward;
    if (diff > 0) {
        forward = true;
    } else {
        forward = false;
        diff = -diff;
    }

    set_direction(ps, forward);

    // Evenly space step toggles in the GPIO buffer
    // Each toggle = step_pin HIGH then LOW = 2 events per microstep
    uint32_t toggles = (uint32_t)diff * 2;
    if (toggles > GPIO_BUFFER_SIZE)
        toggles = GPIO_BUFFER_SIZE;

    // Fixed-point spacing: (GPIO_BUFFER_SIZE << 16) / toggles
    uint32_t spacing = ((uint32_t)GPIO_BUFFER_SIZE << 16) / toggles;
    uint32_t frac = 0;
    uint8_t idx = 0;
    bool high = true;
    GPIO_TypeDef *regs = (GPIO_TypeDef *)ps->step_pin.regs;

    for (uint32_t i = 0; i < toggles && idx < GPIO_BUFFER_SIZE; i++) {
        if (high)
            regs->BSRR = ps->step_mask;
        else
            regs->BSRR = ps->step_reset_mask;
        high = !high;

        frac += spacing;
        idx = (uint8_t)(frac >> 16);
    }
}

// ---- Periodic refresh callback (10 kHz) ----

static uint_fast8_t
phase_stepping_refresh(struct timer *t)
{
    for (uint8_t i = 0; i < num_phase_steppers; i++) {
        struct phase_stepper *ps = phase_steppers[i];
        if (!ps->enabled)
            continue;

        // Apply pending LUT swap
        if (ps->pending_lut) {
            ps->current_lut = (ps->current_lut == ps->phase_shift_lut)
                ? ps->phase_shift_lut_bwd : ps->phase_shift_lut;
            ps->pending_lut = false;
        }

        // Read current stepper position
        uint32_t position = stepper_get_position_by_oid(ps->stepper_oid);

        // Compute motor electrical phase
        uint16_t motor_phase = pos_to_phase(ps, position);
        ps->motor_phase = motor_phase;

        // Lookup correction from LUT
        int8_t correction = ps->current_lut[motor_phase];

        // Compute target (corrected) phase
        uint16_t target_phase = (motor_phase + correction + MOTOR_PERIOD) % MOTOR_PERIOD;

        // Compute phase difference from driver estimate
        int32_t diff = phase_diff(target_phase, ps->driver_phase);

        // Burst steps to correct
        if (diff != 0) {
            burst_steps(ps, diff);
            ps->driver_phase = target_phase;
        }
    }

    // Reschedule for next refresh
    t->waketime += refresh_period_ticks;
    return SF_RESCHEDULE;
}

// ---- MCU Commands ----

void
command_configure_phase_stepping(uint32_t *args)
{
    uint8_t oid = args[0];
    struct phase_stepper *ps = oid_alloc(oid, command_configure_phase_stepping,
                                          sizeof(*ps));
    ps->oid = oid;
    ps->stepper_oid = args[1];

    // Set up GPIO pins (same pins as the stepper)
    ps->step_pin = gpio_out_setup(args[2], 0);
    ps->dir_pin = gpio_out_setup(args[3], 0);

    // BSRR masks for atomic GPIO writes
    ps->step_mask = ps->step_pin.bit;
    ps->step_reset_mask = ps->step_pin.bit << 16;
    ps->dir_set_mask = ps->dir_pin.bit;
    ps->dir_reset_mask = ps->dir_pin.bit << 16;

    // Params
    ps->zero_rotor_phase = args[4];
    ps->steps_per_period = args[5];

    // Default to empty LUT (no correction)
    for (int i = 0; i < MOTOR_PERIOD; i++) {
        ps->phase_shift_lut[i] = 0;
        ps->phase_shift_lut_bwd[i] = 0;
    }
    ps->current_lut = ps->phase_shift_lut;
    ps->pending_lut = false;

    ps->motor_phase = 0;
    ps->driver_phase = 0;
    ps->last_position = 0;
    ps->enabled = false;

    // Register in global list
    if (num_phase_steppers < MAX_PHASE_STEPPERS)
        phase_steppers[num_phase_steppers++] = ps;
}
DECL_COMMAND(command_configure_phase_stepping,
             "configure_phase_stepping oid=%c stepper_oid=%c"
             " step_pin=%u dir_pin=%u zero_phase=%i steps_per_period=%u");

void
command_load_phase_lut(uint32_t *args)
{
    uint8_t oid = args[0];
    struct phase_stepper *ps = oid_lookup(oid, command_configure_phase_stepping);
    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);

    uint16_t expected = MOTOR_PERIOD * 2;
    if (data_len != expected) {
        shutdown("phase_step: bad lut size");
        return;
    }

    for (uint16_t i = 0; i < MOTOR_PERIOD; i++) {
        ps->phase_shift_lut[i] = (int8_t)data[i];
        ps->phase_shift_lut_bwd[i] = (int8_t)data[i + MOTOR_PERIOD];
    }
}
DECL_COMMAND(command_load_phase_lut,
             "load_phase_lut oid=%c data=%*s");

void
command_enable_phase_stepping(uint32_t *args)
{
    uint8_t oid = args[0];
    struct phase_stepper *ps = oid_lookup(oid, command_configure_phase_stepping);
    ps->enabled = args[1] ? true : false;

    if (ps->enabled) {
        // Reset phase tracking on enable
        uint32_t position = stepper_get_position_by_oid(ps->stepper_oid);
        ps->motor_phase = pos_to_phase(ps, position);
        ps->driver_phase = ps->motor_phase;
        ps->current_lut = ps->phase_shift_lut;
    }
}
DECL_COMMAND(command_enable_phase_stepping,
             "enable_phase_stepping oid=%c enable=%c");

// ---- Init ----

void
phase_stepping_init(void)
{
    refresh_period_ticks = timer_from_us(1000000 / REFRESH_FREQ);
    refresh_timer.func = phase_stepping_refresh;
    refresh_timer.waketime = timer_read_time() + refresh_period_ticks;
    sched_add_timer(&refresh_timer);
}
DECL_INIT(phase_stepping_init);

#endif // CONFIG_WANT_TMC_PHASE_STEP
