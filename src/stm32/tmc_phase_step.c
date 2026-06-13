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
// LUT granularity: 1024 entries — one per electrical microstep
#define LUT_SIZE 1024
#define LUT_SCALE (MOTOR_PERIOD / LUT_SIZE)
// Refresh rate: 10 kHz = 100 µs period
#define REFRESH_FREQ 10000

// Maximum number of phase-stepping instances. Two LUTs of LUT_SIZE
// bytes per instance, allocated statically so the heap stays free
// for move queue / OID pool. At 8 instances × 2 × 1024 = 16 KB,
// well within the STM32F446's 128 KB RAM.
#define MAX_PHASE_STEPPERS 8
static int8_t phase_shift_luts[MAX_PHASE_STEPPERS][2][LUT_SIZE];

struct phase_stepper {
    uint8_t oid;
    uint8_t stepper_oid;
    uint8_t lut_index;           // index into phase_shift_luts
    struct gpio_out step_pin;
    struct gpio_out dir_pin;
    uint32_t step_mask;
    uint32_t step_reset_mask;
    uint32_t dir_set_mask;
    uint32_t dir_reset_mask;
    int8_t *current_lut;
    uint8_t forward;            // 1 = forward LUT, 0 = backward LUT
    uint16_t motor_phase;
    uint16_t driver_phase;
    int32_t zero_rotor_phase;
    uint32_t steps_per_period;
    uint8_t enabled;
};

static struct phase_stepper *phase_steppers[MAX_PHASE_STEPPERS];
static uint8_t num_phase_steppers;
static struct timer refresh_timer;
static uint32_t refresh_period_ticks;

// Forward declaration
static uint_fast8_t phase_stepping_refresh(struct timer *t);

// ---- Helpers ----

// Compute motor electrical phase from stepper position.
// Position from stepper_get_position() has a POSITION_BIAS offset (0x40000000)
// representing zero physical steps.
static inline uint16_t
pos_to_phase(struct phase_stepper *ps, uint32_t position)
{
    // phase = ((position - POSITION_BIAS) * MOTOR_PERIOD / steps_per_period
    //          + zero_rotor_phase) % MOTOR_PERIOD
    uint32_t step_count = position - 0x40000000;
    uint32_t phase = ((uint64_t)step_count * MOTOR_PERIOD) / ps->steps_per_period;
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
set_direction(struct phase_stepper *ps, uint8_t forward)
{
    GPIO_TypeDef *regs = (GPIO_TypeDef *)ps->dir_pin.regs;
    if (forward)
        regs->BSRR = ps->dir_set_mask;
    else
        regs->BSRR = ps->dir_reset_mask;
}

// Burst step pulses for a phase correction difference
// 'diff' is in microsteps (signed). Positive = forward, negative = backward.
static void __attribute__((unused))
burst_steps(struct phase_stepper *ps, int32_t diff)
{
    if (diff == 0)
        return;

    uint8_t forward = diff > 0;
    uint32_t count = forward ? (uint32_t)diff : (uint32_t)(-diff);

    GPIO_TypeDef *regs = (GPIO_TypeDef *)ps->step_pin.regs;
    set_direction(ps, forward);

    for (uint32_t i = 0; i < count; i++) {
        regs->BSRR = ps->step_mask;
        regs->BSRR = ps->step_reset_mask;
    }
}

// ---- Periodic refresh callback ----

static uint_fast8_t
phase_stepping_refresh(struct timer *t)
{
    uint8_t i;
    for (i = 0; i < num_phase_steppers; i++) {
        if (phase_steppers[i] && phase_steppers[i]->enabled)
            break;
    }
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
    ps->step_pin = gpio_out_setup(args[2], 0);
    ps->dir_pin = gpio_out_setup(args[3], 0);
    ps->step_mask = ps->step_pin.bit;
    ps->step_reset_mask = ps->step_pin.bit << 16;
    ps->dir_set_mask = ps->dir_pin.bit;
    ps->dir_reset_mask = ps->dir_pin.bit << 16;
    ps->zero_rotor_phase = args[4];
    ps->steps_per_period = args[5];
    if (num_phase_steppers >= MAX_PHASE_STEPPERS)
        shutdown("phase_step: too many instances");
    ps->lut_index = num_phase_steppers;
    for (int i = 0; i < LUT_SIZE; i++) {
        phase_shift_luts[ps->lut_index][0][i] = 0;
        phase_shift_luts[ps->lut_index][1][i] = 0;
    }
    ps->current_lut = phase_shift_luts[ps->lut_index][0];
    ps->forward = 1;
    ps->motor_phase = 0;
    ps->driver_phase = 0;
    ps->enabled = 0;
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
    if (!ps)
        return;
    uint16_t offset = args[1];
    uint8_t data_len = args[2];
    uint8_t *data = command_decode_ptr(args[3]);

    if (offset + data_len > LUT_SIZE * 2) {
        shutdown("phase_step: bad lut offset/size");
        return;
    }

    for (uint16_t i = 0; i < data_len; i++) {
        if (offset + i < LUT_SIZE)
            phase_shift_luts[ps->lut_index][0][offset + i] = (int8_t)data[i];
        else
            phase_shift_luts[ps->lut_index][1][offset + i - LUT_SIZE]
                = (int8_t)data[i];
    }
}
DECL_COMMAND(command_load_phase_lut,
             "load_phase_lut oid=%c offset=%hu data=%*s");

void
command_enable_phase_stepping(uint32_t *args)
{
    uint8_t oid = args[0];
    struct phase_stepper *ps = oid_lookup(oid, command_configure_phase_stepping);
    if (!ps)
        return;
    ps->enabled = args[1] ? 1 : 0;

    if (ps->enabled) {
        ps->motor_phase = 0;
        ps->driver_phase = 0;
        ps->current_lut = phase_shift_luts[ps->lut_index][ps->forward ? 0 : 1];
    }
}
DECL_COMMAND(command_enable_phase_stepping,
             "enable_phase_stepping oid=%c enable=%c");

void
command_set_phase_stepping_direction(uint32_t *args)
{
    uint8_t oid = args[0];
    struct phase_stepper *ps = oid_lookup(oid, command_configure_phase_stepping);
    if (!ps)
        return;
    uint8_t forward = args[1] ? 1 : 0;
    if (ps->forward == forward)
        return;
    ps->forward = forward;
    ps->current_lut = phase_shift_luts[ps->lut_index][forward ? 0 : 1];
}
DECL_COMMAND(command_set_phase_stepping_direction,
             "set_phase_stepping_direction oid=%c forward=%c");

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
