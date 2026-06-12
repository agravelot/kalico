#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);
uint32_t stepper_get_position_by_oid(uint8_t oid);

#endif // stepper.h
