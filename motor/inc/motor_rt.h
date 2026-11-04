/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_RT_H
#define MOTOR_RT_H
#if defined(__riscv)
#include "hpm_soc.h"
#define MOTOR_RAMFUNC ATTR_RAMFUNC
#else
#define MOTOR_RAMFUNC
#endif
#if defined(__GNUC__)
#define MOTOR_INLINE static inline __attribute__((always_inline))
#else
#define MOTOR_INLINE static inline
#endif
#endif
