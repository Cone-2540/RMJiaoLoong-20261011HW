#pragma once

#include <stdint.h>

#ifndef MOTOR_MODEL
#define MOTOR_MODEL 2006
#endif

namespace MotorConfig {
constexpr uint32_t motor_id = 1;
constexpr float direction = 1.0f;
constexpr uint32_t feedback_timeout_ms = 20;
constexpr float angle_limit_deg = 150.0f;
constexpr float startup_current_limit_a = 6.0f;
constexpr float speed_limit_deg_s = 600.0f;
constexpr float temperature_limit_c = 70.0f;

#if MOTOR_MODEL == 2006
constexpr float gear_ratio = 36.0f;
constexpr float command_units_per_amp = 1000.0f;
constexpr float maximum_current_a = 10.0f;
constexpr float load_mass_kg = 0.100f;
constexpr float load_radius_m = 0.065f;
constexpr float torque_constant_nm_per_a = 0.18f;
constexpr float initial_ff_peak_a = load_mass_kg * 9.81f * load_radius_m /
                                    torque_constant_nm_per_a;
constexpr bool has_temperature = false;
#elif MOTOR_MODEL == 3508
constexpr float gear_ratio = 3591.0f / 187.0f;
constexpr float command_units_per_amp = 16384.0f / 20.0f;
constexpr float maximum_current_a = 20.0f;
constexpr float load_mass_kg = 0.500f;
constexpr float load_radius_m = 0.05524f;
constexpr float initial_ff_peak_a = 0.0f;
constexpr bool has_temperature = true;
#else
#error MOTOR_MODEL must be 2006 or 3508
#endif

static_assert(motor_id >= 1 && motor_id <= 8, "Invalid motor ID");
static_assert(direction == 1.0f || direction == -1.0f, "Invalid direction");
}
