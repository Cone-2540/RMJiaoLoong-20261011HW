#ifndef PID_APP_H
#define PID_APP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_max;
    float d_filter_k;
} PidParameters;

typedef struct {
    uint32_t time_ms;
    float target_deg;
    float actual_deg;
    float error_deg;
    float speed_deg_s;
    float current_a;
    float feedforward_a;
} TraceSample;

enum {
    CONTROL_CURRENT = 1,
    CONTROL_FEEDFORWARD = 2,
    CONTROL_SPEED = 3,
    CONTROL_POSITION = 4,
    TRACE_CAPACITY = 500
};

enum {
    FAULT_FEEDBACK = 1,
    FAULT_ZERO = 2,
    FAULT_ANGLE = 4,
    FAULT_TEMPERATURE = 8,
    FAULT_PARAMETER = 16,
    FAULT_CAN = 32
};

extern volatile uint32_t stop_flag;
extern volatile uint32_t control_mode;
extern volatile uint32_t zero_request;
extern volatile uint32_t zero_valid;
extern volatile uint32_t pid_reset_request;
extern volatile uint32_t fault_flags;
extern volatile uint32_t feedback_online;
extern volatile uint32_t feedback_age_ms;
extern volatile uint32_t rx_count;
extern volatile uint32_t rx_invalid_count;
extern volatile uint32_t tx_count;
extern volatile uint32_t tx_error_count;
extern volatile uint32_t can_error;
extern volatile uint32_t control_tick_count;
extern volatile uint32_t control_cycles;
extern volatile uint32_t control_cycles_max;
extern volatile float target_angle_deg;
extern volatile float target_speed_deg_s;
extern volatile float manual_current_a;
extern volatile float current_limit_a;
extern volatile float speed_limit_deg_s;
extern volatile float ff_peak_current_a;
extern volatile float ff_scale;
extern volatile uint32_t ff_enable;
extern volatile PidParameters position_pid_params;
extern volatile PidParameters speed_pid_params;
extern volatile float actual_angle_deg;
extern volatile float actual_speed_deg_s;
extern volatile float angle_error_deg;
extern volatile float speed_reference_deg_s;
extern volatile float ff_current_a;
extern volatile float pid_current_a;
extern volatile float command_current_a;
extern volatile int32_t command_current_raw;
extern volatile int32_t feedback_current_raw;
extern volatile uint32_t encoder_raw;
extern volatile uint32_t motor_temperature_c;
extern volatile float gravity_torque_nm;
extern const uint32_t configured_motor_model;
extern const float maximum_gravity_torque_nm;
extern volatile uint32_t step_request;
extern volatile float step_delta_deg;
extern volatile uint32_t trace_state;
extern volatile uint32_t trace_count;
extern volatile uint32_t settling_time_ms;
extern volatile float step_overshoot_deg;
extern volatile TraceSample trace_samples[TRACE_CAPACITY];

void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif
