#include "app.h"
#include "PID.hpp"
#include "Motor.hpp"
#include "can.h"
#include "tim.h"
#include <math.h>

extern "C" {
volatile uint32_t stop_flag = 1;
volatile uint32_t control_mode = CONTROL_POSITION;
volatile uint32_t zero_request = 0;
volatile uint32_t zero_valid = 0;
volatile uint32_t pid_reset_request = 0;
volatile uint32_t fault_flags = 0;
volatile uint32_t feedback_online = 0;
volatile uint32_t feedback_age_ms = UINT32_MAX;
volatile uint32_t rx_count = 0;
volatile uint32_t rx_invalid_count = 0;
volatile uint32_t tx_count = 0;
volatile uint32_t tx_error_count = 0;
volatile uint32_t can_error = 0;
volatile uint32_t control_tick_count = 0;
volatile uint32_t control_cycles = 0;
volatile uint32_t control_cycles_max = 0;
volatile float target_angle_deg = 0.0f;
volatile float target_speed_deg_s = 0.0f;
volatile float manual_current_a = 0.0f;
volatile float current_limit_a = MotorConfig::startup_current_limit_a;
volatile float speed_limit_deg_s = MotorConfig::speed_limit_deg_s;
volatile float ff_peak_current_a = MotorConfig::initial_ff_peak_a;
volatile float ff_scale = 1.0f;
volatile uint32_t ff_enable = 1;
volatile PidParameters position_pid_params = {4.0f, 0.0f, 0.0f, 1000.0f, 0.8f};
volatile PidParameters speed_pid_params = {0.02f, 0.0f, 0.0f, 1000.0f, 0.8f};
volatile float actual_angle_deg = 0.0f;
volatile float actual_speed_deg_s = 0.0f;
volatile float angle_error_deg = 0.0f;
volatile float speed_reference_deg_s = 0.0f;
volatile float ff_current_a = 0.0f;
volatile float pid_current_a = 0.0f;
volatile float command_current_a = 0.0f;
volatile int32_t command_current_raw = 0;
volatile int32_t feedback_current_raw = 0;
volatile uint32_t encoder_raw = 0;
volatile uint32_t motor_temperature_c = 0;
volatile float gravity_torque_nm = 0.0f;
extern const uint32_t configured_motor_model = MOTOR_MODEL;
extern const float maximum_gravity_torque_nm =
    MotorConfig::load_mass_kg * 9.81f * MotorConfig::load_radius_m;
volatile uint32_t step_request = 0;
volatile float step_delta_deg = 10.0f;
volatile uint32_t trace_state = 0;
volatile uint32_t trace_count = 0;
volatile uint32_t settling_time_ms = UINT32_MAX;
volatile float step_overshoot_deg = 0.0f;
volatile TraceSample trace_samples[TRACE_CAPACITY] = {};
}

namespace {
Motor motor;
PID position_pid(0, 0, 0, 0, 0, 0);
PID speed_pid(0, 0, 0, 0, 0, 0);
volatile uint32_t last_feedback_tick = 0;
volatile bool received_feedback = false;
volatile bool feedback_restarted = false;
float zero_offset_deg = 0.0f;
uint32_t previous_mode = 0;
bool was_running = false;
float trace_target = 0.0f;
float trace_direction = 1.0f;
uint32_t last_outside_ms = UINT32_MAX;
uint32_t trace_start_cycles = 0;

float clamp(float value, float limit) {
    return value < -limit ? -limit : (value > limit ? limit : value);
}

bool validParameters(const volatile PidParameters &params) {
    return isfinite(params.kp) && params.kp >= 0 &&
           isfinite(params.ki) && params.ki >= 0 &&
           isfinite(params.kd) && params.kd >= 0 &&
           isfinite(params.i_max) && params.i_max >= 0 &&
           isfinite(params.d_filter_k) && params.d_filter_k >= 0 &&
           params.d_filter_k < 1;
}

void configurePid(PID &pid, const volatile PidParameters &params, float limit) {
    pid.setParameters(params.kp, params.ki, params.kd, params.i_max,
                      limit, params.d_filter_k);
}

void resetPid() {
    position_pid.reset();
    speed_pid.reset();
}

void sendCurrent(float current) {
    const uint32_t mailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
    if (HAL_CAN_IsTxMessagePending(&hcan1, mailboxes)) {
        HAL_CAN_AbortTxRequest(&hcan1, mailboxes);
    }
    if (stop_flag != 0 || fault_flags != 0) current = 0.0f;
    command_current_a = current;
    int16_t raw = static_cast<int16_t>(current * MotorConfig::direction *
                                      MotorConfig::command_units_per_amp);
    command_current_raw = raw;
    uint16_t bits = static_cast<uint16_t>(raw);
    uint8_t data[8] = {0};
    uint32_t slot = (MotorConfig::motor_id - 1) % 4 * 2;
    data[slot] = static_cast<uint8_t>(bits >> 8);
    data[slot + 1] = static_cast<uint8_t>(bits);
    CAN_TxHeaderTypeDef header = {};
    header.StdId = MotorConfig::motor_id <= 4 ? 0x200 : 0x1FF;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;
    uint32_t mailbox = 0;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK) {
        ++tx_count;
    } else {
        ++tx_error_count;
        stop_flag = 1;
        fault_flags |= FAULT_CAN;
        command_current_a = 0;
        command_current_raw = 0;
        HAL_CAN_AbortTxRequest(&hcan1, mailboxes);
    }
}

void recordStep(bool running) {
    if (trace_state != 1) return;
    if (!running || target_angle_deg != trace_target ||
        control_mode != CONTROL_POSITION || stop_flag != 0) {
        trace_state = 3;
        settling_time_ms = UINT32_MAX;
        return;
    }
    uint32_t index = trace_count;
    trace_samples[index].time_ms = (DWT->CYCCNT - trace_start_cycles) /
                                  (SystemCoreClock / 1000);
    trace_samples[index].target_deg = target_angle_deg;
    trace_samples[index].actual_deg = actual_angle_deg;
    trace_samples[index].error_deg = angle_error_deg;
    trace_samples[index].speed_deg_s = actual_speed_deg_s;
    trace_samples[index].current_a = command_current_a;
    trace_samples[index].feedforward_a = ff_current_a;
    if (fabsf(angle_error_deg) > 0.1f) last_outside_ms = index;
    float overshoot = trace_direction * (actual_angle_deg - trace_target);
    if (overshoot > step_overshoot_deg) step_overshoot_deg = overshoot;
    trace_count = index + 1;
    if (trace_count == TRACE_CAPACITY) {
        trace_state = 2;
        settling_time_ms = UINT32_MAX;
        if (last_outside_ms == UINT32_MAX) {
            settling_time_ms = 0;
        } else if (last_outside_ms + 1 < trace_count) {
            settling_time_ms = trace_samples[last_outside_ms + 1].time_ms;
        }
    }
}

void controlStep() {
    ++control_tick_count;
    uint32_t now = HAL_GetTick();
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();
    bool received = received_feedback;
    uint32_t feedback_tick = last_feedback_tick;
    bool restarted = feedback_restarted;
    feedback_restarted = false;
    float motor_angle = motor.angle();
    actual_speed_deg_s = motor.speed();
    encoder_raw = motor.encoder();
    feedback_current_raw = motor.currentRaw();
    motor_temperature_c = motor.temperature();
    __set_PRIMASK(interrupt_state);
    feedback_age_ms = received ? now - feedback_tick : UINT32_MAX;
    feedback_online = received && feedback_age_ms < MotorConfig::feedback_timeout_ms;
    if (!feedback_online || restarted) {
        zero_valid = 0;
        stop_flag = 1;
    }
    if (zero_request != 0) {
        if (stop_flag != 0 && feedback_online != 0 &&
            fabsf(actual_speed_deg_s) < 5.0f) {
            zero_offset_deg = motor_angle;
            zero_valid = 1;
            target_angle_deg = 0;
            target_speed_deg_s = 0;
            resetPid();
        }
        zero_request = 0;
    }
    actual_angle_deg = motor_angle - zero_offset_deg;
    can_error = HAL_CAN_GetError(&hcan1);
    uint32_t faults = 0;
    if (!feedback_online) faults |= FAULT_FEEDBACK;
    if (!zero_valid) faults |= FAULT_ZERO;
    if (zero_valid && fabsf(actual_angle_deg) > MotorConfig::angle_limit_deg)
        faults |= FAULT_ANGLE;
    if (MotorConfig::has_temperature &&
        motor_temperature_c >= MotorConfig::temperature_limit_c)
        faults |= FAULT_TEMPERATURE;
    if ((hcan1.Instance->ESR & CAN_ESR_BOFF) != 0) faults |= FAULT_CAN;
    if (!isfinite(target_angle_deg) ||
        fabsf(target_angle_deg) > MotorConfig::angle_limit_deg ||
        !isfinite(target_speed_deg_s) || !isfinite(manual_current_a) ||
        !isfinite(current_limit_a) || current_limit_a <= 0 ||
        current_limit_a > MotorConfig::maximum_current_a ||
        !isfinite(speed_limit_deg_s) || speed_limit_deg_s <= 0 ||
        !isfinite(ff_peak_current_a) || ff_peak_current_a < 0 ||
        ff_peak_current_a > MotorConfig::maximum_current_a ||
        !isfinite(ff_scale) || fabsf(ff_scale) > 3.0f ||
        !validParameters(position_pid_params) || !validParameters(speed_pid_params) ||
        control_mode < CONTROL_CURRENT || control_mode > CONTROL_POSITION)
        faults |= FAULT_PARAMETER;
    fault_flags = faults;
    if (faults != 0) stop_flag = 1;
    if (pid_reset_request != 0 || previous_mode != control_mode) {
        resetPid();
        pid_reset_request = 0;
    }
    previous_mode = control_mode;
    bool running = stop_flag == 0 && faults == 0;
    if (step_request != 0) {
        float next_target = target_angle_deg + step_delta_deg;
        if (running && control_mode == CONTROL_POSITION &&
            isfinite(step_delta_deg) && step_delta_deg != 0 &&
            fabsf(next_target) <= MotorConfig::angle_limit_deg) {
            target_angle_deg = next_target;
            trace_target = next_target;
            trace_direction = step_delta_deg > 0 ? 1.0f : -1.0f;
            trace_count = 0;
            trace_state = 1;
            trace_start_cycles = DWT->CYCCNT;
            settling_time_ms = UINT32_MAX;
            last_outside_ms = UINT32_MAX;
            step_overshoot_deg = 0;
        } else {
            trace_state = 3;
            settling_time_ms = UINT32_MAX;
        }
        step_request = 0;
    }
    angle_error_deg = target_angle_deg - actual_angle_deg;
    float gravity_sine = sinf(actual_angle_deg * 0.017453292519943295f);
    gravity_torque_nm = maximum_gravity_torque_nm * gravity_sine;
    if (!running) {
        resetPid();
        was_running = false;
        speed_reference_deg_s = 0;
        ff_current_a = 0;
        pid_current_a = 0;
        sendCurrent(0);
        recordStep(false);
        return;
    }
    if (!was_running) resetPid();
    was_running = true;
    configurePid(position_pid, position_pid_params, speed_limit_deg_s);
    ff_current_a = ff_enable && control_mode != CONTROL_CURRENT ?
                   ff_peak_current_a * ff_scale * gravity_sine : 0;
    ff_current_a = clamp(ff_current_a, current_limit_a);
    configurePid(speed_pid, speed_pid_params, current_limit_a);
    speed_reference_deg_s = 0;
    pid_current_a = 0;
    float output = ff_current_a;
    if (control_mode == CONTROL_CURRENT) {
        output = manual_current_a;
    } else if (control_mode == CONTROL_SPEED || control_mode == CONTROL_POSITION) {
        speed_reference_deg_s = control_mode == CONTROL_POSITION ?
            position_pid.calc(target_angle_deg, actual_angle_deg) :
            clamp(target_speed_deg_s, speed_limit_deg_s);
        pid_current_a = speed_pid.calc(speed_reference_deg_s, actual_speed_deg_s,
                                      ff_current_a);
        output += pid_current_a;
    }
    if (!isfinite(output)) {
        stop_flag = 1;
        fault_flags |= FAULT_PARAMETER;
        resetPid();
        output = 0;
    }
    sendCurrent(clamp(output, current_limit_a));
    recordStep(running);
}
}

extern "C" void App_Init(void) {
    stop_flag = 1;
    resetPid();
    CAN_FilterTypeDef filter = {};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (0x200 + MotorConfig::motor_id) << 5;
    filter.FilterMaskIdHigh = 0x7FF << 5;
    filter.FilterMaskIdLow = 0x0006;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan1) != HAL_OK ||
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    sendCurrent(0);
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *handle) {
    if (handle->Instance != CAN1) return;
    while (HAL_CAN_GetRxFifoFillLevel(handle, CAN_RX_FIFO0) != 0) {
        CAN_RxHeaderTypeDef header = {};
        uint8_t data[8] = {0};
        if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, data) != HAL_OK) {
            ++rx_invalid_count;
            return;
        }
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
            header.DLC != 8 || header.StdId != 0x200 + MotorConfig::motor_id) {
            ++rx_invalid_count;
            continue;
        }
        uint32_t now = HAL_GetTick();
        bool restart = received_feedback &&
            now - last_feedback_tick >= MotorConfig::feedback_timeout_ms;
        if (!motor.receive(data, restart)) {
            ++rx_invalid_count;
            continue;
        }
        if (restart) {
            feedback_restarted = true;
            stop_flag = 1;
        }
        received_feedback = true;
        last_feedback_tick = now;
        ++rx_count;
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *handle) {
    if (handle->Instance != TIM6) return;
    uint32_t started = DWT->CYCCNT;
    controlStep();
    control_cycles = DWT->CYCCNT - started;
    if (control_cycles > control_cycles_max) control_cycles_max = control_cycles;
}
