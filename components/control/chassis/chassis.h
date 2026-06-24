#pragma once

/*
 * chassis.h - Differential-drive chassis for the follow-me suitcase.
 *
 * Mechanics (from the project hardware):
 *   - Two REAR wheels are driven by motors (left + right).
 *   - Two FRONT wheels are passive casters (universal wheels).
 * This is a classic rear-axle differential drive: forward speed `v` plus yaw
 * rate `omega` are realised by spinning the two rear wheels at different
 * speeds; the front casters follow freely.
 *
 * The driver targets a generic dual H-bridge such as the TB6612FNG or DRV8833:
 * each motor has two direction pins (IN1/IN2) and one PWM (speed) pin. The
 * hardware specifics are fully configurable through chassis_config_t / Kconfig,
 * so adapting to a different driver only means changing pins and (if your
 * driver takes a single DIR pin) the small piece of code in chassis_apply_duty.
 *
 * Speed control is OPEN LOOP (no wheel encoders on this build): a normalised
 * duty in [-1, 1] is mapped to PWM. `max_speed_mps` is a calibration constant
 * that tells the kinematics what wheel linear speed full duty corresponds to,
 * so the follow controller can reason in m/s. If you later add encoders, wrap
 * chassis_set_velocity() with a PID and the rest of the stack is unchanged.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "esp_err.h"
#else
typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* PWM (LEDC) */
    int pwm_freq_hz;           /* e.g. 20000 -> above audible range */
    int pwm_resolution_bits;   /* e.g. 10 -> duty 0..1023 */
    int ledc_speed_mode;       /* LEDC_LOW_SPEED_MODE on ESP32-S3 */
    int ledc_timer;            /* LEDC_TIMER_0.. */
    int left_ledc_channel;     /* LEDC_CHANNEL_0.. */
    int right_ledc_channel;

    /* Left (rear-left) motor pins */
    int left_pwm_gpio;
    int left_in1_gpio;
    int left_in2_gpio;
    bool left_invert;          /* swap forward/backward if wired mirrored */

    /* Right (rear-right) motor pins */
    int right_pwm_gpio;
    int right_in1_gpio;
    int right_in2_gpio;
    bool right_invert;

    /* Kinematics / calibration */
    float track_width_m;       /* distance between the two rear wheels (m) */
    float max_speed_mps;       /* wheel linear speed at duty = 1.0 (m/s) */
    float min_duty;            /* dead-zone compensation: smallest duty that
                                  actually makes the motor turn (0..1) */
    float max_duty;            /* upper clamp, <= 1.0 (limit top speed/current) */
    float slew_per_call;       /* max |duty| change per chassis_set_* call, for
                                  current-spike / gear protection. 0 disables. */
} chassis_config_t;

typedef struct {
    chassis_config_t cfg;
    int duty_max;              /* (1 << resolution) - 1 */
    bool initialized;
    float last_left_duty;
    float last_right_duty;
} chassis_t;

/* Sensible defaults; override pins via Kconfig in the application. */
chassis_config_t chassis_default_config(void);

esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg);
void chassis_deinit(chassis_t *ch);

/*
 * Command a body velocity.
 *   v_mps   : forward linear speed, + = forward, - = reverse.
 *   omega_rps: yaw rate, + = turn LEFT (CCW), - = turn RIGHT (CW).
 * Internally mixed into per-wheel duties and clamped to the configured limits.
 */
esp_err_t chassis_set_velocity(chassis_t *ch, float v_mps, float omega_rps);

/* Command normalised per-wheel duty directly, each in [-1, 1]. */
esp_err_t chassis_set_duty(chassis_t *ch, float left, float right);

/* Coast to a stop (PWM 0, both direction pins low). */
void chassis_stop(chassis_t *ch);

/* Active short brake (both direction pins high). Stronger than coast. */
void chassis_brake(chassis_t *ch);

/* ---- Pure kinematics (no hardware; unit-testable on PC) ------------------ */

/*
 * Differential-drive mixing. Given a body command (v, omega) produce the two
 * normalised wheel duties in [-1, 1].
 *   left  = (v - omega * track/2) / max_speed
 *   right = (v + omega * track/2) / max_speed
 * If either wheel would exceed +-1 both are scaled down together so the turn
 * geometry (the ratio between wheels) is preserved instead of being clipped.
 */
void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty);

#ifdef __cplusplus
}
#endif
