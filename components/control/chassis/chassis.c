#include "chassis.h"

#include <math.h>
#include <string.h>

/* ---- Pure kinematics (compiled on both ESP and PC) ----------------------- */

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

void chassis_diff_drive_mix(float v_mps, float omega_rps, float track_width_m,
                            float max_speed_mps, float *left_duty,
                            float *right_duty)
{
    float l = 0.0f;
    float r = 0.0f;
    if (max_speed_mps > 1e-6f) {
        const float half = 0.5f * track_width_m;
        l = (v_mps - omega_rps * half) / max_speed_mps;
        r = (v_mps + omega_rps * half) / max_speed_mps;
    }

    /* If saturated, scale both wheels equally to preserve the turn ratio. */
    float m = fabsf(l);
    if (fabsf(r) > m) {
        m = fabsf(r);
    }
    if (m > 1.0f) {
        l /= m;
        r /= m;
    }

    if (left_duty) {
        *left_duty = clampf(l, -1.0f, 1.0f);
    }
    if (right_duty) {
        *right_duty = clampf(r, -1.0f, 1.0f);
    }
}

chassis_config_t chassis_default_config(void)
{
    chassis_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.pwm_freq_hz = 20000;
    cfg.pwm_resolution_bits = 10;
#ifdef ESP_PLATFORM
    cfg.ledc_speed_mode = 0; /* LEDC_LOW_SPEED_MODE */
    cfg.ledc_timer = 0;      /* LEDC_TIMER_0 */
    cfg.left_ledc_channel = 0;
    cfg.right_ledc_channel = 1;
#endif
    cfg.left_pwm_gpio = -1;
    cfg.left_in1_gpio = -1;
    cfg.left_in2_gpio = -1;
    cfg.left_invert = false;
    cfg.right_pwm_gpio = -1;
    cfg.right_in1_gpio = -1;
    cfg.right_in2_gpio = -1;
    cfg.right_invert = false;

    cfg.track_width_m = 0.30f;  /* ~30 cm between rear wheels (typical case) */
    cfg.max_speed_mps = 0.8f;   /* calibrate: wheel speed at full duty */
    cfg.min_duty = 0.10f;       /* dead-zone compensation */
    cfg.max_duty = 0.90f;       /* leave headroom / limit current */
    cfg.slew_per_call = 0.08f;  /* smooth ramp, protects gearbox */
    return cfg;
}

#ifdef ESP_PLATFORM

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "chassis";

static esp_err_t cfg_dir_pin(int gpio)
{
    if (gpio < 0) {
        return ESP_OK;
    }
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io);
    if (ret == ESP_OK) {
        gpio_set_level(gpio, 0);
    }
    return ret;
}

static esp_err_t cfg_pwm_channel(const chassis_config_t *cfg, int channel,
                                 int gpio)
{
    ledc_channel_config_t ch = {
        .gpio_num = gpio,
        .speed_mode = (ledc_mode_t)cfg->ledc_speed_mode,
        .channel = (ledc_channel_t)channel,
        .timer_sel = (ledc_timer_t)cfg->ledc_timer,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    return ledc_channel_config(&ch);
}

esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg)
{
    if (ch == NULL || cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(ch, 0, sizeof(*ch));
    ch->cfg = *cfg;
    ch->duty_max = (1 << cfg->pwm_resolution_bits) - 1;

    ledc_timer_config_t tcfg = {
        .speed_mode = (ledc_mode_t)cfg->ledc_speed_mode,
        .timer_num = (ledc_timer_t)cfg->ledc_timer,
        .duty_resolution = (ledc_timer_bit_t)cfg->pwm_resolution_bits,
        .freq_hz = cfg->pwm_freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&tcfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %d", ret);
        return ret;
    }

    ret = cfg_pwm_channel(cfg, cfg->left_ledc_channel, cfg->left_pwm_gpio);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = cfg_pwm_channel(cfg, cfg->right_ledc_channel, cfg->right_pwm_gpio);
    if (ret != ESP_OK) {
        return ret;
    }

    if (cfg_dir_pin(cfg->left_in1_gpio) != ESP_OK ||
        cfg_dir_pin(cfg->left_in2_gpio) != ESP_OK ||
        cfg_dir_pin(cfg->right_in1_gpio) != ESP_OK ||
        cfg_dir_pin(cfg->right_in2_gpio) != ESP_OK) {
        return ESP_FAIL;
    }

    ch->initialized = true;
    ESP_LOGI(TAG, "init: PWM L=GPIO%d R=GPIO%d track=%.2fm vmax=%.2fm/s",
             cfg->left_pwm_gpio, cfg->right_pwm_gpio, cfg->track_width_m,
             cfg->max_speed_mps);
    return ESP_OK;
}

/* Drive one motor: sign -> direction pins, magnitude -> PWM duty. */
static void apply_motor(chassis_t *ch, int channel, int in1, int in2,
                        bool invert, float duty)
{
    const chassis_config_t *cfg = &ch->cfg;
    float d = duty;
    if (invert) {
        d = -d;
    }

    int forward;
    float mag = fabsf(d);
    if (mag < 1e-3f) {
        /* Coast. */
        if (in1 >= 0) {
            gpio_set_level(in1, 0);
        }
        if (in2 >= 0) {
            gpio_set_level(in2, 0);
        }
        ledc_set_duty((ledc_mode_t)cfg->ledc_speed_mode,
                      (ledc_channel_t)channel, 0);
        ledc_update_duty((ledc_mode_t)cfg->ledc_speed_mode,
                         (ledc_channel_t)channel);
        return;
    }

    forward = (d > 0.0f);
    if (in1 >= 0) {
        gpio_set_level(in1, forward ? 1 : 0);
    }
    if (in2 >= 0) {
        gpio_set_level(in2, forward ? 0 : 1);
    }

    /* Map [min_duty, max_duty] magnitude onto the PWM range so the motor
     * actually starts moving instead of buzzing below its dead-zone. */
    mag = clampf(mag, 0.0f, cfg->max_duty);
    float scaled = cfg->min_duty + mag * (1.0f - cfg->min_duty);
    uint32_t raw = (uint32_t)(scaled * (float)ch->duty_max + 0.5f);
    if (raw > (uint32_t)ch->duty_max) {
        raw = (uint32_t)ch->duty_max;
    }
    ledc_set_duty((ledc_mode_t)cfg->ledc_speed_mode, (ledc_channel_t)channel,
                  raw);
    ledc_update_duty((ledc_mode_t)cfg->ledc_speed_mode,
                     (ledc_channel_t)channel);
}

static float slew(float target, float current, float max_step)
{
    if (max_step <= 0.0f) {
        return target;
    }
    float delta = target - current;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }
    return current + delta;
}

esp_err_t chassis_set_duty(chassis_t *ch, float left, float right)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    left = clampf(left, -1.0f, 1.0f);
    right = clampf(right, -1.0f, 1.0f);

    left = slew(left, ch->last_left_duty, ch->cfg.slew_per_call);
    right = slew(right, ch->last_right_duty, ch->cfg.slew_per_call);
    ch->last_left_duty = left;
    ch->last_right_duty = right;

    apply_motor(ch, ch->cfg.left_ledc_channel, ch->cfg.left_in1_gpio,
                ch->cfg.left_in2_gpio, ch->cfg.left_invert, left);
    apply_motor(ch, ch->cfg.right_ledc_channel, ch->cfg.right_in1_gpio,
                ch->cfg.right_in2_gpio, ch->cfg.right_invert, right);
    return ESP_OK;
}

esp_err_t chassis_set_velocity(chassis_t *ch, float v_mps, float omega_rps)
{
    if (ch == NULL || !ch->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    float l = 0.0f;
    float r = 0.0f;
    chassis_diff_drive_mix(v_mps, omega_rps, ch->cfg.track_width_m,
                           ch->cfg.max_speed_mps, &l, &r);
    return chassis_set_duty(ch, l, r);
}

void chassis_stop(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    ch->last_left_duty = 0.0f;
    ch->last_right_duty = 0.0f;
    apply_motor(ch, ch->cfg.left_ledc_channel, ch->cfg.left_in1_gpio,
                ch->cfg.left_in2_gpio, ch->cfg.left_invert, 0.0f);
    apply_motor(ch, ch->cfg.right_ledc_channel, ch->cfg.right_in1_gpio,
                ch->cfg.right_in2_gpio, ch->cfg.right_invert, 0.0f);
}

void chassis_brake(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    const chassis_config_t *cfg = &ch->cfg;
    ch->last_left_duty = 0.0f;
    ch->last_right_duty = 0.0f;
    if (cfg->left_in1_gpio >= 0) {
        gpio_set_level(cfg->left_in1_gpio, 1);
    }
    if (cfg->left_in2_gpio >= 0) {
        gpio_set_level(cfg->left_in2_gpio, 1);
    }
    if (cfg->right_in1_gpio >= 0) {
        gpio_set_level(cfg->right_in1_gpio, 1);
    }
    if (cfg->right_in2_gpio >= 0) {
        gpio_set_level(cfg->right_in2_gpio, 1);
    }
    ledc_set_duty((ledc_mode_t)cfg->ledc_speed_mode,
                  (ledc_channel_t)cfg->left_ledc_channel, 0);
    ledc_update_duty((ledc_mode_t)cfg->ledc_speed_mode,
                     (ledc_channel_t)cfg->left_ledc_channel);
    ledc_set_duty((ledc_mode_t)cfg->ledc_speed_mode,
                  (ledc_channel_t)cfg->right_ledc_channel, 0);
    ledc_update_duty((ledc_mode_t)cfg->ledc_speed_mode,
                     (ledc_channel_t)cfg->right_ledc_channel);
}

void chassis_deinit(chassis_t *ch)
{
    if (ch == NULL || !ch->initialized) {
        return;
    }
    chassis_stop(ch);
    ledc_stop((ledc_mode_t)ch->cfg.ledc_speed_mode,
              (ledc_channel_t)ch->cfg.left_ledc_channel, 0);
    ledc_stop((ledc_mode_t)ch->cfg.ledc_speed_mode,
              (ledc_channel_t)ch->cfg.right_ledc_channel, 0);
    ch->initialized = false;
}

#else /* !ESP_PLATFORM : PC stubs so kinematics can be unit-tested ---------- */

esp_err_t chassis_init(chassis_t *ch, const chassis_config_t *cfg)
{
    (void)ch;
    (void)cfg;
    return ESP_ERR_INVALID_STATE;
}
void chassis_deinit(chassis_t *ch) { (void)ch; }
esp_err_t chassis_set_velocity(chassis_t *ch, float v, float w)
{
    (void)ch;
    (void)v;
    (void)w;
    return ESP_ERR_INVALID_STATE;
}
esp_err_t chassis_set_duty(chassis_t *ch, float l, float r)
{
    (void)ch;
    (void)l;
    (void)r;
    return ESP_ERR_INVALID_STATE;
}
void chassis_stop(chassis_t *ch) { (void)ch; }
void chassis_brake(chassis_t *ch) { (void)ch; }

#endif /* ESP_PLATFORM */
