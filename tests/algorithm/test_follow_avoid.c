/*
 * PC-side unit tests for the follow + obstacle-avoidance algorithm.
 * Pure logic, no hardware: compile and run with run_tests.sh.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "chassis.h"
#include "follow_avoid.h"

#define M_PI 3.14159265358979323846

static int approx(float a, float b, float tol) { return fabsf(a - b) < tol; }

/* Drive the algorithm for n steps with constant inputs (lets the accel-limited
 * command settle), returning the final output. */
static fa_output_t run_steps(fa_ctx_t *ctx, const fa_target_t *t,
                             const fa_obstacle_field_t *f, const fa_range_t *ul,
                             const fa_range_t *ur, int n, float dt)
{
    fa_output_t o;
    memset(&o, 0, sizeof(o));
    for (int i = 0; i < n; ++i) {
        o = fa_update(ctx, t, f, ul, ur, dt);
    }
    return o;
}

/* ------------------------------------------------------------ kinematics */

static void test_diff_drive_straight(void)
{
    float l = 0, r = 0;
    chassis_diff_drive_mix(0.5f, 0.0f, 0.30f, 0.8f, &l, &r);
    assert(approx(l, 0.625f, 1e-4f));
    assert(approx(r, 0.625f, 1e-4f));
}

static void test_diff_drive_pivot(void)
{
    float l = 0, r = 0;
    chassis_diff_drive_mix(0.0f, 2.0f, 0.30f, 0.8f, &l, &r);
    /* Pure left turn: left wheel reverses, right wheel forwards, symmetric. */
    assert(l < 0.0f && r > 0.0f);
    assert(approx(l, -r, 1e-4f));
}

static void test_diff_drive_saturation(void)
{
    float l = 0, r = 0;
    chassis_diff_drive_mix(0.8f, 4.0f, 0.30f, 0.8f, &l, &r);
    /* Right wheel would exceed 1.0, so both scale down but keep their ratio. */
    assert(r <= 1.0f + 1e-4f);
    assert(approx(r, 1.0f, 1e-3f));
    assert(l > 0.0f && l < r);
}

/* ------------------------------------------------------------ wheel PID (算法2) */

static void mk_pid(chassis_pid_t *pid, float kp, float ki, float kd, float lim)
{
    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = -lim;
    pid->out_max = lim;
    chassis_pid_reset(pid);
}

static void test_pid_sign_and_clamp(void)
{
    chassis_pid_t pid;
    mk_pid(&pid, 1000.0f, 0.0f, 0.0f, 400.0f);
    /* P-only, error = +1 -> kp*err = 1000, clamped to out_max. */
    assert(approx(chassis_pid_step(&pid, 1.0f, 0.0f, 0.02f), 400.0f, 1e-3f));
    mk_pid(&pid, 1000.0f, 0.0f, 0.0f, 400.0f);
    assert(approx(chassis_pid_step(&pid, 0.0f, 1.0f, 0.02f), -400.0f, 1e-3f));
}

static void test_pid_integrator_antiwindup(void)
{
    chassis_pid_t pid;
    mk_pid(&pid, 0.0f, 500.0f, 0.0f, 400.0f);
    float out = 0.0f;
    for (int i = 0; i < 1000; ++i) {
        out = chassis_pid_step(&pid, 1.0f, 0.0f, 0.02f); /* constant error */
    }
    /* I-only term must saturate at the limit, never beyond it. */
    assert(out <= 400.0f + 1e-3f);
    assert(approx(out, 400.0f, 1e-2f));
}

/* Closed-loop: feed-forward + PID against a simple first-order wheel model must
 * converge to the speed set-point (proves the integral kills the FF mismatch). */
static void test_pid_closed_loop_converges(void)
{
    chassis_pid_t pid;
    mk_pid(&pid, 200.0f, 300.0f, 5.0f, 400.0f);
    const float ff = 625.0f;        /* us per m/s feed-forward */
    const float plant_g = 0.0012f;  /* m/s per us (motor weaker than FF assumes) */
    const float beta = 0.3f;        /* first-order lag */
    const float ts = 0.5f;          /* target wheel speed (m/s) */
    const float dt = 0.02f;

    float meas = 0.0f;
    for (int i = 0; i < 800; ++i) {
        const float total = ff * ts + chassis_pid_step(&pid, ts, meas, dt);
        meas += (plant_g * total - meas) * beta;
    }
    assert(approx(meas, ts, 0.02f)); /* tracks the set-point despite FF error */
}

static void test_pid_reset_clears_state(void)
{
    chassis_pid_t pid;
    mk_pid(&pid, 0.0f, 500.0f, 0.0f, 400.0f);
    for (int i = 0; i < 50; ++i) {
        chassis_pid_step(&pid, 1.0f, 0.0f, 0.02f);
    }
    assert(pid.i_term > 1.0f);
    chassis_pid_reset(&pid);
    assert(approx(pid.i_term, 0.0f, 1e-6f));
    assert(!pid.has_prev);
}

/* ------------------------------------------------------------ obstacle field */

static void test_obstacle_field(void)
{
    fa_obstacle_field_t f;
    fa_obstacle_reset(&f, 36, (float)M_PI);
    assert(f.num_sectors == 36);

    fa_obstacle_add(&f, 0.0f, 0.80f);
    fa_obstacle_add(&f, 0.0f, 0.50f); /* keeps the closer one */
    int center = (int)((0.0f + 0.5f * f.fov_rad) / f.sector_width_rad);
    assert(f.min_dist_m[center] <= 0.51f);

    /* Outside the FOV (behind) must be ignored. */
    fa_obstacle_add(&f, (float)M_PI, 0.10f);
    for (int i = 0; i < f.num_sectors; ++i) {
        assert(f.min_dist_m[i] > 0.49f);
    }
}

/* ------------------------------------------------------------ follow */

static void test_follow_straight(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa_target_t t = {.valid = true, .distance_m = 3.0f, .bearing_rad = 0.0f};
    fa_output_t o = run_steps(&fa, &t, NULL, NULL, NULL, 300, 0.02f);
    assert(o.state == FA_STATE_FOLLOW);
    assert(o.v_mps > 0.5f);            /* drives forward to catch up */
    assert(fabsf(o.omega_rps) < 0.15f);/* roughly straight */
}

static void test_follow_turns_toward_target(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa_target_t t = {.valid = true, .distance_m = 3.0f, .bearing_rad = 0.5f};
    fa_output_t o = run_steps(&fa, &t, NULL, NULL, NULL, 300, 0.02f);
    assert(o.omega_rps > 0.3f);        /* target on the left -> turn left */
    assert(o.v_mps > 0.1f);
}

static void test_follow_holds_in_stop_band(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    /* Comfortable distance (within follow_distance - stop_band) -> hold. */
    fa_target_t t = {.valid = true, .distance_m = 0.9f, .bearing_rad = 0.0f};
    fa_output_t o = run_steps(&fa, &t, NULL, NULL, NULL, 200, 0.02f);
    assert(o.v_mps < 0.05f);
}

/* ------------------------------------------------------------ avoidance */

static void test_emergency_stop_on_ultrasonic(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa_target_t t = {.valid = true, .distance_m = 3.0f, .bearing_rad = 0.0f};
    /* Get moving first. */
    fa_output_t moving = run_steps(&fa, &t, NULL, NULL, NULL, 300, 0.02f);
    assert(moving.v_mps > 0.3f);

    /* Now an obstacle appears 0.20 m off a front corner. */
    fa_range_t ul = {.valid = true, .dist_m = 0.20f};
    fa_output_t o = fa_update(&fa, &t, NULL, &ul, NULL, 0.02f);
    assert(o.state == FA_STATE_ESTOP);
    assert(o.v_mps < moving.v_mps);    /* commanded to brake */
}

static void test_avoid_steers_around_front_obstacle(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa_target_t t = {.valid = true, .distance_m = 3.0f, .bearing_rad = 0.0f};

    fa_obstacle_field_t f;
    fa_obstacle_reset(&f, 36, (float)M_PI);
    fa_obstacle_add(&f, 0.0f, 0.50f);  /* wall straight ahead, sides are open */

    fa_output_t o = run_steps(&fa, &t, &f, NULL, NULL, 50, 0.02f);
    assert(o.blocked);
    assert(o.state == FA_STATE_AVOID);
    assert(fabsf(o.chosen_heading_rad) > 0.2f); /* veer to a clear heading */
}

/* ------------------------------------------------------------ search */

static void test_search_when_target_lost(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    /* Establish a last-known bearing on the left. */
    fa_target_t seen = {.valid = true, .distance_m = 2.0f, .bearing_rad = 0.3f};
    run_steps(&fa, &seen, NULL, NULL, NULL, 100, 0.02f);

    /* Target disappears for ~1 s. */
    fa_target_t lost = {.valid = false};
    fa_output_t o = run_steps(&fa, &lost, NULL, NULL, NULL, 50, 0.02f);
    assert(o.state == FA_STATE_SEARCH);
    assert(o.omega_rps > 0.2f);        /* rotates toward last-known (left) */
    assert(o.v_mps < 0.1f);            /* without driving forward */
}

static void test_idle_when_never_seen(void)
{
    fa_ctx_t fa;
    fa_init(&fa, NULL);
    fa_target_t lost = {.valid = false};
    fa_output_t o = run_steps(&fa, &lost, NULL, NULL, NULL, 50, 0.02f);
    assert(o.state == FA_STATE_IDLE);
    assert(approx(o.v_mps, 0.0f, 1e-3f));
    assert(approx(o.omega_rps, 0.0f, 1e-3f));
}

int main(void)
{
    test_diff_drive_straight();
    test_diff_drive_pivot();
    test_diff_drive_saturation();
    test_pid_sign_and_clamp();
    test_pid_integrator_antiwindup();
    test_pid_closed_loop_converges();
    test_pid_reset_clears_state();
    test_obstacle_field();
    test_follow_straight();
    test_follow_turns_toward_target();
    test_follow_holds_in_stop_band();
    test_emergency_stop_on_ultrasonic();
    test_avoid_steers_around_front_obstacle();
    test_search_when_target_lost();
    test_idle_when_never_seen();

    printf("All follow_avoid algorithm tests passed.\n");
    return 0;
}
