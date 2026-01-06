/**
 * @file    hall.c
 * @brief   Hall sensor processing (speed/distance) for bike computer.
 */

#include "hall.h"

#include <string.h>

/* -------- internal helpers -------- */
static float ema_alpha(float dt_s, float tau_s)
{
    if (tau_s <= 0.0f) {
        return 1.0f;
    }
    if (dt_s <= 0.0f) {
        return 0.0f;
    }
    return dt_s / (tau_s + dt_s);
}

void Hall_Init(hall_state_t *h, float wheel_circ_m)
{
    if (h == NULL) {
        return;
    }
    memset(h, 0, sizeof(*h));

    /* defaults (safe for bicycle) */
    h->wheel_circ_m    = wheel_circ_m;
    h->v_max_kmh       = 120.0f;
    h->debounce_ms     = 3u;
    h->stop_timeout_ms = 2500u;
    h->ema_tau_s       = 0.6f;
}

void Hall_SetParams(hall_state_t *h,
                    uint32_t debounce_ms,
                    uint32_t stop_timeout_ms,
                    float v_max_kmh,
                    float ema_tau_s)
{
    if (h == NULL) {
        return;
    }
    h->debounce_ms     = debounce_ms;
    h->stop_timeout_ms = stop_timeout_ms;
    h->v_max_kmh       = v_max_kmh;
    h->ema_tau_s       = ema_tau_s;
}

void Hall_ResetTrip(hall_state_t *h)
{
    if (h == NULL) {
        return;
    }
    h->pulse_count     = 0;
    h->last_pulse_ms   = 0;
    h->last_period_ms  = 0;
    h->last_task_ms    = 0;
    h->speed_inst_kmh  = 0.0f;
    h->speed_filt_kmh  = 0.0f;
    h->distance_m      = 0.0f;
    h->rejected_pulses = 0;
}

void Hall_OnEdgeMs(hall_state_t *h, uint32_t now_ms)
{
    if (h == NULL) {
        return;
    }

    /* First pulse: just latch timestamp. */
    if (h->last_pulse_ms == 0u) {
        h->last_pulse_ms = now_ms;
        h->last_task_ms  = now_ms;
        return;
    }

    const uint32_t dt_ms = (uint32_t)(now_ms - h->last_pulse_ms); /* modulo arithmetic */
    h->last_pulse_ms = now_ms;

    /* debounce / anti-spike */
    if (dt_ms < h->debounce_ms) {
        h->rejected_pulses++;
        return;
    }

    /* Avoid divide by zero. */
    const uint32_t dt_ms_safe = (dt_ms == 0u) ? 1u : dt_ms;

    /* v[km/h] = 3.6 * C[m] / dt[s] = 3600 * C / dt[ms] */
    const float v_kmh = 3600.0f * h->wheel_circ_m / (float)dt_ms_safe;

    /* sanity max speed */
    if (h->v_max_kmh > 0.0f && v_kmh > h->v_max_kmh) {
        h->rejected_pulses++;
        return;
    }

    /* accepted pulse */
    h->last_period_ms = dt_ms_safe;
    h->pulse_count++;
    h->distance_m = (float)h->pulse_count * h->wheel_circ_m;
    h->speed_inst_kmh = v_kmh;

    /* EMA update with alpha based on Δt */
    const float dt_s = (float)dt_ms_safe / 1000.0f;
    const float a    = ema_alpha(dt_s, h->ema_tau_s);
    if (h->speed_filt_kmh <= 0.0f) {
        /* Fast convergence on first valid measurement */
        h->speed_filt_kmh = v_kmh;
    } else {
        h->speed_filt_kmh = h->speed_filt_kmh + a * (v_kmh - h->speed_filt_kmh);
    }

    /* keep maintenance time consistent */
    h->last_task_ms = now_ms;
}

void Hall_TaskMs(hall_state_t *h, uint32_t now_ms)
{
    if (h == NULL) {
        return;
    }
    if (h->last_pulse_ms == 0u) {
        /* no pulses yet */
        h->last_task_ms = now_ms;
        return;
    }

    /* Stop detection */
    const uint32_t since_pulse_ms = (uint32_t)(now_ms - h->last_pulse_ms);
    if (since_pulse_ms > h->stop_timeout_ms) {
        h->speed_inst_kmh = 0.0f;
    }

    /* EMA decay towards zero between pulses / at stop. */
    const uint32_t dt_ms = (h->last_task_ms == 0u) ? 0u : (uint32_t)(now_ms - h->last_task_ms);
    h->last_task_ms = now_ms;

    if (dt_ms == 0u) {
        return;
    }

    const float dt_s = (float)dt_ms / 1000.0f;
    const float a    = ema_alpha(dt_s, h->ema_tau_s);
    h->speed_filt_kmh = h->speed_filt_kmh + a * (0.0f - h->speed_filt_kmh);

    /* prevent tiny negative or denormal values */
    if (h->speed_filt_kmh < 0.01f) {
        h->speed_filt_kmh = 0.0f;
    }
}
