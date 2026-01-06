/**
 * @file    hall.h
 * @brief   Hall sensor processing (speed/distance) for bike computer.
 *
 * Design goals:
 *  - robust pulse validation (debounce + v_max sanity)
 *  - speed from period between pulses: v = C / Δt
 *  - distance from pulse count: s = N * C
 *  - stable display via EMA filter
 *  - stop detection (timeout)
 */

#ifndef INC_HALL_H_
#define INC_HALL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Runtime state and configuration of the Hall processing.
 */
typedef struct
{
    /* --- configuration --- */
    float     wheel_circ_m;       /**< Wheel circumference C [m]. */
    float     v_max_kmh;          /**< Sanity max speed [km/h]. */
    uint32_t  debounce_ms;        /**< Reject pulses faster than this [ms]. */
    uint32_t  stop_timeout_ms;    /**< If no pulse for this time -> speed=0 [ms]. */
    float     ema_tau_s;          /**< EMA time constant τ [s]. */

    /* --- dynamic state --- */
    volatile uint32_t pulse_count;      /**< Accepted pulse count N. */
    volatile uint32_t last_pulse_ms;    /**< Timestamp of last accepted pulse [ms]. */
    volatile uint32_t last_period_ms;   /**< Δt of last accepted period [ms]. */
    uint32_t          last_task_ms;     /**< Timestamp of last Hall_TaskMs() [ms]. */

    float speed_inst_kmh;   /**< Instantaneous speed from last period [km/h]. */
    float speed_filt_kmh;   /**< Filtered speed (EMA) [km/h]. */
    float distance_m;       /**< Distance s [m]. */

    /* --- diagnostics --- */
    uint32_t rejected_pulses;
} hall_state_t;

/**
 * @brief Initialize Hall state with defaults.
 *
 * Defaults are conservative for a bicycle application and may be adjusted
 * by Hall_SetParams().
 */
void Hall_Init(hall_state_t *h, float wheel_circ_m);

/**
 * @brief Set runtime parameters.
 */
void Hall_SetParams(hall_state_t *h,
                    uint32_t debounce_ms,
                    uint32_t stop_timeout_ms,
                    float v_max_kmh,
                    float ema_tau_s);

/**
 * @brief Reset trip counters (distance, speed, pulses).
 */
void Hall_ResetTrip(hall_state_t *h);

/**
 * @brief Process a Hall pulse edge.
 *
 * Call from EXTI ISR (or from Input Capture ISR if used).
 * @param now_ms Current timestamp [ms] (e.g., HAL_GetTick()).
 */
void Hall_OnEdgeMs(hall_state_t *h, uint32_t now_ms);

/**
 * @brief Periodic maintenance (stop timeout + EMA decay).
 *
 * Call from main loop (e.g., every 10...50 ms) or from a periodic timer.
 */
void Hall_TaskMs(hall_state_t *h, uint32_t now_ms);

/* Convenience getters (optional) */
static inline float Hall_SpeedInstKmh(const hall_state_t *h) { return h->speed_inst_kmh; }
static inline float Hall_SpeedFiltKmh(const hall_state_t *h) { return h->speed_filt_kmh; }
static inline float Hall_DistanceM(const hall_state_t *h)     { return h->distance_m; }
static inline uint32_t Hall_Pulses(const hall_state_t *h)     { return h->pulse_count; }

#ifdef __cplusplus
}
#endif

#endif /* INC_HALL_H_ */
