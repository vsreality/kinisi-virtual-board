/* Register-level output model for the v0.4 firmware configuration. See bmi088.py.
 * The two backends share the same deterministic streams and golden-vector tests. */
#ifndef KINISI_BMI088_MODEL_H
#define KINISI_BMI088_MODEL_H
#include <math.h>
#include <stdint.h>
#include <string.h>

#define BMI088_PI 3.14159265358979323846
#define BMI088_G 9.80665
#define BMI088_RAD (BMI088_PI / 180.0)
#define BMI088_PERIOD_US 10000u

typedef struct {
    double values[6], signal[6], noise[6], bias[6], gain[6];
    uint64_t accel_us, gyro_us, now_us, next_us[2];
    uint32_t rng[2];
} bmi088_model_t;

static double bmi088_uniform(uint32_t *state) {
    uint32_t x = (*state += UINT32_C(0x6d2b79f5));
    x = (x ^ (x >> 15)) * (x | 1u);
    x ^= x + (x ^ (x >> 7)) * (x | 61u);
    return ((double)(x ^ (x >> 14)) + .5) / 4294967296.;
}

static double bmi088_normal(uint32_t *state) {
    double radius = sqrt(-2. * log(bmi088_uniform(state)));
    return radius * cos(2. * BMI088_PI * bmi088_uniform(state));
}

static void bmi088_init(bmi088_model_t *s, uint32_t seed, double temp_c) {
    memset(s, 0, sizeof(*s));
    s->rng[0] = seed ^ UINT32_C(0x9e3779b9);
    s->rng[1] = seed ^ UINT32_C(0x85ebca6b);
    s->next_us[0] = BMI088_PERIOD_US;
    s->next_us[1] = BMI088_PERIOD_US + 2500;
    s->signal[2] = BMI088_G;
    for (int i = 0; i < 6; ++i) {
        double offset = (i < 3 ? .020 * BMI088_G : BMI088_RAD) * (2. * bmi088_uniform(&seed) - 1.);
        double tco = (i < 3 ? .0002 * BMI088_G : .015 * BMI088_RAD) * (2. * bmi088_uniform(&seed) - 1.);
        double tcs = (i < 3 ? .00002 : .0003) * (2. * bmi088_uniform(&seed) - 1.);
        s->bias[i] = offset + (temp_c - 25.) * tco;
        s->gain[i] = 1. + (temp_c - 25.) * tcs;
    }
}

static void bmi088_advance(bmi088_model_t *s, uint64_t at_us,
                           const double body_accel[3], const double body_gyro[3]) {
    static const double density[] = {160e-6 * BMI088_G, 160e-6 * BMI088_G, 190e-6 * BMI088_G,
                                     .014 * BMI088_RAD, .014 * BMI088_RAD, .014 * BMI088_RAD};
    for (int die = 0; die < 2; ++die) {
        const double *input = die ? body_gyro : body_accel;
        const double cutoff = die ? 32. : 40.;
        const double quantum = (die ? 500. * BMI088_RAD : 6. * BMI088_G) / 32768.;
        uint64_t *stamp = die ? &s->gyro_us : &s->accel_us;
        for (uint64_t start = s->now_us; start < at_us;) {
            uint64_t end = at_us < s->next_us[die] ? at_us : s->next_us[die];
            double retain = exp(-2. * BMI088_PI * cutoff * (double)(end - start) / 1e6);
            for (int axis = 0; axis < 3; ++axis) {
                int i = 3 * die + axis;
                double target = axis == 2 ? input[axis] : -input[axis];
                s->signal[i] = target + (s->signal[i] - target) * retain;
            }
            if (end == s->next_us[die]) {
                double rho = *stamp ? exp(-2. * BMI088_PI * cutoff * BMI088_PERIOD_US / 1e6) : 0.;
                double innovation = sqrt(1. - rho*rho);
                double root_bandwidth = sqrt(BMI088_PI * cutoff / 2.);
                for (int i = 3 * die; i < 3 * die + 3; ++i) {
                    s->noise[i] = rho * s->noise[i] + innovation * bmi088_normal(&s->rng[die]);
                    double reading = s->signal[i] * s->gain[i] + s->bias[i] + density[i] * root_bandwidth * s->noise[i];
                    double code = floor(reading / quantum + .5);
                    s->values[i] = fmax(-32768., fmin(32767., code)) * quantum;
                }
                *stamp = end;
                s->next_us[die] += BMI088_PERIOD_US;
            }
            start = end;
        }
    }
    s->now_us = at_us;
}

#endif
