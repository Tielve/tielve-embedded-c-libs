//
// Created by Jonathan Tielve
//

#ifndef TIELVE_FIXED_POINT_H
#define TIELVE_FIXED_POINT_H

#include <stdint.h>
#include <stdlib.h>

#define Q15SHIFT 15
#define Q7SHIFT 7
#define Q15MAX 32767
#define Q15MIN -32768
#define Q7MAX 127
#define Q7MIN -128

typedef int32_t q31_t;
typedef int16_t q15_t;
typedef int8_t  q7_t;

typedef struct {
    uint16_t rows;
    uint16_t cols;
    q15_t *data;
}q15_mat_t;

typedef struct {
    uint16_t rows;
    uint16_t cols;
    q7_t *data;
}q7_mat_t;

// --------------
// Q15 Operations
// --------------
static inline q15_t float_to_q15(float x) {
    if (x >= 1.0f) x = 0.99997f;
    if (x <= -1.0f) x = -1.0f;
    return (q15_t)(x * 32768.0f);
}

static inline float q15_to_float(q15_t x) {
    return (float)x / 32768.0f;
}

static inline q15_t int_to_q15(int16_t x) {
    return (q15_t)(x<<Q15SHIFT);
}

static inline int16_t q15_to_int(q15_t x) {
    return (int16_t)(x>>Q15SHIFT);
}

static inline q15_t q15_add(q15_t a, q15_t b) {
    return a + b;
}
    
static inline q15_t q15_sub(q15_t a, q15_t b) {
    return a - b;
}

static inline q15_t q15_mul(q15_t a, q15_t b) {
    return (q15_t)(((int32_t)a * b) >> Q15SHIFT);
}

static inline q15_t q15_div(q15_t a, q15_t b) {
    if(b == 0) {
        if(a >= 0) {
            return Q15MAX;
        }
        else {
            return Q15MIN;
        }
    }
    return ((int32_t)a << Q15SHIFT) / b;
}

static inline q15_t q15_clamp(q15_t x) {
    if (x > Q15MAX) return Q15MAX;
    if (x < Q15MIN) return Q15MIN;
    return x;
}

static inline int32_t q15_clamp_int(int32_t x) {
    if (x > Q15MAX) return Q15MAX;
    if (x < Q15MIN) return Q15MIN;
    return x;
}

static inline q15_t q15_add_sat(q15_t a, q15_t b) {
    return q15_clamp(a + b);
}
    
static inline q15_t q15_sub_sat(q15_t a, q15_t b) {
    return q15_clamp(a - b);
}

static inline q15_t q15_mul_sat(q15_t a, q15_t b) {
    return q15_clamp((q15_t)(((int32_t)a * b) >> Q15SHIFT));
}

static inline q15_t q15_div_sat(q15_t a, q15_t b) {
    if(b == 0) {
        if(a >= 0) {
            return Q15MAX;
        }
        return Q15MIN;
    }
    q15_t c = (q15_t)(((int32_t)a << Q15SHIFT) / b);
    return q15_clamp(c);
}

static inline q15_t q15_abs(q15_t x) {
    return x < 0 ? -x : x;
}

static inline q15_t q15_neg(q15_t x) {
    return -x;
}

static inline q15_t q15_abs_sat(q15_t x) {
    if(x == Q15MIN) {
        return Q15MAX;
    }
    return q15_abs(x);
}

static inline q15_t q15_neg_sat(q15_t x) {
    if(x == Q15MIN) {
        return Q15MAX;
    }
    return q15_neg(x);
}

static inline q15_t q15_rand() {
    return (q15_t)(rand() % 65536 - 32768);
}

static inline q15_t q15_exp_approx(q15_t x) {
    // Use Taylor: 1 + x + x²/2 + x³/6
    q15_t x1 = q15_clamp(x);
    q15_t x2 = q15_mul_sat(x1,x1);
    q15_t x3 = q15_mul_sat(x2,x1);

    q15_t result = Q15MAX + x1 + (x2 >> 1) + (x3 >> 3);
    return result;
}

static inline q15_t q15_log_approx(q15_t x) {
    q15_t x1 = q15_clamp(x);
    x1 = x1 - Q15MAX;
    q15_t x2 = q15_mul_sat(x1, x1);
    x2 = x2 >> 1;
    q15_t x3 = q15_mul_sat(x2, x1);
    x3 = x3/3;
    q15_t x4 = q15_mul_sat(x3, x1);
    x4 = x4 >> 2;
    q15_t result = x1 - x2 + x3 - x4;
    if (result > 0) {
        return Q15MIN;
    }
    return x1 - x2 + x3 - x4;
}

static inline q15_t q15_neglog_approx(q15_t x) {
    return q15_abs_sat(q15_log_approx(x));
}

static inline q15_t q15_sqrt_approx(q15_t x) {
    if (x <= 0) {
        return 0;
    }

    q15_t guess = x >> 1;
    for (int i = 0; i < 4; i++) {
        guess = (guess + (x << Q15SHIFT) / guess) >> 1;
    }
    return guess;
}

// -------------
// Q7 Operations
// -------------
static inline q7_t float_to_q7(float x) {
    if (x >= 1.0f) x = 0.992f;
    if (x <= -1.0f) x = -1.0f;
    return (q7_t)(x * 128.0f);
}

static inline float q7_to_float(q7_t x) {
    return (float)x / 128.0f;
}

static inline q7_t int_to_q7(int8_t x) {
    return (q7_t)(x<<Q7SHIFT);
}

static inline int8_t q7_to_int(q7_t x) {
    return (int8_t)(x>>Q7SHIFT);
}

static inline q7_t q7_add(q7_t a, q7_t b) {
    return a + b;
}

static inline q7_t q7_sub(q7_t a, q7_t b) {
    return a - b;
}

static inline q7_t q7_mul(q7_t a, q7_t b) {
    return (q7_t)(((int16_t)a * b) >> Q7SHIFT);
}

static inline q7_t q7_div(q7_t a, q7_t b) {
    if(b == 0) {
        if(a >= 0) {
            return Q7MAX;
        }
        else {
            return Q7MIN;
        }
    }
    return ((int16_t)a << Q7SHIFT)/b;
}

static inline q7_t q7_clamp(q7_t x) {
    if (x > Q7MAX) return Q7MAX;
    if (x < Q7MIN) return Q7MIN;
    return x;
}

static inline int16_t q7_clamp_int(int16_t x) {
    if (x > Q7MAX) return Q7MAX;
    if (x < Q7MIN) return Q7MIN;
    return x;
}

static inline q7_t q7_add_sat(q7_t a, q7_t b) {
    return q7_clamp(a + b);
}

static inline q7_t q7_sub_sat(q7_t a, q7_t b) {
    return q7_clamp(a - b);
}

static inline q7_t q7_mul_sat(q7_t a, q7_t b) {
    return (q7_t)q7_clamp_int((((int16_t)a * b) >> Q7SHIFT));
}

static inline q7_t q7_div_sat(q7_t a, q7_t b) {
    if(b == 0) {
        if(a >= 0) {
            return Q7MAX;
        }
        else {
            return Q7MIN;
        }
    }
    return q7_clamp(((int16_t)a << Q7SHIFT)/b);
}

static inline q7_t q7_abs(q7_t x) {
    return x < 0 ? -x : x;
}

static inline q7_t q7_neg(q7_t x) {
    return -x;
}

static inline q7_t q7_abs_sat(q7_t x) {
    if(x == Q7MIN) {
        return Q7MAX;
    }
    return q7_abs(x);
}

static inline q7_t q7_neg_sat(q7_t x) {
    if(x == Q7MIN) {
        return Q7MAX;
    }
    return q7_neg(x);
}

static inline q7_t q7_rand() {
    return (q7_t)(rand() % 256 - 128);
}

static inline q7_t q7_exp_approx(q7_t x) {
    // Use Taylor: 1 + x + x²/2 + x³/6
    q7_t x1 = q7_clamp(x);
    q7_t x2 = q7_mul_sat(x1,x1);
    q7_t x3 = q7_mul_sat(x2,x1);

    q7_t result = Q7MAX + x1 + (x2 >> 1) + (x3 >> 3);
    return result;
}

static inline q7_t q7_log_approx(q7_t x) {
    q7_t x1 = q7_clamp(x);
    x1 = x1 - Q7MAX;
    q7_t x2 = q7_mul_sat(x1, x1);
    x2 = x2 >> 1;
    q7_t x3 = q7_mul_sat(x2, x1);
    x3 = x3/3;
    q7_t x4 = q7_mul_sat(x3, x1);
    x4 = x4 >> 2;
    q7_t result = x1 - x2 + x3 - x4;
    if (result > 0) {
        return Q7MIN;
    }
    return x1 - x2 + x3 - x4;
}

static inline q7_t q7_neglog_approx(q7_t x) {
    return q7_abs_sat(q7_log_approx(x));
}

static inline q7_t q7_sqrt_approx(q7_t x) {
    if (x <= 0) {
        return 0;
    }

    q7_t guess = x >> 1;
    for (int i = 0; i < 4; i++) {
        guess = (guess + (x << Q7SHIFT) / guess) >> 1;
    }
    return guess;
}
#endif // TIELVE_FIXED_POINT_H
