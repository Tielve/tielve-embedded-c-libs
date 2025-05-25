//
// Created by Jonathan Tielve
//

#ifndef TIELVE_FIXED_POINT_H
#define TIELVE_FIXED_POINT_H
#include <stdint.h>

typedef int16_t q15_t;
typedef int8_t  q7_t;

#define Q15SHIFT 15
#define Q7SHIFT 7
#define Q15MAX 32767
#define Q15MIN -32768
#define Q7MAX 127
#define Q7MIN -128

/*
 *
 * Q15 operations
 * --------------
 *
 */
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
    int32_t result = ((int32_t)a << Q15SHIFT) / b;
    return (q15_t)result;
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
    return (q15_t)q15_clamp_int((((int32_t)a * b) >> Q15SHIFT));
}

static inline q15_t q15_div_sat(q15_t a, q15_t b) {
    if(b == 0) {
        if(a >= 0) {
            return Q15MAX;
        }
        return Q15MIN;
    }
    return (q15_t)q15_clamp_int(((int32_t)a << Q15SHIFT) / b);
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

/*
 *
 * Q7 operations
 * -------------
 *
 */
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
    return (q7_t)(((int16_t)a << Q7SHIFT)/b);
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
    return (q7_t)q7_clamp_int(((int16_t)a << Q7SHIFT)/b);
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
#endif // TIELVE_FIXED_POINT_H
