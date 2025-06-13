//
// Created by Jonathan Tielve
//

#ifndef TIELVE_NN_ACTIVATION_H
#define TIELVE_NN_ACTIVATION_H

#define Q15ALPHA 328/32768
#define Q7ALPHA 2/128

#include "../math/tielve_fixed_point.h"

static inline q15_t q15_relu(q15_t x) {
    x = q15_clamp(x);
    if (x > 0) {
        return x;
    } else {
        return 0;
    }
}

static inline q15_t q15_leaky_relu(q15_t x) {
    if (x < 0) {
        return (x * Q15ALPHA) >> 15;
    } else {
        return x;
    }
}

// sigmoid(x) ≈  .5 + x/4 - x^3/48
static inline q15_t q15_sigmoid(q15_t x) {
    x = q15_clamp(x);
    q15_t square = q15_mul(x, x);           // x^2
    q15_t cube = q15_mul(square, x);        // x^3
    q15_t linear = q15_mul(x, Q15MAX / 4);  // x*0.25 == x/4
    q15_t cubic  = q15_div(cube, 48);       // x^3/48
    q15_t y = q15_sub(linear, cubic);       // x*0.25 - x^3/48
    return q15_add(Q15MAX / 2, y);
}

// tanh(x) ≈ x - (x^3 / 3)
static inline q15_t q15_tanh(q15_t x) {
    x = q15_clamp(x);
    q15_t square = q15_mul(x, x);       // x^2
    q15_t cube = q15_mul(square, x);    // x^3
    q15_t y = q15_div(cube, 3);         // x^3/3
    return q15_sub(x, y);               // x - (x^3/3)
}

static inline void q15_softmax(const q15_t *input, q15_t *output, uint16_t length) {
    q15_t max_val = input[0];
    for (uint16_t i = 1; i < length; ++i) {
        if (input[i] > max_val) max_val = input[i];
    }

    int32_t sum = 0;
    int32_t exp_vals[32];

    for (uint16_t i = 0; i < length; ++i) {
        int16_t shifted = input[i] - max_val;
        exp_vals[i] = shifted > -8 * 256 ? (1 << 14) + ((shifted * (1 << 14)) >> 8) : 0;
        sum += exp_vals[i];
    }

    for (uint16_t i = 0; i < length; ++i) {
        output[i] = (q15_t)((exp_vals[i] << 15) / sum);
    }
}

static inline q7_t q7_relu(q7_t x) {
    if (x > 0) {
        return x;
    } else {
        return 0;
    }
}

static inline q7_t q7_leaky_relu(q7_t x) {
    if (x < 0) {
        return (x * Q7ALPHA) >> 15;
    } else {
        return x;
    }
}

// sigmoid(x) ≈  .5 + x/4 - x^3/48
static inline q7_t q7_sigmoid(q7_t x) {
    x = q7_clamp(x);
    q7_t square = q7_mul(x, x);         // x^2
    q7_t cube = q7_mul(square, x);      // x^3
    q7_t linear = q7_mul(x, Q7MAX / 4); // x*0.25
    q7_t cubic  = q7_div(cube, 48);     // x^3/48
    q7_t y = q7_sub(linear, cubic);     // x*0.25 - x^3/48
    return q7_add(Q7MAX / 2, y);
}

// tanh(x) ≈ x - (x^3 / 3)
static inline q7_t q7_tanh(q7_t x) {
    x = q7_clamp(x);
    q7_t square = q7_mul(x, x);     // x^2
    q7_t cube = q7_mul(square, x);  // x^3
    q7_t y = q7_div(cube, 3);       // x^3/3
    return q7_sub(x, y);            // x - x^3/3
}

static inline void q7_softmax(const q7_t *input, q7_t *output, uint16_t length) {
    q7_t max_val = input[0];
    for (uint16_t i = 1; i < length; ++i) {
        if (input[i] > max_val) max_val = input[i];
    }

    uint16_t i;
    int16_t sum = 0;
    int16_t exp_vals[32];

    for (i = 0; i < length; ++i) {
        int8_t diff = input[i] - max_val;
        exp_vals[i] = (diff > -8) ? (1 << 7) + (diff << 4) : 0;
        sum += exp_vals[i];
    }

    if (sum == 0) {
        sum = 1;
    }

    for (i = 0; i < length; ++i) {
        output[i] = (q7_t)((exp_vals[i] << 7) / sum);
    }
}

// --------------------
// Activation Functions
// --------------------
static inline void q15_mat_relu(q15_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q15_relu(A->data[i]);
    }
}

static inline void q7_mat_relu(q7_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q7_relu(A->data[i]);
    }
}

static inline void q15_mat_leaky_relu(q15_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q15_leaky_relu(A->data[i]);
    }
}

static inline void q7_mat_leaky_relu(q7_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q7_leaky_relu(A->data[i]);
    }
}

static inline void q15_mat_sigmoid(q15_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q15_sigmoid(A->data[i]);
    }
}

static inline void q7_mat_sigmoid(q7_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q7_sigmoid(A->data[i]);
    }
}

static inline void q15_mat_tanh(q15_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q15_tanh(A->data[i]);
    }
}

static inline void q7_mat_tanh(q7_mat_t *A) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q7_tanh(A->data[i]);
    }
}
#endif //TIELVE_ACTIVATION_H
