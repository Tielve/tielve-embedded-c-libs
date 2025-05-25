//
// Created by Jonathan Tielve
//
#ifndef TIELVE_ACTIVATION_H
#define TIELVE_ACTIVATION_H

#include "tielve_fixed_point.h"

static inline q15_t q15_relu(q15_t x) {
    x = q15_clamp(x);
    if (x > 0) {
        return x;
    } else {
        return 0;
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

static inline q7_t q7_relu(q7_t x) {
    if (x > 0) {
        return x;
    } else {
        return 0;
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

#endif //TIELVE_ACTIVATION_H
