//
// Created by Jonathan Tielve
//
#ifndef TIELVE_MATRIX_H
#define TIELVE_MATRIX_H

#include "tielve_fixed_point.h"
#include "tielve_activation.h"

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

static inline uint16_t mat_q15_index(const q15_mat_t *m, uint16_t row, uint16_t col) {
    return row * m->cols + col;
}

static inline uint16_t mat_q7_index(const q7_mat_t *m, uint16_t row, uint16_t col) {
    return row * m->cols + col;
}

#endif //TIELVE_MATRIX_H
