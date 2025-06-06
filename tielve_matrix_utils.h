//
// Created by Jonathan Tielve
//

#ifndef TIELVE_MATRIX_UTILS_H
#define TIELVE_MATRIX_UTILS_H

#include <stdlib.h>
#include <string.h>
#include "tielve_fixed_point.h"

// ------------------
// Creation and Free Functions
// ------------------
static inline q15_mat_t q15_mat_create(uint16_t rows, uint16_t cols)
{
    q15_mat_t mat;
    mat.rows = rows;
    mat.cols = cols;
    mat.data = (q15_t*)calloc(rows * cols, sizeof(q15_t));

    if (!mat.data) {
        mat.rows = 0;
        mat.cols = 0;
    }

    return mat;
}

static inline void q15_mat_free(q15_mat_t* mat)
{
    if (mat && mat->data) {
        free(mat->data);
        mat->data = NULL;
        mat->rows = 0;
        mat->cols = 0;
    }
}

static inline q7_mat_t q7_mat_create(uint16_t rows, uint16_t cols)
{
    q7_mat_t mat;
    mat.rows = rows;
    mat.cols = cols;
    mat.data = (q7_t*)calloc(rows * cols, sizeof(q7_t));

    if (!mat.data) {
        mat.rows = 0;
        mat.cols = 0;
    }

    return mat;
}

static inline void q7_mat_free(q7_mat_t* mat)
{
    if (mat && mat->data) {
        free(mat->data);
        mat->data = NULL;
        mat->rows = 0;
        mat->cols = 0;
    }
}

static inline void q15_mat_random(q15_mat_t *mat, int rows, int cols) {
    mat->data = (q15_t*)calloc(rows * cols, sizeof(q15_t));
    mat->rows = rows;
    mat->cols = cols;
    for (int i = 0; i < rows * cols; ++i) {
        mat->data[i] = q15_rand();
    }
}

static inline void q7_mat_random(q7_mat_t *mat, int rows, int cols) {
    mat->data = (q7_t*)calloc(rows * cols, sizeof(q7_t));
    mat->rows = rows;
    mat->cols = cols;
    for (int i = 0; i < rows * cols; ++i) {
        mat->data[i] = q7_rand();
    }
}

static inline void q15_mat_zero(q15_mat_t *mat, int rows, int cols) {
    mat->data = (q15_t*)calloc(rows * cols, sizeof(q15_t));
    mat->rows = rows;
    mat->cols = cols;
    for (int i = 0; i < rows * cols; ++i) {
        mat->data[i] = 0;
    }
}

static inline void q7_mat_zero(q7_mat_t *mat, int rows, int cols) {
    mat->data = (q7_t*)calloc(rows * cols, sizeof(q7_t));
    mat->rows = rows;
    mat->cols = cols;
    for (int i = 0; i < rows * cols; ++i) {
        mat->data[i] = 0;
    }
}

// -----------------------------
// "Setters & Getters" Functions
// -----------------------------
static inline uint16_t q15_mat_index(const q15_mat_t *m, uint16_t row, uint16_t col) {
    return row * m->cols + col;
}

static inline uint16_t q7_mat_index(const q7_mat_t *m, uint16_t row, uint16_t col) {
    return row * m->cols + col;
}

static inline q15_t q15_mat_get(const q15_mat_t* m, uint16_t row, uint16_t col)
{
    return m->data[q15_mat_index(m, row, col)];
}

static inline void q15_mat_set(q15_mat_t* m, uint16_t row, uint16_t col, q15_t value)
{
    m->data[q15_mat_index(m, row, col)] = value;
}

static inline q7_t q7_mat_get(const q7_mat_t* m, uint16_t row, uint16_t col) {
    return m->data[q7_mat_index(m, row, col)];
}

static inline void q7_mat_set(q7_mat_t* m, uint16_t row, uint16_t col, q7_t value)
{
    m->data[q7_mat_index(m, row, col)] = value;
}

// ----------------------
// Mathematical Functions
// ----------------------
static inline void q15_mat_add(const q15_mat_t *A, const q15_mat_t *B, q15_mat_t *C) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        C->data[i] = q15_add_sat(A->data[i], B->data[i]);
    }
}

static inline void q15_mat_sub(const q15_mat_t *A, const q15_mat_t *B, q15_mat_t *C) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        C->data[i] = q15_sub_sat(A->data[i], B->data[i]);
    }
}

static inline void q15_mat_mul(const q15_mat_t* A, const q15_mat_t* B, q15_mat_t* C) {
    for (uint16_t i = 0; i < A->rows; i++) {
        for (uint16_t j = 0; j < B->cols; j++) {
            int32_t sum = 0;
            for (uint16_t k = 0; k < B->cols; k++) {
                sum += (int32_t)q15_mat_get(A, i, k) * q15_mat_get(B, k, j);
            }
            q15_mat_set(C, i, j, q15_clamp((q15_t)(sum >> Q15SHIFT)));
        }
    }
}

static inline void q15_mat_scalar_mul(q15_mat_t *A, q15_t scalar) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q15_mul_sat(A->data[i], scalar);
    }
}

static inline q31_t q15_mat_sum(q15_mat_t *A) {
    q31_t sum = 0;
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        sum += (q31_t)q15_mat_get(A, i, 0);
    }
    return sum;
}

static inline void q7_mat_add(const q7_mat_t *A, const q7_mat_t *B, q7_mat_t *C) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        C->data[i] = q7_add_sat(A->data[i], B->data[i]);
    }
}

static inline void q7_mat_sub(const q7_mat_t *A, const q7_mat_t *B, q7_mat_t *C) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        C->data[i] = q7_sub_sat(A->data[i], B->data[i]);
    }
}

static inline void q7_mat_mul(const q7_mat_t* A, const q7_mat_t* B, q7_mat_t* C) {
    for (uint16_t i = 0; i < A->rows; i++) {
        for (uint16_t j = 0; j < B->cols; j++) {
            int32_t sum = 0;
            for (uint16_t k = 0; k < B->cols; k++) {
                sum += (int32_t)q7_mat_get(A, i, k) * q7_mat_get(B, k, j);
            }
            q7_mat_set(C, i, j, q7_clamp((q7_t)(sum >> Q7SHIFT)));
        }
    }
}

static inline void q7_mat_scalar_mul(q7_mat_t *A, q7_t scalar) {
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        A->data[i] = q7_mul_sat(A->data[i], scalar);
    }
}

static inline q15_t q7_mat_sum(const q7_mat_t *A) {
    q15_t sum = 0;
    for (uint16_t i = 0; i < A->rows * A->cols; i++) {
        sum += (q15_t)q7_mat_get(A, i, 0);
    }
    return sum;
}

// ------------------------
// Transformation Functions
// ------------------------
static inline void q15_mat_transpose(const q15_mat_t *A, q15_mat_t *B) {
    for (uint16_t i = 0; i < A->rows; ++i) {
        for (uint16_t j = 0; j < A->cols; ++j) {
            q15_mat_set(B, j, i, q15_mat_get(A, i, j));
        }
    }
}

static inline void q7_mat_transpose(const q7_mat_t *A, q7_mat_t *B) {
    for (uint16_t i = 0; i < A->rows; ++i) {
        for (uint16_t j = 0; j < A->cols; ++j) {
            q7_mat_set(B, j, i, q7_mat_get(A, i, j));
        }
    }
}

static inline bool q15_mat_inverse(const q15_mat_t *A, q15_mat_t *inv) {
    if (A->rows != A->cols) {
        return false;
    }

    uint16_t N = A->rows;

    for (uint16_t i = 0; i < N; ++i) {
        for (uint16_t j = 0; j < N; ++j) {
            q15_mat_set(inv, i, j, (i == j) ? Q15MAX : 0);
        }
    }

    q15_mat_t tmp = *A;
    tmp.data = malloc(N * N * sizeof(q15_t));
    if (!tmp.data) return false;
    memcpy(tmp.data, A->data, N * N * sizeof(q15_t));

    for (uint16_t i = 0; i < N; ++i) {
        q15_t pivot = q15_mat_get(&tmp, i, i);
        if (pivot == 0) {
            free(tmp.data);
            return false;
        }

        for (uint16_t j = 0; j < N; ++j) {
            q15_mat_set(&tmp, i, j, q15_div_sat(q15_mat_get(&tmp, i, j), pivot));
            q15_mat_set(inv, i, j, q15_div_sat(q15_mat_get(inv, i, j), pivot));
        }

        for (uint16_t k = 0; k < N; ++k) {
            if (k == i) continue;
            q15_t factor = q15_mat_get(&tmp, k, i);
            for (uint16_t j = 0; j < N; ++j) {
                q15_t val1 = q15_mat_get(&tmp, k, j);
                q15_t val2 = q15_mul_sat(factor, q15_mat_get(&tmp, i, j));
                q15_mat_set(&tmp, k, j, q15_sub_sat(val1, val2));

                val1 = q15_mat_get(inv, k, j);
                val2 = q15_mul_sat(factor, q15_mat_get(inv, i, j));
                q15_mat_set(inv, k, j, q15_sub_sat(val1, val2));
            }
        }
    }

    free(tmp.data);
    return true;
}

static inline bool q15_mat_solve(const q15_mat_t *A, const q15_mat_t *B, q15_mat_t *X) {
    if (A->rows != A->cols || B->cols != 1 || B->rows != A->rows) {
        return false;
    }

    uint16_t N = A->rows;

    q15_mat_t tmpA = *A;
    q15_mat_t tmpB = *B;

    tmpA.data = malloc(N * N * sizeof(q15_t));
    tmpB.data = malloc(N * sizeof(q15_t));
    if (!tmpA.data || !tmpB.data) return false;

    memcpy(tmpA.data, A->data, N * N * sizeof(q15_t));
    memcpy(tmpB.data, B->data, N * sizeof(q15_t));

    for (uint16_t i = 0; i < N; ++i) {
        q15_t pivot = q15_mat_get(&tmpA, i, i);
        if (pivot == 0) {
            free(tmpA.data); free(tmpB.data);
            return false;
        }

        for (uint16_t j = 0; j < N; ++j)
            q15_mat_set(&tmpA, i, j, q15_div(q15_mat_get(&tmpA, i, j), pivot));
        tmpB.data[i] = q15_div(tmpB.data[i], pivot);

        for (uint16_t k = 0; k < N; ++k) {
            if (k == i) continue;
            q15_t factor = q15_mat_get(&tmpA, k, i);
            for (uint16_t j = 0; j < N; ++j) {
                q15_t val = q15_sub_sat(q15_mat_get(&tmpA, k, j),
                                    q15_mul_sat(factor, q15_mat_get(&tmpA, i, j)));
                q15_mat_set(&tmpA, k, j, val);
            }
            tmpB.data[k] = q15_sub_sat(tmpB.data[k], q15_mul_sat(factor, tmpB.data[i]));
        }
    }

    for (uint16_t i = 0; i < N; ++i) {
        X->data[i] = tmpB.data[i];
    }
    free(tmpA.data);
    free(tmpB.data);
    return true;
}

static inline bool q7_mat_inverse(const q7_mat_t *A, q7_mat_t *inv) {
    if (A->rows != A->cols) {
        return false;
    }

    uint16_t N = A->rows;

    for (uint16_t i = 0; i < N; ++i) {
        for (uint16_t j = 0; j < N; ++j) {
            q7_mat_set(inv, i, j, (i == j) ? Q15MAX : 0);
        }
    }

    q7_mat_t tmp = *A;
    tmp.data = malloc(N * N * sizeof(q7_t));
    if (!tmp.data) return false;
    memcpy(tmp.data, A->data, N * N * sizeof(q7_t));

    for (uint16_t i = 0; i < N; ++i) {
        q7_t pivot = q7_mat_get(&tmp, i, i);
        if (pivot == 0) {
            free(tmp.data);
            return false;
        }

        for (uint16_t j = 0; j < N; ++j) {
            q7_mat_set(&tmp, i, j, q7_div_sat(q7_mat_get(&tmp, i, j), pivot));
            q7_mat_set(inv, i, j, q7_div_sat(q7_mat_get(inv, i, j), pivot));
        }

        for (uint16_t k = 0; k < N; ++k) {
            if (k == i) continue;
            q7_t factor = q7_mat_get(&tmp, k, i);
            for (uint16_t j = 0; j < N; ++j) {
                q7_t val1 = q7_mat_get(&tmp, k, j);
                q7_t val2 = q7_mul_sat(factor, q7_mat_get(&tmp, i, j));
                q7_mat_set(&tmp, k, j, q7_sub_sat(val1, val2));

                val1 = q7_mat_get(inv, k, j);
                val2 = q7_mul_sat(factor, q7_mat_get(inv, i, j));
                q7_mat_set(inv, k, j, q7_sub_sat(val1, val2));
            }
        }
    }

    free(tmp.data);
    return true;
}

static inline bool q7_mat_solve(const q7_mat_t *A, const q7_mat_t *B, q7_mat_t *X) {
    if (A->rows != A->cols || B->cols != 1 || B->rows != A->rows) {
        return false;
    }

    uint16_t N = A->rows;

    q7_mat_t tmpA = *A;
    q7_mat_t tmpB = *B;

    tmpA.data = malloc(N * N * sizeof(q7_t));
    tmpB.data = malloc(N * sizeof(q7_t));
    if (!tmpA.data || !tmpB.data) return false;

    memcpy(tmpA.data, A->data, N * N * sizeof(q7_t));
    memcpy(tmpB.data, B->data, N * sizeof(q7_t));

    for (uint16_t i = 0; i < N; ++i) {
        q7_t pivot = q7_mat_get(&tmpA, i, i);
        if (pivot == 0) {
            free(tmpA.data); free(tmpB.data);
            return false;
        }

        for (uint16_t j = 0; j < N; ++j)
            q7_mat_set(&tmpA, i, j, q7_div(q7_mat_get(&tmpA, i, j), pivot));
        tmpB.data[i] = q7_div(tmpB.data[i], pivot);

        for (uint16_t k = 0; k < N; ++k) {
            if (k == i) continue;
            q7_t factor = q7_mat_get(&tmpA, k, i);
            for (uint16_t j = 0; j < N; ++j) {
                q7_t val = q7_sub_sat(q7_mat_get(&tmpA, k, j),
                                    q7_mul_sat(factor, q7_mat_get(&tmpA, i, j)));
                q7_mat_set(&tmpA, k, j, val);
            }
            tmpB.data[k] = q7_sub_sat(tmpB.data[k], q7_mul_sat(factor, tmpB.data[i]));
        }
    }

    for (uint16_t i = 0; i < N; ++i) {
        X->data[i] = tmpB.data[i];
    }
    free(tmpA.data);
    free(tmpB.data);
    return true;
}

// ------
// Argmax
// ------
static inline uint16_t q15_argmax(const q15_t *vec, uint16_t length) {
    uint16_t index = 0;
    q15_t max_val = vec[0];

    for (uint16_t i = 1; i < length; ++i) {
        if (vec[i] > max_val) {
            max_val = vec[i];
            index = i;
        }
    }
    return index;
}

static inline uint16_t q7_argmax(const q7_t *vec, uint16_t length) {
    uint16_t index = 0;
    q7_t max_val = vec[0];

    for (uint16_t i = 1; i < length; ++i) {
        if (vec[i] > max_val) {
            max_val = vec[i];
            index = i;
        }
    }
    return index;
}


#endif //TIELVE_MATRIX_UTILS_H
