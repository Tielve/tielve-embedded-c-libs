#ifndef TIELVE_NN_LOSS_H
#define TIELVE_NN_LOSS_H

#include "../math/tielve_fixed_point.h"

// --------------
// Loss Functions
// --------------
static inline q15_t q15_mse_loss(const q15_mat_t *predicted, const q15_mat_t *target) {
    if (!predicted || !target || predicted->rows != target->rows || predicted->cols != target->cols)
        return 0;

    int32_t sum = 0;
    uint32_t total_elements = predicted->rows * predicted->cols;

    for (uint32_t i = 0; i < total_elements; ++i) {
        q15_t diff = predicted->data[i] - target->data[i];
        int32_t sq = (diff * diff) >> 15;
        sum += sq;
    }

    return (q15_t)(sum / total_elements);
}

static inline q15_t q15_cross_entropy(q15_t *predictions, q15_t *targets, uint16_t length) {
    q31_t sum = 0;
    for (uint16_t i = 0; i < length; ++i) {
        if (targets[i] > 0) {
            q15_t log_p = q15_neglog_approx(predictions[i]);
            sum += targets[i] * log_p;
        }
    }
    return (q15_t)(sum >> 15);
}


static inline q15_t q15_softmax_cross_entropy(const q15_t *logits, uint16_t len, uint16_t target_index) {
    if (target_index >= len) {
        return Q15MAX;
    }

    q15_t max = Q15MIN;
    for (uint16_t i = 0; i < len; ++i) {
        if (logits[i] > max) max = logits[i];
    }

    q15_t exps[len];
    int32_t sum_exp = 0;
    for (uint16_t i = 0; i < len; ++i) {
        q15_t shifted = q15_clamp(logits[i] - max);
        exps[i] = q15_exp_approx(shifted);
        sum_exp = (int32_t)exps[i] + sum_exp;
    }

    if (sum_exp == 0) {
        return Q15MAX;
    }

    q15_t softmax = ((int32_t)exps[target_index] << Q15SHIFT) / sum_exp;
    return q15_neglog_approx(softmax);
}

static inline q7_t q7_mse_loss(const q7_mat_t *predicted, const q7_mat_t *target) {
    if (!predicted || !target || predicted->rows != target->rows || predicted->cols != target->cols)
        return 0;

    int16_t sum = 0;
    uint16_t total_elements = predicted->rows * predicted->cols;

    for (uint16_t i = 0; i < total_elements; ++i) {
        q7_t diff = predicted->data[i] - target->data[i];
        int16_t sq = (diff * diff) >> 7;
        sum += sq;
    }

    return (q7_t)(sum / total_elements);
}

static inline q7_t q7_cross_entropy(q7_t *predictions, q7_t *targets, uint16_t length) {
    q31_t sum = 0;
    for (uint16_t i = 0; i < length; ++i) {
        if (targets[i] > 0) {
            q7_t log_p = q7_neglog_approx(predictions[i]);
            sum += (q15_t)targets[i] * log_p;
        }
    }
    return (q7_t)(sum >> 7);
}

static inline q7_t q7_softmax_cross_entropy(const q7_t *logits, uint16_t len, uint16_t target_index) {
    if (target_index >= len) {
        return Q7MAX;
    }

    q7_t max = Q7MIN;
    for (uint16_t i = 0; i < len; ++i) {
        if (logits[i] > max) max = logits[i];
    }


    q7_t exps[len];
    int32_t sum_exp = 0;
    for (uint16_t i = 0; i < len; ++i) {
        q7_t shifted = q7_clamp(logits[i] - max);
        exps[i] = q7_exp_approx(shifted);
        sum_exp = (int32_t)exps[i] + sum_exp;
    }

    if (sum_exp == 0) {
        return Q7MAX;
    }

    q15_t softmax = ((int32_t)exps[target_index] << (Q7SHIFT + Q15SHIFT)) / sum_exp;
    return q15_neglog_approx(softmax);
}
#endif
