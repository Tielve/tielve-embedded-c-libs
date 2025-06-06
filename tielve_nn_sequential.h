#ifndef TIELVE_NN_SEQUENTIAL_H
#define TIELVE_NN_SEQUENTIAL_H

#include "tielve_fixed_point.h"

typedef bool (*q15_layer_forward_fn)(void *layer, q15_mat_t *input, q15_mat_t *output);
typedef bool (*q7_layer_forward_fn)(void *layer, q7_mat_t *input, q7_mat_t *output);

typedef struct {
    void **layers;
    q15_layer_forward_fn *fns;
    uint16_t num_layers;
} q15_sequential_model_t;

typedef struct {
    void **layers;
    q7_layer_forward_fn *fns;
    uint16_t num_layers;
} q7_sequential_model_t;

// --------------------------
// Sequential Model Functions
// --------------------------
static inline bool q15_model_forward(q15_sequential_model_t *model, q15_mat_t *input, q15_mat_t *output) {
    static q15_mat_t intermediate1, intermediate2;
    q15_mat_t *cur_in = input;
    q15_mat_t *cur_out;

    for (uint16_t i = 0; i < model->num_layers; ++i) {
        if (i % 2 == 0) {
            cur_out = &intermediate1;
        } else {
            cur_out = &intermediate2;
        }

        if (!model->fns[i](model->layers[i], cur_in, cur_out)) {
            return false;
        }

        cur_in = cur_out;
    }

    *output = *cur_in;
    return true;
}

static inline bool q7_model_forward(q7_sequential_model_t *model, q7_mat_t *input, q7_mat_t *output) {
    static q7_mat_t intermediate1, intermediate2;
    q7_mat_t *cur_in = input;
    q7_mat_t *cur_out;

    for (uint16_t i = 0; i < model->num_layers; ++i) {
        if (i % 2 == 0) {
            cur_out = &intermediate1;
        } else {
            cur_out = &intermediate2;
        }

        if (!model->fns[i](model->layers[i], cur_in, cur_out)) {
            return false;
        }

        cur_in = cur_out;
    }

    *output = *cur_in;
    return true;
}

#endif
