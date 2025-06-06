#ifndef TIELVE_NN_COMPUTATION_H
#define TIELVE_NN_COMPUTATION_H
#ifdef TRAINING_MODE
#include "nn_backprop.h"
#endif
#include "tielve_fixed_point.h"
#include "tielve_matrix_utils.h"

typedef struct {
    q15_mat_t weights;
    q15_mat_t bias;
    uint16_t input_size;
    uint16_t output_size;
} q15_dense_layer_t;

typedef struct {
    q7_mat_t weights;
    q7_mat_t bias;
    uint16_t input_size;
    uint16_t output_size;
} q7_dense_layer_t;

typedef struct {
    q15_mat_t kernel;
    q15_t bias;
    uint8_t stride;
    uint8_t padding;
} q15_conv2d_layer_t;

typedef struct {
    q7_mat_t kernel;
    q7_t bias;
    uint8_t stride;
    uint8_t padding;
} q7_conv2d_layer_t;

typedef struct {
    q15_mat_t *depthwise_kernels;
    q15_t *pointwise_kernels;
    q15_t *biases;
    uint8_t input_channels;
    uint8_t output_channels;
    uint8_t kernel_size;
    uint8_t stride;
    uint8_t padding;
} q15_dws_conv_layer_t;

typedef struct {
    q7_mat_t *depthwise_kernels;
    q7_t *pointwise_kernels;
    q7_t *biases;
    uint8_t input_channels;
    uint8_t output_channels;
    uint8_t kernel_size;
    uint8_t stride;
    uint8_t padding;
} q7_dws_conv_layer_t;

// ---------------------
// Dense Layer Functions
// ---------------------
static inline bool q15_dense_init(q15_dense_layer_t *layer, uint16_t input_size, uint16_t output_size, q15_t *weight_data, q15_t *bias_data) {
    layer->input_size = input_size;
    layer->output_size = output_size;

    layer->weights.rows = output_size;
    layer->weights.cols = input_size;
    layer->weights.data = weight_data;

    layer->bias.rows = output_size;
    layer->bias.cols = 1;
    layer->bias.data = bias_data;

    return true;
}

static inline bool q7_dense_init(q7_dense_layer_t *layer, uint16_t input_size, uint16_t output_size, q7_t *weight_data, q7_t *bias_data) {
    layer->input_size = input_size;
    layer->output_size = output_size;

    layer->weights.rows = output_size;
    layer->weights.cols = input_size;
    layer->weights.data = weight_data;

    layer->bias.rows = output_size;
    layer->bias.cols = 1;
    layer->bias.data = bias_data;

    return true;
}

static inline void q15_dense_random_init(q15_dense_layer_t *layer, int input_size, int output_size) {
    static q15_t weight_data[8192];
    static q15_t bias_data[64];

    layer->weights = (q15_mat_t){
        .rows = output_size,
        .cols = input_size,
        .data = weight_data
    };

    layer->bias = (q15_mat_t){
        .rows = output_size,
        .cols = 1,
        .data = bias_data
    };

    layer->input_size = input_size;
    layer->output_size = output_size;

    for (int i = 0; i < input_size * output_size; ++i) {
        weight_data[i] = q15_rand();
    }

    for (int i = 0; i < output_size; ++i) {
        bias_data[i] = q15_rand();
    }
}

static inline void q7_dense_random_init(q7_dense_layer_t *layer, int input_size, int output_size) {
    static q7_t weight_data[8192];
    static q7_t bias_data[64];

    layer->weights = (q7_mat_t){
        .rows = output_size,
        .cols = input_size,
        .data = weight_data
    };

    layer->bias = (q7_mat_t){
        .rows = output_size,
        .cols = 1,
        .data = bias_data
    };

    layer->input_size = input_size;
    layer->output_size = output_size;

    for (int i = 0; i < input_size * output_size; ++i) {
        weight_data[i] = q7_rand();
    }

    for (int i = 0; i < output_size; ++i) {
        bias_data[i] = q7_rand();
    }
}

static inline bool q15_dense_forward(const q15_dense_layer_t *layer, const q15_mat_t *input, q15_mat_t *output) {
    if (input->rows != layer->input_size || input->cols != 1 || output->rows != layer->output_size || output->cols != 1) {
        return false;
    }

    q15_mat_mul(&layer->weights, input, output);
    q15_mat_add(output, &layer->bias, output);

    return true;
}

static inline bool q7_dense_forward(const q7_dense_layer_t *layer, const q7_mat_t *input, q7_mat_t *output) {
    if (layer == NULL || input == NULL || input->rows != layer->input_size || input->cols != 1 || output->rows != layer->output_size || output->cols != 1) {
        return false;
    }

    q7_mat_mul(&layer->weights, input, output);
    q7_mat_add(output, &layer->bias, output);

    return true;
}

// ---------------------------
// Convolution Layer Functions
// ---------------------------
static inline void q15_conv2d_random_init(q15_conv2d_layer_t *layer, uint16_t kernel_size, uint8_t stride, uint8_t padding) {
    layer->padding = padding;
    layer->stride = stride;
    layer->kernel.rows = kernel_size;
    layer->kernel.cols = kernel_size;
    q15_mat_random(&layer->kernel, layer->kernel.rows, layer->kernel.cols);

    for (uint16_t i = 0; i < kernel_size * kernel_size; ++i) {
        layer->kernel.data[i] = rand() % 2049 - 1024;
    }
    layer->bias = q15_rand();
}

static inline void q7_conv2d_random_init(q7_conv2d_layer_t *layer, uint16_t kernel_size, uint8_t stride, uint8_t padding) {
    layer->padding = padding;
    layer->stride = stride;
    layer->kernel.rows = kernel_size;
    layer->kernel.cols = kernel_size;
    q7_mat_random(&layer->kernel, layer->kernel.rows, layer->kernel.cols);

    for (uint16_t i = 0; i < kernel_size * kernel_size; ++i) {
        layer->kernel.data[i] = rand() % 33 - 16;
    }
    layer->bias = q7_rand();
}

static inline bool q15_conv2d_forward(q15_conv2d_layer_t *layer, q15_mat_t *input, q15_mat_t *output) {
    int stride = layer->stride;
    int padding = layer->padding;

    int kernel_h = layer->kernel.rows;
    int kernel_w = layer->kernel.cols;
    int input_h = input->rows;
    int input_w = input->cols;

    int padded_h = input_h + 2 * padding;
    int padded_w = input_w + 2 * padding;
    int out_h = ((padded_h - kernel_h) / stride) + 1;
    int out_w = ((padded_w - kernel_w) / stride) + 1;

    if (output->rows != out_h || output->cols != out_w) {
        return false;
    }

    for (int i = 0; i < out_h; ++i) {
        for (int j = 0; j < out_w; ++j) {
            int32_t acc = 0;

            for (int m = 0; m < kernel_h; ++m) {
                for (int n = 0; n < kernel_w; ++n) {
                    int in_row = i * stride + m - padding;
                    int in_col = j * stride + n - padding;

                    q15_t in_val = 0;
                    if (in_row >= 0 && in_row < input_h && in_col >= 0 && in_col < input_w) {
                        in_val = input->data[in_row * input_w + in_col];
                    }

                    q15_t k_val = layer->kernel.data[m * kernel_w + n];
                    acc += (int32_t)in_val * k_val;
                }
            }

            acc += ((int32_t)layer->bias << 15);

            output->data[i * out_w + j] = q15_clamp(acc >> 15);
        }
    }

    return true;
}

static inline bool q7_conv2d_forward(q7_conv2d_layer_t *layer, q7_mat_t *input, q7_mat_t *output) {
    int stride = layer->stride;
    int padding = layer->padding;

    int kernel_h = layer->kernel.rows;
    int kernel_w = layer->kernel.cols;
    int input_h = input->rows;
    int input_w = input->cols;

    int padded_h = input_h + 2 * padding;
    int padded_w = input_w + 2 * padding;
    int out_h = ((padded_h - kernel_h) / stride) + 1;
    int out_w = ((padded_w - kernel_w) / stride) + 1;

    if (output->rows != out_h || output->cols != out_w) {
        return false;
    }

    for (int i = 0; i < out_h; ++i) {
        for (int j = 0; j < out_w; ++j) {
            int32_t acc = 0;

            for (int m = 0; m < kernel_h; ++m) {
                for (int n = 0; n < kernel_w; ++n) {
                    int in_row = i * stride + m - padding;
                    int in_col = j * stride + n - padding;

                    q7_t in_val = 0;
                    if (in_row >= 0 && in_row < input_h && in_col >= 0 && in_col < input_w) {
                        in_val = input->data[in_row * input_w + in_col];
                    }

                    q7_t k_val = layer->kernel.data[m * kernel_w + n];
                    acc += (int32_t)in_val * k_val;
                }
            }

            acc += ((int32_t)layer->bias << 15);

            output->data[i * out_w + j] = q7_clamp(acc >> 15);
        }
    }

    return true;
}

static inline bool q15_dws_conv2d_forward(q15_dws_conv_layer_t *layer, q15_mat_t **input_channels, q15_mat_t **output_channels) {
    uint8_t Cin = layer->input_channels;
    uint8_t Cout = layer->output_channels;
    uint8_t kernel = layer->kernel_size;
    uint8_t stride = layer->stride;
    uint8_t padding = layer->padding;

    int32_t output_rows = (input_channels[0]->rows - layer->kernel_size + 2 * padding) / stride + 1;
    int32_t output_cols = (input_channels[0]->cols - layer->kernel_size + 2 * padding) / stride + 1;

    q15_mat_t depthwise_outputs[Cin];
    q15_t depthwise_bufs[Cin][output_rows * output_cols];

    for (uint8_t c = 0; c < Cin; ++c) {
        depthwise_outputs[c].rows = output_rows;
        depthwise_outputs[c].cols = output_cols;
        depthwise_outputs[c].data = depthwise_bufs[c];

        if (!q15_conv2d_forward(&(q15_conv2d_layer_t){
            .kernel = layer->depthwise_kernels[c],
            .bias = 0,
            .stride = stride,
            .padding = padding
        }, input_channels[c], &depthwise_outputs[c])) {
            return false;
        }
    }

    uint16_t rows = depthwise_outputs[0].rows;
    uint16_t cols = depthwise_outputs[0].cols;

    for (uint8_t oc = 0; oc < Cout; ++oc) {
        q15_mat_t *out = output_channels[oc];
        if (out->rows != rows || out->cols != cols) return false;

        for (uint16_t i = 0; i < rows; ++i) {
            for (uint16_t j = 0; j < cols; ++j) {
                int32_t acc = 0;
                for (uint8_t ic = 0; ic < Cin; ++ic) {
                    q15_t val = depthwise_outputs[ic].data[i * cols + j];
                    q15_t weight = layer->pointwise_kernels[oc * Cin + ic];
                    acc += (int32_t)val * weight;
                }
                acc += ((int32_t)layer->biases[oc] << 15);
                out->data[i * cols + j] = q15_clamp(acc >> 15);
            }
        }
    }

    return true;
}

static inline bool q7_dws_conv2d_forward(q7_dws_conv_layer_t *layer, q7_mat_t **input_channels, q7_mat_t **output_channels) {
    uint8_t Cin = layer->input_channels;
    uint8_t Cout = layer->output_channels;
    uint8_t kernel = layer->kernel_size;
    uint8_t stride = layer->stride;
    uint8_t padding = layer->padding;

    int32_t output_rows = (input_channels[0]->rows - layer->kernel_size + 2 * padding) / stride + 1;
    int32_t output_cols = (input_channels[0]->cols - layer->kernel_size + 2 * padding) / stride + 1;

    q7_mat_t depthwise_outputs[Cin];
    q7_t depthwise_bufs[Cin][output_rows * output_cols];

    for (uint8_t c = 0; c < Cin; ++c) {
        depthwise_outputs[c].rows = output_rows;
        depthwise_outputs[c].cols = output_cols;
        depthwise_outputs[c].data = depthwise_bufs[c];

        if (!q7_conv2d_forward(&(q7_conv2d_layer_t){
            .kernel = layer->depthwise_kernels[c],
            .bias = 0,
            .stride = stride,
            .padding = padding
        }, input_channels[c], &depthwise_outputs[c])) {
            return false;
        }
    }

    uint16_t rows = depthwise_outputs[0].rows;
    uint16_t cols = depthwise_outputs[0].cols;

    for (uint8_t oc = 0; oc < Cout; ++oc) {
        q7_mat_t *out = output_channels[oc];
        if (out->rows != rows || out->cols != cols) return false;

        for (uint16_t i = 0; i < rows; ++i) {
            for (uint16_t j = 0; j < cols; ++j) {
                int32_t acc = 0;
                for (uint8_t ic = 0; ic < Cin; ++ic) {
                    q7_t val = depthwise_outputs[ic].data[i * cols + j];
                    q7_t weight = layer->pointwise_kernels[oc * Cin + ic];
                    acc += (int32_t)val * weight;
                }
                acc += ((int32_t)layer->biases[oc] << 15);
                out->data[i * cols + j] = q7_clamp(acc >> 15);
            }
        }
    }

    return true;
}

// -----------------
// Pooling Functions
// -----------------
static inline bool q15_max_pool2d(const q15_mat_t *input, q15_mat_t *output, uint8_t pool_size, uint8_t stride) {
    if (!input || !output || !input->data || !output->data) return false;

    for (uint16_t i = 0; i < output->rows; ++i) {
        for (uint16_t j = 0; j < output->cols; ++j) {
            int16_t max_val = INT16_MIN;

            for (uint8_t m = 0; m < pool_size; ++m) {
                for (uint8_t n = 0; n < pool_size; ++n) {
                    uint16_t r = i * stride + m;
                    uint16_t c = j * stride + n;
                    if (r < input->rows && c < input->cols) {
                        int16_t val = input->data[r * input->cols + c];
                        if (val > max_val) max_val = val;
                    }
                }
            }

            output->data[i * output->cols + j] = (q15_t)max_val;
        }
    }

    return true;
}

static inline bool q15_avg_pool2d(const q15_mat_t *input, q15_mat_t *output, uint8_t pool_size, uint8_t stride) {
    if (!input || !output || !input->data || !output->data) return false;

    for (uint16_t i = 0; i < output->rows; ++i) {
        for (uint16_t j = 0; j < output->cols; ++j) {
            int32_t sum = 0;
            uint16_t count = 0;

            for (uint8_t m = 0; m < pool_size; ++m) {
                for (uint8_t n = 0; n < pool_size; ++n) {
                    uint16_t r = i * stride + m;
                    uint16_t c = j * stride + n;
                    if (r < input->rows && c < input->cols) {
                        sum += input->data[r * input->cols + c];
                        count++;
                    }
                }
            }

            output->data[i * output->cols + j] = (q15_t)(sum / count);
        }
    }

    return true;
}

static inline bool q7_max_pool2d(const q7_mat_t *input, q7_mat_t *output, uint8_t pool_size, uint8_t stride) {
    if (!input || !output || !input->data || !output->data) return false;

    for (uint16_t i = 0; i < output->rows; ++i) {
        for (uint16_t j = 0; j < output->cols; ++j) {
            int16_t max_val = INT16_MIN;

            for (uint8_t m = 0; m < pool_size; ++m) {
                for (uint8_t n = 0; n < pool_size; ++n) {
                    uint16_t r = i * stride + m;
                    uint16_t c = j * stride + n;
                    if (r < input->rows && c < input->cols) {
                        int16_t val = input->data[r * input->cols + c];
                        if (val > max_val) max_val = val;
                    }
                }
            }

            output->data[i * output->cols + j] = (q7_t)max_val;
        }
    }

    return true;
}

static inline bool q7_avg_pool2d(const q7_mat_t *input, q7_mat_t *output, uint8_t pool_size, uint8_t stride) {
    if (!input || !output || !input->data || !output->data) return false;

    for (uint16_t i = 0; i < output->rows; ++i) {
        for (uint16_t j = 0; j < output->cols; ++j) {
            int32_t sum = 0;
            uint16_t count = 0;

            for (uint8_t m = 0; m < pool_size; ++m) {
                for (uint8_t n = 0; n < pool_size; ++n) {
                    uint16_t r = i * stride + m;
                    uint16_t c = j * stride + n;
                    if (r < input->rows && c < input->cols) {
                        sum += input->data[r * input->cols + c];
                        count++;
                    }
                }
            }

            output->data[i * output->cols + j] = (q7_t)(sum / count);
        }
    }

    return true;
}

#endif
