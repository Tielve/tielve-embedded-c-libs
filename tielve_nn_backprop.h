//
// Created by Jonathan Tielve
//

#ifndef TIELVE_NN_BACKPROP_H
#define TIELVE_NN_BACKPROP_H

#include "tielve_fixed_point.h"

static inline void q15_dense_backward(q15_t* input, q15_t* output_grad, q15_t* weights, q15_t* weight_grad, q15_t* bias_grad, q15_t* input_grad, int batch_size, int input_size, int output_size) {
    for (int i = 0; i < output_size; ++i) {
        int32_t sum = 0;
        for (int batch = 0; batch < batch_size; batch++) {
            sum += output_grad[batch * output_size + i];
        }
        bias_grad[i] = sum / batch_size;
    }

    for (int i = 0; i < input_size; ++i) {
        for (int j = 0; j < output_size; ++j) {
            int32_t sum = 0;
            for (int batch = 0; batch < batch_size; batch++) {
                int32_t product = (int32_t)input[batch * input_size + i] * output_grad[batch * output_size + j];
                sum += product >> Q15SHIFT;
            }
            weight_grad[i * output_size + j] = sum / batch_size;
        }
    }

    for (int i = 0; i < batch_size; ++i) {
        for (int j = 0; j < input_size; ++j) {
            int32_t sum = 0;
            for (int k = 0; k < output_size; ++k) {
                int32_t product = (int32_t)output_grad[i * output_size + k] * weights[j * output_size + k];
                sum += product >> Q15SHIFT;
            }
            input_grad[i * input_size + j] = sum;
        }
    }
}

static inline void q7_dense_backward(q7_t* input, q7_t* output_grad, q7_t* weights, q7_t* weight_grad, q7_t* bias_grad, q7_t* input_grad, int batch_size, int input_size, int output_size) {
    for (int i = 0; i < output_size; ++i) {
        int16_t sum = 0;
        for (int batch = 0; batch < batch_size; batch++) {
            sum += output_grad[batch * output_size + i];
        }
        bias_grad[i] = sum / batch_size;
    }

    for (int i = 0; i < input_size; ++i) {
        for (int j = 0; j < output_size; ++j) {
            int16_t sum = 0;
            for (int batch = 0; batch < batch_size; batch++) {
                int16_t product = (int16_t)input[batch * input_size + i] * output_grad[batch * output_size + j];
                sum += product >> Q7SHIFT;
            }
            weight_grad[i * output_size + j] = sum / batch_size;
        }
    }

    for (int i = 0; i < batch_size; ++i) {
        for (int j = 0; j < input_size; ++j) {
            int16_t sum = 0;
            for (int k = 0; k < output_size; ++k) {
                int16_t product = (int16_t)output_grad[i * output_size + k] * weights[j * output_size + k];
                sum += product >> Q7SHIFT;
            }
            input_grad[i * input_size + j] = sum;
        }
    }
}

static inline void q15_conv2d_backward(q15_t* input, q15_t* output_grad, q15_t* kernel, q15_t* kernel_grad, q15_t* bias_grad, q15_t* input_grad, int batch_size, int in_h, int in_w, int in_channels, int out_h, int out_w, int out_channels, int kernel_h, int kernel_w, int stride, int padding) {
    for (int out_ch = 0; out_ch < out_channels; out_ch++) {
        int32_t sum = 0;

        for (int batch = 0; batch < batch_size; batch++) {
            for (int y = 0; y < out_h; y++) {
                for (int x = 0; x < out_w; x++) {
                    int idx = batch * (out_h * out_w * out_channels) +
                             y * (out_w * out_channels) + x * out_channels + out_ch;
                    sum += output_grad[idx];
                }
            }
        }

        bias_grad[out_ch] = sum / (batch_size * out_h * out_w);
    }
    for (int out_ch = 0; out_ch < out_channels; out_ch++) {
        for (int in_ch = 0; in_ch < in_channels; in_ch++) {
            for (int ky = 0; ky < kernel_h; ky++) {
                for (int kx = 0; kx < kernel_w; kx++) {
                    int32_t sum = 0;

                    for (int batch = 0; batch < batch_size; batch++) {
                        for (int y = 0; y < out_h; y++) {
                            for (int x = 0; x < out_w; x++) {
                                int in_y = y * stride - padding + ky;
                                int in_x = x * stride - padding + kx;

                                if (in_y >= 0 && in_y < in_h && in_x >= 0 && in_x < in_w) {
                                    int input_idx = batch * (in_h * in_w * in_channels) +
                                                   in_y * (in_w * in_channels) +
                                                   in_x * in_channels + in_ch;
                                    int grad_idx = batch * (out_h * out_w * out_channels) +
                                                  y * (out_w * out_channels) +
                                                  x * out_channels + out_ch;

                                    int32_t product = (int32_t)input[input_idx] * output_grad[grad_idx];
                                    sum += product >> 15;
                                }
                            }
                        }
                    }

                    int kernel_idx = ky * (kernel_w * in_channels * out_channels) +
                                    kx * (in_channels * out_channels) +
                                    in_ch * out_channels + out_ch;
                    kernel_grad[kernel_idx] = sum / (batch_size * out_h * out_w);
                }
            }
        }
    }
    for (int i = 0; i < batch_size * in_h * in_w * in_channels; i++) {
    input_grad[i] = 0;
    }

    for (int batch = 0; batch < batch_size; batch++) {
        for (int in_ch = 0; in_ch < in_channels; in_ch++) {
            for (int in_y = 0; in_y < in_h; in_y++) {
                for (int in_x = 0; in_x < in_w; in_x++) {
                    int32_t sum = 0;

                    for (int out_ch = 0; out_ch < out_channels; out_ch++) {
                        for (int ky = 0; ky < kernel_h; ky++) {
                            for (int kx = 0; kx < kernel_w; kx++) {

                                int out_y = (in_y + padding - ky) / stride;
                                int out_x = (in_x + padding - kx) / stride;

                                if (out_y >= 0 && out_y < out_h && out_x >= 0 && out_x < out_w &&
                                    (in_y + padding - ky) % stride == 0 &&
                                    (in_x + padding - kx) % stride == 0) {

                                    int kernel_idx = ky * (kernel_w * in_channels * out_channels) +
                                                    kx * (in_channels * out_channels) +
                                                    in_ch * out_channels + out_ch;
                                    int grad_idx = batch * (out_h * out_w * out_channels) +
                                                  out_y * (out_w * out_channels) +
                                                  out_x * out_channels + out_ch;

                                    int32_t product = (int32_t)kernel[kernel_idx] * output_grad[grad_idx];
                                    sum += product >> 15;
                                }
                            }
                        }
                    }
                    
                    int input_idx = batch * (in_h * in_w * in_channels) +
                                   in_y * (in_w * in_channels) +
                                   in_x * in_channels + in_ch;
                    input_grad[input_idx] = sum;
                }
            }
        }
    }
}

static inline void q7_conv2d_backward(q7_t* input, q7_t* output_grad, q7_t* kernel, q7_t* kernel_grad, q7_t* bias_grad, q7_t* input_grad, int batch_size, int in_h, int in_w, int in_channels, int out_h, int out_w, int out_channels, int kernel_h, int kernel_w, int stride, int padding) {

}

static inline void q15_relu_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        if (input[i] > 0) {
            input_grad[i] = output_grad[i];
        } else {
            input_grad[i] = 0;
        }
    }
}

static inline void q7_relu_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        if (input[i] > 0) {
            input_grad[i] = output_grad[i];
        } else {
            input_grad[i] = 0;
        }
    }
}

static inline void q15_leaky_relu_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        if (input[i] > 0) {
            input_grad[i] = output_grad[i];
        } else {
            input_grad[i] = (output_grad[i] * alpha) >> 15;
        }
    }
}

static inline void q7_leaky_relu_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        if (input[i] > 0) {
            input_grad[i] = output_grad[i];
        } else {
            input_grad[i] = (output_grad[i] * alpha) >> 15;
        }
    }
}

static inline void q15_sigmoid_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        int32_t one_minus_output = Q15MAX - output_grad[i];
        int32_t derivative = ((int32_t)output_grad[i] * one_minus_output) >> 15;
        input_grad[i] = ((int32_t)output_grad[i] * derivative) >> 15;
    }
}

static inline void q7_sigmoid_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        int32_t one_minus_output = Q15MAX - output_grad[i];
        int32_t derivative = ((int32_t)output_grad[i] * one_minus_output) >> 15;
        input_grad[i] = ((int32_t)output_grad[i] * derivative) >> 15;
    }
}

static inline void q15_tanh_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        int32_t output_sq = ((int32_t)output_grad[i] * output_grad[i]) >> 15;
        q15_t derivative = Q15MAX - output_sq;
        input_grad[i] = ((int32_t)output_grad[i] * derivative) >> 15;
    }
}

static inline void q7_tanh_backward(q15_t* input, q15_t* output_grad, q15_t* input_grad, int size) {
    for (int i = 0; i < size; i++) {
        int32_t output_sq = ((int32_t)output_grad[i] * output_grad[i]) >> 15;
        q15_t derivative = Q15MAX - output_sq;
        input_grad[i] = ((int32_t)output_grad[i] * derivative) >> 15;
    }
}
#endif //TIELVE_NN_BACKPROP_H
