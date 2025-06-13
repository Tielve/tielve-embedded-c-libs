#ifndef TIELVE_NN_REGULARIZATION_H
#define TIELVE_NN_REGULARIZATION_H

#include <string.h>
#include "../math/tielve_fixed_point.h"
#include "../math/tielve_matrix_utils.h"

// ------------------------
// Regularization Functions
// ------------------------
static inline void q15_dropout(q15_t* input, q15_t* output, int size, float dropout_rate, bool training) {
    if (!training) {
        memcpy(output, input, size * sizeof(q15_t));
        return;
    }

    q15_t prob = (q15_t)((Q15MAX - dropout_rate) * Q15MAX);
    q15_t scale = Q15MAX/(Q15MAX - dropout_rate);

    for (int i = 0; i < size; i++) {
        if (q15_rand() < prob) {
            output[i] = (input[i] * scale) >> 15;
        } else {
            output[i] = 0;
        }
    }
}

static inline void q7_dropout(q7_t* input, q7_t* output, int size, float dropout_rate, bool training) {
    if (!training) {
        memcpy(output, input, size * sizeof(q7_t));
        return;
    }

    q7_t prob = (q7_t)((Q7MAX - dropout_rate) * Q7MAX);
    q7_t scale = Q7MAX/(Q7MAX-dropout_rate);

    for (int i = 0; i < size; i++) {
        if (q7_rand() < prob) {
            output[i] = (input[i] * scale) >> 15;
        } else {
            output[i] = 0;
        }
    }
}

static inline void q15_layer_norm(q15_t* input, q15_t* output, int batch_size, int features, q15_t* gamma, q15_t* beta, q15_t epsilon) {
    for (int sample = 0; sample < batch_size; sample++) {
        int32_t sum = 0;

        for (int f = 0; f < features; f++) {
            sum += input[sample * features + f];
        }

        q15_t mean = sum / features;
        int32_t variance_sum = 0;

        for (int f = 0; f < features; f++) {
            int16_t diff = input[sample * features + f] - mean;
            variance_sum += (int32_t)diff * diff;
        }

        q15_t variance = variance_sum / features;
        q15_t std_dev = q15_sqrt_approx(variance + epsilon);

        for (int f = 0; f < features; f++) {
            int16_t diff = input[sample * features + f] - mean;
            q15_t normalized = (diff << Q15SHIFT) / std_dev;
            if (gamma && beta) {
                output[sample * features + f] = ((int32_t)gamma[f] * normalized >> Q15SHIFT) + beta[f];
            } else {
                output[sample * features + f] = normalized;
            }
        }
    }
}

static inline void q7_layer_norm(q7_t* input, q7_t* output, int batch_size, int features, q7_t* gamma, q7_t* beta, q7_t epsilon) {
    for (int sample = 0; sample < batch_size; sample++) {
        int32_t sum = 0;

        for (int f = 0; f < features; f++) {
            sum += input[sample * features + f];
        }

        q7_t mean = sum / features;
        int32_t variance_sum = 0;

        for (int f = 0; f < features; f++) {
            int16_t diff = input[sample * features + f] - mean;
            variance_sum += (int32_t)diff * diff;
        }

        q7_t variance = variance_sum / features;
        q7_t std_dev = q7_sqrt_approx(variance + epsilon);

        for (int f = 0; f < features; f++) {
            int16_t diff = input[sample * features + f] - mean;
            q7_t normalized = (diff << Q7SHIFT) / std_dev;
            if (gamma && beta) {
                output[sample * features + f] = ((int16_t)gamma[f] * normalized >> Q7SHIFT) + beta[f];
            } else {
                output[sample * features + f] = normalized;
            }
        }
    }
}

static inline void q15_batch_norm_train(q15_t* input, q15_t* output, int batch_size, int num_features, q15_t* gamma, q15_t* beta, q15_t* running_mean, q15_t* running_var, q15_t momentum, q15_t epsilon){
    q15_t means[num_features];
    for (int f = 0; f < num_features; f++) {
        int32_t sum = 0;

        for (int sample = 0; sample < batch_size; sample++) {
            int idx = sample * num_features + f;
            sum += input[idx];
        }

        means[f] = sum / batch_size;
        q15_t variances[num_features];

        for (int i = 0; i < num_features; i++) {
            int32_t variance_sum = 0;
            q15_t mean = means[i];

            for (int sample = 0; sample < batch_size; sample++) {
                int idx = sample * num_features + i;
                int16_t diff = input[idx] - mean;
                variance_sum += (int32_t)diff * diff;
            }

            variances[i] = variance_sum / batch_size;
        }

        for (int sample = 0; sample < batch_size; sample++) {
            for (int feature = 0; feature < num_features; feature++) {
                int idx = sample * num_features + feature;

                q15_t mean = means[feature];
                q15_t variance = variances[feature];
                q15_t std_dev = q15_sqrt_approx(variance + epsilon);

                int16_t diff = input[idx] - mean;
                q15_t normalized = (diff << Q15SHIFT) / std_dev;

                if (gamma && beta) {
                    output[idx] = ((int32_t)gamma[feature] * normalized >> Q15SHIFT) + beta[feature];
                } else {
                    output[idx] = normalized;
                }
            }
        }

        for (int feature = 0; feature < num_features; feature++) {
            int32_t new_running_mean = ((int32_t)momentum * running_mean[feature] >> Q15SHIFT) +
                                       (((int32_t)(Q15MAX - momentum) * means[feature]) >> Q15SHIFT);
            running_mean[feature] = new_running_mean;

            int32_t new_running_var = ((int32_t)momentum * running_var[feature] >> Q15SHIFT) +
                                      (((int32_t)(Q15MAX - momentum) * variances[feature]) >> Q15SHIFT);
            running_var[feature] = new_running_var;
        }
    }
}

static inline void q15_batch_norm_infer(q15_t* input, q15_t* output, int batch_size, int num_features, q15_t* gamma, q15_t* beta, q15_t* running_mean, q15_t* running_var, q15_t epsilon){
    for (int sample = 0; sample < batch_size; sample++) {
        for (int feature = 0; feature < num_features; feature++) {
            int idx = sample * num_features + feature;

            q15_t mean = running_mean[feature];
            q15_t variance = running_var[feature];
            q15_t std_dev = q15_sqrt_approx(variance + epsilon);

            int16_t diff = input[idx] - mean;
            q15_t normalized = (diff << 15) / std_dev;

            if (gamma && beta) {
                output[idx] = ((int32_t)gamma[feature] * normalized >> 15) + beta[feature];
            } else {
                output[idx] = normalized;
            }
        }
    }
}

static inline void q15_batch_norm(bool training, q15_t* input, q15_t* output, int batch_size, int num_features, q15_t* gamma, q15_t* beta, q15_t* running_mean, q15_t* running_var, q15_t momentum, q15_t epsilon) {
    if (training) {
        q15_batch_norm_train(input, output, batch_size, num_features, gamma, beta, running_mean, running_var, momentum, epsilon);
    } else {
        q15_batch_norm_infer(input, output, batch_size, num_features, gamma, beta, running_mean, running_var, epsilon);
    }
}

static inline void q7_batch_norm_train(q7_t* input, q7_t* output, int batch_size, int num_features, q7_t* gamma, q7_t* beta, q7_t* running_mean, q7_t* running_var, q7_t momentum, q7_t epsilon){
    q7_t means[num_features];
    for (int f = 0; f < num_features; f++) {
        int32_t sum = 0;

        for (int sample = 0; sample < batch_size; sample++) {
            int idx = sample * num_features + f;
            sum += input[idx];
        }

        means[f] = sum / batch_size;
        q7_t variances[num_features];

        for (int i = 0; i < num_features; i++) {
            int32_t variance_sum = 0;
            q7_t mean = means[i];

            for (int sample = 0; sample < batch_size; sample++) {
                int idx = sample * num_features + i;
                int16_t diff = input[idx] - mean;
                variance_sum += (int32_t)diff * diff;
            }

            variances[i] = variance_sum / batch_size;
        }

        for (int sample = 0; sample < batch_size; sample++) {
            for (int feature = 0; feature < num_features; feature++) {
                int idx = sample * num_features + feature;

                q7_t mean = means[feature];
                q7_t variance = variances[feature];
                q7_t std_dev = q7_sqrt_approx(variance + epsilon);

                int16_t diff = input[idx] - mean;
                q7_t normalized = (diff << Q7SHIFT) / std_dev;

                if (gamma && beta) {
                    output[idx] = ((int32_t)gamma[feature] * normalized >> Q7SHIFT) + beta[feature];
                } else {
                    output[idx] = normalized;
                }
            }
        }

        for (int feature = 0; feature < num_features; feature++) {
            int32_t new_running_mean = ((int32_t)momentum * running_mean[feature] >> Q7SHIFT) + (((int32_t)(Q7MAX - momentum) * means[feature]) >> Q7SHIFT);
            running_mean[feature] = new_running_mean;

            int32_t new_running_var = ((int32_t)momentum * running_var[feature] >> Q7SHIFT) + (((int32_t)(Q7MAX - momentum) * variances[feature]) >> Q7SHIFT);
            running_var[feature] = new_running_var;
        }
    }
}

static inline void q7_batch_norm_infer(q7_t* input, q7_t* output, int batch_size, int num_features, q7_t* gamma, q7_t* beta, q7_t* running_mean, q7_t* running_var, q7_t epsilon){
    for (int sample = 0; sample < batch_size; sample++) {
        for (int feature = 0; feature < num_features; feature++) {
            int idx = sample * num_features + feature;

            q7_t mean = running_mean[feature];
            q7_t variance = running_var[feature];
            q7_t std_dev = q7_sqrt_approx(variance + epsilon);

            int16_t diff = input[idx] - mean;
            q7_t normalized = (diff << Q7SHIFT) / std_dev;

            if (gamma && beta) {
                output[idx] = ((int32_t)gamma[feature] * normalized >> Q7SHIFT) + beta[feature];
            } else {
                output[idx] = normalized;
            }
        }
    }
}

static inline void q7_batch_norm(bool training, q7_t* input, q7_t* output, int batch_size, int num_features, q7_t* gamma, q7_t* beta, q7_t* running_mean, q7_t* running_var, q7_t momentum, q7_t epsilon) {
    if (training) {
        q7_batch_norm_train(input, output, batch_size, num_features, gamma, beta, running_mean, running_var, momentum, epsilon);
    } else {
        q7_batch_norm_infer(input, output, batch_size, num_features, gamma, beta, running_mean, running_var, epsilon);
    }
}
#endif
