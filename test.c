//
// Created by Jonathan Tielve
//

#include "tielve_nn_regularization.h"
#include <stdio.h>
#include <math.h>

#define Q15MAX 32767
#define FLOAT_TO_Q15(x) ((int16_t)((x) * Q15MAX))
#define Q15_TO_FLOAT(x) ((float)(x) / Q15MAX)
void verify_batch_norm(int16_t* output, int batch_size, int num_features);

// Test data: 3 samples, 2 features each
void test_batch_norm() {
    printf("=== Batch Norm Test Case ===\n");

    // Input data in floating point for reference (scaled for Q15)
    float input_float[6] = {
        0.1f, 0.4f,  // Sample 0: [Feature 0, Feature 1]
        0.2f, 0.5f,  // Sample 1: [Feature 0, Feature 1]
        0.3f, 0.6f   // Sample 2: [Feature 0, Feature 1]
    };

    // Convert to Q15
    int16_t input_q15[6];
    for (int i = 0; i < 6; i++) {
        input_q15[i] = FLOAT_TO_Q15(input_float[i]);
    }

    printf("Input (Q15):\n");
    printf("Sample 0: %d, %d (%.3f, %.3f)\n", input_q15[0], input_q15[1],
           Q15_TO_FLOAT(input_q15[0]), Q15_TO_FLOAT(input_q15[1]));
    printf("Sample 1: %d, %d (%.3f, %.3f)\n", input_q15[2], input_q15[3],
           Q15_TO_FLOAT(input_q15[2]), Q15_TO_FLOAT(input_q15[3]));
    printf("Sample 2: %d, %d (%.3f, %.3f)\n", input_q15[4], input_q15[5],
           Q15_TO_FLOAT(input_q15[4]), Q15_TO_FLOAT(input_q15[5]));

    // Manual calculation for verification:
    // Feature 0 across samples: [0.1, 0.2, 0.3]
    // Mean = 0.2, Variance = ((0.1-0.2)² + (0.2-0.2)² + (0.3-0.2)²)/3 = 0.00667
    // Std = sqrt(0.00667) ≈ 0.0816

    // Feature 1 across samples: [0.4, 0.5, 0.6]
    // Mean = 0.5, Variance = 0.00667, Std ≈ 0.0816

    printf("\nExpected batch statistics:\n");
    printf("Feature 0: mean=0.200, std≈0.082\n");
    printf("Feature 1: mean=0.500, std≈0.082\n");

    // Expected normalized values:
    // Sample 0: [(0.1-0.2)/0.082, (0.4-0.5)/0.082] = [-1.22, -1.22]
    // Sample 1: [(0.2-0.2)/0.082, (0.5-0.5)/0.082] = [0.00, 0.00]
    // Sample 2: [(0.3-0.2)/0.082, (0.6-0.5)/0.082] = [1.22, 1.22]

    printf("\nExpected normalized output:\n");
    printf("Sample 0: [-1.22, -1.22]\n");
    printf("Sample 1: [0.00, 0.00]\n");
    printf("Sample 2: [1.22, 1.22]\n");

    // Parameters for testing
    int batch_size = 3;
    int num_features = 2;
    int16_t epsilon = 32;  // Small constant
    int16_t momentum = FLOAT_TO_Q15(0.9f);  // 0.9 for running averages

    // Initialize gamma (scale) to 1.0 and beta (shift) to 0.0
    int16_t gamma[2] = {Q15MAX, Q15MAX};  // [1.0, 1.0]
    int16_t beta[2] = {0, 0};             // [0.0, 0.0]

    // Initialize running statistics to zero
    int16_t running_mean[2] = {0, 0};
    int16_t running_var[2] = {Q15MAX, Q15MAX};  // Initialize to 1.0

    printf("\nTest parameters:\n");
    printf("Batch size: %d, Features: %d\n", batch_size, num_features);
    printf("Epsilon: %d (%.6f)\n", epsilon, Q15_TO_FLOAT(epsilon));
    printf("Momentum: %d (%.3f)\n", momentum, Q15_TO_FLOAT(momentum));
    printf("Gamma: [%.3f, %.3f]\n", Q15_TO_FLOAT(gamma[0]), Q15_TO_FLOAT(gamma[1]));
    printf("Beta: [%.3f, %.3f]\n", Q15_TO_FLOAT(beta[0]), Q15_TO_FLOAT(beta[1]));

    // Output arrays
    int16_t output_train[6];
    int16_t output_inference[6];

    printf("\n=== Test Training Mode ===\n");
    // TODO: Call your batch_norm_train_q15 function here
    q15_batch_norm_train(input_q15, output_train, batch_size, num_features, gamma, beta, running_mean, running_var, momentum, epsilon);
    printf("After training, running statistics should be updated\n");
    printf("Running mean should be close to: [%.3f, %.3f]\n",
           0.1 * Q15_TO_FLOAT(momentum), 0.1 * Q15_TO_FLOAT(momentum));

    printf("\n=== Test Inference Mode ===\n");
    // TODO: Call your batch_norm_inference_q15 function here
    q15_batch_norm_infer(input_q15, output_inference, batch_size, num_features, gamma, beta, running_mean, running_var, epsilon);
    verify_batch_norm(output_train, batch_size, num_features);
    verify_batch_norm(output_inference, batch_size, num_features);
    printf("Raw output values (Q15):\n");
    for (int i = 0; i < 6; i++) {
        printf("output[%d] = %d (%.3f)\n", i, output_train[i], Q15_TO_FLOAT(output_train[i]));
    }
    printf("\n=== Verification Steps ===\n");
    printf("1. Check training output matches expected normalized values\n");
    printf("2. Check running_mean and running_var were updated\n");
    printf("3. Test inference mode with the updated running statistics\n");
    printf("4. Verify each feature has mean≈0, std≈1 after normalization\n");
}

// Helper to verify batch norm results
void verify_batch_norm(int16_t* output, int batch_size, int num_features) {
    if (!output || batch_size <= 0 || num_features <= 0) {
        printf("Invalid parameters!\n");
        return;
    }

    printf("\n=== Verification ===\n");



    for (int feature = 0; feature < num_features; feature++) {
        // Calculate mean and std for this feature across batch
        int32_t sum = 0;
        for (int sample = 0; sample < batch_size; sample++) {
            sum += output[sample * num_features + feature];
        }
        float mean = Q15_TO_FLOAT(sum / batch_size);

        int32_t var_sum = 0;
        int16_t feat_mean = sum / batch_size;
        for (int sample = 0; sample < batch_size; sample++) {
            int16_t diff = output[sample * num_features + feature] - feat_mean;
            var_sum += (int32_t)diff * diff;
        }
        float variance = Q15_TO_FLOAT(var_sum / batch_size);
        float std_dev = sqrt(variance);

        printf("Feature %d: mean=%.6f (should be ~0), std=%.6f (should be ~1)\n",
               feature, mean, std_dev);
    }
}

int main() {
    test_batch_norm();
    return 0;
}