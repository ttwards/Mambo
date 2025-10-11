/*
 * Copyright (c) 2024 BayLibre SAS
 * Copyright (c) 2024 Your Name/Company // Add your copyright
 * Copyright (c) 2024 [Your Name/Company for Test File] // Add your copyright for this test file
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h> // For memcmp
#include <stdbool.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/drivers/pwm.h>
#include "matrix_storage.h" // Include the header for the API under test
#include "zephyr/dsp/types.h"
#include "zephyr/kernel.h"
#include "zephyr/sys/time_units.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(matrix_storage_test, LOG_LEVEL_INF);

#define G 9.7887f

#define MAX_ACCEPTED_BIAS   0.28f
#define CACHED_SAMPLE_COUNT 500

// Helper function to print a matrix (optional, but useful for debugging)
void print_matrix(const char *name, const float matrix[MATRIX_ROWS][MATRIX_COLS])
{
	LOG_INF("%s:", name);
	for (int i = 0; i < MATRIX_ROWS; i++) {
		LOG_INF("  [ ");
		for (int j = 0; j < MATRIX_COLS; j++) {
			// Format nicely for readability
			LOG_INF("%8.3f ", (double)matrix[i][j]);
		}
		LOG_INF("]");
	}
}

#include <arm_math.h>
#include <stdio.h>  // For LOG_INF (optional, for debugging)
#include <stdlib.h> // For malloc/free (if using dynamic allocation)
#include <string.h> // For memcpy
#include <math.h>   // For fabs, sqrt
#include "devices.h"

#define NUM_PARAMS 6 // beta_0 to beta_5
#define ACCEL_AXES 3 // x, y, z

// --- User Configuration ---
#define MAX_ITERATIONS  500   // Maximum LM iterations
#define INITIAL_MU      1e-3f // Initial damping factor (can be tuned)
#define MU_FACTOR       10.0f // Factor to increase/decrease mu
#define ERROR_TOLERANCE 1e-6f // Convergence tolerance for error change
#define STEP_TOLERANCE  1e-6f // Convergence tolerance for parameter change norm

float32_t accel[3];            // Placeholder for accelerometer data
float32_t gyro[3];             // Placeholder for gyroscope data
float32_t gyro_sum[3];         // Placeholder for gyroscope data
float32_t current_temp = 0.0f; // Placeholder for temperature data

int gyro_sample_cnt = 0;

static const struct pwm_dt_spec pwm = PWM_DT_SPEC_GET(DT_CHOSEN(ares_pwm));
static float target_temp = 50.0f;
static float temp_pwm_output = 19900000.0f;

PID_NEW_INSTANCE(DT_NODELABEL(imu_temp_pid), ins)
struct pid_data *temp_pwm_pid = &PID_INS_NAME(DT_NODELABEL(imu_temp_pid), ins);

float IMU_temp_read(const struct device *dev)
{
	struct sensor_value temp;
	sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
	current_temp = sensor_value_to_double(&temp);
	return current_temp;
}

int IMU_temp_pwm_set(const struct device *dev)
{
#ifdef CONFIG_IMU_PWM_TEMP_CTRL
	pid_calc(temp_pwm_pid);
	return pwm_set_pulse_dt(&pwm, ((int)temp_pwm_output < 0) ? 0 : (int)temp_pwm_output);
#else
	return 0;
#endif // CONFIG_IMU_PWM_TEMP_CTRL
}
uint32_t current_time = 0;
uint32_t previous_time = 0;

bool read = false;
bool stable = false;
// struct JFData *data = NULL;

bool measure = true;
bool stop = true;

int last_press = 0;
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	if (k_cyc_to_ms_near32(k_cycle_get_32() - last_press) > 150 &&
	    k_cyc_to_ms_near32(k_cycle_get_32() - last_press) < 400) {
		measure = false;
		LOG_INF("Stop reading accelerometer data");
	}
	stop = !stop;
	last_press = k_cycle_get_32();
	if (stop) {
		LOG_INF("Pausing accelerometer data");
	}
	LOG_INF("Button pressed");
}

struct imu_info {
	void *fifo_reserved;
	float accel[3];
	float gyro[3];
};

K_FIFO_DEFINE(imu_fifo);
K_HEAP_DEFINE(imu_heap, 1080 * sizeof(struct imu_info));

void IMU_thread(void)
{
	int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure button GPIO: %d", ret);
		return;
	}
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d", ret,
		       button.port->name, button.pin);
		return;
	}
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	pid_reg_input(temp_pwm_pid, &current_temp, &target_temp);
	pid_reg_output(temp_pwm_pid, &temp_pwm_output);
	pid_reg_time(temp_pwm_pid, &current_time, &previous_time);
	// data = jf_send_init(uart_dev, 1);
	// jf_channel_add(data, &accel[0], PTR_FLOAT);
	// jf_channel_add(data, &accel[1], PTR_FLOAT);
	// jf_channel_add(data, &accel[2], PTR_FLOAT);
	float accel_norm = 0.0f;
	float gyro_norm = 0.0f;
	float accel_sum = 0.0f;
	int sample_cnt = 0;
	measure = true;
	while (1) {
		k_msleep(1);
		previous_time = current_time;
		current_time = k_cycle_get_32();

		sensor_sample_fetch(accel_dev);
		sensor_sample_fetch(gyro_dev);
		IMU_temp_read(accel_dev);
		IMU_temp_pwm_set(accel_dev);

		struct sensor_value accel_data[3];
		struct sensor_value gyro_data[3];
		if (!measure || fabsf(current_temp - target_temp) <= 1.5f) {
			sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_XYZ, accel_data);
			sensor_channel_get(gyro_dev, SENSOR_CHAN_GYRO_XYZ, gyro_data);

			accel[0] = sensor_value_to_float(&accel_data[0]);
			accel[1] = sensor_value_to_float(&accel_data[1]);
			accel[2] = sensor_value_to_float(&accel_data[2]);
			gyro[0] = sensor_value_to_float(&gyro_data[0]);
			gyro[1] = sensor_value_to_float(&gyro_data[1]);
			gyro[2] = sensor_value_to_float(&gyro_data[2]);
			gyro_norm =
				sqrtf(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]);
			accel_norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
					   accel[2] * accel[2]);
			accel_sum += accel_norm;
			sample_cnt++;
			if (measure && fabsf(accel_sum / sample_cnt - accel_norm) > 0.2f &&
			    fabsf(accel_norm - G) < MAX_ACCEPTED_BIAS && gyro_norm < 0.18f) {
				accel[0] = 0.0f;
				accel[1] = 0.0f;
				accel[2] = 0.0f;

				read = false;
				stable = false;
				sample_cnt = 0;
				accel_sum = 0.0f;

				LOG_INF("Accel and gyro not stable %f %f", (double)accel_norm,
					(double)gyro_norm);
			} else {
				stable = true;
			}
		} else {
			k_msleep(500);
			LOG_INF("Temperature not stable %f", (double)current_temp);
			sample_cnt = 0;
			stable = false;
			accel_norm = 0.0f;
			accel_sum = 0.0f;
			read = false;
		}
	}
}
K_THREAD_DEFINE(IMU_thread_id, 2048, IMU_thread, NULL, NULL, NULL, 0, 0, 0);

// --- Placeholder Function ---
/**
 * @brief User-defined function to acquire accelerometer data.
 * @param data_buffer Buffer to store the accelerometer readings.
 *                    Should be filled with [N][3] values:
 *                    data_buffer[0] = x0, data_buffer[1] = y0, data_buffer[2] = z0,
 *                    data_buffer[3] = x1, data_buffer[4] = y1, data_buffer[5] = z1, ...
 * @param num_samples The number of 3-axis samples (N) collected.
 * @return uint32_t The actual number of samples acquired (N).
 */
uint32_t acquire_accelerometer_data(float32_t *data_buffer, uint32_t num_samples_requested)
{
	// --- USER IMPLEMENTATION REQUIRED ---
	if (num_samples_requested < 6) {
		LOG_ERR("Need at least 6 samples for calibration");
		return 0; // Need at least a few orientations
	}

	// Should represent static measurements in different orientations
	// Format: {x0, y0, z0, x1, y1, z1, ...}
	float32_t sample_data[16 * ACCEL_AXES] = {0.0f};

	measure = true;
	int sample = 0;

	LOG_INF("Start reading accelerometer data");
	for (int i = 0; i < sizeof(sample_data) / sizeof(float32_t) / 3; i++) {
		int sample_cnt = -CACHED_SAMPLE_COUNT;
		while (stop) {
			k_msleep(1);
		}
		stop = true;
		int fifo_size = 0;

#define PURGE_FIFO                                                                                 \
	while (!k_fifo_is_empty(&imu_fifo)) {                                                      \
		struct imu_info *info = k_fifo_get(&imu_fifo, K_NO_WAIT);                          \
		k_heap_free(&imu_heap, info);                                                      \
		fifo_size = 0;                                                                     \
	}
		PURGE_FIFO

		while (1) {
			if (!measure) {
				LOG_INF("Stopping measurement");
				goto end;
			}
			if (sample_cnt > 0 && !read) {
				if (sample_cnt < 4000) {
					LOG_ERR("Unstable reading %d with %d samples", i,
						sample_cnt);
					sample_cnt = -CACHED_SAMPLE_COUNT;
					sample_data[i * 3] = 0.0f;
					sample_data[i * 3 + 1] = 0.0f;
					sample_data[i * 3 + 2] = 0.0f;
					PURGE_FIFO
					k_msleep(1000);
				} else {
					LOG_INF("End reading %d with %d samples", i, sample_cnt);
					sample_data[i * 3] /= sample_cnt;
					sample_data[i * 3 + 1] /= sample_cnt;
					sample_data[i * 3 + 2] /= sample_cnt;
					LOG_INF("Accel data: %f %f %f", (double)sample_data[i * 3],
						(double)sample_data[i * 3 + 1],
						(double)sample_data[i * 3 + 2]);
					float accel_norm = sqrtf(
						sample_data[i * 3] * sample_data[i * 3] +
						sample_data[i * 3 + 1] * sample_data[i * 3 + 1] +
						sample_data[i * 3 + 2] * sample_data[i * 3 + 2]);
					if (fabsf(accel_norm - G) > MAX_ACCEPTED_BIAS) {
						LOG_ERR("Accel data not stable %f",
							(double)accel_norm);
						sample_data[i * 3] = 0.0f;
						sample_data[i * 3 + 1] = 0.0f;
						sample_data[i * 3 + 2] = 0.0f;
						PURGE_FIFO
						k_msleep(1000);
					} else {
						LOG_INF("Accel data stable %f", (double)accel_norm);
					}
					break;
				}
			}
			if (stable) {
				if (sample_cnt % 1000 == 0) {
					LOG_INF("Reading %d with %d samples", i, sample_cnt);
				}
				if (fifo_size >= CACHED_SAMPLE_COUNT) {
					struct imu_info *imu_read =
						k_fifo_get(&imu_fifo, K_NO_WAIT);
					if (imu_read == NULL) {
						LOG_ERR("imu_read is NULL");
						continue;
					}
					if (sample_cnt >= 0) {
						sample_data[i * 3] += imu_read->accel[0];
						sample_data[i * 3 + 1] += imu_read->accel[1];
						sample_data[i * 3 + 2] += imu_read->accel[2];

						float accel_norm = sqrtf(
							imu_read->accel[0] * imu_read->accel[0] +
							imu_read->accel[1] * imu_read->accel[1] +
							imu_read->accel[2] * imu_read->accel[2]);
						if (fabsf(accel_norm - G) > MAX_ACCEPTED_BIAS) {
							read = false;
							stable = false;
						}

						gyro_sum[0] += imu_read->gyro[0];
						gyro_sum[1] += imu_read->gyro[1];
						gyro_sum[2] += imu_read->gyro[2];

						gyro_sample_cnt++;
					}
					k_heap_free(&imu_heap, imu_read);
					fifo_size--;
					if (sample_cnt++ == 1) {
						LOG_INF("Beginning reading %d", i);
					}
				}
				if (fifo_size == 0) {
					read = true;
				}

				struct imu_info *imu_data = k_heap_aligned_alloc(
					&imu_heap, 4, sizeof(struct imu_info), K_NO_WAIT);
				memcpy(&imu_data->accel[0], &accel[0], sizeof(accel));
				memcpy(&imu_data->gyro[0], &gyro[0], sizeof(gyro));
				k_fifo_put(&imu_fifo, imu_data);
				fifo_size++;
				// LOG_INF("Sampled %d samples", fifo_size);
			}
			k_msleep(1);
		}
		sample++;
	}
end:
	LOG_INF("End reading accelerometer data");
	LOG_INF("Sampled %d samples", sample);

	uint32_t count_to_copy = (num_samples_requested < sample) ? num_samples_requested : sample;

	if (data_buffer) {
		memcpy(data_buffer, sample_data, count_to_copy * ACCEL_AXES * sizeof(float32_t));
	}
	return count_to_copy;
	// --- End User Implementation Area ---
}

// --- Helper Function: Calculate Residuals and Jacobian ---
/**
 * @brief Calculates the residuals and the Jacobian matrix.
 * @param beta Current parameter vector [beta0..beta5].
 * @param accel_data Flattened accelerometer data [x0,y0,z0, x1,y1,z1, ...].
 * @param num_samples Number of accelerometer samples (N).
 * @param residuals_out Output buffer for residuals [r0, r1, ..., rN-1].
 * @param J_out CMSIS-DSP matrix instance for the Jacobian (N x NUM_PARAMS). Data buffer
 * must be pre-allocated.
 * @return arm_status ARM_MATH_SUCCESS if successful.
 */
arm_status calculate_residuals_and_jacobian(const float32_t beta[NUM_PARAMS],
					    const float32_t *accel_data, uint32_t num_samples,
					    float32_t *residuals_out,
					    arm_matrix_instance_f32 *J_out)
{
	if (!beta || !accel_data || !residuals_out || !J_out || !J_out->pData) {
		return ARM_MATH_ARGUMENT_ERROR;
	}
	if (J_out->numRows != num_samples || J_out->numCols != NUM_PARAMS) {
		return ARM_MATH_SIZE_MISMATCH;
	}

	float32_t *pJ = J_out->pData;

	for (uint32_t i = 0; i < num_samples; ++i) {
		// Get raw accelerometer data for this sample
		const float32_t *x_raw =
			&accel_data[i * ACCEL_AXES]; // x_raw[0]=xi0, x_raw[1]=xi1, x_raw[2]=xi2

		// Extract parameters for clarity
		float32_t b0 = beta[0], b1 = beta[1], b2 = beta[2]; // Offsets
		float32_t b3 = beta[3], b4 = beta[4],
			  b5 = beta[5]; // Scale factors (inverse scales in formula)

		// --- Check for division by zero ---
		// Scale factors should not be zero. Add small epsilon? Or handle error?
		// For simplicity, we'll assume they are non-zero here. A robust
		// implementation might add constraints or checks.
		if (fabsf(b3) < 1e-9f || fabsf(b4) < 1e-9f || fabsf(b5) < 1e-9f) {
			LOG_INF("Error: Scale factor near zero (b3, b4, or b5).");
			return ARM_MATH_SINGULAR; // Or a different error
		}

		// Calculate calibrated components and their squares
		float32_t term0 = (x_raw[0] - b0) / b3;
		float32_t term1 = (x_raw[1] - b1) / b4;
		float32_t term2 = (x_raw[2] - b2) / b5;

		float32_t term0_sq = term0 * term0;
		float32_t term1_sq = term1 * term1;
		float32_t term2_sq = term2 * term2;

		// Calculate residual r_i = g^2 - |A_beta(x_i)|^2
		residuals_out[i] = G * G - (term0_sq + term1_sq + term2_sq);

		// Calculate Jacobian elements (∂r_i / ∂β_j)
		// ∂r/∂β₀ = - ∂(|A|^2)/∂β₀ = - 2 * term0 * (-1/β₃) = 2 * term0 / b3
		// ∂r/∂β₁ = - ∂(|A|^2)/∂β₁ = - 2 * term1 * (-1/β₄) = 2 * term1 / b4
		// ∂r/∂β₂ = - ∂(|A|^2)/∂β₂ = - 2 * term2 * (-1/β₅) = 2 * term2 / b5
		// ∂r/∂β₃ = - ∂(|A|^2)/∂β₃ = - 2 * term0 * (-(x₀ - β₀)/β₃²) = 2 * term0 *
		// term0 / b3 = 2 * term0_sq / b3 ∂r/∂β₄ = - ∂(|A|^2)/∂β₄ = - 2 * term1 *
		// (-(x₁ - β₁)/β₄²) = 2
		// * term1 * term1 / b4 = 2 * term1_sq / b4 ∂r/∂β₅ = - ∂(|A|^2)/∂β₅ = - 2 *
		// term2 *
		// (-(x₂ - β₂)/β₅²) = 2 * term2 * term2 / b5 = 2 * term2_sq / b5

		uint32_t row_offset = i * NUM_PARAMS;
		pJ[row_offset + 0] = 2.0f * term0 / b3;
		pJ[row_offset + 1] = 2.0f * term1 / b4;
		pJ[row_offset + 2] = 2.0f * term2 / b5;
		pJ[row_offset + 3] = 2.0f * term0_sq / b3;
		pJ[row_offset + 4] = 2.0f * term1_sq / b4;
		pJ[row_offset + 5] = 2.0f * term2_sq / b5;
	}
	return ARM_MATH_SUCCESS;
}

// --- Helper Function: Calculate Sum of Squared Errors ---
float32_t calculate_sum_sq_error(const float32_t *residuals, uint32_t num_samples)
{
	// float32_t sum_sq = 0.0f;
	// for (uint32_t i = 0; i < num_samples; ++i) {
	// 	sum_sq += residuals[i] * residuals[i];
	// }
	// return sum_sq;
	// CMSIS-DSP equivalent (though a loop is fine):
	float32_t sum_sq;
	// arm_power_f32(residuals, num_samples, &sum_sq); // Actually calculates sum(x^2),
	// need arm_dot_prod?
	arm_dot_prod_f32(residuals, residuals, num_samples, &sum_sq);
	return sum_sq;
}

// --- Helper Function: Calculate Norm of a Vector ---
float32_t calculate_vector_norm(const float32_t *vector, uint32_t size)
{
	float32_t norm_sq = 0.0f;
	arm_dot_prod_f32(vector, vector, size, &norm_sq);
	return sqrtf(norm_sq);
}

// --- Main Levenberg-Marquardt Optimization Function ---
/**
 * @brief Performs Levenberg-Marquardt optimization for accelerometer calibration.
 *
 * @param initial_beta Initial guess for parameters [beta0..beta5]. Content will be
 * modified.
 * @param num_samples Number of accelerometer samples (N).
 * @param max_mem_bytes Maximum memory (in bytes) available for dynamic allocation of
 * matrices. Adjust based on target platform memory constraints. If 0, uses stack allocation
 * assuming N is reasonably small (e.g., < 50). Stack allocation is untested here.
 * @param final_beta Output buffer where the optimized parameters will be stored.
 * @return arm_status ARM_MATH_SUCCESS on convergence, ARM_MATH_ARGUMENT_ERROR if max
 * iterations reached, or other error codes (e.g., ARM_MATH_SINGULAR, ARM_MATH_MEMORY).
 */
arm_status lm_optimize_accel_calibration(
	float32_t initial_beta[NUM_PARAMS], uint32_t num_samples_max,
	uint32_t max_mem_bytes, // For dynamic allocation, 0 for potential stack allocation
	float32_t final_beta[NUM_PARAMS])
{
	arm_status status = ARM_MATH_SUCCESS;

// 1. Acquire Data
// Allocate buffer for data (use max_mem or stack?)
// For simplicity, let's assume max_mem allows for data + matrices, or use a fixed max N.
// Let's assume a fixed max N for data buffer for now if max_mem is tricky.
#define MAX_SAMPLES_FOR_BUFFER 100 // Example limit
	if (num_samples_max > MAX_SAMPLES_FOR_BUFFER) {
		LOG_INF("Error: num_samples_max exceeds internal buffer limit.");
		return ARM_MATH_ARGUMENT_ERROR;
	}
	float32_t accel_data_buffer[MAX_SAMPLES_FOR_BUFFER * ACCEL_AXES];
	uint32_t N = acquire_accelerometer_data(accel_data_buffer, num_samples_max);
	if (N == 0) {
		LOG_INF("Error: Failed to acquire accelerometer data.");
		return ARM_MATH_ARGUMENT_ERROR; // Or specific error
	}
	if (N < NUM_PARAMS) {
		LOG_INF("Error: Need at least %d samples for %d parameters.", NUM_PARAMS,
			NUM_PARAMS);
		return ARM_MATH_ARGUMENT_ERROR;
	}
	LOG_INF("Acquired %u accelerometer samples.", N);

	// 2. Allocate Memory for Matrices and Vectors
	// Sizes:
	// beta: NUM_PARAMS
	// residuals: N
	// J: N x NUM_PARAMS
	// J_T: NUM_PARAMS x N
	// JtJ: NUM_PARAMS x NUM_PARAMS
	// Aug_JtJ: NUM_PARAMS x NUM_PARAMS
	// Inv_Aug_JtJ: NUM_PARAMS x NUM_PARAMS
	// Jtr: NUM_PARAMS
	// delta: NUM_PARAMS
	// temp_beta: NUM_PARAMS
	// temp_residuals: N

	// Total float elements approx: N + N*P + P*N + P*P + P*P + P*P + P + P + P + N
	// = 2N + 2NP + 3P^2 + 3P  (where P=NUM_PARAMS=6)
	// = 2N + 12N + 3*36 + 3*6 = 14N + 108 + 18 = 14N + 126
	// If N=50, size = 14*50 + 126 = 700 + 126 = 826 floats => ~3.3 kB

	// --- Using dynamic allocation (adjust based on max_mem_bytes) ---
	// Note: On embedded systems without malloc, use static allocation.
	//       Requires knowing max N beforehand.
	float32_t *residuals_vec = (float32_t *)malloc(N * sizeof(float32_t));
	float32_t *jacobian_data = (float32_t *)malloc(N * NUM_PARAMS * sizeof(float32_t));
	float32_t *jacobian_T_data = (float32_t *)malloc(NUM_PARAMS * N * sizeof(float32_t));
	float32_t *JtJ_data = (float32_t *)malloc(NUM_PARAMS * NUM_PARAMS * sizeof(float32_t));
	float32_t *Aug_JtJ_data = (float32_t *)malloc(NUM_PARAMS * NUM_PARAMS * sizeof(float32_t));
	float32_t *Inv_Aug_JtJ_data =
		(float32_t *)malloc(NUM_PARAMS * NUM_PARAMS * sizeof(float32_t));
	float32_t *Jtr_vec = (float32_t *)malloc(NUM_PARAMS * sizeof(float32_t));   // J' * r
	float32_t *delta_vec = (float32_t *)malloc(NUM_PARAMS * sizeof(float32_t)); // Update step
	float32_t *current_beta = (float32_t *)malloc(NUM_PARAMS * sizeof(float32_t));
	float32_t *temp_beta = (float32_t *)malloc(NUM_PARAMS * sizeof(float32_t));
	float32_t *temp_residuals_vec = (float32_t *)malloc(N * sizeof(float32_t));

	if (!residuals_vec || !jacobian_data || !jacobian_T_data || !JtJ_data || !Aug_JtJ_data ||
	    !Inv_Aug_JtJ_data || !Jtr_vec || !delta_vec || !current_beta || !temp_beta ||
	    !temp_residuals_vec) {
		LOG_INF("Error: Memory allocation failed.");
		// Free allocated memory before returning
		free(residuals_vec);
		free(jacobian_data);
		free(jacobian_T_data);
		free(JtJ_data);
		free(Aug_JtJ_data);
		free(Inv_Aug_JtJ_data);
		free(Jtr_vec);
		free(delta_vec);
		free(current_beta);
		free(temp_beta);
		free(temp_residuals_vec);
		return ARM_MATH_ARGUMENT_ERROR;
	}

	// Initialize beta
	memcpy(current_beta, initial_beta, NUM_PARAMS * sizeof(float32_t));

	// Initialize CMSIS Matrix Instances
	arm_matrix_instance_f32 J_mat, J_T_mat, JtJ_mat, Aug_JtJ_mat, Inv_Aug_JtJ_mat;
	arm_matrix_instance_f32 Jtr_mat,
		delta_mat;                     // Treat vectors as Nx1 matrices for mult
	arm_matrix_instance_f32 residuals_mat; // N x 1 matrix for J'*r calculation

	arm_mat_init_f32(&J_mat, N, NUM_PARAMS, jacobian_data);
	arm_mat_init_f32(&J_T_mat, NUM_PARAMS, N, jacobian_T_data);
	arm_mat_init_f32(&JtJ_mat, NUM_PARAMS, NUM_PARAMS, JtJ_data);
	arm_mat_init_f32(&Aug_JtJ_mat, NUM_PARAMS, NUM_PARAMS, Aug_JtJ_data);
	arm_mat_init_f32(&Inv_Aug_JtJ_mat, NUM_PARAMS, NUM_PARAMS, Inv_Aug_JtJ_data);

	// Treat vectors as matrices for multiplication:
	arm_mat_init_f32(&residuals_mat, N, 1,
			 residuals_vec); // Use the main residuals vector
	arm_mat_init_f32(&Jtr_mat, NUM_PARAMS, 1, Jtr_vec);
	arm_mat_init_f32(&delta_mat, NUM_PARAMS, 1, delta_vec);

	// 3. LM Iteration Loop
	float32_t current_error, prev_error, delta_norm;
	float32_t mu = INITIAL_MU; // Damping factor
	uint32_t iter = 0;
	int step_accepted = 0;

	// Calculate initial error
	status = calculate_residuals_and_jacobian(current_beta, accel_data_buffer, N, residuals_vec,
						  &J_mat);
	if (status != ARM_MATH_SUCCESS) {
		goto cleanup_and_exit;
	}
	current_error = calculate_sum_sq_error(residuals_vec, N);
	prev_error = current_error; // Initialize prev_error

	LOG_INF("Initial Beta: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", (double)current_beta[0],
		(double)current_beta[1], (double)current_beta[2], (double)current_beta[3],
		(double)current_beta[4], (double)current_beta[5]);
	LOG_INF("Iter 0: Error = %.6e", (double)current_error);

	while (iter < MAX_ITERATIONS) {
		iter++;
		step_accepted = 0;

		// Calculate Jacobian (if not already done) and J'*r
		if (iter > 1) { // Recalculate J and r for the new beta
			status = calculate_residuals_and_jacobian(current_beta, accel_data_buffer,
								  N, residuals_vec, &J_mat);
			if (status != ARM_MATH_SUCCESS) {
				LOG_INF("Iteration %u: Jacobian/Residual calculation failed "
					"(Status "
					"%d).",
					iter, status);
				goto cleanup_and_exit;
			}
			// Update error for convergence check later (already have residuals)
			current_error = calculate_sum_sq_error(residuals_vec, N);
		}

		// Calculate J'
		status = arm_mat_trans_f32(&J_mat, &J_T_mat);
		if (status != ARM_MATH_SUCCESS) {
			LOG_INF("Iter %u: J Transpose failed.", iter);
			goto cleanup_and_exit;
		}

		// Calculate J'*r (RHS of the normal equation)
		// Ensure residuals_mat points to the correct data (already does)
		status = arm_mat_mult_f32(&J_T_mat, &residuals_mat, &Jtr_mat);
		if (status != ARM_MATH_SUCCESS) {
			LOG_INF("Iter %u: J'*r multiplication failed.", iter);
			goto cleanup_and_exit;
		}

		// Calculate J'*J (Approx Hessian / 2)
		status = arm_mat_mult_f32(&J_T_mat, &J_mat, &JtJ_mat);
		if (status != ARM_MATH_SUCCESS) {
			LOG_INF("Iter %u: J'*J multiplication failed.", iter);
			goto cleanup_and_exit;
		}

		// --- Inner loop for adjusting mu ---
		while (1) { // Loop until a step is accepted or mu gets too large (add
			    // safety break?)
			// Create Augmented Matrix: Aug = J'*J + mu*I
			// Copy JtJ to Aug_JtJ first
			memcpy(Aug_JtJ_data, JtJ_data, NUM_PARAMS * NUM_PARAMS * sizeof(float32_t));
			// Add mu to diagonal elements
			for (int k = 0; k < NUM_PARAMS; ++k) {
				Aug_JtJ_data[k * NUM_PARAMS + k] += mu;
			}

			// Try to invert the augmented matrix: Inv = inv(Aug)
			status = arm_mat_inverse_f32(&Aug_JtJ_mat, &Inv_Aug_JtJ_mat);

			if (status == ARM_MATH_SUCCESS) {
				// Calculate step: delta = Inv * (J'*r)
				status = arm_mat_mult_f32(&Inv_Aug_JtJ_mat, &Jtr_mat, &delta_mat);
				if (status != ARM_MATH_SUCCESS) {
					LOG_INF("Iter %u: Delta calculation failed.", iter);
					goto cleanup_and_exit;
				} // Should not fail if inverse succeeded

				// Calculate proposed new beta: temp_beta = current_beta -
				// delta Note the '-' sign based on the derivation in the
				// image (Eq 9, 11, 16)
				for (int k = 0; k < NUM_PARAMS; ++k) {
					temp_beta[k] = current_beta[k] - delta_vec[k];
				}

				// Calculate error with the proposed temp_beta
				// Need temporary residual calculation without modifying J
				arm_status temp_status = calculate_residuals_and_jacobian(
					temp_beta, accel_data_buffer, N, temp_residuals_vec,
					&J_mat); // J content will be overwritten but we
						 // need it only for residual calculation
						 // here. A dedicated residual func would be
						 // better.
				if (temp_status != ARM_MATH_SUCCESS) {
					// If temp_beta leads to invalid state (e.g., scale
					// factor zero), reject step
					LOG_INF("Iter %u: Temp residual calculation failed "
						"(likely "
						"bad step). Increasing mu.",
						iter);
					mu *= MU_FACTOR;
					continue; // Try with larger mu
				}
				float32_t temp_error =
					calculate_sum_sq_error(temp_residuals_vec, N);

				// Check if the step reduces the error
				if (temp_error < current_error) {
					// Step accepted!
					step_accepted = 1;
					memcpy(current_beta, temp_beta,
					       NUM_PARAMS * sizeof(float32_t)); // Update beta
					prev_error = current_error; // Store error before update
					current_error = temp_error; // Update current error
					mu /= MU_FACTOR; // Decrease mu for next iteration
					LOG_INF("Iter %u: Step ACCEPTED. Error = %.6e -> "
						"%.6e, mu = "
						"%.2e",
						iter, (double)prev_error, (double)current_error,
						(double)mu);
					break; // Exit mu adjustment loop
				} else {
					// Step rejected, increase mu and try again
					mu *= MU_FACTOR;
					LOG_INF("Iter %u: Step REJECTED. Error = %.6e >= "
						"%.6e, "
						"Increasing mu = %.2e",
						iter, (double)temp_error, (double)current_error,
						(double)mu);
					// Add safety break?
					if (mu > 1e10f) {
						LOG_INF("Iter %u: Mu too large, possibly "
							"stuck.",
							iter);
						status = ARM_MATH_SUCCESS;
						goto cleanup_and_exit;
					}
				}
			} else {
				// Matrix inversion failed (singular or ill-conditioned)
				LOG_INF("Iter %u: Matrix inversion failed (Status %d). "
					"Increasing "
					"mu = %.2e",
					iter, status, (double)(mu * MU_FACTOR));
				mu *= MU_FACTOR;
				// Add safety break?
				if (mu > 1e10f) {
					LOG_INF("Iter %u: Mu too large after inversion "
						"failure.",
						iter);
					status = ARM_MATH_ARGUMENT_ERROR;
					goto cleanup_and_exit;
				}
			}
		} // End of mu adjustment loop

		// Check for convergence
		delta_norm = calculate_vector_norm(delta_vec, NUM_PARAMS);
		LOG_INF("Iter %u: Delta Norm = %.4e, Error Change = %.4e", iter, (double)delta_norm,
			(double)fabsf(current_error - prev_error));

		if (fabsf(current_error - prev_error) < ERROR_TOLERANCE &&
		    delta_norm < STEP_TOLERANCE) {
			LOG_INF("Convergence reached after %u iterations.", iter);
			status = ARM_MATH_SUCCESS;
			break; // Exit main loop
		}
		if (current_error < ERROR_TOLERANCE * ERROR_TOLERANCE) { // Check absolute error too
			LOG_INF("Convergence reached (absolute error small) after %u "
				"iterations.",
				iter);
			status = ARM_MATH_SUCCESS;
			break;
		}

	} // End of main iteration loop

	if (iter >= MAX_ITERATIONS) {
		LOG_INF("Maximum iterations (%d) reached without convergence.", MAX_ITERATIONS);
		status = ARM_MATH_ARGUMENT_ERROR;
	}

cleanup_and_exit:
	// Copy final result if successful or last state if not converged
	if (status == ARM_MATH_SUCCESS) {
		memcpy(final_beta, current_beta, NUM_PARAMS * sizeof(float32_t));
		LOG_INF("Final Beta: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", (double)final_beta[0],
			(double)final_beta[1], (double)final_beta[2], (double)final_beta[3],
			(double)final_beta[4], (double)final_beta[5]);
		LOG_INF("Final Error: %.6e", (double)current_error);
	} else {
		LOG_INF("Optimization failed with status: %d", status);
	}

	// Free allocated memory
	free(residuals_vec);
	free(jacobian_data);
	free(jacobian_T_data);
	free(JtJ_data);
	free(Aug_JtJ_data);
	free(Inv_Aug_JtJ_data);
	free(Jtr_vec);
	free(delta_vec);
	free(current_beta);
	free(temp_beta);
	free(temp_residuals_vec);

	return status;
}

// --- Example Usage ---
int main()
{
	// Initial guess for parameters (e.g., no offset, scale=1.0)
	// Beta: [offset_x, offset_y, offset_z, scale_x, scale_y, scale_z]
	// Note: Model uses 1/scale, so initial guess for b3,b4,b5 should be 1.0
	float32_t initial_beta[NUM_PARAMS] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f};
	float32_t final_beta[NUM_PARAMS];

	// Max number of samples to request from placeholder function
	uint32_t num_samples_to_request = 50;

	// Optional: Estimate memory requirement or set a limit
	uint32_t memory_limit_bytes = 0; // Set a limit if needed, 0 for test default

	LOG_INF("Starting Accelerometer Calibration...");

	arm_status result =
		lm_optimize_accel_calibration(initial_beta, // Provide initial guess
					      num_samples_to_request, memory_limit_bytes,
					      final_beta // Output buffer for results
		);

	float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
	if (result == ARM_MATH_SUCCESS) {
		LOG_INF("Calibration successful!");
		gyro_bias[0] = gyro_sum[0] / gyro_sample_cnt;
		gyro_bias[1] = gyro_sum[1] / gyro_sample_cnt;
		gyro_bias[2] = gyro_sum[2] / gyro_sample_cnt;
		LOG_INF("Gyro calibration: [%.3f, %.3f, %.3f]",
			(double)gyro_sum[0] / gyro_sample_cnt,
			(double)gyro_sum[1] / gyro_sample_cnt,
			(double)gyro_sum[2] / gyro_sample_cnt);
		// final_beta now contains the optimized parameters
		// Use these parameters to calibrate future accelerometer readings:
		// accel_calibrated_x = (accel_raw_x - final_beta[0]) / final_beta[3];
		// accel_calibrated_y = (accel_raw_y - final_beta[1]) / final_beta[4];
		// accel_calibrated_z = (accel_raw_z - final_beta[2]) / final_beta[5];

		float matrix_to_save1[MATRIX_ROWS][MATRIX_COLS] = {
			{final_beta[0], final_beta[1], final_beta[2]},
			{final_beta[3], final_beta[4], final_beta[5]},
			{gyro_bias[0], gyro_bias[1], gyro_bias[2]}};

		const float matrix_to_save2[MATRIX_ROWS][MATRIX_COLS] = {{0.0f}};

		// Buffers to read data into
		// float matrix_read1[MATRIX_ROWS][MATRIX_COLS];
		// float matrix_read2[MATRIX_ROWS][MATRIX_COLS];

		LOG_INF("--- Matrix Storage Test Start ---");

		// 1. --- Initial Check: Verify matrices do not exist ---
		LOG_INF("Step 1: Checking if matrices exist initially...");
		bool exists = matrix_storage_exists();
		if (exists) {
			LOG_INF("FAIL: Matrices exist before saving.");
			// Try to delete them to clean up for the rest of the test
			LOG_INF("Attempting to delete existing matrices...");
			matrix_storage_delete();
			// Re-check
			exists = matrix_storage_exists();
			if (exists) {
				LOG_INF("FAIL: Could not delete pre-existing matrices. "
					"Aborting.");
				return -1;
			} else {
				LOG_INF("INFO: Pre-existing matrices deleted successfully. "
					"");
			}
		} else {
			LOG_INF("OK: Matrices do not exist initially.");
		}

		// 2. --- Write Matrices ---
		LOG_INF("Step 2: Writing matrices to storage...");
		print_matrix("Matrix 1 to save", matrix_to_save1);

		int ret = matrix_storage_save(matrix_to_save1, matrix_to_save2);
		if (ret == 0) {
			LOG_INF("OK: matrix_storage_save() succeeded.");
		} else {
			LOG_INF("FAIL: matrix_storage_save() failed with error %d", ret);
			return ret; // Cannot proceed if save failed
		}

		// 2b. --- Verify Existence After Write ---
		LOG_INF("Step 2b: Checking if matrices exist after saving...");
		exists = matrix_storage_exists();
		if (exists) {
			LOG_INF("OK: Matrices exist after saving (as expected).");
		} else {
			LOG_INF("FAIL: Matrices do not exist after saving.");
			return -1; // Something went wrong
		}
	} else {
		LOG_INF("Calibration failed or did not converge (Status: %d).", result);
		// Handle error or use last computed beta in final_beta if status is
		// NO_CONVERGENCE
	}

	float32_t accel_cali[7];
	float32_t gyro_cali[3] = {0.0f, 0.0f, 0.0f};
	// jf_channel_add(data, &accel_cali[0], PTR_FLOAT);
	// jf_channel_add(data, &accel_cali[1], PTR_FLOAT);
	// jf_channel_add(data, &accel_cali[2], PTR_FLOAT);
	// jf_channel_add(data, &accel_cali[3], PTR_FLOAT);
	// jf_channel_add(data, &accel_cali[4], PTR_FLOAT);
	// jf_channel_add(data, &accel_cali[5], PTR_FLOAT);
	// jf_channel_add(data, &accel_cali[6], PTR_FLOAT);
	// jf_channel_add(data, 0, RAW);
	// jf_channel_add(data, &gyro_cali[0], PTR_FLOAT);
	// jf_channel_add(data, &gyro_cali[1], PTR_FLOAT);
	// jf_channel_add(data, &gyro_cali[2], PTR_FLOAT);
	while (1) {
		accel_cali[0] = (accel[0] - final_beta[0]) / final_beta[3];
		accel_cali[1] = (accel[1] - final_beta[1]) / final_beta[4];
		accel_cali[2] = (accel[2] - final_beta[2]) / final_beta[5];
		accel_cali[3] =
			sqrtf(accel_cali[0] * accel_cali[0] + accel_cali[1] * accel_cali[1] +
			      accel_cali[2] * accel_cali[2]);
		accel_cali[4] =
			sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
		accel_cali[5] = accel_cali[3] - G;
		accel_cali[6] = accel_cali[4] - G;

		gyro_cali[0] = gyro[0] - gyro_bias[0];
		gyro_cali[1] = gyro[1] - gyro_bias[1];
		gyro_cali[2] = gyro[2] - gyro_bias[2];
		k_msleep(1);
	}

	return 0;
}

// int main(void)
// {
// 	int ret;
// 	bool exists;

// 	// --- Define Test Data ---
// 	const float matrix_to_save1[MATRIX_ROWS][MATRIX_COLS] = {
// 		{1.1f, 2.2f, 3.3f, 4.4f}, {5.5f, 6.6f, 7.7f, 8.8f},
// {9.9f, 10.1f, 11.2f, 12.3f}};

// 	const float matrix_to_save2[MATRIX_ROWS][MATRIX_COLS] = {{-1.0f, -2.0f, -3.0f,
// -4.0f},
// {10.0f, 20.0f, 30.0f, 40.0f},
// {0.1f, 0.2f, 0.3f, 0.4f}};

// 	// Buffers to read data into
// 	float matrix_read1[MATRIX_ROWS][MATRIX_COLS];
// 	float matrix_read2[MATRIX_ROWS][MATRIX_COLS];

// 	LOG_INF("--- Matrix Storage Test Start ---");

// 	// 1. --- Initial Check: Verify matrices do not exist ---
// 	LOG_INF("Step 1: Checking if matrices exist initially...");
// 	exists = matrix_storage_exists();
// 	if (exists) {
// 		LOG_INF("FAIL: Matrices unexpectedly exist before saving.");
// 		// Try to delete them to clean up for the rest of the test
// 		LOG_INF("Attempting to delete existing matrices...");
// 		matrix_storage_delete();
// 		// Re-check
// 		exists = matrix_storage_exists();
// 		if (exists) {
// 			LOG_INF("FAIL: Could not delete pre-existing matrices. Aborting.");
// 			return -1;
// 		} else {
// 			LOG_INF("INFO: Pre-existing matrices deleted successfully. Continuing
// " 			       "test.");
// 		}
// 	} else {
// 		LOG_INF("OK: Matrices do not exist initially (as expected).");
// 	}

// 	// 2. --- Write Matrices ---
// 	LOG_INF("Step 2: Writing matrices to storage...");
// 	print_matrix("Matrix 1 to save", matrix_to_save1);
// 	print_matrix("Matrix 2 to save", matrix_to_save2);

// 	ret = matrix_storage_save(matrix_to_save1, matrix_to_save2);
// 	if (ret == 0) {
// 		LOG_INF("OK: matrix_storage_save() succeeded.");
// 	} else {
// 		LOG_INF("FAIL: matrix_storage_save() failed with error %d", ret);
// 		return ret; // Cannot proceed if save failed
// 	}

// 	// 2b. --- Verify Existence After Write ---
// 	LOG_INF("Step 2b: Checking if matrices exist after saving...");
// 	exists = matrix_storage_exists();
// 	if (exists) {
// 		LOG_INF("OK: Matrices exist after saving (as expected).");
// 	} else {
// 		LOG_INF("FAIL: Matrices do not exist after saving.");
// 		return -1; // Something went wrong
// 	}

// 	// 3. --- Read Matrices ---
// 	LOG_INF("Step 3: Reading matrices from storage...");
// 	// Clear read buffers first to ensure they are populated by the read function
// 	memset(matrix_read1, 0, sizeof(matrix_read1));
// 	memset(matrix_read2, 0, sizeof(matrix_read2));

// 	ret = matrix_storage_read(matrix_read1, matrix_read2);
// 	if (ret == 0) {
// 		LOG_INF("OK: matrix_storage_read() succeeded.");
// 		print_matrix("Matrix 1 read", matrix_read1);
// 		print_matrix("Matrix 2 read", matrix_read2);
// 	} else {
// 		LOG_INF("FAIL: matrix_storage_read() failed with error %d", ret);
// 		return ret; // Cannot proceed if read failed
// 	}

// 	// 4. --- Verify Read Data ---
// 	LOG_INF("Step 4: Verifying read matrices against original data...");
// 	bool match1 = compare_matrices(matrix_to_save1, matrix_read1);
// 	bool match2 = compare_matrices(matrix_to_save2, matrix_read2);

// 	if (match1 && match2) {
// 		LOG_INF("OK: Read matrices match the saved matrices.");
// 	} else {
// 		LOG_INF("FAIL: Read matrices DO NOT match the saved matrices!");
// 		if (!match1) {
// 			LOG_INF(" -> Matrix 1 differs.");
// 		}
// 		if (!match2) {
// 			LOG_INF(" -> Matrix 2 differs.");
// 		}
// 		// Optional: Print matrices again if they didn't match
// 		// print_matrix("Original Matrix 1", matrix_to_save1);
// 		// print_matrix("Read Matrix 1", matrix_read1);
// 		// print_matrix("Original Matrix 2", matrix_to_save2);
// 		// print_matrix("Read Matrix 2", matrix_read2);
// 		return -1; // Verification failed
// 	}

// 	// 5. --- Delete Matrices ---
// 	LOG_INF("Step 5: Deleting matrices from storage...");
// 	ret = matrix_storage_delete();
// 	// According to the API doc, delete returns 0 on success or if already deleted.
// 	if (ret == 0) {
// 		LOG_INF("OK: matrix_storage_delete() succeeded (or item was not found).");
// 	} else {
// 		// This path should ideally not be taken unless there's an I/O error
// 		LOG_INF("FAIL: matrix_storage_delete() failed with error %d", ret);
// 		// Continue to final check anyway, but report failure.
// 		return ret;
// 	}

// 	// 6. --- Final Check: Verify matrices are gone ---
// 	LOG_INF("Step 6: Checking if matrices exist after deletion...");
// 	exists = matrix_storage_exists();
// 	if (!exists) {
// 		LOG_INF("OK: Matrices do not exist after deletion (as expected).");
// 	} else {
// 		LOG_INF("FAIL: Matrices still exist after calling delete!");
// 		return -1;
// 	}

// 	LOG_INF("--- Matrix Storage Test End: SUCCESS ---");
// 	return 0; // Indicate success
// }

// /*
//  * Note: To compile and run this test, you would need:
//  * 1. A C compiler (like GCC).
//  * 2. The implementation file (`matrix_storage.c` or similar) that provides
//  *    the actual logic for the functions declared in `matrix_storage.h`.
//  * 3. The underlying ZMS (Zephyr Module Storage) library/implementation and
//  *    its dependencies (like a flash driver and filesystem core), likely within
//  *    a Zephyr RTOS build environment.
//  *
//  * Example compilation command (highly simplified, assuming standalone C files):
//  * gcc main.c matrix_storage.c -o test_matrix_storage -I.
//  *
//  * In a Zephyr environment, you would add this main.c to your application source
//  * files (e.g., in src/main.c or a dedicated test source file) and build the
//  * Zephyr application as usual (e.g., using `west build`). The ZMS backend
//  * (flash area, etc.) would need to be configured in your board's devicetree
//  * and Kconfig files.
//  */
