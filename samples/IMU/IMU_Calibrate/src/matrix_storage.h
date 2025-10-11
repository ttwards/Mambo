/*
 * Copyright (c) 2024 BayLibre SAS
 * Copyright (c) 2024 Your Name/Company // Add your copyright
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MATRIX_STORAGE_H_
#define MATRIX_STORAGE_H_

#include <stdbool.h>
#include <stddef.h> // For size_t

#define MATRIX_ROWS 3
#define MATRIX_COLS 3

#define ZMS_MIN_SECTOR_COUNT 2

/**
 * @brief Saves two 3x3 float matrices to ZMS storage.
 *
 * This function attempts to initialize the ZMS filesystem if not already done,
 * then writes the provided matrices under a predefined internal ID.
 *
 * @param matrix1 Pointer to the first 3x3 float matrix.
 * @param matrix2 Pointer to the second 3x3 float matrix.
 * @return 0 on success.
 * @return -errno code on failure (e.g., -ENOSPC, -EIO, -EINVAL).
 */
int matrix_storage_save(const float matrix1[MATRIX_ROWS][MATRIX_COLS],
			const float matrix2[MATRIX_ROWS][MATRIX_COLS]);

/**
 * @brief Reads two 3x3 float matrices from ZMS storage.
 *
 * This function attempts to initialize the ZMS filesystem if not already done,
 * then reads the matrices associated with a predefined internal ID.
 *
 * @param matrix1 Pointer to a buffer to store the first 3x3 float matrix.
 * @param matrix2 Pointer to a buffer to store the second 3x3 float matrix.
 * @return 0 on success.
 * @return -ENOENT if the matrices are not found in storage.
 * @return -EIO if the read data size does not match the expected size.
 * @return Other -errno codes on ZMS or flash driver failures.
 */
int matrix_storage_read(float matrix1[MATRIX_ROWS][MATRIX_COLS],
			float matrix2[MATRIX_ROWS][MATRIX_COLS]);

/**
 * @brief Checks if the matrices exist in ZMS storage.
 *
 * This function attempts to initialize the ZMS filesystem if not already done,
 * then checks for the presence and correct size of the matrix data.
 *
 * @return true if the matrices exist and have the correct size.
 * @return false if the matrices do not exist, have an incorrect size,
 *         or an error occurred during initialization or check.
 */
bool matrix_storage_exists(void);

/**
 * @brief Deletes the stored matrices from ZMS storage.
 *
 * This function attempts to initialize the ZMS filesystem if not already done,
 * then deletes the matrices associated with a predefined internal ID.
 *
 * @return 0 on success or if the item was already deleted.
 * @return -errno code on failure during deletion (e.g., -EIO). Note that
 *         -ENOENT from the underlying zms_delete is treated as success here.
 */
int matrix_storage_delete(void);

#endif /* MATRIX_STORAGE_H_ */
