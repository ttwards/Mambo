/*
 * Copyright (c) 2024 BayLibre SAS
 * Copyright (c) 2024 Your Name/Company // Add your copyright
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "matrix_storage.h"
#include "zephyr/logging/log_core.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/zms.h>
#include <zephyr/sys/util.h> // For ARRAY_SIZE
#include <string.h>          // For memcpy

#include <zephyr/logging/log.h>
// Define CONFIG_MATRIX_STORAGE_LOG_LEVEL in prj.conf
LOG_MODULE_REGISTER(matrix_storage, LOG_LEVEL_INF);

// --- Configuration ---

// The label of the partition designated in the device tree's chosen node
// (zephyr,storage-partition). Usually "storage".
#define MATRIX_STORAGE_PARTITION storage_partition

// The unique ID used within ZMS to store the matrix data.
// Pick a value unlikely to clash with other potential ZMS users.
#define MATRIX_DATA_ID 0x4D415452

// Derive flash parameters from the chosen partition
#define ZMS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(MATRIX_STORAGE_PARTITION)
#define ZMS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(MATRIX_STORAGE_PARTITION)
#define ZMS_PARTITION_SIZE   FIXED_PARTITION_SIZE(MATRIX_STORAGE_PARTITION)

// --- ZMS Instance and State ---

static struct zms_fs fs;
static bool zms_initialized;
static K_MUTEX_DEFINE(zms_lock); // Protect concurrent access to init and R/W

// Combined buffer structure for writing/reading
typedef struct {
	float matrix1[MATRIX_ROWS][MATRIX_COLS];
	float matrix2[MATRIX_ROWS][MATRIX_COLS];
} matrix_data_t;

// Calculate the exact size expected for the stored data
static const size_t expected_data_size = sizeof(matrix_data_t);

// --- Internal Functions ---

/**
 * @brief Initializes the ZMS filesystem instance.
 *
 * Should be called with the zms_lock held.
 * Does nothing if already initialized.
 *
 * @return 0 on success or if already initialized.
 * @return -errno code on failure.
 */
static int zms_fs_init(void)
{
	int rc = 0;

	if (zms_initialized) {
		return 0;
	}

	fs.flash_device = ZMS_PARTITION_DEVICE;
	if (!device_is_ready(fs.flash_device)) {
		LOG_ERR("Flash device %s not ready", fs.flash_device->name);
		return -ENODEV;
	}

	fs.offset = ZMS_PARTITION_OFFSET;

	// ZMS requires sector_size to be a power of 2 and >= flash page size.
	// We'll use the flash page size if it meets the criteria, otherwise
	// log an error (though ZMS might work if page size is smaller, it's
	// less efficient and not the primary design target).
	struct flash_pages_info page_info;

	rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &page_info);
	if (rc != 0) {
		LOG_ERR("Failed to get page info for offset 0x%lx (err %d)", (long)fs.offset, rc);
		return rc;
	}

	// Basic check: ZMS requires sector size to be power of 2.
	// flash_get_page_info doesn't guarantee this, although common.
	if (page_info.size == 0 || !is_power_of_two(page_info.size)) {
		// ZMS technically only *prefers* power of 2 for alignment, but might
		// fail if write block size > page_info.size. Let's enforce power of 2.
		LOG_ERR("Flash page size %u is not a power of 2, required by ZMS", page_info.size);
		// If this happens, you might need to configure ZMS sector size manually
		// to a larger, power-of-2 value that's a multiple of page_info.size.
		return -EINVAL;
	}

	fs.sector_size = page_info.size;

	// Calculate sector count based on partition size
	if (ZMS_PARTITION_SIZE < fs.sector_size * ZMS_MIN_SECTOR_COUNT) {
		LOG_ERR("Partition size %u is too small for ZMS (need at least %u sectors of size "
			"%u)",
			ZMS_PARTITION_SIZE, ZMS_MIN_SECTOR_COUNT, fs.sector_size);
		return -ENOSPC; // Or -EINVAL
	}
	fs.sector_count = ZMS_PARTITION_SIZE / fs.sector_size;
	// ZMS needs at least 2 sectors (one active, one for GC)
	if (fs.sector_count < ZMS_MIN_SECTOR_COUNT) {
		LOG_ERR("Calculated sector count %u is less than ZMS minimum %d", fs.sector_count,
			ZMS_MIN_SECTOR_COUNT);
		return -EINVAL;
	}

	LOG_DBG("Initializing ZMS on %s: offset=0x%lx, sector_size=%u, sector_count=%u",
		fs.flash_device->name, (long)fs.offset, fs.sector_size, fs.sector_count);

	rc = zms_mount(&fs);
	if (rc != 0) {
		LOG_ERR("Failed to mount ZMS filesystem (err %d)", rc);
		// Consider attempting zms_clear() here if mount fails badly?
		// For now, just report the error.
		return rc;
	}

	LOG_DBG("ZMS mounted successfully.");
	zms_initialized = true;
	return 0;
}

// --- Public API Implementation ---

int matrix_storage_save(const float matrix1[MATRIX_ROWS][MATRIX_COLS],
			const float matrix2[MATRIX_ROWS][MATRIX_COLS])
{
	int rc;
	matrix_data_t data_to_write;

	if (matrix1 == NULL || matrix2 == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&zms_lock, K_FOREVER);

	rc = zms_fs_init();
	if (rc != 0) {
		goto out;
	}

	// Prepare the data buffer
	memcpy(data_to_write.matrix1, matrix1, sizeof(data_to_write.matrix1));
	memcpy(data_to_write.matrix2, matrix2, sizeof(data_to_write.matrix2));

	// Write the combined data
	rc = zms_write(&fs, MATRIX_DATA_ID, &data_to_write, sizeof(data_to_write));
	if (rc < 0) { // zms_write returns negative errno on error
		LOG_ERR("Failed to write matrix data (ID: 0x%x): %d", MATRIX_DATA_ID, rc);
		// rc already holds the errno
	} else {
		LOG_DBG("Matrices saved successfully (ID: 0x%x)", MATRIX_DATA_ID);
		rc = 0; // Success
	}

out:
	k_mutex_unlock(&zms_lock);
	return rc;
}

int matrix_storage_read(float matrix1[MATRIX_ROWS][MATRIX_COLS],
			float matrix2[MATRIX_ROWS][MATRIX_COLS])
{
	int rc;
	matrix_data_t read_data;

	if (matrix1 == NULL || matrix2 == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&zms_lock, K_FOREVER);

	rc = zms_fs_init();
	if (rc != 0) {
		goto out;
	}

	// Read the data from ZMS
	rc = zms_read(&fs, MATRIX_DATA_ID, &read_data, sizeof(read_data));

	if (rc < 0) { // Error during read (e.g., -ENOENT if not found)
		LOG_DBG("Failed to read matrix data (ID: 0x%x): %d", MATRIX_DATA_ID, rc);
		// rc already holds the errno
	} else if (rc != expected_data_size) {
		// Read succeeded, but size is wrong (corruption?)
		LOG_ERR("Read matrix data size mismatch (ID: 0x%x): expected %u, got %d",
			MATRIX_DATA_ID, (unsigned int)expected_data_size, rc);
		rc = -EIO; // Input/output error seems appropriate
	} else {
		// Read succeeded and size is correct
		memcpy(matrix1, read_data.matrix1, sizeof(read_data.matrix1));
		memcpy(matrix2, read_data.matrix2, sizeof(read_data.matrix2));
		LOG_DBG("Matrices read successfully (ID: 0x%x)", MATRIX_DATA_ID);
		rc = 0; // Success
	}

out:
	k_mutex_unlock(&zms_lock);
	return rc;
}

bool matrix_storage_exists(void)
{
	int rc;
	ssize_t data_len;
	bool exists = false;

	k_mutex_lock(&zms_lock, K_FOREVER);

	rc = zms_fs_init();
	if (rc != 0) {
		goto out; // Cannot determine existence if init fails
	}

	data_len = zms_get_data_length(&fs, MATRIX_DATA_ID);

	if (data_len < 0) {
		if (data_len == -ENOENT) {
			LOG_DBG("Matrix data (ID: 0x%x) does not exist.", MATRIX_DATA_ID);
			exists = false;
		} else {
			LOG_ERR("Error checking data length for ID 0x%x: %d", MATRIX_DATA_ID,
				(int)data_len);
			exists = false; // Treat errors as "does not exist" for safety
		}
	} else if (data_len != expected_data_size) {
		LOG_WRN("Matrix data (ID: 0x%x) exists but has wrong size: %d (expected %u)",
			MATRIX_DATA_ID, (int)data_len, (unsigned int)expected_data_size);
		exists = false; // Treat wrong size as not existing correctly
	} else {
		LOG_DBG("Matrix data (ID: 0x%x) exists with correct size.", MATRIX_DATA_ID);
		exists = true;
	}

out:
	k_mutex_unlock(&zms_lock);
	return exists;
}

int matrix_storage_delete(void)
{
	int rc;

	k_mutex_lock(&zms_lock, K_FOREVER);

	rc = zms_fs_init();
	if (rc != 0) {
		goto out;
	}

	rc = zms_delete(&fs, MATRIX_DATA_ID);
	if (rc < 0) {
		if (rc == -ENOENT) {
			LOG_DBG("Matrix data (ID: 0x%x) already deleted or never existed.",
				MATRIX_DATA_ID);
			rc = 0; // Treat as success
		} else {
			LOG_ERR("Failed to delete matrix data (ID: 0x%x): %d", MATRIX_DATA_ID, rc);
			// rc holds the error code
		}
	} else {
		LOG_DBG("Matrix data deleted successfully (ID: 0x%x)", MATRIX_DATA_ID);
		rc = 0; // Success
	}

out:
	k_mutex_unlock(&zms_lock);
	return rc;
}
