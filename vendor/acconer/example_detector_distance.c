// Copyright (c) Acconeer AB, 2022-2025
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "acc_definitions_a121.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"
#include "stm32wbxx_hal.h"
#include "stm32wbxx_hal_uart.h"
#include "string.h"

//UART_HandleTypeDef huart1;

typedef enum
{
	DISTANCE_PRESET_CONFIG_NONE = 0,
	DISTANCE_PRESET_CONFIG_BALANCED,
	DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

#define SENSOR_ID (1U)
// 2 seconds should be enough even for long ranges and high signal quality
#define SENSOR_TIMEOUT_MS (2000U)

typedef struct
{
	acc_sensor_t                     *sensor;
	acc_detector_distance_config_t   *config;
	acc_detector_distance_handle_t   *handle;
	void                             *buffer;
	uint32_t                          buffer_size;
	uint8_t                          *detector_cal_result_static;
	uint32_t                          detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
} distance_detector_resources_t;

static void cleanup(distance_detector_resources_t *resources);

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);

static bool initialize_detector_resources(distance_detector_resources_t *resources);

static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *sensor_cal_result, void *buffer, uint32_t buffer_size);

static bool do_full_detector_calibration(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result);

static bool do_detector_calibration_update(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result);

static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result);

static void print_distance_result(const acc_detector_distance_result_t *result);

int acc_example_detector_distance(int argc, char *argv[]);

int acc_example_detector_distance(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	distance_detector_resources_t resources = {0};

	//printf("Acconeer software version %s\n", acc_version_get());
	char version_msg[100]; 
	const char* version = acc_version_get();
	int msg_len = snprintf(version_msg, sizeof(version_msg), "Acconeer software version %s\n", version);
	if (msg_len > 0) {
    HAL_UART_Transmit(&huart1, (uint8_t*)version_msg, msg_len, HAL_MAX_DELAY);
	}
	
	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	resources.config = acc_detector_distance_config_create();
	if (resources.config == NULL)
	{
		//printf("acc_detector_distance_config_create() failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Assembly test: Memory allocation failed\n", strlen("Assembly test: Memory allocation failed\n"), HAL_MAX_DELAY);
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	set_config(resources.config, DISTANCE_PRESET_CONFIG_BALANCED);

	if (!initialize_detector_resources(&resources))
	{
		//printf("Initializing detector resources failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Initializing detector resources failed\n", strlen("Initializing detector resources failed\n"), HAL_MAX_DELAY);

		cleanup(&resources);
		return EXIT_FAILURE;
	}

	// Print the configuration
	acc_detector_distance_config_log(resources.handle, resources.config);

	/* Turn the sensor on */
	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	resources.sensor = acc_sensor_create(SENSOR_ID);
	if (resources.sensor == NULL)
	{
		//printf("acc_sensor_create() failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"acc_sensor_create() failed\n", strlen("acc_sensor_create() failed\n"), HAL_MAX_DELAY);

		cleanup(&resources);
		return EXIT_FAILURE;
	}

	acc_cal_result_t sensor_cal_result;

	if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
	{
		//printf("Sensor calibration failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Sensor calibration failed\n", strlen("Sensor calibration failed\n"), HAL_MAX_DELAY);

		cleanup(&resources);
		return EXIT_FAILURE;
	}

	if (!do_full_detector_calibration(&resources, &sensor_cal_result))
	{
		//printf("Detector calibration failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Detector calibration failed\n", strlen("Detector calibration failed\n"), HAL_MAX_DELAY);

		cleanup(&resources);
		return EXIT_FAILURE;
	}

	while (true)
	{
		acc_detector_distance_result_t result = {0};

		if (!do_detector_get_next(&resources, &sensor_cal_result, &result))
		{
			//printf("Could not get next result\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"Could not get next result\n", strlen("Could not get next result\n"), HAL_MAX_DELAY);

			cleanup(&resources);
			return EXIT_FAILURE;
		}

		/* If "calibration needed" is indicated, the sensor needs to be recalibrated and the detector calibration updated */
		if (result.calibration_needed)
		{
			//printf("Sensor recalibration and detector calibration update needed ... \n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"Sensor recalibration and detector calibration update needed ... \n", strlen("Sensor recalibration and detector calibration update needed ... \n"), HAL_MAX_DELAY);

			
			if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
			{
				//printf("Sensor calibration failed\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)"Sensor calibration failed\n", strlen("Sensor calibration failed\n"), HAL_MAX_DELAY);

				cleanup(&resources);
				return EXIT_FAILURE;
			}

			/* Once the sensor is recalibrated, the detector calibration should be updated and measuring can continue. */
			if (!do_detector_calibration_update(&resources, &sensor_cal_result))
			{
				//printf("Detector calibration update failed\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)"Detector calibration update failed\n", strlen("Detector calibration update failed\n"), HAL_MAX_DELAY);

				cleanup(&resources);
				return EXIT_FAILURE;
			}

			//printf("Sensor recalibration and detector calibration update done!\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"Sensor recalibration and detector calibration update done!\n", strlen("Sensor recalibration and detector calibration update done!\n"), HAL_MAX_DELAY);

		}
		else
		{
			print_distance_result(&result);
		}
		vTaskDelay(500);
	}

	cleanup(&resources);

	//printf("Done!\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)"Done!\n", strlen("Done!\n"), HAL_MAX_DELAY);


	return EXIT_SUCCESS;
}

static void cleanup(distance_detector_resources_t *resources)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	acc_detector_distance_config_destroy(resources->config);
	acc_detector_distance_destroy(resources->handle);

	acc_integration_mem_free(resources->buffer);
	acc_integration_mem_free(resources->detector_cal_result_static);

	if (resources->sensor != NULL)
	{
		acc_sensor_destroy(resources->sensor);
	}
}

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset)
{
	// Add configuration of the detector here
	switch (preset)
	{
		case DISTANCE_PRESET_CONFIG_NONE:
			// Add configuration of the detector here
			break;

		case DISTANCE_PRESET_CONFIG_BALANCED:
			acc_detector_distance_config_start_set(detector_config, 0.1f);
			acc_detector_distance_config_end_set(detector_config, 4.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 0U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 15.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;

		case DISTANCE_PRESET_CONFIG_HIGH_ACCURACY:
			acc_detector_distance_config_start_set(detector_config, 0.1f);
			acc_detector_distance_config_end_set(detector_config, 4.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 2U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_3);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 20.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;
	}
}

static bool initialize_detector_resources(distance_detector_resources_t *resources)
{
	resources->handle = acc_detector_distance_create(resources->config);
	if (resources->handle == NULL)
	{
		//printf("acc_detector_distance_create() failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"acc_detector_distance_create() failed\n", strlen("acc_detector_distance_create() failed\n"), HAL_MAX_DELAY);

		return false;
	}

	if (!acc_detector_distance_get_sizes(resources->handle, &(resources->buffer_size), &(resources->detector_cal_result_static_size)))
	{
		//printf("acc_detector_distance_get_sizes() failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"acc_detector_distance_get_sizes() failed\n", strlen("acc_detector_distance_get_sizes() failed\n"), HAL_MAX_DELAY);

		return false;
	}

	resources->buffer = acc_integration_mem_alloc(resources->buffer_size);
	if (resources->buffer == NULL)
	{
		//printf("sensor buffer allocation failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"sensor buffer allocation failed\n", strlen("sensor buffer allocation failed\n"), HAL_MAX_DELAY);

		return false;
	}

	resources->detector_cal_result_static = acc_integration_mem_alloc(resources->detector_cal_result_static_size);
	if (resources->detector_cal_result_static == NULL)
	{
		//printf("calibration buffer allocation failed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)"calibration buffer allocation failed\n", strlen("calibration buffer allocation failed\n"), HAL_MAX_DELAY);

		return false;
	}

	return true;
}

static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *sensor_cal_result, void *buffer, uint32_t buffer_size)
{
	bool           status              = false;
	bool           cal_complete        = false;
	const uint16_t calibration_retries = 1U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		do
		{
			status = acc_sensor_calibrate(sensor, &cal_complete, sensor_cal_result, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
		} while (status && !cal_complete);
	}

	if (status)
	{
		/* Reset sensor after calibration by disabling/enabling it */
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}

	return status;
}

static bool do_full_detector_calibration(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_calibrate(resources->sensor,
		                                         resources->handle,
		                                         sensor_cal_result,
		                                         resources->buffer,
		                                         resources->buffer_size,
		                                         resources->detector_cal_result_static,
		                                         resources->detector_cal_result_static_size,
		                                         &resources->detector_cal_result_dynamic,
		                                         &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}

static bool do_detector_calibration_update(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_update_calibration(resources->sensor,
		                                                  resources->handle,
		                                                  sensor_cal_result,
		                                                  resources->buffer,
		                                                  resources->buffer_size,
		                                                  &resources->detector_cal_result_dynamic,
		                                                  &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}

static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result)
{
	bool result_available = false;

	do
	{
		if (!acc_detector_distance_prepare(resources->handle,
		                                   resources->config,
		                                   resources->sensor,
		                                   sensor_cal_result,
		                                   resources->buffer,
		                                   resources->buffer_size))
		{
			//printf("acc_detector_distance_prepare() failed\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"acc_detector_distance_prepare() failed\n", strlen("acc_detector_distance_prepare() failed\n"), HAL_MAX_DELAY);

			return false;
		}

		if (!acc_sensor_measure(resources->sensor))
		{
			//printf("acc_sensor_measure() failed\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"acc_sensor_measure() failed\n", strlen("acc_sensor_measure() failed\n"), HAL_MAX_DELAY);
		
			return false;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			//printf("Sensor interrupt timeout\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"Sensor interrupt timeout\n", strlen("Sensor interrupt timeout\n"), HAL_MAX_DELAY);

			return false;
		}

		if (!acc_sensor_read(resources->sensor, resources->buffer, resources->buffer_size))
		{
			//printf("acc_sensor_read() failed\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"acc_sensor_read() failed\n", strlen("acc_sensor_read() failed\n"), HAL_MAX_DELAY);

			return false;
		}

		if (!acc_detector_distance_process(resources->handle,
		                                   resources->buffer,
		                                   resources->detector_cal_result_static,
		                                   &resources->detector_cal_result_dynamic,
		                                   &result_available,
		                                   result))
		{
			//printf("acc_detector_distance_process() failed\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)"acc_detector_distance_process() failed\n", strlen("acc_detector_distance_process() failed\n"), HAL_MAX_DELAY);

			return false;
		}
	} while (!result_available);

	return true;
}

static const uint8_t sliding_window_length = 10;
static float (*sliding_window_dist)[10];
static float (*sliding_window_str)[10];

static float *avg_value; //distance
static float *avg_strength; //strength


#define MAX_DETECTIONS 3

static void print_distance_result(const acc_detector_distance_result_t *result)
{
    float dist_buf[MAX_DETECTIONS]     = {0};
    float strength_buf[MAX_DETECTIONS] = {0};

    /* Copy detected values into fixed-size buffers */
    uint8_t count = result->num_distances;
    if (count > MAX_DETECTIONS)
        count = MAX_DETECTIONS;

    for (uint8_t i = 0; i < count; i++) {
        dist_buf[i]     = result->distances[i];
        strength_buf[i] = result->strengths[i];
    }

    /* ----- PRINT FIXED MESSAGE HEADER ----- */
    char head[32];
    int len = snprintf(head, sizeof(head), "%d detected distances: ", result->num_distances);
    HAL_UART_Transmit(&huart1, (uint8_t*)head, len, HAL_MAX_DELAY);

    /* ----- ALWAYS PRINT EXACTLY 3 FIELDS ----- */
    for (uint8_t i = 0; i < MAX_DETECTIONS; i++)
    {
        char out[64];
        int len2 = snprintf(
            out,
            sizeof(out),
            "%.3f m  (strength: %.3f m)  ",
            dist_buf[i],
            strength_buf[i]
        );

        HAL_UART_Transmit(&huart1, (uint8_t*)out, len2, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart1, (uint8_t*)"\n", 1, HAL_MAX_DELAY);
}
