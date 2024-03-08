/*
 * Copyright (c) 2024 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef EI_FUSION_SENSORS_CONFIG_H
#define EI_FUSION_SENSORS_CONFIG_H

#define NUM_FUSION_SENSORS       1  // number of fusable sensor modules
#define NUM_MAX_FUSIONS          1  // max number of sensor module combinations
#define FUSION_FREQUENCY         12.5f // sampling frequency for fusion samples
#define NUM_MAX_FUSION_AXIS      20 // max number of axis to sample
#define SIZEOF_SENSOR_NAME       20 // char alloc for sensor module name

/** Format used for fusion */
typedef float fusion_sample_format_t;

#endif // EI_FUSION_SENSORS_CONFIG_H
