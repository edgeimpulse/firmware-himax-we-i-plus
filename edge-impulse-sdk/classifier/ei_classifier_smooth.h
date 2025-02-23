/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EI_CLASSIFIER_SMOOTH_H_
#define _EI_CLASSIFIER_SMOOTH_H_

#if EI_CLASSIFIER_OBJECT_DETECTION != 1

#include <stdint.h>

typedef struct ei_classifier_smooth {
    int *last_readings;
    size_t last_readings_size;
    uint8_t min_readings_same;
    float classifier_confidence;
    float anomaly_confidence;
    uint8_t count[EI_CLASSIFIER_LABEL_COUNT + 2] = { 0 };
    size_t count_size = EI_CLASSIFIER_LABEL_COUNT + 2;
} ei_classifier_smooth_t;

/**
 * Initialize a smooth structure. This is useful if you don't want to trust
 * single readings, but rather want consensus
 * (e.g. 7 / 10 readings should be the same before I draw any ML conclusions).
 * This allocates memory on the heap!
 * @param smooth Pointer to an uninitialized ei_classifier_smooth_t struct
 * @param n_readings Number of readings you want to store
 * @param min_readings_same Minimum readings that need to be the same before concluding (needs to be lower than n_readings)
 * @param classifier_confidence Minimum confidence in a class (default 0.8)
 * @param anomaly_confidence Maximum error for anomalies (default 0.3)
 */
void ei_classifier_smooth_init(ei_classifier_smooth_t *smooth, size_t n_readings,
                               uint8_t min_readings_same, float classifier_confidence = 0.8,
                               float anomaly_confidence = 0.3) {
    smooth->last_readings = (int*)ei_malloc(n_readings * sizeof(int));
    for (size_t ix = 0; ix < n_readings; ix++) {
        smooth->last_readings[ix] = -1; // -1 == uncertain
    }
    smooth->last_readings_size = n_readings;
    smooth->min_readings_same = min_readings_same;
    smooth->classifier_confidence = classifier_confidence;
    smooth->anomaly_confidence = anomaly_confidence;
    smooth->count_size = EI_CLASSIFIER_LABEL_COUNT + 2;
}

/**
 * Call when a new reading comes in.
 * @param smooth Pointer to an initialized ei_classifier_smooth_t struct
 * @param result Pointer to a result structure (after calling ei_run_classifier)
 * @returns Label, either 'uncertain', 'anomaly', or a label from the result struct
 */
const char* ei_classifier_smooth_update(ei_classifier_smooth_t *smooth, ei_impulse_result_t *result) {
    // clear out the count array
    memset(smooth->count, 0, EI_CLASSIFIER_LABEL_COUNT + 2);

    // roll through the last_readings buffer
    numpy::roll(smooth->last_readings, smooth->last_readings_size, -1);

    int reading = -1; // uncertain

    // print the predictions
    // printf("[");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        if (result->classification[ix].value >= smooth->classifier_confidence) {
            reading = (int)ix;
        }
    }
#if EI_CLASSIFIER_HAS_ANOMALY
    if (result->anomaly >= smooth->anomaly_confidence) {
        reading = -2; // anomaly
    }
#endif

    smooth->last_readings[smooth->last_readings_size - 1] = reading;

    // now count last 10 readings and see what we actually see...
    for (size_t ix = 0; ix < smooth->last_readings_size; ix++) {
        if (smooth->last_readings[ix] >= 0) {
            smooth->count[smooth->last_readings[ix]]++;
        }
        else if (smooth->last_readings[ix] == -1) { // uncertain
            smooth->count[EI_CLASSIFIER_LABEL_COUNT]++;
        }
        else if (smooth->last_readings[ix] == -2) { // anomaly
            smooth->count[EI_CLASSIFIER_LABEL_COUNT + 1]++;
        }
    }

    // then loop over the count and see which is highest
    uint8_t top_result = 0;
    uint8_t top_count = 0;
    bool met_confidence_threshold = false;
    uint8_t confidence_threshold = smooth->min_readings_same; // XX% of windows should be the same
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT + 2; ix++) {
        if (smooth->count[ix] > top_count) {
            top_result = ix;
            top_count = smooth->count[ix];
        }
        if (smooth->count[ix] >= confidence_threshold) {
            met_confidence_threshold = true;
        }
    }

    if (met_confidence_threshold) {
        if (top_result == EI_CLASSIFIER_LABEL_COUNT) {
            return "uncertain";
        }
        else if (top_result == EI_CLASSIFIER_LABEL_COUNT + 1) {
            return "anomaly";
        }
        else {
            return result->classification[top_result].label;
        }
    }
    return "uncertain";
}

/**
 * Clear up a smooth structure
 */
void ei_classifier_smooth_free(ei_classifier_smooth_t *smooth) {
    ei_free(smooth->last_readings);
}

#endif // #if EI_CLASSIFIER_OBJECT_DETECTION != 1

#endif // _EI_CLASSIFIER_SMOOTH_H_
