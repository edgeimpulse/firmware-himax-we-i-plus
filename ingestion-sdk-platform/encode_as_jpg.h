#ifndef ENCODE_AS_JPG_H_
#define ENCODE_AS_JPG_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "JPEGENC.h"

int encode_as_jpg(uint8_t *framebuffer, size_t framebuffer_size, int width, int height, uint8_t *out_buffer, size_t out_buffer_size, size_t *out_size) {
    static JPEG jpg;
    JPEGENCODE jpe;

    int rc = jpg.open(out_buffer, out_buffer_size);
    if (rc != JPEG_SUCCESS) {
        return rc;
    }

    rc = jpg.encodeBegin(&jpe, width, height, JPEG_PIXEL_GRAYSCALE, JPEG_SUBSAMPLE_444, JPEG_Q_BEST);
    if (rc != JPEG_SUCCESS) {
        return rc;
    }

    int imcuCount = ((width + jpe.cx-1)/ jpe.cx) * ((height + jpe.cy-1) / jpe.cy);

    int bytePp = 1;
    int pitch = bytePp * width;

    for (int i=0; i < imcuCount && rc == JPEG_SUCCESS; i++) {
        // pass a pointer to the upper left corner of each MCU
        // the JPEGENCODE structure is updated by addMCU() after
        // each call
        rc = jpg.addMCU(&jpe, &framebuffer[(jpe.x * bytePp) + (jpe.y * pitch)], pitch);
        if (rc != JPEG_SUCCESS) {
            return rc;
        }
    }

    *out_size = jpg.close();

    return 0;
}

#endif // ENCODE_AS_JPG_H
