#ifndef _BITMAP_HELPERS_H_
#define _BITMAP_HELPERS_H_

#include <stdio.h>
#include <string.h>
#include <string.h>

static void _r565_to_rgb(uint16_t color, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = (color & 0xF800) >> 8;
    *g = (color & 0x07E0) >> 3;
    *b = (color & 0x1F) << 3;
}

// adapted from https://stackoverflow.com/a/2654860/107642
// used for debugging
int create_bitmap_file(const char *filename, uint16_t *buffer, size_t w, size_t h) {
    int filesize = 54 + 3 * w * h;

    uint8_t *img = (uint8_t*)calloc(3 * w * h, 1);
    if (!img) {
        return -1; /* OOM */
    }
    for (size_t i = 0; i < w; i++) {
        for (size_t j = 0; j < h; j++) {
            uint8_t r, g, b;
            uint16_t color = buffer[(j * w) + i];
            _r565_to_rgb(color, &r, &g, &b);

            size_t x = i;
            size_t y = (h - 1) - j;
            img[(x+y*w)*3+2] = (unsigned char)(r);
            img[(x+y*w)*3+1] = (unsigned char)(g);
            img[(x+y*w)*3+0] = (unsigned char)(b);
        }
    }

    uint8_t bmpfileheader[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
    uint8_t bmpinfoheader[40] = { 40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0 };
    uint8_t bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (uint8_t)(filesize    );
    bmpfileheader[ 3] = (uint8_t)(filesize>> 8);
    bmpfileheader[ 4] = (uint8_t)(filesize>>16);
    bmpfileheader[ 5] = (uint8_t)(filesize>>24);

    bmpinfoheader[ 4] = (uint8_t)(       w    );
    bmpinfoheader[ 5] = (uint8_t)(       w>> 8);
    bmpinfoheader[ 6] = (uint8_t)(       w>>16);
    bmpinfoheader[ 7] = (uint8_t)(       w>>24);
    bmpinfoheader[ 8] = (uint8_t)(       h    );
    bmpinfoheader[ 9] = (uint8_t)(       h>> 8);
    bmpinfoheader[10] = (uint8_t)(       h>>16);
    bmpinfoheader[11] = (uint8_t)(       h>>24);

    FILE *f = fopen(filename, "wb");
    if (!f) {
        free(img);
        return -2; /* could not open file */
    }
    fwrite(bmpfileheader, 1, 14, f);
    fwrite(bmpinfoheader, 1, 40, f);
    for(int i = 0; i < h; i++) {
        fwrite(img + (w * (h - i - 1) * 3), 3, w, f);
        fwrite(bmppad, 1, (4 - (w * 3) % 4 ) % 4, f);
    }
    free(img);
    fclose(f);

    return 0;
}

// adapted from https://stackoverflow.com/a/2654860/107642
// used for debugging
int create_bitmap_file(uint8_t *out_buf, float *buffer, size_t w, size_t h) {
    int filesize = 54 + 3 * w * h;

    uint8_t *img = (uint8_t*)calloc(3 * w * h, 1);
    if (!img) {
        return -1; /* OOM */
    }
    for (size_t i = 0; i < w; i++) {
        for (size_t j = 0; j < h; j++) {
            float pixel_f = buffer[(j * w) + i];
            uint32_t pixel = static_cast<uint32_t>(pixel_f);
            uint8_t r = static_cast<uint8_t>(pixel >> 16 & 0xff);
            uint8_t g = static_cast<uint8_t>(pixel >> 8 & 0xff);
            uint8_t b = static_cast<uint8_t>(pixel & 0xff);

            size_t x = i;
            size_t y = (h - 1) - j;
            img[(x+y*w)*3+2] = (unsigned char)(r);
            img[(x+y*w)*3+1] = (unsigned char)(g);
            img[(x+y*w)*3+0] = (unsigned char)(b);
        }
    }

    uint8_t bmpfileheader[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
    uint8_t bmpinfoheader[40] = { 40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0 };
    uint8_t bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (uint8_t)(filesize    );
    bmpfileheader[ 3] = (uint8_t)(filesize>> 8);
    bmpfileheader[ 4] = (uint8_t)(filesize>>16);
    bmpfileheader[ 5] = (uint8_t)(filesize>>24);

    bmpinfoheader[ 4] = (uint8_t)(       w    );
    bmpinfoheader[ 5] = (uint8_t)(       w>> 8);
    bmpinfoheader[ 6] = (uint8_t)(       w>>16);
    bmpinfoheader[ 7] = (uint8_t)(       w>>24);
    bmpinfoheader[ 8] = (uint8_t)(       h    );
    bmpinfoheader[ 9] = (uint8_t)(       h>> 8);
    bmpinfoheader[10] = (uint8_t)(       h>>16);
    bmpinfoheader[11] = (uint8_t)(       h>>24);

    uint8_t *ptr = out_buf;
    size_t bytes;
    bytes = 14;
    memcpy(ptr, bmpfileheader, bytes);
    ptr += bytes;
    bytes = 40;
    memcpy(ptr, bmpinfoheader, bytes);
    ptr += bytes;
    for(int i = 0; i < h; i++) {
        bytes = 3*w;
        memcpy(ptr, img + (w * (h - i - 1) * 3), bytes);
        ptr += bytes;
        bytes = (4 - (w * 3) % 4 ) % 4;
        memcpy(ptr, bmppad, bytes);
        ptr += bytes;
    }
    free(img);
    //fclose(f);

    return 0;
}

int bitmap_from_signal(uint8_t *img_buffer, size_t img_buffer_size, signal_t *signal, size_t w, size_t h) {
    int filesize = 54 + 3 * w * h;
    if (img_buffer_size != filesize) {
        return -3;
    }

    size_t buffer_size = 1024;
    size_t buffer_offset = buffer_size;
    size_t items_left = signal->total_length;
    float *buffer = (float*)malloc(buffer_size * sizeof(float));
    if (!buffer) {
        return -1;
    }
    memset(img_buffer, 0, filesize);

    for (size_t i = 0; i < w; i++) {
        for (size_t j = 0; j < h; j++) {
            if (buffer_offset == buffer_size) {
                size_t items_to_read = buffer_size;
                if (items_to_read > items_left) {
                    items_to_read = items_left;
                }
                signal->get_data((j * w) + i, items_to_read, buffer);
                buffer_offset = 0;
            }

            float pixel_f = buffer[buffer_offset];
            uint32_t pixel = static_cast<uint32_t>(pixel_f);
            uint8_t r = static_cast<uint8_t>(pixel >> 16 & 0xff);
            uint8_t g = static_cast<uint8_t>(pixel >> 8 & 0xff);
            uint8_t b = static_cast<uint8_t>(pixel & 0xff);

            size_t x = i;
            size_t y = (h - 1) - j;

            img_buffer[(x+y*w)*3+0] = (uint8_t)(b);
            img_buffer[(x+y*w)*3+1] = (uint8_t)(g);
            img_buffer[(x+y*w)*3+2] = (uint8_t)(r);

            buffer_offset++;
        }
    }

    uint8_t bmpfileheader[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
    uint8_t bmpinfoheader[40] = { 40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0 };
    uint8_t bmppad[3] = {0,0,0};

    bmpfileheader[ 2] = (uint8_t)(filesize    );
    bmpfileheader[ 3] = (uint8_t)(filesize>> 8);
    bmpfileheader[ 4] = (uint8_t)(filesize>>16);
    bmpfileheader[ 5] = (uint8_t)(filesize>>24);

    bmpinfoheader[ 4] = (uint8_t)(       w    );
    bmpinfoheader[ 5] = (uint8_t)(       w>> 8);
    bmpinfoheader[ 6] = (uint8_t)(       w>>16);
    bmpinfoheader[ 7] = (uint8_t)(       w>>24);
    bmpinfoheader[ 8] = (uint8_t)(       h    );
    bmpinfoheader[ 9] = (uint8_t)(       h>> 8);
    bmpinfoheader[10] = (uint8_t)(       h>>16);
    bmpinfoheader[11] = (uint8_t)(       h>>24);

    uint8_t *ptr = img_buffer;
    size_t bytes;
    bytes = 14;
    memcpy(ptr, bmpfileheader, bytes);
    ptr += bytes;
    bytes = 40;
    memcpy(ptr, bmpinfoheader, bytes);
    ptr += bytes;
    for(int i = 0; i < h; i++) {
        bytes = 3*w;
        memcpy(ptr, img_buffer + (w * (h - i - 1) * 3), bytes);
        ptr += bytes;
        bytes = (4 - (w * 3) % 4 ) % 4;
        memcpy(ptr, bmppad, bytes);
        ptr += bytes;
    }

    free(buffer);

    return 0;
}

#endif // _BITMAP_HELPERS_H_
