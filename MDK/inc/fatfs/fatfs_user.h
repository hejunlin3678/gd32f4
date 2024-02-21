#ifndef __FATFS_H
#define __FATFS_H

#include "ff.h"

typedef struct {
    uint8_t     rw:1;
    uint8_t     path[32];
    char        buffer[512];
} FatfsOps;

extern uint32_t totalSize;
extern uint32_t freeSize;

uint8_t fatfs_init(void);
uint8_t fatfs_getfree(void);
FRESULT fatfs_open_append (const char* path);
FRESULT fatfs_close(void);
FRESULT fatfs_read(const char* path, void *buffer, uint8_t len);
FRESULT fatfs_write(void *buffer);
FRESULT fatfs_test(FatfsOps* ops);

FRESULT fatfs_read_data(const char *path, uint32_t offset, void *buffer, uint32_t len);
FRESULT fatfs_write_data(char *path, uint8_t offset_en, uint32_t offset, void *buffer, uint32_t buffer_len);
FRESULT fatfs_print_data(const char *path, uint32_t file_size);
#endif //__FATFS_H
