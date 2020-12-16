#pragma once

#include <fcntl.h> // open
#include <unistd.h> // close
#include <inttypes.h> // *int*_t
#include <sys/ioctl.h> // ioctl
#include <stdio.h> // perror
#include <string.h> // memset

#include <linux/spi/spidev.h>

#include "rpi_nrf24_map.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    int ce_io_num;
} nrf24_t;

int nrf24_init(nrf24_t *dev, const char *path, int ce_io_num);
int nrf24_free(nrf24_t *dev);

int nrf24_get_register(nrf24_t *dev, uint8_t reg, uint8_t *data, uint8_t len);
int nrf24_set_register(nrf24_t *dev, uint8_t reg, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif