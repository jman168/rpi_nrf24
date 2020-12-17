#pragma once

#include <fcntl.h> // open
#include <unistd.h> // close
#include <inttypes.h> // *int*_t
#include <sys/ioctl.h> // ioctl
#include <stdio.h> // perror
#include <string.h> // memset
#include <sys/mman.h> // mmap

#include <linux/spi/spidev.h>

#include "rpi_nrf24_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BCM2708_PERI_BASE        0x3F000000 // RPi 2 and 3
// #define BCM2708_PERI_BASE        0x20000000 // RPi 1
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

#define INP_GPIO(gpio, g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(gpio, g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

#define GPIO_SET(gpio) *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR(gpio) *(gpio+10) // clears bits which are 1 ignores bits which are 0

typedef struct {
    int fd;
    int ce_io_num;
    volatile unsigned *gpio;
} nrf24_t;

enum nrf24_data_rate_t {
    NRF24_1MBPS = 0,
    NRF24_2MBPS,
    NRF24_250KBPS
};

enum nrf24_crc_t {
    NRF24_CRC_DISABLED = 0,
    NRF24_CRC_1BYTE,
    NRF24_CRC_2BYTES
};

enum nrf24_data_pipe_t {
    NRF24_P0 = 0,
    NRF24_P1,
    NRF24_P2,
    NRF24_P3,
    NRF24_P4,
    NRF24_P5,
    NRF24_ALL_PIPES
};

int nrf24_init(nrf24_t *dev, const char *path, int ce_io_num);
int nrf24_free(nrf24_t *dev);

int nrf24_get_register(nrf24_t *dev, uint8_t reg, uint8_t *data, uint8_t len);
int nrf24_set_register(nrf24_t *dev, uint8_t reg, uint8_t *data, uint8_t len);

int nrf24_flush_tx(nrf24_t *dev);
int nrf24_flush_rx(nrf24_t *dev);

int nrf24_power_up_tx(nrf24_t *dev);
int nrf24_power_up_rx(nrf24_t *dev);
int nrf24_power_down(nrf24_t *dev);

int nrf24_set_data_rate(nrf24_t *dev, enum nrf24_data_rate_t rate);
int nrf24_set_crc(nrf24_t *dev, enum nrf24_crc_t crc);
int nrf24_set_rf_channel(nrf24_t *dev, uint8_t channel);

int nrf24_enable_rx_pipe(nrf24_t *dev, enum nrf24_data_pipe_t pipe);
int nrf24_disable_rx_pipe(nrf24_t *dev, enum nrf24_data_pipe_t pipe);

void nrf24_flip_bytes(uint8_t *data, size_t len);

int nrf24_set_rx_address(nrf24_t *dev, enum nrf24_data_pipe_t pipe, uint8_t *address, uint8_t address_length);
int nrf24_set_tx_address(nrf24_t *dev, uint8_t *address, uint8_t address_length);

int nrf24_set_payload_length(nrf24_t *dev, uint8_t length);

#ifdef __cplusplus
}
#endif