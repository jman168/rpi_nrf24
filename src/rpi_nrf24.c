#include "rpi_nrf24.h"

int nrf24_init(nrf24_t *dev, const char *path, int ce_io_num) {
    dev->fd = open(path, O_RDWR);
    if(dev->fd < 0) {
        perror("open");
		return -1;
    }
    dev->ce_io_num = ce_io_num;

    uint32_t speed = 10*1000*1000;
    ioctl(dev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return 0;
}

int nrf24_free(nrf24_t *dev) {
    int ret = close(dev->fd);
    if(ret < 0) {
        perror("close");
		return -1;
    }
    return 0;
}

int nrf24_get_register(nrf24_t *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    struct spi_ioc_transfer	transaction;
    memset(&transaction, 0, sizeof(transaction));

    uint8_t tx[len+1];
    uint8_t rx[len+1];

    memset(tx, 0, len+1);
    memset(rx, 0, len+1);

    tx[0] = NRF24_CMD_R_REGISTER | (reg & NRF24_REGISTER_MASK);

    transaction.tx_buf = (uint64_t)tx;
    transaction.rx_buf = (uint64_t)rx;
    transaction.len = len+1;

    int ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &transaction);
    if(ret < 0) {
        perror("IO");
        return -1;
    }

    memcpy(data, &rx[1], len);
    return 0;
}

int nrf24_set_register(nrf24_t *dev, uint8_t reg, uint8_t *data, uint8_t len) {
    struct spi_ioc_transfer	transaction;
    memset(&transaction, 0, sizeof(transaction));
}