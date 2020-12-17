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
    
    int mem_fd;
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        perror("GPIO");
        return -1;
    }

    void *gpio_map;
    /* mmap GPIO */
    gpio_map = mmap(
        NULL,             //Any adddress in our space will do
        BLOCK_SIZE,       //Map length
        PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
        MAP_SHARED,       //Shared with other processes
        mem_fd,           //File to map
        GPIO_BASE         //Offset to GPIO peripheral
    );

    close(mem_fd); //No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", (int)gpio_map);//errno also set!
        return -1;
    }

    dev->gpio = (volatile unsigned *)gpio_map;

    INP_GPIO(dev->gpio, ce_io_num);
    OUT_GPIO(dev->gpio, ce_io_num);

    GPIO_CLR(dev->gpio) = 1<<ce_io_num;

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

    uint8_t tx[len+1];

    memset(tx, 0, len+1);

    tx[0] = NRF24_CMD_W_REGISTER | (reg & NRF24_REGISTER_MASK);
    memcpy(&tx[1], data, len);

    transaction.tx_buf = (uint64_t)tx;
    transaction.len = len+1;

    int ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &transaction);
    if(ret < 0) {
        perror("IO");
        return -1;
    }

    return 0;
}

int nrf24_flush_tx(nrf24_t *dev) {
    struct spi_ioc_transfer	transaction;
    memset(&transaction, 0, sizeof(transaction));

    uint8_t tx = NRF24_CMD_FLUSH_TX;

    transaction.tx_buf = (uint64_t)&tx;
    
    transaction.len = 1;

    int ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &transaction);
    if(ret < 0) {
        perror("IO");
        return -1;
    }

    return 0;
}

int nrf24_flush_rx(nrf24_t *dev) {
    struct spi_ioc_transfer	transaction;
    memset(&transaction, 0, sizeof(transaction));

    uint8_t tx = NRF24_CMD_FLUSH_RX;

    transaction.tx_buf = (uint64_t)&tx;
    
    transaction.len = 1;

    int ret = ioctl(dev->fd, SPI_IOC_MESSAGE(1), &transaction);
    if(ret < 0) {
        perror("IO");
        return -1;
    }

    return 0;
}

int nrf24_power_up_tx(nrf24_t *dev) {
    uint8_t config;
    nrf24_get_register(dev, NRF24_REG_CONFIG, &config, 1);
    config = config & (~NRF24_MASK_PRIM_RX); // Set to PTX mode
    config = config | NRF24_MASK_PWR_UP; // Power on
    nrf24_set_register(dev, NRF24_REG_CONFIG, &config, 1);
    nrf24_flush_tx(dev);
    GPIO_SET(dev->gpio) = 1<<dev->ce_io_num;
    return 0;
}

int nrf24_power_up_rx(nrf24_t *dev) {
    uint8_t config;
    nrf24_get_register(dev, NRF24_REG_CONFIG, &config, 1);
    config = config | NRF24_MASK_PRIM_RX; // Set to PTX mode
    config = config | NRF24_MASK_PWR_UP; // Power on
    nrf24_set_register(dev, NRF24_REG_CONFIG, &config, 1);
    nrf24_flush_rx(dev);
    GPIO_SET(dev->gpio) = 1<<dev->ce_io_num;
    return 0;
}

int nrf24_power_down(nrf24_t *dev) {
    GPIO_CLR(dev->gpio) = 1<<dev->ce_io_num;
    uint8_t config;
    nrf24_get_register(dev, NRF24_REG_CONFIG, &config, 1);
    config = config & (~NRF24_MASK_PWR_UP); // Power on
    nrf24_set_register(dev, NRF24_REG_CONFIG, &config, 1);
    return 0;
}

int nrf24_set_data_rate(nrf24_t *dev, enum nrf24_data_rate_t rate) {
    uint8_t rf_setup;
    nrf24_get_register(dev, NRF24_REG_RF_SETUP, &rf_setup, 1);
    
    switch (rate)
    {
        case NRF24_250KBPS:
            rf_setup = rf_setup | (NRF24_MASK_RF_DR_LOW);
            rf_setup = rf_setup & (~NRF24_MASK_RF_DR_HIGH);
            break;

        case NRF24_1MBPS:
            rf_setup = rf_setup & (~NRF24_MASK_RF_DR_LOW);
            rf_setup = rf_setup & (~NRF24_MASK_RF_DR_HIGH);
            break;

        case NRF24_2MBPS:
            rf_setup = rf_setup & (~NRF24_MASK_RF_DR_LOW);
            rf_setup = rf_setup | (NRF24_MASK_RF_DR_HIGH);
            break;

        default:
            printf("Ivalid data rate option, valid options are 250Kbps, 1Mbps, and 2Mbps.\n");
            return -1;
            break;
    }

    nrf24_set_register(dev, NRF24_REG_RF_SETUP, &rf_setup, 1);
    return 0;
}

int nrf24_set_crc(nrf24_t *dev, enum nrf24_crc_t crc) {
    uint8_t config;
    nrf24_get_register(dev, NRF24_REG_CONFIG, &config, 1);
    
    switch (crc)
    {
        case NRF24_CRC_DISABLED:
            config = config & (~NRF24_MASK_EN_CRC);
            break;

        case NRF24_CRC_1BYTE:
            config = config | NRF24_MASK_EN_CRC;
            config = config & (~NRF24_MASK_CRCO);
            break;

        case NRF24_CRC_2BYTES:
            config = config | NRF24_MASK_EN_CRC;
            config = config | NRF24_MASK_CRCO;
            break;
    
        default:
            printf("Invalid CRC option, valid options are Disabled, 1 Byte, and 2 Bytes.\n");
            return -1;
            break;
    }

    nrf24_set_register(dev, NRF24_REG_CONFIG, &config, 1);
    return 0;
}

int nrf24_set_rf_channel(nrf24_t *dev, uint8_t channel) {
    if(channel > 125) {
        printf("Unsupported channel, maximum channel number supported is 125.\n");
        return -1; 
    }
    
    uint8_t rf_ch;
    nrf24_get_register(dev, NRF24_REG_RF_CH, &rf_ch, 1);

    rf_ch = channel & NRF24_MASK_RF_CH;

    nrf24_set_register(dev, NRF24_REG_RF_CH, &rf_ch, 1);
    return 0;
}

int nrf24_enable_rx_pipe(nrf24_t *dev, enum nrf24_data_pipe_t pipe) {
    uint8_t en_rxaddr;
    nrf24_get_register(dev, NRF24_REG_EN_RXADDR, &en_rxaddr, 1);

    switch (pipe)
    {
        case NRF24_P0:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_P0;
            break;
        
        case NRF24_P1:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_P1;
            break;

        case NRF24_P2:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_P2;
            break;

        case NRF24_P3:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_P3;
            break;

        case NRF24_P4:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_P4;
            break;

        case NRF24_P5:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_P5;
            break;

        case NRF24_ALL_PIPES:
            en_rxaddr = en_rxaddr | NRF24_MASK_ERX_ALL;
            break;
    
        default:
            printf("Invalid pipe, valid pipes are P0-P5 and all pipes.\n");
            return -1;
            break;
    }

    nrf24_set_register(dev, NRF24_REG_EN_RXADDR, &en_rxaddr, 1);
    return 0;
}

int nrf24_disable_rx_pipe(nrf24_t *dev, enum nrf24_data_pipe_t pipe) {
    uint8_t en_rxaddr;
    nrf24_get_register(dev, NRF24_REG_EN_RXADDR, &en_rxaddr, 1);

    switch (pipe)
    {
        case NRF24_P0:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_P0);
            break;
        
        case NRF24_P1:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_P1);
            break;

        case NRF24_P2:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_P2);
            break;

        case NRF24_P3:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_P3);
            break;

        case NRF24_P4:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_P4);
            break;

        case NRF24_P5:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_P5);
            break;

        case NRF24_ALL_PIPES:
            en_rxaddr = en_rxaddr & (~NRF24_MASK_ERX_ALL);
            break;
    
        default:
            printf("Invalid pipe, valid pipes are P0-P5 and all pipes.\n");
            return -1;
            break;
    }

    nrf24_set_register(dev, NRF24_REG_EN_RXADDR, &en_rxaddr, 1);
    return 0;
}

void nrf24_flip_bytes(uint8_t *data, size_t len) {
    uint8_t temp;
    for(int i = 0; i < len/2; i++) {
        temp = data[i];
        data[i] = data[(len-1)-i];
        data[(len-1)-i] = temp;
    }
}

int nrf24_set_rx_address(nrf24_t *dev, enum nrf24_data_pipe_t pipe, uint8_t *address, uint8_t address_length) {
    if(address_length > 5 || address_length < 3) {
        printf("Invalid address length, valid address lengths are 3-5.\n");
        return -1;
    }
    
    nrf24_flip_bytes(address, address_length);
    
    if(pipe == NRF24_P0) {
        nrf24_set_register(dev, NRF24_REG_RX_ADDR_P0, address, address_length);
    }
    
    else if(pipe == NRF24_P1) {
        nrf24_set_register(dev, NRF24_REG_RX_ADDR_P1, address, address_length);
    }
    
    else {
        uint8_t rx_addr_p1[address_length];
        nrf24_get_register(dev, NRF24_REG_RX_ADDR_P1, rx_addr_p1, address_length);
        memcpy(&address[1], &rx_addr_p1[1], address_length-1);
        nrf24_set_register(dev, NRF24_REG_RX_ADDR_P1, rx_addr_p1, address_length);

        switch (pipe)
        {
            case NRF24_P2:
                nrf24_set_register(dev, NRF24_REG_RX_ADDR_P2, &address[0], 1);
                break;

            case NRF24_P3:
                nrf24_set_register(dev, NRF24_REG_RX_ADDR_P3, &address[0], 1);
                break;
            
            case NRF24_P4:
                nrf24_set_register(dev, NRF24_REG_RX_ADDR_P4, &address[0], 1);
                break;

            case NRF24_P5:
                nrf24_set_register(dev, NRF24_REG_RX_ADDR_P5, &address[0], 1);
                break;
            
            default:
                printf("Invalid pipe, valid pipes are P0-P5.\n");
                return -2;
                break;
        }
    }
    return 0;
}

int nrf24_set_tx_address(nrf24_t *dev, uint8_t *address, uint8_t address_length) {
    if(address_length > 5 || address_length < 3) {
        printf("Invalid address length, valid address lengths are 3-5.\n");
        return -1;
    }
    
    nrf24_flip_bytes(address, address_length);

    nrf24_set_register(dev, NRF24_REG_RX_ADDR_P0, address, address_length);
    nrf24_set_register(dev, NRF24_REG_TX_ADDR, address, address_length);
    return 0;
}

int nrf24_set_payload_length(nrf24_t *dev, uint8_t length) {
    if(length > 32) {
        printf("Invalid payload length, valid lengths are 0-32 (0 being dynamic payload length).\n");
        return -1;
    }

    if(length == 0) {
        uint8_t features;
        nrf24_get_register(dev, NRF24_REG_FEATURE, &features, 1);
        features = features | NRF24_MASK_EN_DPL;
        nrf24_set_register(dev, NRF24_REG_FEATURE, &features, 1);
        uint8_t dynpd = 0b00111111;
        nrf24_set_register(dev, NRF24_REG_DYNPD, &dynpd, 1);
    } else {
        uint8_t features;
        nrf24_get_register(dev, NRF24_REG_FEATURE, &features, 1);
        features = features & (~NRF24_MASK_EN_DPL);
        nrf24_set_register(dev, NRF24_REG_FEATURE, &features, 1);
        uint8_t dynpd = 0;
        nrf24_set_register(dev, NRF24_REG_DYNPD, &dynpd, 1);
        length = length & NRF24_MASK_RX_PW_P; // Technically this isn't needed because of the if at the beginining, but just in case
        nrf24_set_register(dev, NRF24_REG_RX_PW_P0, &length, 1);
        nrf24_set_register(dev, NRF24_REG_RX_PW_P1, &length, 1);
        nrf24_set_register(dev, NRF24_REG_RX_PW_P2, &length, 1);
        nrf24_set_register(dev, NRF24_REG_RX_PW_P3, &length, 1);
        nrf24_set_register(dev, NRF24_REG_RX_PW_P4, &length, 1);
        nrf24_set_register(dev, NRF24_REG_RX_PW_P5, &length, 1);
    }
    return 0;
}