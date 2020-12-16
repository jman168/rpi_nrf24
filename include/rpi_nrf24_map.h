// Commands
#define NRF24_CMD_R_REGISTER          0b00000000
#define NRF24_CMD_W_REGISTER          0b00100000
#define NRF24_CMD_R_RX_PAYLOAD        0b01100001
#define NRF24_CMD_W_TX_PAYLOAD        0b10100000
#define NRF24_CMD_FLUSH_TX            0b11100001
#define NRF24_CMD_FLUSH_RX            0b11100010
#define NRF24_CMD_REUSE_TX_PL         0b11100011
#define NRF24_CMD_R_RX_PL_WID         0b01100000
#define NRF24_CMD_W_ACK_PAYLOAD       0b10101000
#define NRF24_CMD_W_TX_PAYLOAD_NOACK  0b10110000
#define NRF24_CMD_NOP                 0b11111111


// Registers
#define NRF24_REGISTER_MASK 0b00011111
#define NRF24_REG_CONFIG 0x00
#define NRF24_MASK_PRIM_RX (1<<0)
#define NRF24_MASK_PWR_UP (1<<1)
#define NRF24_MASK_CRCO (1<<2)
#define NRF24_MASK_EN_CRC (1<<3)
#define NRF24_MASK_MASK_MAX_RT (1<<4)
#define NRF24_MASK_MASK_TX_DS (1<<5)
#define NRF24_MASK_MASK_RX_DR (1<<6)

#define NRF24_REG_EN_AA 0x01

#define NRF24_REG_EN_RXADDR 0x02
#define NRF24_MASK_ERX_P0 (1<<0)
#define NRF24_MASK_ERX_P1 (1<<1)
#define NRF24_MASK_ERX_P2 (1<<2)
#define NRF24_MASK_ERX_P3 (1<<3)
#define NRF24_MASK_ERX_P4 (1<<4)
#define NRF24_MASK_ERX_P5 (1<<5)
#define NRF24_MASK_ERX_ALL (0b00111111)

#define NRF24_REG_SETUP_AW 0x03
#define NRF24_REG_SETUP_RETR 0x04

#define NRF24_REG_RF_CH 0x05
#define NRF24_MASK_RF_CH (0b01111111)

#define NRF24_REG_RF_SETUP 0x06
#define NRF24_MASK_RF_DR_HIGH (1<<3)
#define NRF24_MASK_RF_DR_LOW (1<<5)

#define NRF24_REG_STATUS 0x07
#define NRF24_MASK_TX_FULL (1<<0)
#define NRF24_MASK_RX_P_NO (0b111<<1)
#define NRF24_MASK_MAX_RT (1<<4)
#define NRF24_MASK_TX_DS (1<<5)
#define NRF24_MASK_RX_DR (1<<6)



#define NRF24_REG_OBSERVE_TX 0x08
#define NRF24_REG_RPD 0x09
#define NRF24_REG_RX_ADDR_P0 0x0A
#define NRF24_REG_RX_ADDR_P1 0x0B
#define NRF24_REG_RX_ADDR_P2 0x0C
#define NRF24_REG_RX_ADDR_P3 0x0D
#define NRF24_REG_RX_ADDR_P4 0x0E
#define NRF24_REG_RX_ADDR_P5 0x0F
#define NRF24_REG_TX_ADDR 0x10

#define NRF24_REG_RX_PW_P0 0x11
#define NRF24_REG_RX_PW_P1 0x12
#define NRF24_REG_RX_PW_P2 0x13
#define NRF24_REG_RX_PW_P3 0x14
#define NRF24_REG_RX_PW_P4 0x15
#define NRF24_REG_RX_PW_P5 0x16
#define NRF24_MASK_RX_PW_P 0b00111111


#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD 0x1C

#define NRF24_REG_FEATURE 0x1D
#define NRF24_MASK_EN_DYN_ACK (1<<0)
#define NRF24_MASK_EN_ACK_PAY (1<<1)
#define NRF24_MASK_EN_DPL (1<<2)