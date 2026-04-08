/*
 * lora.c - LoRa Telemetry Driver for RFM95W on STM32H7 M4 Core
 *
 * HARDWARE SETUP
 * 
 * RFM95W connected to M4 core via SPI1
 * DIO0 pin connected to a free GPIO configured as EXTI (external interrupt)
 * CS pin managed manually in software (HAL does not auto-manage CS)
 *
 * Pin connections needed:
 *   RFM95W MOSI  -> SPI1 MOSI  PB5
 *   RFM95W MISO  -> SPI1 MISO  PA6
 *   RFM95W SCK   -> SPI1 SCK   PA5
 *   RFM95W NSS   -> GPIO output (CS, defined as LORA_CS_PIN)   PD14
 *   RFM95W DIO0  -> GPIO EXTI input (defined as LORA_DIO0_PIN)
 *   RFM95W RESET -> GPIO output (optional but recommended)
 *
 * CONFIGURATION
 * -----------------------------------------------------------------------------
 * Frequency:      915 MHz (US ISM band, no duty cycle restriction)
 * Spreading Factor: SF8
 * Bandwidth:      125 kHz
 * Coding Rate:    4/5
 * Preamble:       12 symbols (default)
 * Header mode:    Explicit
 * CRC:            Enabled
 * TX Power:       17 dBm (well under 30 dBm FCC limit)
 *
 * These settings give approximately 10-20km range on open water at ~1Hz
 * update rate, with roughly 70ms airtime per packet (~7% duty cycle)
 *
 * PACKET STRUCTURE
 * -----------------------------------------------------------------------------
 * We send a fixed TelemetryPacket_t struct once per second containing:
 *   float    lat          4 bytes   GPS latitude
 *   float    lon          4 bytes   GPS longitude
 *   int16_t  heading      2 bytes   degrees
 *   int16_t  roll         2 bytes   degrees * 100
 *   int16_t  pitch        2 bytes   degrees * 100
 *   uint8_t  battery      1 byte    percentage 0-100
 *   int16_t  wind_speed   2 bytes   knots * 10
 *   int16_t  wind_dir     2 bytes   degrees
 *                        ----------
 *   TOTAL                19 bytes
 *
 * In explicit header mode the receiver learns the payload length from the
 * header automatically so no implicit mode configuration is needed.
 * CRC is enabled so corrupted packets are flagged and discarded.
 *
 * STATE MACHINE
 * -----------------------------------------------------------------------------
 * The driver runs as a simple non-blocking state machine, ticked from the
 * main loop. No RTOS required on the M4 bare metal core.
 *
 *                    LoRa_Tick() called at 1Hz
 *                           |
 *                           v
 *              +------------+------------+
 *              |          IDLE           |
 *              +------------+------------+
 *                           |
 *                    load FIFO, trigger TX
 *                           |
 *                           v
 *              +------------+------------+
 *              |            TX           |
 *              +------------+------------+
 *                           |
 *                    wait for DIO0 EXTI
 *                           |
 *                           v
 *              +------------+------------+
 *              |        WAIT_TX_DONE     |
 *              +------------+------------+
 *                           |
 *               +-----------+-----------+
 *               |                       |
 *        no key received          key received
 *               |                       |
 *               v                       v
 *             IDLE           +----------+----------+
 *                            |          RX          |
 *                            +----------+----------+
 *                                       |
 *                             wait for DIO0 EXTI
 *                                       |
 *                                       v
 *                            +----------+----------+
 *                            |       WAIT_RX        |
 *                            +----------+----------+
 *                                       |
 *                             read FIFO, process
 *                                       |
 *                                       v
 *                            +----------+----------+
 *                            |   PROCESS_COMMAND   |
 *                            +----------+----------+
 *                                       |
 *                                     IDLE
 *
 * DIO0 fires an EXTI interrupt in both TX done and RX done conditions.
 * The ISR sets a flag, the state machine checks and advances on next tick.
 *
 * SPI TRANSACTIONS
 * -----------------------------------------------------------------------------
 * The RFM95W SPI protocol works as follows:
 *   - CS low to begin transaction
 *   - First byte: address | 0x80 for write, address & 0x7F for read
 *   - Subsequent bytes: data (auto-increments through FIFO if writing RegFifo)
 *   - CS high to end transaction
 *
 * Single register write:   CS low | addr|0x80 | value      | CS high
 * Single register read:    CS low | addr&0x7F | dummy byte | CS high
 * FIFO burst write:        CS low | 0x80      | byte0 .. byteN | CS high
 * FIFO burst read:         CS low | 0x00      | dummy*N    | CS high
 *
 * INITIALISATION SEQUENCE
 * -----------------------------------------------------------------------------
 * The following steps must be performed in order on startup:
 *
 *   1. Reset the RFM95W (pull RESET low for x  , then high, wait 5ms)
 *   2. Verify chip by reading RegVersion (addr 0x42), expect 0x12
 *   3. Put chip in SLEEP mode (RegOpMode = 0x00) - registers can only be
 *      written in Sleep or Standby mode
 *   4. Enable LoRa mode (RegOpMode = 0x80) - must be done in sleep
 *   5. Set frequency to 915 MHz by writing RegFrfMsb/Mid/Lsb (0x06/07/08)
 *      Frf = 915000000 / 61.035 = 14991360 = 0xE4C000
 *      RegFrfMsb = 0xE4, RegFrfMid = 0xC0, RegFrfLsb = 0x00
 *   6. Set TX power (RegPaConfig 0x09)
 *      Use PA_BOOST pin: 0x80 | (power - 2) for 2-17 dBm
 *      For 17 dBm: 0x8F
 *   7. Set modem config (RegModemConfig1 0x1D):
 *      BW=125kHz, CR=4/5, explicit header
 *      Value: 0x72
 *   8. Set modem config (RegModemConfig2 0x1E):
 *      SF=8, TX continuous off, CRC on
 *      Value: 0x84
 *   9. Set modem config (RegModemConfig3 0x26):
 *      Low data rate optimize off (only needed for SF11/SF12), AGC on
 *      Value: 0x04
 *  10. Set preamble length (RegPreambleMsb/Lsb 0x20/0x21)
 *      Default 12 symbols: MSB=0x00, LSB=0x0C
 *  11. Set FIFO TX base address (RegFifoTxBaseAddr 0x0E) = 0x00
 *  12. Set FIFO RX base address (RegFifoRxBaseAddr 0x0F) = 0x00
 *      (using full FIFO for each since we only do one at a time)
 *  13. Put chip in Standby mode (RegOpMode = 0x81)
 *  14. Configure DIO0 to trigger on TxDone (RegDioMapping1 0x40 = 0x40)
 *      DIO0 mapping: 01 = TxDone in TX mode, 00 = RxDone in RX mode
 *      Switch this mapping when you switch between TX and RX modes
 *
 * TRANSMIT SEQUENCE
 * -----------------------------------------------------------------------------
 *   1. Put chip in Standby (RegOpMode = 0x81)
 *   2. Set FifoAddrPtr to TxBaseAddr (write 0x00 to RegFifoAddrPtr 0x0D)
 *   3. Write payload bytes to RegFifo (0x00) in a burst SPI transaction
 *   4. Set PayloadLength (RegPayloadLength 0x22) to number of bytes written
 *   5. Set DIO0 mapping to TxDone (RegDioMapping1 0x40 = 0x40)
 *   6. Put chip in TX mode (RegOpMode = 0x83)
 *   7. Wait for DIO0 EXTI interrupt (ISR sets dio0_flag)
 *   8. In state machine: clear IRQ flags (RegIrqFlags 0x12 = 0xFF)
 *   9. Advance state: check if key was received flag is set, go to RX or IDLE
 *
 * RECEIVE SEQUENCE (triggered by key detection)
 * -----------------------------------------------------------------------------
 *   1. Put chip in Standby (RegOpMode = 0x81)
 *   2. Set FifoAddrPtr to RxBaseAddr (write 0x00 to RegFifoAddrPtr 0x0D)
 *   3. Set DIO0 mapping to RxDone (RegDioMapping1 0x40 = 0x00)
 *   4. Put chip in continuous RX mode (RegOpMode = 0x85)
 *   5. Wait for DIO0 EXTI interrupt
 *   6. In state machine: check IRQ flags for RxDone and CRC error
 *      RegIrqFlags 0x12: bit 6 = RxDone, bit 5 = PayloadCrcError
 *   7. If CRC error: discard, clear flags, go back to IDLE
 *   8. If RxDone and no CRC error:
 *      a. Read RegFifoRxCurrentAddr (0x10) to find packet location
 *      b. Write that value to RegFifoAddrPtr (0x0D)
 *      c. Read RegRxNbBytes (0x13) to get packet length
 *      d. Burst read that many bytes from RegFifo (0x00)
 *   9. Process command bytes
 *  10. Clear IRQ flags, return to IDLE
 *
 * KEY DETECTION LOGIC
 * -----------------------------------------------------------------------------
 * The ground station sends a known key before any command. This key is a
 * fixed byte sequence agreed upon by both sides. During normal WAIT_TX_DONE
 * state after each transmission, the chip briefly listens for this key.
 *
 * Suggested approach: after TX done, drop into RX for a short listen window
 * (e.g. 500ms), if nothing arrives go back to IDLE for next 1Hz tick.
 * If the key is detected, stay in RX and wait for the full command packet.
 *
 * Define the key as something like:
 *   #define LORA_CMD_KEY  { 0xAB, 0xCD, 0x12, 0x34 }
 *   #define LORA_CMD_KEY_LEN  4
 *
 * IMPORTANT NOTES
 * -----------------------------------------------------------------------------
 * - Always clear IRQ flags (write 0xFF to RegIrqFlags) after handling them
 *   or the chip will not fire DIO0 again
 * - Always return to Standby before switching between TX and RX modes
 * - Registers can only be written in Sleep or Standby, not during TX/RX
 * - The FIFO is cleared on entry to Sleep mode, do not sleep between
 *   loading FIFO and transmitting
 * - DIO0 mapping must be changed when switching between TX and RX modes
 * - Both sides must use identical SF, BW, CR and frequency or they cannot
 *   hear each other
 *
 * =============================================================================
 * 
 * 
 * VERY BASIC IMPLEMENTATION TO START
 * 
    1. Reset the RFM95
    2. Verify it's alive (read version register, expect 0x12)
    3. Put it in Sleep mode
    4. Enable LoRa mode
    5. Configure registers (frequency, SF, BW, CR, power)
    6. Put it in Standby
    7. Load data into FIFO
    8. Fire TX
    9. Poll until TX done
    10. Repeat
 */


 /* ─── Register addresses ─────────────────────────────────────────── */
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_FIFO_ADDR_PTR        0x0D
#define REG_FIFO_TX_BASE_ADDR    0x0E
#define REG_FIFO_RX_BASE_ADDR    0x0F
#define REG_IRQ_FLAGS            0x10
#define REG_TX_CFG               0x16
#define REG_PAYLOAD_LENGTH       0x17
#define REG_NB_RX_BYTES          0x1D
#define REG_RX_HEADER_INFO       0x1E
#define REG_RX_DATA_ADDR         0x26
#define REG_IRQ_FLAGS            0x10
#define REG_TX_CFG               0x16
#define REG_PAYLOAD_LENGTH       0x17
#define REG_NB_RX_BYTES          0x1D
#define REG_RX_HEADER_INFO       0x1E
#define REG_RX_DATA_ADDR         0x26
#define REG_VERSION              0x42

/* Helper functions */

#define CS_LOW()   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
#define RESET_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET)  // adjust pin
#define RESET_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET)    // adjust pin


/*
   Write a single byte to the given register address
*/
static void SPI_tx_byte(uint8_t addr, uint8_t val)
{
    uint8_t buf[2];
    buf[0] = addr | 0x80;   /* set bit 7 to indicate write */
    buf[1] = val;

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

/*
   Read a single byte from the given register address
*/
static uint8_t SPI_rx_byte(uint8_t addr)
{
    uint8_t tx[2], rx[2];
    tx[0] = addr & 0x7F;    /* clear bit 7 to indicate read */
    tx[1] = 0x00;           /* dummy byte to clock out the response */

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_HIGH();

    return rx[1];           /* first byte is garbage, second is the data */
}

/*
   Write multiple bytes to the RFM95W FIFO starting at the given address
*/
static void SPI_FIFO_tx(uint8_t *data, uint8_t len)
{
    uint8_t addr = REG_FIFO | 0x80;    /* FIFO register with write bit set */

    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);  /* send address */
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY); /* send all bytes */
    CS_HIGH();
}