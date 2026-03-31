# RFM95W Peripheral Setup — STM32H755 M4 Core

This document describes how to configure the STM32H755 M4 core peripherals to drive the RFM95W LoRa module via SPI4. No CubeMX needed — all initialization is done manually with HAL calls in `CM4/Core/Src/main.c`.

---

## Pin Assignments (Morpho CN10)

| Signal   | Pin  | Direction         | Mode             |
|----------|------|-------------------|------------------|
| SPI4_SCK | PE12 | Output (AF)       | AF5, push-pull   |
| SPI4_MISO| PE13 | Input (AF)        | AF5              |
| SPI4_MOSI| PE14 | Output (AF)       | AF5, push-pull   |
| CS       | PE11 | Output GPIO       | Push-pull, idle HIGH |
| RESET    | PE10 | Output GPIO       | Push-pull, active LOW |
| DIO0     | PE9  | Input GPIO + EXTI | Rising edge interrupt |

All pins are on GPIOE — one clock enable covers all of them.

---

## Step 1 — Enable Clocks

```c
__HAL_RCC_GPIOE_CLK_ENABLE();
__HAL_RCC_SPI4_CLK_ENABLE();
```

---

## Step 2 — Configure GPIO Pins

### SPI4 Alternate Function Pins (PE12, PE13, PE14)

```c
GPIO_InitTypeDef gpio = {0};
gpio.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
gpio.Mode      = GPIO_MODE_AF_PP;
gpio.Pull      = GPIO_NOPULL;
gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
gpio.Alternate = GPIO_AF5_SPI4;
HAL_GPIO_Init(GPIOE, &gpio);
```

### CS — PE11 (start HIGH = deselected)

```c
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

gpio.Pin   = GPIO_PIN_11;
gpio.Mode  = GPIO_MODE_OUTPUT_PP;
gpio.Pull  = GPIO_NOPULL;
gpio.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOE, &gpio);
```

### RESET — PE10 (start HIGH = not in reset)

```c
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);

gpio.Pin   = GPIO_PIN_10;
gpio.Mode  = GPIO_MODE_OUTPUT_PP;
gpio.Pull  = GPIO_NOPULL;
gpio.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOE, &gpio);
```

### DIO0 — PE9 (rising edge EXTI)

```c
gpio.Pin  = GPIO_PIN_9;
gpio.Mode = GPIO_MODE_IT_RISING;
gpio.Pull = GPIO_PULLDOWN;
HAL_GPIO_Init(GPIOE, &gpio);
```

---

## Step 3 — Initialize SPI4

SPI4 sits on APB2, which runs at **64 MHz** on the M4. A prescaler of 32 gives **2 MHz** — well within the RFM95W's 10 MHz limit and conservative enough for breadboard wiring.

```c
SPI_HandleTypeDef hspi4 = {0};

hspi4.Instance               = SPI4;
hspi4.Init.Mode              = SPI_MODE_MASTER;
hspi4.Init.Direction         = SPI_DIRECTION_2LINES;
hspi4.Init.DataSize          = SPI_DATASIZE_8BIT;
hspi4.Init.CLKPolarity       = SPI_POLARITY_LOW;   // CPOL=0
hspi4.Init.CLKPhase          = SPI_PHASE_1EDGE;    // CPHA=0  →  SPI Mode 0
hspi4.Init.NSS               = SPI_NSS_SOFT;       // CS controlled manually
hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // 64MHz / 32 = 2MHz
hspi4.Init.FirstBit          = SPI_FIRSTBIT_MSB;
hspi4.Init.TIMode            = SPI_TIMODE_DISABLE;
hspi4.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
hspi4.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

HAL_SPI_Init(&hspi4);
```

> `hspi4` should be declared at file scope (not inside main) so the RFM95W driver can reference it as an extern.

---

## Step 4 — Enable DIO0 Interrupt (EXTI9)

DIO0 shares the EXTI9_5 IRQ line with PE5–PE9.

```c
HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
```

Add the IRQ handler in `stm32h7xx_it.c`:

```c
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}
```

And the callback (wherever you put application callbacks):

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_9)
    {
        // DIO0 fired — TX done or RX done depending on current RFM95W mode
        rfm95w_dio0_isr();  // your driver function
    }
}
```

---

## RFM95W SPI Protocol Summary

The RFM95W register interface works as follows:
- **Write:** assert CS low, send `(reg | 0x80)`, send data byte, deassert CS
- **Read:**  assert CS low, send `(reg & 0x7F)`, send `0x00` dummy, read response, deassert CS
- Address is always the first byte; bit 7 is the R/W flag (1=write, 0=read)

---

## Files to Create

```
CM4/Core/Src/rfm95w.c       — driver (register read/write, init, tx, rx)
CM4/Core/Inc/rfm95w.h       — public API + register defines
CM4/Core/Src/lora_packet.c  — packet encode/decode (telemetry + commands)
CM4/Core/Inc/lora_packet.h  — packet structs and API
```
