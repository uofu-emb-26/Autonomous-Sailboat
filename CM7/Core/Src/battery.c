#include "main.h"
#include "battery.h"

// PA0/PA4/PA5/PA6 = ADC1 channels 16/18/19/3, wired to balance-lead taps B1..B4
// (cumulative from pack negative: B1=cell1, B2=cell1+2, B3=cell1+2+3, B4=full pack)
// PA1 is not used: it's physically tied to the Nucleo's on-board Ethernet PHY
// REF_CLK, which injects noise regardless of firmware ETH usage.
#define BATTERY_CELL_COUNT          4u
#define BATTERY_SAMPLE_PERIOD_MS    500u
// RC filter tau = (82k || 18k) * 100nF ~= 1.48ms; 10ms is ~6.8 tau (>99.8% settled)
#define BATTERY_SETTLE_MS           10u
// Number of raw conversions averaged per channel each cycle (integer average, no floats)
#define BATTERY_AVG_SAMPLES         32u

// Fixed-point conversion constants (no floating point)
#define BATTERY_ADC_VREF_MV         3300
#define BATTERY_ADC_FULL_SCALE      65535
// Divider: Vtap = Vpin * (R_TOP+R_BOTTOM)/R_BOTTOM = Vpin * (82k+18k)/18k
#define BATTERY_DIVIDER_NUM         100
#define BATTERY_DIVIDER_DEN         18

// PD0 gates Q1, which ties the divider network's low side to GND when driven high
#define BATTERY_ENABLE_PORT         GPIOD
#define BATTERY_ENABLE_PIN          GPIO_PIN_0

TaskHandle_t task_battery;

static ADC_HandleTypeDef hadc1;

static const uint32_t battery_adcChannels[BATTERY_CELL_COUNT] =
{
    ADC_CHANNEL_16, // PA0 - B1
    ADC_CHANNEL_18, // PA4 - B2
    ADC_CHANNEL_19, // PA5 - B3
    ADC_CHANNEL_3,  // PA6 - B4
};

static uint16_t battery_rawTaps[BATTERY_CELL_COUNT];

static void    battery_adcInit(void);
static void    battery_sampleTaps(void);
static int32_t battery_countsToTapMv(uint16_t counts);

/**
  * Initialize the hardware.
  */
void battery_hardwareInit()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = BATTERY_ENABLE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BATTERY_ENABLE_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BATTERY_ENABLE_PORT, BATTERY_ENABLE_PIN, GPIO_PIN_RESET);

    battery_adcInit();
}

static void battery_adcInit(void)
{
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = BATTERY_CELL_COUNT;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 1;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.LeftBitShift          = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode      = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    // Scan mode converts ranks 1..4 upward in a single start, one channel per rank,
    // so the sequencer configuration only needs to happen once here.
    static const uint32_t ranks[BATTERY_CELL_COUNT] =
    {
        ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3, ADC_REGULAR_RANK_4
    };

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.SamplingTime           = ADC_SAMPLETIME_64CYCLES_5;
    sConfig.SingleDiff             = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber           = ADC_OFFSET_NONE;
    sConfig.Offset                 = 0;
    sConfig.OffsetRightShift       = DISABLE;
    sConfig.OffsetSignedSaturation = DISABLE;

    for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        sConfig.Channel = battery_adcChannels[i];
        sConfig.Rank    = ranks[i];
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

// Reads BATTERY_AVG_SAMPLES scans of the 4 balance-tap channels and averages
// each channel into battery_rawTaps
static void battery_sampleTaps(void)
{
    uint32_t sums[BATTERY_CELL_COUNT] = {0};

    for (uint32_t s = 0; s < BATTERY_AVG_SAMPLES; s++)
    {
        HAL_ADC_Start(&hadc1);
        for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
        {
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            sums[i] += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
    {
        battery_rawTaps[i] = (uint16_t)(sums[i] / BATTERY_AVG_SAMPLES);
    }
}

// Raw ADC count on one tap -> millivolts at the battery tap (before the divider), fixed-point only
static int32_t battery_countsToTapMv(uint16_t counts)
{
    int32_t pin_mv = ((int32_t)counts * BATTERY_ADC_VREF_MV) / BATTERY_ADC_FULL_SCALE;
    return (pin_mv * BATTERY_DIVIDER_NUM) / BATTERY_DIVIDER_DEN;
}

void battery_handler(void *argument)
{
    for(;;)
    {
        HAL_GPIO_WritePin(BATTERY_ENABLE_PORT, BATTERY_ENABLE_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(BATTERY_SETTLE_MS));

        battery_sampleTaps();

        HAL_GPIO_WritePin(BATTERY_ENABLE_PORT, BATTERY_ENABLE_PIN, GPIO_PIN_RESET);

        int32_t cell_mv[BATTERY_CELL_COUNT];
        int32_t tap_mv  = 0;
        int32_t prev_mv = 0;
        for (uint32_t i = 0; i < BATTERY_CELL_COUNT; i++)
        {
            tap_mv = battery_countsToTapMv(battery_rawTaps[i]);

            int32_t diff = tap_mv - prev_mv;
            cell_mv[i] = (diff < 0) ? 0 : diff; // noise near a tap boundary can otherwise go negative

            prev_mv = tap_mv;
        }

        printf("Battery: %ld [ %ld (%u), %ld (%u), %ld (%u), %ld (%u) ]\r\n",
               (long)tap_mv,
               (long)cell_mv[0], battery_rawTaps[0],
               (long)cell_mv[1], battery_rawTaps[1],
               (long)cell_mv[2], battery_rawTaps[2],
               (long)cell_mv[3], battery_rawTaps[3]);

        vTaskDelay(pdMS_TO_TICKS(BATTERY_SAMPLE_PERIOD_MS));
    }
}