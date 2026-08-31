#include "main.h"
#include "battery.h"

TaskHandle_t task_battery;

/**
  * Initialize the hardware.
  */
void battery_hardwareInit()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // ADC_HandleTypeDef hadc1;
    // hadc1.Instance = ADC1;
    // hadc1.Init.ClockPrescaler           = ADC_CLOCK_ASYNC_DIV4;   /* tune to your PER_CK */
    // hadc1.Init.Resolution               = ADC_RESOLUTION_16B;
    // hadc1.Init.ScanConvMode             = ADC_SCAN_ENABLE;
    // hadc1.Init.EOCSelection             = ADC_EOC_SEQ_CONV;
    // hadc1.Init.LowPowerAutoWait         = DISABLE;
    // hadc1.Init.ContinuousConvMode       = ENABLE;
    // hadc1.Init.NbrOfConversion          = NUM_CHANNELS;
    // hadc1.Init.DiscontinuousConvMode    = DISABLE;
    // hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    // hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    // hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    // hadc1.Init.Overrun                  = ADC_OVR_DATA_OVERWRITTEN;
    // hadc1.Init.LeftBitShift             = ADC_LEFTBITSHIFT_NONE;
    // hadc1.Init.OversamplingMode         = DISABLE;
    // if (HAL_ADC_Init(&hadc1) != HAL_OK)
    // {
    //     Error_Handler();
    // }

    // ADC_ChannelConfTypeDef sConfig = {0};
    // sConfig.Channel      = ADC_CHANNEL_15;    // matches the pin you set to analog
    // sConfig.Rank         = ADC_REGULAR_RANK_1;
    // sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    // sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    // sConfig.OffsetNumber = ADC_OFFSET_NONE;
    // sConfig.Offset       = 0;

    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    // {
    //     Error_Handler();
    // }
}

void battery_handler(void *argument)
{
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}