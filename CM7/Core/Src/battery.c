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
    // hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
    // hadc1.Init.Resolution            = ADC_RESOLUTION_16B;   // H7 supports up to 16-bit
    // hadc1.Init.ScanConvMode          = DISABLE;
    // hadc1.Init.ContinuousConvMode    = DISABLE;
    // hadc1.Init.DiscontinuousConvMode = DISABLE;
    // hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    // hadc1.Init.NbrOfConversion       = 1;
    // hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    // hadc1.Init.LowPowerAutoWait      = DISABLE;
    // hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    // hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
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