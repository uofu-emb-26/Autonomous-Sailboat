
// void task_PWM_SERVO(void *argument)
// {
//   // 20ms = 2000 ticks at 1mhz
//   // 0.5 ms pulse is right
//   // 1.5 ms pulse is center (slightly off kilter by 30 degeres)
//   // 2.5 ms pulse is fully left
//   // Servo has a period of 2ms 
//   // and the duty cycle is between 0.5 and 2.5ms
//   // Every 20 ms (50hz) we send pulse
//   // .5 translates to 500 ticks
// __HAL_RCC_GPIOE_CLK_ENABLE();
// }