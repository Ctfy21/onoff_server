void initialize_ds3231();

void modifyTime(int* array_timeTemp, uint8_t hourStart, uint8_t minuteStart, uint8_t hourWorking, uint8_t minuteWorking, uint8_t hourSunset, uint8_t minuteSunset, uint8_t maxLingtning);

void prepareTime(int* array_time);

void timerLight(void *pvParameters);

void pwm_set_white_led(int duty);

void pwm_set_red_led(int duty);

