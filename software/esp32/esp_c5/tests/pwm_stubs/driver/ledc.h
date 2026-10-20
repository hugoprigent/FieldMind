#pragma once
#include <cstdint>
enum ledc_mode_t { LEDC_LOW_SPEED_MODE = 0 };
enum ledc_timer_t { LEDC_TIMER_0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3 };
uint32_t ledc_get_freq(ledc_mode_t mode, ledc_timer_t timer);
