#pragma once
#include <cstdint>
#define SOC_LEDC_CHANNEL_NUM 6
struct ledc_channel_handle_t {
  uint8_t pin, channel, channel_resolution, timer_num;
  uint32_t freq_hz;
};
uint32_t ledcReadFreq(uint8_t pin);
