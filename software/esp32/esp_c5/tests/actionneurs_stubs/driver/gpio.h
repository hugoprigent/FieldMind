#pragma once
#include <Arduino.h>
using gpio_num_t=int;
inline void gpio_set_level(gpio_num_t pin, int value) { niveaux[pin]=value; }
