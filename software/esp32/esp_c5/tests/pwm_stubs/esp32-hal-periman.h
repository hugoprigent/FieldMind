#pragma once
#include <cstdint>
enum peripheral_bus_type_t { ESP32_BUS_TYPE_LEDC };
void *perimanGetPinBus(uint8_t pin, peripheral_bus_type_t type);
