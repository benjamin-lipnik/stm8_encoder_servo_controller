#ifndef STM8_UTIL_H
#define STM8_UTIL_H

#include "stm8s003.h"
#include "uart.h"

void util_delay_milliseconds(uint8_t milliseconds);
void util_print_in_bin(uint8_t value);

#endif
