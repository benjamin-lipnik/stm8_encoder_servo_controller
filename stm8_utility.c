#include "stm8_utility.h"

void util_delay_milliseconds(uint8_t milliseconds) {
    for(uint8_t m = 0; m < milliseconds; m++) {
        for(uint8_t i = 0; i < 160; i++) {
          for(uint8_t j = 0; j < 10; j++) {
            __asm__("nop");
            __asm__("nop");
            __asm__("nop");
            __asm__("nop");
          }
        }
    }
}

void util_print_in_bin(uint8_t value) {
  for(uint8_t i = 0; i < 8; i++) {
    uart_write(48 + ((value&_BV(i))>0));
  }
}
