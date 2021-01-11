#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm8s003.h"
#include "uart.h"
#include "stm8_utility.h"

#define TONE_C6 956
#define TONE_D6 851
#define TONE_E6 758

#define TONE_C7 478

struct Settings{
  uint16_t pwm_input_max;
  uint16_t pwm_input_min;
  int16_t tick_180_deg;
  uint8_t measure_angle;
  uint8_t measure_pwm;
};

const struct Settings default_settings = {2010,994,5400,1,1};
struct Settings current_settings;

int putchar(int c) {
    uart_write(c);
    return 0;
}

//INPUTI
  //ENKODREJA
    //PORTB
//PB4, PB5 sta inputa enkoderja
#define ENC_PIN1 4
#define ENC_PIN2 5

//OUTPUTI
  //MOTOR 1
    //PORTD
      //DIR1 -> PD2
      //PWM1 -> PD3
#define DIR1_PIN 2
#define PWM1_PIN  3

void outputs_init() {
  PD_DDR |= _BV(DIR1_PIN) | _BV(PWM1_PIN);
  PD_CR1 |= _BV(DIR1_PIN) | _BV(PWM1_PIN);
}
void set_output1(uint8_t power, int8_t dir) {
  if(power == 0 || dir == 0) {
    TIM2_CCR2L = 0;
    PD_ODR &= ~_BV(DIR1_PIN);
  }
  else if(dir < 0) {
    PD_ODR &= ~_BV(DIR1_PIN);
    TIM2_CCR2L = power;
  }else {
    PD_ODR |= _BV(DIR1_PIN);
    TIM2_CCR2L = 255-power;
  }
}


const volatile int8_t rotacijska_tabela[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
volatile int16_t enc_cnt = 0;
volatile uint8_t last_ab = 0;

void enc_interrupt() __interrupt(EXTI1_ISR) {
  //PB_IDR
  uint8_t ab = !(PB_IDR & _BV(ENC_PIN2)) << 1 | !(PB_IDR & _BV(ENC_PIN1));
  enc_cnt += rotacijska_tabela[last_ab << 2 | ab];
  last_ab = ab;
}
void enc_inputs_init() {
  PB_DDR &= ~(_BV(ENC_PIN1) | _BV(ENC_PIN2));
  PB_CR2 |= _BV(ENC_PIN1) | _BV(ENC_PIN2);
}

volatile uint16_t delta = 0;
volatile uint16_t tmp = 0;

void input_capture() __interrupt(TIM1_CC_ISR) {
  uint8_t end = TIM1_SR1 & _BV(1);//CC1IF
  uint16_t ccr = TIM1_CCR1H << 8 | TIM1_CCR1L;

  if(PC_IDR & _BV(6)) {
    TIM1_CCER1 |= _BV(1);
    tmp = ccr;
  }
  else {
    delta = ccr - tmp;
    if(delta > 2050)
      delta = 2050;
    TIM1_CCER1 &= ~_BV(1);
  }
}

void timer1_init() {
  TIM1_IER |= _BV(1); //CC1IE
  TIM1_EGR |= _BV(1);//CC1G
  TIM1_CCMR1 |= _BV(0) | (0b1111<<4); //4x filter, TI1FP1
  TIM1_CCER1 |= _BV(0); //Enable capture
  TIM1_PSCRL |= 15;
  TIM1_CR1 |= _BV(TIM1_CR1_CEN) | _BV(TIM1_CR1_ARPE);
}

void timer2_init() {
  //Timer
  TIM2_PSCR = 0b11;

  TIM2_ARRH = 0;
  TIM2_ARRL = 255; //Max / Top

  //CH2

  //TIM2_IER |= _BV(TIM2_IER_UIE);
  TIM2_CR1 |= _BV(TIM2_CR1_CEN);

  TIM2_CCMR2 |=  _BV(5) | _BV(6); //pwm mode 1 //bita 0 in 1 moreta bit na nic ce ces met izhod
  TIM2_CCER1 |= _BV(4);

  TIM2_CCR2H = 0;
  TIM2_CCR2L = 0; //Duty

}
void tone(uint16_t t0, int16_t ms_duration) {
  //prvo shranimo nastavitve timerja2
  uint8_t pscr = TIM2_PSCR;
  uint16_t top = TIM2_ARRH << 8 | TIM2_ARRL;
  uint16_t duty = TIM2_CCR2H << 8 | TIM2_CCR2L;
  //shranimo nastavitev dir pina
  uint8_t dir_pin = PD_ODR & _BV(DIR1_PIN);

  //nastavimo nastavitve za zvok
  PD_ODR &= ~_BV(DIR1_PIN);
  TIM2_PSCR = 4; //16x division

  TIM2_ARRH = t0>>8;
  TIM2_ARRL = t0;

  uint16_t new_duty = t0>>3;
  TIM2_CCR2H = new_duty >> 8;
  TIM2_CCR2L = new_duty;

  //wait while the tone plays
  while(ms_duration-- > 1)
    util_delay_milliseconds(1);

  //restore settings
  TIM2_PSCR = pscr;
  TIM2_ARRH = top >> 8;
  TIM2_ARRL = top;
  TIM2_CCR2H = duty >> 8;
  TIM2_CCR2L = duty;
  PD_ODR |= dir_pin;

}

void timer4_init() {
  TIM4_CR1 |= _BV(0);//CEN
  TIM4_IER |= _BV(0); //UIE
  TIM4_PSCR = 0b101;
  TIM4_ARR = 255;
}

void find_zero(int8_t dir) {
  set_output1(255, dir);
  util_delay_milliseconds(100);

  //enc_cnt = 0;
  int16_t last_cnt = 0;
  uint8_t steps = 0;

  while(1) {
    if(abs(enc_cnt - last_cnt) < 5) {
        if(++steps >= 5) {
          set_output1(0,0);
          util_delay_milliseconds(100);
          //enc_cnt=0;
          return;
        }
    }
    else {
      steps = 0;
    }
    last_cnt = enc_cnt;
    util_delay_milliseconds(40);
  }

}

char * serial_reader() {
  static char str_buffer[51] = {0};
  static uint8_t buffer_index = 0;
  if(uart_available()) {
    char read = uart_read();
    if(read == '\n' || read == '\r' || read == ';') {
      if(buffer_index == 0)
        return 0;

      str_buffer[buffer_index] = 0;
      buffer_index = 0;
      return str_buffer;
    }
    str_buffer[buffer_index++] = read;
  }
  return 0;
}

int main () {
    CLK_CKDIVR = 0;//16mhz

    CPU_CCR |= _BV(5) | _BV(3);
    EXTI_CR1 = 0xff; //rising and falling edge for interrupts

    uart_init(9600);
    timer1_init();
    enc_inputs_init();
    enable_interrupts();
    outputs_init();
    timer2_init();
    PC_CR1 |= _BV(6); //PULLUP

    //start
    current_settings = default_settings;

    #define PAVZA 100
    #define NOTA 500

    tone(TONE_C6, NOTA);
    util_delay_milliseconds(PAVZA);
    tone(TONE_C6, NOTA);
    util_delay_milliseconds(PAVZA);

    if(current_settings.measure_pwm) {
      for(uint8_t j = 0; j < 3; j++) {
        tone(TONE_D6, NOTA/2);
        util_delay_milliseconds(PAVZA/2);
      }

      uint8_t m_low_steps = 0;
      uint8_t m_high_steps = 0;
      uint8_t measured = 0;

      while(measured != 0b11) {
        if(delta < 1500) {
          m_high_steps = 0;
          if(++m_low_steps == 20) {
            current_settings.pwm_input_min = delta;
            measured |= _BV(0);
            tone(TONE_D6, 1000);
          }
          util_delay_milliseconds(100);
        }
        else {
          m_low_steps = 0;
          if(++m_high_steps == 20) {
            current_settings.pwm_input_max = delta;
            measured |= _BV(1);
            tone(TONE_D6, 1000);
          }
          util_delay_milliseconds(100);
        }
      }
    }

    if(current_settings.measure_angle) {
      for(uint8_t i = 0; i < 3; i++) {
        tone(TONE_E6, NOTA/2);
        util_delay_milliseconds(PAVZA/2);
      }

      find_zero(-1);

      tone(TONE_E6, 1500);
      util_delay_milliseconds(100);

      enc_cnt = 0;
      find_zero(1);
      current_settings.tick_180_deg = enc_cnt;
    }
    float m = current_settings.tick_180_deg / 1000.0f;

    tone(TONE_C7, NOTA);
    util_delay_milliseconds(PAVZA);
    util_delay_milliseconds(PAVZA);

    float delta_align_k = (1000.0f/(current_settings.pwm_input_max-current_settings.pwm_input_min));
    float delta_align_n = 2000.0f - delta_align_k * current_settings.pwm_input_max;

    int16_t target = 0;

    while(1) {
      //util_delay_milliseconds(100);
      //printf("%u\n\r", delta);
      //printf("Hello world!\n\r");

      uint16_t error = abs(target - enc_cnt);

      if(error > 500) { //full power
        set_output1(255, (target > enc_cnt)*2-1);
      }
      else if(error > 20) {
        const float k = 0.22f;
        set_output1(k * error + 145.625f, (target > enc_cnt)*2-1);
      }
      else {
        set_output1(0,0);
      }

      //char * input = serial_reader();
      //if(input) {
      //  target = atoi(input);
      //}
      int16_t aligned_delta = delta_align_k * delta + delta_align_n;
      if(aligned_delta > 2000) aligned_delta = 2000;
      if(aligned_delta < 1000) aligned_delta = 1000;

      target = ((int16_t)aligned_delta-1000) * m;

      printf("e %d, pi %d, pa %d\n\r", enc_cnt, delta, aligned_delta);
    }
}
