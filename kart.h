"""
Copyright 2020 - 2023 Julian Metzler

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

#include <Arduino.h>


#define PIN_HORN 2
#define PIN_BUZZER 7
#define PIN_UART_SEL 6
#define PIN_WS2812_DATA 8
#define PIN_LED 13

#define PIN_INPUT_CLOCK 5
#define PIN_INPUT_LOAD 4
#define PIN_INPUT_DATA 3

#define PIN_OUTPUT_CLOCK 12
#define PIN_OUTPUT_LOAD 11
#define PIN_OUTPUT_ENABLE 10
#define PIN_OUTPUT_DATA 9

#define PIN_SWUART_F_RX A0
#define PIN_SWUART_F_TX A1
#define PIN_SWUART_R_RX A2
#define PIN_SWUART_R_TX A3
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5
#define PIN_ANALOG_THROTTLE A6
#define PIN_ANALOG_BRAKE A7


#define NUM_INPUTS 16
#define NUM_OUTPUTS 24
#define INPUT_DEBOUNCE_TIME_MS 20

#define WS2812_NUM_LEDS 56


#define INPUT_POS_MOTOR_REAR_ENABLE 11
#define INPUT_POS_MOTOR_FRONT_ENABLE 10
#define INPUT_POS_HORN 9
#define INPUT_POS_INDICATOR_HAZARD 8
#define INPUT_POS_INDICATOR_RIGHT 12
#define INPUT_POS_INDICATOR_LEFT 13
#define INPUT_POS_HEADLIGHTS_HIGH 14
#define INPUT_POS_HEADLIGHTS_LOW 15
#define INPUT_POS_IGNITION 3
#define INPUT_POS_FORWARD 2

#define OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE 0
#define OUTPUT_POS_IND_MOTOR_FRONT_ERROR 1
#define OUTPUT_POS_IND_MOTOR_REAR_ACTIVE 2
#define OUTPUT_POS_IND_MOTOR_REAR_ERROR 3
#define OUTPUT_POS_IND_HEADLIGHTS 4
#define OUTPUT_POS_IND_INDICATOR_LEFT 5
#define OUTPUT_POS_IND_INDICATOR_RIGHT 6
#define OUTPUT_POS_IND_INDICATOR_HAZARD 7
#define OUTPUT_POS_IND_HORN 15
#define OUTPUT_POS_HEADLIGHT_LEFT_HIGH 16
#define OUTPUT_POS_HEADLIGHT_LEFT_LOW 17
#define OUTPUT_POS_HEADLIGHT_RIGHT_HIGH 18
#define OUTPUT_POS_HEADLIGHT_RIGHT_LOW 19
#define OUTPUT_POS_INDICATOR_LEFT 20
#define OUTPUT_POS_INDICATOR_RIGHT 21
#define OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT 22
#define OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR 23


typedef struct {
  uint8_t state;
  uint8_t prevState;
  uint8_t changed;
  uint8_t internalState;
  uint8_t internalPrevState;
  uint64_t internalLastChange;
} kart_input_t;

typedef struct {
  uint8_t state;
} kart_output_t;

typedef enum {
  STATE_SHUTDOWN,
  STATE_SELFTEST_DELAY,
  STATE_OPERATIONAL
} kart_state_t;

enum kart_selftest_substate {
  ST_START,
  ST_BUZZER_OFF,
  ST_WS2812_GREEN,
  ST_WS2812_BLUE,
  ST_END
};


void kart_init();
void kart_updateInputs();
void kart_updateOutputs();
uint8_t kart_getInput(uint8_t pos);
uint8_t kart_inputChanged(uint8_t pos);
void kart_setOutput(uint8_t pos, uint8_t state);
void kart_setHorn(uint8_t state);
void kart_startup();
void kart_shutdown();
void kart_loop();
void kart_selftest_loop();