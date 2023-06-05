/*
Copyright 2023 Julian Metzler

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
*/

#include <Arduino.h>
#include <SoftwareSerial.h>


#define SERIAL_DEBUG


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


#define ADC_TOLERANCE 20          // [bit] ADC tolerance (before input is discarded as implausible)
#define THROTTLE_RATE_LIMIT 1000  // [bit] Maximum positive throttle change per loop cycle (positive meaning away from 0 in both directions)
#define ADC_FILTER_SIZE 5         // Number of ADC readings to use for median filter

#define THROTTLE_MAX 500  // [bit] Maximum throttle value
#define BRAKE_MAX 500     // [bit] Maximum brake value

#define FEEDBACK_RX_TIMEOUT 50  // Motor board feedback receive timeout in milliseconds


#define NUM_INPUTS 16
#define NUM_OUTPUTS 24
#define INPUT_DEBOUNCE_TIME_MS 20

#define WS2812_NUM_LEDS 56
#define WS2812_SEG_0_START 0
#define WS2812_SEG_0_END 12
#define WS2812_SEG_1_START 13
#define WS2812_SEG_1_END 22
#define WS2812_SEG_2_START 23
#define WS2812_SEG_2_END 32
#define WS2812_SEG_3_START 33
#define WS2812_SEG_3_END 42
#define WS2812_SEG_4_START 43
#define WS2812_SEG_4_END 55
#define WS2812_COLOR_LIGHT 0x400000
#define WS2812_COLOR_BRAKE 0xFF0000
#define WS2812_COLOR_INDICATOR 0xFF4000
#define WS2812_COLOR_REVERSE 0xFFFFFF


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

typedef struct {
  uint8_t state;
  uint64_t startTime;
  uint64_t stepStartTime;
} kart_stateMachine_t;

typedef enum {
  STATE_SHUTDOWN,
  STATE_ADC_CALIBRATION,
  STATE_STARTING_UP,
  STATE_OPERATIONAL,
  STATE_SHUTTING_DOWN
} kart_state_t;

typedef enum {
  HL_OFF,
  HL_DRL,
  HL_LOW,
  HL_HIGH
} kart_headlights_t;

typedef enum {
  DIR_FORWARD,
  DIR_REVERSE
} kart_direction_t;

typedef enum {
  TURN_OFF,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_HAZARD
} kart_turn_indicator_t;

enum kart_adc_cal_substate {
  AC_START,
  AC_BEEP_START,
  AC_CALIBRATE,
  AC_BEEP_END,
  AC_END
};

enum kart_startup_substate {
  ST_START,
  ST_ALL_ON,
  ST_BUZZER_OFF,
  ST_WS2812_GREEN,
  ST_WS2812_BLUE,
  ST_MAINBOARD_ENABLE,
  ST_MAINBOARD_DEADTIME,
  ST_END
};

enum kart_shutdown_substate {
  SD_START,
  SD_MAINBOARD_DISABLE,
  SD_MAINBOARD_DEADTIME,
  SD_END
};

enum kart_turn_indicator_substate {
  TI_START,
  TI_ON_BEEP,
  TI_ON,
  TI_OFF_BEEP,
  TI_OFF,
  TI_END,
  TI_INACTIVE
};

typedef struct {
  uint16_t start;
  int16_t steer;
  int16_t speed;
  uint16_t checksum;
} kart_serial_command_t;

typedef struct {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} kart_serial_feedback_t;

typedef struct {
  int16_t minThrottle;
  int16_t maxThrottle;
  int16_t minBrake;
  int16_t maxBrake;
} kart_adc_calibration_values_t;


void kart_init();
void kart_updateInputs();
void kart_updateOutputs();
uint8_t kart_getInput(uint8_t pos);
uint8_t kart_inputChanged(uint8_t pos);
void kart_setOutput(uint8_t pos, uint8_t state);
int16_t kart_prepare_adc_value(int16_t in, int16_t minIn, int16_t maxIn, int16_t minOut, int16_t maxOut);
int16_t kart_adc_rate_limit(int16_t inVal, int16_t prevVal, int16_t maxRateInc, int16_t maxRateDec);
void kart_updateWS2812();
void kart_sendSetpointFront(int16_t steer, int16_t speed);
void kart_sendSetpointRear(int16_t steer, int16_t speed);
uint8_t kart_readFeedback(SoftwareSerial *swuart, kart_serial_feedback_t *feedbackOut);
uint8_t kart_readFeedbackFront();
uint8_t kart_readFeedbackRear();
void kart_setHorn(uint8_t state);
void kart_calibrate_adc();
void kart_startup();
void kart_shutdown();
void kart_startTurnIndicator();
void kart_stopTurnIndicator();
void kart_processMotorFrontEnableSwitch();
void kart_processMotorRearEnableSwitch();
void kart_processHeadlightsSwitch();
void kart_processHazardButton();
void kart_processForwardReverseSwitch();
void kart_processHornButton();
void kart_processTurnIndicatorSwitch();
void kart_loop();
void kart_adc_calibration_loop();
void kart_startup_loop();
void kart_shutdown_loop();
void kart_turnIndicator_loop();
void kart_operation_loop();