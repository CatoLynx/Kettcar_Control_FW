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


//#define SERIAL_DEBUG

//#define MAINBOARD_SOFTWARE_SERIAL
#define MAINBOARD_HARDWARE_SERIAL

#if defined(SERIAL_DEBUG) && defined(MAINBOARD_HARDWARE_SERIAL)
#error "SERIAL_DEBUG and MAINBOARD_HARDWARE_SERIAL can not be used simultaneously!"
#endif

#if defined(MAINBOARD_SOFTWARE_SERIAL) && defined(MAINBOARD_HARDWARE_SERIAL)
#error "MAINBOARD_SOFTWARE_SERIAL and MAINBOARD_HARDWARE_SERIAL can not be used simultaneously!"
#endif

#ifdef MAINBOARD_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#endif


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

#define THROTTLE_MAX 1000            // [bit] Maximum throttle value
#define BRAKE_MAX 1000               // [bit] Maximum brake value
#define OUTPUT_HARD_LIMIT_MIN -1000  // [bit] Absolute lower limit for throttle/brake output
#define OUTPUT_HARD_LIMIT_MAX 1000   // [bit] Absolute upper limit for throttle/brake output

#define USART_BAUD 115200
#define USART_TX_INTERVAL 50       // [ms] Interval for sending USART data
#define FEEDBACK_RX_TIMEOUT 30     // Motor board feedback receive timeout in milliseconds
#define USART_CONTROL_MODE_VLT 23  // Voltage mode request
#define USART_CONTROL_MODE_TRQ 42  // Torque mode request

#define SPEED_FILTER_SIZE 5  // Number of speed feedback readings to use for median filter

// Formula for converting acceleration to km/h per second: (pi * <wheel diameter in cm>) * 0.0006 km/(rpm*cm*h) * (<acceleration> rpm / <loop interval> ms)
// To convert to m/s², divide by 3.6
// With current tires, this should yield -5 rpm / 50ms = -6.7 km/h/s = -1.86 m/s²
#define BRAKE_LIGHT_DECEL_THRESHOLD -5  // Deceleration threshold for brake light activation

// Formula for speed in km/h: (pi * <wheel diameter in cm>) * 0.0006 km/(rpm*cm*h) * <rotation speed in rpm>
// With current tires, this should yield 1 rpm = 0.0672 km/h
#define BRAKE_LIGHT_MIN_SPEED 20  // Minimum speed for automatic brake light activation
#define MODE_CHANGE_MAX_SPEED 20  // Maximum speed for control mode change

#define BRAKE_LIGHT_DECEL_AFTER_TIME 500  // Additional on time after auto activation of brake light in ms

#define INACTIVITY_TIMEOUT (5 * 60 * 1000)  // Duration of inactivity (no button presses or analog inputs > 0) before shutdown in ms

#define NUM_INPUTS 16
#define NUM_OUTPUTS 24
#define INPUT_DEBOUNCE_TIME_MS 40

#define WS2812_NUM_LEDS 56
#define WS2812_SEG_0_START 0
#define WS2812_SEG_0_END 11
#define WS2812_SEG_1_START 12
#define WS2812_SEG_1_END 22
#define WS2812_SEG_2_START 23
#define WS2812_SEG_2_END 32
#define WS2812_SEG_3_START 33
#define WS2812_SEG_3_END 43
#define WS2812_SEG_4_START 44
#define WS2812_SEG_4_END 55
#define WS2812_COLOR_LIGHT 0x200000
#define WS2812_COLOR_BRAKE 0xFF0000
#define WS2812_COLOR_INDICATOR 0xFF4000
#define WS2812_COLOR_REVERSE 0xFFFFFF
#define WS2812_TURN_INDICATOR_ANIM_INTERVAL 20  // Turn indicator animation timestep in ms


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
#define INPUT_POS_CONTROL_MODE 1

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
  STATE_SHUTDOWN_INACTIVITY,
  STATE_ADC_CALIBRATION,
  STATE_MANUAL_ENABLE,
  STATE_STARTING_UP,
  STATE_OPERATIONAL,
  STATE_SHUTTING_DOWN,
  STATE_SHUTTING_DOWN_INACTIVITY,
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
  CTRL_VLT = 23,
  CTRL_TRQ = 42
} kart_control_mode_t;

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

enum kart_manual_enable_substate {
  ME_START,
  ME_BEEP_START,
  ME_WAIT_RELEASE,
  ME_ACTIVE,
  ME_BEEP_END,
  ME_END
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

enum kart_reverse_beep_substate {
  RB_START,
  RB_BEEP,
  RB_PAUSE,
  RB_END,
  RB_INACTIVE
};

enum kart_motor {
  MOT_FRONT,
  MOT_REAR
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
void kart_turnOffWS2812();
#ifdef MAINBOARD_HARDWARE_SERIAL
void selectMotorForUART(uint8_t motor);
#endif
void kart_sendSetpointFront(int16_t speed);
void kart_sendSetpointRear(int16_t speed);
void kart_sendControlModeFront(int16_t mode);
void kart_sendControlModeRear(int16_t mode);
#ifdef MAINBOARD_SOFTWARE_SERIAL
uint8_t kart_readFeedback(SoftwareSerial *swuart, kart_serial_feedback_t *feedbackOut);
#endif
#ifdef MAINBOARD_SOFTWARE_SERIAL
uint8_t kart_readFeedback(kart_serial_feedback_t *feedbackOut);
#endif
uint8_t kart_readFeedbackFront();
uint8_t kart_readFeedbackRear();
void kart_setHorn(uint8_t state);
void kart_calibrate_adc();
void kart_start_manual_enable_mode();
void kart_startup();
void kart_shutdown();
void kart_startTurnIndicator();
void kart_stopTurnIndicator();
void kart_startReverseBeep();
void kart_stopReverseBeep();
void kart_processMotorFrontEnableSwitch();
void kart_processMotorRearEnableSwitch();
void kart_processHeadlightsSwitch();
void kart_processHazardButton();
void kart_processForwardReverseSwitch();
void kart_processControlModeSwitch();
void kart_processHornButton();
void kart_processTurnIndicatorSwitch();
void kart_loop();
void kart_adc_calibration_loop();
void kart_manual_enable_loop();
void kart_startup_loop();
void kart_shutdown_loop();
void kart_turnIndicator_loop();
void kart_reverseBeep_loop();
void kart_operation_loop();
