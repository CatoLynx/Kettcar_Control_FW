/*
  Copyright 2023-2024 Julian Metzler

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

#include "kart.h"
#include "MedianFilter.h"
#include "util.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>


static Adafruit_NeoPixel kart_ws2812(WS2812_NUM_LEDS, PIN_WS2812_DATA, NEO_GRB + NEO_KHZ800);

#ifdef MAINBOARD_SOFTWARE_SERIAL
static SoftwareSerial kart_swuartFront(PIN_SWUART_F_RX, PIN_SWUART_F_TX);
static SoftwareSerial kart_swuartRear(PIN_SWUART_R_RX, PIN_SWUART_R_TX);
#endif

static kart_serial_command_t kart_commandFront = { 0, 0, 0, 0 };
static kart_serial_command_t kart_commandRear = { 0, 0, 0, 0 };
kart_serial_feedback_t kart_feedbackFront = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
kart_serial_feedback_t kart_feedbackRear = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint16_t kart_feedbackErrorCounterFront = 0;
static uint16_t kart_feedbackErrorCounterRear = 0;

static kart_input_t kart_inputs[NUM_INPUTS] = { { 0, 0, 0, 0, 0, 0 } };
static kart_output_t kart_outputs[NUM_OUTPUTS] = { { 0 } };
static sMedianFilter_t kart_medianFilterThrottle;
static sMedianNode_t kart_medianBufferThrottle[ADC_FILTER_SIZE];
static sMedianFilter_t kart_medianFilterBrake;
static sMedianNode_t kart_medianBufferBrake[ADC_FILTER_SIZE];
static sMedianFilter_t kart_medianFilterSpeed;
static sMedianNode_t kart_medianBufferSpeed[SPEED_FILTER_SIZE];
static int16_t kart_throttleInput = 0;
static int16_t kart_brakeInput = 0;
static int16_t kart_prevThrottleInput = 0;
static int16_t kart_prevBrakeInput = 0;
static int16_t kart_setpointFront = 0;
static int16_t kart_setpointRear = 0;
static int16_t kart_speedAbs = 0;
static int16_t kart_prevSpeedAbs = 0;
static uint64_t kart_lastUsartUpdate = 0;
static uint64_t kart_lastActivity = 0;

static uint8_t kart_motorFrontEnabled = 0;
static uint8_t kart_motorRearEnabled = 0;

static kart_state_t kart_state = STATE_SHUTDOWN;
static kart_headlights_t kart_headlights = HL_OFF;
static kart_direction_t kart_direction = DIR_FORWARD;
static kart_control_mode_t kart_controlMode = CTRL_VLT;
static bool kart_controlModeChanged = false;
static kart_turn_indicator_t kart_turnIndicator = TURN_OFF;
static bool kart_ignitionOn = false;
static uint8_t kart_brakeLightState = 0;
static uint8_t kart_prevBrakeLightState = 0;
static uint64_t kart_brakeLightDecelTime = 0;
static kart_adc_calibration_values_t kart_adc_calibration_values = { 0, 0, 0, 0 };
static kart_led_animation_t kart_ledAnimation = ANIM_NONE;
static uint32_t kart_ledAnimationStart = 0;

static kart_stateMachine_t kart_smAdcCalibration = { AC_START, 0, 0 };
static kart_stateMachine_t kart_smManualEnable = { ME_START, 0, 0 };
static kart_stateMachine_t kart_smStartup = { ST_START, 0, 0 };
static kart_stateMachine_t kart_smShutdown = { SD_START, 0, 0 };
static kart_stateMachine_t kart_smTurnIndicator = { TI_INACTIVE, 0, 0 };
static uint8_t kart_turnIndicatorNumActiveLEDs = 0;
static uint64_t kart_turnIndicatorLastAnimationUpdate = 0;
static kart_stateMachine_t kart_smReverseBeep = { RB_INACTIVE, 0, 0 };


void kart_init() {
  kart_medianFilterThrottle.numNodes = ADC_FILTER_SIZE;
  kart_medianFilterThrottle.medianBuffer = kart_medianBufferThrottle;
  MEDIANFILTER_Init(&kart_medianFilterThrottle);

  kart_medianFilterBrake.numNodes = ADC_FILTER_SIZE;
  kart_medianFilterBrake.medianBuffer = kart_medianBufferBrake;
  MEDIANFILTER_Init(&kart_medianFilterBrake);

  kart_medianFilterSpeed.numNodes = SPEED_FILTER_SIZE;
  kart_medianFilterSpeed.medianBuffer = kart_medianBufferSpeed;
  MEDIANFILTER_Init(&kart_medianFilterSpeed);

  // Load ADC calibration values
  kart_adc_calibration_values.minThrottle = ((uint16_t)EEPROM.read(0x00) << 8) | EEPROM.read(0x01);
  kart_adc_calibration_values.maxThrottle = ((uint16_t)EEPROM.read(0x02) << 8) | EEPROM.read(0x03);
  kart_adc_calibration_values.minBrake = ((uint16_t)EEPROM.read(0x04) << 8) | EEPROM.read(0x05);
  kart_adc_calibration_values.maxBrake = ((uint16_t)EEPROM.read(0x06) << 8) | EEPROM.read(0x07);

#ifdef SERIAL_DEBUG
  Serial.println("Calibration values in use:");
  Serial.print("  Throttle Min: ");
  Serial.println(kart_adc_calibration_values.minThrottle);
  Serial.print("  Throttle Max: ");
  Serial.println(kart_adc_calibration_values.maxThrottle);
  Serial.print("  Brake Min:    ");
  Serial.println(kart_adc_calibration_values.minBrake);
  Serial.print("  Brake Max:    ");
  Serial.println(kart_adc_calibration_values.maxBrake);
#endif

  digitalWrite(PIN_HORN, LOW);
  digitalWrite(PIN_BUZZER, LOW);
  digitalWrite(PIN_UART_SEL, LOW);
  digitalWrite(PIN_WS2812_DATA, LOW);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_INPUT_CLOCK, LOW);
  digitalWrite(PIN_INPUT_LOAD, HIGH);
  digitalWrite(PIN_OUTPUT_CLOCK, LOW);
  digitalWrite(PIN_OUTPUT_LOAD, LOW);
  digitalWrite(PIN_OUTPUT_ENABLE, LOW);
  digitalWrite(PIN_OUTPUT_DATA, LOW);
  digitalWrite(PIN_SWUART_F_TX, HIGH);
  digitalWrite(PIN_SWUART_R_TX, HIGH);

  pinMode(PIN_HORN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_UART_SEL, OUTPUT);
  pinMode(PIN_WS2812_DATA, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_INPUT_CLOCK, OUTPUT);
  pinMode(PIN_INPUT_LOAD, OUTPUT);
  pinMode(PIN_INPUT_DATA, INPUT);
  pinMode(PIN_OUTPUT_CLOCK, OUTPUT);
  pinMode(PIN_OUTPUT_LOAD, OUTPUT);
  pinMode(PIN_OUTPUT_ENABLE, OUTPUT);
  pinMode(PIN_OUTPUT_DATA, OUTPUT);
  pinMode(PIN_SWUART_F_RX, INPUT);
  pinMode(PIN_SWUART_F_TX, OUTPUT);
  pinMode(PIN_SWUART_R_RX, INPUT);
  pinMode(PIN_SWUART_R_TX, OUTPUT);
  pinMode(PIN_I2C_SDA, INPUT);
  pinMode(PIN_I2C_SCL, INPUT);
  pinMode(PIN_ANALOG_THROTTLE, INPUT);
  pinMode(PIN_ANALOG_BRAKE, INPUT);

  kart_ws2812.begin();

#ifdef MAINBOARD_SOFTWARE_SERIAL
  kart_swuartFront.begin(USART_BAUD);
  kart_swuartRear.begin(USART_BAUD);
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
  Serial.begin(USART_BAUD);
#endif
}

void kart_updateInputs() {
  uint64_t now = millis();
  digitalWrite(PIN_INPUT_LOAD, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_INPUT_LOAD, HIGH);
  delayMicroseconds(10);

  for (uint8_t i = 0; i < NUM_INPUTS / 8; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t bit = !digitalRead(PIN_INPUT_DATA);
      uint8_t listIndex = i * 8 + j;

      // Reset Changed flag in any case since it's only set for one cycle
      kart_inputs[listIndex].changed = 0;

      // Always set internal / raw state flags
      if (bit != kart_inputs[listIndex].internalPrevState) {
        kart_inputs[listIndex].internalLastChange = now;
      }
      kart_inputs[listIndex].internalPrevState = kart_inputs[listIndex].internalState;
      kart_inputs[listIndex].internalState = bit;

      // If debounce deadtime passed without any further change, carry over the internal values
      if (now - kart_inputs[listIndex].internalLastChange > INPUT_DEBOUNCE_TIME_MS) {
        if (bit != kart_inputs[listIndex].state) {
          kart_inputs[listIndex].changed = 1;
          kart_lastActivity = now;
          kart_inputs[listIndex].prevState = kart_inputs[listIndex].state;
          kart_inputs[listIndex].state = bit;
        }
      }

      digitalWrite(PIN_INPUT_CLOCK, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_INPUT_CLOCK, LOW);
      delayMicroseconds(10);
    }
  }
}

void kart_updateOutputs() {
  for (uint8_t i = 0; i < NUM_OUTPUTS / 8; i++) {
    uint8_t byte = 0x00;
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t listIndex = i * 8 + j;
      if (kart_outputs[listIndex].state) {
        byte |= 1 << (7 - j);
      }
    }
    shiftOut(PIN_OUTPUT_DATA, PIN_OUTPUT_CLOCK, MSBFIRST, byte);
  }

  digitalWrite(PIN_OUTPUT_LOAD, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_OUTPUT_LOAD, LOW);
  delayMicroseconds(10);
}

uint8_t kart_getInput(uint8_t pos) {
  if (pos >= NUM_INPUTS) return 0;
  return kart_inputs[pos].state;
}

uint8_t kart_inputChanged(uint8_t pos) {
  if (pos >= NUM_INPUTS) return 0;
  return kart_inputs[pos].changed;
}

void kart_setOutput(uint8_t pos, uint8_t state) {
  if (pos >= NUM_OUTPUTS) return;
  kart_outputs[pos].state = !!state;
}

int16_t kart_prepare_adc_value(int16_t in, int16_t minIn, int16_t maxIn, int16_t minOut, int16_t maxOut) {
  // In:  Raw ADC value
  // Out: - 0 if ADC value is outside the calibrated range (plus tolerance)
  //      - Otherwise, a value from 0 to 1000 proportional to input

  if (minIn - in > ADC_TOLERANCE) return 0;
  if (in - maxIn > ADC_TOLERANCE) return 0;
  if (in < minIn) in = minIn;
  if (in > maxIn) in = maxIn;
  return map(in, minIn, maxIn, minOut, maxOut);
}

int16_t kart_adc_rate_limit(int16_t inVal, int16_t prevVal, int16_t maxRateInc, int16_t maxRateDec) {
  if ((prevVal > 0 && inVal < 0) || (prevVal < 0 && inVal > 0)) {
    // Sign switch. Adjust inVal to 0 to separate the "decrease in magnitude"
    // and "increase in magnitude" phases.
    inVal = 0;
  }

  if (prevVal > 0 && inVal >= 0) {
    if (inVal > prevVal) {
      // Stay positive, increase in magnitude
      if (inVal - prevVal > maxRateInc) return prevVal + maxRateInc;
    } else {
      // Stay positive, decrease in magnitude
      if (prevVal - inVal > maxRateDec) return prevVal - maxRateDec;
    }
  } else if (prevVal < 0 && inVal <= 0) {
    if (inVal < prevVal) {
      // Stay negative, increase in magnitude
      if (prevVal - inVal > maxRateInc) return prevVal - maxRateInc;
    } else {
      // Stay negative, decrease in magnitude
      if (inVal - prevVal > maxRateDec) return prevVal + maxRateDec;
    }
  }

  // In any other situation, just pass through
  return inVal;
}

uint32_t kart_getLedAnimationColor(uint8_t segNum) {
  uint32_t now = millis();
  uint32_t timeDiff = now - kart_ledAnimationStart;
  switch (kart_ledAnimation) {
    case ANIM_FLASH_ORANGE:
    case ANIM_FLASH_BLUE: {
        uint32_t cycleTime = timeDiff % 480;
        uint8_t subCycleTime = cycleTime % 60;
        if (cycleTime >= 350) return 0x000000;
        if (cycleTime >= 280) return (kart_ledAnimation == ANIM_FLASH_ORANGE ? WS2812_COLOR_INDICATOR : 0x0000FF);
        if (subCycleTime >=  0 && subCycleTime < 30) return (kart_ledAnimation == ANIM_FLASH_ORANGE ? WS2812_COLOR_INDICATOR : 0x0000FF);
        if (subCycleTime >= 30 && subCycleTime < 60) return 0x000000;
        break;
      }
  }
}

void kart_updateWS2812() {
  kart_ws2812.clear();
  uint32_t seg0Color = 0;
  uint32_t seg1Color = 0;
  uint32_t seg2Color = 0;
  uint32_t seg3Color = 0;
  uint32_t seg4Color = 0;
  uint8_t tiAnimationActiveLeft = 0;
  uint8_t tiAnimationActiveRight = 0;

  if (kart_state == STATE_OPERATIONAL || kart_state == STATE_STARTING_UP) {
    // Segment 0: Animation > Indicator > Brake > Light
    if (kart_ledAnimation != ANIM_NONE) {
      seg0Color = kart_getLedAnimationColor(0);
    } else if (kart_turnIndicator == TURN_LEFT || kart_turnIndicator == TURN_HAZARD) {
      if (kart_smTurnIndicator.state == TI_ON_BEEP || kart_smTurnIndicator.state == TI_ON) {
        seg0Color = WS2812_COLOR_INDICATOR;
        tiAnimationActiveLeft = 1;
      }
    } else if (kart_brakeLightState) {
      seg0Color = WS2812_COLOR_BRAKE;
    } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
      seg0Color = WS2812_COLOR_LIGHT;
    }

    // Segment 1: Animation > Brake > Reverse > Light
    if (kart_ledAnimation != ANIM_NONE) {
      seg1Color = kart_getLedAnimationColor(1);
    } else if (kart_brakeLightState) {
      seg1Color = WS2812_COLOR_BRAKE;
    } else if (kart_direction == DIR_REVERSE) {
      seg1Color = WS2812_COLOR_REVERSE;
    } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
      seg1Color = WS2812_COLOR_LIGHT;
    }

    // Segment 2: Animation > Reverse > Brake > Light
    if (kart_ledAnimation != ANIM_NONE) {
      seg2Color = kart_getLedAnimationColor(2);
    } else if (kart_direction == DIR_REVERSE) {
      seg2Color = WS2812_COLOR_REVERSE;
    } else if (kart_brakeLightState) {
      seg2Color = WS2812_COLOR_BRAKE;
    } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
      seg2Color = WS2812_COLOR_LIGHT;
    }

    // Segment 3: Animation > Brake > Reverse > Light
    if (kart_ledAnimation != ANIM_NONE) {
      seg3Color = kart_getLedAnimationColor(3);
    } else if (kart_brakeLightState) {
      seg3Color = WS2812_COLOR_BRAKE;
    } else if (kart_direction == DIR_REVERSE) {
      seg3Color = WS2812_COLOR_REVERSE;
    } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
      seg3Color = WS2812_COLOR_LIGHT;
    }

    // Segment 4: Animation > Indicator > Brake > Light
    if (kart_ledAnimation != ANIM_NONE) {
      seg4Color = kart_getLedAnimationColor(4);
    } else if (kart_turnIndicator == TURN_RIGHT || kart_turnIndicator == TURN_HAZARD) {
      if (kart_smTurnIndicator.state == TI_ON_BEEP || kart_smTurnIndicator.state == TI_ON) {
        seg4Color = WS2812_COLOR_INDICATOR;
        tiAnimationActiveRight = 1;
      }
    } else if (kart_brakeLightState) {
      seg4Color = WS2812_COLOR_BRAKE;
    } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
      seg4Color = WS2812_COLOR_LIGHT;
    }

    for (uint8_t i = WS2812_SEG_0_START; i <= WS2812_SEG_0_END; i++) {
      if (tiAnimationActiveLeft) {
        uint8_t inactiveLEDs = WS2812_SEG_0_END + 1 - kart_turnIndicatorNumActiveLEDs;
        kart_ws2812.setPixelColor(i, (i < inactiveLEDs) ? 0x000000 : seg0Color);
      } else {
        kart_ws2812.setPixelColor(i, seg0Color);
      }
    }
    for (uint8_t i = WS2812_SEG_1_START; i <= WS2812_SEG_1_END; i++) {
      kart_ws2812.setPixelColor(i, seg1Color);
    }
    for (uint8_t i = WS2812_SEG_2_START; i <= WS2812_SEG_2_END; i++) {
      kart_ws2812.setPixelColor(i, seg2Color);
    }
    for (uint8_t i = WS2812_SEG_3_START; i <= WS2812_SEG_3_END; i++) {
      kart_ws2812.setPixelColor(i, seg3Color);
    }
    for (uint8_t i = WS2812_SEG_4_START; i <= WS2812_SEG_4_END; i++) {
      if (tiAnimationActiveRight) {
        uint8_t activeLEDs = WS2812_SEG_4_START + kart_turnIndicatorNumActiveLEDs;
        kart_ws2812.setPixelColor(i, (i < activeLEDs) ? seg4Color : 0x000000);
      } else {
        kart_ws2812.setPixelColor(i, seg4Color);
      }
    }
  }

  kart_ws2812.show();
}

void kart_turnOffWS2812() {
  kart_ws2812.clear();
  kart_ws2812.show();
}

#ifdef MAINBOARD_HARDWARE_SERIAL
void selectMotorForUART(uint8_t motor) {
  if (motor == MOT_FRONT) {
    digitalWrite(PIN_UART_SEL, HIGH);
  } else if (motor == MOT_REAR) {
    digitalWrite(PIN_UART_SEL, LOW);
  }
  delayMicroseconds(UART_HW_SWITCH_POST_DELAY_US);
}
#endif

void kart_sendSetpointFront(int16_t speed) {
  kart_commandFront.start = 0xABCD;
  kart_commandFront.steer = 0;
  kart_commandFront.speed = speed;
  kart_commandFront.checksum = (uint16_t)(kart_commandFront.start ^ kart_commandFront.steer ^ kart_commandFront.speed);

#ifdef MAINBOARD_SOFTWARE_SERIAL
  kart_swuartFront.write((uint8_t *)&kart_commandFront, sizeof(kart_commandFront));
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
  selectMotorForUART(MOT_FRONT);
  Serial.write((uint8_t *)&kart_commandFront, sizeof(kart_commandFront));
  delayMicroseconds(8 * UART_HW_SWITCH_BYTE_DELAY_US);  // 8 bytes, µs per byte
#endif

  // Enable "Active" LED if setpoint != 0
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, speed != 0);
}

void kart_sendSetpointRear(int16_t speed) {
  kart_commandRear.start = 0xABCD;
  kart_commandRear.steer = 0;
  kart_commandRear.speed = speed;
  kart_commandRear.checksum = (uint16_t)(kart_commandRear.start ^ kart_commandRear.steer ^ kart_commandRear.speed);

#ifdef MAINBOARD_SOFTWARE_SERIAL
  kart_swuartRear.write((uint8_t *)&kart_commandRear, sizeof(kart_commandRear));
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
  selectMotorForUART(MOT_REAR);
  Serial.write((uint8_t *)&kart_commandRear, sizeof(kart_commandRear));
  delayMicroseconds(8 * UART_HW_SWITCH_BYTE_DELAY_US);  // 8 bytes, µs per byte
#endif

  // Enable "Active" LED if setpoint != 0
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, speed != 0);
}

void kart_sendControlModeFront(int16_t mode) {
  kart_commandFront.start = 0xF0CC;
  kart_commandFront.steer = mode;
  kart_commandFront.speed = 0;
  kart_commandFront.checksum = (uint16_t)(kart_commandFront.start ^ kart_commandFront.steer ^ kart_commandFront.speed);

#ifdef MAINBOARD_SOFTWARE_SERIAL
  kart_swuartFront.write((uint8_t *)&kart_commandFront, sizeof(kart_commandFront));
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
  selectMotorForUART(MOT_FRONT);
  Serial.write((uint8_t *)&kart_commandFront, sizeof(kart_commandFront));
  delayMicroseconds(8 * UART_HW_SWITCH_BYTE_DELAY_US);  // 8 bytes, µs per byte
#endif
}

void kart_sendControlModeRear(int16_t mode) {
  kart_commandRear.start = 0xF0CC;
  kart_commandRear.steer = mode;
  kart_commandRear.speed = 0;
  kart_commandRear.checksum = (uint16_t)(kart_commandRear.start ^ kart_commandRear.steer ^ kart_commandRear.speed);

#ifdef MAINBOARD_SOFTWARE_SERIAL
  kart_swuartRear.write((uint8_t *)&kart_commandRear, sizeof(kart_commandRear));
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
  selectMotorForUART(MOT_REAR);
  Serial.write((uint8_t *)&kart_commandRear, sizeof(kart_commandRear));
  delayMicroseconds(8 * UART_HW_SWITCH_BYTE_DELAY_US);  // 8 bytes, µs per byte
#endif
}

#ifdef MAINBOARD_SOFTWARE_SERIAL
uint8_t kart_readFeedback(SoftwareSerial *uart, kart_serial_feedback_t *feedbackOut) {
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
  uint8_t kart_readFeedback(kart_serial_feedback_t * feedbackOut) {
    HardwareSerial *uart = &Serial;
#endif
    // Returns 0 on success, 1 on error
    uint64_t startTime = millis();
    kart_serial_feedback_t feedbackIn = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // Wait for start value with timeout
    uint8_t waitingForByteNo = 0;
    while (millis() - startTime < FEEDBACK_RX_TIMEOUT) {
      if (kart_state == STATE_OPERATIONAL) {
        kart_turnIndicator_loop();
        kart_reverseBeep_loop();
      }
      if (!(uart->available())) continue;
      uint8_t byte = uart->read();
      if (waitingForByteNo == 0) {
        if (byte != 0xCD) continue;
        waitingForByteNo = 1;
      } else if (waitingForByteNo == 1) {
        if (byte != 0xAB) continue;
        break;
      }
    }
    if (millis() - startTime >= FEEDBACK_RX_TIMEOUT) {
#ifdef SERIAL_DEBUG
      Serial.println("Rx timeout");
#endif
      return 1;
    }

    // Manually set this since we just checked it and know it's correct
    feedbackIn.start = 0xABCD;
#ifdef SERIAL_DEBUG
    Serial.println("Start byte found");
#endif

    // Start value (2 bytes) found, now read the remaining bytes with timeout
    uint8_t bytesRemaining = sizeof(kart_serial_feedback_t) - 2;
    uint8_t *pFeedback = (uint8_t *)&feedbackIn + 2;
    while (millis() - startTime < FEEDBACK_RX_TIMEOUT) {
      if (kart_state == STATE_OPERATIONAL) {
        kart_turnIndicator_loop();
        kart_reverseBeep_loop();
      }
      if (!(uart->available())) continue;
      *pFeedback = uart->read();
      pFeedback++;
      bytesRemaining--;
      if (bytesRemaining == 0) break;
    }
    if (millis() - startTime >= FEEDBACK_RX_TIMEOUT) {
#ifdef SERIAL_DEBUG
      Serial.println("Rx timeout");
#endif
      return 1;
    }

#ifdef SERIAL_DEBUG
    uint8_t *rawData = (uint8_t *)&feedbackIn;
    Serial.print("Raw feedback: ");
    for (uint8_t i = 0; i < sizeof(kart_serial_feedback_t); i++) {
      Serial.print(rawData[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
#endif

    // Verify checksum
    uint16_t checksum;
    checksum = (uint16_t)(feedbackIn.start ^ feedbackIn.cmd1 ^ feedbackIn.cmd2 ^ feedbackIn.speedR_meas ^ feedbackIn.speedL_meas
                          ^ feedbackIn.batVoltage ^ feedbackIn.boardTemp ^ feedbackIn.cmdLed);

    // Check validity of the new data
    if (feedbackIn.start == 0xABCD && checksum == feedbackIn.checksum) {
      // Copy the new data
      memcpy(feedbackOut, &feedbackIn, sizeof(kart_serial_feedback_t));

      // Print data to built-in Serial
#ifdef SERIAL_DEBUG
      Serial.print("1: ");
      Serial.print(feedbackOut->cmd1);
      Serial.print(" 2: ");
      Serial.print(feedbackOut->cmd2);
      Serial.print(" 3: ");
      Serial.print(feedbackOut->speedR_meas);
      Serial.print(" 4: ");
      Serial.print(feedbackOut->speedL_meas);
      Serial.print(" 5: ");
      Serial.print(feedbackOut->batVoltage);
      Serial.print(" 6: ");
      Serial.print(feedbackOut->boardTemp);
      Serial.print(" 7: ");
      Serial.println(feedbackOut->cmdLed);
#endif
    } else {
#ifdef SERIAL_DEBUG
      Serial.println("Non-valid data skipped");
#endif
      return 1;
    }

    return 0;
  }

  uint8_t kart_readFeedbackFront() {
#ifdef MAINBOARD_SOFTWARE_SERIAL
    // Enable Rx on this channel
    kart_swuartFront.listen();
#ifdef SERIAL_DEBUG
    Serial.println("Front listening");
#endif
    return kart_readFeedback(&kart_swuartFront, &kart_feedbackFront);
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
    selectMotorForUART(MOT_FRONT);
    // Flush buffer, might contain stale data from other mainboard
    while (Serial.available()) Serial.read();
    return kart_readFeedback(&kart_feedbackFront);
#endif
  }

  uint8_t kart_readFeedbackRear() {
#ifdef MAINBOARD_SOFTWARE_SERIAL
    // Enable Rx on this channel
    kart_swuartRear.listen();
#ifdef SERIAL_DEBUG
    Serial.println("Rear listening");
#endif
    return kart_readFeedback(&kart_swuartRear, &kart_feedbackRear);
#endif
#ifdef MAINBOARD_HARDWARE_SERIAL
    selectMotorForUART(MOT_REAR);
    // Flush buffer, might contain stale data from other mainboard
    while (Serial.available()) Serial.read();
    return kart_readFeedback(&kart_feedbackRear);
#endif
  }

  void kart_setHorn(uint8_t state) {
    digitalWrite(PIN_HORN, !!state);
  }

  void kart_calibrate_adc() {
    kart_smAdcCalibration.state = AC_START;
    kart_smAdcCalibration.startTime = millis();
    kart_smAdcCalibration.stepStartTime = kart_smAdcCalibration.startTime;
  }

  void kart_start_manual_enable_mode() {
    kart_smManualEnable.state = ME_START;
    kart_smManualEnable.startTime = millis();
    kart_smManualEnable.stepStartTime = kart_smManualEnable.startTime;
  }

  void kart_startup() {
    kart_smStartup.state = ST_START;
    kart_smStartup.startTime = millis();
    kart_smStartup.stepStartTime = kart_smStartup.startTime;
  }

  void kart_shutdown() {
    kart_smShutdown.state = SD_START;
    kart_smShutdown.startTime = millis();
    kart_smShutdown.stepStartTime = kart_smShutdown.startTime;
  }

  void kart_startTurnIndicator() {
    kart_smTurnIndicator.state = TI_START;
    kart_smTurnIndicator.startTime = millis();
    kart_smTurnIndicator.stepStartTime = kart_smTurnIndicator.startTime;
  }

  void kart_stopTurnIndicator() {
    kart_smTurnIndicator.state = TI_END;
    kart_smTurnIndicator.stepStartTime = millis();
  }

  void kart_startReverseBeep() {
    kart_smReverseBeep.state = RB_START;
    kart_smReverseBeep.startTime = millis();
    kart_smReverseBeep.stepStartTime = kart_smReverseBeep.startTime;
  }

  void kart_stopReverseBeep() {
    kart_smReverseBeep.state = RB_END;
    kart_smReverseBeep.stepStartTime = millis();
  }

  void kart_processMotorFrontEnableSwitch() {
    kart_motorFrontEnabled = kart_getInput(INPUT_POS_MOTOR_FRONT_ENABLE);
  }

  void kart_processMotorRearEnableSwitch() {
    kart_motorRearEnabled = kart_getInput(INPUT_POS_MOTOR_REAR_ENABLE);
  }

  void kart_processHeadlightsSwitch() {
    if (!kart_getInput(INPUT_POS_HEADLIGHTS_LOW) && !kart_getInput(INPUT_POS_HEADLIGHTS_HIGH)) {
      // State: Off (Daytime running lights)
      // L+R "low" on if turn indicator is off, L+R "high" off, instrument lights off
      if (kart_smTurnIndicator.state == TI_INACTIVE) {
        kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      } else {
        if (kart_turnIndicator != TURN_LEFT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        if (kart_turnIndicator != TURN_RIGHT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      }
      kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 0);
      kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 0);
      kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
      kart_setOutput(OUTPUT_POS_IND_HORN, 0);
      if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
      kart_headlights = HL_DRL;
    } else if (kart_getInput(INPUT_POS_HEADLIGHTS_LOW) && !kart_getInput(INPUT_POS_HEADLIGHTS_HIGH)) {
      // State: Low
      // L+R "low" on if turn indicator is off, L "high" off, R "high" on, instrument lights on
      if (kart_smTurnIndicator.state == TI_INACTIVE) {
        kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      } else {
        if (kart_turnIndicator != TURN_LEFT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        if (kart_turnIndicator != TURN_RIGHT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      }
      kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 0);
      kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 1);
      kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 1);
      kart_setOutput(OUTPUT_POS_IND_HORN, 1);
      if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
      kart_headlights = HL_LOW;
    } else if (kart_getInput(INPUT_POS_HEADLIGHTS_LOW) && kart_getInput(INPUT_POS_HEADLIGHTS_HIGH)) {
      // State: High
      // L+R "low" on if turn indicator is off, L+R "high" on, instrument lights on
      if (kart_smTurnIndicator.state == TI_INACTIVE) {
        kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      } else {
        if (kart_turnIndicator != TURN_LEFT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        if (kart_turnIndicator != TURN_RIGHT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      }
      kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 1);
      kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 1);
      kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 1);
      kart_setOutput(OUTPUT_POS_IND_HORN, 1);
      if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
      kart_headlights = HL_HIGH;
    } else if (!kart_getInput(INPUT_POS_HEADLIGHTS_LOW) && kart_getInput(INPUT_POS_HEADLIGHTS_HIGH)) {
      // State: High flash
      // L+R "low" on if turn indicator is off, L "high" on, R "high" off, instrument lights off
      if (kart_smTurnIndicator.state == TI_INACTIVE) {
        kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      } else {
        if (kart_turnIndicator != TURN_LEFT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
        if (kart_turnIndicator != TURN_RIGHT && kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
      }
      kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 1);
      kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 0);
      kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
      kart_setOutput(OUTPUT_POS_IND_HORN, 0);
      if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
      kart_headlights = HL_HIGH_FLASH;
    }
    kart_updateWS2812();
  }

  void kart_processHazardButton() {
    if (kart_getInput(INPUT_POS_INDICATOR_HAZARD)) {
      if (kart_ledAnimation != ANIM_NONE) {
        // If an animation is active, disable it
        kart_ledAnimation = ANIM_NONE;
        kart_ledAnimationStart = 0;

        // Revert to state determined by indicator switch
        kart_processTurnIndicatorSwitch();

        // Illuminate button again if necessary
        kart_processHeadlightsSwitch();
      } else if (kart_turnIndicator == TURN_HAZARD) {
        // If hazards are on, temporarily set to OFF
        // to enable kart_processTurnIndicatorSwitch() to execute
        kart_stopTurnIndicator();
        kart_turnIndicator = TURN_OFF;

        // Revert to state determined by indicator switch
        kart_processTurnIndicatorSwitch();

        // Illuminate button again if necessary
        kart_processHeadlightsSwitch();
      } else {
        if (kart_turnIndicator == TURN_LEFT) {
          // If hazard button is pressed while turn indicator is set to left,
          // enable orange flashing animation
          kart_stopTurnIndicator();
          kart_turnIndicator = TURN_OFF;
          kart_ledAnimation = ANIM_FLASH_ORANGE;
          kart_ledAnimationStart = millis();
        } else if (kart_turnIndicator == TURN_RIGHT) {
          // If hazard button is pressed while turn indicator is set to right,
          // enable blue flashing animation
          kart_stopTurnIndicator();
          kart_turnIndicator = TURN_OFF;
          kart_ledAnimation = ANIM_FLASH_BLUE;
          kart_ledAnimationStart = millis();
        } else {
          // Else, enable hazards
          kart_turnIndicator = TURN_HAZARD;
          kart_startTurnIndicator();
        }
      }
    }
  }

  void kart_processForwardReverseSwitch() {
    if (!kart_getInput(INPUT_POS_FORWARD)) {
      kart_direction = DIR_REVERSE;
      kart_startReverseBeep();
    } else {
      kart_direction = DIR_FORWARD;
      kart_stopReverseBeep();
    }
    kart_updateWS2812();
  }

  void kart_processControlModeSwitch() {
    if (!kart_getInput(INPUT_POS_CONTROL_MODE)) {
      if (kart_controlMode != CTRL_TRQ) kart_controlModeChanged = true;
      kart_controlMode = CTRL_TRQ;
    } else {
      if (kart_controlMode != CTRL_VLT) kart_controlModeChanged = true;
      kart_controlMode = CTRL_VLT;
    }
  }

  void kart_processHornButton() {
    kart_setHorn(kart_getInput(INPUT_POS_HORN));
  }

  void kart_processTurnIndicatorSwitch() {
    // Ignore switch if animation is active
    if (kart_ledAnimation != ANIM_NONE) return;

    // Ignore switch if hazards are on
    if (kart_turnIndicator == TURN_HAZARD) return;

    if (!kart_getInput(INPUT_POS_INDICATOR_LEFT) && !kart_getInput(INPUT_POS_INDICATOR_RIGHT)) {
      // State: No turn
      kart_stopTurnIndicator();
      kart_turnIndicator = TURN_OFF;
    } else if (kart_getInput(INPUT_POS_INDICATOR_LEFT) && !kart_getInput(INPUT_POS_INDICATOR_RIGHT)) {
      // State: Turn left
      kart_turnIndicator = TURN_LEFT;
      kart_startTurnIndicator();
    } else if (!kart_getInput(INPUT_POS_INDICATOR_LEFT) && kart_getInput(INPUT_POS_INDICATOR_RIGHT)) {
      // State: Turn right
      kart_turnIndicator = TURN_RIGHT;
      kart_startTurnIndicator();
    }
  }

  void kart_processIgnitionButton() {
    if (kart_getInput(INPUT_POS_IGNITION)) {
      if (kart_ignitionOn == true) {
        // If ignition is on, shut off
        kart_ignitionOn = false;
      } else {
        // Else, start up
        kart_ignitionOn = true;
      }
    }
  }

  void kart_loop() {
    uint64_t now = millis();
    kart_updateInputs();
    if (kart_inputChanged(INPUT_POS_IGNITION)) kart_processIgnitionButton();
    kart_setOutput(OUTPUT_POS_IND_IGNITION, kart_ignitionOn);
    kart_prevThrottleInput = kart_throttleInput;
    kart_prevBrakeInput = kart_brakeInput;
    kart_prevBrakeLightState = kart_brakeLightState;
    // Limit rising throttle but allow instant falling for safety
    kart_throttleInput = kart_adc_rate_limit(MEDIANFILTER_Insert(&kart_medianFilterThrottle, kart_prepare_adc_value(analogRead(PIN_ANALOG_THROTTLE), kart_adc_calibration_values.minThrottle, kart_adc_calibration_values.maxThrottle, 0, THROTTLE_MAX)), kart_prevThrottleInput, THROTTLE_RATE_LIMIT, 10000);
    kart_brakeInput = MEDIANFILTER_Insert(&kart_medianFilterBrake, kart_prepare_adc_value(analogRead(PIN_ANALOG_BRAKE), kart_adc_calibration_values.minBrake, kart_adc_calibration_values.maxBrake, 0, BRAKE_MAX));

    if (kart_throttleInput > 0 || kart_brakeInput > 0) kart_lastActivity = now;

    switch (kart_state) {
      case STATE_SHUTDOWN:
        {
          // Check for ignition on
          if (kart_ignitionOn == true) {
            if (!kart_getInput(INPUT_POS_FORWARD) && kart_getInput(INPUT_POS_INDICATOR_RIGHT) && kart_getInput(INPUT_POS_INDICATOR_HAZARD) && kart_getInput(INPUT_POS_HORN)) {
              // If Reverse, Indicator Right, Hazard and Horn buttons/switches are pressed during startup, enter manual board enable mode
              // This can be used when reprogramming the mainboards
              kart_start_manual_enable_mode();
              kart_state = STATE_MANUAL_ENABLE;
            } else if (kart_getInput(INPUT_POS_INDICATOR_HAZARD) && kart_getInput(INPUT_POS_HORN)) {
              // If Hazard and Horn buttons are pressed during startup, enter ADC calibration mode
              kart_calibrate_adc();
              kart_state = STATE_ADC_CALIBRATION;
            } else {
              kart_startup();
              kart_state = STATE_STARTING_UP;
            }
          }
          break;
        }

      case STATE_SHUTDOWN_INACTIVITY:
        {
          // Check for ignition off
          if (kart_ignitionOn == false) kart_state = STATE_SHUTDOWN;
          break;
        }

      case STATE_ADC_CALIBRATION:
        {
          kart_adc_calibration_loop();
          if (kart_smAdcCalibration.state == AC_END) {
            kart_startup();
            kart_state = STATE_STARTING_UP;
          }
          break;
        }

      case STATE_MANUAL_ENABLE:
        {
          kart_manual_enable_loop();
          if (kart_smManualEnable.state == ME_END) {
            kart_state = STATE_SHUTDOWN;
          }
          break;
        }

      case STATE_STARTING_UP:
        {
          kart_startup_loop();
          kart_turnIndicator_loop();
          if (kart_smStartup.state == ST_END) kart_state = STATE_OPERATIONAL;
          break;
        }

      case STATE_OPERATIONAL:
        {
          kart_turnIndicator_loop();
          kart_reverseBeep_loop();
          kart_operation_loop();

          // Check for ignition off
          if (kart_ignitionOn == false) {
            kart_shutdown();
            kart_state = STATE_SHUTTING_DOWN;
          }

          // Check for inactivity timeout
          if (now - kart_lastActivity > INACTIVITY_TIMEOUT) {
            kart_shutdown();
            kart_state = STATE_SHUTTING_DOWN_INACTIVITY;
          }
          break;
        }

      case STATE_SHUTTING_DOWN:
      case STATE_SHUTTING_DOWN_INACTIVITY:
        {
          kart_shutdown_loop();
          kart_turnIndicator_loop();
          if (kart_smShutdown.state == SD_END) kart_state = (kart_state == STATE_SHUTTING_DOWN_INACTIVITY ? STATE_SHUTDOWN_INACTIVITY : STATE_SHUTDOWN);
          break;
        }
    }

    kart_updateOutputs();
  }

  void kart_adc_calibration_loop() {
    uint64_t now = millis();
    uint64_t totalTimePassed = now - kart_smAdcCalibration.startTime;
    uint64_t stepTimePassed = now - kart_smAdcCalibration.stepStartTime;

    switch (kart_smAdcCalibration.state) {
      case AC_START:
        {
          // Beep: ADC calibration start
          tone(PIN_BUZZER, 2000);
          kart_smAdcCalibration.state = AC_BEEP_START;
          kart_smAdcCalibration.stepStartTime = now;
          break;
        }

      case AC_BEEP_START:
        {
          if (stepTimePassed >= 500) {
            // Stop beep
            noTone(PIN_BUZZER);

            // Prepare calibration
#ifdef SERIAL_DEBUG
            Serial.println("Starting ADC calibration");
#endif
            kart_adc_calibration_values.minThrottle = 1023;
            kart_adc_calibration_values.maxThrottle = 0;
            kart_adc_calibration_values.minBrake = 1023;
            kart_adc_calibration_values.maxBrake = 0;

            kart_smAdcCalibration.state = AC_CALIBRATE;
            kart_smAdcCalibration.stepStartTime = now;
          }
          break;
        }

      case AC_CALIBRATE:
        {
          // Do calibration
          int16_t adcThrottle = analogRead(PIN_ANALOG_THROTTLE);
          int16_t adcBrake = analogRead(PIN_ANALOG_BRAKE);
          if (adcThrottle + (ADC_TOLERANCE / 2) < kart_adc_calibration_values.minThrottle) kart_adc_calibration_values.minThrottle = adcThrottle + (ADC_TOLERANCE / 2);
          if (adcThrottle - (ADC_TOLERANCE / 2) > kart_adc_calibration_values.maxThrottle) kart_adc_calibration_values.maxThrottle = adcThrottle - (ADC_TOLERANCE / 2);
          if (adcBrake + (ADC_TOLERANCE / 2) < kart_adc_calibration_values.minBrake) kart_adc_calibration_values.minBrake = adcBrake + (ADC_TOLERANCE / 2);
          if (adcBrake - (ADC_TOLERANCE / 2) > kart_adc_calibration_values.maxBrake) kart_adc_calibration_values.maxBrake = adcBrake - (ADC_TOLERANCE / 2);

          if (stepTimePassed >= 5000) {
            // Finish calibration
#ifdef SERIAL_DEBUG
            Serial.println("New ADC calibration values:");
            Serial.print("  Throttle Min: ");
            Serial.println(kart_adc_calibration_values.minThrottle);
            Serial.print("  Throttle Max: ");
            Serial.println(kart_adc_calibration_values.maxThrottle);
            Serial.print("  Brake Min:    ");
            Serial.println(kart_adc_calibration_values.minBrake);
            Serial.print("  Brake Max:    ");
            Serial.println(kart_adc_calibration_values.maxBrake);
#endif

            if ((kart_adc_calibration_values.maxThrottle - kart_adc_calibration_values.minThrottle < 50 && kart_adc_calibration_values.maxThrottle - kart_adc_calibration_values.minThrottle > -50)
                || (kart_adc_calibration_values.maxBrake - kart_adc_calibration_values.minBrake < 50 && kart_adc_calibration_values.maxBrake - kart_adc_calibration_values.minBrake > -50)) {  // Spread too narrow
#ifdef SERIAL_DEBUG
              Serial.println("Aborting calibration - spread too narrow!");
              // Beep: ADC calibration end (Fail)
              tone(PIN_BUZZER, 200);
              kart_smAdcCalibration.state = AC_BEEP_END;
              kart_smAdcCalibration.stepStartTime = now;
              break;
#endif
            }

            EEPROM.write(0x00, kart_adc_calibration_values.minThrottle >> 8);
            EEPROM.write(0x01, kart_adc_calibration_values.minThrottle & 0xFF);
            EEPROM.write(0x02, kart_adc_calibration_values.maxThrottle >> 8);
            EEPROM.write(0x03, kart_adc_calibration_values.maxThrottle & 0xFF);
            EEPROM.write(0x04, kart_adc_calibration_values.minBrake >> 8);
            EEPROM.write(0x05, kart_adc_calibration_values.minBrake & 0xFF);
            EEPROM.write(0x06, kart_adc_calibration_values.maxBrake >> 8);
            EEPROM.write(0x07, kart_adc_calibration_values.maxBrake & 0xFF);

            // Beep: ADC calibration end (Success)
            tone(PIN_BUZZER, 4000);
            kart_smAdcCalibration.state = AC_BEEP_END;
            kart_smAdcCalibration.stepStartTime = now;
          }
          break;
        }

      case AC_BEEP_END:
        {
          if (stepTimePassed >= 500) {
            // Stop beep
            noTone(PIN_BUZZER);
            kart_smAdcCalibration.state = AC_END;
            kart_smAdcCalibration.stepStartTime = now;
          }
          break;
        }
    }
  }

  void kart_manual_enable_loop() {
    uint64_t now = millis();
    uint64_t totalTimePassed = now - kart_smManualEnable.startTime;
    uint64_t stepTimePassed = now - kart_smManualEnable.stepStartTime;

    switch (kart_smManualEnable.state) {
      case ME_START:
        {
          // Beep: Manual enable start
          tone(PIN_BUZZER, 200);
          kart_smManualEnable.state = ME_BEEP_START;
          kart_smManualEnable.stepStartTime = now;
          break;
        }

      case ME_BEEP_START:
        {
          if (stepTimePassed >= 500) {
            // Stop beep
            noTone(PIN_BUZZER);
            kart_smManualEnable.state = ME_WAIT_RELEASE;
            kart_smManualEnable.stepStartTime = now;
          }
          break;
        }

      case ME_WAIT_RELEASE:
        {
          // Wait until Hazard and Horn buttons are released
          if (!kart_getInput(INPUT_POS_INDICATOR_HAZARD) && !kart_getInput(INPUT_POS_HORN)) {
            kart_smManualEnable.state = ME_ACTIVE;
            kart_smManualEnable.stepStartTime = now;
          }
          break;
        }

      case ME_ACTIVE:
        {
          // Pass-through from buttons to enable outputs
          kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT, kart_getInput(INPUT_POS_INDICATOR_HAZARD));
          kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR, kart_getInput(INPUT_POS_HORN));

          // Leave mode when ignition is turned off
          if (kart_ignitionOn == false) {
            // Beep: Manual enable end
            tone(PIN_BUZZER, 4000);
            kart_smManualEnable.state = ME_BEEP_END;
            kart_smManualEnable.stepStartTime = now;
          }
          break;
        }

      case ME_BEEP_END:
        {
          if (stepTimePassed >= 500) {
            // Stop beep
            noTone(PIN_BUZZER);
            kart_smManualEnable.state = ME_END;
            kart_smManualEnable.stepStartTime = now;
          }
          break;
        }
    }
  }

  void kart_startup_loop() {
    uint64_t now = millis();
    uint64_t totalTimePassed = now - kart_smStartup.startTime;
    uint64_t stepTimePassed = now - kart_smStartup.stepStartTime;

    switch (kart_smStartup.state) {
      case ST_START:
        {
          // Test buzzer
          tone(PIN_BUZZER, 1500);

          // Turn on all indicators
          kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, 1);
          kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, 1);
          kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, 1);
          kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, 1);
          kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 1);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
          kart_setOutput(OUTPUT_POS_IND_HORN, 1);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 1);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 1);

          // Set LED strip to red
          for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
            kart_ws2812.setPixelColor(i, kart_ws2812.Color(255, 0, 0));
          }
          kart_ws2812.show();
          kart_smStartup.state = ST_ALL_ON;
          kart_smStartup.stepStartTime = now;
          break;
        }

      case ST_ALL_ON:
        {
          if (stepTimePassed >= 50) {
            // Turn off buzzer
            noTone(PIN_BUZZER);
            kart_smStartup.state = ST_BUZZER_OFF;
            kart_smStartup.stepStartTime = now;
          }
          break;
        }

      case ST_BUZZER_OFF:
        {
          if (totalTimePassed >= 666) {
            // Set LED strip to green
            for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
              kart_ws2812.setPixelColor(i, kart_ws2812.Color(0, 255, 0));
            }
            kart_ws2812.show();
            kart_smStartup.state = ST_WS2812_GREEN;
            kart_smStartup.stepStartTime = now;
          }
          break;
        }

      case ST_WS2812_GREEN:
        {
          if (stepTimePassed >= 666) {
            // Set LED strip to blue
            for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
              kart_ws2812.setPixelColor(i, kart_ws2812.Color(0, 0, 255));
            }
            kart_ws2812.show();
            kart_smStartup.state = ST_WS2812_BLUE;
            kart_smStartup.stepStartTime = now;
          }
          break;
        }

      case ST_WS2812_BLUE:
        {
          if (totalTimePassed >= 2000) {
            // Turn off all indicators and LED strip
            kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, 0);
            kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, 0);
            kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, 0);
            kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, 0);
            kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
            kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
            kart_setOutput(OUTPUT_POS_IND_HORN, 0);
            kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
            kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
            for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
              kart_ws2812.setPixelColor(i, kart_ws2812.Color(0, 0, 0));
            }
            kart_ws2812.show();

            // Enable mainboards
            kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT, 1);
            kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR, 1);
            kart_smStartup.state = ST_MAINBOARD_ENABLE;
            kart_smStartup.stepStartTime = now;
          }
          break;
        }

      case ST_MAINBOARD_ENABLE:
        {
          if (stepTimePassed >= 500) {
            // Stop enabling mainboards
            kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT, 0);
            kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR, 0);
            kart_smStartup.state = ST_MAINBOARD_DEADTIME;
            kart_smStartup.stepStartTime = now;
          }
          break;
        }

      case ST_MAINBOARD_DEADTIME:
        {
          if (stepTimePassed >= 0) {
            // Wait after enabling mainboards

            // Process all switches/buttons once to establish initial state
            kart_processMotorFrontEnableSwitch();
            kart_processMotorRearEnableSwitch();
            kart_processHeadlightsSwitch();
            kart_processHazardButton();
            kart_processForwardReverseSwitch();
            kart_processControlModeSwitch();
            kart_processHornButton();
            kart_processTurnIndicatorSwitch();

            kart_smStartup.state = ST_END;
            kart_smStartup.stepStartTime = now;
          }
          break;
        }
    }
  }

  void kart_shutdown_loop() {
    uint64_t now = millis();
    uint64_t totalTimePassed = now - kart_smShutdown.startTime;
    uint64_t stepTimePassed = now - kart_smShutdown.stepStartTime;

    switch (kart_smShutdown.state) {
      case SD_START:
        {
          // Turn off horn
          kart_setHorn(0);

          // If an animation is active, disable it
          kart_ledAnimation = ANIM_NONE;

          // Turn on buzzer
          tone(PIN_BUZZER, 750);

          // Disable mainboards
          kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT, 1);
          kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR, 1);
          kart_smShutdown.state = SD_MAINBOARD_DISABLE;
          kart_smShutdown.stepStartTime = now;
          break;
        }

      case SD_MAINBOARD_DISABLE:
        {
          if (stepTimePassed >= 50) {
            // Turn off buzzer
            noTone(PIN_BUZZER);
            kart_smShutdown.state = SD_BUZZER_OFF;
            kart_smShutdown.stepStartTime = now;
          }
          break;
        }

      case SD_BUZZER_OFF:
        {
          if (stepTimePassed >= 450) {
            // Stop disabling mainboards
            kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT, 0);
            kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR, 0);
            kart_smShutdown.state = SD_MAINBOARD_DEADTIME;
            kart_smShutdown.stepStartTime = now;
          }
          break;
        }

      case SD_MAINBOARD_DEADTIME:
        {
          if (stepTimePassed >= 2000) {
            // Wait after disabling mainboards

            // Turn off headlights and turn indicators
            kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 0);
            kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 0);
            kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 0);
            kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 0);
            kart_headlights = HL_OFF;
            kart_stopTurnIndicator();
            kart_turnIndicator = TURN_OFF;

            // Turn off all indicators
            kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, 0);
            kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, 0);
            kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, 0);
            kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, 0);
            kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
            kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
            kart_setOutput(OUTPUT_POS_IND_HORN, 0);
            kart_smShutdown.state = SD_END;
            kart_smShutdown.stepStartTime = now;
          }
          break;
        }
    }
  }

  void kart_turnIndicator_loop() {
    uint64_t now = millis();
    uint64_t totalTimePassed = now - kart_smTurnIndicator.startTime;
    uint64_t stepTimePassed = now - kart_smTurnIndicator.stepStartTime;

    switch (kart_smTurnIndicator.state) {
      case TI_START:
      case TI_OFF:
        {
          // Turn off DRL when starting
          if (kart_smTurnIndicator.state == TI_START) {
            if (kart_turnIndicator == TURN_LEFT || kart_turnIndicator == TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 0);
            if (kart_turnIndicator == TURN_RIGHT || kart_turnIndicator == TURN_HAZARD) kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 0);
          }

          // If TI_START, no delay is needed to ensure that indicators turn on right away
          if (kart_smTurnIndicator.state == TI_START || stepTimePassed >= 345) {
            // Start beep
            tone(PIN_BUZZER, 1500);

            // Turn on indicator
            switch (kart_turnIndicator) {
              case TURN_OFF:
                {
                  // This shouldn't be needed
                  kart_setOutput(OUTPUT_POS_INDICATOR_LEFT, 0);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
                  kart_setOutput(OUTPUT_POS_INDICATOR_RIGHT, 0);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
                  break;
                }

              case TURN_LEFT:
                {
                  kart_setOutput(OUTPUT_POS_INDICATOR_LEFT, 1);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 1);
                  kart_setOutput(OUTPUT_POS_INDICATOR_RIGHT, 0);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
                  break;
                }

              case TURN_RIGHT:
                {
                  kart_setOutput(OUTPUT_POS_INDICATOR_LEFT, 0);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
                  kart_setOutput(OUTPUT_POS_INDICATOR_RIGHT, 1);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 1);
                  break;
                }

              case TURN_HAZARD:
                {
                  kart_setOutput(OUTPUT_POS_INDICATOR_LEFT, 1);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 1);
                  kart_setOutput(OUTPUT_POS_INDICATOR_RIGHT, 1);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 1);
                  kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
                  break;
                }
            }

            // Set animation variables
            kart_turnIndicatorNumActiveLEDs = 1;
            kart_turnIndicatorLastAnimationUpdate = now;

            kart_smTurnIndicator.state = TI_ON_BEEP;
            kart_smTurnIndicator.stepStartTime = now;
            kart_updateWS2812();
          }
          break;
        }

      case TI_ON_BEEP:
        {
          if (stepTimePassed >= 5) {
            // Stop beep
            noTone(PIN_BUZZER);
            kart_smTurnIndicator.state = TI_ON;
            kart_smTurnIndicator.stepStartTime = now;
          }
          break;
        }

      case TI_ON:
        {
          // Update LED animation.
          // As long as WS2812_TURN_INDICATOR_ANIM_INTERVAL is longer than the beep duration,
          // we only need to check this in state TI_ON
          if (now - kart_turnIndicatorLastAnimationUpdate >= WS2812_TURN_INDICATOR_ANIM_INTERVAL) {
            if (kart_turnIndicatorNumActiveLEDs < (WS2812_SEG_0_END - WS2812_SEG_0_START + 1)) {
              kart_turnIndicatorNumActiveLEDs++;
              kart_updateWS2812();
              kart_turnIndicatorLastAnimationUpdate = now;
            }
          }

          if (stepTimePassed >= 345) {
            // Start beep
            tone(PIN_BUZZER, 1200);

            // Turn off indicator
            kart_setOutput(OUTPUT_POS_INDICATOR_LEFT, 0);
            kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
            kart_setOutput(OUTPUT_POS_INDICATOR_RIGHT, 0);
            kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
            if (kart_turnIndicator == TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
            kart_smTurnIndicator.state = TI_OFF_BEEP;
            kart_smTurnIndicator.stepStartTime = now;
            kart_updateWS2812();
          }
          break;
        }

      case TI_OFF_BEEP:
        {
          if (stepTimePassed >= 5) {
            // Stop beep
            noTone(PIN_BUZZER);
            kart_smTurnIndicator.state = TI_OFF;
            kart_smTurnIndicator.stepStartTime = now;
          }
          break;
        }

      case TI_END:
        {
          // Stop beep
          noTone(PIN_BUZZER);

          // Turn off indicator
          kart_setOutput(OUTPUT_POS_INDICATOR_LEFT, 0);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
          kart_setOutput(OUTPUT_POS_INDICATOR_RIGHT, 0);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
          if (kart_turnIndicator == TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
          kart_updateWS2812();

          // Turn DRL back on
          if (kart_headlights != HL_OFF) {
            kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
            kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
          }

          kart_smTurnIndicator.state = TI_INACTIVE;
          kart_smTurnIndicator.stepStartTime = now;
          break;
        }

      case TI_INACTIVE:
        {
          // Nothing to do
          break;
        }
    }
  }

  void kart_reverseBeep_loop() {
    uint64_t now = millis();
    uint64_t totalTimePassed = now - kart_smReverseBeep.startTime;
    uint64_t stepTimePassed = now - kart_smReverseBeep.stepStartTime;

    switch (kart_smReverseBeep.state) {
      case RB_START:
      case RB_PAUSE:
        {
          // If RB_START, no delay is needed to ensure that we beep right away
          if (kart_smReverseBeep.state == RB_START || stepTimePassed >= 500) {
            // Start beep
            tone(PIN_BUZZER, 2000);
            kart_smReverseBeep.state = RB_BEEP;
            kart_smReverseBeep.stepStartTime = now;
          }
          break;
        }

      case RB_BEEP:
        {
          if (stepTimePassed >= 500) {
            // Stop beep
            noTone(PIN_BUZZER);
            kart_smReverseBeep.state = RB_PAUSE;
            kart_smReverseBeep.stepStartTime = now;
          }
          break;
        }

      case RB_END:
        {
          // Stop beep
          noTone(PIN_BUZZER);
          kart_smReverseBeep.state = RB_INACTIVE;
          kart_smReverseBeep.stepStartTime = now;
          break;
        }

      case RB_INACTIVE:
        {
          // Nothing to do
          break;
        }
    }
  }

  void kart_operation_loop() {
    // Check Motor Front Enable switch
    if (kart_inputChanged(INPUT_POS_MOTOR_FRONT_ENABLE)) {
      kart_processMotorFrontEnableSwitch();
    }

    // Check Motor Rear Enable switch
    if (kart_inputChanged(INPUT_POS_MOTOR_REAR_ENABLE)) {
      kart_processMotorRearEnableSwitch();
    }

    // Check Headlights switch
    if (kart_inputChanged(INPUT_POS_HEADLIGHTS_LOW) || kart_inputChanged(INPUT_POS_HEADLIGHTS_HIGH)) {
      kart_processHeadlightsSwitch();
    }

    // Check Hazard button
    if (kart_inputChanged(INPUT_POS_INDICATOR_HAZARD)) {
      kart_processHazardButton();
    }

    // Check Forward/Reverse switch
    if (kart_inputChanged(INPUT_POS_FORWARD)) {
      kart_processForwardReverseSwitch();
    }

    // Check Control Mode switch
    if (kart_inputChanged(INPUT_POS_CONTROL_MODE)) {
      kart_processControlModeSwitch();
    }

    // Check Horn button
    if (kart_inputChanged(INPUT_POS_HORN)) {
      kart_processHornButton();
    }

    // Check Turn Indicator switch
    if (kart_inputChanged(INPUT_POS_INDICATOR_LEFT) || kart_inputChanged(INPUT_POS_INDICATOR_RIGHT)) {
      kart_processTurnIndicatorSwitch();
    }

    // If an animation is active, keep updating the LEDs
    if (kart_ledAnimation != ANIM_NONE) {
      kart_updateWS2812();
    }

    uint64_t now = millis();
    if (now - kart_lastUsartUpdate >= USART_TX_INTERVAL) {
      kart_lastUsartUpdate = now;

#ifdef SERIAL_DEBUG
      Serial.print("Throttle In:  ");
      Serial.println(kart_throttleInput);
      Serial.print("Brake In:     ");
      Serial.println(kart_brakeInput);
#endif

      int16_t processedThrottleInputFront = kart_throttleInput;
      int16_t processedBrakeInputFront = kart_brakeInput;
      int16_t throttleOutputFront = 0;
      int16_t processedBrakeInputRear = kart_brakeInput;
      int16_t processedThrottleInputRear = kart_throttleInput;
      int16_t throttleOutputRear = 0;

      // Calculate speed average - right wheel is subtracted because it spins in the opposite direction
      int16_t speedAvgFront = (kart_feedbackFront.speedL_meas - kart_feedbackFront.speedR_meas) / 2;
      int16_t speedAvgRear = (kart_feedbackRear.speedL_meas - kart_feedbackRear.speedR_meas) / 2;
      int16_t speedAvgAbsFront = speedAvgFront < 0 ? -speedAvgFront : speedAvgFront;
      int16_t speedAvgAbsRear = speedAvgRear < 0 ? -speedAvgRear : speedAvgRear;

      // Calculate speedBlend: 0...1 maps to 10...60 rpm
      float speedBlendFront = clamp_float(map_float(speedAvgAbsFront, 10.0f, 60.0f, 0.0f, 1.0f), 0.0f, 1.0f);
      float speedBlendRear = clamp_float(map_float(speedAvgAbsRear, 10.0f, 60.0f, 0.0f, 1.0f), 0.0f, 1.0f);

      // To get the overall abs speed value, use a median filter with the lower of front and back abs speed.
      // Use the lower of the two because it is more likely that wheels are slipping than blocked.
      kart_prevSpeedAbs = kart_speedAbs;
      kart_speedAbs = MEDIANFILTER_Insert(&kart_medianFilterSpeed, ((speedAvgAbsFront < speedAvgAbsRear) ? speedAvgAbsFront : speedAvgAbsRear));
      int16_t kart_acceleration = kart_speedAbs - kart_prevSpeedAbs;

      // Determine if the brake light should be enabled

      // Always enable when brake is pressed
      kart_brakeLightState = (kart_brakeInput > 0);

      // Also enable on sufficient deceleration (if speed is above minimum)
      if ((kart_acceleration <= BRAKE_LIGHT_DECEL_THRESHOLD) && (kart_speedAbs >= BRAKE_LIGHT_MIN_SPEED)) {
        kart_brakeLightState = 1;
        kart_brakeLightDecelTime = now;
      }

      // Also enable if minimum on time after auto activation has not yet passed
      if (now - kart_brakeLightDecelTime < BRAKE_LIGHT_DECEL_AFTER_TIME) kart_brakeLightState = 1;

      // Update brake lights if brake light state has changed
      if (kart_brakeLightState != kart_prevBrakeLightState) kart_updateWS2812();

      // At small speeds, fade out throttle if brake is pressed
      if (processedBrakeInputFront > 30) {
        processedThrottleInputFront *= speedBlendFront;
      }
      if (processedBrakeInputRear > 30) {
        processedThrottleInputRear *= speedBlendRear;
      }

      // Make sure brake always works against the current direction
      if (speedAvgFront > 0) {
        // Moving forward
        processedBrakeInputFront = -processedBrakeInputFront * speedBlendFront;
      } else {
        // Moving backward
        processedBrakeInputFront = processedBrakeInputFront * speedBlendFront;
      }
      if (speedAvgRear > 0) {
        // Moving forward
        processedBrakeInputRear = -processedBrakeInputRear * speedBlendRear;
      } else {
        // Moving backward
        processedBrakeInputRear = processedBrakeInputRear * speedBlendRear;
      }

      // Handle forward / reverse
      if (kart_direction == DIR_REVERSE) {
        throttleOutputFront = processedBrakeInputFront - processedThrottleInputFront;
        throttleOutputRear = processedBrakeInputRear - processedThrottleInputRear;
      } else {
        throttleOutputFront = processedBrakeInputFront + processedThrottleInputFront;
        throttleOutputRear = processedBrakeInputRear + processedThrottleInputRear;
      }

      // Set setpoints
      if (((throttleOutputFront > 0) != (speedAvgFront > 0) /* if signs differ */) && (speedAvgFront > 60)) {
        // When braking at a reasonably high speed, always send to both boards
        kart_setpointFront = throttleOutputFront;
      } else {
        // Otherwise, only send to enabled boards
        kart_setpointFront = kart_motorFrontEnabled ? throttleOutputFront : 0;
      }
      if (((throttleOutputRear > 0) != (speedAvgRear > 0) /* if signs differ */) && (speedAvgRear > 60)) {
        // When braking at a reasonably high speed, always send to both boards
        kart_setpointRear = throttleOutputRear;
      } else {
        // Otherwise, only send to enabled boards
        kart_setpointRear = kart_motorRearEnabled ? throttleOutputRear : 0;
      }

      // If a board has an error, always set setpoint to 0
      // Probably not sensible
      //if (feedbackErrorFront) kart_setpointFront = 0;
      //if (feedbackErrorRear) kart_setpointRear = 0;

      // Hard output limit
      if (kart_setpointFront < OUTPUT_HARD_LIMIT_MIN) kart_setpointFront = OUTPUT_HARD_LIMIT_MIN;
      if (kart_setpointFront > OUTPUT_HARD_LIMIT_MAX) kart_setpointFront = OUTPUT_HARD_LIMIT_MAX;
      if (kart_setpointRear < OUTPUT_HARD_LIMIT_MIN) kart_setpointRear = OUTPUT_HARD_LIMIT_MIN;
      if (kart_setpointRear > OUTPUT_HARD_LIMIT_MAX) kart_setpointRear = OUTPUT_HARD_LIMIT_MAX;

#ifdef SERIAL_DEBUG
      Serial.print("Setpoint Front: ");
      Serial.println(kart_setpointFront);
      Serial.print("Setpoint Rear:  ");
      Serial.println(kart_setpointRear);
#endif

      // If control mode needs to be changed and speed is low enough, change control mode
      if (kart_controlModeChanged == true && kart_speedAbs <= MODE_CHANGE_MAX_SPEED) {
        kart_sendControlModeFront(kart_controlMode);
        kart_sendControlModeRear(kart_controlMode);
        kart_controlModeChanged = false;
      }

      /*
         Feedback is read AFTER sending the setpoints. This is obviously suboptimal since it means that the setpoints
         Get calculated based on outdated feedback. However, this solves one particular issue:
         If the feedback is read from both boards before sending the setpoints to both boards,
         since the reading process flushes the input buffer and waits for a new frame, it effectively synchronizes
         the main loop to the feedback. Now, if you read front first and rear afterwards, the loop is synced to rear.
         This means that the setpoints get sent with a very reproducible timing in relation to the rear feedback.
         This leads to a "beat frequency" effect with the feedback of the front board, which will always be just slightly off
         compared to the rear board. Thus, the front commands have no defined relation to the front feedback.
         This shouldn't matter, but it does. For some unknown reason, when the command overlays the end of the feedback,
         even though they are on separate wires, this causes the motor controller to not receive the data correctly.
         Since the commands are synced to the feedback, this will occur for many subsequent loop cycles, accumulating
         until the serial timeout period is reached and a timeout error occurs.
         Now, if we read the front feedback, then send the front setpoint, then do the same for rear, both commands
         are synced to their respective feedback and no beat effect can occur, thus preventing this
         serial timeout situation from arising.
         This necessitates reading at least one of the feedbacks after sending a new setpoint, and since both setpoints
         must always be identical, this means that the setpoints need to be calculated with at least one of two feedbacks
         being stale. This should be acceptable in practice, since the 50ms Tx interval is still pretty quick compared to
         the possible speed change of the kart. (if the speed changes significantly within 50ms, you have other problems)
      */

      // Read front feedback
      uint8_t feedbackErrorFront = kart_readFeedbackFront();
      if (feedbackErrorFront) {
        kart_feedbackErrorCounterFront++;
        if (kart_feedbackErrorCounterFront >= 3) {
          kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, true);
        }
      } else {
        kart_feedbackErrorCounterFront = 0;
        kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, false);
      }

      // Send front setpoint
      kart_sendSetpointFront(kart_setpointFront);

      // Read rear feedback
      uint8_t feedbackErrorRear = kart_readFeedbackRear();
      if (feedbackErrorRear) {
        kart_feedbackErrorCounterRear++;
        if (kart_feedbackErrorCounterRear >= 3) {
          kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, true);
        }
      } else {
        kart_feedbackErrorCounterRear = 0;
        kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, false);
      }

      // Send rear setpoint
      kart_sendSetpointRear(kart_setpointRear);
    }
  }
