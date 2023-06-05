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

#include "kart.h"
#include "MedianFilter.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>


static Adafruit_NeoPixel kart_ws2812(WS2812_NUM_LEDS, PIN_WS2812_DATA, NEO_GRB + NEO_KHZ800);
static SoftwareSerial kart_swuartFront(PIN_SWUART_F_RX, PIN_SWUART_F_TX);
static SoftwareSerial kart_swuartRear(PIN_SWUART_R_RX, PIN_SWUART_R_TX);
static kart_serial_command_t kart_commandFront = { 0, 0, 0, 0 };
static kart_serial_command_t kart_commandRear = { 0, 0, 0, 0 };
kart_serial_feedback_t kart_feedbackFront = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
kart_serial_feedback_t kart_feedbackRear = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

static kart_input_t kart_inputs[NUM_INPUTS] = { { 0, 0, 0, 0, 0, 0 } };
static kart_output_t kart_outputs[NUM_OUTPUTS] = { { 0 } };
static sMedianFilter_t kart_medianFilterThrottle;
static sMedianNode_t kart_medianBufferThrottle[ADC_FILTER_SIZE];
static sMedianFilter_t kart_medianFilterBrake;
static sMedianNode_t kart_medianBufferBrake[ADC_FILTER_SIZE];
static int16_t kart_throttleInput = 0;
static int16_t kart_brakeInput = 0;
static int16_t kart_throttleOutput = 0;
static int16_t kart_prevThrottleOutput = 0;
static int16_t kart_setpointFront = 0;
static int16_t kart_setpointRear = 0;

static uint8_t kart_motorFrontEnabled = 0;
static uint8_t kart_motorRearEnabled = 0;

static kart_state_t kart_state = STATE_SHUTDOWN;
static kart_headlights_t kart_headlights = HL_OFF;
static kart_direction_t kart_direction = DIR_FORWARD;
static kart_turn_indicator_t kart_turnIndicator = TURN_OFF;
static kart_adc_calibration_values_t kart_adc_calibration_values = { 0, 0, 0, 0 };

static kart_stateMachine_t kart_smAdcCalibration = { AC_START, 0, 0 };
static kart_stateMachine_t kart_smStartup = { ST_START, 0, 0 };
static kart_stateMachine_t kart_smShutdown = { SD_START, 0, 0 };
static kart_stateMachine_t kart_smTurnIndicator = { TI_INACTIVE, 0, 0 };
static kart_stateMachine_t kart_smReverseBeep = { RB_INACTIVE, 0, 0 };


void kart_init() {
  kart_medianFilterThrottle.numNodes = ADC_FILTER_SIZE;
  kart_medianFilterThrottle.medianBuffer = kart_medianBufferThrottle;
  MEDIANFILTER_Init(&kart_medianFilterThrottle);

  kart_medianFilterBrake.numNodes = ADC_FILTER_SIZE;
  kart_medianFilterBrake.medianBuffer = kart_medianBufferBrake;
  MEDIANFILTER_Init(&kart_medianFilterBrake);

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

  kart_swuartFront.begin(115200);
  kart_swuartRear.begin(115200);
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

void kart_updateWS2812() {
  kart_ws2812.clear();
  uint32_t seg0Color = 0;
  uint32_t seg1Color = 0;
  uint32_t seg2Color = 0;
  uint32_t seg3Color = 0;
  uint32_t seg4Color = 0;

  // Segment 0: Indicator > Brake > Light
  if (kart_turnIndicator == TURN_LEFT || kart_turnIndicator == TURN_HAZARD) {
    if (kart_smTurnIndicator.state == TI_ON_BEEP || kart_smTurnIndicator.state == TI_ON) {
      seg0Color = WS2812_COLOR_INDICATOR;
    }
  } else if (kart_brakeInput > 0) {
    seg0Color = WS2812_COLOR_BRAKE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg0Color = WS2812_COLOR_LIGHT;
  }

  // Segment 1: Brake > Reverse > Light
  if (kart_brakeInput > 0) {
    seg1Color = WS2812_COLOR_BRAKE;
  } else if (kart_direction == DIR_REVERSE) {
    seg1Color = WS2812_COLOR_REVERSE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg1Color = WS2812_COLOR_LIGHT;
  }

  // Segment 2: Reverse > Brake > Light
  if (kart_direction == DIR_REVERSE) {
    seg2Color = WS2812_COLOR_REVERSE;
  } else if (kart_brakeInput > 0) {
    seg2Color = WS2812_COLOR_BRAKE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg2Color = WS2812_COLOR_LIGHT;
  }

  // Segment 3: Brake > Reverse > Light
  if (kart_brakeInput > 0) {
    seg3Color = WS2812_COLOR_BRAKE;
  } else if (kart_direction == DIR_REVERSE) {
    seg3Color = WS2812_COLOR_REVERSE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg3Color = WS2812_COLOR_LIGHT;
  }

  // Segment 4: Indicator > Brake > Light
  if (kart_turnIndicator == TURN_RIGHT || kart_turnIndicator == TURN_HAZARD) {
    if (kart_smTurnIndicator.state == TI_ON_BEEP || kart_smTurnIndicator.state == TI_ON) {
      seg4Color = WS2812_COLOR_INDICATOR;
    }
  } else if (kart_brakeInput > 0) {
    seg4Color = WS2812_COLOR_BRAKE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg4Color = WS2812_COLOR_LIGHT;
  }

  for (uint8_t i = WS2812_SEG_0_START; i <= WS2812_SEG_0_END; i++) {
    kart_ws2812.setPixelColor(i, seg0Color);
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
    kart_ws2812.setPixelColor(i, seg4Color);
  }

  kart_ws2812.show();
}

void kart_sendSetpointFront(int16_t speed) {
  kart_commandFront.start = 0xABCD;
  kart_commandFront.steer = 0;
  kart_commandFront.speed = speed;
  kart_commandFront.checksum = (uint16_t)(kart_commandFront.start ^ kart_commandFront.steer ^ kart_commandFront.speed);
  kart_swuartFront.write((uint8_t *)&kart_commandFront, sizeof(kart_commandFront));

  // Enable "Active" LED if setpoint != 0
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, speed != 0);
}

void kart_sendSetpointRear(int16_t speed) {
  kart_commandRear.start = 0xABCD;
  kart_commandRear.steer = 0;
  kart_commandRear.speed = speed;
  kart_commandRear.checksum = (uint16_t)(kart_commandRear.start ^ kart_commandRear.steer ^ kart_commandRear.speed);
  kart_swuartRear.write((uint8_t *)&kart_commandRear, sizeof(kart_commandRear));

  // Enable "Active" LED if setpoint != 0
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, speed != 0);
}

uint8_t kart_readFeedback(SoftwareSerial *swuart, kart_serial_feedback_t *feedbackOut) {
  uint64_t startTime = millis();
  kart_serial_feedback_t feedbackIn = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // Wait for start value with timeout
  uint8_t waitingForByteNo = 0;
  while (millis() - startTime < FEEDBACK_RX_TIMEOUT) {
    if (!(swuart->available())) continue;
    uint8_t byte = swuart->read();
    if (waitingForByteNo == 0) {
      if (byte != 0xAB) continue;
      waitingForByteNo = 1;
    } else if (waitingForByteNo == 1) {
      if (byte != 0xCD) continue;
      break;
    }
  }
  if (millis() - startTime >= FEEDBACK_RX_TIMEOUT) return 1;

  // Manually set this since we just checked it and know it's correct
  feedbackIn.start = 0xABCD;

  // Start value (2 bytes) found, now read the remaining bytes with timeout
  uint8_t bytesRemaining = sizeof(kart_serial_feedback_t) - 2;
  uint8_t *pFeedback = (uint8_t *)&feedbackIn;
  while (millis() - startTime < FEEDBACK_RX_TIMEOUT) {
    if (!(swuart->available())) continue;
    *pFeedback = swuart->read();
    pFeedback++;
    bytesRemaining--;
    if (bytesRemaining == 0) break;
  }
  if (millis() - startTime >= FEEDBACK_RX_TIMEOUT) return 1;

  // Verify checksum
  uint16_t checksum;
  checksum = (uint16_t)(feedbackIn.start ^ feedbackIn.cmd1 ^ feedbackIn.cmd2 ^ feedbackIn.speedR_meas ^ feedbackIn.speedL_meas
                        ^ feedbackIn.batVoltage ^ feedbackIn.boardTemp ^ feedbackIn.cmdLed);

  // Check validity of the new data
  if (feedbackIn.start == 0xABCD && checksum == feedbackIn.checksum) {
    // Copy the new data
    memcpy(feedbackOut, &feedbackIn, sizeof(kart_serial_feedback_t));

    // Print data to built-in Serial
    /*Serial.print("1: ");
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
    Serial.println(feedbackOut->cmdLed);*/
  } else {
    //Serial.println("Non-valid data skipped");
  }

  return 0;
}

uint8_t kart_readFeedbackFront() {
  // Enable Rx on this channel
  kart_swuartFront.listen();
  return kart_readFeedback(&kart_swuartFront, &kart_feedbackFront);
}

uint8_t kart_readFeedbackRear() {
  // Enable Rx on this channel
  kart_swuartRear.listen();
  return kart_readFeedback(&kart_swuartRear, &kart_feedbackRear);
}

void kart_setHorn(uint8_t state) {
  digitalWrite(PIN_HORN, !!state);
}

void kart_calibrate_adc() {
  kart_smAdcCalibration.state = AC_START;
  kart_smAdcCalibration.startTime = millis();
  kart_smAdcCalibration.stepStartTime = kart_smAdcCalibration.startTime;
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
    kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 0);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 0);
    kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
    kart_setOutput(OUTPUT_POS_IND_HORN, 0);
    if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
    kart_headlights = HL_DRL;
  } else if (kart_getInput(INPUT_POS_HEADLIGHTS_LOW) && !kart_getInput(INPUT_POS_HEADLIGHTS_HIGH)) {
    // State: Low
    kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 0);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 1);
    kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 1);
    kart_setOutput(OUTPUT_POS_IND_HORN, 1);
    if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
    kart_headlights = HL_LOW;
  } else if (kart_getInput(INPUT_POS_HEADLIGHTS_LOW) && kart_getInput(INPUT_POS_HEADLIGHTS_HIGH)) {
    // State: High
    kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_LOW, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_LOW, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_LEFT_HIGH, 1);
    kart_setOutput(OUTPUT_POS_HEADLIGHT_RIGHT_HIGH, 1);
    kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 1);
    kart_setOutput(OUTPUT_POS_IND_HORN, 1);
    if (kart_turnIndicator != TURN_HAZARD) kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
    kart_headlights = HL_HIGH;
  }
  kart_updateWS2812();
}

void kart_processHazardButton() {
  if (kart_getInput(INPUT_POS_INDICATOR_HAZARD)) {
    if (kart_turnIndicator == TURN_HAZARD) {
      // If hazards are on, temporarily set to OFF
      // to enable kart_processTurnIndicatorSwitch() to execute
      kart_stopTurnIndicator();
      kart_turnIndicator = TURN_OFF;

      // Revert to state determined by indicator switch
      kart_processTurnIndicatorSwitch();

      // Illuminate button again if necessary
      kart_processHeadlightsSwitch();
    } else {
      // Else, enable hazards
      kart_turnIndicator = TURN_HAZARD;
      kart_startTurnIndicator();
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

void kart_processHornButton() {
  kart_setHorn(kart_getInput(INPUT_POS_HORN));
}

void kart_processTurnIndicatorSwitch() {
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

void kart_loop() {
  kart_updateInputs();
  kart_throttleInput = MEDIANFILTER_Insert(&kart_medianFilterThrottle, kart_prepare_adc_value(analogRead(PIN_ANALOG_THROTTLE), kart_adc_calibration_values.minThrottle, kart_adc_calibration_values.maxThrottle, 0, THROTTLE_MAX));
  kart_brakeInput = MEDIANFILTER_Insert(&kart_medianFilterBrake, kart_prepare_adc_value(analogRead(PIN_ANALOG_THROTTLE), kart_adc_calibration_values.minBrake, kart_adc_calibration_values.maxBrake, 0, BRAKE_MAX));

  switch (kart_state) {
    case STATE_SHUTDOWN:
      {
        // Check for ignition on
        if (kart_getInput(INPUT_POS_IGNITION)) {
          if (kart_getInput(INPUT_POS_INDICATOR_HAZARD) && kart_getInput(INPUT_POS_HORN)) {
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

    case STATE_ADC_CALIBRATION:
      {
        kart_adc_calibration_loop();
        if (kart_smAdcCalibration.state == AC_END) {
          kart_startup();
          kart_state = STATE_STARTING_UP;
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
        kart_operation_loop();
        kart_turnIndicator_loop();
        kart_reverseBeep_loop();

        // Check for ignition off
        if (!kart_getInput(INPUT_POS_IGNITION)) {
          kart_shutdown();
          kart_state = STATE_SHUTTING_DOWN;
        }
        break;
      }

    case STATE_SHUTTING_DOWN:
      {
        kart_shutdown_loop();
        kart_turnIndicator_loop();
        if (kart_smShutdown.state == SD_END) kart_state = STATE_SHUTDOWN;
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
        if (stepTimePassed >= 250) {
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

          // Process all switches/buttons once to establish initial state
          kart_processMotorFrontEnableSwitch();
          kart_processMotorRearEnableSwitch();
          kart_processHeadlightsSwitch();
          kart_processHazardButton();
          kart_processForwardReverseSwitch();
          kart_processHornButton();
          kart_processTurnIndicatorSwitch();

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
        if (stepTimePassed >= 2000) {
          // Wait after enabling mainboards

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

        // Disable mainboards
        kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_FRONT, 1);
        kart_setOutput(OUTPUT_POS_MOTOR_CONTROLLER_ENABLE_REAR, 1);
        kart_smShutdown.state = SD_MAINBOARD_DISABLE;
        kart_smShutdown.stepStartTime = now;
        break;
      }

    case SD_MAINBOARD_DISABLE:
      {
        if (stepTimePassed >= 500) {
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
          kart_smTurnIndicator.state = RB_PAUSE;
          kart_smTurnIndicator.stepStartTime = now;
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

  // Check Horn button
  if (kart_inputChanged(INPUT_POS_HORN)) {
    kart_processHornButton();
  }

  // Check Turn Indicator switch
  if (kart_inputChanged(INPUT_POS_INDICATOR_LEFT) || kart_inputChanged(INPUT_POS_INDICATOR_RIGHT)) {
    kart_processTurnIndicatorSwitch();
  }

  // Read feedback
  uint8_t feedbackErrorFront = kart_readFeedbackFront();
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, feedbackErrorFront);
  uint8_t feedbackErrorRear = kart_readFeedbackRear();
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, feedbackErrorRear);

  // Process throttle and brake
  // Limit throttle increase (both directions) but allow instant deceleration for safety
  // TODO: Use feedback here!
  kart_throttleOutput = kart_adc_rate_limit(kart_throttleInput - kart_brakeInput, kart_prevThrottleOutput, THROTTLE_RATE_LIMIT, 10000);
  kart_prevThrottleOutput = kart_throttleOutput;

  // Set setpoints
  if (kart_throttleOutput <= 0) {
    // When braking, always send to both boards
    kart_setpointFront = kart_throttleOutput;
    kart_setpointRear = kart_throttleOutput;
  } else {
    // Otherwise, only send to enabled boards
    kart_setpointFront = kart_motorFrontEnabled ? kart_throttleOutput : 0;
    kart_setpointRear = kart_motorRearEnabled ? kart_throttleOutput : 0;
  }

  // If a board has an error, always set setpoint to 0
  if (feedbackErrorFront) kart_setpointFront = 0;
  if (feedbackErrorRear) kart_setpointRear = 0;

  // Send setpoints
  kart_sendSetpointFront(kart_setpointFront);
  kart_sendSetpointRear(kart_setpointRear);
}