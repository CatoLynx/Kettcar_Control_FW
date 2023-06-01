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
#include <Adafruit_NeoPixel.h>


kart_input_t kartInputs[NUM_INPUTS] = { { 0, 0, 0, 0, 0, 0 } };
kart_output_t kartOutputs[NUM_OUTPUTS] = { { 0 } };
Adafruit_NeoPixel ws2812(WS2812_NUM_LEDS, PIN_WS2812_DATA, NEO_GRB + NEO_KHZ800);

// State of the kart
kart_state_t kart_state = STATE_SHUTDOWN;
kart_stateMachine_t kart_smStartup = { ST_START, 0, 0 };
kart_stateMachine_t kart_smShutdown = { SD_START, 0, 0 };
kart_stateMachine_t kart_smTurnIndicator = { TI_INACTIVE, 0, 0 };
kart_headlights_t kart_headlights = HL_OFF;
kart_direction_t kart_direction = DIR_FORWARD;
kart_turn_indicator_t kart_turnIndicator = TURN_OFF;


void kart_init() {
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

  ws2812.begin();
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
      kartInputs[listIndex].changed = 0;

      // Always set internal / raw state flags
      if (bit != kartInputs[listIndex].internalPrevState) {
        kartInputs[listIndex].internalLastChange = now;
      }
      kartInputs[listIndex].internalPrevState = kartInputs[listIndex].internalState;
      kartInputs[listIndex].internalState = bit;

      // If debounce deadtime passed without any further change, carry over the internal values
      if (now - kartInputs[listIndex].internalLastChange > INPUT_DEBOUNCE_TIME_MS) {
        if (bit != kartInputs[listIndex].state) {
          kartInputs[listIndex].changed = 1;
          kartInputs[listIndex].prevState = kartInputs[listIndex].state;
          kartInputs[listIndex].state = bit;
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
      if (kartOutputs[listIndex].state) {
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
  return kartInputs[pos].state;
}

uint8_t kart_inputChanged(uint8_t pos) {
  if (pos >= NUM_INPUTS) return 0;
  return kartInputs[pos].changed;
}

void kart_setOutput(uint8_t pos, uint8_t state) {
  if (pos >= NUM_OUTPUTS) return;
  kartOutputs[pos].state = !!state;
}

void kart_updateWS2812() {
  ws2812.clear();
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
  } else if (0 /* BRAKE */) {
    seg0Color = WS2812_COLOR_BRAKE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg0Color = WS2812_COLOR_LIGHT;
  }

  // Segment 1: Brake > Reverse > Light
  if (0 /* BRAKE */) {
    seg1Color = WS2812_COLOR_BRAKE;
  } else if (kart_direction == DIR_REVERSE) {
    seg1Color = WS2812_COLOR_REVERSE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg1Color = WS2812_COLOR_LIGHT;
  }

  // Segment 2: Reverse > Brake > Light
  if (kart_direction == DIR_REVERSE) {
    seg2Color = WS2812_COLOR_REVERSE;
  } else if (0 /* BRAKE */) {
    seg2Color = WS2812_COLOR_BRAKE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg2Color = WS2812_COLOR_LIGHT;
  }

  // Segment 3: Brake > Reverse > Light
  if (0 /* BRAKE */) {
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
  } else if (0 /* BRAKE */) {
    seg4Color = WS2812_COLOR_BRAKE;
  } else if (kart_headlights == HL_LOW || kart_headlights == HL_HIGH) {
    seg4Color = WS2812_COLOR_LIGHT;
  }

  for (uint8_t i = WS2812_SEG_0_START; i <= WS2812_SEG_0_END; i++) {
    ws2812.setPixelColor(i, seg0Color);
  }
  for (uint8_t i = WS2812_SEG_1_START; i <= WS2812_SEG_1_END; i++) {
    ws2812.setPixelColor(i, seg1Color);
  }
  for (uint8_t i = WS2812_SEG_2_START; i <= WS2812_SEG_2_END; i++) {
    ws2812.setPixelColor(i, seg2Color);
  }
  for (uint8_t i = WS2812_SEG_3_START; i <= WS2812_SEG_3_END; i++) {
    ws2812.setPixelColor(i, seg3Color);
  }
  for (uint8_t i = WS2812_SEG_4_START; i <= WS2812_SEG_4_END; i++) {
    ws2812.setPixelColor(i, seg4Color);
  }

  ws2812.show();
}

void kart_setHorn(uint8_t state) {
  digitalWrite(PIN_HORN, !!state);
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

void kart_processMotorFrontEnableSwitch() {
}

void kart_processMotorRearEnableSwitch() {
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
  } else {
    kart_direction = DIR_FORWARD;
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

  switch (kart_state) {
    case STATE_SHUTDOWN:
      {
        // Check for ignition on
        if (kart_getInput(INPUT_POS_IGNITION)) {
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
          ws2812.setPixelColor(i, ws2812.Color(255, 0, 0));
        }
        ws2812.show();
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
            ws2812.setPixelColor(i, ws2812.Color(0, 255, 0));
          }
          ws2812.show();
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
            ws2812.setPixelColor(i, ws2812.Color(0, 0, 255));
          }
          ws2812.show();
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
            ws2812.setPixelColor(i, ws2812.Color(0, 0, 0));
          }
          ws2812.show();

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
}