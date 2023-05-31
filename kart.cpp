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

#include "kart.h"
#include <Adafruit_NeoPixel.h>


kart_input_t kartInputs[NUM_INPUTS] = { { 0, 0, 0, 0, 0, 0 } };
kart_output_t kartOutputs[NUM_OUTPUTS] = { { 0 } };
Adafruit_NeoPixel ws2812(WS2812_NUM_LEDS, PIN_WS2812_DATA, NEO_GRB + NEO_KHZ800);

// State of the kart
kart_state_t global_kartState = STATE_SHUTDOWN;
uint64_t global_selftestStart = 0;
int kart_selftestSubstate = ST_START;


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
        if (bit != kartInputs[listIndex].prevState) {
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

void kart_setHorn(uint8_t state) {
  digitalWrite(PIN_HORN, !!state);
}

void kart_startup() {
  // Test buzzer
  tone(PIN_BUZZER, 1500);

  // Test all indicator lights
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, 1);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, 1);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, 1);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, 1);
  kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 1);
  kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 1);
  kart_setOutput(OUTPUT_POS_IND_HORN, 1);
  kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 1);
  kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 1);
  kart_updateOutputs();
  for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
    ws2812.setPixelColor(i, ws2812.Color(255, 0, 0));
  }
  ws2812.show();
  global_kartState = STATE_SELFTEST_DELAY;
  kart_selftestSubstate = ST_START;
  global_selftestStart = millis();
}

void kart_shutdown() {
  kart_setHorn(0);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, 0);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, 0);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, 0);
  kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, 0);
  kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
  kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
  kart_setOutput(OUTPUT_POS_IND_HORN, 0);
  kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
  kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
  kart_updateOutputs();
}

void kart_loop() {
  kart_updateInputs();

  switch (global_kartState) {
    case STATE_SHUTDOWN:
      {
        // Check for ignition on
        if (kart_getInput(INPUT_POS_IGNITION)) {
          kart_startup();
        }
        break;
      }

    case STATE_SELFTEST_DELAY:
      {
        kart_selftest_loop();
        if (kart_selftestSubstate == ST_END) global_kartState = STATE_OPERATIONAL;
        break;
      }

    case STATE_OPERATIONAL:
      {
        // Check horn button
        if (kart_inputChanged(INPUT_POS_HORN)) {
          kart_setHorn(kart_getInput(INPUT_POS_HORN));
        }

        // Check for ignition off
        if (!kart_getInput(INPUT_POS_IGNITION)) {
          kart_shutdown();
          global_kartState = STATE_SHUTDOWN;
        }
        break;
      }
  }

  kart_updateOutputs();
}

void kart_selftest_loop() {
  uint64_t now = millis();
  switch (kart_selftestSubstate) {
    case ST_START:
      {
        if (now - global_selftestStart >= 250) {
          noTone(PIN_BUZZER);
          kart_selftestSubstate = ST_BUZZER_OFF;
        }
        break;
      }

    case ST_BUZZER_OFF:
      {
        if (now - global_selftestStart >= 666 && now - global_selftestStart < 1332) {
          for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
            ws2812.setPixelColor(i, ws2812.Color(0, 255, 0));
          }
          ws2812.show();
          kart_selftestSubstate = ST_WS2812_GREEN;
        }
        break;
      }

    case ST_WS2812_GREEN:
      {
        if (now - global_selftestStart >= 1332 && now - global_selftestStart < 2000) {
          for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
            ws2812.setPixelColor(i, ws2812.Color(0, 0, 255));
          }
          ws2812.show();
          kart_selftestSubstate = ST_WS2812_BLUE;
        }
        break;
      }

    case ST_WS2812_BLUE:
      {
        if (now - global_selftestStart >= 2000) {
          kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ERROR, 0);
          kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ERROR, 0);
          kart_setOutput(OUTPUT_POS_IND_MOTOR_FRONT_ACTIVE, 0);
          kart_setOutput(OUTPUT_POS_IND_MOTOR_REAR_ACTIVE, 0);
          kart_setOutput(OUTPUT_POS_IND_HEADLIGHTS, 0);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_HAZARD, 0);
          kart_setOutput(OUTPUT_POS_IND_HORN, 0);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_LEFT, 0);
          kart_setOutput(OUTPUT_POS_IND_INDICATOR_RIGHT, 0);
          kart_updateOutputs();
          for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
            ws2812.setPixelColor(i, ws2812.Color(0, 0, 0));
          }
          ws2812.show();
          kart_selftestSubstate = ST_END;
        }
        break;
      }
  }
}