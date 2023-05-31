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


// State of the LED on pin 13 - just a kind of heartbeat indicator
uint8_t global_ledState = 0;


void setup() {
  Serial.begin(115200);

  kart_init();

  /* Blinker
  while (1) {
    tone(PIN_BUZZER, 1500);
    delay(5);
    noTone(PIN_BUZZER);
    delay(350);
    tone(PIN_BUZZER, 1200);
    delay(5);
    noTone(PIN_BUZZER);
    delay(350);
  }
  */
}

void loop() {
  global_ledState = !global_ledState;
  //digitalWrite(PIN_LED, kart_getInput(INPUT_POS_IGNITION));
  kart_loop();
}
