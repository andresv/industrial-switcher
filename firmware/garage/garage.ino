
/*
 * Copyright (c) 2013, Andres Vahter
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Andres Vahter (andres.vahter@gmail.com)
 */
 
#include <UTFT.h>

#define G_GREEN_LED 3
#define G_RED_LED 4

#define RS_PIN 28
#define WR_PIN 27
#define CS_PIN 14
#define RST_PIN 13

// select font
extern uint8_t SmallFont[];
UTFT myGLCD(HX8340B_8, RS_PIN, WR_PIN, CS_PIN, RST_PIN);

void setup() {                
  pinMode(G_GREEN_LED, OUTPUT);
  pinMode(G_RED_LED, OUTPUT);
  myGLCD.InitLCD();
  myGLCD.setFont(SmallFont);
}

bool fill = true;
void loop() {
  if (fill) {
    // Clear the screen and draw the frame
    myGLCD.clrScr();

    myGLCD.setColor(255, 0, 0);
    myGLCD.fillRect(0, 0, 219, 13);
    //myGLCD.print("TFT", CENTER, 1);
    fill = false;
  }

  digitalWrite(G_GREEN_LED, HIGH);
  delay(250);
  digitalWrite(G_GREEN_LED, LOW);
  delay(250);
  
  digitalWrite(G_RED_LED, HIGH);
  delay(250);
  digitalWrite(G_RED_LED, LOW);
  delay(250);
}
