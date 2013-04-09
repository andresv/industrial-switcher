
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
#include <UTouch.h>

#define G_GREEN_LED 3
#define G_RED_LED 4
#define OUTPUT_1 11
#define OUTPUT_2 12

#define BOX_WIDTH 30
#define BOX_X_OFFSET 0
#define BOX_Y_OFFSET 28

// TFT pins
#define RS_PIN 28
#define WR_PIN 27
#define CS_PIN 14
#define RST_PIN 13

// Touch pins
#define T_CLK_PIN 10
#define T_TCS_PIN 9
#define T_DIN_PIN 8
#define T_OUT_PIN 7
#define T_IRQ_PIN 6

extern uint8_t SmallFont[];
extern uint8_t SevenSegNumFont[];

UTFT TFT(HX8340B_8, RS_PIN, WR_PIN, CS_PIN, RST_PIN);
UTouch Touch(T_CLK_PIN, T_TCS_PIN, T_DIN_PIN, T_OUT_PIN, T_IRQ_PIN);

void draw_main_screen();
void switch_out_1(bool onoff);
void switch_out_2(bool onoff);
void switch_out_3(bool onoff);
void switch_out_4(bool onoff);

void setup() {                
    pinMode(G_GREEN_LED, OUTPUT);
    pinMode(G_RED_LED, OUTPUT);
    pinMode(OUTPUT_1, OUTPUT);
    pinMode(OUTPUT_2, OUTPUT);

    TFT.InitLCD();
    Touch.InitTouch(LANDSCAPE);
    Touch.setPrecision(PREC_LOW);
    draw_main_screen();
}

void draw_main_screen() {
    TFT.clrScr();

    // draw output indicators
    TFT.setColor(VGA_WHITE);
    TFT.drawRect(BOX_X_OFFSET, BOX_Y_OFFSET, BOX_WIDTH, BOX_WIDTH + BOX_Y_OFFSET);
    TFT.drawRect(BOX_X_OFFSET, BOX_Y_OFFSET+1*BOX_WIDTH, BOX_WIDTH, 2*BOX_WIDTH + BOX_Y_OFFSET);
    TFT.drawRect(BOX_X_OFFSET, BOX_Y_OFFSET+2*BOX_WIDTH, BOX_WIDTH, 3*BOX_WIDTH + BOX_Y_OFFSET);
    TFT.drawRect(BOX_X_OFFSET, BOX_Y_OFFSET+3*BOX_WIDTH, BOX_WIDTH, 4*BOX_WIDTH + BOX_Y_OFFSET);

    
    // color chamber
    TFT.setFont(SmallFont);
    TFT.print("painting", 55, 10);
    TFT.setFont(SevenSegNumFont);
    TFT.printNumI(25, 55, 26);
    TFT.setColor(VGA_RED);
    TFT.printNumI(60, 135, 26);

    // garage
    TFT.setFont(SmallFont);
    TFT.setColor(VGA_WHITE);
    TFT.print("garage", 65, 150);
    TFT.setFont(SevenSegNumFont);
    TFT.printNumI(10, 55, 100);
    TFT.setColor(VGA_RED);
    TFT.printNumI(25, 135, 100);

    switch_out_1(HIGH);
    //switch_out_2(HIGH);
    switch_out_3(HIGH);
    //switch_out_4(HIGH);
}

void switch_out_1(bool onoff) {
    if (onoff) {
        digitalWrite(OUTPUT_1, HIGH);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1, BOX_WIDTH-1, BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        digitalWrite(OUTPUT_1, LOW);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1, BOX_WIDTH-1, BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

void switch_out_2(bool onoff) {
    if (onoff) {
        digitalWrite(OUTPUT_2, HIGH);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1*BOX_WIDTH+1, BOX_WIDTH-1, 2*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        digitalWrite(OUTPUT_2, LOW);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1*BOX_WIDTH+1, BOX_WIDTH-1, 2*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

void switch_out_3(bool onoff) {
    if (onoff) {
        //digitalWrite(OUTPUT_2, HIGH);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+2*BOX_WIDTH+1, BOX_WIDTH-1, 3*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        //digitalWrite(OUTPUT_2, LOW);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+2*BOX_WIDTH+1, BOX_WIDTH-1, 3*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

void switch_out_4(bool onoff) {
    if (onoff) {
        //digitalWrite(OUTPUT_2, HIGH);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+3*BOX_WIDTH+1, BOX_WIDTH-1, 4*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        //digitalWrite(OUTPUT_2, LOW);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+3*BOX_WIDTH+1, BOX_WIDTH-1, 4*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

uint8_t i = 0;
uint32_t tx=0;
uint32_t ty=0;
void loop() {
    if (Touch.dataAvailable()) {
        digitalWrite(G_GREEN_LED, HIGH);
        delay(50);
        digitalWrite(G_GREEN_LED, LOW);

        Touch.read();
        tx = Touch.getX();
        ty = Touch.getY();

        // sometimes x is reported as 219
        // and y is reported as 0, actually they are not
        // filter those touches out
        if (tx != 219 && ty != 0) {
            // check if lower right sector was touched
            if (tx > 120 && ty > 90) {
                TFT.setColor(VGA_WHITE);
                TFT.setFont(SevenSegNumFont);
                TFT.printNumI(i%99, 55, 100);
                i++;
            }
            // check if upper right sector was touched
            else if (tx > 120 && ty < 80) {
                TFT.setColor(VGA_WHITE);
                TFT.setFont(SevenSegNumFont);
                TFT.printNumI(i%99, 55, 26);
                i++;
            }
        }
    }
}
