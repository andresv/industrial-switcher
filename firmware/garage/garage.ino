
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
#include "ntctable.h"

#define G_GREEN_LED 3
#define G_RED_LED 4
// silk screen is wrong on v1.0 board, reverse order here
// 2.3 and 2.4 are not mapped in Energia, they are handled as raw GPIO
#define OUTPUT_3 12
#define OUTPUT_4 11

#define NTC1_PIN A1 
#define NTC2_PIN A0

// graphics
#define BOX_WIDTH 30
#define BOX_X_OFFSET 0
#define BOX_Y_OFFSET 28

#define CUR_TEMP1_X 55
#define CUR_TEMP1_Y 26
#define SET_TEMP1_X 135
#define SET_TEMP1_Y 26

#define CUR_TEMP2_X 55
#define CUR_TEMP2_Y 100
#define SET_TEMP2_X 135
#define SET_TEMP2_Y 100

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

#define UPPER_HYSTERESIS 5
#define LOWER_HYSTERESIS 5
#define CONTROL_OFF -127
#define HIGH_TEMP 60
#define HIGH_TEMP_TIME 60*60 // seconds

extern uint8_t SmallFont[];
extern uint8_t SevenSegNumFont[];

UTFT TFT(HX8340B_8, RS_PIN, WR_PIN, CS_PIN, RST_PIN);
UTouch Touch(T_CLK_PIN, T_TCS_PIN, T_DIN_PIN, T_OUT_PIN, T_IRQ_PIN);

int8_t current_temp_1;
uint8_t temp_1_index = 0;
int8_t temp_1_choices[] = {CONTROL_OFF, HIGH_TEMP, 25};
int8_t current_temp_2;
uint8_t temp_2_index = 0;
int8_t temp_2_choices[] = {CONTROL_OFF, 25};

uint32_t heater_start_timestamp = 0;
uint32_t seconds_count = 0;
bool hysteresis_1_raising = true;
bool hysteresis_2_raising = false;

void draw_main_screen();
void draw_set_temp_1();
void draw_set_temp_2();
void clear_number(int x1, int y1);
void switch_out_1(bool onoff);
void switch_out_2(bool onoff);
void switch_out_3(bool onoff);
void switch_out_4(bool onoff);

void setup() {                
    pinMode(G_GREEN_LED, OUTPUT);
    pinMode(G_RED_LED, OUTPUT);
    P2DIR |= (1<<4); // OUTPUT_1
    P2DIR |= (1<<3); // OUTPUT_2
    pinMode(OUTPUT_3, OUTPUT);
    pinMode(OUTPUT_4, OUTPUT);
    pinMode(NTC1_PIN, INPUT);
    pinMode(NTC2_PIN, INPUT);

    TFT.InitLCD();
    Touch.InitTouch(LANDSCAPE);
    Touch.setPrecision(PREC_MEDIUM);
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
    TFT.printNumI(25, CUR_TEMP1_X, CUR_TEMP1_Y);
    draw_set_temp_1();

    // garage
    TFT.setFont(SmallFont);
    TFT.setColor(VGA_WHITE);
    TFT.print("garage", 65, 150);
    TFT.setFont(SevenSegNumFont);
    TFT.printNumI(10, CUR_TEMP2_X, CUR_TEMP2_Y);
    draw_set_temp_2();
}

void draw_set_temp_1() {
    if (temp_1_index >= sizeof(temp_1_choices)/sizeof(int8_t)) {
        temp_1_index = 0;
    }
    if (temp_1_choices[temp_1_index] == CONTROL_OFF) {
        clear_number(SET_TEMP1_X, SET_TEMP1_Y);
    }
    else {
        // HIGH_TEMP degrees should be active only for 1 h
        if (temp_1_choices[temp_1_index] == HIGH_TEMP) {
            heater_start_timestamp = millis();
        }
        else {
            heater_start_timestamp = 0;
            TFT.setColor(VGA_WHITE);
            TFT.setFont(SmallFont);
            TFT.print("      ", 149, 10);
        }

        TFT.setColor(VGA_RED);
        TFT.setFont(SevenSegNumFont);
        TFT.printNumI(temp_1_choices[temp_1_index], SET_TEMP1_X, SET_TEMP1_Y);
    }
}

void draw_set_temp_2() {
    if (temp_2_index >= sizeof(temp_2_choices)/sizeof(int8_t)) {
        temp_2_index = 0;
    }
    if (temp_2_choices[temp_2_index] == CONTROL_OFF) {
        clear_number(SET_TEMP2_X, SET_TEMP2_Y);
    }
    else {
        TFT.setColor(VGA_RED);
        TFT.setFont(SevenSegNumFont);
        TFT.printNumI(temp_2_choices[temp_2_index], SET_TEMP2_X, SET_TEMP2_Y);
    }
}

void clear_number(int x1, int y1) {
    TFT.setColor(VGA_BLACK);
    TFT.fillRect(x1, y1, x1 + 60, y1 + 60);
}

void switch_out_4(bool onoff) {
    if (onoff) {
        digitalWrite(OUTPUT_4, HIGH);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1, BOX_WIDTH-1, BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        digitalWrite(OUTPUT_4, LOW);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1, BOX_WIDTH-1, BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

void switch_out_3(bool onoff) {
    if (onoff) {
        digitalWrite(OUTPUT_3, HIGH);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1*BOX_WIDTH+1, BOX_WIDTH-1, 2*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        digitalWrite(OUTPUT_3, LOW);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+1*BOX_WIDTH+1, BOX_WIDTH-1, 2*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

void switch_out_2(bool onoff) {
    if (onoff) {
        P2OUT |= (1<<3);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+2*BOX_WIDTH+1, BOX_WIDTH-1, 3*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        P2OUT &= ~(1<<3);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+2*BOX_WIDTH+1, BOX_WIDTH-1, 3*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

void switch_out_1(bool onoff) {
    if (onoff) {
        P2OUT |= (1<<4);
        TFT.setColor(VGA_GREEN);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+3*BOX_WIDTH+1, BOX_WIDTH-1, 4*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
    else {
        P2OUT &= ~(1<<4);
        TFT.setColor(VGA_BLACK);
        TFT.fillRect(BOX_X_OFFSET+1, BOX_Y_OFFSET+3*BOX_WIDTH+1, BOX_WIDTH-1, 4*BOX_WIDTH + BOX_Y_OFFSET-1);
    }
}

int convert_raw_to_celsius(int rawtemp) {
    int current_celsius = 0;
    int i = 0;

    for (i=1; i < NUMTEMPS; i++) {
        if (temptable[i][0] > rawtemp) {
            int realtemp = temptable[i-1][1] + (rawtemp - temptable[i-1][0]) * (temptable[i][1] - temptable[i-1][1]) / (temptable[i][0] - temptable[i-1][0]);
            if (realtemp > 255) {
                realtemp = 255;
            }

            current_celsius = realtemp;
            break;
        }
    }

    // Overflow: We just clamp to 0 degrees celsius
    if (i == NUMTEMPS) {
        current_celsius = 1000;
    }

    return current_celsius;
}

int read_temp_1() {
    uint16_t rawtemp = analogRead(NTC1_PIN);
    return convert_raw_to_celsius(rawtemp);
}

int read_temp_2() {
    uint16_t rawtemp = analogRead(NTC2_PIN);
    return convert_raw_to_celsius(rawtemp);
}

void read_temp_sensors() {
    current_temp_1 = read_temp_1();
    current_temp_2 = read_temp_2();

    if (current_temp_1 > 99) {
        current_temp_1 = 99;
    }
    else if (current_temp_1 < 0) {
        current_temp_1 = 0;
    }
    if (current_temp_2 > 99) {
        current_temp_2 = 99;
    }
    else if (current_temp_2 < 0) {
        current_temp_2 = 0;
    }

    TFT.setFont(SevenSegNumFont);
    TFT.setColor(VGA_WHITE);
    TFT.printNumI(current_temp_1, CUR_TEMP1_X, CUR_TEMP1_Y);
    TFT.printNumI(current_temp_2, CUR_TEMP2_X, CUR_TEMP2_Y);
}

uint32_t tx=0;
uint32_t ty=0;
void loop() {
    if (Touch.dataAvailable()) {
        digitalWrite(G_GREEN_LED, HIGH);
        delay(100);
        digitalWrite(G_GREEN_LED, LOW);

        Touch.read();
        tx = Touch.getX();
        ty = Touch.getY();

        // sometimes x is reported as 219
        // and y is reported as 0, actually they are not
        // filter those touches out
        if (tx != 219 && ty != 0) {
            // check if lower right sector was touched
            if (ty > 90) {
                temp_2_index++;
                draw_set_temp_2();
            }
            // check if upper right sector was touched
            else if (ty < 80) {
                temp_1_index++;
                draw_set_temp_1();
            }
        }
    }

    read_temp_sensors();

    // HIGH_TEMP degrees should be active only for 1 h
    if (heater_start_timestamp > 0) {
        seconds_count = (millis() - heater_start_timestamp) / 1000;

        if (seconds_count < HIGH_TEMP_TIME) {
            TFT.setColor(VGA_WHITE);
            TFT.setFont(SmallFont);
            TFT.printNumI((HIGH_TEMP_TIME - seconds_count) / 60, 149, 12, 2, '0');
            TFT.print(":", 164, 11);
            TFT.printNumI((HIGH_TEMP_TIME - seconds_count) % 60, 172, 12, 2, '0');
        }
        else {
            TFT.setColor(VGA_WHITE);
            TFT.setFont(SmallFont);
            TFT.print("      ", 149, 10);
            heater_start_timestamp = 0;
            temp_1_index++;
            draw_set_temp_1();
        }
    }

    //---------------------------------------------------------------
    // 1 ON-OFF regulator
    //---------------------------------------------------------------
    // upper point
    if (hysteresis_1_raising && (current_temp_1 >= (temp_1_choices[temp_1_index] + UPPER_HYSTERESIS))) {
        hysteresis_1_raising = false;
    }
    // lower point
    else if (!hysteresis_1_raising && (current_temp_1 <= (temp_1_choices[temp_1_index] - LOWER_HYSTERESIS))) {
        hysteresis_1_raising = true;
    }
    // raising
    if (hysteresis_1_raising) {
        switch_out_4(HIGH);
    }
    // falling
    else {
        switch_out_4(LOW);
    }

    //---------------------------------------------------------------
    // 2 ON-OFF regulator
    //---------------------------------------------------------------
    // upper point
    if (hysteresis_2_raising && (current_temp_2 >= (temp_2_choices[temp_2_index] + UPPER_HYSTERESIS))) {
        hysteresis_2_raising = false;
    }
    // lower point
    else if (!hysteresis_2_raising && (current_temp_2 <= (temp_2_choices[temp_2_index] - LOWER_HYSTERESIS))) {
        hysteresis_2_raising = true;
    }
    // raising
    if (hysteresis_2_raising) {
        switch_out_1(HIGH);
    }
    // falling
    else {
        switch_out_1(LOW);
    }
}
