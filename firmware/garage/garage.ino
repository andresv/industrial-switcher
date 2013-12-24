
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
#include "garage.h"

#define G_GREEN_LED 3
#define G_RED_LED 4
// silk screen is wrong on v1.0 board, reverse order here
// 2.3 and 2.4 are not mapped in Energia, they are handled as raw GPIO
#define OUTPUT_3 12
#define OUTPUT_4 11

// temperature
#define NTC1_PIN A1
#define NTC2_PIN A0
#define SMOOTHING_BUFFER_SIZE 5
// It shows how many quick measurements are taken in the row for filtering
#define FILTER_BUFFER_SIZE 5
// It shows how many elements are discarded from the beginning
// and at the end of buffer
#define FILTER_OUT_COUNT 1
// for example we have following sorted measurements: 34, 56, 56, 58, 60
// 34 and 60 will be discarded if FILTER_OUT_COUNT is 1

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

// ON/OFF regulator
#define UPPER_HYSTERESIS 5
#define LOWER_HYSTERESIS 5
#define CONTROL_OFF -127
#define HIGH_TEMP 60
#define HIGH_TEMP_TIME 60*60 // seconds

#define SAVED_STATE_FRAM_ADDRESS 0xFF1C
#define FRAM_MAGIC 0xCAFEBABE

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

uint16_t filter_buf[FILTER_BUFFER_SIZE];
container_t filter_container;

short temp_1_buf[SMOOTHING_BUFFER_SIZE];
container_t temp_1_container;
short temp_2_buf[SMOOTHING_BUFFER_SIZE];
container_t temp_2_container;

uint32_t heater_last_timestamp = 0;
bool heater_is_on = false;
uint32_t milli_count = 0;
bool hysteresis_1_raising = true;
bool hysteresis_2_raising = false;

unsigned long *fram_params_ptr;
typedef struct {
    uint32_t magic;
    uint8_t temp_1_index;
    uint8_t temp_2_index;
    uint32_t milli_count;
}fram_params_t;

fram_params_t framparams;

void draw_main_screen();
void draw_set_temp_1();
void draw_set_temp_2();
void clear_number(int x1, int y1);
void switch_out_1(bool onoff);
void switch_out_2(bool onoff);
void switch_out_3(bool onoff);
void switch_out_4(bool onoff);
void save_parameters_to_fram();
void read_parameters_from_fram();
int convert_raw_to_celsius(int rawtemp);

void init_containers() {
    short temp;
    uint8_t i;

    filter_container.index = 0;
    filter_container.len = FILTER_BUFFER_SIZE;
    filter_container.buffer = (short*)filter_buf;

    temp_1_container.index = 0;
    temp_1_container.len = SMOOTHING_BUFFER_SIZE;
    temp_1_container.buffer = temp_1_buf;

    temp_2_container.index = 0;
    temp_2_container.len = SMOOTHING_BUFFER_SIZE;
    temp_2_container.buffer = temp_2_buf;

    // fill in averaging buffers with initial temperature values
    temp = convert_raw_to_celsius(analogRead(NTC2_PIN)); // temp_1 is NTC2
    for (i=0; i<SMOOTHING_BUFFER_SIZE; i++) {
        temp_1_container.buffer[i] = temp;
    }

    temp = convert_raw_to_celsius(analogRead(NTC1_PIN)); // temp_2 is NTC1
    for (i=0; i<SMOOTHING_BUFFER_SIZE; i++) {
        temp_2_container.buffer[i] = temp;
    }
}

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

    read_parameters_from_fram();

    init_containers();

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
    TFT.print("in", 80, 10);
    draw_set_temp_1();

    // garage
    TFT.setFont(SmallFont);
    TFT.setColor(VGA_WHITE);
    TFT.print("out", 75, 150);
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
            heater_last_timestamp = millis();
            heater_is_on = true;
        }
        else {
            heater_is_on = false;
            milli_count = 0;
            TFT.setColor(VGA_WHITE);
            TFT.setFont(SmallFont);
            TFT.print("      ", 149, 10);
        }

        TFT.setColor(VGA_RED);
        TFT.setFont(SevenSegNumFont);
        TFT.printNumI(temp_1_choices[temp_1_index], SET_TEMP1_X, SET_TEMP1_Y, 2, '0');
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
        TFT.printNumI(temp_2_choices[temp_2_index], SET_TEMP2_X, SET_TEMP2_Y, 2, '0');
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

    // Overflow: Clamp to -50 degrees celsius
    if (i == NUMTEMPS) {
        current_celsius = -50;
    }

    return current_celsius;
}

void container_append(container_t* container, short element) {
    container->buffer[container->index] = element;
    container->index++;
    if (container->index == container->len) {
        container->index = 0;
    }
}

short container_average(container_t* container) {
    short sum = 0;
    uint8_t i;

    for (i = 0; i < container->len; i++) {
        sum += container->buffer[i];
    }
    return (sum / container->len);
}

void container_sort(container_t* container) {
    // bubble sort
    uint8_t i, j;
    short temp;

    for (i=0; i<container->len ; i++) {
       for (j=0; j<(container->len-1)-i; j++) {
         if (container->buffer[j] < container->buffer[j+1] ) {
            temp = container->buffer[j];
            container->buffer[j] = container->buffer[j+1];
            container->buffer[j+1] = temp;
         }
       }
    }
}

int read_temp(int adc_nr) {
    uint8_t i = 0;
    uint16_t sum = 0;

    // take couple of readings as fast as possible
    for (i=0; i<FILTER_BUFFER_SIZE; i++) {
        container_append(&filter_container, analogRead(adc_nr));
    }

    // filter out smallest and biggest readings (might be outliers)
    container_sort(&filter_container);
    for (i=FILTER_OUT_COUNT; i<FILTER_BUFFER_SIZE - FILTER_OUT_COUNT; i++) {
        sum += filter_container.buffer[i];
    }

    return convert_raw_to_celsius(sum/(FILTER_BUFFER_SIZE - 2*FILTER_OUT_COUNT));
}

void read_temp_sensors() {
    container_append(&temp_1_container, read_temp(NTC2_PIN));
    current_temp_1 = container_average(&temp_1_container);

    container_append(&temp_2_container, read_temp(NTC1_PIN));
    current_temp_2 = container_average(&temp_2_container);

    // only 2 digits can be shown and we do not support negative numbers
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
    TFT.printNumI(current_temp_1, CUR_TEMP1_X, CUR_TEMP1_Y, 2, '0');
    TFT.printNumI(current_temp_2, CUR_TEMP2_X, CUR_TEMP2_Y, 2, '0');
}

void save_parameters_to_fram() {
    framparams.magic = FRAM_MAGIC;
    framparams.temp_1_index = temp_1_index;
    framparams.temp_2_index = temp_2_index;
    framparams.milli_count = milli_count;
    fram_params_ptr = (unsigned long *)SAVED_STATE_FRAM_ADDRESS;
    memcpy((void*)fram_params_ptr, &framparams, sizeof(fram_params_t));
}

void read_parameters_from_fram() {
    fram_params_t* parameters;
    fram_params_ptr = (unsigned long *)SAVED_STATE_FRAM_ADDRESS;
    parameters = (fram_params_t*)fram_params_ptr;
    if (parameters->magic == FRAM_MAGIC) {
        temp_1_index = parameters->temp_1_index;
        temp_2_index = parameters->temp_2_index;
        milli_count = parameters->milli_count;
    }
}

uint32_t tx=0;
uint32_t ty=0;
bool led_on = true;
void loop() {
    // status blink
    if (led_on) {
        digitalWrite(G_GREEN_LED, HIGH);
        led_on = false;
    }
    else {
        digitalWrite(G_GREEN_LED, LOW);
        led_on = true;
    }

    if (Touch.dataAvailable()) {
        delay(100);

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
    if (heater_is_on) {
        uint32_t timestamp = millis();
        milli_count += (timestamp - heater_last_timestamp);
        heater_last_timestamp = timestamp;

        if (milli_count/1000 < HIGH_TEMP_TIME) {
            TFT.setColor(VGA_WHITE);
            TFT.setFont(SmallFont);
            TFT.printNumI((HIGH_TEMP_TIME - milli_count/1000) / 60, 149, 12, 2, '0');
            TFT.print(":", 164, 11);
            TFT.printNumI((HIGH_TEMP_TIME - milli_count/1000) % 60, 172, 12, 2, '0');
        }
        else {
            TFT.setColor(VGA_WHITE);
            TFT.setFont(SmallFont);
            TFT.print("      ", 149, 10);
            heater_is_on = false;
            milli_count = 0;
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

    save_parameters_to_fram();
}
