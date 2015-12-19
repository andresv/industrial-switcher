#ifndef GARAGE_H
#define GARAGE_H

// typedefs must be in .h file if they are used in functions
// http://ramblings.sagar.org/2012/01/why-doesnt-this-work-on-arduino-typedef.html
typedef struct {
    uint8_t index;
    uint8_t len;
    short* buffer;
} container_t;

#endif