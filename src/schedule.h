#ifndef SCHEDULE_H
#define SCHEDULE_H

#include <Arduino.h>

struct SunriseTime {
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
};

extern const SunriseTime sunriseTable[];
extern const int tableSize;

SunriseTime getSunrise(uint8_t month, uint8_t day, int* dayNumber = nullptr);

#endif // SCHEDULE_H