#include <Wire.h>
#include <RTClib.h>
#include <avr/sleep.h>
#include "schedule.h"

RTC_DS3231 rtc;

const int interruptPin = 2;  // Підключений до SQW/INT DS3231
const int solenoidPin = 9;

// Розклад пробудження залежно від дня тижня (0 - неділя, 1 - понеділок ... 6 - субота)
const bool sleepMode = false;

bool alarmTriggered = false;

void wakeUp() {
    alarmTriggered = true;
}

DateTime getNextDay(DateTime day) {
    return day + TimeSpan(1, 0, 0, 0);
}

const int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};

int getDayOfYear(DateTime dt) {
  int dayOfYear = dt.day();
  for (int i = 0; i < dt.month() - 1; i++) {
    dayOfYear += daysInMonth[i];
  }
  // Перевірка на високосний рік
  if (dt.month() > 2 && dt.year() % 4 == 0 && (dt.year() % 100 != 0 || dt.year() % 400 == 0)) {
    dayOfYear += 1;
  }
  return dayOfYear;
}


void setNextAlarm() {
  DateTime now = rtc.now();
  DateTime nextDay = getNextDay(now);
  DateTime nextSchedule = now + TimeSpan(10);
  SunriseTime time = sunriseTable[getDayOfYear(nextDay)];

  DateTime alarmTime = DateTime(nextDay.year(), nextDay.month(), nextDay.day(), time.hour, time.minute, 0);

  uint16_t year   = nextDay.year();
  uint8_t  month  = nextDay.month();
  uint8_t  day    = nextDay.day();
  uint8_t  hour   = time.hour;
  uint8_t  minute = time.minute;
  uint8_t  second = 0;

  Serial.print("Setting alarm for: ");
  Serial.print(year); Serial.print("-");
  Serial.print(month); Serial.print("-");
  Serial.print(day); Serial.print(" ");
  Serial.print(hour); Serial.print(":");
  Serial.print(minute); Serial.print(":");
  Serial.println(second);
  rtc.clearAlarm(1);
  rtc.setAlarm1(
      nextSchedule,
      DS3231_A1_Hour // Будильник спрацьовує в заданий час наступного дня
  );

  Serial.print("Наступне пробудження: ");
  Serial.print(alarmTime.hour());
  Serial.print(":");
  Serial.print(alarmTime.minute());
  Serial.println(" год.");
}

void enterSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void activateSolenoid() {
    digitalWrite(solenoidPin, HIGH);
    digitalWrite(13, HIGH);
    delay(3000);
    digitalWrite(solenoidPin, LOW);
    digitalWrite(13, LOW);
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    rtc.begin();

    
    // Налаштовуємо переривання
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp, FALLING);

    pinMode(solenoidPin, OUTPUT);

    Serial.println("Система запущена. Входимо в режим сну.");
    setNextAlarm();
    if (sleepMode) {
        Serial.println(">>> Sleep mode");
        // входимо в сон одразу після налаштування
        enterSleep();
    } else {
        Serial.println(">>> Run mode");
        // у Run-режимі не спимо, просто чекаємо переривання у loop()
    }
}

void loop() {
    if (!alarmTriggered) return;

    // обробляємо будильник
    alarmTriggered = false;
    Serial.println("Alarm! Firing solenoid...");
    activateSolenoid();
    setNextAlarm();

    // для Sleep-режиму – знову спимо
    if (sleepMode) {
        enterSleep();
    }   
}
