#include <Wire.h>
#include <RTClib.h>
#include <avr/sleep.h>
#include "schedule.h"
#include <EEPROM.h>

RTC_DS3231 rtc;

const int interruptPin = 2;  
const int solenoidPin = 9;
const int buttonPin = 3;
DateTime alarmTime;


const bool sleepMode = false;

bool alarmTriggered = false;

const int maxRecords = 250;  
const int eepromStart = 2;   
const int solenoidTime = 5000;

uint16_t readRecordCount() {
    uint16_t count;
    EEPROM.get(0, count);
    if (count > maxRecords) count = 0; 
    return count;
}

void writeRecordCount(uint16_t count) {
    EEPROM.put(0, count);
}

void saveTriggerTime(const DateTime& dt, uint16_t index) {
    uint32_t timestamp = dt.unixtime();
    int addr = eepromStart + index * sizeof(uint32_t);
    EEPROM.put(addr, timestamp);
    Serial.print("Записано у EEPROM за індексом "); Serial.println(index);
}

DateTime readTriggerTime(uint16_t index) {
    uint32_t timestamp;
    int addr = eepromStart + index * sizeof(uint32_t);
    EEPROM.get(addr, timestamp);
    return DateTime(timestamp);
}


void wakeUp() {
    alarmTriggered = true;
}

DateTime getNextDay(DateTime day) {
    return day + TimeSpan(1, 0, 0, 0);
}

void printDateTime(const DateTime& dt) {
    Serial.print("Дата: ");
    if (dt.day() < 10) Serial.print("0");
    Serial.print(dt.day());
    Serial.print(".");
  
    if (dt.month() < 10) Serial.print("0");
    Serial.print(dt.month());
    Serial.print(".");
  
    Serial.print(dt.year());
  
    Serial.print("  Час: ");
    if (dt.hour() < 10) Serial.print("0");
    Serial.print(dt.hour());
    Serial.print(":");
  
    if (dt.minute() < 10) Serial.print("0");
    Serial.print(dt.minute());
    Serial.print(":");
  
    if (dt.second() < 10) Serial.print("0");
    Serial.print(dt.second());
  
    Serial.println();
}  



void listLog() {
    uint16_t count = readRecordCount();
    Serial.print("Збережено записів: "); Serial.println(count);
    for (uint16_t i = 0; i < count; i++) {
        DateTime dt = readTriggerTime(i);
        Serial.print("[" + String(i) + "] ");
        printDateTime(dt);
    }
}

int getSecondsForDay(int dayOfYear) {
    const int jun22 = 173; 
    const int minMinutes = 30;
    const int maxMinutes = 60;
    int daysToJun22 = abs(dayOfYear - jun22);
    if (daysToJun22 > 182) daysToJun22 = 365 - daysToJun22;
    return (minMinutes + daysToJun22*(maxMinutes - minMinutes)/182)*60;
}

void setNextAlarm() {
  int dayNumber;
  DateTime now = rtc.now();
  DateTime nextDay = getNextDay()    // для Sleep-режиму – знову спимоnow);
  SunriseTime time  = getSunrise(nextDay.month(), nextDay.day(), &dayNumber);
  alarmTime = DateTime(nextDay.year(), nextDay.month(), nextDay.day(), time.hour, time.minute) + TimeSpan(getSecondsForDay(dayNumber));
  rtc.clearAlarm(1);
  rtc.setAlarm1(
    alarmTime, DS3231_A1_Hour 
  );

}

void enterSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}

void activateSolenoid() {
    digitalWrite(solenoidPin, HIGH);
    digitalWrite(13, HIGH);
    delay(solenoidTime);
    digitalWrite(solenoidPin, LOW);
    digitalWrite(13, LOW);

    DateTime now = rtc.now();
    uint16_t count = readRecordCount();
    saveTriggerTime(now, count);

    count++;
    if (count >= maxRecords) count = 0;  
    writeRecordCount(count);   
}

DateTime getLastTriggerTime() {
    uint16_t count = readRecordCount();
    if (count == 0 && EEPROM.read(eepromStart) == 0xFF) {
        return DateTime((uint32_t)0);  
    }
    uint16_t lastIndex = (count == 0) ? maxRecords - 1 : count - 1;
    return readTriggerTime(lastIndex);
}



void setup() {
    Serial.begin(9600);
    Wire.begin();
    rtc.begin();
    delay(1000);
  
    rtc.writeSqwPinMode(DS3231_OFF);
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp, FALLING);

    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(3), wakeUp, FALLING);

    pinMode(solenoidPin, OUTPUT);
    digitalWrite(solenoidPin, LOW);

    Serial.println("Система запущена. Входимо в режим сну.");
    setNextAlarm();
    if (sleepMode) {
        Serial.println(">>> Sleep mode");
        enterSleep();
    } else {
        Serial.println(">>> Run mode");
    }
    alarmTriggered = false;
}

void checkSerial(String command) {
    if (command == "GET_ALARM") {
        Serial.print("Встановлений час будильника: ");
        printDateTime(alarmTime);
    }

    else if(command == "LIST_LOG") {
        listLog();
    }

    else if(command == "LAST_LOG") {
        DateTime last = getLastTriggerTime();
        Serial.print("Останній запис: ");
        printDateTime(last);
    }

    else if(command == "TIME_NOW") {
        printDateTime(rtc.now());
    }
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        checkSerial(command);
    }

    if (!alarmTriggered) {
        return;
    }

    alarmTriggered = false;
    Serial.println("Alarm! Firing solenoid...");
    activateSolenoid();
    setNextAlarm();

    if (sleepMode) {
        enterSleep();
    }   
}
