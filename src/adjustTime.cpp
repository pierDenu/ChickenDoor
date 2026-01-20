#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();

  if (rtc.lostPower()) {
    Serial.println("RTC втратила живлення, встановлюємо час...");

    // Встановлюємо час: рік, місяць, день, година, хвилина, секунда
    rtc.adjust(DateTime(2025, 6, 24, 15, 06, 0));  // <-- зміни на актуальний час
  }

  Serial.println("Час встановлено.");
}

void loop() {
  DateTime now = rtc.now();
  Serial.print("Поточний час: ");
  Serial.print(now.year()); Serial.print("-");
  Serial.print(now.month()); Serial.print("-");
  Serial.print(now.day()); Serial.print(" ");
  Serial.print(now.hour()); Serial.print(":");
  Serial.print(now.minute()); Serial.print(":");
  Serial.println(now.second());
  delay(1000);
}
