/* Example code for getting humidity from a Wia temperature shield */

#include <WiaPeripherals.h>

WiaPeripherals wiaPeripherals;

void setup() {
  Serial.begin(115200);
  wiaPeripherals.init();
}

void loop() {
  Serial.print("Humidity : ");
  Serial.print(wiaPeripherals.getHumidity());
  Serial.print("%");
  Serial.println();

  delay(2000);
}