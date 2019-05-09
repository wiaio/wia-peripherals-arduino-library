/* Example code for getting temperature from a Wia temperature shield */

#include <WiaPeripherals.h>

WiaPeripherals wiaPeripherals();

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Humidity : ");
  Serial.println(wiaPeripherals.getHumidity());
  Serial.println();

  delay(2000);
}
