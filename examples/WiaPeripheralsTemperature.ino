/* Example code for getting temperature from a Wia temperature shield */

#include <WiaPeripherals.h>

WiaPeripherals wiaPeripherals();

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print("Temperature in Celsius : ");
  Serial.println(wiaPeripherals.getTemperature());
  Serial.println();

  delay(2000);
}
