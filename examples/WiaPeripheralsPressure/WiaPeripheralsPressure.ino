/* Example code for getting pressure from a Wia temperature shield */

#include <WiaPeripherals.h>

WiaPeripherals wiaPeripherals;

void setup() {
  Serial.begin(115200);
  wiaPeripherals.init();
}

void loop() {
  Serial.print("Pressure : ");
  Serial.print(wiaPeripherals.getPressure());
  Serial.print("PA");
  Serial.println();

  Serial.println();
  delay(2000);
}