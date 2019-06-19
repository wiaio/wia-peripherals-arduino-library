/* Example code for getting temperature from a Wia temperature shield */

#include <WiaPeripherals.h>

WiaPeripherals wiaPeripherals;

void setup() {
  Serial.begin(115200);
  wiaPeripherals.init();
}

void loop() {
  Serial.print("Temperature Celsius: ");
  Serial.print(wiaPeripherals.getTemperatureCelsius());
  Serial.print("*C");
  Serial.println();

  Serial.print("Temperature Fahrenheit: ");
  Serial.print(wiaPeripherals.getTemperatureFahrenheit());
  Serial.print("*F");
  Serial.println();
  
  Serial.println();
  delay(2000);
}