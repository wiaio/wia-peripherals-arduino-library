/* Example code for getting temperature, humidity from a Wia temperature shield */

#include <WiaPeripherals.h>

#define DHTPIN 22     
#define DHTTYPE DHT22

WiaPeripherals wiaPeripherals(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  wiaPeripherals.begin();
}

void loop() {
  // Wait a few seconds between measurements.
  delay(2000);
  Serial.print("Humidity: ");
  Serial.println(wiaPeripherals.getHumidity());
  Serial.print("Temperature Celsius: ");
  Serial.println(wiaPeripherals.getTemperatureCelsius());
  Serial.print("Temperature Fahrenheit: ");
  Serial.println(wiaPeripherals.convertCtoF(wiaPeripherals.getTemperatureCelsius()));
}
