#include <LIS2DU12Sensor.h>

LIS2DU12Sensor sensor (&Wire);
uint8_t sensor_id;

void setup()
{
  Serial.begin(115200);
  sensor.ReadID(&sensor_id);
  Serial.printf("Id:%d\r\n", sensor_id);
}

void loop()
{
  delay(1000);//blink(LED_BUILTIN);
}

void blink(uint8_t pin)
{
  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(975);
}
