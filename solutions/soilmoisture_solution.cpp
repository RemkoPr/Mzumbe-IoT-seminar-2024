#include <Dwenguino.h>


void setup()
{
  initDwenguino();
  pinMode(A6, INPUT);
}


void loop()
{
  Serial.println(analogRead(A6));
  delay(10);
}