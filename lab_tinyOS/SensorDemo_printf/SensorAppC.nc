#include "printf.h"

configuration SensorAppC
{
}

implementation
{
  components MainC, SensorC, LedsC;
  components new TimerMilliC() as Timer0;
  components PrintfC;
  components SerialStartC;

  components new DemoSensorC() as Sensor;

  components new SensirionSht11C();
  components new HamamatsuS1087ParC();
  

  SensorC -> MainC.Boot;
  
  SensorC.readTemp -> SensirionSht11C.Temperature;
  SensorC.readHumidity -> SensirionSht11C.Humidity;
  SensorC.readPhoto -> HamamatsuS1087ParC;
  
  SensorC.Timer0 -> Timer0;
  SensorC.Leds -> LedsC;
}

