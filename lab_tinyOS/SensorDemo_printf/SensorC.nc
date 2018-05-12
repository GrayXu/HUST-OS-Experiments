#include "Timer.h"
#include "SensirionSht11.h"

/*read and print raw data without any process*/

module SensorC @safe()
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Leds;
  uses interface Boot;
  uses interface Read<uint16_t> as readTemp;
  uses interface Read<uint16_t> as readHumidity;
  uses interface Read<uint16_t> as readPhoto;
}

implementation
{
  #define FREQUENCY 1000
  
  uint16_t tempData;
  uint16_t humidityData;
  uint16_t photoData;
  bool locked =FALSE;

  event void Boot.booted()
  {
    call Timer0.startPeriodic(FREQUENCY);
  }

  
  
  event void Timer0.fired()
  {
    call readTemp.read();
    call readHumidity.read();
    call readPhoto.read();
  }
  
  event void readTemp.readDone(error_t result, uint16_t val){
    if(result == SUCCESS){
    double temp = 10*(-40.1+0.01*val);
    	uint32_t a = temp/10;
    	uint32_t b = temp-temp/10*10;
      printf("temp: %d.%d\n",a,b);
    	printfflush();
        call Leds.led0Toggle();
    }
  }
  
  event void readHumidity.readDone(error_t result, uint16_t val){
    if(result == SUCCESS){
    double temp = (-4+0.0405*val+(-2.8/1000000)*val*val)*10;
    	uint32_t a = temp/10;
    	uint32_t b = temp-temp/10*10;
    	
    	printf("humidity: %d.%d%\n",a,b);
    	printfflush();
		call Leds.led1Toggle();
    }
  }
  
  event void readPhoto.readDone(error_t result, uint16_t val){
    if(result == SUCCESS){
    	double temp = 0.0625*1000000*(val*1.5/4096/10000)*1000*10;
    	uint32_t a = temp/10;
    	uint32_t b = temp-temp/10*10;
    	printf("photo: %d.%d lux\n",a,b);
    	printfflush();
      call Leds.led2Toggle();
    }
  }
  
}

