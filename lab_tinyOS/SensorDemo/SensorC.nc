#include "Timer.h"
#include "Sensor.h" 
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

  uses interface Packet;
  uses interface AMPacket;
  uses interface AMSend;
  uses interface SplitControl as Control;
}

implementation
{
  #define FREQUENCY 100
  
  uint16_t tempData;
  uint16_t humidityData;
  uint16_t photoData;
  
  message_t packet;
  bool locked =FALSE;

  event void Boot.booted()
  {
    // call Timer0.startPeriodic(FREQUENCY);
    call Control.start();
  }

  event void Control.startDone(error_t err) 
	{
    	if (err == SUCCESS) 
		    call Timer0.startPeriodic(1000);
  }

  event void Control.stopDone(error_t err)
  {
    //do sth
  }
  
  event void Timer0.fired()
  {
     if(locked){
       return;
     }
    call readTemp.read();
    call readHumidity.read();
    call readPhoto.read();
  }
  
  event void readTemp.readDone(error_t result, uint16_t val){
    if(result == SUCCESS){
      SensorMsg *payload = (SensorMsg*) call Packet.getPayload(&packet, sizeof(SensorMsg));
      if(!payload)return;

      payload->nodeid = TOS_NODE_ID;
      payload->kind = TEMPORARY;//0
      payload->data = val;

      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(SensorMsg)) == SUCCESS)
      {
        call Leds.led0Toggle();
        locked = TRUE;
      }
    }
  }
  
  event void readHumidity.readDone(error_t result, uint16_t val){
    if(result == SUCCESS){
      SensorMsg *payload = (SensorMsg*) call Packet.getPayload(&packet, sizeof(SensorMsg));
      if(!payload)return;

      payload->nodeid = TOS_NODE_ID;
      payload->kind = HUMIDITY;//1
      payload->data = val;

      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(SensorMsg)) == SUCCESS)
      {
        call Leds.led1Toggle();
         locked = TRUE;
      }
    }
  }
  
  event void readPhoto.readDone(error_t result, uint16_t val){
    if(result == SUCCESS){
      SensorMsg *payload = (SensorMsg*) call Packet.getPayload(&packet, sizeof(SensorMsg));
      if(!payload)return;

      payload->nodeid = TOS_NODE_ID;
      payload->kind = 2;//2
      payload->data = val;

      if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(SensorMsg)) == SUCCESS)
      {
        call Leds.led2Toggle();
         locked = TRUE;
      }
    }
  }

  event void AMSend.sendDone(message_t* msg, error_t err) 
  {
     if (&packet == msg) 
     {
       locked = FALSE;
     }
  }
  
}

