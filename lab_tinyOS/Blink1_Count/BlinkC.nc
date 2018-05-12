#include "Timer.h"

module BlinkC @safe()
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Leds;
  uses interface Boot;
}
implementation
{
  uint32_t i = 0;
  event void Boot.booted()
  {
    call Timer0.startPeriodic( 1000 );
  }

  event void Timer0.fired()
  {
    dbg("BlinkC", "Timer 0 fired @ %s.\n", sim_time_string());
    
    printf("%d\n",i);
      
    if((i&1) == 1){ call Leds.led0On();}
    else call Leds.led0Off();
    
    if((i&2) == 2){ call Leds.led1On();}
    else call Leds.led1Off(); 
    
    if((i&4) == 4){ call Leds.led2On();}
    else call Leds.led2Off();
    
    
    
    i++;
    
    if(i == 8) i = 0;
   }
 
}

