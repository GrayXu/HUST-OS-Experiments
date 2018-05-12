#include "Timer.h"

module BlinkC @safe()
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Timer<TMilli> as Timer1;
  uses interface Timer<TMilli> as Timer2;
  uses interface Leds;
  uses interface Boot;
}

implementation
{
  uint32_t start;
  
  task void computeSmallTask(){
    uint32_t temp = 0;
    //int variable start would be updated in another func
    for(temp = start;temp<start+1000;temp++){
      //do computing
    }
  }
  
  task void computeTask(){
    uint32_t max = 400001;
    uint32_t iteration = max/1000;
    uint32_t i = 0;
    
    for(i=0;i<iteration;i++){
      start = i*1000;
      post computeSmallTask();
    }
    
    start = (i+1)*100;
    
    for(i = iteration*1000;i<max;i++){
      //do computing
    }

  }
  
  event void Boot.booted()
  {
    call Timer0.startPeriodic( 250 );
    call Timer1.startPeriodic( 500 );
    call Timer2.startPeriodic( 1000 );
  }

  event void Timer0.fired()
  {
    dbg("BlinkC", "Timer 0 fired @ %s.\n", sim_time_string());
    printf("here is a LED uint8: 0\n"); 
    printfflush();
    post computeTask();//without this line this program would be normal Blink
    call Leds.led0Toggle();
  }
  
  event void Timer1.fired()
  {
    dbg("BlinkC", "Timer 1 fired @ %s \n", sim_time_string());
    call Leds.led1Toggle();
    printf("here is a LED uint8: 1\n");
    printfflush();
  }
  
  event void Timer2.fired()
  {
    dbg("BlinkC", "Timer 2 fired @ %s.\n", sim_time_string());
    call Leds.led2Toggle();
    printf("here is a LED uint8: 2\n");
    printfflush();
  }
}

