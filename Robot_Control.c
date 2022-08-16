/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
                     
#include "mstimer.h"
#include "simpletools.h"
#include "abdrive.h"
#include "SD_library/go_stop_smooth.h"
#include "ping.h"
#include "fdserial.h"

fdserial *xbee;

#define DEFAULT_VAL 10

int main()                                    // main function
{
  /*
  int turn;
  int start_time;
  int end_time;
  int final_time;
  
  xbee = fdserial_open(9, 8, 0, 9600);
  
  mstime_start();
  //goAndStopSmooth(5,DIR_FWD);
  
  dprint(xbee,"2==start\n");
  
  drive_setRampStep(10);                      // 10 ticks/sec / 20 ms

  drive_ramp(128, 128);                       // Forward 2 RPS

  dprint(xbee,"time measure start\n");
  start_time = mstime_get();
  dprint(xbee,"time = %d\n",  start_time);
  
  // While disatance greater than or equal
  // to 20 cm, wait 5 ms & recheck.
  while(ping_cm(7) >= 20) pause(5);           // Wait until object in range

  drive_ramp(0, 0);                           // Then stop
  
  end_time = mstime_get();
  
  final_time = end_time - start_time;
  
  dprint(xbee,"time = %d\n",  end_time);
  dprint(xbee,"move time = %d\n", final_time);
  dprint(xbee,"program end\n");
  */

  xbee = fdserial_open(9, 8, 0, 9600);

  writeChar(xbee, CLS);
  dprint(xbee, "Click this terminal, \n");
  dprint(xbee, "type goto arguments: \n\n");

  char  c;
  int   digit_1 = DEFAULT_VAL, digit_2 = DEFAULT_VAL, digit_3 = DEFAULT_VAL;
  int   final_val;
  int   i;
  
 
  while(1)
  {
    c = fdserial_rxChar(xbee);
    dprint(xbee, "You typed: %x\n", c);
    if(c != -1)
    {
      if(c == 0xd)
      {
        final_val = digit_3*100 + digit_2*10 + digit_1;
        
        dprint(xbee, "final value: %d\n", final_val);
        /*
        for(i=0; i<36; i++)
        {
          drive_goto(final_val,-final_val);
        }
        */
        drive_goto(final_val,-final_val);
    
        digit_3 = DEFAULT_VAL;
        digit_2 = DEFAULT_VAL;
        digit_1 = DEFAULT_VAL;
      }
      else
      {
        if(digit_3 == DEFAULT_VAL)
        {
            digit_3 = c-0x30;
        }
        else if(digit_2 == DEFAULT_VAL)
        {
            digit_2 = c-0x30;
        }
        else  // digit_1 == 10
        {
          digit_1 = c - 0x30;
        }        
      }
      
    }
  }  
  
}

 


/*
fdserial *xbee;

int main()
{
  xbee = fdserial_open(9, 8, 0, 9600);

  writeChar(xbee, CLS);
  dprint(xbee, "Click this terminal, \n");
  dprint(xbee, "and type on keybaord...\n\n");

  char c;
 
  while(1)
  {
    c = fdserial_rxChar(xbee);
    if(c != -1)
    {
      dprint(xbee, "You typed: %c\n", c);
    }
  }  
}
*/
