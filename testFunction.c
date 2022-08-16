/*
  Where I will be testing and creating functions for SD project.
  
  
  Resource: http://learn.parallax.com/propeller-c-tutorials 
  
*/
//Goal: Make a function that allows the robot to ramp up and down while keeping track of the time it took to move.


#include "simpletools.h"                      // Include simple tools
#include "mstimer.h"                          //Include timer library
#include "abdrive.h"                          //Include drive library(goto, ramp, etc.)
#include "ping.h"                             //Include ping library

#define SPEED_STEP 10                         // 10 ticks/sec / 20 ms


int goAndTrackTime(int stopDistance);



int main()                                    // Main function
{
  // Add startup code here.
  printf("1==Start\n");
  goAndTrackTime(30);
  while(1)
  {
    // Add main loop code here.
    
  }  
}

int goAndTrackTime(int stopDistance)
{
  int startTime;
  int endTime;
  int finalTime;
  int i;
  int j;
  
  mstime_start();
  
  drive_setRampStep(SPEED_STEP);
  drive_ramp(128, 128); 
  
  startTime = mstime_get();
  //printf("start time: %d\n", startTime);
  
  while(ping_cm(7) >= stopDistance) pause(5);
  
  drive_ramp(0,0);
  
  endTime = mstime_get();
  //printf("end time: %d\n", endTime);
  finalTime = endTime - startTime;
  //printf("time in motion: %d\n", finalTime);
  
  return finalTime;
}