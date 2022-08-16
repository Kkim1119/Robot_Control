/*
  Where I will be testing and creating functions for SD project.
  
  
  Resource: http://learn.parallax.com/propeller-c-tutorials 
  
*/
//Goal: Make a function that allows the robot to ramp up and down while keeping track of the time it took to move.


#include "simpletools.h"                      // Include simple tools
#include "mstimer.h"                          //Include timer library
#include "abdrive.h"                          //Include drive library(goto, ramp, etc.)
#include "ping.h"                             //Include ping library
#include "stdlib.h"                           //Include math library
#include "fdserial.h"                         //Makes XBee wireless connection work

fdserial *xbee;

#define SPEED_STEP 10                         // 10 ticks/sec / 20 ms
#define SETS       5                          // The amount of move/turn sets that will be executed
#define SIMULATION_RUN 5                      //The amount of times that scan/move data will be returned(1 run= 1 scan, 1 move)
#define NUMBER_SCAN_TURNS 1                   //36 turns = 1 complete scan
 

int go_and_track_time(int stopDistance);      //Makes the robot move until it arrives near a wall/object and returns run time
int generate_rand_num(int maxNum);            //Generates a random number between 1 and maxNum
int robot_turn_random();                      //Makes the robot turn 0,90, 180, or -90 degrees randomly, returns rand value
int* move_to_new_location(int maxSets);       //Makes the robot move to random location, returns data in turn, ms format
int* robot_scan();                                  //Makes robot take 360-degree scan at current location. Returns data(turn, ping)



int main()                                    // Main function
{
  xbee = fdserial_open(9, 8, 0, 9600);
  
  //int *scanData;
  int *moveData;
  //int degrees[] ={0  ,10 ,20 ,30 ,40 ,50 ,60 ,70 ,80 ,90 ,100,110,120,130,140,150,160,170,
  //               180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360};
  int i;
 
  dprint(xbee,"2==Start\n");
  /*
  dprint(xbee, "1,scan");
  for(i=0; i<36; i++)
  {
    //scanData = robot_scan();                            //Robot will scan/store data: turn, ping, turn, ping...
    pause(100);
    Printing the SCAN data in the MAP DATA FORMAT
    //dprint(xbee, "1,scan");
    dprint(xbee,",%d", degrees[i]);
    dprint(xbee,",%d", robot_scan()[i]);
  
  }
  dprint(xbee, "\n\n");
  
  */

  moveData = move_to_new_location(SETS);        //Robot will turn & move/store data 5 times : turn, move, turn, move, ...

  
  
  
  /**Printing the MOVE data in the MAP DATA FORMAT**/
  dprint(xbee, "1,move");
  for(i=0; i<(SETS*2); i++)
  {
    dprint(xbee,",%d", moveData[i]);
  }
   dprint(xbee, "\n\n");
 
}

int go_and_track_time(int stopDistance)       //stopDistance = distance between a detected wall and robot when stopped
{
  int startTime;  //Time recorded when robot started to move
  int endTime;    //Time recorded when robot stopped
  int finalTime;  //Time that the robot moved for
  
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

int generate_rand_num(int maxNum)
{
  int randomNumber;
  srand(time(NULL));
  randomNumber = (rand() % maxNum) + 1;
  //printf("random number: %d\n", randomNumber);
  
  return randomNumber;
}

int robot_turn_random()
{
  int choice = generate_rand_num(3);          //assigns variable choice a value of 1,2, or 3.
  
  switch(choice)
  {
    case 1:
      drive_goto(-27,27);                     //Turns 90 degrees counter-clockwise(left turn)
      
      break;
      
    case 2:
      drive_goto(27,-27);                     //Turns 90 degrees clockwise(right turn)
      
      break;
      
    case 3:
      drive_goto(54,-54);                     //Turns 180 degrees clockwise(turns around)
      
      break;
  }
  
  return choice;
}
int* move_to_new_location(int maxSets)        //maxSets = the amount of turn/move sets that the robot will perform
{
  int rangeAndSize = maxSets * 2;
  
  int moveDataSet[rangeAndSize];
  int i;
  int randomTurn;
  int timeMoved;
  
  for(i=0; i<rangeAndSize; i=i+2)
  {
    randomTurn = robot_turn_random();
    
    if(randomTurn == 1)
    {
      moveDataSet[i] = -90;
    }
    else if(randomTurn == 2)
    {
      moveDataSet[i] = 90;
    }
    else
    {
      moveDataSet[i] = 180;
    }
      
    timeMoved = go_and_track_time(30);
    
    moveDataSet[i+1] = timeMoved;
  }
  
  return moveDataSet;
}

/* Code does not work as intended(ping and degrees return weird random numbers)

int* robot_scan()
{
  int i;
  int ping_value;
  
  int pingDataSet[NUMBER_SCAN_TURNS];
 
  for(i = 0; i < NUMBER_SCAN_TURNS; i=i++)
  {
    ping_value = ping_cm(7);
    //pause(100);
    pingDataSet[i] = ping_value;
    
    drive_goto(-3,3);
  
    //pause(100);
  }
  drive_goto(-3,3);
  
  return pingDataSet;
}
*/