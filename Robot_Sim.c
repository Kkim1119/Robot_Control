/*
  Where I will be testing and creating functions for SD project.
  
  
  Resource: http://learn.parallax.com/propeller-c-tutorials 
  
*/
//Goal: Make a function that allows the robot to ramp up and down while keeping track of the time it took to move.


#include <simpletools.h>                      // Include simple tools
#include <mstimer.h>                         //Include timer library
#include <abdrive.h>                          //Include drive library(goto, ramp, etc.)
#include <ping.h>                             //Include ping library
#include <stdlib.h>                           //Include math library

fdserial *xbee;
int *moveData;
//int *scanData;

#define SPEED_STEP 10                         // 10 ticks/sec / 20 ms
#define SETS       1                          // The amount of move/turn sets that will be executed
#define SIMULATION_RUN 5                      //The amount of times that scan/move data will be returned(1 run= 1 scan, 1 move)     
#define NUMBER_SCAN_TURNS 1                   //36 turns = 1 complete scan
#define SCAN_MOVE_SET 10 
#define BUFFER_SIZE 200
#define BUFFER_LOW 0
#define BUFFER_HIGH 1

int go_and_track_time(int stopDistance);      //Makes the robot move until it arrives near a wall/object and returns run time
int generate_rand_num(int maxNum);            //Generates a random number between 1 and maxNum
int robot_turn_random();                      //Makes the robot turn 0,90, 180, or -90 degrees randomly, returns rand value
void move_to_new_location(int maxSets);       //Makes the robot move to random location, returns data in turn, ms format
void robot_scan();                            //Makes robot take 360-degree scan at current location. Returns data(turn, ping)


int stateValue;
int iState;
enum state{INIT, IDLE, WAIT_CMD, MOVE_CMD, SCAN_CMD, SCAN_MOVE_CMD, GOTO_CMD};

unsigned char data_storage_low[BUFFER_SIZE] = {NULL};
unsigned char data_storage_high[BUFFER_SIZE] = {NULL};
//functions used by state machine ---------------------------

int goto_next(int next_state)
{
   iState = next_state;
   return 0;
}

int get_step(void)
{
  return iState;
}

// Main function --------------------------------------------

int main()                                    
{
  int i;
  
  xbee = fdserial_open(9, 8, 0, 9600);
    
  goto_next(INIT);
  
  while(1)
  {    
    switch(get_step())
    {
      case INIT:
        initialize();
        goto_next(IDLE);
        break;
        
      case IDLE:
        communication();
        break;

      case WAIT_CMD:
        i = wait_cmd();
        //dprint(xbee, "i:%x\n", i);
        goto_next(i);
        break;
        
      case MOVE_CMD:
        move_cmd(); 
        //goto_next(SEND_DATA);
        goto_next(IDLE);
        break;
        
      case SCAN_CMD:
        scan_cmd();
        goto_next(IDLE);
        break;
      
      case SCAN_MOVE_CMD:
        scan_move_cmd();
        goto_next(IDLE);
        break;
        
      case GOTO_CMD:
        goto_cmd();
        goto_next(IDLE);
        break;
        
      default:
        goto_next(IDLE);
        break;
    }
  }
  
}

//functions that represent the states of the robot(+debugging tool) --------------

char debug_stop(fdserial *serial, char *str)
{
  char c;
  
  dprint(serial, str);
  
  while(1)
  {
    c = fdserial_rxChar(serial);
    if(c != -1)
    {
      return c;
    }
  }    
  
}

int initialize(void)
{
  dprint(xbee, "Robot is done booting\n");
}

#define COUNTER 10000
#define LOAD_READY 0
#define LOAD_COMPLETE 1
int communication(void)
{
  int data;
  int buffer_pointer;
  int time_out;
  int buffer_type;
  unsigned char buffer_low[BUFFER_SIZE];
  unsigned char buffer_high[BUFFER_SIZE];
  int timer = 0;
  int load_status = LOAD_READY;
  
  dprint(xbee, "communication\n");
  
  while(1)
  {
    buffer_type = BUFFER_LOW;
    buffer_pointer = 0;
    data = fdserial_rxCharTime(xbee, COUNTER);
    dprint(xbee, "2===%x\n", data);
    
    if(data == 0x8000)
    {
      if(buffer_pointer != 0)
      {
        load_data(buffer_low, buffer_high, data_storage_low, data_storage_high);
        load_status = LOAD_COMPLETE;
        
        dprint(xbee, "load status: %s", data_storage_low);
        
        buffer_pointer = 0;
        dprint(xbee, "data received, time out");
        
      }
      else
      {
        dprint(xbee, "no data received, time out");
      }  
    }
    else if(data != -1)             //Data Received
    {
      dprint(xbee, "3===\n");
      if(buffer_type == BUFFER_LOW)
      {
        buffer_low[buffer_pointer] = data;
        buffer_pointer++;
        if(buffer_pointer > BUFFER_SIZE - 1)
        {
          buffer_type = BUFFER_HIGH;
          buffer_pointer = 0;
        }
      }
      else
      {
        buffer_high[buffer_pointer] = data;
        buffer_pointer++;
        if(buffer_pointer > BUFFER_SIZE - 1)
        {
          load_data(buffer_low, buffer_high, data_storage_low, data_storage_high);
          load_status = LOAD_COMPLETE;
          buffer_pointer = 0;
        }
      }
      
    }
    else          //Data not given
    {
      //PLaceholder
    }
  } 
}


int load_data(unsigned char *src_low, unsigned char *src_high, unsigned char *dest_low, unsigned char *dest_high)
{
  int i;
  
  for(i=0; i< BUFFER_SIZE; i++)
  {
    *(src_low + i)  = *(dest_low + i);
    *(src_high + i) = *(dest_high + i);
    
  }
}

int wait_cmd(void)
{
  char c;
  int iCmd;
  char const default_val = 0;
  char byte_1 = default_val, byte_2 = default_val, byte_3 = default_val;
  
  dprint(xbee, "wait_cmd\n");
  
  while(1)
  {
    c = fdserial_rxChar(xbee);
    if(c != -1)
    {
      if(c == 0x30)
      {
        
        dprint(xbee, "run command: %x \n", byte_2);
        
        switch(byte_2)
        {
          case 0x32:
            iCmd = MOVE_CMD;
            break;
          case 0x33:
            iCmd = SCAN_CMD;
            break;
          case 0x34:
            iCmd = SCAN_MOVE_CMD;
            break;
          case 0x35:
            iCmd = GOTO_CMD;
            break;
          default:
            iCmd = WAIT_CMD;
            break;
        }
        
        byte_3 = default_val;
        byte_2 = default_val;
        byte_1 = default_val;

        return (iCmd);
      }
      else
      {
        if(byte_3 == default_val)
        {
          byte_3 = c;
        }
        else if(byte_2 == default_val)
        {
          byte_2 = c;
        }
        else  // digit_1 == 10
        {
          byte_1 = c;
        }        
      }    
    }
  } 

}


int move_cmd(void)
{

  dprint(xbee, "move_cmd\n");
  
  move_to_new_location(SETS);        //Robot will turn & move/store data 5 times : turn, move, turn, move, ...
  
}

int scan_cmd(void)
{
  
  dprint(xbee, "scan_cmd\n");
  
  robot_scan();
  
}

int scan_move_cmd(void)
{
  dprint(xbee, "scan_move_cmd\n");
  scan_and_move();
}

int goto_cmd(void)
{

  dprint(xbee, "goto_cmd\n");
  

}


//functions used by the different state functions -------------------

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

/*
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

int robot_scan()        //maxSets = the amount of turn/move sets that the robot will perform
{
  int rangeAndSize = 72;
  int degrees = 0;
  //int scanDataSet[rangeAndSize];
  int i;
  
  for(i=0; i<rangeAndSize; i=i+2)
  {
    //scanDataSet[i] = degrees;
   
    //scanDataSet[i+1] = 0;
    degrees = degrees + 10;
    drive_goto(-3,3);
    pause(100);
  }
  
  //return scanDataSet;
}
*/
void move_to_new_location(int maxSets)        //New move function, now incorporated with the data printing
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
  
  dprint(xbee, "1,move");
  for(i=0; i< rangeAndSize; i++)
  {
    dprint(xbee,",%d", moveDataSet[i]);
  }
  dprint(xbee, "\n\n");
}

#define SCAN_STEP 38

void robot_scan()                         //New scan function, now incorporated with the data printing
{
  double scanDataDegree[SCAN_STEP];
  int scanDataPing[SCAN_STEP];
  int i;

  for(i=0; i<SCAN_STEP; i++)
  {
    
    scanDataDegree[i] = (double)(i) * (360/((double)SCAN_STEP));
    scanDataPing[i] = ping_cm(7);
    
    drive_goto(-3,3);
 
    pause(100);
    
  }
  
  dprint(xbee, "1,scan");
  for(i=0; i< SCAN_STEP; i++)
  {
    dprint(xbee,",%f", scanDataDegree[i]);
    dprint(xbee,",%d", scanDataPing[i]);
  }
  dprint(xbee, "\n\n");

}

void scan_and_move(void)
{
  robot_scan();
  move_to_new_location(SETS);
}