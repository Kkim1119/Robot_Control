/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/

#define TABLE_LENGTH 14
#define DIR_FWD       1
#define DIR_REV      -1     //reverse direction

int const speed_table[TABLE_LENGTH] = {0  , 10, 20, 30, 40, 50, 60, 70, 80, 90,100,110,120,128};
int const pause_table[TABLE_LENGTH] = {300,280,260,240,220,200,180,160,140,120,100, 80, 60, 40};

/*
 time : argument x 100ms
 direction : DIR_FWD, DIR_REV
 */
void goAndStopSmooth(int time, int direction)
{
    int i;
   
    for(i=0; i<TABLE_LENGTH; i++)
    { 
       drive_speed(speed_table[i]*direction ,speed_table[i]*direction );
       pause(pause_table[i]);
    }
    
    pause(time * 100);
    
    
    for(i=TABLE_LENGTH-1; i>=0; i--)
    {
       drive_speed(speed_table[i]*direction ,speed_table[i]*direction);
       pause(pause_table[i]);
    }
    
}


