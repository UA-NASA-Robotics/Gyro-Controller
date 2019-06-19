//*****************************************
//********TIMER.H file code****************
//*****************************************

#ifndef TIMER_H_
#define TIMER_H_
#include <stdbool.h>
#include <stdlib.h>

 typedef struct{
        unsigned long timerInterval;
        unsigned long lastMillis;
    }timers_t;
    
bool timerDone(timers_t * t);
void setTimerInterval(timers_t * t, unsigned long interval);
void resetTimer(timers_t * t);
void globalTimerTracker( );
unsigned long millis(void);
void delay(unsigned int val);
//*****************************************
//**********    END OF H file  ************
//*****************************************

#endif