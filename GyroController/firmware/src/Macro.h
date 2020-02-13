/* 
 * File:   Macro.h
 * Author: John
 *
 * Created on May 1, 2018, 4:28 PM
 */

#ifndef MACRO_H
#define	MACRO_H
#include "changeNotification.h"
#include "Motor.h"

typedef enum{
    Drive_Monitoing,
    TURNING_MACRO
            
}MacroTypes;

void handleMacroStatus();
void handleCANmacro(short _macroID, short _macroDATA);
void configureMacro(int macroID, int macroData);
void macroComplete(MacroTypes lastMacro);
bool isMacroRunning();
void runMacro();
void stopMacro();
bool turnDegrees(int _rotation);
bool monitorDrive();
void updateMotors(float updatedAngle);
void moveMotors(float _degrees,float currentAngle);
void testMoitorDrive(int dist,int speed);
void testTurnDegrees(int degrees);

#endif	/* MACRO_H */

