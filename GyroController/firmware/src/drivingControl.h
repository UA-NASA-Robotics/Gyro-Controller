/* 
 * File:   drivingControl.h
 * Author: John
 *
 * Created on May 10, 2018, 4:56 PM
 */

#ifndef DRIVINGCONTROL_H
#define	DRIVINGCONTROL_H

#include <stdbool.h>
#define COUNTS_PER_CENTI 3636
#include "Motor.h"
#include "MotorDefinitions.h"
#include "motorHandler.h"


void saveMotorParms();
void restoreMotors();
void slowRightSpeed();
void slowLeftSpeed();


#endif	/* DRIVINGCONTROL_H */

