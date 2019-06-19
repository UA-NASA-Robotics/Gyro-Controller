#include "drivingControl.h"
#include "Timers.h"
#include "Definitions.h"
#define SPEED_REDUCTION_FACTOR  0.20
void modifiyDriveSpeed(int leftSpeed, int rightSpeed);

void slowLeftSpeed()
{
    //modifiyDriveSpeed(LeftMotor.Velocity - SPEED_REDUCTION_FACTOR, RightMotor.Velocity);
    setMotorControlMode(&LeftMotor,Velocity,getMotorVelocity(&LeftMotor)* SPEED_REDUCTION_FACTOR);
    setMotorControlMode(&LeftMotor,Position,getMotorVelocity(&LeftMotor)* SPEED_REDUCTION_FACTOR); 
    long remainingCounts = getMotorTargetCounts(&LeftMotor) - getMotorPosition(&LeftMotor);
    setMotorCounts(&LeftMotor,remainingCounts);
   
//    printf("leftSpeed: %d\r",getMotorVelocity(&LeftMotor));
//    printf("rightSpeed: %d\r",getMotorVelocity(&RightMotor));
//    printf("motorCount: %ld\r",remainingCounts);
}
void slowRightSpeed()
{
   // modifiyDriveSpeed(LeftMotor.Velocity, RightMotor.Velocity - SPEED_REDUCTION_FACTOR);
    setMotorControlMode(&RightMotor,Velocity,getMotorVelocity(&RightMotor)* SPEED_REDUCTION_FACTOR);
    setMotorControlMode(&RightMotor,Position,getMotorVelocity(&RightMotor)* SPEED_REDUCTION_FACTOR);  
    long remainingCounts = getMotorTargetCounts(&RightMotor) - getMotorPosition(&RightMotor);
    setMotorCounts(&RightMotor,remainingCounts);
}
void modifiyDriveSpeed(int leftSpeed, int rightSpeed)
{
    setMotorControlMode(&LeftMotor,Velocity,LeftMotor.Velocity);
    setMotorControlMode(&RightMotor,Velocity,RightMotor.Velocity);
    setMotorControlMode(&LeftMotor,Position,LeftMotor.Velocity);            
    setMotorControlMode(&RightMotor,Position,RightMotor.Velocity);  
//    setMotorCounts(&LeftMotor, getMotorTargetCounts(&LeftMotor));
//    setMotorCounts(&RightMotor, getMotorRemainingCounts(&RightMotor));
}
void saveMotorParms()
{
    getMotorVelocity(&LeftMotor);
    getMotorVelocity(&RightMotor);
    getMotorTargetCounts(&LeftMotor);
    getMotorTargetCounts(&RightMotor);
    
    requestMotorData(&LeftMotor         , REQUEST_TARGET_COUNTS);
    requestMotorData(&RightMotor        , REQUEST_TARGET_COUNTS);
    requestMotorData(&LeftMotor         , ENCODER_POSITION_REQUESTED);
    requestMotorData(&RightMotor        , ENCODER_POSITION_REQUESTED);
    
    requestMotorData(&LeftMotor         , REQUEST_VELOCITY);
    requestMotorData(&RightMotor        , REQUEST_VELOCITY);
    
   
    while(!getMotorFreshVelocity(&LeftMotor)){delay(1);}
    while(!getMotorFreshVelocity(&RightMotor)){delay(1);}
    while(!getFreshTargetCounts(&RightMotor)){delay(1);}
    while(!getFreshTargetCounts(&LeftMotor)){delay(1);}
    
}
void restoreMotors()
{
    
    int leftVel = getMotorVelocity(&LeftMotor);
    int RightVel = getMotorVelocity(&RightMotor);

    setMotorControlMode(&LeftMotor,Velocity,leftVel);
    setMotorControlMode(&RightMotor,Velocity,RightVel);
    setMotorControlMode(&LeftMotor,Position,leftVel);            
    setMotorControlMode(&RightMotor,Position,RightVel);   
    saveMotorParms();
    long remainingCountsLeft = getMotorTargetCounts(&LeftMotor) - getMotorPosition(&LeftMotor);
    long remainingCountsRight = getMotorTargetCounts(&RightMotor) - getMotorPosition(&RightMotor);
    setMotorCounts(&LeftMotor, remainingCountsLeft);
    setMotorCounts(&RightMotor, remainingCountsRight);
}