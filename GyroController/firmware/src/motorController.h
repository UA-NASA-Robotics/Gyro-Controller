/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _MOTOR_CONTROLLER_H    /* Guard against multiple inclusion */
#define _MOTOR_CONTROLLER_H

#include <stdint.h>

//MOTORS GET A 3 HEX ADDRESS
//When sending the motor address to send data - SDO requires a prefix
#define SDO_SENDING_PREFIX      0x600

//When the motor sends back to you, there is a prefix
#define SDO_RECEIVING_PREFIX    0x500

//The low byte of the motor's address
#define MOTOR_R_ADDRESS         0x7C    //SEND 0x67C - RECIEVE 0x57C
#define MOTOR_L_ADDRESS         0x7D    //SEND 0x67D - RECIEVE 0x57D
#define MOTOR_BUCKET_ADDRESS    0x7E    //SEND 0x67E - RECIEVE 0x57E
#define MOTOR_CONVEYOR_ADDRESS  0x7F    //SEND 0x67F - RECIEVE 0x57F

#define MAXCURRENTBG65              20000
#define MAXRPM                      4000
#define ACCEL_CONST                 1000000 //in rev/min^2, value should be between 100k and 10k *this value will probably have to be changed under load. 

#define  CLEAR_ERRORS               0x3000, 0x00, 0x01
#define  SET_MOTOR                  0x3900, 0x00, 0x01
#define  SET_POLES                  0x3910, 0x00, 10
#define  MOTOR_POLARITY             0x3911, 0x00, 0x02
#define  SET_FEEDBACK_ENCODER       0x3350, 0x00, 2410 //2410 for encoder 
#define  SET_FEEDBACK_HALL          0x3350, 0x00, 2378 //2378 for hall 
#define  SET_SVEL_FEEDBACK_ENCODER  0x3550, 0x00, 2410 //2410 for encoder 
#define  SET_SVEL_FEEDBACK_HALL     0x3550, 0x00, 2378 //2378 for hall
#define  SET_FEEDBACK_RESOLUTION    0x3962, 0x00, 2000 
#define  CURRENT_LIMIT_POS          0x3221, 0, MAXCURRENTBG65
#define  CURRENT_LIMIT_NEG          0x3223, 0, MAXCURRENTBG65
#define  VELOCITY_LIMIT_POS         0x3321, 0x00, MAXRPM
#define  VELOCITY_LIMIT_NEG         0x3323, 0x00, MAXRPM
#define  POWER_ENABLE               0x3004, 0x00, 0x01
#define  POWER_DISABLE              0x3004, 0x00, 0x00

#define  MODE_VEL                   0x3003, 0x00, 0x3
#define  VEL_ACC                    0x3380, 0x00, ACCEL_CONST
#define  VEL_DEC                    0x3381, 0x00, ACCEL_CONST

void initMotorLeft(void);
void initMotor(uint8_t motorAddress);
void sendMotorCommand(long value);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */