#include "device_driver.h"

#ifndef DC_MOTOR_H
#define DC_MOTOR_H


typedef struct {
    GPIO_TypeDef *dir_port;
    uint8_t pin_in1;
    uint8_t pin_in2;
    volatile unsigned int *ccr;
} DCMotor_t;

void DCMotor_Init(DCMotor_t *motor, GPIO_TypeDef *port, uint8_t in1, uint8_t in2, volatile unsigned int *ccr);
void DCMotor_Start(DCMotor_t *motor, unsigned int speed);
void DCMotor_Stop(DCMotor_t *motor);

#endif