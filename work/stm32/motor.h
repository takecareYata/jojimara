#include "device_driver.h"

#ifndef DC_MOTOR_H
#define DC_MOTOR_H


typedef struct {
    GPIO_TypeDef *dir_port;
    uint8_t pin_in1;
    uint8_t pin_in2;
    volatile uint32_t *ccr;
} DCMotor_t;

void dcmotor_init(DCMotor_t *motor, GPIO_TypeDef *port, uint8_t in1, uint8_t in2, volatile uint32_t *ccr);
void dcmotor_start(DCMotor_t *motor, uint32_t speed);
void dcmotor_stop(DCMotor_t *motor);

#endif