#include "motor.h"

// 모터 통합 초기화
// 에어컨:   PA6 (IN1), PA7 (IN2), PB10 (ENA -> TIM2_CH3)
// 공기청정: PB8 (IN3), PB9 (IN4), PA1 (ENB -> TIM2_CH2)
void dcmotor_init(DCMotor_t *motor, GPIO_TypeDef *port, uint8_t in1, uint8_t in2, volatile uint32_t *ccr) {
    motor->dir_port = port;
    motor->pin_in1  = in1;
    motor->pin_in2  = in2;
    motor->ccr      = ccr;
    dcmotor_stop(motor);
}

void dcmotor_start(DCMotor_t *motor, uint32_t speed) {
    Macro_Clear_Bit(motor->dir_port->ODR, motor->pin_in1);
    Macro_Set_Bit(motor->dir_port->ODR, motor->pin_in2);
    *(motor->ccr) = speed;
}

void dcmotor_stop(DCMotor_t *motor) {
    Macro_Clear_Bit(motor->dir_port->ODR, motor->pin_in1);
    Macro_Clear_Bit(motor->dir_port->ODR, motor->pin_in2);
    *(motor->ccr) = 0;
}
