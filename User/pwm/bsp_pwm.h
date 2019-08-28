#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "stm32f10x.h"


#define            ADVANCE_TIM                   TIM1
#define            ADVANCE_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            ADVANCE_TIM_CLK               RCC_APB2Periph_TIM1

/*
PWM_3D2  PA8
PWM_3D   PB15

PWM_2D   PB13
PWM2D2   PB14


*/

// TIM1 PB13 output
#define            PWM_2D_TIM_CH1N_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_2D_TIM_CH1N_PORT          GPIOB
#define            PWM_2D_TIM_CH1N_PIN           GPIO_Pin_13

#define            PWM_2D_TIM_CH2N_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_2D_TIM_CH2N_PORT          GPIOB
#define            PWM_2D_TIM_CH2N_PIN           GPIO_Pin_14


// TIM1 PB15 output
#define            PWM_3D_TIM_CH3N_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_3D_TIM_CH3N_PORT          GPIOB
#define            PWM_3D_TIM_CH3N_PIN           GPIO_Pin_15

#define            PWM_3D_TIM_CH4N_GPIO_CLK      RCC_APB2Periph_GPIOA
#define            PWM_3D_TIM_CH4N_PORT          GPIOA
#define            PWM_3D_TIM_CH4N_PIN           GPIO_Pin_11

void ADVANCE_TIM_Init(void);
void ADVANCE_2D_PWM_Config(int duty_2d);
void ADVANCE_3D_PWM_Config(int duty_3d);
void ADVANCE_TIM_GPIO_Config(void);

void ADVANCE_2D_1_PWM_Config(int duty_2d);
void ADVANCE_3D_1_PWM_Config(int duty_3d);


#endif
