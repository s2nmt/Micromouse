#ifndef TB6612FNG_DRIVER_H
#define TB6612FNG_DRIVER_H
#include <stdint.h>
#include "PID.h"
#include "encoder.h"
#include "stm32f1xx.h"

// MOTOR IDs
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

/* PID Controller parameters */
#define PID_KP 130.0f
#define PID_KI 0.05f
#define PID_KD 0.1f

#define PID_TAU 0.2f

#define PID_LIM_MIN 1500.0f
#define PID_LIM_MAX 5000.0f

#define PID_LIM_MIN_INT -5000.0f
#define PID_LIM_MAX_INT 500.0f

#define SAMPLE_TIME_S 0.001f


#define PID_KP_MOTOR 11.0f
#define PID_KI_MOTOR 1.1f
#define PID_KD_MOTOR 5.0f

#define PID_TAU_MOTOR 0.2f

#define PID_LIM_MIN_MOTOR 0.0f
#define PID_LIM_MAX_MOTOR 200.0f

#define PID_LIM_MIN_INT_MOTOR -500.0f
#define PID_LIM_MAX_INT_MOTOR 500.0f

#define SAMPLE_TIME_S_MOTOR 0.001f


typedef enum
{
    IDLE,
    SPEED_MODE,
    POSITION_MODE,
} system_state_t;

typedef struct
{
    double out_position;
    double target_position;
    double out_speed;
    double target_speed;
} motor_state_t;

typedef struct
{
    GPIO_TypeDef *dir_port;           // direction pin port
    uint16_t dir_pin_A;               // direction pin  number
    uint16_t dir_pin_B;               // direction pin  number
    volatile uint32_t *ccr;           // direction pin port
    uint16_t pwm_pin;                 // pwm pin number
    uint8_t motor_direction_inversed; // 0 for normal, 1 for inversed
} motor_parameter_t;

typedef struct
{
    encoder_t enc;
    PIDController pid;   // PID controller instance
    motor_parameter_t m; // Motor parameters
} motor_controller_t;


#endif // MOTOR_CONTROLLER_H
