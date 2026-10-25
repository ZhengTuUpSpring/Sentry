#ifndef CON_4310
#define CON_4310

#include "apply_4310.h"
#include "stdbool.h"
#include "pid.h"
#define CAN_YAW_CONTROL 0x05

#define YAW_SPEED_KP 5.0f
#define YAW_SPEED_KI 0.0f
#define YAW_SPPED_KD 0.0f

#define YAW_POS_KP 0.001f
#define YAW_POS_KI 0.0f
#define YAW_POS_KD 0.0f

#define YAW_IMU_KP 15.0f
#define YAW_IMU_KI 0.0f
#define YAW_IMU_KD 0.1f

#define YAW_PID_KP 0.08f
#define YAW_PID_KI 0.0005f
#define YAW_PID_KD 0.0f
#define YAW_PID_OUTMAX 20
extern pid_type_def DM4310_pid;
extern pid_type_def yaw_pos_pid;
extern PID_t YAW_IMU_SPEED_PID;

extern bool motor1_init(motor_t *motor1);
extern void dm_motor_fbdata(motor_t *motor, uint8_t *rx_data);
extern void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
extern motor_t DM_4310;

extern void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel) ;
extern void YAW_4310motor_init(void);
extern void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
extern void mit_ctrl(CAN_HandleTypeDef* hcan, motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor);
extern void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);

/* ------------------------------------------------------------------------- */
/* Friction feedforward: Models both Coulomb (constant, direction-dependent) */
/* and viscous (speed-proportional) friction.                                */
/*                                                                           */
/*   tau = Tc * tanh(alpha * omega) + Bv * omega                             */
/*                                                                           */
/* Tc    每 Coulomb friction torque (N﹞m)                                     */
/* Bv    每 viscous damping coefficient (N﹞m / (rad/s))                       */
/* alpha 每 tanh steepness (s/rad), controls how sharp the zero-crossing is   */
/* omega 每 angular velocity (rad/s)                                          */
/* ------------------------------------------------------------------------- */
typedef struct
{
    float coulomb_torque_nm;
    float viscous_coefficient_nm_per_rad_s;
    float tanh_slope_s_per_rad;
    float output_per_nm;
} friction_compensation_t;

extern void friction_compensation_init(friction_compensation_t *compensation,
                                float coulomb_torque_nm,
                                float viscous_coefficient_nm_per_rad_s,
                                float tanh_slope_s_per_rad,
                                float output_per_nm);

/* Return the feedforward torque in N﹞m. */
extern float friction_compensation_calc_torque(const friction_compensation_t *compensation,
                                        float angular_velocity_rad_s);

extern float friction_compensation_calc_output(const friction_compensation_t *compensation,
                                        float angular_velocity_rad_s);

extern friction_compensation_t YAW_fric;

#endif
