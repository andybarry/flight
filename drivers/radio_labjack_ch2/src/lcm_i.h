#ifndef LCM_I_H_
#define LCM_I_H_

int lcm_get_servo();
int lcm_publish_init();
void lcm_publish_u_actual(double timestamp, double aileron, double elevator, double throttle, double rudder, double kp_height, double kd_height, double kp_yaw, double kd_yaw, double kp_pitch, double kd_pitch, double kp_roll, double kd_roll);


#endif /* LCM_I_H_ */
