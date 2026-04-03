#ifndef __CONTROLLER_A_H
#define __CONTROLLER_A_H
#include "types.h"

void ctrl_a_init(void);
float ctrl_a_step(axis_t axis, float theta_deg, float omega_deg_s, float theta_ref_deg,
                  float dt);

#endif /* __CONTROLLER_A_H */

