#ifndef __CONTROLLER_B_H
#define __CONTROLLER_B_H
#include "types.h"

void ctrl_b_init(void);
float ctrl_b_step(axis_t axis, float theta_deg, float omega_deg_s, float theta_ref_deg,
                  float dt);

#endif /* __CONTROLLER_B_H */

