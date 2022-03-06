#ifndef SERVO_H_
#define SERVO_H_


#include <stdio.h>
#include "bcm_host.h"


int pca9685_init();
void pca9685_move_legs_synchronized(int servos);

#endif /* SERVO_H_ */