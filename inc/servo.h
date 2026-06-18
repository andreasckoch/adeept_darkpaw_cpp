#ifndef SERVO_H_
#define SERVO_H_


#include <stdio.h>

#include "pca9685.h"

Pca9685Device pca9685_init();
void pca9685_move_legs_synchronized(Pca9685Device *servos);
bool servo_set_pulse_microseconds(Pca9685Device *servos, int channel, int pulse_microsec);

#endif /* SERVO_H_ */
