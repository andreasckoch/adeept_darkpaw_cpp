#include "servo.h"

int main(int, char**) {
    int servos = pca9685_init();
    pca9685_move_legs_synchronized(servos);
    return 0;
}
