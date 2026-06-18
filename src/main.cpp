#include "servo.h"

int main(int, char**) {
    int servos = pca9685_init();
    if (servos < 0) {
        return 1;
    }

    pca9685_move_legs_synchronized(servos);
    return 0;
}
