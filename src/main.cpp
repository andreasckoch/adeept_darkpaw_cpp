#include "servo.h"

int main(int, char**) {
    Pca9685Device servos = pca9685_init();
    if (!pca9685_is_open(&servos)) {
        return 1;
    }

    pca9685_move_legs_synchronized(&servos);
    pca9685_close(&servos);
    return 0;
}
