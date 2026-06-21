#include "servo.h"

#include <stdio.h>
#include <unistd.h>

#include "servo_calibration.h"

static void _legs_transition(Pca9685Device *servos, int steps, const int positions[SERVO_COUNT], const int goal_positions[SERVO_COUNT]);

Pca9685Device pca9685_init()
{
    Pca9685Device servos = pca9685_make_device(PCA9685_DEFAULT_I2C_BUS, PCA9685_DEFAULT_ADDRESS);
    if (!pca9685_open(&servos))
    {
        return servos;
    }

    if (!pca9685_set_pwm_frequency(&servos, PCA9685_SERVO_FREQUENCY_HZ))
    {
        printf("Error: PCA9685 frequency setup failed\n");
        pca9685_close(&servos);
    }

    return servos;
}

bool servo_set_pulse_microseconds(Pca9685Device *servos, int channel, int pulse_microsec)
{
    if (!servo_is_valid_index(channel))
    {
        printf("Error: invalid servo channel (%d)\n", channel);
        return false;
    }

    int clamped_pulse_microsec = servo_clamp_pulse_microseconds(channel, pulse_microsec);
    if (clamped_pulse_microsec != pulse_microsec)
    {
        printf("Warning: clamped servo [%d] pulse from %d us to %d us\n",
               channel, pulse_microsec, clamped_pulse_microsec);
    }

    int pulse_ticks = pca9685_pulse_microseconds_to_ticks(clamped_pulse_microsec);
    if (!pca9685_set_channel_ticks(servos, channel, pulse_ticks))
    {
        printf("Error: failed to set servo [%d] pulse to %d us (%d ticks)\n",
               channel, clamped_pulse_microsec, pulse_ticks);
        return false;
    }

    return true;
}

void pca9685_move_legs_synchronized(Pca9685Device *servos)
{
    if (!pca9685_is_open(servos))
    {
        printf("Error: invalid servo controller; refusing to move legs.\n");
        return;
    }

    /*
    *   (leg 1 - front left)
    *   - servo 0:
    *   lower limit: back
    *   upper limit: front
    *   - servos 1 and 2:
    *   both uppper limit: leg high
    *   both lower  limit: leg low
    *   1 lower limit - 2 upper limit: leg close
    *   1 upper limit - 2 lower limit: leg far
    *
    *   (leg 2 - back left)
    *   - servo 3:
    *   lower limit: front
    *   upper limit: back
    *   - servos 4 and 5:
    *   both uppper limit: leg low
    *   both lower  limit: leg high
    *   4 lower limit - 5 upper limit: leg far
    *   4 upper limit - 5 lower limit: leg close
    *
    *   (leg 3 - front right)
    *   - servo 6:
    *   lower limit: front
    *   upper limit: back
    *   - servos 7 and 8:
    *   both uppper limit: leg far
    *   both lower  limit: leg close
    *   7 lower limit - 8 upper limit: leg low
    *   7 upper limit - 8 lower limit: leg high
    *
    *   (leg 4 - back right)
    *   - servo 9:
    *   lower limit: back
    *   upper limit: front
    *   - servos 10 and 11:
    *   both uppper limit: leg high
    *   both lower  limit: leg low
    *   10 lower limit - 11 upper limit: leg close
    *   10 upper limit - 11 lower limit: leg far
    */

    /* Servo positions (pulses in microseconds) */
    int positions_legs_low[SERVO_COUNT] = {
        1700, 380, 380,
        1700, 1400, 1500,
        380, 1400, 1650,
        380, 650, 1000,
    };

    int positions_legs_high[SERVO_COUNT] = {
        1700, 1500, 1500,
        1700, 380, 500,
        380, 380, 550,
        380, 1500, 2100,
    };

    int positions_legs_close[SERVO_COUNT] = {
        1700, 380, 1500,
        1700, 1400, 500,
        380, 1400, 550,
        380, 650, 2100,
    };

    int positions_legs_far[SERVO_COUNT] = {
        1700, 1500, 380,
        1700, 380, 1500,
        380, 380, 1650,
        380, 1500, 1000,
    };

    _legs_transition(servos, 20, positions_legs_high, positions_legs_low);
    usleep(5000000);
    _legs_transition(servos, 20, positions_legs_low, positions_legs_far);
    usleep(5000000);
    _legs_transition(servos, 20, positions_legs_far, positions_legs_close);
    usleep(5000000);
    _legs_transition(servos, 20, positions_legs_close, positions_legs_high);
}

static void _legs_transition(Pca9685Device *servos, int steps, const int positions[SERVO_COUNT], const int goal_positions[SERVO_COUNT])
{
    for (int step = 1; step <= steps; step++)
    {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (positions[i] == goal_positions[i])
            {
                continue;
            }

            int pulse_microsec = positions[i] + step * (int)((goal_positions[i] - positions[i]) / steps);
            printf("Step [%d] - Servo [%d] - pulse_microsec [%d]\n", step, i, pulse_microsec);
            servo_set_pulse_microseconds(servos, i, pulse_microsec);
            usleep(2000);
        }
    }
}
