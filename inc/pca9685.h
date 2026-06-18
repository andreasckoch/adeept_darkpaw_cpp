#ifndef PCA9685_H_
#define PCA9685_H_

#include <stdbool.h>

static const int PCA9685_DEFAULT_I2C_BUS = 1;
static const int PCA9685_DEFAULT_ADDRESS = 0x40;

static const int PCA9685_MODE1 = 0x00;
static const int PCA9685_MODE2 = 0x01;
static const int PCA9685_LED0_ON_L = 0x06;
static const int PCA9685_LED0_OFF_L = 0x08;
static const int PCA9685_PRESCALE = 0xFE;

static const int PCA9685_RESTART = 0x80;
static const int PCA9685_SLEEP = 0x10;
static const int PCA9685_ALLCALL = 0x01;
static const int PCA9685_OUTDRV = 0x04;

static const int PCA9685_OSCILLATOR_HZ = 25000000;
static const int PCA9685_RESOLUTION = 4096;
static const int PCA9685_SERVO_FREQUENCY_HZ = 50;
static const int MICROSECONDS_PER_SECOND = 1000000;

struct Pca9685Device
{
    int i2c_bus;
    int i2c_address;
    int handle;
    bool gpio_initialized;
};

Pca9685Device pca9685_make_device(int i2c_bus, int i2c_address);
bool pca9685_open(Pca9685Device *device);
void pca9685_close(Pca9685Device *device);
bool pca9685_is_open(const Pca9685Device *device);

int pca9685_read_register(const Pca9685Device *device, int reg);
bool pca9685_write_register(const Pca9685Device *device, int reg, int value);
bool pca9685_set_pwm_frequency(Pca9685Device *device, int frequency_hz);
bool pca9685_set_channel_ticks(const Pca9685Device *device, int channel, int off_ticks);

static inline int pca9685_pulse_microseconds_to_ticks(int pulse_microsec)
{
    return (int)((pulse_microsec * PCA9685_RESOLUTION * PCA9685_SERVO_FREQUENCY_HZ) / MICROSECONDS_PER_SECOND);
}

static inline int pca9685_ticks_to_pulse_microseconds(int ticks)
{
    return (int)((ticks * MICROSECONDS_PER_SECOND) / (PCA9685_RESOLUTION * PCA9685_SERVO_FREQUENCY_HZ));
}

#endif /* PCA9685_H_ */
