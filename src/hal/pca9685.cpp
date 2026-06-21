#include "pca9685.h"

#include "pigpio.h"

#include <stdio.h>
#include <unistd.h>

Pca9685Device pca9685_make_device(int i2c_bus, int i2c_address)
{
    Pca9685Device device;
    device.i2c_bus = i2c_bus;
    device.i2c_address = i2c_address;
    device.handle = -1;
    device.gpio_initialized = false;
    return device;
}

bool pca9685_open(Pca9685Device *device)
{
    if (device == NULL)
    {
        return false;
    }

    int gpio_custom_port = gpioCfgSocketPort(8889);
    int gpio_init = gpioInitialise();

    printf("gpio custom port: %d\n", gpio_custom_port);
    printf("gpioInit: %d\n", gpio_init);
    if (gpio_init < 0)
    {
        printf("Error: GPIO not initialized (%d)\n", gpio_init);
        return false;
    }

    device->gpio_initialized = true;
    device->handle = i2cOpen(device->i2c_bus, device->i2c_address, 0);
    printf("Servos: %d\n", device->handle);
    if (device->handle < 0)
    {
        printf("Error: PCA9685 I2C open failed on bus %d at address 0x%02X (%d)\n",
               device->i2c_bus, device->i2c_address, device->handle);
        printf("Check that /dev/i2c-%d exists and that i2cdetect -y %d shows device 0x%02X.\n",
               device->i2c_bus, device->i2c_bus, device->i2c_address);
        gpioTerminate();
        device->gpio_initialized = false;
        return false;
    }

    return true;
}

void pca9685_close(Pca9685Device *device)
{
    if (device == NULL)
    {
        return;
    }

    if (device->handle >= 0)
    {
        i2cClose(device->handle);
        device->handle = -1;
    }

    if (device->gpio_initialized)
    {
        gpioTerminate();
        device->gpio_initialized = false;
    }
}

bool pca9685_is_open(const Pca9685Device *device)
{
    return device != NULL && device->handle >= 0;
}

int pca9685_read_register(const Pca9685Device *device, int reg)
{
    if (!pca9685_is_open(device))
    {
        return -1;
    }

    return i2cReadByteData(device->handle, reg);
}

bool pca9685_write_register(const Pca9685Device *device, int reg, int value)
{
    if (!pca9685_is_open(device))
    {
        return false;
    }

    return i2cWriteByteData(device->handle, reg, value) == 0;
}

bool pca9685_set_pwm_frequency(Pca9685Device *device, int frequency_hz)
{
    if (!pca9685_is_open(device) || frequency_hz <= 0)
    {
        return false;
    }

    int mode = pca9685_read_register(device, PCA9685_MODE1);
    printf("Mode: %d\n", mode);
    if (mode < 0)
    {
        return false;
    }

    float prescale_value = ((float)PCA9685_OSCILLATOR_HZ / (PCA9685_RESOLUTION * frequency_hz)) - 0.5f;
    int prescale = (int)prescale_value;

    if (!pca9685_write_register(device, PCA9685_MODE2, PCA9685_OUTDRV))
    {
        return false;
    }
    if (!pca9685_write_register(device, PCA9685_MODE1, PCA9685_ALLCALL))
    {
        return false;
    }
    usleep(5000);

    int mode1 = pca9685_read_register(device, PCA9685_MODE1);
    if (mode1 < 0)
    {
        return false;
    }

    mode1 = mode1 & ~PCA9685_SLEEP;
    if (!pca9685_write_register(device, PCA9685_MODE1, mode1))
    {
        return false;
    }
    usleep(5000);

    int oldmode = pca9685_read_register(device, PCA9685_MODE1);
    if (oldmode < 0)
    {
        return false;
    }

    int newmode = (oldmode & 0x7F) | PCA9685_SLEEP;
    if (!pca9685_write_register(device, PCA9685_MODE1, newmode))
    {
        return false;
    }
    if (!pca9685_write_register(device, PCA9685_PRESCALE, prescale))
    {
        return false;
    }
    if (!pca9685_write_register(device, PCA9685_MODE1, oldmode))
    {
        return false;
    }
    usleep(5000);

    return pca9685_write_register(device, PCA9685_MODE1, oldmode | PCA9685_RESTART);
}

bool pca9685_set_channel_ticks(const Pca9685Device *device, int channel, int off_ticks)
{
    if (!pca9685_is_open(device) || channel < 0 || channel >= 16 || off_ticks < 0 || off_ticks >= PCA9685_RESOLUTION)
    {
        return false;
    }

    int base_reg = PCA9685_LED0_ON_L + (4 * channel);
    return pca9685_write_register(device, base_reg, 0) &&
           pca9685_write_register(device, base_reg + 1, 0) &&
           pca9685_write_register(device, base_reg + 2, off_ticks & 0xFF) &&
           pca9685_write_register(device, base_reg + 3, off_ticks >> 8);
}
