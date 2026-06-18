#include "pigpio.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#define DEFAULT_I2C_BUS     1
#define DEFAULT_PCA9685_ADDR 0x40

#define MODE1               0x00
#define MODE2               0x01
#define PRESCALE            0xFE

struct Options
{
    int i2c_bus;
    int i2c_addr;
    bool skip_camera;
};

static void print_usage(const char *program)
{
    printf("Usage: %s [--i2c-bus N] [--address 0x40] [--skip-camera]\n", program);
}

static bool parse_int(const char *value, int *parsed)
{
    char *end = NULL;
    long result = strtol(value, &end, 0);
    if (end == value || *end != '\0')
    {
        return false;
    }

    *parsed = (int)result;
    return true;
}

static bool parse_args(int argc, char **argv, Options *options)
{
    options->i2c_bus = DEFAULT_I2C_BUS;
    options->i2c_addr = DEFAULT_PCA9685_ADDR;
    options->skip_camera = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--i2c-bus") == 0)
        {
            if (i + 1 >= argc || !parse_int(argv[i + 1], &options->i2c_bus))
            {
                fprintf(stderr, "Invalid --i2c-bus value\n");
                return false;
            }
            if (options->i2c_bus < 0)
            {
                fprintf(stderr, "--i2c-bus must be zero or greater\n");
                return false;
            }
            i++;
        }
        else if (strcmp(argv[i], "--address") == 0)
        {
            if (i + 1 >= argc || !parse_int(argv[i + 1], &options->i2c_addr))
            {
                fprintf(stderr, "Invalid --address value\n");
                return false;
            }
            if (options->i2c_addr < 0 || options->i2c_addr > 0x7F)
            {
                fprintf(stderr, "--address must be a 7-bit I2C address, e.g. 0x40\n");
                return false;
            }
            i++;
        }
        else if (strcmp(argv[i], "--skip-camera") == 0)
        {
            options->skip_camera = true;
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            exit(0);
        }
        else
        {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            return false;
        }
    }

    return true;
}

static std::vector<std::string> list_i2c_devices()
{
    std::vector<std::string> devices;
    DIR *dir = opendir("/dev");
    if (dir == NULL)
    {
        return devices;
    }

    struct dirent *entry = NULL;
    while ((entry = readdir(dir)) != NULL)
    {
        if (strncmp(entry->d_name, "i2c-", 4) == 0)
        {
            std::stringstream path;
            path << "/dev/" << entry->d_name;
            devices.push_back(path.str());
        }
    }

    closedir(dir);
    std::sort(devices.begin(), devices.end());
    return devices;
}

static bool contains_i2c_bus(const std::vector<std::string> &devices, int bus)
{
    std::stringstream expected;
    expected << "/dev/i2c-" << bus;

    for (std::vector<std::string>::const_iterator it = devices.begin(); it != devices.end(); ++it)
    {
        if (*it == expected.str())
        {
            return true;
        }
    }

    return false;
}

static bool command_exists(const char *command)
{
    std::stringstream check;
    check << "command -v " << command << " >/dev/null 2>&1";
    return system(check.str().c_str()) == 0;
}

static void print_status(const char *name, bool ok)
{
    printf("[%s] %s\n", ok ? "OK" : "WARN", name);
}

static bool check_pigpiod_running()
{
    return system("pgrep -x pigpiod >/dev/null 2>&1") == 0;
}

static bool run_camera_check(bool skip_camera)
{
    printf("\n== Camera diagnostics ==\n");
    if (skip_camera)
    {
        printf("[SKIP] Camera check skipped by --skip-camera\n");
        return true;
    }

    if (command_exists("rpicam-hello"))
    {
        printf("Running: rpicam-hello --list-cameras\n");
        int result = system("rpicam-hello --list-cameras");
        print_status("rpicam-hello command completed", result == 0);
        return result == 0;
    }

    if (command_exists("libcamera-hello"))
    {
        printf("Running: libcamera-hello --list-cameras\n");
        int result = system("libcamera-hello --list-cameras");
        print_status("libcamera-hello command completed", result == 0);
        return result == 0;
    }

    printf("[WARN] No rpicam-hello or libcamera-hello command found\n");
    printf("       Install rpicam-apps/libcamera-apps and verify the camera stack manually.\n");
    return false;
}

static bool run_i2c_check(const Options &options)
{
    bool ok = true;

    printf("\n== I2C diagnostics ==\n");
    std::vector<std::string> devices = list_i2c_devices();
    if (devices.empty())
    {
        printf("[WARN] No /dev/i2c-* devices found\n");
        printf("       Enable I2C with: sudo raspi-config nonint do_i2c 0 && sudo reboot\n");
        ok = false;
    }
    else
    {
        printf("Detected I2C devices:\n");
        for (std::vector<std::string>::const_iterator it = devices.begin(); it != devices.end(); ++it)
        {
            printf("  %s\n", it->c_str());
        }
    }

    if (!contains_i2c_bus(devices, options.i2c_bus))
    {
        printf("[WARN] Expected /dev/i2c-%d is missing\n", options.i2c_bus);
        printf("       The current robot code uses I2C bus %d by default.\n", options.i2c_bus);
        printf("       On Raspberry Pi 4 header pins GPIO2/GPIO3 this is usually /dev/i2c-1.\n");
        ok = false;
    }
    else
    {
        printf("[OK] /dev/i2c-%d is present\n", options.i2c_bus);
    }

    return ok;
}

static bool run_pca9685_check(const Options &options)
{
    bool ok = true;

    printf("\n== PCA9685 diagnostics ==\n");
    printf("Opening bus %d address 0x%02X\n", options.i2c_bus, options.i2c_addr);

    int handle = i2cOpen(options.i2c_bus, options.i2c_addr, 0);
    if (handle < 0)
    {
        printf("[WARN] i2cOpen failed: %d\n", handle);
        printf("       Check that I2C is enabled, the PCA9685 is powered, SDA/SCL are wired,\n");
        printf("       common ground is connected, and the address is correct.\n");
        printf("       A quick check is: i2cdetect -y %d\n", options.i2c_bus);
        return false;
    }

    int mode1 = i2cReadByteData(handle, MODE1);
    int mode2 = i2cReadByteData(handle, MODE2);
    int prescale = i2cReadByteData(handle, PRESCALE);

    if (mode1 < 0 || mode2 < 0 || prescale < 0)
    {
        printf("[WARN] Failed to read one or more PCA9685 registers\n");
        ok = false;
    }
    else
    {
        printf("[OK] PCA9685 read-only register check succeeded\n");
        printf("     MODE1:    0x%02X\n", mode1);
        printf("     MODE2:    0x%02X\n", mode2);
        printf("     PRESCALE: 0x%02X\n", prescale);
    }

    i2cClose(handle);
    return ok;
}

static void print_servo_power_checklist()
{
    printf("\n== Servo power and wiring checklist ==\n");
    printf("- External servo power supply connected and rated for all 12 servos\n");
    printf("- Raspberry Pi, Robot HAT/PCA9685, and servo power supply share common ground\n");
    printf("- PCA9685 appears at the expected I2C address, usually 0x40\n");
    printf("- pigpiod daemon is stopped when using this direct pigpio C API code path\n");
    printf("- Robot is lifted or otherwise physically safe before any motion test\n");
}

int main(int argc, char **argv)
{
    Options options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    bool ok = true;

    printf("Adeept Darkpaw hardware diagnostics\n");
    printf("This tool performs read-only checks and does not command servos.\n");

    printf("\n== pigpio diagnostics ==\n");
    print_status("Running as root/sudo", geteuid() == 0);
    if (geteuid() != 0)
    {
        printf("       Direct pigpio hardware access commonly requires sudo.\n");
    }

    bool pigpiod_running = check_pigpiod_running();
    print_status("pigpiod daemon is not running", !pigpiod_running);
    if (pigpiod_running)
    {
        printf("       Stop it for direct API tests: sudo systemctl stop pigpiod\n");
    }

    int gpio_init = gpioInitialise();
    printf("gpioInitialise(): %d\n", gpio_init);
    if (gpio_init < 0)
    {
        printf("[WARN] pigpio initialization failed\n");
        printf("       Try running with sudo, and check that no other GPIO process owns the hardware.\n");
        ok = false;
    }
    else
    {
        printf("[OK] pigpio initialized\n");
    }

    ok = run_i2c_check(options) && ok;
    if (gpio_init >= 0)
    {
        ok = run_pca9685_check(options) && ok;
    }
    else
    {
        printf("\n== PCA9685 diagnostics ==\n");
        printf("[SKIP] Skipping PCA9685 check because pigpio did not initialize\n");
    }

    ok = run_camera_check(options.skip_camera) && ok;
    print_servo_power_checklist();

    if (gpio_init >= 0)
    {
        gpioTerminate();
    }

    printf("\nDiagnostics result: %s\n", ok ? "PASS" : "WARNINGS");
    return ok ? 0 : 1;
}
