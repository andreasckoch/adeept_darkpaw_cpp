#include "pigpio.h"

#include "servo.h"
#include "utils.h"

/* Registers */
#define PCA9685_ADDR        0x40
#define MODE1               0x00
#define MODE2               0x01
#define PRESCALE            0xFE

/* Bits */
#define RESTART             0x80
#define SLEEP               0x10
#define ALLCALL             0x01
#define OUTDRV              0x04

/* Constants */
#define OSC_CLK             25000000
#define RESOLUTION          4096
#define FREQ                50
#define MICROSEC_PER_SEC    1000000


int _prescale;
int _positions[12];

/* Ranges for the 12 servo motors (in microseconds) */
const int _limits[24] = {
    [0]  =  380, [1]  = 1700, [2]  = 380, [3]  = 1500, [4]  =  380, [5]  = 1500,
    [6]  =  380, [7]  = 1700, [8]  = 380, [9]  = 1400, [10] =  500, [11] = 1500,
    [12] =  380, [13] = 1400, [14] = 380, [15] = 1500, [16] =  550, [17] = 1650,
    [18] =  380, [19] = 1900, [20] = 650, [21] = 1500, [22] = 1000, [23] = 2100,
};

int _get_pulse_counter_unit(int pulse_microsec);
int _get_pulse_microsec(int pulse_counter_unit);
void _legs_transition(int servos, int steps, int positions[12], int goal_positions[12]);

/* alternative way of calculating pulse in counter unit */
// int prescale;
// pulse = 1000;           // pulse in microseconds
// pulselength = 1000000;  // 1,000,000 microseconds per second
// prescale = _prescale;
// prescale += 1;
// prescale *= pulselength;
// prescale /= _osc_clk;
// printf("pulse = %s\n", get_binary_byte_string(pulse).c_str());

/* Turn LEDS on */
// gpioSetMode(5, PI_OUTPUT);
// gpioSetMode(6, PI_OUTPUT);
// gpioSetMode(13, PI_OUTPUT);

// gpioWrite(5, 0);
// gpioWrite(6, 0);
// gpioWrite(13, 0);

int pca9685_init()
{
    int gpioInit = 0;
    int gpioCustomPort = gpioCfgSocketPort(8889);
    gpioInit = gpioInitialise();
    printf("gpio custom port: %d\n", gpioCustomPort);
    printf("gpioInit: %d\n", gpioInit);
    if (gpioInit >= 0)
    {

        int servos;
        servos = i2cOpen(1, PCA9685_ADDR, 0);
        printf("Servos: %d\n", servos);

        printf("Mode: %d\n", i2cReadByteData(servos, MODE1));

        /* Setting the PCA9685 frequency, must be 50Hz for the SG-90 servos */
        float prescale;
        int oldmode;
        int newmode;

        prescale = (OSC_CLK / (RESOLUTION * FREQ)) - 0.5; /* now 25*10^6 is 25Mhz of the internal clock, 4096 is 12 bit resolution and 50hz is the wanted frequency setup) */
        _prescale = (int)prescale;   // round = typecast and addition of 0.5 --> -0.5 instead of -1


        /* Setup of I2C interface for the device */
        i2cWriteByteData(servos, MODE2, OUTDRV);            /* set mode 2 OUTDRV=1, everything else to 0 */
        i2cWriteByteData(servos, MODE1, ALLCALL);           /* set mode 1 ALLCALL=1 */
        usleep(5000);                                       /* let oscillator catch up */
        int mode1 = i2cReadByteData(servos, MODE1);
        mode1 = mode1 &~ SLEEP;                             /* wake up */
        i2cWriteByteData(servos, MODE1, mode1);
        usleep(5000);

        /* now there is a whole sequence to set up the frequency (frequency can only be set in sleep mode) */
        oldmode = i2cReadByteData(servos, MODE1); /* getting current mode */
        newmode = (oldmode & 0x7F) | SLEEP;       /* sleep mode definition 0x10 = 0000 1000 (with first bit 0 -> & 0x7F) --> set bit 7 to 0 (restart), bit 4 to 1 (sleep) and leave the rest */
        i2cWriteByteData(servos, MODE1, newmode); /* going to sleep now */
        i2cWriteByteData(servos, PRESCALE, _prescale);    /* setting up the frequency now 0xFE = 1111 1110 (prescaler register) */
        i2cWriteByteData(servos, MODE1, oldmode); /* coming back to the old mode */
        usleep(5000);
        i2cWriteByteData(servos, MODE1, oldmode | RESTART); /* final step on frequency set up 0x80 = 1000 0000 */

        return servos;
    }
    else
    {
        printf("Error: GPIO not initialized");
        return gpioInit;
    }
}

void pca9685_move_legs_synchronized(int servos)
{
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
    int positions_legs_low[12] = {
        [0] =  _limits[1], [1]   =  _limits[2], [2]   =  _limits[4],
        [3] =  _limits[7], [4]   =  _limits[9], [5]   = _limits[11],  
        [6] = _limits[12], [7]   = _limits[13], [8]   = _limits[17],
        [9] = _limits[18], [10]  = _limits[20], [11]  = _limits[22],
    };

    int positions_legs_high[12] = {
        [0] =  _limits[1], [1]   =  _limits[3], [2]   =  _limits[5],
        [3] =  _limits[7], [4]   =  _limits[8], [5]   = _limits[10],
        [6] = _limits[12], [7]   = _limits[14], [8]   = _limits[16],
        [9] = _limits[18], [10]  = _limits[21], [11]  = _limits[23],
    };
    int positions_legs_close[12] = {
        [0] =  _limits[1], [1]   =  _limits[2], [2]   =  _limits[5],
        [3] =  _limits[7], [4]   =  _limits[9], [5]   = _limits[10],  
        [6] = _limits[12], [7]   = _limits[13], [8]   = _limits[16],
        [9] = _limits[18], [10]  = _limits[20], [11]  = _limits[23],
    };

    int positions_legs_far[12] = {
        [0] =  _limits[1], [1]   =  _limits[3], [2]   =  _limits[4],
        [3] =  _limits[7], [4]   =  _limits[8], [5]   = _limits[11],
        [6] = _limits[12], [7]   = _limits[14], [8]   = _limits[17],
        [9] = _limits[18], [10]  = _limits[21], [11]  = _limits[22],
    };
    // int positions_1[12] = {
    //     [0] =  _limits[1], [1]   =  _limits[3], [2]   =  _limits[5],
    //     [3] =  _limits[7], [4]   =  _limits[8], [5]   = _limits[10],  
    //     [6] = _limits[12], [7]   = _limits[14], [8]   = _limits[16],
    //     [9] = _limits[18], [10]  = _limits[20], [11]  = _limits[23],
    // };

    // int positions_2[12] = {
    //     [0] =  _limits[1], [1]   =  _limits[3], [2]   =  _limits[5],
    //     [3] =  _limits[7], [4]   =  _limits[8], [5]   = _limits[10],
    //     [6] = _limits[12], [7]   = _limits[14], [8]   = _limits[16],
    //     [9] = _limits[18], [10]  = _limits[21], [11]  = _limits[23],
    // };

    _legs_transition(servos, 20, positions_legs_high, positions_legs_low);
    usleep(5000000);
    _legs_transition(servos, 20, positions_legs_low, positions_legs_far);
    usleep(5000000);
    _legs_transition(servos, 20, positions_legs_far, positions_legs_close);
    usleep(5000000);
    _legs_transition(servos, 20, positions_legs_close, positions_legs_high);

}

int _get_pulse_counter_unit(int pulse_microsec)
{
    return (int)((pulse_microsec * RESOLUTION * FREQ) / (MICROSEC_PER_SEC));    /* pulse in counter unit */
}

int _get_pulse_microsec(int pulse_counter_unit)
{
    return (int)((pulse_counter_unit * MICROSEC_PER_SEC) / (RESOLUTION * FREQ));    /* pulse in counter unit */
}

void _legs_transition(int servos, int steps, int positions[12], int goal_positions[12])
{
    for (int step = 1; step <= steps; step++)
    {
        for (int i = 0; i < 12; i++)
        {
            if (positions[i] == goal_positions[i]) {continue;}
            int pulse_microsec = positions[i] + step * (int)((goal_positions[i] - positions[i]) / steps);
            int pulse_counter_unit = _get_pulse_counter_unit(pulse_microsec);
            printf("Step [%d] - Servo [%d] - pulse_microsec [%d]\n", step, i, pulse_microsec);
            i2cWriteByteData(servos, (4 * i) + 8, pulse_counter_unit & 0xFF);
            i2cWriteByteData(servos, (4 * i) + 9, pulse_counter_unit >> 8);
            usleep(2000);
        }
    }
}
